#include <pybind11/eigen.h>
#include <pybind11/embed.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <ros/package.h>

#include <Eigen/Dense>
#include <iostream>
#include <vector>

#include "neural_network/python_interpreter.h"

/*
  This head file is used to integrate forward python model into cpp
*/

namespace py = pybind11;

class PyTorchForwardModel
{
 private:
  py::object model;
  py::object torch;
  py::object np;
  std::string device;
  bool initialized;

 public:
  PyTorchForwardModel() : initialized(false) {}

  bool initialize()
  {
    try
    {
      /* initialize the python interpreter */
      if (!neural_network::PythonInterpreter::initialize())
      {
        return false;
      }

      /* import the necessary modules */
      torch = py::module::import("torch");
      np = py::module::import("numpy");
      py::module sys = py::module::import("sys");
      // sys.attr("path").attr("append")("/home/dwl/real-world-flight");
      /* cpu or cuda */
      py::object cuda_available = torch.attr("cuda").attr("is_available")();
      device = cuda_available.cast<bool>() ? "cuda" : "cpu";

      /* import the class */
      auto delta_rev_net_code = R"(
import torch
import torch.nn as nn

class RevBlock(nn.Module):
    def __init__(self, channels):
        super(RevBlock, self).__init__()
        self.channels = channels
        half_channels = channels // 2
        
        self.F = nn.Sequential(
            nn.Linear(half_channels, half_channels),
            nn.BatchNorm1d(half_channels),
            nn.ReLU(),
            nn.Linear(half_channels, half_channels)
        )
        
        self.G = nn.Sequential(
            nn.Linear(half_channels, half_channels),
            nn.BatchNorm1d(half_channels),
            nn.ReLU(),
            nn.Linear(half_channels, half_channels)
        )
    
    def forward(self, x, reverse=False):
        x1, x2 = torch.chunk(x, 2, dim=1)
        
        if not reverse:
            y1 = x1 + self.F(x2)
            y2 = x2 + self.G(y1)
        else:
            y2 = x2 - self.G(x1)
            y1 = x1 - self.F(y2)
            
        return torch.cat([y1, y2], dim=1)

class DeltaRevNet(nn.Module):
    def __init__(self, input_dim=3, hidden_dim=32):
        super(DeltaRevNet, self).__init__()
        
        # Ensure hidden_dim is even
        if hidden_dim % 2 != 0:
            hidden_dim += 1
            
        # Input projection layer
        self.input_proj = nn.Linear(input_dim, hidden_dim)
        
        # RevNet blocks
        self.rev_blocks = nn.ModuleList([
            RevBlock(hidden_dim) for _ in range(4)
        ])
        
        # Output projection layer
        self.output_proj = nn.Linear(hidden_dim, input_dim)
        
    def forward(self, x, reverse=False):
        # Input projection
        x = self.input_proj(x)
        
        # Forward or reverse pass through RevNet blocks
        if not reverse:
            for block in self.rev_blocks:
                x = block(x, reverse=False)
        else:
            for block in reversed(self.rev_blocks):
                x = block(x, reverse=True)
        
        # Output projection
        x = self.output_proj(x)
        return x
            )";

      /* execute the model definition code */
      py::exec(delta_rev_net_code);

      py::object DeltaRevNet = py::eval("DeltaRevNet");
      model = DeltaRevNet();
      model = model.attr("to")(device);

      /* load the pre-trained weights */
      py::object checkpoint = torch.attr("load")(
          ros::package::getPath("traj_opt") + "/weights/forward_weight.pth",
          py::dict(py::arg("map_location") = device));
      model.attr("load_state_dict")(checkpoint["model_state_dict"]);
      model.attr("eval")();
      initialized = true;
      std::cout << "[Forward_H]: PyTorch model successfully initialized on " << device
                << std::endl;
      return true;
    }
    catch (const std::exception& e)
    {
      std::cerr << "[Forward_H]: Error initializing PyTorch model: " << e.what()
                << std::endl;
      return false;
    }
  }

  /*
    main interface:
    INPUT:
      normalized angles(vectorXd): a1_1, a2_1, a3_1, ... a1_n, a2_n, a3_n
      need_grads(bool): whether to calculate the gradients
    OUTPUT:
      position(vectorXd): x1, y1, z1, x2, y2, z2, ...
      gradients(vectorXd): dx1/da1_1, dy1/da1_1, dz1/da1_1, dx1/da1_2, dy1/da1_2,
                           dz1/da1_2, ...
  */
  std::pair<Eigen::MatrixXd, Eigen::VectorXd> forward(const Eigen::VectorXd& input,
                                                      bool need_grads = false)
  {
    if (!initialized && !initialize())
    {
      throw std::runtime_error("[Forward_H]: Failed to initialize model!");
    }

    try
    {
      py::array_t<double> input_array = py::cast(input); /* transform to py::array */
      /* The model input is Nx3 */
      input_array = input_array.attr("reshape")(-1, 3);
      py::object input_tensor =
          torch.attr("from_numpy")(input_array).attr("float")().attr("to")(device);

      if (need_grads)
      {
        input_tensor.attr("requires_grad_")(true);
      }

      /* forward */
      py::object output_tensor = model(input_tensor);
      /* transform to cpu and numpy */
      py::array_t<double> output_array =
          output_tensor.attr("detach")().attr("cpu")().attr("numpy")().attr("flatten")();

      /* transform to cpp data structure  */
      Eigen::Map<Eigen::VectorXd> output(static_cast<double*>(output_array.request().ptr),
                                         output_array.size());

      /* calculate the gradients */
      Eigen::VectorXd gradients;
      if (need_grads)
      {
        /* input = N*3 */
        int input_rows = input.rows();  // N
        int input_cols = input.cols();  // 3
        int intput_tensor_size = input_rows * input_cols;

        std::vector<double> all_grads, all_grads_reshaped;
        for (int i = 0; i < 3; ++i)
        {
          /* select the i-th column of output */
          /* select(1, i) = the i-th col, 1 means the col */
          py::object grad_i =
              torch.attr("autograd")
                  .attr("grad")(output_tensor.attr("select")(1, i).attr("sum")(),
                                input_tensor, py::arg("retain_graph") = true);

          /* transform to list, because the saw grad_i is a tuple, but have some bugs in
           * dealing with tuple.*/
          py::list grad_list = py::cast<py::list>(grad_i);
          grad_i = grad_list[0]; /* get the first element, which is a tensor */
          py::array_t<double> grad_array = grad_i.attr("cpu")().attr("numpy")();

          /* transform to eigen matrix and then push into vector */
          auto grad_matrix = py::cast<Eigen::MatrixXd>(grad_array);
          for (int r = 0; r < grad_matrix.rows(); ++r)
          {
            for (int c = 0; c < grad_matrix.cols(); ++c)
            {
              all_grads.push_back(grad_matrix(r, c));
            }
          }
        }

        for (int i = 0; i < intput_tensor_size; ++i)
        {
          /* the grad of three output to the i input */
          all_grads_reshaped.push_back(all_grads[i]);
          all_grads_reshaped.push_back(all_grads[i + intput_tensor_size]);
          all_grads_reshaped.push_back(all_grads[i + 2 * intput_tensor_size]);
        }
        /* transform to eigen vector */
        gradients = Eigen::Map<Eigen::VectorXd>(all_grads_reshaped.data(),
                                                all_grads_reshaped.size());
      }
      return {output, gradients};
    }
    catch (const std::exception& e)
    {
      std::cerr << "Error during forward pass: " << e.what() << std::endl;
      throw;
    }
  }
};