#include <pybind11/eigen.h>
#include <pybind11/embed.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <ros/package.h>

#include <Eigen/Dense>
#include <iostream>
#include <vector>

#include "neural_network/python_interpreter.h"

namespace py = pybind11;

/*
  This head file is used to integrate workspace probability model into cpp
*/

class WorkspaceProbabilityModel
{
 private:
  py::object model;
  py::object torch;
  py::object np;
  std::string device;
  bool initialized;

 public:
  WorkspaceProbabilityModel() : initialized(false) {}

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

      /* add the model path to the python system path */
      // sys.attr("path").attr("append")("/home/dwl/real-world-flight");  //
      // 根据实际路径调整

      /* determine the device */
      py::object cuda_available = torch.attr("cuda").attr("is_available")();
      device = cuda_available.cast<bool>() ? "cuda" : "cpu";
      std::cout << "[Workspace_Model_H]: Using device: " << device << std::endl;

      /* import the model definition */
      auto workspace_mlp_code = R"(
import torch
import torch.nn as nn

class WorkspaceMLP(nn.Module):
    def __init__(self):
        super(WorkspaceMLP, self).__init__()
        self.network = nn.Sequential(
            nn.Linear(3, 64),
            nn.LeakyReLU(0.01),
            nn.Linear(64, 256),
            nn.LeakyReLU(0.01),
            nn.Linear(256, 128),
            nn.LeakyReLU(0.01),
            nn.Linear(128, 64),
            nn.LeakyReLU(0.01),
            nn.Linear(64, 1),
            nn.Sigmoid()
        )
    
    def forward(self, x):
        return self.network(x)
      )";

      py::exec(workspace_mlp_code);
      py::object WorkspaceMLP = py::eval("WorkspaceMLP");
      model = WorkspaceMLP();
      model = model.attr("to")(device);
      py::object checkpoint = torch.attr("load")(
          ros::package::getPath("traj_opt") + "/weights/workspace_probability_weight.pth",
          py::dict(py::arg("map_location") = device));
      model.attr("load_state_dict")(checkpoint);
      model.attr("eval")();

      initialized = true;
      std::cout << "[Workspace_Model_H]: Workspace probability model successfully "
                   "initialized on "
                << device << std::endl;
      return true;
    }
    catch (const std::exception& e)
    {
      std::cerr << "[Workspace_Model_H]: Error initializing workspace probability "
                   "model: "
                << e.what() << std::endl;
      return false;
    }
  }

  std::pair<Eigen::VectorXd, Eigen::VectorXd> forward(const Eigen::VectorXd& input,
                                                      bool need_grads = false)
  {
    if (!initialized && !initialize())
    {
      throw std::runtime_error(
          "[Workspace_Model_H]: Failed to initialize workspace "
          "probability model");
    }

    try
    {
      py::array_t<double> input_array = py::cast(input);
      py::object input_tensor =
          torch.attr("from_numpy")(input_array).attr("float")().attr("to")(device);
      input_tensor = input_tensor.attr("reshape")(-1, 3);

      /* set whether to need gradients */
      if (need_grads)
      {
        input_tensor.attr("requires_grad_")(true);
      }

      py::object output_tensor = model(input_tensor);

      /* transform the output tensor to eigen vector */
      py::array_t<double> output_array =
          output_tensor.attr("detach")().attr("cpu")().attr("numpy")().attr("flatten")();
      Eigen::Map<Eigen::VectorXd> output(static_cast<double*>(output_array.request().ptr),
                                         output_array.size());

      Eigen::VectorXd gradients;
      /* if need gradients, calculate the gradients */
      // std::cout << "Selecting output tensor dimension 1, index 0" << std::endl;
      auto selected = output_tensor.attr("select")(1, 0);
      // std::cout << "Selected tensor: " << selected.str().cast<std::string>() <<
      // std::endl;
      // std::cout << "Selected tensor shape: ";
      // py::tuple shape = selected.attr("shape");
      // for (int i = 0; i < py::len(shape); i++)
      // {
      //   std::cout << shape[i].cast<int>() << " ";
      // }
      // std::cout << std::endl;

      if (need_grads)
      {
        py::object grad =
            torch.attr("autograd")
                .attr("grad")(output_tensor.attr("select")(1, 0), input_tensor,
                              py::arg("grad_outputs") =
                                  torch.attr("ones")(output_tensor.attr("size")(0))
                                      .attr("to")(device),
                              py::arg("retain_graph") = true);

        /* transform to list, because the saw grad_i is a tuple, but have some bugs in
         * dealing with tuple.*/
        py::list grad_list = py::cast<py::list>(grad);
        py::object grad_tensor = grad_list[0];
        // std::cout << "Grad tensor size: ";
        // py::tuple shape = grad_tensor.attr("shape");
        // for (int i = 0; i < py::len(shape); i++)
        // {
        //   std::cout << shape[i].cast<int>() << " ";
        // }
        // std::cout << std::endl;
        // std::cout << "Grad tensor: " << grad_tensor.str().cast<std::string>()
        //           << std::endl;
        /* grad_list[0] is a tensor, so we need to transform it to eigen vector */
        py::array_t<double> grad_array =
            grad_tensor.attr("detach")().attr("cpu")().attr("numpy")().attr("flatten")();
        gradients = py::cast<Eigen::VectorXd>(grad_array);
      }

      return {output, gradients};
    }
    catch (const std::exception& e)
    {
      std::cerr << "Error during workspace probability forward pass: " << e.what()
                << std::endl;
      throw;
    }
  }
};