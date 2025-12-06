#! /home/dwl/miniconda3/envs/torch_env/bin/python

import rospy
from traj_opt.srv import workspace_probability_srv, workspace_probability_srvResponse
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim


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

# Global model variable
model = None
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

def handle_network(req):
    # print("calling")
    global model
    
    # Load model if not loaded
    if model is None:
        model = WorkspaceMLP().to(device)
        checkpoint = torch.load('/home/dwl/real-world-flight/workspace_probability_weight.pth', 
                              map_location=device,
                              weights_only=True)  # Add weights_only=True
        model.load_state_dict(checkpoint)  # Load directly since weights_only=True
        model.eval()
    
    # Process input data
    input_data = np.array(req.input).reshape(-1, 3)
    input_tensor = torch.from_numpy(input_data).float().to(device)
    
    # Set requires_grad if needed
    if req.need_grads:
        input_tensor.requires_grad_(True)
    
    # Forward pass
    output_tensor = model(input_tensor)
    
    # Prepare response
    response = workspace_probability_srvResponse()
    response.prediction = output_tensor.detach().cpu().numpy().flatten().tolist()

    if req.need_grads:
        # Calculate gradients for all outputs
        grad = torch.autograd.grad(output_tensor[:,0],
                                input_tensor,
                                grad_outputs=torch.ones(len(output_tensor), device=device, dtype=torch.float32),
                                retain_graph=True)[0]
        response.grad = [float(x) for x in grad.detach().cpu().numpy().flatten().tolist()]
    return response

def server():
    rospy.init_node("workspace_probability_server")
    s = rospy.Service("/workspace_probability_service", workspace_probability_srv, handle_network)
    print("Workspace_probability ready.")
    rospy.spin()

if __name__ == "__main__":
    server()