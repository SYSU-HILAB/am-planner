#! /home/dwl/miniconda3/envs/torch_env/bin/python

import rospy
from traj_opt.srv import forward_srv, forward_srvResponse
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim


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

# Global model variable
model = None
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

def handle_network(req):
    # ROS_INFO("calling")
    global model
    
    # Load model if not loaded
    if model is None:
        model = DeltaRevNet().to(device)
        checkpoint = torch.load('/home/dwl/real-world-flight/forward_weight.pth', map_location=device)
        model.load_state_dict(checkpoint['model_state_dict'])
        model.eval()
    
    # Process input data
    input_data = np.array(req.input).reshape(-1, 3)
    print(f"[Forward Server]: input_data: {input_data}")
    input_tensor = torch.from_numpy(input_data).float().to(device)
    # for i in range(input_data.shape[1]):
    #     max_pos = np.max(input_data[:, i])
    #     print(f"max_input {i} : {max_pos}")
    
    # Set requires_grad if needed
    if req.need_grads:
        input_tensor.requires_grad_(True)
    
    # Forward pass
    output_tensor = model(input_tensor)
    
    # Prepare response
    response = forward_srvResponse()
    response.prediction = output_tensor.detach().cpu().numpy().flatten().tolist()

    # Calculate gradients if needed
    if req.need_grads:
        grads = []
        for i in range(output_tensor.shape[1]):  # For each output component
            grad = torch.autograd.grad(output_tensor[:, i].sum(),
                                    input_tensor,
                                    retain_graph=True)[0]  # (N, 3)
            grads.append(grad.cpu().numpy())
        
        # Transpose to reorder dimensions
        grads = np.array(grads)  # shape: (3, N, 3)
        grads = np.transpose(grads, (1, 2, 0))  # shape: (N, 3, 3)
        response.grad = grads.flatten().tolist()
    return response

def server():
    rospy.init_node("forward_server")
    s = rospy.Service("/forward_service", forward_srv, handle_network)
    print("Forward service ready.")
    rospy.spin()

if __name__ == "__main__":
    server()