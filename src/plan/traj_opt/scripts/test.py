#! /home/dwl/miniconda3/envs/torch_env/bin/python
import rospy
import numpy as np
from traj_opt.srv import forward_srv

def test_forward_client():
    # 等待服务可用
    rospy.wait_for_service('/forward_service')
    
    try:
        # 创建服务客户端
        forward_client = rospy.ServiceProxy('/forward_service', forward_srv)
        
        # 生成随机输入数据 (生成一个3维向量作为测试)
        random_input = np.random.rand(3).tolist()
        
        # 调用服务
        need_grads = True  # 设置是否需要梯度
        response = forward_client(random_input, need_grads)
        
        # 打印结果
        print("Input data:", random_input)
        print("Prediction:", response.prediction)
        if need_grads:
            print("Gradients:", response.grad)
            
    except rospy.ServiceException as e:
        print("Service call failed:", e)

if __name__ == "__main__":
    # 初始化ROS节点
    rospy.init_node('forward_client_test', anonymous=True)
    test_forward_client()