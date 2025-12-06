#!/usr/bin/env python3
import subprocess
import time
import random
import signal
import sys
import math
import rospy
import numpy as np
import logging
import argparse
from std_msgs.msg import Float64
from dit.Polynomial_DiT.trajectory_predictor import TrajectoryPredictor
import torch
import os

logger = logging.getLogger("ILGuidedRunner")
logger.setLevel(logging.INFO)
handler = logging.StreamHandler(sys.stdout)
formatter = logging.Formatter('[%(asctime)s][%(levelname)s] %(message)s')
handler.setFormatter(formatter)
if not logger.hasHandlers():
    logger.addHandler(handler)
random.seed(42)
np.random.seed(42)
torch.manual_seed(42)
    

class ILGuidedRunner:
    def __init__(self, mode=1, roll=40):
        self.roscore_process = None
        self.sim_process = None
        self.running = True
        self.simulation_finished = False
        self.finish_sim_value = None
        if mode == 1:
            print("Loading guided points predictor...")
            device = 'cuda' if torch.cuda.is_available() else 'cpu'
            self.trajectory_predictor = TrajectoryPredictor(
                model_path="checkpoints/Polynomial-DiT-local/strike_checkpoint_latest.pth", 
                data_path="datasets/strike_processed.txt",
                mode="polynomial",
                device=device)
        else:
            print("Do not use guided points predictor")
            self.trajectory_predictor = None
        self.mode = mode
        self.fixed_roll = roll
        
    def signal_handler(self, signum, frame):
        logger.info("Received interrupt signal, closing all processes...")
        self.running = False
        self.cleanup_all()
        sys.exit(0)
        
    def cleanup_all(self):
        logger.info("Cleaning up processes...")
        if self.sim_process and self.sim_process.poll() is None:
            self.sim_process.terminate()
            try:
                self.sim_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.sim_process.kill()

        if self.roscore_process and self.roscore_process.poll() is None:
            self.roscore_process.terminate()
            try:
                self.roscore_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.roscore_process.kill()
        
        # Clean up residual processes
        subprocess.run(['pkill', '-f', 'se3_node'], check=False)
    
    def start_roscore(self):
        logger.info("Starting roscore...")
        self.roscore_process = subprocess.Popen(['roscore'], 
                                               stdout=subprocess.DEVNULL, 
                                               stderr=subprocess.DEVNULL)
        time.sleep(2)
        logger.info("roscore started")


    def generate_random_quaternion_z_up(self):
        yaw = 0.0
        if self.fixed_roll is not None:
            roll = self.fixed_roll
        else:
            roll = random.uniform(20, 60)
        roll = np.deg2rad(roll)
        pitch = 0.0  
        
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        
        return w, x, y, z, roll

    def set_random_object_params(self):
        while True:
            r = math.sqrt(random.uniform(1.5**2, 3.0**2))
            theta = random.uniform(math.pi/2, 3*math.pi/2)
            start_x = r * math.cos(theta)
            start_y = r * math.sin(theta)
            if start_x < 0:
                break
        
        start_z = 1.2
        logger.info(f"Start point: ({start_x:.3f}, {start_y:.3f}, {start_z:.3f})")
        
        object_px = 0.0
        object_py = 0.0
        object_pz = 1.2
        object_qw, object_qx, object_qy, object_qz, roll = self.generate_random_quaternion_z_up()
        logger.info(f"Object roll: {np.rad2deg(roll):.1f}°")

        end_x = 3.0
        end_y = 0.0
        end_z = 1.2

        # Set waypoints based on planning mode
        if roll >= np.deg2rad(20):
            if self.mode == 0:
                guide_pt_list = []
            elif self.mode == 1:
                conditions = np.array([[start_x, start_y, roll]], dtype=np.float32)
                t_dense = np.linspace(0, 1, 8)
                trajectories = self.trajectory_predictor.predict_trajectory(conditions, t_dense)
                guide_pt_list = [trajectories[0, :], trajectories[-1, :]]
        else:
            guide_pt_list = []
        
        try:
            subprocess.run(['rosparam', 'set', '/object_px', str(object_px)], check=True)
            subprocess.run(['rosparam', 'set', '/object_py', str(object_py)], check=True)
            subprocess.run(['rosparam', 'set', '/object_pz', str(object_pz)], check=True)
            subprocess.run(['rosparam', 'set', '/object_qw', str(object_qw)], check=True)
            subprocess.run(['rosparam', 'set', '/object_qx', str(object_qx)], check=True)
            subprocess.run(['rosparam', 'set', '/object_qy', str(object_qy)], check=True)
            subprocess.run(['rosparam', 'set', '/object_qz', str(object_qz)], check=True)
            subprocess.run(['rosparam', 'set', '/end_x', str(end_x)], check=True)
            subprocess.run(['rosparam', 'set', '/end_y', str(end_y)], check=True)
            subprocess.run(['rosparam', 'set', '/end_z', str(end_z)], check=True)
            subprocess.run(['rosparam', 'set', '/start_x', str(start_x)], check=True)
            subprocess.run(['rosparam', 'set', '/start_y', str(start_y)], check=True) 
            subprocess.run(['rosparam', 'set', '/start_z', str(start_z)], check=True)
            
            if guide_pt_list:
                num_pre_point = int(len(guide_pt_list)/2)
                num_post_point = int(len(guide_pt_list)/2)
                subprocess.run(['rosparam', 'set', '/num_pre_point', str(num_pre_point)], check=True)
                subprocess.run(['rosparam', 'set', '/num_post_point', str(num_post_point)], check=True)
                for i in range(num_pre_point):
                    subprocess.run(['rosparam', 'set', f'/pre_x_{i}', str(guide_pt_list[i][0])], check=True)
                    subprocess.run(['rosparam', 'set', f'/pre_y_{i}', str(guide_pt_list[i][1])], check=True)
                    subprocess.run(['rosparam', 'set', f'/pre_z_{i}', str(guide_pt_list[i][2])], check=True)
                    subprocess.run(['rosparam', 'set', f'/post_x_{i}', str(guide_pt_list[i+num_post_point][0])], check=True)
                    subprocess.run(['rosparam', 'set', f'/post_y_{i}', str(guide_pt_list[i+num_post_point][1])], check=True)
                    subprocess.run(['rosparam', 'set', f'/post_z_{i}', str(guide_pt_list[i+num_post_point][2])], check=True)
            else:
                subprocess.run(['rosparam', 'set', '/num_pre_point', '0'], check=True)
                subprocess.run(['rosparam', 'set', '/num_post_point', '0'], check=True)
                
            logger.info("Parameters set successfully")
        except subprocess.CalledProcessError as e:
            logger.error(f"Error setting parameters: {e}")
        
    def simulation_finished_callback(self, msg):
        logger.info("Simulation finished signal received")
        self.simulation_finished = True
        self.finish_sim_value = msg.data
        

    def run_simulation(self):
        logger.info("Starting simulation...")
        self.simulation_finished = False
        self.finish_sim_value = None
        
        self.sim_process = subprocess.Popen(
            ['roslaunch', 'plan_manage', f'IL_guided_{self.mode}.launch']
        )

        while self.running and not self.simulation_finished:
            if self.sim_process.poll() is not None:
                break
            try:
                rospy.sleep(0.1)
            except rospy.ROSInterruptException:
                break

        return self.simulation_finished

    def run(self):
        signal.signal(signal.SIGINT, self.signal_handler)
        
        try:
            self.start_roscore()
            rospy.init_node('demo_runner', anonymous=True)
            rospy.Subscriber('/finish_simulation', Float64, self.simulation_finished_callback)
            
            self.set_random_object_params()
            success = self.run_simulation()
            
            # Output results
            if success and self.finish_sim_value is not None:
                logger.info("Finish IL-guided Optimization")
            else:
                logger.info("Planning Failed")
                
        except KeyboardInterrupt:
            logger.info("User interrupted execution")
        except Exception as e:
            logger.error(f"Execution error: {e}")
        finally:
            self.cleanup_all()
            logger.info("Processes cleaned up")

def main():
    parser = argparse.ArgumentParser(
        description='IL-guided Optimization'
    )
    parser.add_argument('--mode', '-m', type=int, default=1, choices=[0, 1],
                       help='Planning mode: 0: baseline, 1: IL-guided (default: 1)')
    parser.add_argument('--roll', '-r', type=float, default=40,
                       help='Object roll angle in degrees (default: 40)')
    args = parser.parse_args()
    
    runner = ILGuidedRunner(mode=args.mode, roll=args.roll)
    runner.run()

if __name__ == "__main__":
    main()