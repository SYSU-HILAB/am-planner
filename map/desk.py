#!/usr/bin/env python
import rospy
import numpy as np
import tf
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
import transforms3d as t3d


class PlatformPublisher:
    """Publishes platform point cloud data based on object parameters."""

    def __init__(self):
        """Initialize the platform publisher."""
        self.pub = rospy.Publisher('/global_map', PointCloud2, queue_size=1)
        
        # Flag to determine if reading from topic or parameters
        self.use_topic = rospy.get_param('/se3_node/use_object_odom', False)
        self.odom_data = {'px': 0.0, 'py': 0.0, 'pz': 0.0, 'qw': 1.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0}
        
        # Subscribe to odometry topic if flag is true
        self.odom_sub = rospy.Subscriber('/odom_converter/converted_odom1', Odometry, self.odom_callback)
        
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def odom_callback(self, msg):
        """Callback for odometry topic."""
        self.odom_data = {
            'px': msg.pose.pose.position.x,
            'py': msg.pose.pose.position.y,
            'pz': msg.pose.pose.position.z,
            'qw': msg.pose.pose.orientation.w,
            'qx': msg.pose.pose.orientation.x,
            'qy': msg.pose.pose.orientation.y,
            'qz': msg.pose.pose.orientation.z
        }
    
    def timer_callback(self, event):
        """Timer callback to publish platform data."""
        self.use_topic = rospy.get_param('/se3_node/use_object_odom', False)
        self.publish_platform()

    def generate_platform_points(self, resolution=0.01, threshold_distance=0.03):
        """Generate platform point cloud data.

        Args:
            resolution: Resolution of the point cloud in meters.
            threshold_distance: Threshold distance to avoid object in point cloud.

        Returns:
            List of 3D points representing the platform.
        """
        object_height = rospy.get_param('/object_height', 0.1)
        obj_to_desk_distance = object_height / 2.0

        points = []
        size = 0.6  # 0.6m x 0.6m platform
        mode = rospy.get_param('/mode', -1)
        if mode == 0 or mode == 1:
            # Check if using topic data
            if self.use_topic:
                object_px = self.odom_data['px']
                object_py = self.odom_data['py']
                object_pz = self.odom_data['pz']
                object_qw = self.odom_data['qw']
                object_qx = self.odom_data['qx']
                object_qy = self.odom_data['qy']
                object_qz = self.odom_data['qz']
            else:
                # Use parameters
                object_px = rospy.get_param('/object_px', 0.0)
                object_py = rospy.get_param('/object_py', 0.0)
                object_pz = rospy.get_param('/object_pz', 0.0)
                object_qw = rospy.get_param('/object_qw', 1.0)
                object_qx = rospy.get_param('/object_qx', 0.0)
                object_qy = rospy.get_param('/object_qy', 0.0)
                object_qz = rospy.get_param('/object_qz', 0.0)
        elif mode == 2:
            task = rospy.get_param('/task', '')
            position = rospy.get_param(f'/{task}/object/position', [0.0, 0.0, 0.0])
            quaternion = rospy.get_param(f'/{task}/object/quaternion', [1.0, 0.0, 0.0, 0.0])
            object_px, object_py, object_pz = position
            object_qw, object_qx, object_qy, object_qz = quaternion
        else:
            # print("Has no object parameters!")
            return None

        quaternion = [object_qw,object_qx, object_qy, object_qz]
        rotation_matrix = np.array(t3d.quaternions.quat2mat(quaternion)[:3, :3])

        z_axis = rotation_matrix[:, 2]
        x_axis = rotation_matrix[:, 0]
        y_axis = rotation_matrix[:, 1]

        object_pos = np.array([object_px, object_py, object_pz])
        desk_center = object_pos - obj_to_desk_distance * z_axis

        # Generate the point cloud of a desk
        for u in np.arange(-size/2, size/2, resolution):
            for v in np.arange(-size/2, size/2, resolution):
                point = desk_center + u * x_axis + v * y_axis
                if np.abs(point - desk_center).max() > threshold_distance:
                    points.append([point[0], point[1], point[2]])
                for h in np.arange(0.0, point[2], resolution):
                    vertical_point = np.array([point[0], point[1], h])
                    if np.abs(vertical_point - desk_center).max() > threshold_distance:
                        points.append(vertical_point.tolist())

        return points

    def publish_platform(self):
        """Publish platform point cloud data."""
        points = self.generate_platform_points()
        if not points:
            return
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "world"
        pc2_msg = pc2.create_cloud_xyz32(header, points)
        self.pub.publish(pc2_msg)


def main():
    """Main function to initialize and run the platform publisher."""
    rospy.init_node('platform_publisher', anonymous=True)
    platform_pub = PlatformPublisher()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass