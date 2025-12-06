#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header

def generate_wall_points(height, thickness, width, gap_height, gap_width, resolution=0.01):
    points = []
    
    for y in np.arange(-width/2, width/2, resolution):
        for x in np.arange(-thickness/2, thickness/2, resolution):
            # Calculate the center of the gap
            gap_center = height / 2
            
            # Calculate the bottom and top of the gap
            gap_bottom = gap_center - gap_height / 2
            gap_top = gap_center + gap_height / 2
            
            # Generate points for the bottom part of the wall
            for z in np.arange(0, gap_bottom, resolution):
                points.append([x, y, z])
            
            # Generate points for the top part of the wall
            for z in np.arange(gap_top, 2, resolution):
                points.append([x, y, z])

    return points

def publish_point_cloud():
    rospy.init_node('wall_point_cloud_publisher', anonymous=True)
    pub = rospy.Publisher('/global_map', PointCloud2, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    # Wall parameters
    wall_height = 2.0  # 4 meters high
    wall_thickness = 0.5  # 0.2 meters thick
    wall_width = 2.0  # 8 meters wide
    gap_height = 0.30  # Height of the gap in meters (50 cm)
    gap_width = wall_width  # Width of the gap (full width of the wall)

    while not rospy.is_shutdown():
        # Generate point cloud data
        points = generate_wall_points(wall_height, wall_thickness, wall_width, gap_height, gap_width)

        # Create header
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "world"

        # Create PointCloud2 message
        pc2_msg = pc2.create_cloud_xyz32(header, points)

        # Publish the message
        pub.publish(pc2_msg)
        rospy.loginfo("Published point cloud with %d points", len(points))
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_point_cloud()
    except rospy.ROSInterruptException:
        pass