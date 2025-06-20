from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([


        Node(
            package='py_pkg',
            executable='pub',
            name='kinect_publisher',
            output='screen'
        ),

        Node(
            package='py_pkg',
            executable='d_aruco',
            name='kinect_aruco_detector',
            output='screen'
        ),

        Node(
            package='py_pkg',
            executable='d_vel',
            name='velocidad_aruco',
            output='screen'
        ),

        Node(
            package='py_pkg',
            executable='servo',
            name='servo_aruco_sujetar',
            output='screen'
        ),

        Node(
            package='py_pkg',
            executable='lidar',
            name='lidar_points',
            output='screen'
        ),

        Node(
            package='py_pkg',
            executable='trans',
            name='tf_transf',
            output='screen'
        )



    ])