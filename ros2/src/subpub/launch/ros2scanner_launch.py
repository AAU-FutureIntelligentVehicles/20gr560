from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
	return LaunchDescription([
		Node(
			package='subpub',
			node_executable='scanner',
			name='scanner'
		),
		Node(
			package='rviz2',
			node_namespace='rviz2',
			node_executable='rviz2',
			name='rviz',
			arguments=['-d/home/vdr/ros2/src/subpub/launch/scanconfig.rviz']
		),
	
])