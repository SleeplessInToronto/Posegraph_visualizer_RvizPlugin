"""
lio_sam.launch.py — LIO-SAM backend for the slam_sim_world simulation.

Run AFTER sim_world.launch.py is up:

  ros2 launch slam_sim_world sim_world.launch.py   # terminal 1
  ros2 launch slam_adapters lio_sam.launch.py      # terminal 2

Nodes started:
  lidar_ring_preprocessor   Adds ring field to Gazebo PointCloud2 for LIO-SAM
  lio_sam_imuPreintegration  IMU pre-integration odometry
  lio_sam_imageProjection    Organises raw cloud into range image
  lio_sam_featureExtraction  Extracts edge/surface features
  lio_sam_mapOptimization    Pose-graph optimisation + loop closure
                             → publishes /factor_graph
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('slam_adapters'),
        'lio_sam_sim.yaml')

    sim_time = {'use_sim_time': True}

    # Adds ring field (row index) to the organized Gazebo PointCloud2.
    # LIO-SAM imageProjection needs this to assign each point to a scan ring.
    ring_prep = Node(
        package='slam_adapters',
        executable='lidar_ring_preprocessor',
        remappings=[
            ('input',  '/lidar/points'),
            ('output', '/lidar/points_lio'),
        ],
        parameters=[sim_time],
        output='screen')

    imu_preintegration = Node(
        package='lio_sam',
        executable='lio_sam_imuPreintegration',
        name='lio_sam_imuPreintegration',
        parameters=[config, sim_time],
        output='screen')

    image_projection = Node(
        package='lio_sam',
        executable='lio_sam_imageProjection',
        name='lio_sam_imageProjection',
        parameters=[config, sim_time],
        output='screen')

    feature_extraction = Node(
        package='lio_sam',
        executable='lio_sam_featureExtraction',
        name='lio_sam_featureExtraction',
        parameters=[config, sim_time],
        output='screen')

    map_optimization = Node(
        package='lio_sam',
        executable='lio_sam_mapOptimization',
        name='lio_sam_mapOptimization',
        parameters=[config, sim_time],
        output='screen')

    return LaunchDescription([
        ring_prep,
        imu_preintegration,
        image_projection,
        feature_extraction,
        map_optimization,
    ])
