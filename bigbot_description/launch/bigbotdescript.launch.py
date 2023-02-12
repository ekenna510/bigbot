#!/usr/bin/env python3
# usage
# ros2 launch bigbot_description bigbotdescript.launch.py use_sim_time:=false
# ros2 launch bigbot_description bigbotdescript.launch.py use_rviz:=true use_sim_time:=false
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess

import xacro
from launch.actions import OpaqueFunction




def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    use_rviz = LaunchConfiguration('use_rviz', default='true')



    urdf_file = os.path.join(get_package_share_directory('bigbot_description'), 'urdf', 'robot2.urdf')    
    assert os.path.exists(urdf_file), "The  doesnt exist in "+str(urdf_file)
    print(urdf_file)

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    rsp_params = {'robot_description': robot_desc}

    sdf_file = os.path.join(get_package_share_directory('bigbot_description'), 'urdf', 'robot2.sdf')    
    assert os.path.exists(sdf_file), "The box_bot.xacro doesnt exist in "+str(sdf_file)
    print(sdf_file)

    rviz_file =os.path.join(get_package_share_directory('bigbot_description'), 'rviz', 'odom.rviz')   
    assert os.path.exists(rviz_file), "The rviz doesnt exist in "+str(rviz_file)
    print(rviz_file)

    world_file_name = 'livingroom.world'
    world = os.path.join(get_package_share_directory('motorcontrol'), 'worlds', world_file_name)
    print(world)

    # packagepath 
    pkg_motorcontrol = os.path.join(get_package_share_directory('motorcontrol'), 'launch')
    print(pkg_motorcontrol)

    mycmd='gazebo'

    return LaunchDescription([

    DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
    DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Use rviz if true'),
     Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
                {"robot_description": robot_desc},{"publish_frequency":2.0}],
        output="both"),
    Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_file],
        output='both',
        condition=IfCondition(use_rviz)
        ),


    # Node(
    #     package="joint_state_publisher", 
    #     executable="joint_state_publisher" ,
    #     name= "joint_state_publisher",
    #     parameters=[        
    #         {"robot_description": robot_desc},{"publish_frequency":2.0},
    #         {"use_gui": 'true' }    
    #             ],
    #     output="both",
    #     condition=IfCondition(use_sim_time)
    #     ),
    # ExecuteProcess(
    #         cmd=[mycmd, '--verbose', '-s', 'libgazebo_ros_factory.so' , world],
    #         output='both',
    #         condition=IfCondition(use_sim_time)
    #         ),
    # Node(package='spawnrobot', 
    #     executable='spawnrobot2', 
    #     arguments=['0.01','0.02','0.03' ], 
    #     output='screen',
    #     condition=IfCondition(use_sim_time)
    #     ),

  ])