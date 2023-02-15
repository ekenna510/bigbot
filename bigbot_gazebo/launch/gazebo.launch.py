#!/usr/bin/env python3
# usage
# run full gazebo
# ros2 launch bigbot_gazebo gazebo.launch.py use_sim_time:=true 
# run headless
# ros2 launch bigbot_gazebo gazebo.launch.py use_sim_time:=true guic:=gzserver

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition



def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    sdf_file = os.path.join(get_package_share_directory('bigbot_gazebo'), 'urdf', 'robot2.sdf')    
    assert os.path.exists(sdf_file), "The box_bot.xacro doesnt exist in "+str(sdf_file)
    print(sdf_file)

    world_file_name = 'livingroom.world'
    world = os.path.join(get_package_share_directory('bigbot_gazebo'), 'worlds', world_file_name)

    guic= LaunchConfiguration('guic', default='gazebo')
    #gui= LaunchConfiguration('guic', default='false')
    #mycmd = 'gzserver'
    #if (gui == 'true'):
    #    mycmd = 'gazebo'
    #    print("full")
    #else:
    #    print("headless")

    #print("mycmd",mycmd)
    #print("world",world)

    return LaunchDescription([

        ExecuteProcess(
            cmd=[guic, '--verbose', '-s', 'libgazebo_ros_factory.so' , world],
            output='both',
            condition=IfCondition(use_sim_time)
            ),


        Node(package='spawnrobot', 
        executable='spawnrobot2', 
        arguments=['0.01','0.02','0.03' ], 
        output='both',
        condition=IfCondition(use_sim_time)
        ),


  ])