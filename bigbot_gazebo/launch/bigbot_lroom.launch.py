#!/usr/bin/env python3
# usage
#ros2 launch bigbot_gazebo bigbot_lroom.launch.py use_sim_time:=true guic:=true
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch_ros.actions 
from launch.conditions import IfCondition


def generate_launch_description():
    #gui1 = launch.actions.DeclareLaunchArgument( 'gui', default_value='false')
    #print(gui1)
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    guic= LaunchConfiguration('guic', default='true')

    #world_file_name = 'turtlebot3_houses/' + TURTLEBOT3_MODEL + '.model'
    world_file_name = 'livingroom.world'
    world = os.path.join(get_package_share_directory('bigbot_gazebo'), 'worlds', world_file_name)
    launch_file_dir = os.path.join(get_package_share_directory('bigbot_gazebo'), 'launch')
    spawn_launch_dir=os.path.join(get_package_share_directory('spawnrobot'), 'launch')

    #urdffile = os.path.join(get_package_share_directory('turtlebot_description'),''

    namespacename=''
    x='0.0'
    y='0.0'
    z='0.0'
    #doc=xacro.process_file()


    #guic.parse
    #mycmd = 'gzserver'
    #if (gui == 'true'):
    mycmd = 'gazebo'
    #    print("full")
    #else:
    #    print("headless")

    print("mycmd",mycmd)
    print("world",world)
    #

    urdf_file_name = 'robot2.urdf'



    urdf = os.path.join(
        get_package_share_directory('bigbot_description'),
        'urdf',
        urdf_file_name)
    print("urdf : {}".format(urdf))        

    # Major refactor of the robot_state_publisher
    # Reference page: https://github.com/ros2/demos/pull/426
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    rsp_params = {'robot_description': robot_desc}

    return LaunchDescription( [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),    
        DeclareLaunchArgument(
            'guic',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),    

        ExecuteProcess(
            cmd=[mycmd, '--verbose', '-s', 'libgazebo_ros_factory.so' , world],
            output='both'),

        #IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher.launch.py']),
        #    launch_arguments={'use_sim_time': use_sim_time}.items()
        #    #parameters=[{'publish_frequency':100}],
        #
        #),

        Node(package='spawnrobot', 
        executable='spawnrobot2', 
        arguments=['0.01','0.02','0.03' ], 
        output='both',
        condition=IfCondition(use_sim_time)
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[rsp_params, {'use_sim_time': use_sim_time}])        


    ]
    )
    