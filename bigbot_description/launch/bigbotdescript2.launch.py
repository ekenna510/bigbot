#!/usr/bin/env python3
# This accepts urdf, rvizconfig, world, use_sim_time
# uses OpaqueFunction
# usage
# 
# defaults - no sim
# ros2 launch bigbot_description bigbotdescript2.launch.py
#
# defaults sim set
# ros2 launch bigbot_description bigbotdescript2.launch.py use_sim_time:=true
#
# setting everything except default use_rviz
# ros2 launch bigbot_description bigbotdescript2.launch.py model:=robot2.urdf rvizconfig:=odom.rviz world:=livingroom.world use_sim_time:=true
#
# 
# 
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

def launch_setup(context, *args, **kwargs):

    model = LaunchConfiguration('model').perform(context)
    #print("model is ",type(model)," ",model)

    urdf_file = os.path.join(get_package_share_directory('bigbot_description'), 'urdf', model)    
    assert os.path.exists(urdf_file), "The box_bot.xacro doesnt exist in "+str(urdf_file)
    print("urdf_file file ",urdf_file )

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    rsp_params = {'robot_description': robot_desc}
    
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    #print("use_sim_time is ",type(use_sim_time)," ",use_sim_time)

    statenode=Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_desc},{"publish_frequency":2.0}, {'use_sim_time': use_sim_time } ],
        output="both")
    #print("statenode is ",type(statenode)," ",statenode)

    jointnode = Node(package="joint_state_publisher", 
            executable="joint_state_publisher" ,
            name= "joint_state_publisher",
            parameters=[        
                {"robot_description": robot_desc},{"publish_frequency":2.0},
                {"use_gui": 'true' }    
                ],
            output="both" #,condition=IfCondition(LaunchConfiguration('use_sim_time'))  
        )



    rviz_config = LaunchConfiguration('rvizconfig').perform(context)
    #print("rviz_config is ",type(rviz_config)," ",rviz_config)

    rviz_config_dir = os.path.join( get_package_share_directory('bigbot_description'),'rviz',rviz_config)
    print("rviz_config_dir ", rviz_config_dir)

    use_rviz = LaunchConfiguration('use_rviz').perform(context)
    #print("use_rviz is ",type(use_rviz)," ",use_rviz)


    if use_rviz == "true":
        #print("using rviz")
        rviznode = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='both',
            condition=IfCondition(LaunchConfiguration('use_rviz') )
        )
        #print("node created")
        #print("rviznode is ",type(rviznode )," ",rviznode )

    worldfile =  LaunchConfiguration('world').perform(context)
    world = os.path.join(get_package_share_directory('bigbot_gazebo'), 'worlds', worldfile)
    mycmd = 'gazebo'
    print("mycmd",mycmd)
    print("world",world)

    if use_sim_time  == "true":
        print("gazebo")
        # Gazebo launch
        gazebo=ExecuteProcess(
            cmd=[mycmd, '--verbose', '-s', 'libgazebo_ros_factory.so' , world],
            output='both'
        ),
        #print("using sim_time")
        spawnnode = Node(
        	package='spawnrobot', 
    	    executable='spawnrobot2', 
        	arguments=["0.0","0.5","0.01" ], 
    	    output='both'#,            condition=IfCondition(use_sim_time)
    	)
        print("after spawn")
        #print("spawnnode is ",type(spawnnode)," ",spawnnode)


        #print("jointnode is ",type(jointnode)," ",jointnode)
    print("before return")
    if use_sim_time == "true" and use_rviz == "true":
        print("use_sim_time == true and use_rviz = true")
        return [statenode,jointnode,rviznode,gazebo,spawnnode ]  #jointnode 
    else:
        if use_sim_time == "true" and use_rviz == "false":
            print("use_sim_time == true and use_rviz = false")
            return [statenode,jointnode,gazebo,spawnnode] #jointnode 
        else:
            if use_sim_time == "false" and use_rviz == "true":
                print("use_sim_time == true and use_rviz = false")
                return [statenode,jointnode,rviznode ]
            if use_sim_time == "false" and use_rviz == "false":
                print("use_sim_time == true and use_rviz = false")
                return [statenode,jointnode ]

def generate_launch_description():

    # why does these work here when argument decalred below???
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    rvizconfig = LaunchConfiguration('rvizconfig', default='odom.rviz')
    model =  LaunchConfiguration('model', default='robot2.urdf')
    world = LaunchConfiguration('world', default='livingroom.world')

    #rviz_config = LaunchConfiguration('rvizconfig', default='odom.rviz')
    #print(rviz_config)
    #build rviz config

   
    return LaunchDescription([
    DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
    DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Use rviz if true'
        ),
    DeclareLaunchArgument(
            'rvizconfig',
            default_value='odom.rviz',
            description='which rviz config file in rviz folder'),
    DeclareLaunchArgument(
            'model',
            default_value='robot2.urdf',
            description='which model xacro file in urdf folder'),
    DeclareLaunchArgument(
          'world',
          default_value='livingroom.world',
          description='SDF world file'),
    OpaqueFunction(function=launch_setup )  
    ])       
    
     