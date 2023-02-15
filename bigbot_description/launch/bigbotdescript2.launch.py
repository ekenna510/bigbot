
# normal imports
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
# if import
from launch.conditions import IfCondition 
import xacro
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource



def launch_setup(context, *args, **kwargs):

    model = LaunchConfiguration('model').perform(context)
    #print("model is ",type(model)," ",model)

    xacro_file = os.path.join(get_package_share_directory('motorcontrol'), 'urdf', model)    
    assert os.path.exists(xacro_file), "The box_bot.xacro doesnt exist in "+str(xacro_file)
    print("xacro file ",xacro_file )

    robot_description_config = xacro.process_file(xacro_file)

    robot_desc = robot_description_config.toxml()
    #print(" robot_desc ",robot_desc )
    
    statenode=Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
                {"robot_description": robot_desc},{"publish_frequency":2.0}
                ],
        output="both")
    #print("statenode is ",type(statenode)," ",statenode)

    rviz_config = LaunchConfiguration('rvizconfig').perform(context)
    #print("rviz_config is ",type(rviz_config)," ",rviz_config)

    rviz_config_dir = os.path.join( get_package_share_directory('motorcontrol'),'rviz',rviz_config)
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

    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    #print("use_sim_time is ",type(use_sim_time)," ",use_sim_time)
    worldfile =  LaunchConfiguration('world').perform(context)
    world_dir = os.path.join(get_package_share_directory('motorcontrol'), 'worlds', worldfile)

    if use_sim_time  == "true":
        # Gazebo launch
        gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'),
            )
        )        
        #print("using sim_time")
        spawnnode = Node(
        	package='spawnrobot', 
    	    executable='spawnrobot2', 
        	arguments=["0.0","0.5","0.01" ], 
    	    output='both',
            condition=IfCondition(use_sim_time)
    	)
        #print("spawnnode is ",type(spawnnode)," ",spawnnode)

#        jointnode = Node(package="joint_state_publisher", 
#            executable="joint_state_publisher" ,
#            name= "joint_state_publisher",
#            parameters=[        
#                {"robot_description": robot_desc},{"publish_frequency":2.0},
#                {"use_gui": 'true' }    
#                ],
#            output="both",
#            condition=IfCondition(LaunchConfiguration('use_sim_time'))  
#        )
        #print("jointnode is ",type(jointnode)," ",jointnode)
    #print("before return")
    if use_sim_time == "true" and use_rviz == "true":
        return [statenode,gazebo,rviznode,spawnnode ]  #jointnode 
    else:
        if use_sim_time == "true" and use_rviz == "false":
            return [statenode,gazebo,spawnnode] #jointnode 
        else:
            if use_sim_time == "false" and use_rviz == "true":
                return [statenode,rviznode ]
            if use_sim_time == "false" and use_rviz == "false":
                return [statenode ]

def generate_launch_description():

    # why does these work here when argument decalred below???
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    rvizconfig = LaunchConfiguration('rvizconfig', default='odom.rviz')
    model =  LaunchConfiguration('model', default='bigbot4.xacro')
    world = LaunchConfiguration('world', default='robot.world')

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
            default_value='false',
            description='Use rviz if true'
        ),
    DeclareLaunchArgument(
            'rvizconfig',
            default_value='odom.rviz',
            description='which rviz config file in rviz folder'),
    DeclareLaunchArgument(
            'model',
            default_value='bigbot4.xacro',
            description='which model xacro file in urdf folder'),
    DeclareLaunchArgument(
          'world',
          default_value='robot.world',
          description='SDF world file'),
    OpaqueFunction(function=launch_setup )  
    ])       
    
     