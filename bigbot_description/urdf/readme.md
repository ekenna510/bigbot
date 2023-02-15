# xacro description

## bigbot.xacro
root is base_footprint base_link is wheel radius above base_footprint everything else off base_link. Laser is not rotated

> **Gazebo**
> - differential_drive_controller using base_footprint
> - gpu_ray laser

## bigbot2.xacro root is base_link  This attaches to chassis wheels and laser off chassis. Laser is rotated 90 degrees.

**Gazebo**
- differential_drive_controller using base_link
- gpu_ray laser

## bigbot3.xacro 
root is base_footprint baselink is wheel radius above base_footprint everything else off base_link. Laser is rotated 90 degrees

**Gazebo**
- differential_drive_controller using base_footprint
- gpu_ray laser

## bigbot3a.xacro 
This only has base_footprint and baselink some kind of test

## bigbot4.xacro 
root is base_footprint base_link is wheel_radius above base_footprint. everything else off base_link.	Laser is rotated 90 degrees

**Gazebo**
- differential_drive_controller using base_footprint
- joint_state
- gpu_ray laser

## bigbot5.xacro
m2wr pulled off internet somewhere with chassis. Some test I did.

## build urdf
substitute correct number

xacro bigbot2.xacro > robot2.urdf
gz sdf -p  robot2.urdf > robot2.sdf