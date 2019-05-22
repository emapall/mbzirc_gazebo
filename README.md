# mbzirc_gazebo

This repo contains all the gazebo-relative files necessary for the simulation of our drone (of course, standard gazebo plugins are not contained in this repo, since they are installed along with Gazebo simulator).

This repo, although being a submodule of the main MBZRIC repo, contains all the necessary files to simulate our drone in gazebo and be controlled with the ardupilot plugin.

#### Worlds
The name of the worlds are self explanatory: they contain the drone, and may or may not contain 
  -an actor, wihich is a red baloon circling
  -the skin for the environment 
  
  
 ## ROS and camera: IMPORTANT 
 ###### ROS
Currently, inside the /launch folder, there are two launch files that launch gazebo as a ros package, inside ROS framework. Currenlty, if you want to use the camera, it will NOT work if you launch gazebo inside ros

###### CAMERA
In order to obtain the simulated images, gazebo saves them in ```gazebo_models/MBZIRColo_gimbal_camera/model.sdf``` 
```
          <save enabled="true">
            <path>/tmp/MBZIRColo_camera_raw</path>
          </save>
```
This folder is memor-mapped, meaning that although on the file system, it is saved in your ram, so it deletes (as all the /tmp folder) when you reboot. The point is that gazebo is saving images ad a certain framerate ```<update_rate>number, Hertz</update_rate>``` so it can quickly fill up your pc' memory. If you wish NOT to use the camera, then just change the code and disable the saving ```save enabled="false"```; Obviously, you can also change the framerate. 

If you do, please be careful not to commit these changes onto the repository, and remember that when pulling new changes, you may need to re-do the changes. 

## STARTING THE SIMULATION W/O ROS

navigate in the main folder of this repo (e.g. ```roscd mbzirc_gazebo```);
``` 
gazebo --verbose [-u if you want the simulation to start paused] worlds/world_name.world
```
Then you should start ardupilot with 
```
cd <folder in which you installed ARDUPILOT>/ArduCopter
sim_vehicle.py --console -f gazebo-MBZIRColo --map 
```
But if you are using the control module, it is highly likely that when you start that module, you start also ardupilot.

## STARTING THE SIMULATION W ROS
just ```
roslaunch mbzirc_gazebo <name of one of the files in /launch>.launch ``` 
(at the present moment, you shouldn't need to use gazebo within ROS)
