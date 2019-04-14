echo 'export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$(rospack find mbzirc_gazebo)/gazebo_models/' >> ~/.bashrc
echo 'export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:$(catkin locate)/build/mbzirc_gazebo' >> ~/.bashrc
echo 'export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:/usr/lib/x86_64-linux-gnu/gazebo-9/plugins/' >> ~/.bashrc
