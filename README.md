## coppeliasim_ros_control
This package contains a ros_control back end for coppeliasim (i.e. ~ gazebo_ros_control but for coppeliasim).
### Dependencies:
  * [vrep_ros_packages:](https://github.com/jhu-lcsr/vrep_ros_packages)
   - vrep_common
   - vrep_skeleton_msg_and_srv
  
### HOW TO RUN IT:
### general plugin/master branch:
   * clone the repository in your <workspace>/src folder
   * build the newly cloned package
   * once built, you need to copy this plugin file libsimExtRosControl.so from your <workspace>/devel/lib folder to coppeliasim's main folder (along all the other plugins)
   * start a terminal with roscore
   * then start coppeliasim 
      - check that plugin libsimExtRosControl.so is correctly loaded in coppeliasim's trace (i.e. in coppeliasim's console)
   * load a scene from coppeliasim folder and start the simulation in coppeliasim
   * create controller.yaml configuration file 
   * use current launch file to load your robot urdf file 
