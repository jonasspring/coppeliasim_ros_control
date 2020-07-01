## coppeliasim_ros_control
This package contains a ros_control back end for coppeliasim (i.e. ~ gazebo_ros_control but for coppeliasim).
### Dependencies:
  * [vrep_ros_packages:](https://github.com/jhu-lcsr/vrep_ros_packages)
   - vrep_common
   - vrep_skeleton_msg_and_srv
  
### HOW TO RUN IT:
### general plugin/master branch:
   * clone the repository in your <workspace>/src folder
   * compile the package
   * once compiled, you need to copy the plugin file libsimExtRosControl.so from your <workspace>/devel/lib folder to coppeliasim's main folder (along all the other plugins)
   * start a terminal with roscore
   * then start coppeliasim 
      - check that plugin libsimExtRosControl.so is correctly loaded in coppeliasim's trace (i.e. in coppeliasim's console)
   * load any robot in coppeliasim, load the robot_description to the param server.
   * then start the simulation in coppeliasim
   * create configuration [yaml] file for your controller 
   * use current launch file to load your load ros_controller 

#### Notes:
   *  currenlty, both position and velocity controllers works properly. but the effort controller not properly working!
