youbot_camera
=============

Provides a node which calibrates the camera position based on an AR tag fixed to the base.
Also has a launch file for bringing up the camera.

Bringing up the camera
----------------------

```> roslaunch youbot_camera openni.launch```

This will call the openni.launch in the openni_launch package with the camera name xtion.
It will also run a script which will keep the XnSensorServer running until the launch is cancelled.
This is important because the robot has been known to lock up when openni is running without the XnSensorServer.
  
Make sure the camera drivers are installed and the environmental variable XN_SERVER_LOCATION is defined.
If that variable is not defined, find the path to XnSensorServer, which should be in the bin of the installed drivers.
Then put that path into the variable like so:

```> echo XN_SERVER_LOCATION=path```

If you want that variable to be automatically defined in every new terminal you run, add that line to your .bashrc file.
Make sure that the XnSensorServer has global execute permissions.
If it does not, give it such permissions by going to its location and running:

```> sudo chmod a+x XnSensorServer```

Running the calibrator
----------------------

Make sure the ar_pose package is installed. 
If you want a groovy version, one can be found here: https://github.com/mjcarroll/ccny_vision/tree/groovy-devel
Change the camera topics in ar_pose_single.h from camera/stuff to xtion/stuff. (This should really be parameterized)
Run the calibrator:

```> roslaunch youbot_camera ar_camera_calibrator.launch```

This will initially publish a default transform from the gripper_palm_link to the xtion_camera frame.
If your youbot_description and youbot_oodl packages are up to date, the xtion_camera frame will be the root link of the xtion camera.
 
To begin calibration, make sure the ar_marker is within the camera view, then call

```> rosservice call /ar_camera_calibrator/start```

When you are satisfied, lock the position by calling

```> rosservice call /ar_camera_calibrator/stop```
