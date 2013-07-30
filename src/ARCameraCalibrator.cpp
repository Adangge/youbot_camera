#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <iostream>
#include <fstream>

bool calibrate=false; //whether to calibrate the camera this step
tf::Transform transform_gripper2camera;
std::string urdf_location=""; //location of the urdf file, read from a param
std::vector<std::string> lines; //every line in the urdf file
int whichline = 0; //which line to change

bool read_urdf(std::string location) {
  //open the file for read and write
  ROS_INFO_STREAM("Will open urdf at location " << location);
  std::ifstream myfile;
  myfile.open(urdf_location.c_str());
  if (myfile.is_open()) {
    ROS_INFO("Urdf is open, reading...");
    lines.clear();
    std::string line;
    int i=0;
    whichline=0;
    bool last_line_was_flag=false;

    while (myfile.good())
    {
      //push each line into the vector
      getline(myfile,line);
      lines.push_back(line);

      //see if the current line is the flag
      if ( !last_line_was_flag
           && (line.find("AR_CALIBRATOR:EDIT_HERE"))!=std::string::npos)
      {
        last_line_was_flag=true;
      }
      //if the last was the flag, check that this line has an origin tag
      if (last_line_was_flag
           && (line.find("origin")!=std::string::npos))
      {
        whichline=i;
        ROS_INFO("Found line to edit: line %d: '%s'",i,line.c_str());
      }
      else last_line_was_flag=false;
      i++;
    } //end of file
    myfile.close();
    if (whichline==0)
    {
      ROS_ERROR("Desired line not found in urdf");
      return false;
    }
    return true;
  } //end of 'if (myfile.is_open())'
  else {
    ROS_ERROR("Error openning urdf file");
    return false;
  }
}

bool write_urdf() {
  //replace the urdf's camera pose with the calibrated pose
  double x,y,z,r,p,yaw;
  x=transform_gripper2camera.getOrigin().getX();
  y=transform_gripper2camera.getOrigin().getY();
  z=transform_gripper2camera.getOrigin().getZ();
  tf::Matrix3x3(transform_gripper2camera.getRotation()).getRPY(r,p,yaw);
  std::stringstream updated_line;
  updated_line << "                  <origin xyz=\"";
  updated_line << x << " " << y << " " << z;
  updated_line << " rpy=\"" << r << " " << p << " " << yaw << "\" />";
  ROS_INFO_STREAM("replacing old origin with: '" << updated_line << "'");
  lines.at(whichline)=updated_line.str();

  //copy old file before writing in case something goes wrong
  std::ifstream src(urdf_location.c_str(), std::ios::binary);
  std::stringstream temp;
  temp << urdf_location << ".temp";
  std::ofstream dst(temp.str().c_str(),std::ios::binary);
  if (src.is_open() && dst.is_open()) dst << src.rdbuf();
  else {
    ROS_ERROR("Error creating backup of urdf");
    return false;
  }

  //write new urdf file with modified camera origin
  std::ofstream myfile;
  //open the file for writing, truncate because we're writing the whole thing
  myfile.open(urdf_location.c_str(), std::ios::out | std::ios::trunc);
  if (myfile.is_open()) {
    ROS_INFO("Urdf is open, writing...");
    for (std::vector<std::string>::iterator it = lines.begin(); it != lines.end(); ++it)
    {
      //write each line to the file
      if (myfile.good()) myfile<<(*it)<<std::endl;
      else
      {
        ROS_ERROR("Something went wrong with writing the urdf");
        return false;
      }
    } //end iterator
    myfile.close();
    return true;
  } //end of 'if (myfile.is_open())'
  else {
    ROS_ERROR("Error opening the urdf file for writing");
    return false;
  }
}

bool calibration_start(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  //recalculate gripper->openni_pose transform
  ROS_INFO("Beginning camera pose calibration based on ar tag location");
  calibrate=true;
  return true;
}

int main(int argc, char** argv) {
  ros::init(argc,argv,"ar_camera_calibrator");
  ros::NodeHandle n;

  tf::TransformListener tf_listener;
  tf::TransformBroadcaster tf_broadcaster;

  ros::ServiceServer start_service = n.advertiseService("ar_camera_calibrator/calibrate", calibration_start);

  //get the urdf location parameter
  if (n.getParam("/urdf_location", urdf_location)) {

    if (read_urdf(urdf_location))
    {
      //set the transform to the one that was read
      double x,y,z,r,p,yaw;
      if (sscanf(lines.at(whichline).c_str(),
            "%*s<origin xyz=\"%lf %lf %lf\" rpy=\"%lf %lf %lf\" />%*s",
            &x,&y,&z,&r,&p,&yaw)
          <6)
      {
        ROS_ERROR("Error parsing origin tag, shutting down");
        return 0;
      }
      else {
        ROS_INFO("Read transform xyz=%lf %lf %lf  rpy=%lf %lf %lf",x,y,z,r,p,y);
        transform_gripper2camera.setOrigin(tf::Vector3(x,y,z));
        transform_gripper2camera.setRotation(tf::createQuaternionFromRPY(r,p,y));
      }
    }
    else {
      ROS_ERROR("Error reading urdf, shutting down");
      return 0;
    }
  }
  else {
    ROS_ERROR_STREAM("Error getting urdf location, quitting");
//    transform_gripper2camera.setOrigin(tf::Vector3(0.04,0,0));
//    transform_gripper2camera.setRotation(tf::createQuaternionFromRPY(0,1.57,0));
    return 0;
  }

  ros::Rate rate(10.0);
  while (n.ok()) {

    if (calibrate)
    {

      //tranform from base_footprint to gripper_palm_link
      tf::Transform transform_marker2camera;
      bool ok = true;
            //get transform from ar marker to camera
      try {
        tf::StampedTransform transform;
        tf_listener.lookupTransform(
  		"/ar_marker",
  		"/xtion_camera",
  		ros::Time(0), //latest
  		transform);
	tf::Quaternion q;
        q.setRPY(0,0,1.57);
	tf::Transform rot;
	rot.setRotation(q);
	transform_marker2camera=rot*transform;
      }
      catch (tf::TransformException ex) {
        ROS_ERROR("ARCameraCalibrator.cpp: couldn't get grippper orientation: %s",ex.what());
        ok=false;
      }

      if (ok) {
	tf_broadcaster.sendTransform(
		tf::StampedTransform(transform_marker2camera,ros::Time::now(),
		"ar_marker_fixed", "xtion_calibrated"));
	try {
	  tf::StampedTransform transform;
          tf_listener.lookupTransform(
  		"/gripper_palm_link",
  		"/xtion_calibrated",
  		ros::Time(0), //latest
  		transform);
	  transform_gripper2camera.setOrigin(transform.getOrigin());
	  transform_gripper2camera.setRotation(transform.getRotation());
          calibrate=false;
          if (!write_urdf()) {
            ROS_ERROR("Couldn't write URDF, shutting down");
            return 0;
          }
	}
        catch (tf::TransformException ex) {
          ROS_ERROR("ARCameraCalibrator.cpp: couldn't get grippper to camera transform: %s",ex.what());
        }
      }
    } //end "if calibrate"

    ros::spinOnce();
    rate.sleep();
  }

}

/*
//get transform from ar_marker_fixed to ar_marker
      tf::Transform transform_arDiff;
      try {
        tf::StampedTransform transform;
        tf_listener.lookupTransform(
  		"/ar_marker",
  		"/ar_marker_fixed",
  		ros::Time(0), //latest
  		transform);
	transform_arDiff.setOrigin(transform.getOrigin());
	transform_arDiff.setRotation(transform.getRotation());
        ROS_INFO("AR orientation difference: %f", 
		tf::tfDot(
			tf::Vector3(0,0,1),
			tf::quatRotate(transform_arDiff.getRotation(),tf::Vector3(0,0,1))
		)
	);
      }
      catch (tf::TransformException ex) {
        ROS_ERROR("ARCameraCalibrator.cpp: couldn't get ar marker error: %s",ex.what());
        ok=false;
      }
*/
