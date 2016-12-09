#include <algorithm>
#include <assert.h>

#include <vulcano_base_gazebo/omnidrive_plugin.h>

#include <gazebo/math/gzmath.hh>
#include <sdf/sdf.hh>

#include <math.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Odometry.h>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/circular_buffer.hpp>

namespace gazebo {

  static double normalize(double z)
  {
    return atan2(sin(z),cos(z));
  }
  
  static double angle_diff(double a, double b)
  {
    double d1, d2;
    a = normalize(a);
    b = normalize(b);
    d1 = a-b;
    d2 = 2*M_PI - fabs(d1);
    if(d1 > 0)
      d2 *= -1.0;
    if(fabs(d1) < fabs(d2))
      return(d1);
    else
      return(d2);
  }
  
  enum {
	FRONT_RIGHT_W=0,
	FRONT_LEFT_W=1,
    BACK_RIGHT_W=2,
    BACK_LEFT_W=3,
	FRONT_RIGHT_MW=4,
	FRONT_LEFT_MW=5,
    BACK_RIGHT_MW=6,
    BACK_LEFT_MW=7,
  };


  OmniDrivePlugin::OmniDrivePlugin() {  }

  // Destructor
  OmniDrivePlugin::~OmniDrivePlugin() {
    delete transform_broadcaster_;
    rosnode_->shutdown();
    delete rosnode_;
  }

  // Load the controller
  void OmniDrivePlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {

    parent = _parent;
    world = _parent->GetWorld();
   
    robot_namespace_ = "";
    if (!_sdf->HasElement("robotNamespace"))
      ROS_INFO("OmniDrivePlugin Plugin missing <robotNamespace>, defaults to \"%s\"", robot_namespace_.c_str());
    else
      robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
    
    broadcast_tf_ = false;
    if (!_sdf->HasElement("broadcastTF")) 
      if (!broadcast_tf_)
    	  ROS_INFO("OmniDrivePlugin Plugin (ns = %s) missing <broadcastTF>, defaults to false.",  robot_namespace_.c_str());
      else
	ROS_INFO("OmniDrivePlugin Plugin (ns = %s) missing <broadcastTF>, defaults to true.", robot_namespace_.c_str());    
    else
       broadcast_tf_ = _sdf->GetElement("broadcastTF")->Get<bool>();
   
    // Wheels
    joint_front_right_wheel_name_ = "front_right_wheel_joint";
    if (!_sdf->HasElement("frontRightWheelJoint")) 
      ROS_WARN("OmniDrivePlugin Plugin (ns = %s) missing <frontRightWheelJoint>, defaults to \"%s\"",
	       robot_namespace_.c_str(), joint_front_right_wheel_name_.c_str());
    else
      joint_front_right_wheel_name_ = _sdf->GetElement("frontRightWheelJoint")->Get<std::string>();
    
    joint_front_left_wheel_name_ = "front_left_wheel_joint";
    if (!_sdf->HasElement("frontLeftWheelJoint")) 
      ROS_WARN("OmniDrivePlugin Plugin (ns = %s) missing <frontLeftWheelJoint>, defaults to \"%s\"",
	       robot_namespace_.c_str(), joint_front_left_wheel_name_.c_str());
    else
      joint_front_left_wheel_name_ = _sdf->GetElement("frontLeftWheelJoint")->Get<std::string>();
     
    joint_back_right_wheel_name_ = "back_right_wheel_joint";
    if (!_sdf->HasElement("backRightWheelJoint")) 
      ROS_WARN("OmniDrivePlugin Plugin (ns = %s) missing <backRightWheelJoint>, defaults to \"%s\"",
	       robot_namespace_.c_str(), joint_back_right_wheel_name_.c_str());
    else
      joint_back_right_wheel_name_ = _sdf->GetElement("backRightWheelJoint")->Get<std::string>();

    joint_back_left_wheel_name_ = "back_left_wheel_joint";
    if (!_sdf->HasElement("backLeftWheelJoint")) 
      ROS_WARN("OmniDrivePlugin Plugin (ns = %s) missing <backLeftWheelJoint>, defaults to \"%s\"",
	       robot_namespace_.c_str(), joint_back_left_wheel_name_.c_str());
    else
      joint_back_left_wheel_name_ = _sdf->GetElement("backLeftWheelJoint")->Get<std::string>();


    // Motor Wheels
    joint_front_right_motor_wheel_name_ = "front_right_motor_wheel_joint";
    if (!_sdf->HasElement("frontRightMotorWheelJoint")) 
      ROS_WARN("OmniDrivePlugin Plugin (ns = %s) missing <frontRightMotorWheelJoint>, defaults to \"%s\"",
	       robot_namespace_.c_str(), joint_front_right_motor_wheel_name_.c_str());
    else
      joint_front_right_motor_wheel_name_ = _sdf->GetElement("frontRightMotorWheelJoint")->Get<std::string>();
    
    joint_front_left_motor_wheel_name_ = "front_left_motor_wheel_joint";
    if (!_sdf->HasElement("frontLeftMotorWheelJoint")) 
      ROS_WARN("OmniDrivePlugin Plugin (ns = %s) missing <frontLeftMotorWheelJoint>, defaults to \"%s\"",
	       robot_namespace_.c_str(), joint_front_left_motor_wheel_name_.c_str());
    else
      joint_front_left_motor_wheel_name_ = _sdf->GetElement("frontLeftMotorWheelJoint")->Get<std::string>();
     
    joint_back_right_motor_wheel_name_ = "back_right_motor_wheel_joint";
    if (!_sdf->HasElement("backRightMotorWheelJoint")) 
      ROS_WARN("OmniDrivePlugin Plugin (ns = %s) missing <backRightMotorWheelJoint>, defaults to \"%s\"",
	       robot_namespace_.c_str(), joint_back_right_motor_wheel_name_.c_str());
    else
      joint_back_right_motor_wheel_name_ = _sdf->GetElement("backRightMotorWheelJoint")->Get<std::string>();

    joint_back_left_motor_wheel_name_ = "back_left_motor_wheel_joint";
    if (!_sdf->HasElement("backLeftMotorWheelJoint")) 
      ROS_WARN("OmniDrivePlugin Plugin (ns = %s) missing <backLeftMotorWheelJoint>, defaults to \"%s\"",
	       robot_namespace_.c_str(), joint_back_left_motor_wheel_name_.c_str());
    else
      joint_back_left_motor_wheel_name_ = _sdf->GetElement("backLeftMotorWheelJoint")->Get<std::string>();
        
    wheel_diameter_ = 0.186;
    if (!_sdf->HasElement("wheelDiameter")) 
      ROS_WARN("OmniDrivePlugin Plugin (ns = %s) missing <wheelDiameter>, defaults to %f",
	       robot_namespace_.c_str(),wheel_diameter_);
    else 
      wheel_diameter_ = _sdf->GetElement("wheelDiameter")->Get<double>();
    
    
    wheel_base_ = 0.934;
    if (!_sdf->HasElement("wheelBase")) 
      ROS_WARN("OmniDrivePlugin Plugin (ns = %s) missing <wheelBase>, defaults to value from robot_description: %f",
	       robot_namespace_.c_str(), wheel_base_);
    else
      wheel_base_ =_sdf->GetElement("wheelBase")->Get<double>();
      
    track_width_ = 0.57;
    if (!_sdf->HasElement("trackWidth")) 
      ROS_WARN("OmniDrivePlugin Plugin (ns = %s) missing <trackWidth>, defaults to value from robot_description: %f",
	       robot_namespace_.c_str(), track_width_);
    else
      track_width_ =_sdf->GetElement("trackWidth")->Get<double>();

    wheel_torque_ = 10.0;
    if (!_sdf->HasElement("wheelTorque")) 
      ROS_WARN("OmniDrivePlugin Plugin (ns = %s) missing <wheelTorque>, defaults to %f",
	       robot_namespace_.c_str(), wheel_torque_);
    else 
      wheel_torque_ = _sdf->GetElement("wheelTorque")->Get<double>();

    motor_wheel_torque_ = 50.0;
    if (!_sdf->HasElement("motorWheelTorque")) 
      ROS_WARN("OmniDrivePlugin Plugin (ns = %s) missing <motorWheelTorque>, defaults to %f",
	       robot_namespace_.c_str(), motor_wheel_torque_);
    else 
      motor_wheel_torque_ = _sdf->GetElement("motorWheelTorque")->Get<double>();
    

    command_topic_ = "cmd_vel";
    if (!_sdf->HasElement("commandTopic")) 
      ROS_WARN("OmniDrivePlugin Plugin (ns = %s) missing <commandTopic>, defaults to \"%s\"",
          robot_namespace_.c_str(), command_topic_.c_str());
    else 
      command_topic_ = _sdf->GetElement("commandTopic")->Get<std::string>();
    
    odometry_topic_ = "odom";
    if (!_sdf->HasElement("odometryTopic")) 
      ROS_WARN("OmniDrivePlugin Plugin (ns = %s) missing <odometryTopic>, defaults to \"%s\"",
          robot_namespace_.c_str(), odometry_topic_.c_str());
    else
      odometry_topic_ = _sdf->GetElement("odometryTopic")->Get<std::string>();
    
    odometry_frame_ = "odom";
    if (!_sdf->HasElement("odometryFrame")) 
      ROS_WARN("OmniDrivePlugin Plugin (ns = %s) missing <odometryFrame>, defaults to \"%s\"",
         robot_namespace_.c_str(), odometry_frame_.c_str());
    else
      odometry_frame_ = _sdf->GetElement("odometryFrame")->Get<std::string>();
 
    robot_base_frame_ = "base_footprint";
    if (!_sdf->HasElement("robotBaseFrame")) 
      ROS_WARN("OmniDrivePlugin Plugin (ns = %s) missing <robotBaseFrame>, defaults to \"%s\"",
          robot_namespace_.c_str(), robot_base_frame_.c_str());
    else 
      robot_base_frame_ = _sdf->GetElement("robotBaseFrame")->Get<std::string>();
    
    update_rate_ = 100.0;
    if (!_sdf->HasElement("updateRate")) 
      ROS_WARN("OmniDrivePlugin (ns = %s) missing <updateRate>, defaults to %f",
          robot_namespace_.c_str(), update_rate_);
    else
      update_rate_ = _sdf->GetElement("updateRate")->Get<double>();

	// Odometry source
	odom_type_ = WORLD; 
	std::string odom_source;
	if (!_sdf->HasElement("odometrySource"))
		ROS_WARN("OmniDrivePlugin (ns = %s) missing <odometrySource>, defaults to 'world'", robot_namespace_.c_str());
	else {
	    odom_source = _sdf->GetElement("odometrySource")->Get<std::string>();		    
	    if (odom_source == "world") odom_type_ = WORLD;
	    else if (odom_source == "encoder") odom_type_ = ENCODER;
	    else ROS_ERROR("<odometrySource> tag not recognized, defaults to 'world'");
		}
    
    this->mirrored_axes_ = false;
    spin_ = 1.0;
    if (!_sdf->HasElement("mirrored_axes")) 
      ROS_WARN("OmniDrivePlugin (ns = %s) missing <mirrored_axes>, defaults to false", robot_namespace_.c_str());
    else 
      this->mirrored_axes_ = _sdf->GetElement("mirrored_axes")->Get<bool>(); 
    if (this->mirrored_axes_) spin_ = -1.0;
    

    joint_state_history_size_ = update_rate_;
    if (!_sdf->HasElement("jointStateHistorySize"))
        ROS_WARN("OmniDrivePlugin (ns = %s) missing jointStateHistorySize, defaults to %u (updateRate)", robot_namespace_.c_str(), joint_state_history_size_);
    else
      joint_state_history_size_ = _sdf->GetElement("jointStateHistorySize")->Get<unsigned int>(); 

    joint_state_mean_.resize(NUMBER_OF_JOINTS);
    joint_state_history_.resize(NUMBER_OF_JOINTS);
    for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
        joint_state_history_[i] = boost::circular_buffer<double>(joint_state_history_size_);
    }

    // Initialize update rate stuff
    if (update_rate_ > 0.0)
      update_period_ = 1.0 / update_rate_;
    else
      update_period_ = 0.0;
    last_update_time_ = world->GetSimTime();

    // Initialize velocity stuff
    joint_reference_[FRONT_RIGHT_W] = 0;
    joint_reference_[FRONT_LEFT_W] = 0;
    joint_reference_[BACK_RIGHT_W] = 0;
    joint_reference_[BACK_LEFT_W] = 0;
    joint_reference_[FRONT_RIGHT_MW] = 0;
    joint_reference_[FRONT_LEFT_MW] = 0;
    joint_reference_[BACK_RIGHT_MW] = 0;
    joint_reference_[BACK_LEFT_MW] = 0;

    v_ref_x_ = 0;
    v_ref_y_ = 0;
    w_ref_ = 0;    
    alive_ = true;

    joints_[FRONT_LEFT_W] =parent->GetJoint(joint_front_left_wheel_name_);
    joints_[FRONT_RIGHT_W] = parent->GetJoint(joint_front_right_wheel_name_);
    joints_[BACK_LEFT_W] = parent->GetJoint(joint_back_left_wheel_name_);
    joints_[BACK_RIGHT_W] = parent->GetJoint(joint_back_right_wheel_name_);
    joints_[FRONT_LEFT_MW] =parent->GetJoint(joint_front_left_motor_wheel_name_);
    joints_[FRONT_RIGHT_MW] = parent->GetJoint(joint_front_right_motor_wheel_name_);
    joints_[BACK_LEFT_MW] = parent->GetJoint(joint_back_left_motor_wheel_name_);
    joints_[BACK_RIGHT_MW] = parent->GetJoint(joint_back_right_motor_wheel_name_);


    if (!joints_[FRONT_LEFT_W]) {
      char error[200];
      snprintf(error, 200,
          "OmniDrivePlugin Plugin (ns = %s) couldn't get left front hinge joint named \"%s\"",
	       robot_namespace_.c_str(), joint_front_left_wheel_name_.c_str());
      gzthrow(error);
    }

    if (!joints_[FRONT_RIGHT_W]) {
      char error[200];
      snprintf(error, 200,
          "OmniDrivePlugin Plugin (ns = %s) couldn't get right front hinge joint named \"%s\"",
	       robot_namespace_.c_str(), joint_front_right_wheel_name_.c_str());
      gzthrow(error);
    }

    if (!joints_[BACK_LEFT_W]) {
	 char error[200];
	 snprintf(error, 200,
		 "OmniDrivePlugin Plugin (ns = %s) couldn't get left rear hinge joint named \"%s\"",
		  robot_namespace_.c_str(), joint_back_left_wheel_name_.c_str());
	 gzthrow(error);
   }

   if (!joints_[BACK_RIGHT_W]) {
	 char error[200];
	 snprintf(error, 200,
		  "OmniDrivePlugin Plugin (ns = %s) couldn't get right rear hinge joint named \"%s\"",
		 robot_namespace_.c_str(), joint_back_right_wheel_name_.c_str());
	 gzthrow(error);
   }

    if (!joints_[FRONT_LEFT_MW]) {
      char error[200];
      snprintf(error, 200,
          "OmniDrivePlugin Plugin (ns = %s) couldn't get left front hinge joint named \"%s\"",
	       robot_namespace_.c_str(), joint_front_left_motor_wheel_name_.c_str());
      gzthrow(error);
    }

    if (!joints_[FRONT_RIGHT_MW]) {
      char error[200];
      snprintf(error, 200,
          "OmniDrivePlugin Plugin (ns = %s) couldn't get right front hinge joint named \"%s\"",
	       robot_namespace_.c_str(), joint_front_right_motor_wheel_name_.c_str());
      gzthrow(error);
    }

    if (!joints_[BACK_LEFT_MW]) {
	 char error[200];
	 snprintf(error, 200,
		 "OmniDrivePlugin Plugin (ns = %s) couldn't get left rear hinge joint named \"%s\"",
		  robot_namespace_.c_str(), joint_back_left_motor_wheel_name_.c_str());
	 gzthrow(error);
   }

   if (!joints_[BACK_RIGHT_MW]) {
	 char error[200];
	 snprintf(error, 200,
		  "OmniDrivePlugin Plugin (ns = %s) couldn't get right rear hinge joint named \"%s\"",
		 robot_namespace_.c_str(), joint_back_right_motor_wheel_name_.c_str());
	 gzthrow(error);
   }

    joints_[FRONT_LEFT_W]->SetMaxForce(0, wheel_torque_);
    joints_[FRONT_RIGHT_W]->SetMaxForce(0, wheel_torque_);
    joints_[BACK_LEFT_W]->SetMaxForce(0, wheel_torque_);
    joints_[BACK_RIGHT_W]->SetMaxForce(0, wheel_torque_);
    joints_[FRONT_LEFT_MW]->SetMaxForce(0, motor_wheel_torque_);
    joints_[FRONT_RIGHT_MW]->SetMaxForce(0, motor_wheel_torque_);
    joints_[BACK_LEFT_MW]->SetMaxForce(0, motor_wheel_torque_);
    joints_[BACK_RIGHT_MW]->SetMaxForce(0, motor_wheel_torque_);

    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    pose_encoder_.x = 0;
    pose_encoder_.y = 0;
    pose_encoder_.theta = 0;

    rosnode_ = new ros::NodeHandle(robot_namespace_);

    ROS_INFO("Starting OmniDrivePlugin Plugin (ns = %s)!", robot_namespace_.c_str());

    tf_prefix_ = tf::getPrefixParam(*rosnode_);
    transform_broadcaster_ = new tf::TransformBroadcaster();

    // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
    ros::SubscribeOptions so =
      ros::SubscribeOptions::create<geometry_msgs::Twist>(command_topic_, 1,
          boost::bind(&OmniDrivePlugin::cmdVelCallback, this, _1),
          ros::VoidPtr(), &queue_);

    //Subscribe to Gazebo model states to get the real angular speed
    //It is only used to compute the yaw angle of the simulated odometry

    gazebo_state_sub_=rosnode_->subscribe("/gazebo/model_states",100,&OmniDrivePlugin::gazeboStateCallback,this);

    cmd_vel_subscriber_ = rosnode_->subscribe(so);

    odometry_publisher_ = rosnode_->advertise<nav_msgs::Odometry>(odometry_topic_, 1);

    // start custom queue for omnidrive
    this->callback_queue_thread_ =
      boost::thread(boost::bind(&OmniDrivePlugin::QueueThread, this));

    // listen to the update event (broadcast every simulation iteration)
    this->update_connection_ =
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&OmniDrivePlugin::UpdateChild, this));

  }

  void OmniDrivePlugin::Reset()
  {
    last_update_time_ = parent->GetWorld()->GetSimTime();
    pose_encoder_.x = 0;
    pose_encoder_.y = 0;
    pose_encoder_.theta = 0;
    v_ref_x_ = 0;
    v_ref_y_ = 0;
    w_ref_ = 0;    
    joints_[FRONT_LEFT_W]->SetMaxForce(0, wheel_torque_);
    joints_[FRONT_RIGHT_W]->SetMaxForce(0, wheel_torque_);
    joints_[BACK_LEFT_W]->SetMaxForce(0, wheel_torque_);
    joints_[BACK_RIGHT_W]->SetMaxForce(0, wheel_torque_);
    joints_[FRONT_LEFT_MW]->SetMaxForce(0, motor_wheel_torque_);
    joints_[FRONT_RIGHT_MW]->SetMaxForce(0, motor_wheel_torque_);
    joints_[BACK_LEFT_MW]->SetMaxForce(0, motor_wheel_torque_);
    joints_[BACK_RIGHT_MW]->SetMaxForce(0, motor_wheel_torque_);
  }

  // Update the controller
  void OmniDrivePlugin::UpdateChild()
  {
    for (int i = 0;i < 4; i++){
	if ( fabs(wheel_torque_ -joints_[i]->GetMaxForce ( 0 )) > 1e-6 ) 
	  joints_[i]->SetMaxForce ( 0, wheel_torque_ );
      }

    for (int i = 4;i < 8; i++){
	if ( fabs(motor_wheel_torque_ -joints_[i]->GetMaxForce ( 0 )) > 1e-6 ) 
	  joints_[i]->SetMaxForce ( 0, motor_wheel_torque_ );
      }
   
    UpdateOdometryEncoder();
        
    common::Time current_time = this->world->GetSimTime();
    double seconds_since_last_update =(current_time - last_update_time_).Double();
    
    if (seconds_since_last_update > update_period_) {

      publishOdometry(seconds_since_last_update);

      // Update robot in case new velocities have been requested
      getJointReferences();
      joints_[FRONT_LEFT_W]->SetVelocity(0, joint_reference_[FRONT_LEFT_W] / (wheel_diameter_ / 2.0));
      joints_[FRONT_RIGHT_W]->SetVelocity(0, joint_reference_[FRONT_RIGHT_W] / (wheel_diameter_ / 2.0));
      joints_[BACK_LEFT_W]->SetVelocity(0, joint_reference_[BACK_LEFT_W] / (wheel_diameter_ / 2.0));
      joints_[BACK_RIGHT_W]->SetVelocity(0, joint_reference_[BACK_RIGHT_W] / (wheel_diameter_ / 2.0));

      /*
      ROS_INFO("flw: %5.2f frw: %5.2f blw: %5.2f brw: %5.2f", 
                joint_reference_[FRONT_LEFT_W] / (wheel_diameter_ / 2.0),
                joint_reference_[FRONT_RIGHT_W] / (wheel_diameter_ / 2.0),
                joint_reference_[BACK_LEFT_W] / (wheel_diameter_ / 2.0),
                joint_reference_[BACK_RIGHT_W] / (wheel_diameter_ / 2.0)
                );
      */      
      joints_[FRONT_LEFT_MW]->SetAngle(0, joint_reference_[FRONT_LEFT_MW]);
      joints_[FRONT_RIGHT_MW]->SetAngle(0, joint_reference_[FRONT_RIGHT_MW]);
      joints_[BACK_LEFT_MW]->SetAngle(0, joint_reference_[BACK_LEFT_MW]);
      joints_[BACK_RIGHT_MW]->SetAngle(0, joint_reference_[BACK_RIGHT_MW]);
     
      last_update_time_+= common::Time(update_period_);
      }
  }

double OmniDrivePlugin::saturation(double u, double min, double max)
{
 if (u>max) u=max;
 if (u<min) u=min;
 return u; 
}

double OmniDrivePlugin::radnorm( double value ) 
{
  while (value > M_PI) value -= M_PI;
  while (value < -M_PI) value += M_PI;
  return value;
}

double OmniDrivePlugin::radnorm2( double value ) 
{
  while (value > 2.0*M_PI) value -= 2.0*M_PI;
  while (value < -2.0*M_PI) value += 2.0*M_PI;
  return value;
}

double OmniDrivePlugin::radnormHalf( double value)
{ //norms the angle so it is between -M_PI/2 and M_PI/2
double eps = 1e-5;
 while (value > 0.5*M_PI + eps) value -= M_PI;
 while (value < -0.5*M_PI - eps) value += M_PI;
 return value;
}

//norms a double value so if it is rounded to zero when it's below an epsylon
double OmniDrivePlugin::normToZero(double value)
{
    double eps = 1e-4;
    if (std::abs(value) < eps)
        return 0;
    return value;
}

//return the sign, as -1 or 1, of the value. 0 is positive
double OmniDrivePlugin::sign( double value)
{
    return (value < 0) ? -1 : 1;
}

//checks that v and w have the same sign. 0 is positive
bool OmniDrivePlugin::checkSign(double v, double w)
{
    return (v < 0) == (w < 0);
}

//wheels can operate forward or backward, so each pair of angle+velocity has it's mirrored pair that results in the same movement
//this function checks which one of both keeps the less change from the current configuration
void OmniDrivePlugin::normJointReference(double &wheel_speed, double &wheel_angle, double current_wheel_speed, double current_wheel_angle)
{
    double mirrored_wheel_speed = -wheel_speed;
    double mirrored_wheel_angle = (wheel_angle > 0) ? wheel_angle - M_PI : wheel_angle + M_PI;

    double change = std::abs(wheel_speed-current_wheel_speed)*std::abs(wheel_angle-current_wheel_angle);
    double mirrored_change = std::abs(mirrored_wheel_speed-current_wheel_speed)*std::abs(mirrored_wheel_angle-current_wheel_angle);

    if (mirrored_change < change)
    {
        wheel_speed = mirrored_wheel_speed;
        wheel_angle = mirrored_wheel_angle;
    }
}

//as the motorwheels can only rotate between their lower and upper limits, this function checks that the reference is between that limit
//if it isn't, sets the mirrored pair
void OmniDrivePlugin::setJointReferenceBetweenMotorWheelLimits(double &wheel_speed, double &wheel_angle, int joint_number)
{
    double lower_limit = joints_[joint_number]->GetLowerLimit(0).Radian();
    double upper_limit = joints_[joint_number]->GetUpperLimit(0).Radian();

    // if angle is between limits, do nothing
    if (lower_limit <= wheel_angle && wheel_angle <= upper_limit)    
        return;

    // if angle is below the lower_limit, add pi and change speed sign
    if (wheel_angle < lower_limit) {
        wheel_angle += M_PI;
        wheel_speed = -wheel_speed;
        return;
        ROS_INFO_THROTTLE(1, "angle below limit");
    }
    // if angle is above the upper_limit, substract pi and change speed sign
    if (upper_limit < wheel_angle) {
        wheel_angle -= M_PI;
        wheel_speed = -wheel_speed;
        ROS_INFO_THROTTLE(1, "angle above limit");
        return;
    }
}

void OmniDrivePlugin::getJointReferences()
  {
    boost::mutex::scoped_lock scoped_lock(lock);

	  // Speed references for motor control	  
	  double vx = v_ref_x_;
	  double vy = v_ref_y_;
	  double w = w_ref_;
	  double L = wheel_base_;   
	  double W = track_width_;
	  

	  // joint references are calculated so they are constrained in the following order:
	  // - (1) motorwheel angle is between its rotation limits.
	  // - (2) keep the less change between the reference and the current joint state, by setting 
	  //   the calculated reference or it's mirrored reference ( byt adding/substractin +/-M_PI
	  //   to the motor_wheel orientation and switching the sign of the wheel speed.
	  // - (3) keep the orientation between -M_PI/2 and +M_PI/2, so the encoders are below the base.
	  // in the code, the constraints are checked in reversed order, so the more constraining (1) is checked at the end

	  double x1 = L/2.0; double y1 = W/2.0;
	  double wx1 = vx + w * y1;
	  double wy1 = vy + w * x1;
	  double q1 = - sign(wx1) * sqrt( wx1*wx1 + wy1*wy1 );  
	  //double a1 = radnorm( atan2( wy1, wx1 ) );
	  double a1 = radnormHalf( atan2( wy1,wx1 )); // contraint (3)
	  double x2 = L/2.0; double y2 = W/2.0;
	  double wx2 = vx - w * y2;
	  double wy2 = vy + w * x2;
	  double q2 = sign(wx2) * sqrt( wx2*wx2 + wy2*wy2 );
	  //double a2 = radnorm( atan2( wy2, wx2 ) );
	  double a2 = radnormHalf( atan2 (wy2,wx2)); // contraint (3)
	  double x3 = L/2.0; double y3 = W/2.0;
	  double wx3 = vx - w * y3;
	  double wy3 = vy - w * x3;
	  double q3 = sign(wx3)*sqrt( wx3*wx3 + wy3*wy3 );
	  //double a3 = radnorm( atan2( wy3, wx3 ) );
	  double a3 = radnormHalf( atan2(wy3 , wx3)); // contraint (3)
	  double x4 = L/2.0; double y4 = W/2.0;
	  double wx4 = vx + w * y4;
	  double wy4 = vy - w * x4;
	  double q4 = -sign(wx4)*sqrt( wx4*wx4 + wy4*wy4 );
	  //double a4 = radnorm( atan2( wy4, wx4 ) );
	  double a4 = radnormHalf( atan2(wy4,wx4)); // contraint (3)
	  


      //constraint (2)
      normJointReference(q1,a1,joint_state_mean_[FRONT_RIGHT_W], joint_reference_[FRONT_RIGHT_MW]);
      normJointReference(q2,a2,joint_state_mean_[FRONT_LEFT_W],  joint_reference_[FRONT_LEFT_MW]);
      normJointReference(q3,a3,joint_state_mean_[BACK_LEFT_W],  joint_reference_[BACK_LEFT_MW]);
      normJointReference(q4,a4,joint_state_mean_[BACK_RIGHT_W],   joint_reference_[BACK_RIGHT_MW]);

      //constraint (1)
      setJointReferenceBetweenMotorWheelLimits(q1, a1, FRONT_RIGHT_MW);
      setJointReferenceBetweenMotorWheelLimits(q2, a2, FRONT_LEFT_MW);
      setJointReferenceBetweenMotorWheelLimits(q3, a3, BACK_LEFT_MW);
      setJointReferenceBetweenMotorWheelLimits(q4, a4, BACK_RIGHT_MW);

	  //ROS_INFO("wy1 wx1 %5.2f %5.2f", wy1, wx1);
      //ROS_INFO_THROTTLE(1,"q1234=(%5.2f, %5.2f, %5.2f, %5.2f)   a1234=(%5.2f, %5.2f, %5.2f, %5.2f)", q1,q2,q3,q4, a1,a2,a3,a4);
	
	  // Motor control actions	  
	  // Axis are not reversed in the omni (swerve) configuration
      joint_reference_[FRONT_RIGHT_W] = q1;  
      joint_reference_[FRONT_LEFT_W] = q2;
	  joint_reference_[BACK_LEFT_W] = q3;
	  joint_reference_[BACK_RIGHT_W] = q4;

      joint_reference_[FRONT_RIGHT_MW] = a1;  
      joint_reference_[FRONT_LEFT_MW] = a2;
	  joint_reference_[BACK_LEFT_MW] = a3;
	  joint_reference_[BACK_RIGHT_MW] = a4;
  }


  // Finalize the controller
  void OmniDrivePlugin::FiniChild()
  {
    alive_ = false;
    queue_.clear();
    queue_.disable();
    rosnode_->shutdown();
    callback_queue_thread_.join();
  }
  
  void OmniDrivePlugin::gazeboStateCallback(const gazebo_msgs::ModelStatesConstPtr& msg)
  {
    boost::mutex::scoped_lock scoped_lock(lock);   
    omega_=msg->twist[1].angular.z;
  }
  
  void OmniDrivePlugin::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg)
  {
    boost::mutex::scoped_lock scoped_lock(lock);   
    v_ref_x_ = cmd_msg->linear.x;
    v_ref_y_ = cmd_msg->linear.y;
    w_ref_ = cmd_msg->angular.z;   // is in rads ?
    // ROS_INFO("v_ref_x_ = %5.2f", v_ref_x_);    
  }

  void OmniDrivePlugin::QueueThread() {
    static const double timeout = 0.01;
    
    while (alive_ && rosnode_->ok()) {
      queue_.callAvailable(ros::WallDuration(timeout));
    }
  }


  void OmniDrivePlugin::UpdateJointStateHistoryMean()
  {
    for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
        double sum = std::accumulate(joint_state_history_[i].begin(), joint_state_history_[i].end(), 0.0);
        joint_state_mean_[i] = sum / joint_state_history_[i].size();
    }

    ROS_INFO_THROTTLE(1, "joint_state_mean: %2.5f %2.5f %2.5f %2.5f %2.5f %2.5f %2.5f %2.5f", joint_state_mean_[0],joint_state_mean_[1],joint_state_mean_[2],joint_state_mean_[3],joint_state_mean_[4],joint_state_mean_[5],joint_state_mean_[6],joint_state_mean_[7]);
  }

  void OmniDrivePlugin::UpdateOdometryEncoder()
  {
	// Linear speed of each wheel
	double v1, v2, v3, v4; 
	v1 = normToZero(joints_[FRONT_RIGHT_W]->GetVelocity(0)  * (wheel_diameter_ / 2.0));
	v2 = normToZero(joints_[FRONT_LEFT_W]->GetVelocity(0) * (wheel_diameter_ / 2.0));
	v3 = normToZero(joints_[BACK_LEFT_W]->GetVelocity(0) * (wheel_diameter_ / 2.0));
	v4 = normToZero(joints_[BACK_RIGHT_W]->GetVelocity(0) * (wheel_diameter_ / 2.0));
	// Angular pos of each wheel
    double a1, a2, a3, a4; 
    double aa2;
    a1 = normToZero( joints_[FRONT_RIGHT_MW]->GetAngle(0).Radian() );  
    a2 = normToZero( joints_[FRONT_LEFT_MW]->GetAngle(0).Radian() );
    a3 = normToZero( joints_[BACK_LEFT_MW]->GetAngle(0).Radian() );
    a4 = normToZero( joints_[BACK_RIGHT_MW]->GetAngle(0).Radian() );


	// keep a history of size N of the joint_states
    joint_state_history_[FRONT_RIGHT_W].push_back(v1);
    joint_state_history_[FRONT_LEFT_W].push_back(v2);
    joint_state_history_[BACK_LEFT_W].push_back(v3);
    joint_state_history_[BACK_RIGHT_W].push_back(v4);

    joint_state_history_[FRONT_RIGHT_MW].push_back(radnorm(a1));
    joint_state_history_[FRONT_LEFT_MW].push_back( radnorm(a2));
    joint_state_history_[BACK_LEFT_MW].push_back( radnorm(a3));
    joint_state_history_[BACK_RIGHT_MW].push_back(  radnorm(a4));

	// calculate the mean of the history of each joint
    UpdateJointStateHistoryMean();
//    double v1x = -spin_ * v1 * cos( a1 ); double v1y = -spin_ * v1 * sin( a1 );  // spin for mirrored axes    
//    double v2x = -v2 * cos( a2 ); double v2y = -v2 * sin( a2 );
//    double v3x = -v3 * cos( a3 ); double v3y = -v3 * sin( a3 );
//    double v4x = -spin_ * v4 * cos( a4 ); double v4y = -spin_ * v4 * sin( a4 );
    double v1x = -v1 * cos( a1 ); double v1y = -v1 * sin( a1 ); 
    double v2x = v2 * cos( a2 ); double v2y = v2 * sin( a2 );
    double v3x = v3 * cos( a3 ); double v3y = v3 * sin( a3 );
    double v4x = -v4 * cos( a4 ); double v4y = -v4 * sin( a4 );
    
    double C = (v4y + v1y) / 2.0;
    double B = (v2x + v1x) / 2.0;
    double D = (v2y + v3y) / 2.0;
    double A = (v3x + v4x) / 2.0;
    double E = (v1y + v2y) / 2.0;
    double F = (v3y + v4y) / 2.0;
    double G = (v1x + v4x) / 2.0;
    double H = (v2x + v3x) / 2.0;
    double w = ((E-F)/wheel_base_ + (G-H)/track_width_) / 2.0;
    double vx = (A+B) / 2.0;
    double vy = (C+D) / 2.0; 

	// Get real freq.
    common::Time current_time = parent->GetWorld()->GetSimTime();
    double seconds_since_last_update = (current_time - last_odom_update_ ).Double();
    last_odom_update_ = current_time;

    // Compute Position
    double fSamplePeriod = seconds_since_last_update;
    pose_encoder_.x += cos(pose_encoder_.theta) * vx * fSamplePeriod + cos(M_PI_2 + pose_encoder_.theta) * vy * fSamplePeriod;
    pose_encoder_.y += sin(pose_encoder_.theta) * vx * fSamplePeriod + sin(M_PI_2 + pose_encoder_.theta) * vy * fSamplePeriod;
	pose_encoder_.theta += w * fSamplePeriod;  
    // ROS_INFO("Odom estimated x=%5.2f  y=%5.2f a=%5.2f", robot_pose_px_, robot_pose_py_, robot_pose_pa_);

    tf::Quaternion qt;
    tf::Vector3 vt;
    qt.setRPY(0,0,pose_encoder_.theta);
    vt = tf::Vector3 (pose_encoder_.x, pose_encoder_.y, 0);

    odom_.pose.pose.position.x = vt.x();
    odom_.pose.pose.position.y = vt.y();
    odom_.pose.pose.position.z = vt.z();

    odom_.pose.pose.orientation.x = qt.x();
    odom_.pose.pose.orientation.y = qt.y();
    odom_.pose.pose.orientation.z = qt.z();
    odom_.pose.pose.orientation.w = qt.w();

    odom_.twist.twist.angular.z = w ;
    odom_.twist.twist.linear.x = vx;
    odom_.twist.twist.linear.y = vy;    
  }
  
  void OmniDrivePlugin::publishOdometry(double step_time)
  {
    ros::Time current_time = ros::Time::now();
    std::string odom_frame = tf::resolve(tf_prefix_, odometry_frame_);
    std::string base_footprint_frame = tf::resolve(tf_prefix_, robot_base_frame_);

    tf::Quaternion qt;
    tf::Vector3 vt;

    if(odom_type_==ENCODER){
      // getting data from encoder integration
      qt = tf::Quaternion (odom_.pose.pose.orientation.x, odom_.pose.pose.orientation.y, odom_.pose.pose.orientation.z, odom_.pose.pose.orientation.w );
      vt = tf::Vector3 ( odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z );
    }
    else{
      // getting data form gazebo world
      math::Pose pose = parent->GetWorldPose();
      qt = tf::Quaternion ( pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w );
      vt = tf::Vector3 ( pose.pos.x, pose.pos.y, pose.pos.z );
      
      odom_.pose.pose.position.x = vt.x();
      odom_.pose.pose.position.y = vt.y();
      odom_.pose.pose.position.z = vt.z();
      
      odom_.pose.pose.orientation.x = qt.x();
      odom_.pose.pose.orientation.y = qt.y();
      odom_.pose.pose.orientation.z = qt.z();
      odom_.pose.pose.orientation.w = qt.w();

      // get velocity in /odom frame
      math::Vector3 linear;
      linear = parent->GetWorldLinearVel();
      odom_.twist.twist.angular.z = parent->GetWorldAngularVel().z;

      // convert velocity to child_frame_id (aka base_footprint)
      float yaw = pose.rot.GetYaw();
      odom_.twist.twist.linear.x = cosf ( yaw ) * linear.x + sinf ( yaw ) * linear.y;
      odom_.twist.twist.linear.y = cosf ( yaw ) * linear.y - sinf ( yaw ) * linear.x;
    }

    tf::Transform base_footprint_to_odom(qt, vt);
    if (this->broadcast_tf_) {
    	transform_broadcaster_->sendTransform(
        tf::StampedTransform(base_footprint_to_odom, current_time,
            odom_frame, base_footprint_frame));
    }


    // Scaling the covariance values as the robot travels: this should help EKF preferring imu_data to odom data when the robot has travelled long distances.
    // TO BE TESTED (increase or decrease the covariance?!)
    double travelled_distance = sqrt(odom_.pose.pose.position.x*odom_.pose.pose.position.x+odom_.pose.pose.position.y*odom_.pose.pose.position.y);
    double scaling=0;//.000005;
    
     // set covariance
    odom_.pose.covariance[0] = 0.00001+scaling*travelled_distance;
    odom_.pose.covariance[7] = 0.00001+scaling*travelled_distance;
    odom_.pose.covariance[14] = 1000000000000.0;
    odom_.pose.covariance[21] = 1000000000000.0;
    odom_.pose.covariance[28] = 1000000000000.0;
    odom_.pose.covariance[35] = 0.001+scaling*travelled_distance;

    // set header
    odom_.header.stamp = current_time;
    odom_.header.frame_id = odom_frame;
    odom_.child_frame_id = base_footprint_frame;  
 
    odometry_publisher_.publish(odom_);
  }

  GZ_REGISTER_MODEL_PLUGIN(OmniDrivePlugin)
}
