#ifndef OMNIDRIVE_PLUGIN_H_
#define OMNIDRIVE_PLUGIN_H_

#include <map>
// #include </pf_vector.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_msgs/ModelStates.h>

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/circular_buffer.hpp>

// Odometry options
#define WORLD       0
#define ENCODER     1

#define NUMBER_OF_JOINTS 8

namespace gazebo {

  class Joint;
  class Entity;

  class OmniDrivePlugin : public ModelPlugin {

  public:
      OmniDrivePlugin();
      ~OmniDrivePlugin();
      void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

  protected:
      virtual void UpdateChild();
      virtual void FiniChild();

    private:
      void publishOdometry(double step_time);

      // Draw randomly from a zero-mean Gaussian distribution, with standard deviation sigma (polar form of Box-Muller transformation)
      // double pf_ran_gaussian(double sigma);

      // TODO: Populate the covariance part of the odometry message (cfr PR2 odometry)
      // void populateCovariance(const double &residual, nav_msgs::Odometry &msg);
      double saturation(double u, double min, double max);
      double radnorm( double value );
      double radnorm2( double value ); 
      double radnormHalf( double value ); 
      double normToZero(double value);
      double sign( double value ); 
      bool checkSign(double v, double w);

      void normJointReference(double &q, double &a, double cq, double ca);
      void setJointReferenceBetweenMotorWheelLimits(double &wheel_speed, double &wheel_angle, int joint_number);

      void getJointReferences();
      
      void Reset();

      void UpdateOdometryEncoder();
      unsigned int odom_type_;
      //bool first_loop_;
      
      common::Time last_odom_update_;
      geometry_msgs::Pose2D pose_encoder_;
    
      physics::WorldPtr world;
      physics::ModelPtr parent;
      event::ConnectionPtr update_connection_;

      std::string joint_front_right_wheel_name_;      
      std::string joint_front_left_wheel_name_;
      std::string joint_back_right_wheel_name_;
      std::string joint_back_left_wheel_name_;
      std::string joint_front_right_motor_wheel_name_;      
      std::string joint_front_left_motor_wheel_name_;
      std::string joint_back_right_motor_wheel_name_;
      std::string joint_back_left_motor_wheel_name_;

      double wheel_base_;
      double track_width_;
      double wheel_diameter_;
      double wheel_torque_;
      
      double motor_wheel_torque_;
      double joint_reference_[NUMBER_OF_JOINTS];

	  std::vector<double> joint_state_mean_;
	  void UpdateJointStateHistoryMean();
      unsigned int joint_state_history_size_;
      std::vector<boost::circular_buffer<double> > joint_state_history_;

      physics::JointPtr joints_[NUMBER_OF_JOINTS];

      // ROS STUFF
      ros::NodeHandle* rosnode_;
      ros::Publisher odometry_publisher_;
      ros::Subscriber cmd_vel_subscriber_;

      ros::Subscriber gazebo_state_sub_;
      
      tf::TransformBroadcaster *transform_broadcaster_;
      nav_msgs::Odometry odom_;
      std::string tf_prefix_;
      bool broadcast_tf_;

      boost::mutex lock;

      std::string robot_namespace_;
      std::string command_topic_;
      std::string odometry_topic_;
      std::string odometry_frame_;
      std::string robot_base_frame_;

      // Custom Callback Queue
      ros::CallbackQueue queue_;
      boost::thread callback_queue_thread_;
      void QueueThread();

      // Command callback
      void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);

      void gazeboStateCallback(const gazebo_msgs::ModelStates::ConstPtr& msg);

      // internal gazebo w ref
      double omega_;

	  // Linear and angular reference speeds 
      double v_ref_x_;
      double v_ref_y_;
      double w_ref_;
      bool alive_;

      // Update Rate
      double update_rate_;
      double update_period_;
      common::Time last_update_time_;
      
      // Mirrored axes
      bool mirrored_axes_;
      double spin_;
      

  };

}

#endif /* SKID_STEERING_PLUGIN_H_ */
