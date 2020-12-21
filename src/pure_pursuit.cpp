/* A ROS implementation of the Pure pursuit path tracking algorithm (Coulter 1992).

   Terminology (mostly :) follows:
   Coulter, Implementation of the pure pursuit algoritm, 1992 and 
   Sorniotti et al. Path tracking for Automated Driving, 2017.
 */

#include <string>
#include <cmath>
#include <algorithm>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <kdl/frames.hpp>
#include <hunter_msgs/HunterStatus.h>
// TODO: Figure out how to use tf2 DataConversions
// for more elegant and compact code
//#include <tf2_kdl/tf2_kdl.h>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <dynamic_reconfigure/server.h>
#include <std_msgs/Float32MultiArray.h>
using std::string;

class PurePursuit{
public:
  //! Constructor
  PurePursuit();
  //! Compute velocit commands each time new odometry data is received.
  void computeVelocities(nav_msgs::Odometry odom);
  //! Receive path to follow.
  void receivePath(nav_msgs::Path path);
  void stdhunter(hunter_msgs::HunterStatus status);
  void computeCmd(std_msgs::Float32MultiArray xyobsc);
  //! Compute transform that transforms a pose into the robot frame (base_link)
  KDL::Frame transformToBaseLink(const geometry_msgs::Pose& pose, const geometry_msgs::Transform& tf);
  double leff(double f)
  { 
    double t1 = f - (int)f;
    t1 = t1 * 10;//小数点左移。
    if(t1 - (int)t1 >= 0.5) t1 += 1; //四舍五入
    t1 = (int)t1; //取整数部分。
    t1 = t1 / 10;//小数点右移。
    t1+=(int)f;//加上原本的整数部分
    return t1;
  } 
  //! Helper founction for computing eucledian distances in the x-y plane.
  template<typename T1, typename T2>
  double distance(T1 pt1, T2 pt2){
    return sqrt(pow(pt1.x - pt2.x, 2) + pow(pt1.y - pt2.y, 2) + pow(pt1.z - pt2.z, 2));
  }
  //! Run the controller.
  void run();

private:
  //! Dynamic reconfigure callback.
  //void reconfigure(pure_pursuit::PurePursuitConfig &config, uint32_t level);
  // Vehicle parameters
  double L_;//轴距
  // Algorithm variables
  // Position tolerace is measured along the x-axis of the robot!
  double ld_, pos_tol_;//预瞄距离 位姿准确阈值
  // Generic control variables
  double v_max_, v_, w_max_;
  // Control variables for Ackermann steering
  // Steering angle is denoted by delta
  double delta_, delta_vel_, acc_, jerk_, delta_max_;
  double x, y, width;
  nav_msgs::Path path_;
  hunter_msgs::HunterStatus status_;
  unsigned idx_;
  bool goal_reached_;
  bool istop;
  geometry_msgs::Twist cmd_vel_;
  ackermann_msgs::AckermannDriveStamped cmd_acker_;
  // Ros infrastructure
  ros::NodeHandle nh_, nh_private_;
  ros::Subscriber sub_odom_, sub_path_, sub_hunter_, sub_obsc_;
  ros::Publisher pub_vel_, pub_acker_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  //geometry_msgs::TransformStamped lookahead_;
  string map_frame_id_, robot_frame_id_, lookahead_frame_id_, acker_frame_id_, path_frame_id_;
};

PurePursuit::PurePursuit() : ld_(1.5), v_max_(0.1), v_(v_max_), w_max_(1.0), pos_tol_(0.1), idx_(0), goal_reached_(true), istop(false), nh_private_("~"), tf_listener_(tf_buffer_),
                             map_frame_id_("map"), robot_frame_id_("velodyne"),lookahead_frame_id_("lookahead"){                                      //初始化
  // Get parameters from the parameter server
  nh_private_.param<double>("wheelbase", L_, 0.578);
  nh_private_.param<double>("lookahead_distance", ld_, 1.5);//预瞄距离
  //nh_private_.param<double>("linear_velocity", v_, 0.1);
  nh_private_.param<double>("max_rotational_velocity", w_max_, 1.0);
  nh_private_.param<double>("position_tolerance", pos_tol_, 0.1);
  nh_private_.param<double>("steering_angle_velocity", delta_vel_, 100.0);
  nh_private_.param<double>("acceleration", acc_, 100.0);
  nh_private_.param<double>("jerk", jerk_, 100.0);//制动
  nh_private_.param<double>("steering_angle_limit", delta_max_, 0.523);
  nh_private_.param<string>("map_frame_id", map_frame_id_, "map");
  // Frame attached to midpoint of rear axle (for front-steered vehicles).
  nh_private_.param<string>("robot_frame_id", robot_frame_id_, "velodyne");
  // Lookahead frame moving along the path as the vehicle is moving.
  nh_private_.param<string>("lookahead_frame_id", lookahead_frame_id_, "lookahead");
  // Frame attached to midpoint of front axle (for front-steered vehicles).
  nh_private_.param<string>("ackermann_frame_id", acker_frame_id_, "rear_axle_midpoint");

  // Populate messages with static data
  //lookahead_.header.frame_id = robot_frame_id_;
  //lookahead_.child_frame_id = lookahead_frame_id_;

  cmd_acker_.header.frame_id = acker_frame_id_;
  cmd_acker_.drive.steering_angle_velocity = delta_vel_;
  cmd_acker_.drive.acceleration = acc_;
  cmd_acker_.drive.jerk = jerk_;

  sub_hunter_ = nh_.subscribe("/hunter_status", 10, &PurePursuit::stdhunter, this);
  sub_path_ = nh_.subscribe("/trajectory", 10, &PurePursuit::receivePath, this);
  sub_obsc_ = nh_.subscribe("/xychatter", 10, &PurePursuit::computeCmd, this);
  sub_odom_ = nh_.subscribe("/odom_xyth", 10, &PurePursuit::computeVelocities, this);

  
  pub_vel_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  //pub_acker_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("cmd_acker", 10);

  // reconfigure_callback_ = boost::bind(&PurePursuit::reconfigure, this, _1, _2);
  // reconfigure_server_.setCallback(reconfigure_callback_);
}
void PurePursuit::computeCmd(std_msgs::Float32MultiArray xyobsc){
  x = xyobsc.data.at(0);
  y = xyobsc.data.at(1);
  width = xyobsc.data.at(2);
  std::cout<<"obsc     x: "<<x<<", y: "<<y<<", width: "<<width<<std::endl;
  std::cout<<"y - w/2"<<(fabs(y) - width / 2)<<std::endl;
  if(x != 0 && y != 0 && width != 0){
    if(x < 1.5 && (fabs(y) - width / 2) < 0.4){
      std::cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<(fabs(y) - width / 2)<<std::endl;
      istop = true;
    }else{
      istop = false;
      std::cout<<"false"<<std::endl;
    }
  }
}
// void PurePursuit::computeVelocities(nav_msgs::Odometry odom){
void PurePursuit::computeVelocities(nav_msgs::Odometry odom){
  // The velocity commands are computed, each time a new Odometry message is received.
  // Odometry is not used directly, but through the tf tree.
  // Get the current robot pose
  //std::cout<<"has recevied odom"<<std::endl;
  geometry_msgs::TransformStamped tf;
  double yt1;
  try{
    tf = tf_buffer_.lookupTransform(map_frame_id_, robot_frame_id_, ros::Time(0));//从地图坐标系到base_link
    // We first compute the new point to track, based on our current pose,
    // path information and lookahead distance. 
    for (; idx_ < path_.poses.size(); idx_++){          //依次读取路径点 根据预瞄距离读取 达不到预瞄距离就break

      if (idx_ >= (path_.poses.size() - 1)){
        idx_ = 0;
      }
      //std::cout<< idx_ <<"!"<<std::endl;
      //if (distance(path_.poses[idx_].pose.position, tf.transform.translation) > ld_){ //ld = 1m 预瞄距离 大于预瞄距离的点作为路径点
      if (distance(path_.poses[idx_].pose.position, tf.transform.translation) > 1)
        // Transformed lookahead to base_link frame is lateral error
        KDL::Frame F_bl_ld = transformToBaseLink(path_.poses[idx_].pose, tf.transform);
        double yt1 = F_bl_ld.p.y();
        /*static tf::TransformBroadcaster br;gfggf
        tf::Transform transform;
        path_frame_id_ = path_.poses[idx_].header.frame_id;
        br.sendTransform(t uf::StampedTransform(transform, ros::Time(0), path_frame_id_, robot_frame_id_)); 
        tf1 = tf_buffer_.lookupTransform(path_frame_id_, robot_frame_id_, ros::Time(0));
        std::cout<<"tf"<<tf1<<"tf1"<<std::endl;*/
        //lookahead_.transform.translation.x = F_bl_ld.p.x(); lookahead_.transform.translation.y = F_bl_ld.p.y(); lookahead_.transform.translation.z = F_bl_ld.p.z(); //Vector
        //F_bl_ld.M.GetQuaternion(lookahead_.transform.rotation.x, lookahead_.transform.rotation.y, lookahead_.transform.rotation.z, lookahead_.transform.rotation.w); //Rotation
        // F_bl_ld.M.Quaternion(lookahead_.transform.rotation.x, lookahead_.transform.rotation.y, lookahead_.transform.rotation.z, lookahead_.transform.rotation.w); //Rotation
        // TODO: See how the above conversion can be done more elegantly
        // using tf2_kdl and tf2_geometry_msgs
        //std::cout<<lookahead_.transform<<std::endl;
          int dp[11];
          for(int i = 0; i <= 10; i++){
            //std::cout<<i<<std::endl;
            /*if(i <= 5){
              dp[i] = idx_ - i * 3;
              if(dp[i] < 0){
                dp[i] + path_.poses.size();
              }
            }else{
              dp[i] = idx_ + i * 3 - 15;
              if(dp[i] > path_.poses.size()){
                dp[i] - path_.poses.size();
              }
            }*/
            dp[i] = idx_ + i * 2;
            if(dp[i] > path_.poses.size()){
              dp[i] - path_.poses.size();
            }
          }
          double diffX = path_.poses[dp[0]].pose.position.x + path_.poses[dp[4]].pose.position.x + path_.poses[dp[3]].pose.position.x + path_.poses[dp[2]].pose.position.x + path_.poses[dp[1]].pose.position.x - 10 * path_.poses[dp[5]].pose.position.x
                  + path_.poses[dp[6]].pose.position.x + path_.poses[dp[7]].pose.position.x + path_.poses[dp[8]].pose.position.x + path_.poses[dp[9]].pose.position.x + path_.poses[dp[10]].pose.position.x;
          double diffY = path_.poses[dp[0]].pose.position.y + path_.poses[dp[4]].pose.position.y + path_.poses[dp[3]].pose.position.y + path_.poses[dp[2]].pose.position.y + path_.poses[dp[1]].pose.position.y - 10 * path_.poses[dp[5]].pose.position.y
                  + path_.poses[dp[6]].pose.position.y + path_.poses[dp[7]].pose.position.y + path_.poses[dp[8]].pose.position.y + path_.poses[dp[9]].pose.position.y + path_.poses[dp[10]].pose.position.y;
          double curv = sqrt(diffX * diffX + diffY * diffY);
          curv = curv * 1.5;
          //std::cout<<curv<<std::endl;

          if(curv < 1){
            curv = 1;
            ld_ = 1.3;
          }
          if (1 <= curv <=1.8){
            double p = 1 / curv;
            double p1 = leff(p);
            curv = p1;
            ld_ = p1 * 1.3;
          }
          if (curv > 1.8){
            ld_ = 0.6;
            curv = 1.8;
          }

          //std::cout<<curv<<"||"<<ld_<<std::endl;
        //}
        ///////if   前视100个点曲率有剧烈变化则不要ld变换太大 //基于一定预瞄距离的点：比如前20-40个点 不是前20个点
        if (idx_ < path_.poses.size()){//未到终点
          // We are tracking.
          // Compute linear velocity.
          // Right now,this is not very smart :)
          //v_ = copysign(v_max_, v_);
          v_ = 0.8;   //定速
          // Compute the angular velocity.
          // Lateral error is the y-value of the lookahead point (in base_link frame)
          //double yt = lookahead_.transform.translation.y;
          double yt = yt1;
          double ld_2 = ld_ * ld_;
          //cmd_vel_.angular.z = std::min( 2*v_ / ld_2 * yt, w_max_ );// v^2 / 预瞄距离 * y

          // Compute desired Ackermann steering angle
          //cmd_acker_.drive.steering_angle = std::min( atan2(2 * yt * L_, ld_2), delta_max_ );// δ = arctan（ 2 * el * L / ld²） 的角度 
          
          // Set linear velocity for tracking.
          cmd_vel_.linear.x = v_;
          cmd_vel_.angular.z = std::min( atan2(2 * yt * L_, ld_2), delta_max_ );
          
          if(istop){
            if(y < 0){
              yt1 = y - width / 2 - 0.2;
            }else{
              yt1 = y + width / 2 + 0.2;
            }
            cmd_vel_.linear.x = v_ - 0.2;
            cmd_vel_.angular.z = std::min( atan2(2 * yt * L_, 0.25), delta_max_ );
          }
          //std::cout<<tf.transform.translation.x<<","<<tf.transform.translation.y<<","<<status_.steering_angle<<","<<status_.linear_velocity<<","<<cmd_acker_.drive.steering_angle<<","<<cmd_vel_.linear.x<<std::endl;
          //cmd_acker_.header.stamp = ros::Time::now();
          //cmd_acker_.drive.speed = v_;
        }else{
          // We are at the goal!
          // Stop the vehicle
          // The lookahead target is at our current pose.
          //lookahead_.transform = geometry_msgs::Transform();
          //lookahead_.transform.rotation.w = 1.0;
          // Stop moving.
          cmd_vel_.linear.x = 0.0;
          cmd_vel_.angular.z = 0.0;

          //cmd_acker_.header.stamp = ros::Time::now();
          //cmd_acker_.drive.steering_angle = 0.0;
          //cmd_acker_.drive.speed = 0.0;
        }
        break;
      }
    }

    /*if (!path_.poses.empty() && idx_ >= path_.poses.size()){
      // We are approaching the goal,
      // which is closer than ld
      // This is the pose of the goal w.r.t. the base_link frame
      std::cout<<"back!"<<std::endl;
      std::cout<<"back!"<<std::endl;
      std::cout<<"back!"<<std::endl;
      std::cout<<"back!"<<std::endl;
      std::cout<<"back!"<<std::endl;
      KDL::Frame F_bl_end = transformToBaseLink(path_.poses.back().pose, tf.transform);// M - Rotation   p - Vector 路径点最后一点和tf变换的变换
      std::cout<<path_.poses.back().pose.position.x<<"back"<<path_.poses.back()<<std::endl;
      if (fabs(F_bl_end.p.x()) <= pos_tol_){
        // We have reached the goal
        goal_reached_ = true;
        // Reset the path
        path_ = nav_msgs::Path();
      }
      else{
        // We need to extend the lookahead distance
        // beyond the goal point.
        // Find the intersection between the circle of radius ld
        // centered at the robot (origin)
        // and the line defined by the last path pose
        double roll, pitch, yaw;
        F_bl_end.M.GetRPY(roll, pitch, yaw);
        double k_end = tan(yaw); // Slope of line defined by the last path pose
        double l_end = F_bl_end.p.y() - k_end * F_bl_end.p.x();
        double a = 1 + k_end * k_end;
        double b = 2 * l_end;
        double c = l_end * l_end - ld_ * ld_;
        double D = sqrt(b*b - 4*a*c);
        double x_ld = (-b + copysign(D,v_)) / (2*a);
        double y_ld = k_end * x_ld + l_end;
        
        //lookahead_.transform.translation.x = x_ld; lookahead_.transform.translation.y = y_ld; lookahead_.transform.translation.z = F_bl_end.p.z();
        //F_bl_end.M.GetQuaternion(lookahead_.transform.rotation.x, lookahead_.transform.rotation.y,lookahead_.transform.rotation.z, lookahead_.transform.rotation.w);
      }
    }*/
    // Publish the lookahead target transform.
    //lookahead_.header.stamp = ros::Time::now();
    //tf_broadcaster_.sendTransform(lookahead_);
    // Publish the velocities
    pub_vel_.publish(cmd_vel_);
    // Publish ackerman steering setpoints
    pub_acker_.publish(cmd_acker_);
  }
    
  catch (tf2::TransformException &ex){
    ROS_WARN_STREAM(ex.what());
  }
}

void PurePursuit::receivePath(nav_msgs::Path new_path){
  // When a new path received, the previous one is simply discarded
  // It is up to the planner/motion manager to make sure that the new
  // path is feasible.
  // Callbacks are non-interruptible, so this will
  // not interfere with velocity computation callback.
  //std::cout<<new_path.header.frame_id<<std::endl;
  if (new_path.header.frame_id == map_frame_id_){
    path_ = new_path;
    idx_ = 0;
    if (new_path.poses.size() > 0){
      ROS_WARN_STREAM("Received path!");
      goal_reached_ = false;
    }else{
      goal_reached_ = true;
      ROS_WARN_STREAM("Received empty path!");
    }
  }else{
    ROS_WARN_STREAM("The path must be published in the " << map_frame_id_  << " frame! Ignoring path in " << new_path.header.frame_id  << " frame!");
  }
}
void PurePursuit::stdhunter(hunter_msgs::HunterStatus new_status){
  status_ = new_status;
}

KDL::Frame PurePursuit::transformToBaseLink(const geometry_msgs::Pose& pose,const geometry_msgs::Transform& tf){
  // Pose in global (map) frame
  KDL::Frame F_map_pose(KDL::Rotation::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
                        KDL::Vector(pose.position.x, pose.position.y, pose.position.z));
  // Robot (base_link) in global (map) frame
  KDL::Frame F_map_tf(KDL::Rotation::Quaternion(tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w),
                      KDL::Vector(tf.translation.x, tf.translation.y, tf.translation.z));
  // TODO: See how the above conversions can be done more elegantly
  // using tf2_kdl and tf2_geometry_msgs
  return F_map_tf.Inverse()*F_map_pose;
}

void PurePursuit::run(){
  ros::spin();
}

//void PurePursuit::reconfigure(pure_pursuit::PurePursuitConfig &config, uint32_t level){
//  v_max_ = config.max_linear_velocity;
//}

int main(int argc, char**argv){
  ros::init(argc, argv, "pure_pursuit");
  
  PurePursuit controller;
  controller.run();

  return 0;
}
