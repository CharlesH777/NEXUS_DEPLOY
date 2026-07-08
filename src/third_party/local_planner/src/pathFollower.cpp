#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace std;

const double PI = 3.1415926;

double sensorOffsetX = 0;
double sensorOffsetY = 0;
int pubSkipNum = 1;
int pubSkipCount = 0;
bool twoWayDrive = true;
double lookAheadDis = 0.5;
double yawRateGain = 7.5;
double stopYawRateGain = 7.5;
double maxYawRate = 45.0;
double maxSpeed = 1.0;
double maxAccel = 1.0;
double switchTimeThre = 1.0;
double dirDiffThre = 0.1;
double stopDisThre = 0.2;
double slowDwnDisThre = 1.0;
bool useInclRateToSlow = false;
double inclRateThre = 120.0;
double slowRate1 = 0.25;
double slowRate2 = 0.5;
double slowTime1 = 2.0;
double slowTime2 = 2.0;
bool useInclToStop = false;
double inclThre = 45.0;
double stopTime = 5.0;
bool noRotAtStop = false;
bool noRotAtGoal = true;
bool autonomyMode = false;
double autonomySpeed = 1.0;
double joyToSpeedDelay = 2.0;
double pathStaleTime = 1.0;

float joySpeed = 0;
float joySpeedRaw = 0;
float joyYaw = 0;
int safetyStop = 0;

float vehicleX = 0;
float vehicleY = 0;
float vehicleZ = 0;
float vehicleRoll = 0;
float vehiclePitch = 0;
float vehicleYaw = 0;

float vehicleXRec = 0;
float vehicleYRec = 0;
float vehicleZRec = 0;
float vehicleRollRec = 0;
float vehiclePitchRec = 0;
float vehicleYawRec = 0;

float vehicleYawRate = 0;
float vehicleSpeed = 0;

double odomTime = 0;
double joyTime = 0;
double slowInitTime = 0;
double stopInitTime = false;
int pathPointID = 0;
bool pathInit = false;
bool navFwd = true;
double switchTime = 0;
double pathTime = -1.0;

nav_msgs::msg::Path path;

static std::shared_ptr<rclcpp::Clock> g_clock;

static builtin_interfaces::msg::Time ToBuiltinTime(const double stamp_seconds)
{
  builtin_interfaces::msg::Time stamp;
  const auto nanoseconds = static_cast<int64_t>(stamp_seconds * 1e9);
  stamp.sec = static_cast<int32_t>(nanoseconds / 1000000000LL);
  stamp.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000LL);
  return stamp;
}

void odomHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odomIn)
{
  odomTime = rclcpp::Time(odomIn->header.stamp).seconds();

  double roll, pitch, yaw;
  const auto& geoQuat = odomIn->pose.pose.orientation;
  tf2::Matrix3x3(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  vehicleRoll = roll;
  vehiclePitch = pitch;
  vehicleYaw = yaw;
  vehicleX = odomIn->pose.pose.position.x - cos(yaw) * sensorOffsetX + sin(yaw) * sensorOffsetY;
  vehicleY = odomIn->pose.pose.position.y - sin(yaw) * sensorOffsetX - cos(yaw) * sensorOffsetY;
  vehicleZ = odomIn->pose.pose.position.z;

  if ((fabs(roll) > inclThre * PI / 180.0 || fabs(pitch) > inclThre * PI / 180.0) && useInclToStop) {
    stopInitTime = rclcpp::Time(odomIn->header.stamp).seconds();
  }

  if ((fabs(odomIn->twist.twist.angular.x) > inclRateThre * PI / 180.0 || fabs(odomIn->twist.twist.angular.y) > inclRateThre * PI / 180.0) && useInclRateToSlow) {
    slowInitTime = rclcpp::Time(odomIn->header.stamp).seconds();
  }
}

void pathHandler(const nav_msgs::msg::Path::ConstSharedPtr pathIn)
{
  pathTime = g_clock->now().seconds();
  int pathSize = pathIn->poses.size();

  // Guard against empty path to prevent out-of-bounds access in main loop
  if (pathSize == 0) {
    pathInit = false;
    return;
  }

  path.poses.resize(pathSize);
  for (int i = 0; i < pathSize; i++) {
    path.poses[i].pose.position.x = pathIn->poses[i].pose.position.x;
    path.poses[i].pose.position.y = pathIn->poses[i].pose.position.y;
    path.poses[i].pose.position.z = pathIn->poses[i].pose.position.z;
  }

  vehicleXRec = vehicleX;
  vehicleYRec = vehicleY;
  vehicleZRec = vehicleZ;
  vehicleRollRec = vehicleRoll;
  vehiclePitchRec = vehiclePitch;
  vehicleYawRec = vehicleYaw;

  pathPointID = 0;
  pathInit = true;
}

void joystickHandler(const sensor_msgs::msg::Joy::ConstSharedPtr joy)
{
  joyTime = g_clock->now().seconds();

  joySpeedRaw = sqrt(joy->axes[3] * joy->axes[3] + joy->axes[4] * joy->axes[4]);
  joySpeed = joySpeedRaw;
  if (joySpeed > 1.0) joySpeed = 1.0;
  if (joy->axes[4] == 0) joySpeed = 0;
  joyYaw = joy->axes[3];
  if (joySpeed == 0 && noRotAtStop) joyYaw = 0;

  if (joy->axes[4] < 0 && !twoWayDrive) {
    joySpeed = 0;
    joyYaw = 0;
  }

  if (joy->axes[2] > -0.1) {
    autonomyMode = false;
  } else {
    autonomyMode = true;
  }
}

void speedHandler(const std_msgs::msg::Float32::ConstSharedPtr speed)
{
  const double speedTime = g_clock->now().seconds();

  if (autonomyMode && speedTime - joyTime > joyToSpeedDelay && joySpeedRaw == 0) {
    joySpeed = speed->data / maxSpeed;

    if (joySpeed < 0) joySpeed = 0;
    else if (joySpeed > 1.0) joySpeed = 1.0;
  }
}

void stopHandler(const std_msgs::msg::Int8::ConstSharedPtr stop)
{
  safetyStop = stop->data;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("pathFollower");
  g_clock = node->get_clock();

  sensorOffsetX = node->declare_parameter<double>("sensorOffsetX", sensorOffsetX);
  sensorOffsetY = node->declare_parameter<double>("sensorOffsetY", sensorOffsetY);
  pubSkipNum = node->declare_parameter<int>("pubSkipNum", pubSkipNum);
  twoWayDrive = node->declare_parameter<bool>("twoWayDrive", twoWayDrive);
  lookAheadDis = node->declare_parameter<double>("lookAheadDis", lookAheadDis);
  yawRateGain = node->declare_parameter<double>("yawRateGain", yawRateGain);
  stopYawRateGain = node->declare_parameter<double>("stopYawRateGain", stopYawRateGain);
  maxYawRate = node->declare_parameter<double>("maxYawRate", maxYawRate);
  maxSpeed = node->declare_parameter<double>("maxSpeed", maxSpeed);
  maxAccel = node->declare_parameter<double>("maxAccel", maxAccel);
  switchTimeThre = node->declare_parameter<double>("switchTimeThre", switchTimeThre);
  dirDiffThre = node->declare_parameter<double>("dirDiffThre", dirDiffThre);
  stopDisThre = node->declare_parameter<double>("stopDisThre", stopDisThre);
  slowDwnDisThre = node->declare_parameter<double>("slowDwnDisThre", slowDwnDisThre);
  useInclRateToSlow = node->declare_parameter<bool>("useInclRateToSlow", useInclRateToSlow);
  inclRateThre = node->declare_parameter<double>("inclRateThre", inclRateThre);
  slowRate1 = node->declare_parameter<double>("slowRate1", slowRate1);
  slowRate2 = node->declare_parameter<double>("slowRate2", slowRate2);
  slowTime1 = node->declare_parameter<double>("slowTime1", slowTime1);
  slowTime2 = node->declare_parameter<double>("slowTime2", slowTime2);
  useInclToStop = node->declare_parameter<bool>("useInclToStop", useInclToStop);
  inclThre = node->declare_parameter<double>("inclThre", inclThre);
  stopTime = node->declare_parameter<double>("stopTime", stopTime);
  noRotAtStop = node->declare_parameter<bool>("noRotAtStop", noRotAtStop);
  noRotAtGoal = node->declare_parameter<bool>("noRotAtGoal", noRotAtGoal);
  autonomyMode = node->declare_parameter<bool>("autonomyMode", autonomyMode);
  autonomySpeed = node->declare_parameter<double>("autonomySpeed", autonomySpeed);
  joyToSpeedDelay = node->declare_parameter<double>("joyToSpeedDelay", joyToSpeedDelay);
  pathStaleTime = node->declare_parameter<double>("pathStaleTime", pathStaleTime);

  auto subOdom = node->create_subscription<nav_msgs::msg::Odometry>(
      "/state_estimation", rclcpp::QoS(5), odomHandler);

  auto subPath = node->create_subscription<nav_msgs::msg::Path>(
      "/path", rclcpp::QoS(5), pathHandler);

  auto subJoystick = node->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", rclcpp::QoS(5), joystickHandler);

  auto subSpeed = node->create_subscription<std_msgs::msg::Float32>(
      "/speed", rclcpp::QoS(5), speedHandler);

  auto subStop = node->create_subscription<std_msgs::msg::Int8>(
      "/stop", rclcpp::QoS(5), stopHandler);

  auto pubSpeed = node->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/cmd_vel2", rclcpp::QoS(5));
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = "vehicle";

  auto pubSpeedG = node->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::QoS(5));
  geometry_msgs::msg::Twist cmd_velG;



  if (autonomyMode) {
    joySpeed = autonomySpeed / maxSpeed;

    if (joySpeed < 0) joySpeed = 0;
    else if (joySpeed > 1.0) joySpeed = 1.0;
  }

  rclcpp::Rate rate(100);
  while (rclcpp::ok()) {
    try {
      rclcpp::spin_some(node);
    } catch (const rclcpp::exceptions::RCLError&) {
      if (!rclcpp::ok()) {
        break;
      }
      throw;
    }

    if (pathInit && pathTime >= 0.0 && g_clock->now().seconds() - pathTime > pathStaleTime) {
      pathInit = false;
      vehicleSpeed = 0;
      vehicleYawRate = 0;
    }

    if (!pathInit) {
      pubSkipCount--;
      if (pubSkipCount < 0) {
        cmd_vel.header.stamp = ToBuiltinTime(odomTime);
        cmd_vel.twist.linear.x = 0.0;
        cmd_vel.twist.angular.z = 0.0;
        pubSpeed->publish(cmd_vel);
        pubSkipCount = pubSkipNum;
      }

      if (!rclcpp::ok()) {
        break;
      }
      rate.sleep();
      continue;
    }

    if (pathInit) {
      float vehicleXRel = cos(vehicleYawRec) * (vehicleX - vehicleXRec) 
                        + sin(vehicleYawRec) * (vehicleY - vehicleYRec);
      float vehicleYRel = -sin(vehicleYawRec) * (vehicleX - vehicleXRec) 
                        + cos(vehicleYawRec) * (vehicleY - vehicleYRec);

      int pathSize = path.poses.size();
      float endDisX = path.poses[pathSize - 1].pose.position.x - vehicleXRel;
      float endDisY = path.poses[pathSize - 1].pose.position.y - vehicleYRel;
      float endDis = sqrt(endDisX * endDisX + endDisY * endDisY);

      float disX, disY, dis;
      while (pathPointID < pathSize - 1) {
        disX = path.poses[pathPointID].pose.position.x - vehicleXRel;
        disY = path.poses[pathPointID].pose.position.y - vehicleYRel;
        dis = sqrt(disX * disX + disY * disY);
        if (dis < lookAheadDis) {
          pathPointID++;
        } else {
          break;
        }
      }

      disX = path.poses[pathPointID].pose.position.x - vehicleXRel;
      disY = path.poses[pathPointID].pose.position.y - vehicleYRel;
      dis = sqrt(disX * disX + disY * disY);
      float pathDir = atan2(disY, disX);

      float dirDiff = vehicleYaw - vehicleYawRec - pathDir;
      if (dirDiff > PI) dirDiff -= 2 * PI;
      else if (dirDiff < -PI) dirDiff += 2 * PI;
      if (dirDiff > PI) dirDiff -= 2 * PI;
      else if (dirDiff < -PI) dirDiff += 2 * PI;

      if (twoWayDrive) {
        const double time = g_clock->now().seconds();
        if (fabs(dirDiff) > PI / 2 && navFwd && time - switchTime > switchTimeThre) {
          navFwd = false;
          switchTime = time;
        } else if (fabs(dirDiff) < PI / 2 && !navFwd && time - switchTime > switchTimeThre) {
          navFwd = true;
          switchTime = time;
        }
      }

      float joySpeed2 = maxSpeed * joySpeed;
      if (!navFwd) {
        dirDiff += PI;
        if (dirDiff > PI) dirDiff -= 2 * PI;
        joySpeed2 *= -1;
      }

      if (fabs(vehicleSpeed) < 2.0 * maxAccel / 100.0) vehicleYawRate = -stopYawRateGain * dirDiff;
      else vehicleYawRate = -yawRateGain * dirDiff;

      if (vehicleYawRate > maxYawRate * PI / 180.0) vehicleYawRate = maxYawRate * PI / 180.0;
      else if (vehicleYawRate < -maxYawRate * PI / 180.0) vehicleYawRate = -maxYawRate * PI / 180.0;

      if (joySpeed2 == 0 && !autonomyMode) {
        vehicleYawRate = maxYawRate * joyYaw * PI / 180.0;
      } else if (pathSize <= 1 || (dis < stopDisThre && noRotAtGoal)) {
        vehicleYawRate = 0;
      }

      if (pathSize <= 1) {
        joySpeed2 = 0;
      } else if (endDis / slowDwnDisThre < joySpeed) {
        joySpeed2 *= endDis / slowDwnDisThre;
      }

      float joySpeed3 = joySpeed2;
      if (odomTime < slowInitTime + slowTime1 && slowInitTime > 0) joySpeed3 *= slowRate1;
      else if (odomTime < slowInitTime + slowTime1 + slowTime2 && slowInitTime > 0) joySpeed3 *= slowRate2;

      if (fabs(dirDiff) < dirDiffThre && dis > stopDisThre) {
        if (vehicleSpeed < joySpeed3) vehicleSpeed += maxAccel / 100.0;
        else if (vehicleSpeed > joySpeed3) vehicleSpeed -= maxAccel / 100.0;
      } else {
        if (vehicleSpeed > 0) vehicleSpeed -= maxAccel / 100.0;
        else if (vehicleSpeed < 0) vehicleSpeed += maxAccel / 100.0;
      }

      if (odomTime < stopInitTime + stopTime && stopInitTime > 0) {
        vehicleSpeed = 0;
        vehicleYawRate = 0;
      }

      if (safetyStop >= 1) vehicleSpeed = 0;
      if (safetyStop >= 2) vehicleYawRate = 0;

      pubSkipCount--;
      if (pubSkipCount < 0) {
        cmd_vel.header.stamp = ToBuiltinTime(odomTime);
        if (fabs(vehicleSpeed) <= maxAccel / 100.0) cmd_vel.twist.linear.x = 0;
        else cmd_vel.twist.linear.x = vehicleSpeed;
        cmd_vel.twist.angular.z = vehicleYawRate;
        pubSpeed->publish(cmd_vel);


        cmd_velG.linear.x = cmd_vel.twist.linear.x;
        cmd_velG.angular.z =cmd_vel.twist.angular.z; 
        // Keep /cmd_vel2 feedback alive, but leave /cmd_vel free for
        // recovery behaviors when the local planner only has a stop path.
        if (pathSize > 1) {
          pubSpeedG->publish(cmd_velG);
        }


        pubSkipCount = pubSkipNum;
      }
    }

    if (!rclcpp::ok()) {
      break;
    }
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
