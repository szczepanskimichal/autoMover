#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include "DriveController.h"

class AutoMowerDriveNode : public rclcpp::Node
{
public:
    AutoMowerDriveNode()
        : Node("automower_drive_node"),
          currentSpeeds_{0.0, 0.0, 0.0, 0.0},
          frontLeftPosition_(0.0),
          frontRightPosition_(0.0),
          rearLeftPosition_(0.0),
          rearRightPosition_(0.0),
          x_(0.0),
          y_(0.0),
          yaw_(0.0),
          lastRequestedLinear_(0.0),
          lastRequestedAngular_(0.0),
          hasLastCommandTime_(false),
          hasLastTime_(false),
          lastCommandTime_(0, 0, this->get_clock()->get_clock_type()),
          lastTime_(0, 0, this->get_clock()->get_clock_type())
    {
        cmdVelSubscription_ =
            this->create_subscription<geometry_msgs::msg::Twist>(
                "/cmd_vel",
                10,
                std::bind(
                    &AutoMowerDriveNode::cmdVelCallback,
                    this,
                    std::placeholders::_1));

        workModeSubscription_ =
            this->create_subscription<std_msgs::msg::String>(
                "/work_mode",
                10,
                std::bind(
                    &AutoMowerDriveNode::workModeCallback,
                    this,
                    std::placeholders::_1));

        emergencyStopSubscription_ =
            this->create_subscription<std_msgs::msg::Bool>(
                "/emergency_stop",
                10,
                std::bind(
                    &AutoMowerDriveNode::emergencyStopCallback,
                    this,
                    std::placeholders::_1));

        mowerSafetyStopSubscription_ =
            this->create_subscription<std_msgs::msg::Bool>(
                "/mower/safety_stop",
                10,
                std::bind(
                    &AutoMowerDriveNode::mowerSafetyStopCallback,
                    this,
                    std::placeholders::_1));

        bladeEnabledSubscription_ =
            this->create_subscription<std_msgs::msg::Bool>(
                "/mower/blade_enabled",
                10,
                std::bind(
                    &AutoMowerDriveNode::bladeEnabledCallback,
                    this,
                    std::placeholders::_1));

        cutHeightSubscription_ =
            this->create_subscription<std_msgs::msg::Float32>(
                "/mower/cut_height",
                10,
                std::bind(
                    &AutoMowerDriveNode::cutHeightCallback,
                    this,
                    std::placeholders::_1));

        jointStatePublisher_ =
            this->create_publisher<sensor_msgs::msg::JointState>(
                "/joint_states",
                10);

        odomPublisher_ =
            this->create_publisher<nav_msgs::msg::Odometry>(
                "/odom",
                10);

        markerPublisher_ =
            this->create_publisher<visualization_msgs::msg::MarkerArray>(
                "/visualization_marker_array",
                rclcpp::QoS(1).transient_local());

        tfBroadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        markerTimer_ =
            this->create_wall_timer(
                std::chrono::milliseconds(500),
                std::bind(
                    &AutoMowerDriveNode::publishVisualization,
                    this));

        stateUpdateTimer_ =
            this->create_wall_timer(
                std::chrono::milliseconds(50),
                std::bind(
                    &AutoMowerDriveNode::publishState,
                    this));

        publishVisualization();
        // Seed odometry and TF immediately so visualization has a fixed frame.
        publishState();

        RCLCPP_INFO(
            this->get_logger(),
            "autoMower drive node started in mode '%s'",
            workMode_.c_str());
    }

private:
    static constexpr double WHEEL_RADIUS = 0.18;

    static constexpr double TRACK_WIDTH = 0.96;

    static constexpr double BODY_LENGTH = 1.2;

    static constexpr double BODY_WIDTH = 0.8;

    static constexpr double BODY_HEIGHT = 0.2;

    static constexpr double WHEEL_WIDTH = 0.12;

    static constexpr double WHEEL_X = 0.45;

    static constexpr double WHEEL_Y = 0.48;

    static constexpr double DECK_RADIUS = 0.33;

    static constexpr double DECK_HEIGHT = 0.12;

    static constexpr double DECK_X = 0.05;

    static constexpr double DECK_Z = -0.17;

    static constexpr double BLADE_LENGTH = 0.48;

    static constexpr double BLADE_WIDTH = 0.05;

    static constexpr double BLADE_HEIGHT = 0.02;

    static constexpr double LIDAR_MOUNT_X = 0.23;

    static constexpr double LIDAR_MOUNT_Z = 0.18;

    static constexpr double LIDAR_MAST_RADIUS = 0.03;

    static constexpr double LIDAR_MAST_HEIGHT = 0.16;

    static constexpr double LIDAR_RADIUS = 0.075;

    static constexpr double LIDAR_HEIGHT = 0.06;

    static constexpr double HALF_PI = 1.57079632679;

    static constexpr double CMD_VEL_TIMEOUT = 0.25;

    static constexpr double DEFAULT_CUT_HEIGHT = 0.09;

    static constexpr double MIN_CUT_HEIGHT = 0.06;

    static constexpr double MAX_CUT_HEIGHT = 0.12;

    static constexpr double SAFETY_ZONE_X = 0.42;

    static constexpr double SAFETY_ZONE_Z = -0.20;

    static constexpr double SAFETY_ZONE_LENGTH = 0.55;

    static constexpr double SAFETY_ZONE_WIDTH = 0.70;

    static constexpr double SAFETY_ZONE_HEIGHT = 0.03;

    static constexpr double TRAIL_POINT_SPACING = 0.18;

    static constexpr std::size_t MAX_TRAIL_POINTS = 120;

    // Na razie WheelSpeeds są komendą -1..1.
    // Przyjmujemy, że 1.0 = 1 m/s prędkości liniowej koła.
    static constexpr double MAX_WHEEL_LINEAR_SPEED = 1.0;

    void cmdVelCallback(
        const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (emergencyStopActive_ || !driveAllowedInCurrentMode())
        {
            currentSpeeds_ = WheelSpeeds{0.0, 0.0, 0.0, 0.0};
            return;
        }

        hasLastCommandTime_ = true;
        lastCommandTime_ = this->now();
        lastRequestedLinear_ = msg->linear.x;
        lastRequestedAngular_ = msg->angular.z;

        // Cache the latest requested motion. The fixed-rate state loop below
        // consumes it so integration stays smooth even if key events are bursty.
        currentSpeeds_ =
            driveController_.calculate(
                clampForwardMotion(msg->linear.x),
                msg->angular.z);
    }

    void workModeCallback(
        const std_msgs::msg::String::SharedPtr msg)
    {
        if (msg->data == workMode_)
        {
            return;
        }

        workMode_ = msg->data;

        RCLCPP_INFO(
            this->get_logger(),
            "drive mode changed to '%s'",
            workMode_.c_str());
    }

    void emergencyStopCallback(
        const std_msgs::msg::Bool::SharedPtr msg)
    {
        emergencyStopActive_ = msg->data;

        if (emergencyStopActive_)
        {
            currentSpeeds_ = WheelSpeeds{0.0, 0.0, 0.0, 0.0};
            hasLastCommandTime_ = false;
            RCLCPP_WARN(this->get_logger(), "emergency stop active");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "emergency stop released");
        }
    }

    void mowerSafetyStopCallback(
        const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data == mowerSafetyStopActive_)
        {
            return;
        }

        mowerSafetyStopActive_ = msg->data;
        currentSpeeds_ =
            driveController_.calculate(
                clampForwardMotion(lastRequestedLinear_),
                lastRequestedAngular_);

        if (mowerSafetyStopActive_)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "mower safety stop active: blocking forward motion");
        }
        else
        {
            RCLCPP_INFO(
                this->get_logger(),
                "mower safety stop released");
        }
    }

    bool driveAllowedInCurrentMode() const
    {
        return workMode_ == "manual_drive" ||
               workMode_ == "mowing";
    }

    double clampForwardMotion(double linearVelocity) const
    {
        if (!mowerSafetyStopActive_)
        {
            return linearVelocity;
        }

        return std::min(linearVelocity, 0.0);
    }

    void publishState()
    {
        const auto currentTime = this->now();

        if (!hasLastTime_)
        {
            hasLastTime_ = true;
            lastTime_ = currentTime;
        }

        WheelSpeeds speeds = currentSpeeds_;

        if (emergencyStopActive_ || !driveAllowedInCurrentMode())
        {
            speeds = WheelSpeeds{0.0, 0.0, 0.0, 0.0};
            currentSpeeds_ = speeds;
            hasLastCommandTime_ = false;
        }

        // If teleop stops publishing, decay immediately to a stationary target
        // so odometry and TF remain valid without runaway motion.
        if (!hasLastCommandTime_ ||
            (currentTime - lastCommandTime_).seconds() >= CMD_VEL_TIMEOUT)
        {
            speeds = WheelSpeeds{0.0, 0.0, 0.0, 0.0};
            currentSpeeds_ = speeds;
        }
        else if (mowerSafetyStopActive_)
        {
            speeds = driveController_.calculate(
                clampForwardMotion(lastRequestedLinear_),
                lastRequestedAngular_);
            currentSpeeds_ = speeds;
        }

        const double dt =
            (currentTime - lastTime_).seconds();

        lastTime_ = currentTime;

        if (dt > 0.0)
        {
            updateWheelPositions(
                speeds,
                dt);

            updateOdometry(
                speeds,
                dt);

            updateMowingTrail(speeds);
        }

        publishJointStates(
            speeds,
            currentTime);

        publishOdometry(
            speeds,
            currentTime);
    }

    void updateWheelPositions(
        const WheelSpeeds &speeds,
        double dt)
    {
        // WheelSpeeds -> m/s -> rad/s
        const double flAngular =
            speeds.frontLeft *
            MAX_WHEEL_LINEAR_SPEED /
            WHEEL_RADIUS;

        const double frAngular =
            speeds.frontRight *
            MAX_WHEEL_LINEAR_SPEED /
            WHEEL_RADIUS;

        const double rlAngular =
            speeds.rearLeft *
            MAX_WHEEL_LINEAR_SPEED /
            WHEEL_RADIUS;

        const double rrAngular =
            speeds.rearRight *
            MAX_WHEEL_LINEAR_SPEED /
            WHEEL_RADIUS;

        frontLeftPosition_ += flAngular * dt;
        frontRightPosition_ += frAngular * dt;
        rearLeftPosition_ += rlAngular * dt;
        rearRightPosition_ += rrAngular * dt;
    }

    void updateOdometry(
        const WheelSpeeds &speeds,
        double dt)
    {
        const double leftVelocity =
            (speeds.frontLeft +
             speeds.rearLeft) /
            2.0 *
            MAX_WHEEL_LINEAR_SPEED;

        const double rightVelocity =
            (speeds.frontRight +
             speeds.rearRight) /
            2.0 *
            MAX_WHEEL_LINEAR_SPEED;

        const double linearVelocity =
            (leftVelocity + rightVelocity) / 2.0;

        // Nasz DriveController ma:
        // left = throttle + steering
        // right = throttle - steering
        // dlatego używamy left - right,
        // aby dodatnie steering dawało dodatnie yaw.
        const double angularVelocity =
            (leftVelocity - rightVelocity) /
            TRACK_WIDTH;

        x_ +=
            linearVelocity *
            std::cos(yaw_) *
            dt;

        y_ +=
            linearVelocity *
            std::sin(yaw_) *
            dt;

        yaw_ +=
            angularVelocity *
            dt;
    }

    void publishJointStates(
        const WheelSpeeds &speeds,
        const rclcpp::Time &stamp)
    {
        sensor_msgs::msg::JointState jointState;

        jointState.header.stamp = stamp;

        jointState.name = {
            "front_left_wheel_joint",
            "front_right_wheel_joint",
            "rear_left_wheel_joint",
            "rear_right_wheel_joint"};

        jointState.position = {
            frontLeftPosition_,
            frontRightPosition_,
            rearLeftPosition_,
            rearRightPosition_};

        jointState.velocity = {
            speeds.frontLeft *
                MAX_WHEEL_LINEAR_SPEED /
                WHEEL_RADIUS,

            speeds.frontRight *
                MAX_WHEEL_LINEAR_SPEED /
                WHEEL_RADIUS,

            speeds.rearLeft *
                MAX_WHEEL_LINEAR_SPEED /
                WHEEL_RADIUS,

            speeds.rearRight *
                MAX_WHEEL_LINEAR_SPEED /
                WHEEL_RADIUS};

        jointStatePublisher_->publish(jointState);
    }

    void publishOdometry(
        const WheelSpeeds &speeds,
        const rclcpp::Time &stamp)
    {
        const double leftVelocity =
            (speeds.frontLeft +
             speeds.rearLeft) /
            2.0 *
            MAX_WHEEL_LINEAR_SPEED;

        const double rightVelocity =
            (speeds.frontRight +
             speeds.rearRight) /
            2.0 *
            MAX_WHEEL_LINEAR_SPEED;

        const double linearVelocity =
            (leftVelocity + rightVelocity) / 2.0;

        const double angularVelocity =
            (leftVelocity - rightVelocity) /
            TRACK_WIDTH;

        tf2::Quaternion quaternion;

        quaternion.setRPY(
            0.0,
            0.0,
            yaw_);

        nav_msgs::msg::Odometry odom;

        odom.header.stamp = stamp;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_footprint";

        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;

        odom.pose.pose.orientation.x = quaternion.x();
        odom.pose.pose.orientation.y = quaternion.y();
        odom.pose.pose.orientation.z = quaternion.z();
        odom.pose.pose.orientation.w = quaternion.w();

        odom.twist.twist.linear.x =
            linearVelocity;

        odom.twist.twist.angular.z =
            angularVelocity;

        odomPublisher_->publish(odom);

        geometry_msgs::msg::TransformStamped transform;

        transform.header.stamp = stamp;
        transform.header.frame_id = "odom";
        transform.child_frame_id = "base_footprint";

        transform.transform.translation.x = x_;
        transform.transform.translation.y = y_;
        transform.transform.translation.z = 0.0;

        transform.transform.rotation.x = quaternion.x();
        transform.transform.rotation.y = quaternion.y();
        transform.transform.rotation.z = quaternion.z();
        transform.transform.rotation.w = quaternion.w();

        tfBroadcaster_->sendTransform(transform);
    }

    void publishVisualization()
    {
        visualization_msgs::msg::MarkerArray markers;
        const auto stamp = this->now();

        // Markers provide the fastest visible body in Foxglove even without a
        // dedicated URDF layer configuration.
        markers.markers.push_back(
            makeBodyMarker(stamp));

        markers.markers.push_back(
            makeWheelMarker(stamp, 1, WHEEL_X, WHEEL_Y));

        markers.markers.push_back(
            makeWheelMarker(stamp, 2, WHEEL_X, -WHEEL_Y));

        markers.markers.push_back(
            makeWheelMarker(stamp, 3, -WHEEL_X, WHEEL_Y));

        markers.markers.push_back(
            makeWheelMarker(stamp, 4, -WHEEL_X, -WHEEL_Y));

        markers.markers.push_back(
            makeDeckMarker(stamp));

        markers.markers.push_back(
            makeBladeMarker(stamp));

        markers.markers.push_back(
            makeSafetyZoneMarker(stamp));

        markers.markers.push_back(
            makeMowingTrailMarker(stamp));

        markers.markers.push_back(
            makeLidarMastMarker(stamp));

        markers.markers.push_back(
            makeLidarMarker(stamp));

        markerPublisher_->publish(markers);
    }

    visualization_msgs::msg::Marker makeBodyMarker(
        const rclcpp::Time &stamp) const
    {
        visualization_msgs::msg::Marker marker;

        marker.header.stamp = stamp;
        marker.header.frame_id = "base_link";
        marker.ns = "automower";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.orientation.w = 1.0;

        marker.scale.x = BODY_LENGTH;
        marker.scale.y = BODY_WIDTH;
        marker.scale.z = BODY_HEIGHT;

        marker.color.r = 0.1F;
        marker.color.g = 0.35F;
        marker.color.b = 0.1F;
        marker.color.a = 1.0F;

        return marker;
    }

    void bladeEnabledCallback(
        const std_msgs::msg::Bool::SharedPtr msg)
    {
        bladeEnabled_ = msg->data;
    }

    void cutHeightCallback(
        const std_msgs::msg::Float32::SharedPtr msg)
    {
        cutHeight_ = std::clamp(
            static_cast<double>(msg->data),
            MIN_CUT_HEIGHT,
            MAX_CUT_HEIGHT);
    }

    void updateMowingTrail(const WheelSpeeds &speeds)
    {
        if (!bladeEnabled_ || mowerSafetyStopActive_)
        {
            return;
        }

        const double leftVelocity =
            (speeds.frontLeft +
             speeds.rearLeft) /
            2.0 *
            MAX_WHEEL_LINEAR_SPEED;

        const double rightVelocity =
            (speeds.frontRight +
             speeds.rearRight) /
            2.0 *
            MAX_WHEEL_LINEAR_SPEED;

        const double linearVelocity =
            (leftVelocity + rightVelocity) / 2.0;

        if (linearVelocity <= 0.02)
        {
            return;
        }

        geometry_msgs::msg::Point trailPoint;
        trailPoint.x = x_ + std::cos(yaw_) * DECK_X;
        trailPoint.y = y_ + std::sin(yaw_) * DECK_X;
        trailPoint.z = 0.01;

        if (!hasLastTrailPoint_)
        {
            mowingTrail_.push_back(trailPoint);
            lastTrailPoint_ = trailPoint;
            hasLastTrailPoint_ = true;
            return;
        }

        const double dx = trailPoint.x - lastTrailPoint_.x;
        const double dy = trailPoint.y - lastTrailPoint_.y;

        if (std::hypot(dx, dy) < TRAIL_POINT_SPACING)
        {
            return;
        }

        mowingTrail_.push_back(trailPoint);
        lastTrailPoint_ = trailPoint;

        if (mowingTrail_.size() > MAX_TRAIL_POINTS)
        {
            mowingTrail_.erase(mowingTrail_.begin());
        }
    }

    visualization_msgs::msg::Marker makeDeckMarker(
        const rclcpp::Time &stamp) const
    {
        visualization_msgs::msg::Marker marker;

        marker.header.stamp = stamp;
        marker.header.frame_id = "base_link";
        marker.ns = "automower";
        marker.id = 5;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = DECK_X;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = DECK_Z;

        marker.pose.orientation.w = 1.0;

        marker.scale.x = DECK_RADIUS * 2.0;
        marker.scale.y = DECK_RADIUS * 2.0;
        marker.scale.z = DECK_HEIGHT;

        marker.color.r = 0.75F;
        marker.color.g = 0.12F;
        marker.color.b = 0.10F;
        marker.color.a = 1.0F;

        return marker;
    }

    visualization_msgs::msg::Marker makeBladeMarker(
        const rclcpp::Time &stamp) const
    {
        visualization_msgs::msg::Marker marker;

        marker.header.stamp = stamp;
        marker.header.frame_id = "base_link";
        marker.ns = "automower";
        marker.id = 6;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = DECK_X;
        marker.pose.position.y = 0.0;
        marker.pose.position.z =
            DECK_Z - 0.02 + (cutHeight_ - DEFAULT_CUT_HEIGHT);
        marker.pose.orientation.w = 1.0;

        marker.scale.x = BLADE_LENGTH;
        marker.scale.y = BLADE_WIDTH;
        marker.scale.z = BLADE_HEIGHT;

        if (mowerSafetyStopActive_)
        {
            marker.color.r = 0.85F;
            marker.color.g = 0.18F;
            marker.color.b = 0.14F;
        }
        else if (bladeEnabled_)
        {
            marker.color.r = 0.95F;
            marker.color.g = 0.76F;
            marker.color.b = 0.10F;
        }
        else
        {
            marker.color.r = 0.75F;
            marker.color.g = 0.75F;
            marker.color.b = 0.78F;
        }
        marker.color.a = 1.0F;

        return marker;
    }

    visualization_msgs::msg::Marker makeSafetyZoneMarker(
        const rclcpp::Time &stamp) const
    {
        visualization_msgs::msg::Marker marker;

        marker.header.stamp = stamp;
        marker.header.frame_id = "base_link";
        marker.ns = "automower";
        marker.id = 9;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = SAFETY_ZONE_X;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = SAFETY_ZONE_Z;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = SAFETY_ZONE_LENGTH;
        marker.scale.y = SAFETY_ZONE_WIDTH;
        marker.scale.z = SAFETY_ZONE_HEIGHT;

        if (mowerSafetyStopActive_)
        {
            marker.color.r = 0.95F;
            marker.color.g = 0.20F;
            marker.color.b = 0.16F;
            marker.color.a = 0.45F;
        }
        else
        {
            marker.color.r = 0.16F;
            marker.color.g = 0.80F;
            marker.color.b = 0.24F;
            marker.color.a = 0.22F;
        }

        return marker;
    }

    visualization_msgs::msg::Marker makeMowingTrailMarker(
        const rclcpp::Time &stamp) const
    {
        visualization_msgs::msg::Marker marker;

        marker.header.stamp = stamp;
        marker.header.frame_id = "odom";
        marker.ns = "automower";
        marker.id = 10;
        marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.08;
        marker.scale.y = DECK_RADIUS * 1.35;
        marker.scale.z = 0.01;
        marker.color.r = 0.56F;
        marker.color.g = 0.82F;
        marker.color.b = 0.42F;
        marker.color.a = 0.65F;
        marker.points = mowingTrail_;

        return marker;
    }

    visualization_msgs::msg::Marker makeLidarMastMarker(
        const rclcpp::Time &stamp) const
    {
        visualization_msgs::msg::Marker marker;

        marker.header.stamp = stamp;
        marker.header.frame_id = "base_link";
        marker.ns = "automower";
        marker.id = 7;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = LIDAR_MOUNT_X;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = LIDAR_MOUNT_Z + LIDAR_MAST_HEIGHT * 0.5;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = LIDAR_MAST_RADIUS * 2.0;
        marker.scale.y = LIDAR_MAST_RADIUS * 2.0;
        marker.scale.z = LIDAR_MAST_HEIGHT;

        marker.color.r = 0.08F;
        marker.color.g = 0.08F;
        marker.color.b = 0.08F;
        marker.color.a = 1.0F;

        return marker;
    }

    visualization_msgs::msg::Marker makeLidarMarker(
        const rclcpp::Time &stamp) const
    {
        visualization_msgs::msg::Marker marker;
        tf2::Quaternion quaternion;

        quaternion.setRPY(
            HALF_PI,
            0.0,
            0.0);

        marker.header.stamp = stamp;
        marker.header.frame_id = "base_link";
        marker.ns = "automower";
        marker.id = 8;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = LIDAR_MOUNT_X;
        marker.pose.position.y = 0.0;
        marker.pose.position.z =
            LIDAR_MOUNT_Z + LIDAR_MAST_HEIGHT + LIDAR_HEIGHT * 0.5;

        marker.pose.orientation.x = quaternion.x();
        marker.pose.orientation.y = quaternion.y();
        marker.pose.orientation.z = quaternion.z();
        marker.pose.orientation.w = quaternion.w();

        marker.scale.x = LIDAR_RADIUS * 2.0;
        marker.scale.y = LIDAR_RADIUS * 2.0;
        marker.scale.z = LIDAR_HEIGHT;

        marker.color.r = 0.78F;
        marker.color.g = 0.68F;
        marker.color.b = 0.22F;
        marker.color.a = 1.0F;

        return marker;
    }

    visualization_msgs::msg::Marker makeWheelMarker(
        const rclcpp::Time &stamp,
        int id,
        double x,
        double y) const
    {
        visualization_msgs::msg::Marker marker;
        tf2::Quaternion quaternion;

        quaternion.setRPY(
            HALF_PI,
            0.0,
            0.0);

        marker.header.stamp = stamp;
        marker.header.frame_id = "base_link";
        marker.ns = "automower";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = 0.0;

        marker.pose.orientation.x = quaternion.x();
        marker.pose.orientation.y = quaternion.y();
        marker.pose.orientation.z = quaternion.z();
        marker.pose.orientation.w = quaternion.w();

        marker.scale.x = WHEEL_RADIUS * 2.0;
        marker.scale.y = WHEEL_RADIUS * 2.0;
        marker.scale.z = WHEEL_WIDTH;

        marker.color.r = 0.03F;
        marker.color.g = 0.03F;
        marker.color.b = 0.03F;
        marker.color.a = 1.0F;

        return marker;
    }

    DriveController driveController_;

    WheelSpeeds currentSpeeds_;

    double frontLeftPosition_;
    double frontRightPosition_;
    double rearLeftPosition_;
    double rearRightPosition_;

    double x_;
    double y_;
    double yaw_;

    double lastRequestedLinear_;

    double lastRequestedAngular_;

    bool hasLastCommandTime_;

    bool hasLastTime_;

    bool emergencyStopActive_ = false;

    bool mowerSafetyStopActive_ = false;

    bool bladeEnabled_ = false;

    double cutHeight_ = DEFAULT_CUT_HEIGHT;

    bool hasLastTrailPoint_ = false;

    geometry_msgs::msg::Point lastTrailPoint_;

    std::vector<geometry_msgs::msg::Point> mowingTrail_;

    std::string workMode_ = "mowing";

    rclcpp::Time lastCommandTime_;

    rclcpp::Time lastTime_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
        cmdVelSubscription_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
        workModeSubscription_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
        emergencyStopSubscription_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
        mowerSafetyStopSubscription_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
        bladeEnabledSubscription_;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr
        cutHeightSubscription_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
        jointStatePublisher_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr
        odomPublisher_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        markerPublisher_;

    std::unique_ptr<tf2_ros::TransformBroadcaster>
        tfBroadcaster_;

    rclcpp::TimerBase::SharedPtr
        markerTimer_;

    rclcpp::TimerBase::SharedPtr
        stateUpdateTimer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<AutoMowerDriveNode>());

    rclcpp::shutdown();

    return 0;
}