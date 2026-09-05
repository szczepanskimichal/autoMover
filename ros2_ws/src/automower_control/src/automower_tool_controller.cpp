#include <algorithm>
#include <cmath>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

class AutomowerToolController : public rclcpp::Node
{
public:
  AutomowerToolController()
      : Node("automower_tool_controller")
  {
    cmdVelSubscription_ =
        this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            10,
            std::bind(
                &AutomowerToolController::cmdVelCallback,
                this,
                std::placeholders::_1));

    profileSubscription_ =
        this->create_subscription<std_msgs::msg::String>(
            "/tool_profile",
            10,
            std::bind(
                &AutomowerToolController::profileCallback,
                this,
                std::placeholders::_1));

    enabledSubscription_ =
        this->create_subscription<std_msgs::msg::Bool>(
            "/tool_enabled",
            10,
            std::bind(
                &AutomowerToolController::enabledCallback,
                this,
                std::placeholders::_1));

    powerSubscription_ =
        this->create_subscription<std_msgs::msg::Float32>(
            "/tool_power",
            10,
            std::bind(
                &AutomowerToolController::powerCallback,
                this,
                std::placeholders::_1));

    angleSubscription_ =
        this->create_subscription<std_msgs::msg::Float32>(
            "/tool_angle",
            10,
            std::bind(
                &AutomowerToolController::angleCallback,
                this,
                std::placeholders::_1));

    workModeSubscription_ =
        this->create_subscription<std_msgs::msg::String>(
            "/work_mode",
            10,
            std::bind(
                &AutomowerToolController::workModeCallback,
                this,
                std::placeholders::_1));

    emergencyStopSubscription_ =
        this->create_subscription<std_msgs::msg::Bool>(
            "/emergency_stop",
            10,
            std::bind(
                &AutomowerToolController::emergencyStopCallback,
                this,
                std::placeholders::_1));

    engineEnabledSubscription_ =
        this->create_subscription<std_msgs::msg::Bool>(
            "/engine_enabled",
            10,
            std::bind(
                &AutomowerToolController::engineEnabledCallback,
                this,
                std::placeholders::_1));

    engineThrottleSubscription_ =
        this->create_subscription<std_msgs::msg::Float32>(
            "/engine_throttle",
            10,
            std::bind(
                &AutomowerToolController::engineThrottleCallback,
                this,
                std::placeholders::_1));

    augerEngagedSubscription_ =
        this->create_subscription<std_msgs::msg::Bool>(
            "/auger_engaged",
            10,
            std::bind(
                &AutomowerToolController::augerEngagedCallback,
                this,
                std::placeholders::_1));

    chutePositionSubscription_ =
        this->create_subscription<std_msgs::msg::Float32>(
            "/chute_position",
            10,
            std::bind(
                &AutomowerToolController::chutePositionCallback,
                this,
                std::placeholders::_1));

    deflectorPositionSubscription_ =
        this->create_subscription<std_msgs::msg::Float32>(
            "/deflector_position",
            10,
            std::bind(
                &AutomowerToolController::deflectorPositionCallback,
                this,
                std::placeholders::_1));

    augerOverloadSubscription_ =
        this->create_subscription<std_msgs::msg::Bool>(
            "/simulate_auger_overload",
            10,
            std::bind(
                &AutomowerToolController::augerOverloadCallback,
                this,
                std::placeholders::_1));

    resetAugerFaultSubscription_ =
        this->create_subscription<std_msgs::msg::Bool>(
            "/reset_auger_fault",
            10,
            std::bind(
                &AutomowerToolController::resetAugerFaultCallback,
                this,
                std::placeholders::_1));

    engineRunningPublisher_ =
        this->create_publisher<std_msgs::msg::Bool>(
            "/hybrid/engine_running",
            10);

    engineRpmPublisher_ =
        this->create_publisher<std_msgs::msg::Float32>(
            "/hybrid/engine_rpm",
            10);

    batterySocPublisher_ =
        this->create_publisher<std_msgs::msg::Float32>(
            "/hybrid/battery_soc",
            10);

    batteryVoltagePublisher_ =
        this->create_publisher<std_msgs::msg::Float32>(
            "/hybrid/battery_voltage",
            10);

    dcBusCurrentPublisher_ =
        this->create_publisher<std_msgs::msg::Float32>(
            "/hybrid/dc_bus_current",
            10);

    tractionPowerLimitPublisher_ =
        this->create_publisher<std_msgs::msg::Float32>(
            "/hybrid/traction_power_limit",
            10);

    augerEngagedPublisher_ =
        this->create_publisher<std_msgs::msg::Bool>(
            "/hybrid/auger_engaged",
            10);

    augerRpmPublisher_ =
        this->create_publisher<std_msgs::msg::Float32>(
            "/hybrid/auger_rpm",
            10);

    augerOverloadPublisher_ =
        this->create_publisher<std_msgs::msg::Bool>(
            "/hybrid/auger_overload",
            10);

    augerInterlockOkPublisher_ =
        this->create_publisher<std_msgs::msg::Bool>(
            "/hybrid/auger_interlock_ok",
            10);

    augerFaultLatchedPublisher_ =
        this->create_publisher<std_msgs::msg::Bool>(
            "/hybrid/auger_fault_latched",
            10);

    augerResetRequiredPublisher_ =
        this->create_publisher<std_msgs::msg::Bool>(
            "/hybrid/auger_reset_required",
            10);

    augerStatusTextPublisher_ =
        this->create_publisher<std_msgs::msg::String>(
            "/hybrid/auger_status_text",
            10);

    chutePositionPublisher_ =
        this->create_publisher<std_msgs::msg::Float32>(
            "/hybrid/chute_position",
            10);

    deflectorPositionPublisher_ =
        this->create_publisher<std_msgs::msg::Float32>(
            "/hybrid/deflector_position",
            10);

    statusTimer_ =
        this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(
                &AutomowerToolController::updateHybridState,
                this));

    RCLCPP_INFO(
        this->get_logger(),
        "tool controller ready with profile '%s'",
        toolProfile_.c_str());
  }

private:
  static constexpr float MAX_TELEOP_LINEAR_SPEED = 0.35F;
  static constexpr float MAX_TELEOP_ANGULAR_SPEED = 0.5F;
  static constexpr float ENGINE_IDLE_RPM = 1800.0F;
  static constexpr float ENGINE_MAX_RPM = 3600.0F;
  static constexpr float AUGER_MAX_RPM = 1200.0F;
  static constexpr float BATTERY_SOC_FLOOR = 0.20F;
  static constexpr float MIN_AUGER_ENGAGE_THROTTLE = 0.25F;
  static constexpr float MAX_FAULT_RESET_THROTTLE = 0.10F;
  static constexpr float MIN_AUGER_VOLTAGE = 45.0F;

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    lastLinearCommand_ = static_cast<float>(msg->linear.x);
    lastAngularCommand_ = static_cast<float>(msg->angular.z);
  }

  void profileCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    const std::string nextProfile = normalizeProfile(msg->data);

    if (nextProfile == toolProfile_)
    {
      return;
    }

    toolProfile_ = nextProfile;

    RCLCPP_INFO(
        this->get_logger(),
        "tool profile set to '%s'",
        toolProfile_.c_str());
  }

  void enabledCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    augerEngagedCommand_ = msg->data;
  }

  void powerCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    engineThrottleCommand_ = std::clamp(msg->data, 0.0F, 1.0F);
  }

  void angleCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    chutePositionCommand_ = std::clamp(msg->data, -1.0F, 1.0F);
  }

  void workModeCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    workMode_ = msg->data;
  }

  void emergencyStopCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    emergencyStopActive_ = msg->data;

    if (emergencyStopActive_)
    {
      engineEnabledCommand_ = false;
      augerEngagedCommand_ = false;
      engineThrottleCommand_ = 0.0F;
    }
  }

  void engineEnabledCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    engineEnabledCommand_ = msg->data;
  }

  void engineThrottleCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    engineThrottleCommand_ = std::clamp(msg->data, 0.0F, 1.0F);
  }

  void augerEngagedCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    augerEngagedCommand_ = msg->data;
  }

  void chutePositionCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    chutePositionCommand_ = std::clamp(msg->data, -1.0F, 1.0F);
  }

  void deflectorPositionCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    deflectorPositionCommand_ = std::clamp(msg->data, -1.0F, 1.0F);
  }

  void augerOverloadCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    simulateAugerOverload_ = msg->data;
  }

  void resetAugerFaultCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    resetAugerFaultRequested_ = msg->data;
  }

  float clampUnit(float value) const
  {
    return std::clamp(value, 0.0F, 1.0F);
  }

  float moveTowards(float current, float target, float alpha) const
  {
    return current + (target - current) * alpha;
  }

  void updateHybridState()
  {
    const auto now = this->now();
    float dtSeconds = 0.2F;

    if (hasLastUpdateTime_)
    {
      dtSeconds = static_cast<float>((now - lastUpdateTime_).seconds());
    }
    else
    {
      hasLastUpdateTime_ = true;
    }

    lastUpdateTime_ = now;

    const bool augerModeActive =
        !emergencyStopActive_ &&
        workMode_ == "snow_clearing" &&
        toolProfile_ == "auger";

    const bool engineRunning =
        engineEnabledCommand_ &&
        !emergencyStopActive_;

    const bool throttleReadyForAuger =
        engineThrottleCommand_ >= MIN_AUGER_ENGAGE_THROTTLE;

    const bool batteryReadyForAuger =
        batteryVoltage_ >= MIN_AUGER_VOLTAGE;

    const bool resetConditionsMet =
        !simulateAugerOverload_ &&
        !augerEngagedCommand_ &&
        engineThrottleCommand_ <= MAX_FAULT_RESET_THROTTLE &&
        !emergencyStopActive_;

    if (resetAugerFaultRequested_ && resetConditionsMet)
    {
      augerFaultLatched_ = false;
    }

    const bool augerInterlockOk =
        augerModeActive &&
        engineRunning &&
        throttleReadyForAuger &&
        batteryReadyForAuger &&
        !augerFaultLatched_;

    const bool effectiveAugerEngaged =
        augerInterlockOk &&
        augerEngagedCommand_;

    const float tractionDemand = clampUnit(
        std::fabs(lastLinearCommand_) / MAX_TELEOP_LINEAR_SPEED +
        0.6F *
            (std::fabs(lastAngularCommand_) / MAX_TELEOP_ANGULAR_SPEED));

    const float targetEngineRpm = engineRunning
                                      ? ENGINE_IDLE_RPM +
                                            engineThrottleCommand_ *
                                                (ENGINE_MAX_RPM - ENGINE_IDLE_RPM)
                                      : 0.0F;

    const float augerLoad = effectiveAugerEngaged
                                ? (0.35F + engineThrottleCommand_ * 0.65F)
                                : 0.0F;

    float tractionPowerLimit = engineRunning ? 1.0F : 0.45F;

    if (effectiveAugerEngaged)
    {
      tractionPowerLimit -= 0.25F + augerLoad * 0.25F;
    }

    tractionPowerLimit -= tractionDemand * 0.15F;

    if (simulateAugerOverload_ && effectiveAugerEngaged)
    {
      tractionPowerLimit -= 0.25F;
      augerFaultLatched_ = true;
      augerEngagedCommand_ = false;
    }

    if (augerFaultLatched_)
    {
      tractionPowerLimit -= 0.10F;
    }

    tractionPowerLimit = std::clamp(tractionPowerLimit, 0.2F, 1.0F);

    const float targetAugerRpm =
        effectiveAugerEngaged
            ? ((simulateAugerOverload_ ? 0.25F : 0.7F) *
               AUGER_MAX_RPM *
               std::max(engineThrottleCommand_, 0.3F))
            : 0.0F;

    const float baseBusCurrent =
        8.0F +
        tractionDemand * 35.0F +
        (effectiveAugerEngaged ? 18.0F + augerLoad * 25.0F : 0.0F) +
        (simulateAugerOverload_ && effectiveAugerEngaged ? 12.0F : 0.0F);

    const float chargeRatePerSecond = engineRunning
                                          ? (engineThrottleCommand_ * 0.00035F -
                                             tractionDemand * 0.00015F -
                                             augerLoad * 0.00020F)
                                          : (-0.00004F -
                                             tractionDemand * 0.00012F -
                                             augerLoad * 0.00010F);

    batterySoc_ = std::clamp(
        batterySoc_ + chargeRatePerSecond * dtSeconds,
        BATTERY_SOC_FLOOR,
        1.0F);

    engineRpm_ = moveTowards(engineRpm_, targetEngineRpm, 0.35F);
    augerRpm_ = moveTowards(augerRpm_, targetAugerRpm, 0.30F);
    chutePosition_ = moveTowards(chutePosition_, chutePositionCommand_, 0.30F);
    deflectorPosition_ = moveTowards(
        deflectorPosition_,
        deflectorPositionCommand_,
        0.30F);

    dcBusCurrent_ = baseBusCurrent;
    batteryVoltage_ = std::clamp(
        50.8F - (1.0F - batterySoc_) * 4.0F - dcBusCurrent_ * 0.035F,
        43.0F,
        50.8F);

    const bool augerResetRequired = augerFaultLatched_;
    const std::string augerStatusText =
        buildAugerStatusText(
            augerModeActive,
            engineRunning,
            throttleReadyForAuger,
            batteryReadyForAuger,
            augerResetRequired,
            effectiveAugerEngaged);

    publishHybridTopics(
        engineRunning,
        effectiveAugerEngaged,
        tractionPowerLimit,
        augerInterlockOk,
        augerResetRequired,
        augerStatusText);

    resetAugerFaultRequested_ = false;

    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        3000,
        "mode=%s profile=%s engine=%s rpm=%.0f throttle=%.2f auger=%s auger_rpm=%.0f overload=%s fault=%s interlock=%s battery=%.0f%% voltage=%.1fV traction_limit=%.2f chute=%.2f deflector=%.2f status=%s",
        workMode_.c_str(),
        toolProfile_.c_str(),
        engineRunning ? "on" : "off",
        engineRpm_,
        engineThrottleCommand_,
        effectiveAugerEngaged ? "engaged" : "off",
        augerRpm_,
        simulateAugerOverload_ ? "true" : "false",
        augerFaultLatched_ ? "true" : "false",
        augerInterlockOk ? "true" : "false",
        batterySoc_ * 100.0F,
        batteryVoltage_,
        tractionPowerLimit,
        chutePosition_,
        deflectorPosition_,
        augerStatusText.c_str());
  }

  void publishHybridTopics(
      bool engineRunning,
      bool effectiveAugerEngaged,
      float tractionPowerLimit,
      bool augerInterlockOk,
      bool augerResetRequired,
      const std::string &augerStatusText)
  {
    std_msgs::msg::Bool engineRunningMsg;
    engineRunningMsg.data = engineRunning;
    engineRunningPublisher_->publish(engineRunningMsg);

    std_msgs::msg::Float32 engineRpmMsg;
    engineRpmMsg.data = engineRpm_;
    engineRpmPublisher_->publish(engineRpmMsg);

    std_msgs::msg::Float32 batterySocMsg;
    batterySocMsg.data = batterySoc_;
    batterySocPublisher_->publish(batterySocMsg);

    std_msgs::msg::Float32 batteryVoltageMsg;
    batteryVoltageMsg.data = batteryVoltage_;
    batteryVoltagePublisher_->publish(batteryVoltageMsg);

    std_msgs::msg::Float32 dcBusCurrentMsg;
    dcBusCurrentMsg.data = dcBusCurrent_;
    dcBusCurrentPublisher_->publish(dcBusCurrentMsg);

    std_msgs::msg::Float32 tractionPowerLimitMsg;
    tractionPowerLimitMsg.data = tractionPowerLimit;
    tractionPowerLimitPublisher_->publish(tractionPowerLimitMsg);

    std_msgs::msg::Bool augerEngagedMsg;
    augerEngagedMsg.data = effectiveAugerEngaged;
    augerEngagedPublisher_->publish(augerEngagedMsg);

    std_msgs::msg::Float32 augerRpmMsg;
    augerRpmMsg.data = augerRpm_;
    augerRpmPublisher_->publish(augerRpmMsg);

    std_msgs::msg::Bool augerOverloadMsg;
    augerOverloadMsg.data = simulateAugerOverload_ && effectiveAugerEngaged;
    augerOverloadPublisher_->publish(augerOverloadMsg);

    std_msgs::msg::Bool augerInterlockOkMsg;
    augerInterlockOkMsg.data = augerInterlockOk;
    augerInterlockOkPublisher_->publish(augerInterlockOkMsg);

    std_msgs::msg::Bool augerFaultLatchedMsg;
    augerFaultLatchedMsg.data = augerFaultLatched_;
    augerFaultLatchedPublisher_->publish(augerFaultLatchedMsg);

    std_msgs::msg::Bool augerResetRequiredMsg;
    augerResetRequiredMsg.data = augerResetRequired;
    augerResetRequiredPublisher_->publish(augerResetRequiredMsg);

    std_msgs::msg::String augerStatusTextMsg;
    augerStatusTextMsg.data = augerStatusText;
    augerStatusTextPublisher_->publish(augerStatusTextMsg);

    std_msgs::msg::Float32 chutePositionMsg;
    chutePositionMsg.data = chutePosition_;
    chutePositionPublisher_->publish(chutePositionMsg);

    std_msgs::msg::Float32 deflectorPositionMsg;
    deflectorPositionMsg.data = deflectorPosition_;
    deflectorPositionPublisher_->publish(deflectorPositionMsg);
  }

  std::string buildAugerStatusText(
      bool augerModeActive,
      bool engineRunning,
      bool throttleReadyForAuger,
      bool batteryReadyForAuger,
      bool augerResetRequired,
      bool effectiveAugerEngaged) const
  {
    if (emergencyStopActive_)
    {
      return "emergency_stop_active";
    }

    if (augerResetRequired)
    {
      return "fault_latched_reset_required";
    }

    if (!augerModeActive)
    {
      return "switch_to_snow_clearing_auger_profile";
    }

    if (!engineRunning)
    {
      return "engine_off";
    }

    if (!throttleReadyForAuger)
    {
      return "increase_engine_throttle_before_engage";
    }

    if (!batteryReadyForAuger)
    {
      return "battery_voltage_low";
    }

    if (effectiveAugerEngaged)
    {
      return "auger_engaged";
    }

    if (augerEngagedCommand_)
    {
      return "waiting_for_interlock";
    }

    return "ready_to_engage";
  }

  std::string normalizeProfile(const std::string &value) const
  {
    if (value == "blade" || value == "auger")
    {
      return value;
    }

    return "auger";
  }

  std::string toolProfile_ = "auger";
  std::string workMode_ = "manual_drive";

  bool engineEnabledCommand_ = false;
  bool augerEngagedCommand_ = false;
  bool emergencyStopActive_ = false;
  bool simulateAugerOverload_ = false;
  bool resetAugerFaultRequested_ = false;
  bool augerFaultLatched_ = false;

  float engineThrottleCommand_ = 0.0F;
  float chutePositionCommand_ = 0.0F;
  float deflectorPositionCommand_ = 0.0F;

  float lastLinearCommand_ = 0.0F;
  float lastAngularCommand_ = 0.0F;

  float engineRpm_ = 0.0F;
  float augerRpm_ = 0.0F;
  float batterySoc_ = 0.92F;
  float batteryVoltage_ = 50.0F;
  float dcBusCurrent_ = 0.0F;
  float chutePosition_ = 0.0F;
  float deflectorPosition_ = 0.0F;
  bool hasLastUpdateTime_ = false;
  rclcpp::Time lastUpdateTime_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
      cmdVelSubscription_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
      profileSubscription_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
      enabledSubscription_;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr
      powerSubscription_;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr
      angleSubscription_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
      workModeSubscription_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
      emergencyStopSubscription_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
      engineEnabledSubscription_;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr
      engineThrottleSubscription_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
      augerEngagedSubscription_;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr
      chutePositionSubscription_;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr
      deflectorPositionSubscription_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
      augerOverloadSubscription_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
      resetAugerFaultSubscription_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
      engineRunningPublisher_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr
      engineRpmPublisher_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr
      batterySocPublisher_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr
      batteryVoltagePublisher_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr
      dcBusCurrentPublisher_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr
      tractionPowerLimitPublisher_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
      augerEngagedPublisher_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr
      augerRpmPublisher_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
      augerOverloadPublisher_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
      augerInterlockOkPublisher_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
      augerFaultLatchedPublisher_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
      augerResetRequiredPublisher_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
      augerStatusTextPublisher_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr
      chutePositionPublisher_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr
      deflectorPositionPublisher_;

  rclcpp::TimerBase::SharedPtr
      statusTimer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AutomowerToolController>());
  rclcpp::shutdown();
  return 0;
}