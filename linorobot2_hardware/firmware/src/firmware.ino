// Modified by Kevin Lee (9/28)
// Based on firmware.ino.1&2&3
//  - publisher: odometry, imu, range1/data, range2/data (all worked)
//  - subscriber: twist, motor_brake (all worked)
//  - when issuing twist cmd_vel topic, motor can run smoothly while publishing all topics

#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/range.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>
#include <std_msgs/msg/int32.h>

#include "config.h"
#include "motor.h"
#include "kinematics.h"
#include "pid.h"
#include "odometry.h"
#include "imu.h"
#include "hcsr04.h"
#define ENCODER_USE_INTERRUPTS
#define ENCODER_OPTIMIZE_INTERRUPTS
#include "encoder.h"
#include "FIT_PowerManagement.h"
#include "debugmessage.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){rclErrorLoop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)

rcl_publisher_t odom_publisher;
rcl_publisher_t imu_publisher;
// rcl_publisher_t range_publisher;
rcl_publisher_t range1_publisher;
rcl_publisher_t range2_publisher;
rcl_subscription_t twist_subscriber;
rcl_subscription_t custom_subscriber;
rcl_subscription_t led_subscriber;

nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__Imu imu_msg;
// sensor_msgs__msg__Range range_msg;
sensor_msgs__msg__Range range1_msg;
sensor_msgs__msg__Range range2_msg;
geometry_msgs__msg__Twist twist_msg;
std_msgs__msg__Int32 custom_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;

unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
unsigned long prev_odom_update = 0;

enum states
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

Encoder motor1_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, COUNTS_PER_REV1, MOTOR1_ENCODER_INV);
Encoder motor2_encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B, COUNTS_PER_REV2, MOTOR2_ENCODER_INV);
// Encoder motor3_encoder(MOTOR3_ENCODER_A, MOTOR3_ENCODER_B, COUNTS_PER_REV3, MOTOR3_ENCODER_INV);
// Encoder motor4_encoder(MOTOR4_ENCODER_A, MOTOR4_ENCODER_B, COUNTS_PER_REV4, MOTOR4_ENCODER_INV);

Motor motor1_controller(PWM_FREQUENCY, PWM_BITS, MOTOR1_INV, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
Motor motor2_controller(PWM_FREQUENCY, PWM_BITS, MOTOR2_INV, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B);
// Motor motor3_controller(PWM_FREQUENCY, PWM_BITS, MOTOR3_INV, MOTOR3_PWM, MOTOR3_IN_A, MOTOR3_IN_B);
// Motor motor4_controller(PWM_FREQUENCY, PWM_BITS, MOTOR4_INV, MOTOR4_PWM, MOTOR4_IN_A, MOTOR4_IN_B);

PID motor1_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor2_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
// PID motor3_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
// PID motor4_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

FITPM PM(LM5066I_SLAVE_ADDR,LM5066I_I2C_SPEED,BTT6030_IS_L_PIN,BTT6030_IS_R_PIN,BTT6030_IN_L_PIN,BTT6030_IN_R_PIN);

Kinematics kinematics(
    Kinematics::LINO_BASE,
    MOTOR_MAX_RPM,
    MAX_RPM_RATIO,
    MOTOR_OPERATING_VOLTAGE,
    MOTOR_POWER_MAX_VOLTAGE,
    WHEEL_DIAMETER,
    LR_WHEELS_DISTANCE
);

Odometry odometry;
IMU imu;
// HCSR04 range;
HCSR04 range1("sonic1");
HCSR04 range2("sonic2");
bool in_brake = false;

void setup()
{
    pinMode(LED_PIN, OUTPUT);

    bool imu_ok = imu.init();
    int trigPin1 = 25;
    int echoPin1 = 24;
    int trigPin2 = 6;
    int echoPin2 = 5;
    bool range1_ok = range1.init(trigPin1, echoPin1);
    bool range2_ok = range2.init(trigPin2, echoPin2);

    PM.begin();
    PM.registBatteryPercentageCallBack(BatteryCallback);
    PM.led_setup(NUMBER_OF_LED_L,NUMBER_OF_LED_R,LED_PIN_L,LED_PIN_R,NEO_GRB + NEO_KHZ800);

    // if(!imu_ok)
    // {
    //     while(1)
    //     {
    //         flashLED(3);
    //     }
    // }
    Serial.begin(115200);
    set_microros_serial_transports(Serial);
}

void loop() {
    switch (state)
    {
        case WAITING_AGENT:
            dbg_printf("WAITING_AGENT");
            EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
            break;
        case AGENT_AVAILABLE:
            dbg_printf("AGENT_AVAILABLE");
            state = (true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
            if (state == WAITING_AGENT)
            {
                destroyEntities();
            }
            break;
        case AGENT_CONNECTED:
            dbg_printf("AGENT_CONNECTED");
            EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
            if (state == AGENT_CONNECTED)
            {
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
            }
            break;
        case AGENT_DISCONNECTED:
            dbg_printf("AGENT_DISCONNECTED");
            destroyEntities();
            state = WAITING_AGENT;
            break;
        default:
            break;
    }
    // PM.processPowerManagement();
}

void controlCallback(rcl_timer_t * timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
       // moveBase();
       // publishDataTest();
       PM.processLM5066I();
       PM.processBTT6030();
       PM.updateBatteryPercentage();
       PM.process_led();
    }
}

void twistCallback(const void * msgin)
{
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));

    prev_cmd_time = millis();
}

void motorBrakeCallback(const void * msgin)
{
  // example ros client usage: ros2 topic pub -1 /motor_brake std_msgs/msg/Int32 "{data: 2}"
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  digitalWrite(LED_PIN, (msg->data == 0) ? LOW : HIGH);
  if (msg->data ==0) {
    in_brake = true;
    fullStop();
  } else {
    in_brake = false;
  }
}

void ledStatusCallback(const void * msgin)
{
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  switch (msg->data)
  {
      case LED_STATES_OFF:
          dbg_printf("LED_STATES_OFF");
          PM.setLedStatus(LED_STATES_OFF);
          break;
      case LED_STATES_DRIVING:
          dbg_printf("LED_STATES_DRIVING");
          PM.setLedStatus(LED_STATES_DRIVING);
          break;
      case LED_STATES_REVERSE:
          dbg_printf("LED_STATES_REVERSE");
          PM.setLedStatus(LED_STATES_REVERSE);
          break;
      case LED_STATES_TURN_L:
          dbg_printf("LED_STATES_TURN_L");
          PM.setLedStatus(LED_STATES_TURN_L);
          break;
      case LED_STATES_TURN_R:
          dbg_printf("LED_STATES_TURN_R");
          PM.setLedStatus(LED_STATES_TURN_R);
          break;
      case LED_STATES_CAUTION_ZONE:
          dbg_printf("LED_STATES_CAUTION_ZONE");
          PM.setLedStatus(LED_STATES_CAUTION_ZONE);
          break;
      case LED_STATES_STANDBY:
          dbg_printf("LED_STATES_STANDBY");
          PM.setLedStatus(LED_STATES_STANDBY);
          break;
      case LED_STATES_FAULT:
          dbg_printf("LED_STATES_FAULT");
          PM.setLedStatus(LED_STATES_FAULT);
          break;
      case LED_STATES_CHARGING:
          dbg_printf("LED_STATES_CHARGING");
          PM.setLedStatus(LED_STATES_CHARGING);
          break;
      case LED_STATES_BATTERY_LOW:
          dbg_printf("LED_STATES_BATTERY_LOW");
          PM.setLedStatus(LED_STATES_BATTERY_LOW);
          break;
      default:
          break;
  }
}

bool createEntities()
{
    allocator = rcl_get_default_allocator();
    //create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    // create node
    RCCHECK(rclc_node_init_default(&node, "linorobot_base_node", "", &support));
    // create odometry publisher
    RCCHECK(rclc_publisher_init_default(
        &odom_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom/unfiltered"
    ));
    // create IMU publisher
    RCCHECK(rclc_publisher_init_default(
        &imu_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/data"
    ));
    // create range publisher
    // RCCHECK(rclc_publisher_init_default(
    //     &range1_publisher,
    //     &node,
    //     ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
    //     "range1/data"
    // ));
    // RCCHECK(rclc_publisher_init_default(
    //     &range2_publisher,
    //     &node,
    //     ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
    //     "range2/data"
    // ));
    // create motro_brake command subscriber
    RCCHECK(rclc_subscription_init_default(
        &custom_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "motor_brake"
    ));
    // create twist command subscriber
    RCCHECK(rclc_subscription_init_default(
        &twist_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"
    ));
    // create led command subscriber
    RCCHECK(rclc_subscription_init_default(
        &led_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "led_status"
    ));
    // create timer for actuating the motors at 50 Hz (1000/20)
    const unsigned int control_timeout = 20;
    RCCHECK(rclc_timer_init_default(
        &control_timer,
        &support,
        RCL_MS_TO_NS(control_timeout),
        controlCallback
    ));
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 4, & allocator));
    // RCCHECK(rclc_executor_init(&executor, &support.context, 3, & allocator));
    // RCCHECK(rclc_executor_init(&executor, &support.context, 2, & allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &custom_subscriber,
        &custom_msg,
        &motorBrakeCallback,
        ON_NEW_DATA
    ));
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &led_subscriber,
        &custom_msg,
        &ledStatusCallback,
        ON_NEW_DATA
    ));
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &twist_subscriber,
        &twist_msg,
        &twistCallback,
        ON_NEW_DATA
    ));
    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));

    // synchronize time with the agent
    syncTime();
    digitalWrite(LED_PIN, HIGH);

    return true;
}

bool destroyEntities()
{
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_publisher_fini(&odom_publisher, &node);
    rcl_publisher_fini(&imu_publisher, &node);
    rcl_subscription_fini(&twist_subscriber, &node);
    //
    // rcl_publisher_fini(&range1_publisher, &node);
    // rcl_publisher_fini(&range2_publisher, &node);
    rcl_subscription_fini(&custom_subscriber, &node);
    rcl_subscription_fini(&led_subscriber, &node);
    //
    rcl_node_fini(&node);
    rcl_timer_fini(&control_timer);
    rclc_executor_fini(&executor);
    rclc_support_fini(&support);

    digitalWrite(LED_PIN, HIGH);

    return true;
}

void fullStop()
{
    twist_msg.linear.x = 0.0;
    twist_msg.linear.y = 0.0;
    twist_msg.angular.z = 0.0;

    motor1_controller.brake();
    motor2_controller.brake();
    // motor3_controller.brake();
    // motor4_controller.brake();
}

void moveBase()
{
    // brake if there's no command received, or when it's only the first command sent
    if(in_brake || ((millis() - prev_cmd_time) >= 200))
    {
        twist_msg.linear.x = 0.0;
        twist_msg.linear.y = 0.0;
        twist_msg.angular.z = 0.0;

        digitalWrite(LED_PIN, HIGH);
    }
    // get the required rpm for each motor based on required velocities, and base used
    Kinematics::rpm req_rpm = kinematics.getRPM(
        twist_msg.linear.x,
        twist_msg.linear.y,
        twist_msg.angular.z
    );

    // get the current speed of each motor
    float current_rpm1 = motor1_encoder.getRPM();
    float current_rpm2 = motor2_encoder.getRPM();
    // float current_rpm3 = motor3_encoder.getRPM();
    // float current_rpm4 = motor4_encoder.getRPM();
    float current_rpm3 = 0.0;
    float current_rpm4 = 0.0;

    // the required rpm is capped at -/+ MAX_RPM to prevent the PID from having too much error
    // the PWM value sent to the motor driver is the calculated PID based on required RPM vs measured RPM
    motor1_controller.spin(motor1_pid.compute(req_rpm.motor1, current_rpm1));
    motor2_controller.spin(motor2_pid.compute(req_rpm.motor2, current_rpm2));
    // motor3_controller.spin(motor3_pid.compute(req_rpm.motor3, current_rpm3));
    // motor4_controller.spin(motor4_pid.compute(req_rpm.motor4, current_rpm4));

    Kinematics::velocities current_vel = kinematics.getVelocities(
        current_rpm1,
        current_rpm2,
        current_rpm3,
        current_rpm4
    );

    unsigned long now = millis();
    float vel_dt = (now - prev_odom_update) / 1000.0;
    prev_odom_update = now;
    odometry.update(
        vel_dt,
        current_vel.linear_x,
        current_vel.linear_y,
        current_vel.angular_z
    );
}

void publishDataTest()
{
    odom_msg = odometry.getData();
    imu_msg = imu.getData();
    range1_msg = range1.getData();
    range2_msg = range2.getData();
    struct timespec time_stamp = getTime();
    odom_msg.header.stamp.sec = time_stamp.tv_sec;
    odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;
    imu_msg.header.stamp.sec = time_stamp.tv_sec;
    imu_msg.header.stamp.nanosec = time_stamp.tv_nsec;
    range1_msg.header.stamp.sec = time_stamp.tv_sec;
    range1_msg.header.stamp.nanosec = time_stamp.tv_nsec;
    range2_msg.header.stamp.sec = time_stamp.tv_sec;
    range2_msg.header.stamp.nanosec = time_stamp.tv_nsec;
    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
    RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
    RCSOFTCHECK(rcl_publish(&range1_publisher, &range1_msg, NULL));
    RCSOFTCHECK(rcl_publish(&range2_publisher, &range2_msg, NULL));
}

void publishData()
{
    odom_msg = odometry.getData();
    imu_msg = imu.getData();

    struct timespec time_stamp = getTime();

    odom_msg.header.stamp.sec = time_stamp.tv_sec;
    odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    imu_msg.header.stamp.sec = time_stamp.tv_sec;
    imu_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
    RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
}

void syncTime()
{
    // get the current time from the agent
    unsigned long now = millis();
    RCCHECK(rmw_uros_sync_session(10));
    unsigned long long ros_time_ms = rmw_uros_epoch_millis();
    // now we can find the difference between ROS time and uC time
    time_offset = ros_time_ms - now;
}

struct timespec getTime()
{
    struct timespec tp = {0};
    // add time difference between uC time and ROS time to
    // synchronize time with ROS
    unsigned long long now = millis() + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;

    return tp;
}

void rclErrorLoop()
{
    while(true)
    {
        flashLED(2);
    }
}

void flashLED(int n_times)
{
    for(int i=0; i<n_times; i++)
    {
        digitalWrite(LED_PIN, HIGH);
        delay(150);
        digitalWrite(LED_PIN, LOW);
        delay(150);
    }
    delay(1000);
}

void BatteryCallback(int percentage)
{
  dbg_printf("Battery percentage:%d\r\n",percentage);
}

