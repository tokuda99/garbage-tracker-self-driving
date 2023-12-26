#include <micro_ros_arduino.h>
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>


#include <MP.h>

/* multi core macro */
#define REQ_IMU  100
#define POST_IMU  101
#define REQ_CMD  102
#define POST_CMD  103
#define REQ_ODOM  104
#define POST_ODOM  105

// health check function for 
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(LED0, !digitalRead(LED0));
    delay(100);
  }
}

struct Imu {
  float ang_vel_x;
  float ang_vel_y;
  float ang_vel_z;
  float lin_acc_x;
  float lin_acc_y;
  float lin_acc_z;
};
struct Odometry {
  float ang_z;
  float pos_x;
  float pos_y;
  float qt_qz;
  float qt_qw;
};

rcl_allocator_t allocator;
static rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;
static rcl_timer_t timer;

rcl_publisher_t imu_publisher;
rcl_subscription_t cmd_subscriber;  // command subscriber
// static rcl_publisher_t odom_publisher;   // odometry publisher

sensor_msgs__msg__Imu imu_msg;    // imu message
geometry_msgs__msg__Twist cmd_vel_msg;    // command message
nav_msgs__msg__Odometry odom_msg;          // odometry message

const int subcore_imu = 1;
const int subcore_wheel = 2;

void cmd_vel_callback(const void * msgin) {
  geometry_msgs__msg__Twist* cmd_msg = (geometry_msgs__msg__Twist*)msgin;
  MPLog("cmd_vel_callback\n"); 
  static geometry_msgs__msg__Twist cmdout;
  memcpy(&cmdout, cmd_msg, sizeof(geometry_msgs__msg__Twist));
  int8_t sndid = 102;
  int ret = MP.Send(sndid, &cmdout, subcore_wheel);
  if (ret < 0) {
    MPLog("MP.Send cmd error = %d\n", ret);
  }
}


void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {  
  RCLC_UNUSED(last_call_time);
  int8_t sndid;
  int snd_empty = 0;
  int8_t recvid;
  int ret_rec;
  Imu* imu_;
  Odometry* odom_;
  if (timer != NULL) {
    // get imu data from subcore-1
    sndid = 100;
    MP.Send(sndid, snd_empty, subcore_imu);
    ret_rec = MP.Recv(&recvid, &imu_, subcore_imu);
    if (ret_rec>=0 && recvid==101) {
      imu_msg.angular_velocity.x = imu_->ang_vel_x;
      imu_msg.angular_velocity.y = imu_->ang_vel_y;
      imu_msg.angular_velocity.z = imu_->ang_vel_z;
      imu_msg.linear_acceleration.x = imu_->lin_acc_x;
      imu_msg.linear_acceleration.y = imu_->lin_acc_y;
      imu_msg.linear_acceleration.z = imu_->lin_acc_z;
    }

    // // get odometry data from subcore-2
    // sndid = 103;
    // MP.Send(sndid, snd_empty, subcore_wheel);
    // ret_rec = MP.Recv(&recvid, &odom_, subcore_wheel);
    // if (ret_rec>=0 && recvid==104) {
    //   odom_msg.twist.twist.angular.z = odom_->ang_z;
    //   odom_msg.pose.pose.position.x = odom_->pos_x;
    //   odom_msg.pose.pose.position.y = odom_->pos_y;
    //   odom_msg.pose.pose.orientation.z = odom_->qt_qz;
    //   odom_msg.pose.pose.orientation.w = odom_->qt_qw;
    // }
    // add timestamp
    // uint32_t current_time_in_micros = micros();
    // imu_msg.header.stamp.sec = current_time_in_micros/1000000;
    // imu_msg.header.stamp.nanosec = (current_time_in_micros - (imu_msg.header.stamp.sec)*1000000)*1000; 

    // odom_msg->header.stamp.sec = current_time_in_micros/1000000;
    // odom_msg->header.stamp.nanosec = (current_time_in_micros - (odom_msg->header.stamp.sec)*1000000)*1000; 

    // publish Imu
    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));

    // publish odometry
    // RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
  }
}

void setup() {
  set_microros_transports(); 
  delay(2000);
  

  MPLog("Initialize micro-ROS\n");
  // init ros system
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));   

  MPLog("create node <rover_node>\n");
  RCCHECK(rclc_node_init_default(&node, "rover_node", "", &support));

  MPLog("Create subscriber for topic <cmd_vel> \n");
  RCCHECK(rclc_subscription_init_default(
    &cmd_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  MPLog("create publisher for topic <imu> \n");
  RCCHECK(rclc_publisher_init_default(
    &imu_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu"));

  // MPLog("create publisher for odometry <odom> \n");
  // RCCHECK(rclc_publisher_init_default(
  //   &odom_publisher,
  //   &node,
  //   ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
  //   "odom"));

  // crate Imu msgs
  static char imu[5] = {0};
  imu_msg.header.frame_id.data = imu;
  sprintf(imu_msg.header.frame_id.data, "imu");
  imu_msg.header.frame_id.size = 3;
  imu_msg.header.frame_id.capacity=5;

  // // crate Odemetry msgs
  // static char odom[5] = {0};
  // odom_msg.header.frame_id.data = odom;
  // sprintf(odom_msg.header.frame_id.data, "odom");
  // odom_msg.header.frame_id.size = 4;
  // odom_msg.header.frame_id.capacity=5;

  // create timer,
  MPLog("create timer \n");
  const uint32_t timer_timeout = 100;
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback));

  MPLog("initialize rclc_executor\n");  
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_subscriber, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  delay(1000);

  MPLog("Subcore start\n");
  MP.begin(subcore_imu);
  MP.begin(subcore_wheel);
  delay(2000);
  digitalWrite(LED0, HIGH);
  digitalWrite(LED3, HIGH);
}

void loop() {
  delay(10);
  // periodic execution
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  digitalWrite(LED3, !digitalRead(LED3));
}
