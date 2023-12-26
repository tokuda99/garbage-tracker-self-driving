#if (SUBCORE != 2)
#error "Core selection is wrong!!"
#endif

/* multi core lib */
#include <MP.h>

/* ros lib */
#include <micro_ros_arduino.h>
#include <geometry_msgs/msg/twist.h>

/* multi core macro */

/* motor macro */
#define R_ENCODER_OUT_PIN 4 //?
#define L_ENCODER_OUT_PIN 6 //?

const int right_encoderPinA = 4; // エンコーダーのA相ピン
// const int right_encoderPinB = 5; // エンコーダーのB相ピン
const int left_encoderPinA = 6; // エンコーダーのA相ピン
// const int left_encoderPinB = 7; // エンコーダーのB相ピン

#define TORQUE_COMP_POWER 255
#define TORQUE_COMP_TIME_R 5
#define TORQUE_COMP_TIME_L 5

/* loop macro */
#define DELAY_TIME 40

/* multi core param */
int subcore = 2;

/* Use CMSIS library */
#define ARM_MATH_CM4
#define __FPU_PRESENT 1U
#include <arm_math.h>

/* motor param & module*/
// wheel rotation count
volatile uint32_t R = 0;
volatile uint32_t L = 0;
void Encoder0() {  ++R; } // interrupt handling
void Encoder1() {  ++L; } // interrupt handling

void rMotorControl(uint8_t status, int32_t duty) { //0:stop 1:forward 2: backward
  if (status == 0) {
    digitalWrite(9,HIGH);
    analogWrite(3,0);
    return;
  }
  if (status == 1) {
    digitalWrite(12,LOW);
    analogWrite(3,duty); //255->duty
  }
  else if (status == 2) {
    digitalWrite(12,HIGH);
    analogWrite(3,duty); //255->duty
  }
  return;
}

void lMotorControl(uint8_t status, int32_t duty) { //0:stop 1:forward 2: backward
  if (status == 0) {
    digitalWrite(8,LOW);
    analogWrite(11,0);
    return;
  }
  if (status == 1) {
    digitalWrite(13,HIGH);
    analogWrite(11,duty); //255->duty
  }
  else if (status == 2) {
    digitalWrite(13,LOW);
    analogWrite(11,duty); //255->duty
  }
  return;
}

float calc_speed(uint32_t enc_count, uint32_t duration_ms, float* mileage, float target) {
  static const float r  = 0.042; // tire radius (m)
  static const float enc_theta = 0.45;  // a unit degree of the edge encoder 
  if (duration_ms == 0.0) { *mileage = 0.0; return 0.0; }
  float rotation_ = enc_count*enc_theta; // rotation (degree) 
  float mileage_ = PI*r*rotation_/180.0; // convert degree to radian
  float rov_speed_ = mileage_*1000.0/(float)(duration_ms); 
  //Serial.print(String(enc_count) + "," + String(rotation, 6) + "," + String(mileage, 6) + "," + String(rov_speed) + ",");
  *mileage = mileage_;
  if (target < 0) rov_speed_ = -rov_speed_;
  return rov_speed_;
}

// PID control
float R_Kp = 200.0; //?
float R_Ki = 0.0; //?
float R_Kd = 0.0; //?
float L_Kp = 200.0; //?
float L_Ki = 0.0; //?
float L_Kd = 0.0; //?
float VRt =  0.0;
float VLt =  0.0;

int32_t R_duty = 0;
int32_t L_duty = 0;

/* odometry param */
struct Odometry {
  float ang_z;
  float pos_x;
  float pos_y;
  float qt_qz;
  float qt_qw;
};
struct Odometry wheel_odom;


void setup() {
  /* multi core init */
  MP.begin();
  MP.RecvTimeout(MP_RECV_POLLING);

  /* motor init */
  Serial.begin(115200);
  // left motor
  pinMode(13,OUTPUT);
  digitalWrite(13,LOW);
  pinMode(11,OUTPUT);
  digitalWrite(11,LOW);
  pinMode(8,OUTPUT);
  digitalWrite(8,LOW);

  // right motor
  pinMode(12,OUTPUT);
  digitalWrite(12,LOW);
  pinMode(3,OUTPUT);
  digitalWrite(3,LOW);
  pinMode(9,OUTPUT);
  digitalWrite(9,LOW);

  // right encoder
  pinMode(right_encoderPinA, INPUT);
  // pinMode(right_encoderPinB, INPUT);

  // left encoder
	pinMode(left_encoderPinA, INPUT);
  // pinMode(left_encoderPinB, INPUT);
  
  attachInterrupt(right_encoderPinA, Encoder0, CHANGE);
  attachInterrupt(left_encoderPinA, Encoder1, CHANGE);

  memset(&wheel_odom, 0, sizeof(struct Odometry));
  delay(1000);
  digitalWrite(LED2, HIGH);
}

void loop() {
  static uint32_t start_time = 0;
  static uint32_t last_time = 0;
  static float R_last_err = 0.0;
  static float L_last_err = 0.0;
  static float R_err_integ = 0.0;
  static float L_err_integ = 0.0;  
  static const float d = 0.165; 

  float R_Vm, L_Vm;  // actual speeds calcurated by encorders

  int8_t recvid;
  void* msgin;
  int ret_rec = MP.Recv(&recvid, &msgin);
  if (ret_rec >=0) {
    if (recvid == 102) { //POST_CMD
      geometry_msgs__msg__Twist* cmd_vel_msg = (geometry_msgs__msg__Twist*)msgin;
      VRt = cmd_vel_msg->linear.x + cmd_vel_msg->angular.z*d;
      VLt = cmd_vel_msg->linear.x - cmd_vel_msg->angular.z*d;
      start_time = millis();
    }
    // else if (recvid == 103) { //REQ_ODOM
    //   int8_t sndid = 104; //RES_ODOM
    //   int ret_snd = MP.Send(sndid, &wheel_odom);
    // }
  }

  uint32_t current_time = millis();
  uint32_t duration = current_time - last_time; 
  last_time = current_time;  

  float R_err  = 0.0;
  float L_err  = 0.0;
  float R_derr = 0.0;
  float L_derr = 0.0;

  noInterrupts();
  uint32_t cur_R = R; R = 0; 
  uint32_t cur_L = L; L = 0;
  interrupts();
  float R_mileage = 0.0;
  float L_mileage = 0.0;

  R_Vm = calc_speed(cur_R, duration, &R_mileage, VRt);
  L_Vm = calc_speed(cur_L, duration, &L_mileage, VLt);

  if (abs(R_Vm) > 0.0 || abs(L_Vm) > 0.0) {
    // calc odometry
    static float odm_ang_z = 0.0;
    static float odm_pos_x = 0.0;
    static float odm_pos_y = 0.0;
    static float odm_qt_qz = 0.0;
    static float odm_qt_qw = 0.0;

    float duration_sec = (float)duration/1000;
    float last_odm_ang_z = odm_ang_z;
    odm_ang_z += (R_Vm - L_Vm)*duration_sec/(2.*d);
    if (odm_ang_z > 2.*PI) odm_ang_z -= 2.*PI;
    odm_pos_x += (R_Vm + L_Vm)*duration_sec/2.*arm_cos_f32(last_odm_ang_z+odm_ang_z/2);
    odm_pos_y += (R_Vm + L_Vm)*duration_sec/2.*arm_sin_f32(last_odm_ang_z+odm_ang_z/2);
    odm_qt_qz = arm_sin_f32(odm_ang_z/2) - arm_cos_f32(odm_ang_z/2);
    odm_qt_qw = arm_cos_f32(odm_ang_z/2) + arm_sin_f32(odm_ang_z/2);
  
    wheel_odom.ang_z = odm_ang_z;
    wheel_odom.pos_x = odm_pos_x;
    wheel_odom.pos_y = odm_pos_y;
    wheel_odom.qt_qz = odm_qt_qz;
    wheel_odom.qt_qw = odm_qt_qw; 
  }

  float duration_sec = duration/1000.0;
  R_err        = VRt - R_Vm;
  L_err        = VLt - L_Vm;
  R_err_integ += (R_err + R_last_err)*0.5*duration_sec;
  L_err_integ += (L_err + L_last_err)*0.5*duration_sec;
  R_derr       = (R_err - R_last_err)/duration_sec;  
  L_derr       = (L_err - L_last_err)/duration_sec;

  R_duty = (int32_t)(R_Kp*R_err + R_Ki*R_err_integ + R_Kd*R_derr);
  L_duty = (int32_t)(L_Kp*L_err + L_Ki*L_err_integ + L_Kd*L_derr);

  if (abs(R_duty) > 255) {
    if (R_duty > 255)    R_duty = +255;
    else if (R_duty < 0) R_duty = -255;
  }
  if (abs(L_duty) > 255) {
    if (L_duty > 255)    L_duty = +255;
    else if (L_duty < 0) L_duty = -255;
  }
  R_last_err = R_err;
  L_last_err = L_err;

  // if (R_Vm == 0.0 && VRt != 0.0) {
  //   // Torque Compensation
  //   if (VRt > 0.0) {
  //     rMotorControl(1, TORQUE_COMP_POWER); // forward
  //   } else if (VRt < 0.0) {
  //     rMotorControl(2, TORQUE_COMP_POWER); // backward
  //   }
  //   delay(TORQUE_COMP_TIME_R);
  // };

  if (VRt > 0.0) {
    rMotorControl(1, abs(R_duty)); // forward
    delay(10);
  } else if (VRt < 0.0) {
    rMotorControl(2, abs(R_duty)); // backward
    delay(10);

  } else if (VRt == 0.0) {
    rMotorControl(0, 0); // backward
    delay(10);
  }

  // if (L_Vm == 0.0 && VLt != 0.0) {
  //   // Torque Compensation
  //   if (VLt > 0.0) {
  //     lMotorControl(1, TORQUE_COMP_POWER); // forward
  //   } else if (VLt < 0.0) {
  //     lMotorControl(2, TORQUE_COMP_POWER); // backward
  //   }
  //   delay(TORQUE_COMP_TIME_L);
  // };  

  if (VLt > 0.0) {
    lMotorControl(1, abs(L_duty)); // forward
    delay(10);
  } else if (VLt < 0.0) {
    lMotorControl(2, abs(L_duty)); // backward
    delay(10);
  } else if (VLt == 0.0) {
    lMotorControl(0, 0); // backward
    delay(10);

  }

}
