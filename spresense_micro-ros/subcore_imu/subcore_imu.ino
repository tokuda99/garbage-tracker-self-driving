#if (SUBCORE != 1)
#error "Core selection is wrong!!"
#endif

/* multi core lib */
#include <MP.h>

/* ros lib */
#include <micro_ros_arduino.h>
#include <sensor_msgs/msg/imu.h>

/* imu lib */
#include <BMI160Gen.h>

/* multi core macro */
#define REQ_IMU  100

/* imu macro */
#define BAUDRATE  115200
#define SENSE_RATE   200
#define GYRO_RANGE   250
#define ACCL_RANGE     2
#define COUNT_FOR_CALIB 1024
#define deg_to_rad(a) (a/180*M_PI)
#define rad_to_deg(a) (a/M_PI*180)

/* multi core param */
int subcore = 1;

/* imu param & module*/
static float offset_acc_x = 0, offset_acc_y = 0, offset_acc_z = 0;   
float convertRawGyro(int gRaw) 
{
  // ex) if the range is +/-500 deg/s: +/-32768/500 = +/-65.536 LSB/(deg/s)
  float lsb_omega = float(0x7FFF) / GYRO_RANGE;
  return gRaw / lsb_omega;  // deg/sec
}
float convertRawAccel(int aRaw) 
{
  // ex) if the range is +/-2g ; +/-32768/2 = +/-16384 LSB/g
  float lsb_g = float(0x7FFF) / ACCL_RANGE;
  return aRaw / lsb_g;
}
struct ImuMsg {
  float ang_vel_x;
  float ang_vel_y;
  float ang_vel_z;
  float lin_acc_x;
  float lin_acc_y;
  float lin_acc_z;
};
struct ImuMsg imu_msg;

void setup() {
  /* multi core init */
  MP.begin();
  MP.RecvTimeout(MP_RECV_POLLING);

  /* imu init */
  BMI160.begin(BMI160GenClass::I2C_MODE);
  BMI160.setGyroRate(SENSE_RATE);
  BMI160.setAccelerometerRate(SENSE_RATE);
  BMI160.setGyroRange(GYRO_RANGE);
  BMI160.setAccelerometerRange(ACCL_RANGE);

  BMI160.autoCalibrateGyroOffset();
  BMI160.autoCalibrateAccelerometerOffset(X_AXIS, 0);
  BMI160.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
  BMI160.autoCalibrateAccelerometerOffset(Z_AXIS, 1);

  /* imu calibration */
  // calcurate offset
  for (int i = 0; i < COUNT_FOR_CALIB; ++i) {
    int xAcc, yAcc, zAcc;
    BMI160.readAccelerometer(xAcc, yAcc, zAcc);
    offset_acc_x += convertRawAccel(xAcc);
    offset_acc_y += convertRawAccel(yAcc);
    offset_acc_z += convertRawAccel(zAcc)-1.0;
  }
  offset_acc_x /= COUNT_FOR_CALIB;
  offset_acc_y /= COUNT_FOR_CALIB;
  offset_acc_z /= COUNT_FOR_CALIB;
  
  offset_acc_z *= 1000; // The offset supports +/-495.3(mg)
  MPLog("calib_a_x: %f", offset_acc_x);
  MPLog("calib_a_y: %f", offset_acc_y);
  MPLog("calib_a_z: %f", offset_acc_z);
  delay(1000);
  digitalWrite(LED1, HIGH);
}

void loop() {
  static uint32_t start_time = 0;
  static uint32_t last_time = 0;
  int8_t recvid;
  void* msgin;
  int ret_rec = MP.Recv(&recvid, &msgin);
  int8_t sndid = 101;
  if (ret_rec >=0) {
    if (recvid == 100) {
      int ret_sed = MP.Send(sndid, &imu_msg);
      if (ret_sed < 0) { 
        MPLog("MP.Send imu error: %d\n", ret_sed); 
      }
    }
  }

  int rawRoll, rawPitch, rawYaw;
  BMI160.readGyro(rawRoll, rawPitch, rawYaw);
  float omega_roll  = convertRawGyro(rawRoll);
  float omega_pitch = convertRawGyro(rawPitch);
  float omega_yaw   = convertRawGyro(rawYaw);

  int rawXAcc, rawYAcc, rawZAcc;
  BMI160.readAccelerometer(rawXAcc, rawYAcc, rawZAcc);
  float calib_acc_x = convertRawAccel(rawXAcc) - offset_acc_x;
  float calib_acc_y = convertRawAccel(rawYAcc) - offset_acc_y;
  float calib_acc_z = convertRawAccel(rawZAcc) - offset_acc_z;

  imu_msg.ang_vel_x = omega_roll;
  imu_msg.ang_vel_y = omega_pitch;
  imu_msg.ang_vel_z = omega_yaw;
  imu_msg.lin_acc_x = calib_acc_x;
  imu_msg.lin_acc_y = calib_acc_y;
  imu_msg.lin_acc_z = calib_acc_z;
}
