#include "eigen.h"
#include <Wire.h>
#include <Arduino_LSM9DS1.h>
#include <Eigen/Geometry>

using namespace Eigen;

float wx, wy, wz; //gyro measurement
float wx_o, wy_o, wz_o; //constant gyro bias

int skip = 1;

Matrix3f R_gyro; //Rotation matrix from gyro integration

Vector3f Omega; //angular vel. from gyro
Vector3f w_o; // //constant gyro bias vector

uint32_t loop_start_timer;
// #define Tus 10000
#define Tus 10000
// #define Ts 0.01  //100Hz Kalman Iteration speed
#define Ts 0.01

void setup() {
  Serial.begin(115200);

  while (!Serial);
  Serial.println("Started");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  //gyro details
  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Gyroscope in degrees/second");
  Serial.println("X\tY\tZ");

  //initialize R_gyro(0)
  R_gyro << 1, 0, 0,
           0, 1, 0,
           0, 0, 1 ;

  //get gyro bias
  get_gyro_bias();
  Serial.println(w_o(0));
  Serial.println(w_o(1));
  Serial.println(w_o(2));
  delay(10000);
}

void loop() {
  loop_start_timer = micros();

  //obstain gyro readings
  gyroMeasure(Omega);

  //gyro integration
  updateRotation(R_gyro, Omega, Ts);


  //convert to quaternions and print
  Quaternionf q1(R_gyro);
  q1.normalize();

  if (skip > 0)
  {
      // Print comma separated for Serial Plotter
      Serial.print(q1.w()); Serial.print(",");
      Serial.print(q1.x()); Serial.print(",");
      Serial.print(q1.y()); Serial.print(",");
      Serial.println(q1.z());
  }
  skip = -skip;

  while (micros() - loop_start_timer < Tus)
    ;
}
//vector to skew
Matrix3f skew(const Vector3f& v) {
  Matrix3f m;
  m <<    0,   -v(2),  v(1),
        v(2),     0,  -v(0),
       -v(1),  v(0),     0;
  return m;
}

// Update R using exponential map
// complete this part
void updateRotation(Matrix3f &R, const Vector3f &omega, float dt) {
  float theta = omega.norm() * dt;

  if(theta < 1e-6) {
    // Use first-order approx if angle small
    R = R + dt * R * skew(omega);
  } else {
    // Rodrigues formula
    Vector3f u = omega.normalized();
    Matrix3f K = skew(u);
    Matrix3f Exp = Matrix3f::Identity()
                   + sin(theta)*K
                   + (1-cos(theta))*(K*K);
    R = R * Exp;
  }
}

//function to get gyro measurements
void gyroMeasure(Vector3f &Omega)
{
  //For ENU frame - exchange sensor x and y directions
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(wx, wy, wz);
    }
  Omega << wy, wx, wz;
  Omega = Omega*3.14/180.0; //change unit to rad/sec
  Omega = Omega - w_o; //substract bias
}

//function to get gyro bias
void get_gyro_bias()
{
  w_o << 0,0,0;
  Vector3f w_o_sum(0,0,0);
  for (int i = 0; i < 2000; i++) {
    gyroMeasure(Omega);
    w_o_sum = w_o_sum + Omega;
    delay(5);
  }
  w_o = w_o_sum/2000;
}

