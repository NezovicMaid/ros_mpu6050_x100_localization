#include <Arduino.h>
#include <Wire.h>
#include <TinyMPU6050.h>
#include <ros.h>
#include <geometry_msgs/Vector3.h>

// MPU-6050 instanca
MPU6050 mpu(Wire);

// ROS node handle
ros::NodeHandle nh;

// Poruka tipa Vector3 (za R, P, Y)
geometry_msgs::Vector3 rpy_msg;

// Publisher na temu /RPY
ros::Publisher rpy_pub("RPY", &rpy_msg);

// Stepeni u radijane
#define DEG_TO_RADS(x) ((x) * 0.0174533)

void setup() {
  // Serial i ROS setup
  //Serial.begin(9600); // Ostavljeno zakomentarisano kao u tvom radu
  nh.initNode();
  nh.advertise(rpy_pub);
  
  // Inicijalizacija MPU
  //Serial.println("=======================================================");
  //Serial.println("Starting calibration...");
  mpu.Initialize();
  mpu.Calibrate();
  //Serial.println("Calibration complete!");
}

void loop() {
  mpu.Execute();

  // itanje uglova u stepenima
  float roll_deg = mpu.GetAngX();
  float pitch_deg = mpu.GetAngY();
  float yaw_deg = mpu.GetAngZ();

  // Pretvaranje u radijane
  rpy_msg.x = DEG_TO_RADS(roll_deg);
  rpy_msg.y = DEG_TO_RADS(pitch_deg);
  rpy_msg.z = DEG_TO_RADS(yaw_deg);

  // Slanje poruke
  rpy_pub.publish(&rpy_msg);
  nh.spinOnce();
  
  delay(10); // malo ekanje za stabilnost
}