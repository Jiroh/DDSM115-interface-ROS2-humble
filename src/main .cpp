//ddsm115 ros2
#include <micro_ros_arduino.h>
#include <micro_ros_platformio.h>
#include <M5Unified.h>
#include <Arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include "keys.h"
#include <geometry_msgs/msg/twist.h>
#include <FastLED.h>

//assign stop pin #1
const uint8_t stop_button_pin1 = 1;

//LED settings
#define PIN_LED    21 //full color led assign G21
#define NUM_LEDS   1  //number of full color led

CRGB leds[NUM_LEDS];

// serial buffer
unsigned char sendbuf[32];

//variable
unsigned long pre_time;
String mode;

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

const int watch_dog_msec = 1000;
int last_control_msec = 0;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop()
{
  while (1)
  {
    Serial.printf("error_loop\n");
    delay(1);
  }
}


double w_r=0, w_l=0;
//wheel_rad is the wheel radius ,wheel_sep is
double wheel_rad = 0.100, wheel_sep = 0.235;
int lowSpeed = 200;
int highSpeed = 50;
double speed_ang=0, speed_lin=0;

// CRC8 lookup table
const uint8_t CRC8Table[256] = {
    0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
    157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
    35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
    190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
    70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
    219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
    101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
    248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
    140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
    17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
    175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
    50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
    202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
    87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
    233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
    116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53
};

// CRC8 calculation using lookup table
uint8_t crc8(const uint8_t* data, uint8_t len) {
  uint8_t crc = 0x00;
  while (len--) {
    crc = CRC8Table[crc ^ *data++];
  }
  return crc;
}




// twist message cb
void subscription_callback(const void *msgin)
{
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;  
  const float speed_ang = (msg->angular.z);
  const float speed_lin = (msg->linear.x);
  w_r = ((speed_lin/(wheel_rad)) + ((speed_ang*wheel_sep)/(2.0*wheel_rad)))*-5;
  w_l = ((speed_lin/(wheel_rad)) - ((speed_ang*wheel_sep)/(2.0*wheel_rad)))*5;  
  //USBSerial.printf("%f, %f\n", msg->linear.x, msg->angular.z);
  //USBSerial.printf("%f, %f\n", w_r, w_l);
  last_control_msec = millis();
}




void setup()
{
  auto cfg = M5.config();
  M5.begin(cfg);
  set_microros_wifi_transports(keys::wifi_ssid, keys::wifi_pass, keys::agent_ip, keys::agent_port);
  delay(2000);
  
  //serial settings
  Serial1.begin(115200, SERIAL_8N1, 44, 43);
  USBSerial.begin(115200); 

  //full color led
  FastLED.addLeds<WS2812B, PIN_LED, GRB>(leds, NUM_LEDS);
  leds[0] = CRGB(40, 40, 40);
  
  // micro-ros
  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
      &subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "cmd_vel"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
  
  //pin mode
  pinMode(stop_button_pin1, INPUT_PULLUP);
  //pinMode(stop_lamp, OUTPUT);

  //variable initialize
  mode = "run";

  // set velocity loop mode ID=1
  sendbuf[0] = (unsigned char) 0x01;   // motor ID=1
  sendbuf[1] = (unsigned char) 0xA0;   // motor mode switch command
  sendbuf[2] = (unsigned char) 0x00;   // null data
  sendbuf[3] = (unsigned char) 0x00;   // null data
  sendbuf[4] = (unsigned char) 0x00;   // null data
  sendbuf[5] = (unsigned char) 0x00;   // null data
  sendbuf[6] = (unsigned char) 0x00;   // null data
  sendbuf[7] = (unsigned char) 0x00;   // null data
  sendbuf[8] = (unsigned char) 0x00;   // null data
  sendbuf[9] = (unsigned char) 0x02;   // set velosity loop
  
  // transmit mode data ID=1
  Serial1.write(sendbuf, 10); 
  delay(5);// wait delay
  
  // set velocity loop mode ID=2
  sendbuf[0] = (unsigned char) 0x02;   // motor ID=2
  sendbuf[1] = (unsigned char) 0xA0;   // motor mode switch command
  sendbuf[2] = (unsigned char) 0x00;   // null data
  sendbuf[3] = (unsigned char) 0x00;   // null data
  sendbuf[4] = (unsigned char) 0x00;   // null data
  sendbuf[5] = (unsigned char) 0x00;   // null data
  sendbuf[6] = (unsigned char) 0x00;   // null data
  sendbuf[7] = (unsigned char) 0x00;   // null data
  sendbuf[8] = (unsigned char) 0x00;   // null data
  sendbuf[9] = (unsigned char) 0x02;   // set velosity loop
  
  // transmit mode data ID=2
  Serial1.write(sendbuf, 10); 
  delay(5);// wait delay
  
}

void loop()
{

  // Check for emergency stop button press
  if (digitalRead(stop_button_pin1) == LOW) {
     mode = "stop";
     USBSerial.printf("stop mode");
     leds[0] = CRGB::Red;
     FastLED.show();
  }
     else {
     mode = "run";
     USBSerial.printf("run mode");
     leds[0] = CRGB::Blue;
     FastLED.show();    
     
    }

  // Check the current mode
  if (mode == "run") {
     //digitalWrite(stop_lamp, LOW);  // Turn off the stop lamp

    // Process packet data ID=1
    sendbuf[0] = (unsigned char)0x01;  // Motor ID=1 Right
    sendbuf[1] = (unsigned char)0x64;  // Rotate motor command
    sendbuf[2] = (unsigned char)w_r;   // Velocity high 8 bits
    sendbuf[3] = (unsigned char)0x00;  // Velocity low 8 bits
    sendbuf[4] = (unsigned char)0x00;  // Null data
    sendbuf[5] = (unsigned char)0x00;  // Null data
    sendbuf[6] = (unsigned char)0x02;  // Acceleration time
    sendbuf[7] = (unsigned char)0x00;  // Brake
    sendbuf[8] = (unsigned char)0x00;  // Null data

    // CRC8 calculation using lookup table
    sendbuf[9] = crc8(sendbuf, 9);  // CRC8

    // Transmit data for ID=1
    Serial1.write(sendbuf, 10);
    delay(2);  // Wait delay

    // Process packet data ID=2
    sendbuf[0] = (unsigned char)0x02;  // Motor ID=2 Left
    sendbuf[1] = (unsigned char)0x64;  // Rotate motor command
    sendbuf[2] = (unsigned char)w_l;   // Velocity high 8 bits
    sendbuf[3] = (unsigned char)0x00;  // Velocity low 8 bits
    sendbuf[4] = (unsigned char)0x00;  // Null data
    sendbuf[5] = (unsigned char)0x00;  // Null data
    sendbuf[6] = (unsigned char)0x02;  // Acceleration time
    sendbuf[7] = (unsigned char)0x00;  // Brake
    sendbuf[8] = (unsigned char)0x00;  // Null data

    // CRC8 calculation using lookup table
    sendbuf[9] = crc8(sendbuf, 9);  // CRC8

    // Transmit data for ID=2
    Serial1.write(sendbuf, 10);
    delay(2);  // Wait delay

    } 
  else if (mode == "stop") {
    // Stop the motors
    w_r = 0x00;
    w_l = 0x00;
    // Turn on the stop lamp
    //digitalWrite(stop_lamp, HIGH);
   }
 
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
}
