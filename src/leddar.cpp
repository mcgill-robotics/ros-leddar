#include <angles/angles.h>
#include <dynamic_reconfigure/server.h>
#include <leddar/ScanConfig.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include "LeddarC.h"
#include "LeddarProperties.h"

// Leddar handler.
static LeddarHandle handler = NULL;

// Leddar specifications.
#define BEAM_COUNT 16
static std::string frame;
static double max_range;
static double field_of_view;

// ROS publisher.
ros::Publisher pub;

static void log_error(const char* prefix, int code) {
  if (code != LD_SUCCESS) {
    switch (code) {
      case LD_ACCESS_DENIED:
        ROS_ERROR("%s: LD_ACCESS_DENIED", prefix);
        break;
      case LD_TIMEOUT:
        ROS_ERROR("%s: LD_TIMEOUT", prefix);
        break;
      case LD_START_OF_FILE:
        ROS_ERROR("%s: LD_START_OF_FILE", prefix);
        break;
      case LD_END_OF_FILE:
        ROS_ERROR("%s: LD_END_OF_FILE", prefix);
        break;
      case LD_NO_RECORD:
        ROS_ERROR("%s: LD_NO_RECORD", prefix);
        break;
      case LD_ALREADY_STARTED:
        ROS_ERROR("%s: LD_ALREADY_STARTED", prefix);
        break;
      case LD_NO_DATA_TRANSFER:
        ROS_ERROR("%s: LD_NO_DATA_TRANSFER", prefix);
        break;
      case LD_NOT_CONNECTED:
        ROS_ERROR("%s: LD_NOT_CONNECTED", prefix);
        break;
      case LD_INVALID_ARGUMENT:
        ROS_ERROR("%s: LD_INVALID_ARGUMENT", prefix);
        break;
      case LD_ERROR:
        ROS_ERROR("%s: LD_ERROR", prefix);
        break;
      case LD_NOT_ENOUGH_SPACE:
        ROS_ERROR("%s: LD_NOT_ENOUGH_SPACE", prefix);
        break;
    }
  }
}

static void leddar_callback(void* handler) {
  LdDetection detections[BEAM_COUNT];
  unsigned int count = LeddarGetDetectionCount(handler);
  if (count > BEAM_COUNT) {
    count = BEAM_COUNT;
  }

  // Acquire detections from Leddar.
  LeddarGetDetections(handler, detections, count);

  // Construct LaserScan message.
  sensor_msgs::LaserScan msg;
  msg.header.frame_id = frame;
  msg.header.stamp = ros::Time::now();

  // Set up field of view.
  msg.angle_min = angles::from_degrees(-field_of_view / 2.0);
  msg.angle_max = angles::from_degrees(field_of_view / 2.0);
  msg.angle_increment = angles::from_degrees(field_of_view / BEAM_COUNT);
  msg.range_min = 0.0;
  msg.range_max = max_range;

  // Push detections into message.
  for (int i = 0; i < count; i++) {
    msg.ranges.push_back(detections[i].mDistance);
  }

  // Publish and keep going.
  pub.publish(msg);
}

void configure_callback(leddar::ScanConfig& config, uint32_t level) {
  ROS_INFO("Reconfiguring...");
  int code;

  // Set relative intensity of LEDs.
  ROS_DEBUG("INTENSITY: %d", config.intensity);
  code = LeddarSetProperty(handler, PID_LED_INTENSITY, 0, config.intensity);
  log_error("LeddarSetProperty(PID_LED_INTENSITY)", code);

  // Set number of accumulations to perform.
  ROS_DEBUG("ACCUMULATIONS: %d", config.accumulations);
  code = LeddarSetProperty(handler, PID_ACCUMULATION_EXPONENT, 0,
                           config.accumulations);
  log_error("LeddarSetProperty(PID_ACCUMULATION_EXPONENT)", code);

  // Set number of oversamplings to perform between base samples.
  ROS_DEBUG("OVERSAMPLING: %d", config.oversampling);
  code = LeddarSetProperty(handler, PID_OVERSAMPLING_EXPONENT, 0,
                           config.oversampling);
  log_error("LeddarSetProperty(PID_OVERSAMPLING_EXPONENT)", code);

  // Set number of base samples acquired.
  ROS_DEBUG("BASE SAMPLES: %d", config.base_point_count);
  code = LeddarSetProperty(handler, PID_BASE_POINT_COUNT, 0,
                           config.base_point_count);
  log_error("LeddarSetProperty(PID_BASE_POINT_COUNT)", code);

  // Set offset to increase detection threshold.
  ROS_DEBUG("THRESHOLD OFFSET: %d", config.threshold_offset);
  code = LeddarSetProperty(handler, PID_THRESHOLD_OFFSET, 0,
                           config.threshold_offset);
  log_error("LeddarSetProperty(PID_THRESHOLD_OFFSET)", code);

  // Write changes to Leddar.
  code = LeddarWriteConfiguration(handler);
  log_error("LeddarWriteConfiguration()", code);
}

static void stream(LeddarHandle handler) {
  // Start data transfer and set up callback.
  ROS_INFO("Streaming...");

  int code = LeddarStartDataTransfer(handler, LDDL_DETECTIONS);
  log_error("LeddarStartDataTransfer()", code);

  LeddarSetCallback(handler, leddar_callback, handler);
}

static bool connect(LeddarHandle handler, const char* type,
                    const char* serial) {
  int code = LeddarConnect(handler, strdup(type), strdup(serial));
  log_error("LeddarConnect()", code);

  if (code == LD_SUCCESS) {
    ROS_INFO("Connected to %s", serial);
    return true;
  } else {
    ROS_FATAL("Failed to connect to %s", serial);
    return false;
  }
}

int main(int argc, char** argv) {
  // Initialize node.
  ros::init(argc, argv, "leddar");
  ros::NodeHandle nh("~");

  // Initialize publisher.
  pub = nh.advertise<sensor_msgs::LaserScan>(std::string("scan"), 1);

  // Initialize Leddar handler.
  handler = LeddarCreate();

  // Get Leddar specifications.
  if (!nh.hasParam("range")) {
    ROS_FATAL("~range parameter not set");
    return -2;
  }
  if (!nh.hasParam("fov")) {
    ROS_FATAL("~fov parameter not set");
    return -3;
  }
  if (!nh.hasParam("frame")) {
    ROS_FATAL("~frame parameter not set");
    return -4;
  }
  if (!nh.hasParam("type")) {
    ROS_FATAL("~type parameter not set");
    return -5;
  }
  nh.getParam("range", max_range);
  nh.getParam("fov", field_of_view);
  nh.getParam("frame", frame);

  // Get serial port and connect to Leddar.
  std::string serial, type;
  nh.getParam("serial", serial);
  nh.getParam("type", type);
  bool connected = connect(handler, type.c_str(), serial.c_str());
  if (!connected) {
    return -1;
  }

  // Set up dynamic_reconfigure server and callback.
  dynamic_reconfigure::Server<leddar::ScanConfig> server;
  dynamic_reconfigure::Server<leddar::ScanConfig>::CallbackType f;
  f = boost::bind(&configure_callback, _1, _2);
  server.setCallback(f);

  // Start stream until stopped.
  stream(handler);
  ros::spin();

  // Clean up.
  LeddarStopDataTransfer(handler);
  LeddarRemoveCallback(handler, leddar_callback, handler);
  LeddarDestroy(handler);

  return 0;
}
