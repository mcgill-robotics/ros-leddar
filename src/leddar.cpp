#include <ros/ros.h>
#include <ros/console.h>
#include <angles/angles.h>
#include <leddar/ScanConfig.h>
#include <sensor_msgs/LaserScan.h>
#include <dynamic_reconfigure/server.h>

#include <stdio.h>
#include <ctype.h>
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


static unsigned char leddar_callback(void *handler, unsigned int levels) {
    LdDetection detections[BEAM_COUNT];
    unsigned int count = LeddarGetDetectionCount(handler);
    if (count > BEAM_COUNT) {
        count = BEAM_COUNT;
    }

    // Acquire detections from Leddar.
    LeddarGetDetections(handler, detections, sizeof(detections));

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
    return 1;
}


void configure_callback(leddar::ScanConfig &config, uint32_t level) {
    ROS_INFO("Reconfiguring...");

    // Set relative intensity of LEDs.
    ROS_DEBUG("INTENSITY: %d", config.intensity);
    LeddarSetProperty(handler, PID_LED_INTENSITY, 0, config.intensity);
    
    // Set automatic LED intensity.
    ROS_DEBUG("AUTO INTENSITY: %s", config.auto_intensity ? "true" : "false");
    LeddarSetProperty(handler, PID_AUTOMATIC_LED_INTENSITY, 0,
                      config.auto_intensity);

    // Set number of accumulations to perform.
    ROS_DEBUG("ACCUMULATIONS: %d", config.accumulations);
    LeddarSetProperty(handler, PID_ACCUMULATION_EXPONENT, 0,
                      config.accumulations);

    // Set number of oversamplings to perform between base samples.
    ROS_DEBUG("OVERSAMPLING: %d", config.oversampling);
    LeddarSetProperty(handler, PID_OVERSAMPLING_EXPONENT, 0,
                      config.oversampling);

    // Set number of base samples acquired.
    ROS_DEBUG("BASE SAMPLES: %d", config.base_point_count);
    LeddarSetProperty(handler, PID_BASE_POINT_COUNT, 0,
                      config.base_point_count);

    // Set offset to increase detection threshold.
    ROS_DEBUG("THRESHOLD OFFSET: %d", config.threshold_offset);
    LeddarSetProperty(handler, PID_THRESHOLD_OFFSET, 0,
                      config.threshold_offset);

    // Set detection of 2 objects close to each other.
    ROS_DEBUG("DEMERGING: %s", config.object_demerging ? "true" : "false");
    LeddarSetProperty(handler, PID_OBJECT_DEMERGING, 0,
                      config.object_demerging);

    // Write changes to Leddar.
    LeddarWriteConfiguration(handler);
}


static void stream(LeddarHandle handler) {
    // Start data transfer and set up callback.
    ROS_INFO("Streaming...");
    LeddarStartDataTransfer(handler, LDDL_DETECTIONS);
    LeddarAddCallback(handler, leddar_callback, handler);
}


static void connect(LeddarHandle handler, const char* serial) {
    int code = LeddarConnect(handler, serial);
    
    // Use default device` if unspecified.
    if (serial[0] == '\0') {
        serial = "default";
    }

    if (code == LD_SUCCESS) {
        ROS_INFO("Connected to %s", serial);
    } else {
        ROS_FATAL("Failed to connect to %s with code: %d", serial, code);
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
    nh.getParam("range", max_range);
    nh.getParam("fov", field_of_view);
    nh.getParam("frame", frame);

    // Get serial port and connect to Leddar.
    std::string serial;
    nh.getParam("serial", serial);
    connect(handler, serial.c_str());
    if (!LeddarGetConnected(handler)) {
        LeddarDestroy(handler);
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
    LeddarDestroy(handler);

    return 0;
}
