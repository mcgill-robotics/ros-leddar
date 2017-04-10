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

#define ARRAY_LEN( a )  (sizeof(a)/sizeof(a[0]))

// Global variable to avoid passing to each function.
static LeddarHandle gHandle=NULL;

// Leddar specifications.
#define BEAM_COUNT 16
static std::string frame;
static double max_range;
static double field_of_view;

// ROS publisher.
ros::Publisher pub;

// *****************************************************************************
// Function: CheckError
//
/// \brief   Check a return code and if it is not success, display an error
///          message corresponding to the code.
///
/// \param   aCode  The result code to verify.
// *****************************************************************************

static void
CheckError( int aCode )
{
  if ( aCode != LD_SUCCESS )
  {
    printf( "LeddarC error (%d): ", aCode );

    switch ( aCode )
    {
      case LD_ACCESS_DENIED : printf("LD_ACCESS_DENIED\n" ); break;
      case LD_TIMEOUT : printf("LD_TIMEOUT\n" ); break;
      case LD_START_OF_FILE : printf("LD_START_OF_FILE\n" ); break;
      case LD_END_OF_FILE : printf("LD_END_OF_FILE\n" ); break;
      case LD_NO_RECORD : printf("LD_NO_RECORD\n" ); break;
      case LD_ALREADY_STARTED : printf("LD_ALREADY_STARTED\n" ); break;
      case LD_NO_DATA_TRANSFER : printf("LD_NO_DATA_TRANSFER\n" ); break;
      case LD_NOT_CONNECTED : printf("LD_NOT_CONNECTED\n" ); break;
      case LD_INVALID_ARGUMENT : printf("LD_INVALID_ARGUMENT\n" ); break;
      case LD_ERROR : printf("LD_ERROR\n" ); break;
      case LD_NOT_ENOUGH_SPACE : printf("LD_NOT_ENOUGH_SPACE\n" ); break;
    }
  }
}

// *****************************************************************************
// Function: DataCallback
//
/// \brief   This is the function that is called when a new set of data is
///          available. Here we simply display the first 12 detections.
///
/// \param   aHandle  This is the user data parameter that was passed to
///                   LeddarAddCallback. Here by design we know its the handle.
/// \param   aLevels  A bitmask of the data levels received in that frame.
///
/// \return  Non zero to be called again (responding 0 would remove this
///          function from the callback list).
// *****************************************************************************

static void
DataCallback( void *aHandle )
{
  LdDetection lDetections[BEAM_COUNT];
  unsigned int lCount = LeddarGetDetectionCount( aHandle );

  if ( lCount > ARRAY_LEN( lDetections ) )
  {
    lCount = ARRAY_LEN( lDetections );
  }

  LeddarGetDetections( aHandle, lDetections,lCount );

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
  for( int i=0; i<lCount; i++ )
  {
    msg.ranges.push_back(lDetections[i].mDistance);
  }

  // Publish and keep going.
  pub.publish(msg);
}

// *****************************************************************************
// Function: ReadLiveData
//
/// \brief   Start data transfer
// *****************************************************************************

static void
ReadLiveData( void )
{
  // Start data transfer and set up callback.
  ROS_INFO("Streaming...");
  CheckError( LeddarStartDataTransfer( gHandle, LDDL_DETECTIONS ) );
  LeddarSetCallback( gHandle, DataCallback, gHandle );
}

// *****************************************************************************
// Function: ConfigurationMenu
//
/// \brief   Change configuration parameters.
// *****************************************************************************

static void
configure_callback(leddar::ScanConfig &config, uint32_t level)
{
    ROS_INFO("Reconfiguring...");

    // Set relative intensity of LEDs.
    ROS_DEBUG("INTENSITY: %d", config.intensity);
    LeddarSetProperty(gHandle, PID_LED_INTENSITY, 0, config.intensity);

    // Set number of accumulations to perform.
    ROS_DEBUG("ACCUMULATIONS: %d", config.accumulations);
    LeddarSetProperty(gHandle, PID_ACCUMULATION_EXPONENT, 0,
                      config.accumulations);

    // Set number of oversamplings to perform between base samples.
    ROS_DEBUG("OVERSAMPLING: %d", config.oversampling);
    LeddarSetProperty(gHandle, PID_OVERSAMPLING_EXPONENT, 0,
                      config.oversampling);

    // Set number of base samples acquired.
    ROS_DEBUG("BASE SAMPLES: %d", config.base_point_count);
    LeddarSetProperty(gHandle, PID_BASE_POINT_COUNT, 0,
                      config.base_point_count);

    // Set offset to increase detection threshold.
    ROS_DEBUG("THRESHOLD OFFSET: %d", config.threshold_offset);
    LeddarSetProperty(gHandle, PID_THRESHOLD_OFFSET, 0,
                      config.threshold_offset);

    // Write changes to Leddar.
    LeddarWriteConfiguration(gHandle);
}

static void
ConnectSensor( const char* aSerial )
{
  char* lConnectionType = "SERIAL";
  // Use default device` if unspecified.
  if (aSerial[0] == '\0')
  {
      aSerial = "default";
  }

  int lCode = LeddarConnect( gHandle, lConnectionType, aSerial );

  if ( lCode == LD_SUCCESS )
  {
    ROS_INFO("Connected to %s", aSerial);
  }
  else
  {
    ROS_FATAL("Failed to connect to %s with code: %d", aSerial, lCode);
  }
}

// *****************************************************************************
// Function: main
//
/// \brief   Standard C entry point!
// *****************************************************************************

int
main( int argc, char *argv[] )
{
  // Initialize node.
  ros::init(argc, argv, "leddar");
  ros::NodeHandle nh("~");

  // Initialize publisher.
  pub = nh.advertise<sensor_msgs::LaserScan>(std::string("scan"), 1);

  // Initialize Leddar handler.
  gHandle = LeddarCreate();

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
  ConnectSensor( serial.c_str() );
  if ( !LeddarGetConnected( gHandle ) ) {
      LeddarDestroy( gHandle );
      return -1;
  }

  // Set up dynamic_reconfigure server and callback.
  dynamic_reconfigure::Server<leddar::ScanConfig> server;
  dynamic_reconfigure::Server<leddar::ScanConfig>::CallbackType f;
  f = boost::bind(&configure_callback, _1, _2);
  server.setCallback(f);

  ReadLiveData();
  ros::spin();

  // Clean up.
  LeddarStopDataTransfer( gHandle );
  LeddarDestroy( gHandle );

  return 0;
}

// End of file leddar_main.cpp
