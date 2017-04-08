// *****************************************************************************
// Module..: Leddar Enabler -- Software development kit for Leddar products.
//
/// \file    LeddarC.h
///
/// \brief   Declarations for the C interface of the Leddar Enabler.
///
//  \author  Louis Perreault
//
//  \since   June 2014
//
// Platform: Win32
//
// Copyright (c) 2014 LeddarTech Inc. All rights reserved.
// Information contained herein is or may be confidential and proprietary to
// LeddarTech inc. Prior to using any part of the software development kit
// accompanying this notice, you must accept and agree to be bound to the
// terms of the LeddarTech Inc. license agreement accompanying this file.
// *****************************************************************************

#pragma once

#ifdef WIN32
  #ifdef LEDDARC_LIBRARY
    #define LEDDARC_DLL __declspec( dllexport )
  #else
    #define LEDDARC_DLL __declspec( dllimport )
  #endif
#else
  #define LEDDARC_DLL __attribute__ ((visibility ("default")))
#endif

#ifdef __cplusplus
extern "C"
{
#endif

// Result codes that can be returned from LeddarC functions.
// 0 is success, negative is a failure, positive is success
// with warning.
#define LD_ACCESS_DENIED      (-10)
#define LD_TIMEOUT            (-9)
#define LD_START_OF_FILE      (-8)
#define LD_END_OF_FILE        (-7)
#define LD_NO_RECORD          (-6)
#define LD_ALREADY_STARTED    (-5)
#define LD_NO_DATA_TRANSFER   (-4)
#define LD_NOT_CONNECTED      (-3)
#define LD_INVALID_ARGUMENT   (-2)
#define LD_ERROR              (-1)
#define LD_SUCCESS            0
#define LD_NOT_ENOUGH_SPACE   1

typedef void *LeddarHandle;
typedef unsigned short LeddarU16;
typedef unsigned int   LeddarU32;
// This one is just for Matlab for the LeddarListSensors function.
typedef char           LeddarChar;
typedef void         (*LdCallback)( void * );
typedef unsigned char  LeddarBool;

enum LdDataLevels
{
    LDDL_NONE = 0,
    LDDL_STATE = 1,
    LDDL_DETECTIONS = 2
};

typedef struct _LdDetection
{
    float     mDistance;
    float     mAmplitude;
    LeddarU16 mSegment;
    LeddarU16 mFlags;
} LdDetection;

LEDDARC_DLL LeddarHandle
LeddarCreate( void );

LEDDARC_DLL LeddarHandle
LeddarCreateWithConsole( int argc, char ** argv );

LEDDARC_DLL void
LeddarDestroy( LeddarHandle aHandle );

LEDDARC_DLL int
LeddarListSensors( LeddarChar *aAddresses, LeddarU32 *aSize );

LEDDARC_DLL void
LeddarDisconnect( LeddarHandle aHandle );

LEDDARC_DLL int
LeddarConnect( LeddarHandle aHandle, char *aConnectionType,
               char *aConnectionString );

LEDDARC_DLL int
LeddarConnectByDeviceName( LeddarHandle aHandle, char *aConnectionType, char *aDeviceName );

LEDDARC_DLL int
LeddarGetConnected( LeddarHandle aHandle );

LEDDARC_DLL int
LeddarLoadRecord( LeddarHandle aHandle, char *aFileName );

LEDDARC_DLL void
LeddarSetCallback( LeddarHandle aHandle, LdCallback aCallback, void *aUserData );

LEDDARC_DLL int
LeddarRemoveCallback( LeddarHandle aHandle, LdCallback aCallback, void *aUserData );

LEDDARC_DLL int
LeddarStartDataTransfer( LeddarHandle aHandle, LeddarU32 aLevels );

LEDDARC_DLL int
LeddarStopDataTransfer( LeddarHandle aHandle );

LEDDARC_DLL int
LeddarIsNewDataAvailable( LeddarHandle aHandle );

LEDDARC_DLL int
LeddarWaitForData( LeddarHandle aHandle, LeddarU32 aTimeout );

LEDDARC_DLL int
LeddarGetProperty( LeddarHandle aHandle, LeddarU32 aId,
                   LeddarU32 aIndex, double *aValue );

LEDDARC_DLL int
LeddarSetProperty( LeddarHandle aHandle, LeddarU32 aId,
                   LeddarU32 aIndex, double aValue );

LEDDARC_DLL int
LeddarGetTextProperty( LeddarHandle aHandle, LeddarU32 aId,
                   LeddarU32 aIndex, LeddarChar *aValue );

LEDDARC_DLL int
LeddarSetTextProperty( LeddarHandle aHandle, LeddarU32 aId,
                   LeddarU32 aIndex, LeddarChar* aValue );

LEDDARC_DLL int
LeddarGetLedIntensityMapping( LeddarHandle aHandle, LeddarU16 **aIndexList,
                              LeddarU16 **aIntensityList, size_t *aSize );

LEDDARC_DLL int
LeddarWriteConfiguration( LeddarHandle aHandle );

LEDDARC_DLL int
LeddarRestoreConfiguration( LeddarHandle aHandle );

LEDDARC_DLL int
LeddarGetConfigurationModified( LeddarHandle aHandle );

LEDDARC_DLL LeddarU32
LeddarGetDetectionCount( LeddarHandle aHandle );

LEDDARC_DLL int
LeddarGetDetections( LeddarHandle aHandle, LdDetection *aDetections,
                     LeddarU32 aLength );

LEDDARC_DLL int
LeddarGetResult( LeddarHandle aHandle, LeddarU32 aId, LeddarU32 aIndex,
                 double *aValue );

LEDDARC_DLL int
LeddarGetResultArray( LeddarHandle aHandle, LeddarU32 aId, LeddarU32 aIndex,
                      void *aData, LeddarU32 aSize );

LEDDARC_DLL int
LeddarSetCommandParameter( LeddarHandle aHandle, LeddarU32 aId,
                           LeddarU32 aIndex, double aValue );

LEDDARC_DLL int
LeddarExecuteCommand( LeddarHandle aHandle, LeddarU32 aId,
                      LeddarU32 *aResultCode );

LEDDARC_DLL int
LeddarConfigureRecording( char *aDirectory,
                          LeddarU32 aMaxFileSize,
                          LeddarU32 aLevels );

LEDDARC_DLL int
LeddarStartRecording( LeddarHandle aHandle, LeddarChar *aFileName );

LEDDARC_DLL int
LeddarStopRecording( LeddarHandle aHandle );

LEDDARC_DLL int
LeddarGetRecording( LeddarHandle aHandle );

LEDDARC_DLL LeddarU32
LeddarGetRecordSize( LeddarHandle aHandle );

LEDDARC_DLL LeddarU32
LeddarGetRecordingDirectory( LeddarChar *aPath, size_t aSize );

LEDDARC_DLL LeddarU32
LeddarGetMaxRecordFileSize( void );

LEDDARC_DLL int
LeddarGetRecordLoading( LeddarHandle aHandle );

LEDDARC_DLL LeddarU32
LeddarGetCurrentRecordIndex( LeddarHandle aHandle );

LEDDARC_DLL int
LeddarStepForward( LeddarHandle aHandle );

LEDDARC_DLL int
LeddarStepBackward( LeddarHandle aHandle );

LEDDARC_DLL int
LeddarMoveRecordTo( LeddarHandle aHandle, unsigned int aIndex );

LEDDARC_DLL void
LeddarSleep( double aSeconds );

LEDDARC_DLL int
LeddarGetKey( void );

LEDDARC_DLL LeddarBool
LeddarKeyPressed( void );

#ifdef __cplusplus
}
#endif
