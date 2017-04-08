// *****************************************************************************
// Module..: Leddar Enabler -- Software development kit for Leddar products.
//
/// \file    LeddarProperties.h
///
/// \brief   Definition of property ids.
//
//  \since   June 2014
//
// Platform: Win32, Linux
//
// Copyright (c) 2014-2015 LeddarTech Inc. All rights reserved.
// Information contained herein is or may be confidential and proprietary to
// LeddarTech inc. Prior to using any part of the software development kit
// accompanying this notice, you must accept and agree to be bound to the
// terms of the LeddarTech Inc. license agreement accompanying this file.
// *****************************************************************************

#pragma once

enum LdProperties
{
    PID_NAME                       = 0x000022,
    PID_LED_INTENSITY              = 0x00002A,
    PID_ACCUMULATION_EXPONENT      = 0x0000A0,
    PID_OVERSAMPLING_EXPONENT      = 0x0000A1,
    PID_BASE_POINT_COUNT           = 0x0000A2,
    PID_THRESHOLD_OFFSET           = 0x0000A3,
    PID_GAIN_ENABLED               = 0x0000A4,
    PID_TIMEBASE_DELAY             = 0x00012A,
    PID_AMPLITUDE_SCALE            = 0x001002,
    PID_DISTANCE_SCALE             = 0x001003,
    PID_FILTERED_SCALE             = 0x001004,
    PID_REAL_DISTANCE_OFFSET       = 0x001006,
    PID_POINT_STEP                 = 0x001009,
    PID_SEGMENT_COUNT              = 0x001015,
    PID_DEVICE_TYPE                = 0x200002,
    PID_FIELD_OF_VIEW              = 0x200003,
    PID_ACQ_OPTIONS                = 0x0000BC,		// 1: Automatic led intensity, 2: Automatic transimpedance gain, 4: Enable the two object demerge algorithm, 8: Disable xtalk removal algorithm (logical inverse)
    PID_CHANGE_DELAY               = 0x0000BD,
    PID_DISTANCE_RESOLUTION        = 0x0000D8,		// 1000 = mm, 100 = cm, 10 = dm, 1 = m
    PID_CHANNEL_SELECT             = 0x0000D9,		// Bit X = Channel X selected.
    PID_SERIAL_BAUD_RATE           = 0x0000AB,
    PID_SERIAL_PARITY              = 0x0000AD,		// 0: None, 1: Odd, 2: Even
    PID_SERIAL_STOP_BITS           = 0x0000AE,
    PID_SERIAL_ADDRESS             = 0x0000AF,
    PID_SERIAL_MAX_ECHOES          = 0x0000C9,
    PID_CAN_BAUD_RATE              = 0x0000B0,
    PID_CAN_TX_BASE                = 0x0000B1,
    PID_CAN_RX_BASE                = 0x0000B2,
    PID_CAN_FRAME_FORMAT           = 0x0000B3,		// 0: Standard, 1: Extended
    PID_CAN_OPTIONS                = 0x0000D3,		// 1: Send detection on multiple message ID, 2: Delay in msec between a message sent in a message box(es), 4: Delay in msec between cycle of acquisition message frame, 8: Use detection message with flag information
    PID_CAN_MAILBOX_DELAY          = 0x0000D4,
    PID_CAN_CYCLE_DELAY            = 0x0000D5,
    PID_CAN_MAX_ECHOES             = 0x0000D6,
    PID_ZONE_NEAR_LIMIT            = 0x0000C3,
    PID_ZONE_FAR_LIMIT             = 0x0000C2,
    PID_ZONE_ENABLED               = 0x0000C5,		// Bit 1 = Zone 1, Bit 2 = Zone 2
    PID_ZONE_SEGMENT_ENABLED       = 0x0000C4,		// Bit 1 = Segment 1, Bit 2 = Segment 2 ... Bit 16 = Segment 16
    PID_ZONE_RISING_DEBOUNCE       = 0x0000BE,		// Activation Delay
    PID_ZONE_FALLING_DEBOUNCE      = 0x0000BF,		// Deactivation Delay
    PID_OUTPUT_PNP                 = 0x0000C0,
    PID_OUTPUT_INVERTED            = 0x0000C1,
    PID_TEACH_MARGIN               = 0x0000C6,
    PID_CONFIG_MODE                = 0x0000C7,		// 0: Teach, 1: Quick, 2: Advanced
    PID_MEASUREMENT_RATE           = 0x300000,
    PID_USEFUL_RANGE               = 0x300001,
};
