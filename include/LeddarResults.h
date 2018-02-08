// *****************************************************************************
// Module..: Leddar Enabler -- Software development kit for Leddar products.
//
/// \file    LeddarResults.h
///
/// \brief   Definition of result ids.
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

enum LdResults {
  RID_TIMESTAMP = 0x001A11,
  RID_TEMPERATURE = 0x001A03,
  RID_CPU_LOAD = 0x500100,  // Command CID_READ_ADVANCED_SENSOR_INFO must be
                            // executed each time.
  RID_MEASUREMENT_RATE = 0x500101,
};
