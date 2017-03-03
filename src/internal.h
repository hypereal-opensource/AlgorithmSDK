/*
 * The MIT License (MIT)
 * Copyright (c) 2017 Shanghai Chai Ming Huang Info&Tech Co£¬Ltd
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */

#ifndef INTERNAL_H
#define INTERNAL_H

// Use M_PI and so on in math.h
#define _USE_MATH_DEFINES

#include <stdint.h>
#include <math.h>
#include <map>
#include <memory>
#include "HVR_Hmd.h"

#define  MAX_LIGHTSENSOR_NUMBER 128

#ifdef debug_print
#undef debug_print
#endif

// disable warning of opencv's encoding.
#pragma warning (disable:4819)

#if defined(_DEBUG) || defined(ALGORITHMDLL_PRERELEASE)
#define debug_print(...) fprintf(stderr, __VA_ARGS__);
#else
#define debug_print(...)
#endif //  _DEBUG

enum class TRACK_MODE
{
	UNDEFINED,
	LIGHT_HOUSE_MODE
};

typedef enum
{
	//external
	IMU_RAWDATA = 0,      /// imu with raw data, include acce, gyro...
	IMU_QUAT,             /// imu with olny quaternion

	LIGHTSENSOR_RAWDATA, /// light sensor data
	LIGHTHOUSE_SYNC,     /// light house sync number


}PACKET_TYPE;

/************************************************************************/
/* IMU RAWDATA/IMU_QUAT shared                                          */
/************************************************************************/
typedef struct
{
	PACKET_TYPE type;
	uint64_t    timestamp;
	union
	{
		struct {                     /// IMU_RAWDATA used
			IMURawData      Samples[2];
			int16_t			MagneticField[3];
		}rawdata;
		float QuatIncrement[4];          /// IMU_QUAT used
	}_imu;

}imu_packet;


/************************************************************************/
/* LIGHT SENSOR RAWDATA                                                 */
/************************************************************************/
typedef struct
{
	PACKET_TYPE  type;
	uint64_t     timestamp;

	uint16_t ScanNumber;    /// if type === LIGHTHOUSE_SYNC, this field is available.
	uint8_t  ScanAxis;
	uint8_t  SampleCount;
	LightSensorData	LightSensor[MAX_LIGHTSENSOR_NUMBER];
}lightsensor_packet;

typedef struct
{
	double angle;
	bool is_x;
	int id_v;
	int id_LH;
	uint32_t time;
} HitRecord;

/************************************************************************/
/* SOME TRACE DATA FOR DEBUG                                            */
/************************************************************************/
enum SensorHitStatus {
	NOT_HIT_BOTH,
	NOT_HIT_X,
	NOT_HIT_Y,
	ACTIVE
};
static const char * SensorHitStatusString[] = {
	"NOT_HIT_BOTH",
	"NOT_HIT_X",
	"NOT_HIT_Y",
	"ACTIVE"
};

struct SensorHitData {
	SensorHitStatus hitStatus = NOT_HIT_BOTH;
	double xAngle = 0; // in radians
	double yAngle = 0 ; // in radians
	static inline uint32_t angle2time(double angle) { return uint32_t((angle - M_PI / 2) / 2 / M_PI * (5.25 * 1e6) / 60 + 18000.0f); }
	static inline double time2angle(uint32_t timetick) { return ((timetick - 18000.0f) * 60 / (5.25 * 1e6)) * 2 * M_PI + M_PI / 2; }
};

#endif

