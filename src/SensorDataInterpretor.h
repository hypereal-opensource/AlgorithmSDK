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

#pragma once

#include "quaternion.h"
#include "internal.h"
#include <map>

namespace HYPEREAL
{
	class TrackObject;
	class SensorDataInterpretor
	{
	public:
		SensorDataInterpretor(const int slicedatasize, const int sensorNUM, TrackObject * _trackObject, const std::map<int, int>* _vertexMap);
		~SensorDataInterpretor();

		//1: light 1 x
		//2: light 1 y
		//3: light 2 x
		//4: light 2 y
		int ProcessNewSlice(lightsensor_packet & lightSensorReport, const irrmath::Quatd* quat = NULL);

		int ProcessNewIMU(imu_packet & imuReport, irrmath::vector3d* outGyro);
		int SensorDataInterpretor::ProcessNewIMU(imu_packet & imuReport, irrmath::Quatd* outQuat);

		double TimetickToAngle(uint32_t timetick);
		uint32_t AngleToTimetick(double angle);
	public:
		const unsigned int zeroDegreeTick = 100;
		const unsigned int PITick = 43850;

		double PILength = PITick - zeroDegreeTick;
		double eightyDegreeTick = zeroDegreeTick + (PILength * 8.0 / 18.0);

		int SLICEDATASIZE;
		int SENSORNUM;

		static const double _PI;

		// for 1234
		HitRecord* CurrentFrameRecords;
		int CurrentFrameRecordsNum[4];

		irrmath::Quatd CurrentFrameQuat[4];

		bool hasPreFrameRecords;

		int recordrownum = 0;
		int recordrownum2 = 0;
		const std::map<int, int> *vertexMap;
		TrackObject * _trackObject;
	};
}

