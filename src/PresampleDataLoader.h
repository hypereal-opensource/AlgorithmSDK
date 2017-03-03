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
#include "TrackingDataLoader.h"

#include <fstream>

class PresampleDataLoader :
	public TrackingDataLoader
{
public:
	PresampleDataLoader(const char* filename);
	~PresampleDataLoader();
	imu_packet * GetIMUData() override;
	lightsensor_packet * GetLightSensorReport() override;

	int init() override;

	void blockReadData(uint8_t *buffer, size_t size) override;
	TrackingDataType ProceedData() override;
	void SetDataInput(ringbuffer<imu_packet> * _imuRingBuffer, ringbuffer<lightsensor_packet> * _lightSensorRingBuffer) override
	{
		debug_print("PresampleDataLoader cannot be SetDataInput!\n");
		exit(EXIT_FAILURE);
	}

private:
	uint8_t buffer[sizeof(lightsensor_packet)];
	// For replaying data.
	TrackingDataType fetchedDataType;
	uint64_t fetchedDataTimeStamp;

	std::ifstream ifs;
	imu_packet * nextImuPacket;
	lightsensor_packet * nextLightPacket;
	uint64_t timeStampDelta;
	const unsigned long long MAX_INT_NUM = 0x0FFFFFFFFFF;
	bool hasFirstDataArrived;
};

