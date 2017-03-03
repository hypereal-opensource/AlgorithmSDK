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


class RingBufferDataLoader : public TrackingDataLoader
{
public:
	RingBufferDataLoader(ringbuffer<imu_packet> * _imuRingBuffer, ringbuffer<lightsensor_packet> * _lightSensorRingBuffer, std::string outFileName = "");
	~RingBufferDataLoader();
	
	LightSensorReport lightSensorReport;

	imu_packet * GetIMUData() override;
	lightsensor_packet * GetLightSensorReport() override;
	virtual TrackingDataType ProceedData() override;

	int init() override;

	void SetDataInput(ringbuffer<imu_packet> * _imuRingBuffer, ringbuffer<lightsensor_packet> * _lightSensorRingBuffer)
		override
	{
		if (_imuRingBuffer)
		{
			if (imuRingBuffer)
			{
				delete imuRingBuffer;
			}
			imuRingBuffer = _imuRingBuffer;
		}
		if (_lightSensorRingBuffer)
		{
			if (lightSensorRingBuffer)
			{
				delete lightSensorRingBuffer;
			}
			lightSensorRingBuffer = _lightSensorRingBuffer;
		}
	}

private:
	ringbuffer<imu_packet> * imuRingBuffer;
	ringbuffer<lightsensor_packet> * lightSensorRingBuffer;
	void blockReadData(uint8_t *buffer, size_t size) override;
	std::string outFileName;
	std::ofstream foutData;
	//std::mutex mtx;

	int tickCount;
};
