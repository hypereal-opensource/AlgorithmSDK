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

#include "internal.h"
#include "ringbuffer.h"

enum class TrackingDataType {
	FAILED = -1,
	IMU = 0,
	LightHouse = 1,
};

class TrackingDataLoader
{
public:
	TrackingDataLoader();
	~TrackingDataLoader();

	virtual imu_packet * GetIMUData() = 0;

	virtual lightsensor_packet * GetLightSensorReport() = 0;

	virtual void SetDataInput(ringbuffer<imu_packet> * _imuRingBuffer, ringbuffer<lightsensor_packet> * _lightSensorRingBuffer) = 0;

	virtual void SetId(int _id)
	{
		id = _id;
	}

	virtual int init() = 0;
	
	//0 IMU; 1 LightHouse
	virtual TrackingDataType ProceedData();

	uint8_t* getData(){ return _data; }

	static const int DATASIZE = 64;
	uint8_t _data[DATASIZE];

	int search_beginpoint(uint8_t* data, const int buffersize, bool& bIMU);
	int search_LHdatabegin(uint8_t* data, const int buffersize);
	virtual void blockReadData(uint8_t *buffer, size_t size) = 0;
	int alignData();

	int id;
};

