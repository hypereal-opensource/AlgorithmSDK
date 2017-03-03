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

#include "stdafx.h"
#include "RingBufferDataLoader.h"
#include<iostream>
#include "TimeManager.h"
#include "InternalVariables.h"
#include "HVR_Logger.h"
using HYPEREAL::InternalVariables;

RingBufferDataLoader::RingBufferDataLoader(ringbuffer<imu_packet> * _imuRingBuffer, ringbuffer<lightsensor_packet> * _lightSensorRingBuffer, std::string outFileName)
{
	this->outFileName = outFileName;
	HVR_DECLARE_LOGGER(alg_RingBufferDataLoader);
	HVR_LOG_INFO(alg_RingBufferDataLoader, "Init RingBufferDataLoader %s\n", outFileName.c_str());
	debug_print("Init RingBufferDataLoader!!\n");
	imuRingBuffer = _imuRingBuffer;
	lightSensorRingBuffer = _lightSensorRingBuffer;

	if (InternalVariables::GetInstance()->GetInt("define._SerialDataSave", 0) == 1)
	{
		foutData.open(InternalVariables::GetInstance()->Get("PATH_WORKSPACE", "") + outFileName, std::ofstream::binary);
	}
}

RingBufferDataLoader::~RingBufferDataLoader()
{
	if (imuRingBuffer)
	{
		delete imuRingBuffer;
	}
	if (lightSensorRingBuffer)
	{
		delete lightSensorRingBuffer;
	}
}

int RingBufferDataLoader::init()
{
	if (!imuRingBuffer || !lightSensorRingBuffer)
	{
		return -1;
	}
	tickCount = 0;
	return 0;

}


void RingBufferDataLoader::blockReadData(uint8_t *buffer, size_t size)
{

}

void CalculatePointProjection(double *normal, double * center,  double *a, double * b, double *res)
{
	double t = -(normal[0] * (a[0] - center[0]) + normal[1] * (a[1] - center[1]) + normal[2] * (a[2] - center[2])) / ((b[0] - a[0]) * normal[0] + (b[1] - a[1]) * normal[1] + (b[2] - a[2]) * normal[2]);
	res[0] = a[0] + (b[0] - a[0]) * t;
	res[1] = a[1] + (b[1] - a[1]) * t;
	res[2] = a[2] + (b[2] - a[2]) * t;
}

imu_packet * RingBufferDataLoader::GetIMUData()
{
	imu_packet * imuSensorReport = NULL;
	imuSensorReport = new imu_packet();
	size_t readNum = imuRingBuffer->read(imuSensorReport, 1);
	if (readNum == 0)
	{
		delete imuSensorReport;
		return NULL;
	}
	int occupied = 0;
	if ((occupied = int(imuRingBuffer->getOccupied())) > 4)
	{
		HVR_LOG_WARN(alg_RingBufferDataLoader, "Ringbuffer [%s,IMU] has too much data not consumed: %d\n", outFileName.c_str(), occupied);
	}

	if (imuSensorReport && InternalVariables::GetInstance()->GetInt("define._SerialDataSave", 0) == 1)
	{
		imuSensorReport->timestamp = TimeManager::GetMicroSeconds();
		char* data = (char *)(imuSensorReport);
		int size = sizeof(imu_packet) / sizeof(char);
		foutData.write(data, size);
	}
	return imuSensorReport;
}

lightsensor_packet * RingBufferDataLoader::GetLightSensorReport()
{
	lightsensor_packet * lightSensorReport = NULL;
	int occupied = 0;
	if ((occupied = int(lightSensorRingBuffer->getOccupied())) > 4)
	{
		HVR_LOG_WARN(alg_RingBufferDataLoader, "Ringbuffer [%s,LightSensor] has too much data not consumed: %d\n", outFileName.c_str(), occupied);
	}
	lightSensorReport = new lightsensor_packet();
	lightSensorRingBuffer->read(lightSensorReport, 1);
	if (lightSensorReport && InternalVariables::GetInstance()->GetInt("define._SerialDataSave", 0) == 1)
	{
		lightSensorReport->timestamp = TimeManager::GetMicroSeconds();
		char* data = (char *)(lightSensorReport);
		//printf("Write Sensor datas : timestamp = %lld\n", lightSensorReport->timestamp);
		int size = sizeof(lightsensor_packet) / sizeof(char);
		foutData.write(data, size);
	}
	return lightSensorReport;
}

TrackingDataType RingBufferDataLoader::ProceedData()
{
	tickCount++;
	tickCount %= 5;
	if (imuRingBuffer->getOccupied() == 0 && lightSensorRingBuffer->getOccupied() == 0)
		return TrackingDataType::IMU;

	if (tickCount != 0)
	{
		if (imuRingBuffer->getOccupied() > 0)
		{
			return TrackingDataType::IMU;
		}
		else
		{
			if (lightSensorRingBuffer->getOccupied() > 0)
			{
				return TrackingDataType::LightHouse;
			}
		}
	}
	else
	{
		if (lightSensorRingBuffer->getOccupied() > 0)
		{
			return TrackingDataType::LightHouse;
		}
		else
		{
			if (imuRingBuffer->getOccupied() > 0)
			{
				return TrackingDataType::IMU;
			}
		}
	}
	return TrackingDataType::FAILED;

}
