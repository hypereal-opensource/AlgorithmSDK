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
#include "PresampleDataLoader.h"
#include <iostream>
#include "TimeManager.h"
#include "InternalVariables.h"

using namespace std;
using HYPEREAL::InternalVariables;

PresampleDataLoader::PresampleDataLoader(const char* filename)
{
	ifs.open(InternalVariables::GetInstance()->Get("PATH_WORKSPACE", "") + filename, ifstream::binary);
	fetchedDataType = TrackingDataType::FAILED;
	timeStampDelta = 0;
	nextImuPacket = NULL;
	nextLightPacket = NULL;
	fetchedDataTimeStamp = MAX_INT_NUM;
	hasFirstDataArrived = false;
}


PresampleDataLoader::~PresampleDataLoader()
{
	ifs.close();
}


int PresampleDataLoader::init()
{
	if (!ifs.is_open())
	{
		debug_print("-----------------------------------------Can not open sample data file!!!\n");
		exit(EXIT_FAILURE);
		return -1;
	}
	return 0;
}

void PresampleDataLoader::blockReadData(uint8_t *buffer, size_t size)
{
	if (ifs.good())
	{
		ifs.read((char*)buffer, size);
	}
	else if (ifs.eof())
	{
		if (InternalVariables::GetInstance()->GetInt("dv.exitOnNoData", 0) == 1)
		{
			// XXX: may loss some data.
			debug_print("Sample file's end is reached, will exit.\n");
			exit(0);
		}
	}
}


TrackingDataType PresampleDataLoader::ProceedData()
{
	auto nowtime = TimeManager::GetMicroSeconds();
	// Have fetched data.
	if (fetchedDataType == TrackingDataType::IMU || fetchedDataType == TrackingDataType::LightHouse)
	{
		if (nowtime >= fetchedDataTimeStamp) {
			return fetchedDataType;
		} else {
			return TrackingDataType::FAILED;
		}
	}
	//std::cout << "no wait now clock = " << clock() << " ,nextDataTimeStamp = " << nextDataTimeStamp << endl;
	buffer[0] = 255;
	blockReadData(buffer, 4);
	if (buffer[0] == 255)
	{
		return TrackingDataType::FAILED;
	}
	int rawType = *((int *)buffer);
	// 0 is IMU, 1 is LightHouse, otherwise FAILED
	PACKET_TYPE packetType = 
		rawType == 0 ? PACKET_TYPE::IMU_RAWDATA : 
		rawType == 1 ? PACKET_TYPE::IMU_QUAT : 
		rawType == 2 ? PACKET_TYPE::LIGHTSENSOR_RAWDATA : PACKET_TYPE::LIGHTHOUSE_SYNC;
	
	if (packetType == PACKET_TYPE::IMU_RAWDATA || packetType == PACKET_TYPE::IMU_QUAT)
	{
		int size = sizeof(imu_packet) / sizeof(uint8_t) - 4;
		blockReadData(buffer + 4, size);
		nextImuPacket = new imu_packet();
		*nextImuPacket = *((imu_packet*)(buffer));
		fetchedDataTimeStamp = nextImuPacket->timestamp;
		//printf("nextImuPacket->timestamp = %d \n", nextImuPacket->timestamp);
		fetchedDataType = TrackingDataType::IMU;
	}
	else if (packetType == PACKET_TYPE::LIGHTSENSOR_RAWDATA || packetType == PACKET_TYPE::LIGHTHOUSE_SYNC)
	{
		int size = sizeof(lightsensor_packet) / sizeof(uint8_t) - 4;
		blockReadData(buffer + 4, size);
		nextLightPacket = new lightsensor_packet();
		*nextLightPacket = *((lightsensor_packet*)(buffer));
		fetchedDataTimeStamp = nextLightPacket->timestamp;
		fetchedDataType = TrackingDataType::LightHouse;
	}

	if (InternalVariables::GetInstance()->GetInt("dv.replay.veryFastMode", 0) == 1)
	{
		// In very fast mode, advance the clock by record data.
		if (fetchedDataTimeStamp > TimeManager::GetMicroSeconds())
		{
			TimeManager::Advance(fetchedDataTimeStamp - TimeManager::GetMicroSeconds() + 1);
		}
	}
	
	if (!hasFirstDataArrived && fetchedDataTimeStamp < MAX_INT_NUM)
	{
		while (TimeManager::GetMicroSeconds() <= fetchedDataTimeStamp)
		{
			Sleep(5);
		}
		hasFirstDataArrived = true;
		timeStampDelta = TimeManager::GetMicroSeconds() - fetchedDataTimeStamp;
		fetchedDataTimeStamp = TimeManager::GetMicroSeconds();
		TimeManager::Advance(timeStampDelta);
	}


	if (fetchedDataTimeStamp > TimeManager::GetMicroSeconds())
	{
		return TrackingDataType::FAILED;
	}
	return fetchedDataType;
}

imu_packet * PresampleDataLoader::GetIMUData()
{
	if (ProceedData() != TrackingDataType::IMU)
		return NULL;
	fetchedDataType = TrackingDataType::FAILED; // Data consumed, set to unavailable.
	fetchedDataTimeStamp = MAX_INT_NUM;
	return nextImuPacket;
}

lightsensor_packet * PresampleDataLoader::GetLightSensorReport()
{
	if (ProceedData() != TrackingDataType::LightHouse)
		return NULL;
	fetchedDataType = TrackingDataType::FAILED; // Data consumed, set to unavailable.
	fetchedDataTimeStamp = MAX_INT_NUM;
	return nextLightPacket;
}
