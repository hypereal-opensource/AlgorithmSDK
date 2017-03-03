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

#include <math.h>
#include <iostream>
#include <fstream>
#include "SensorDataInterpretor.h"
#include "LogUtil.h"
#include "InternalVariables.h"
#include "TrackObject.h"

using HYPEREAL::InternalVariables;
namespace HYPEREAL
{
	SensorDataInterpretor::SensorDataInterpretor(const int slicedatasize, const int sensorNUM, TrackObject * _trackObject, const std::map<int, int>* _vertexMap)
	{
		CurrentFrameRecords = new HitRecord[sensorNUM * 4];
		for (int i = 0; i < 4; ++i)
		{
			CurrentFrameRecordsNum[i] = 0;
		}

		SLICEDATASIZE = slicedatasize;
		SENSORNUM = sensorNUM;

		vertexMap = _vertexMap;

		hasPreFrameRecords = false;
		this->_trackObject = _trackObject;
	}


	SensorDataInterpretor::~SensorDataInterpretor()
	{
		if (NULL != CurrentFrameRecords)
			delete[] CurrentFrameRecords;
		if (vertexMap)
		{
			delete vertexMap;
		}
	}


	int SensorDataInterpretor::ProcessNewSlice(lightsensor_packet & lightSensorReport, const irrmath::Quatd* quat)
	{
		int ReturnScanID = lightSensorReport.ScanAxis;
		int LH_ID;
		bool IS_X;
		switch (ReturnScanID)
		{
		case 1:
			LH_ID = 0;
			IS_X = false;
			break;
		case 2:
			LH_ID = 0;
			IS_X = true;
			break;
		case 3:
			LH_ID = 1;
			IS_X = false;
			break;
		case 4:
			LH_ID = 1;
			IS_X = true;
			break;
		default:
			return -1;
		}
		if (NULL != quat)
			CurrentFrameQuat[ReturnScanID - 1] = *quat;
		else
			CurrentFrameQuat[ReturnScanID - 1] = irrmath::Quatd(0, 0, 0, 1);


		if (1 == ReturnScanID)	//Split Frame
		{
			if (!hasPreFrameRecords)
			{
				hasPreFrameRecords = true;
			}
			else
			{
				memset(CurrentFrameRecordsNum, 0, sizeof(CurrentFrameRecordsNum));
			}
		}

		int CurRecordNum = (1 == ReturnScanID) ? 0 : CurrentFrameRecordsNum[ReturnScanID - 2];

		for (int i = 0; i < lightSensorReport.SampleCount; i++)
		{
			HitRecord& curRec = CurrentFrameRecords[CurRecordNum];
			curRec.id_LH = LH_ID;
			curRec.is_x = IS_X;
			if (vertexMap)
				curRec.id_v = vertexMap->at(lightSensorReport.LightSensor[i].SensorID);
			else
				curRec.id_v = lightSensorReport.LightSensor[i].SensorID;
			unsigned int timetick = lightSensorReport.LightSensor[i].MeasureResult;
			curRec.angle = TimetickToAngle(timetick);
			curRec.time = timetick;
			++CurRecordNum;
		}

		for (int i = ReturnScanID - 1; i < 4; ++i)
			CurrentFrameRecordsNum[i] = CurRecordNum;

		if (ReturnScanID == 2)
		{
			using namespace std;
			map<int, int> sensorHitCount;
			map<int, size_t> sensorIdToIdxX;
			map<int, size_t> sensorIdToIdxY;
			for (int i = 0; i < CurRecordNum; i++)
			{
				if (CurrentFrameRecords[i].id_LH == 1)
				{
					continue;
				}
				sensorHitCount[CurrentFrameRecords[i].id_v]++;
				if (CurrentFrameRecords[i].is_x)
				{
					sensorIdToIdxX[CurrentFrameRecords[i].id_v] = i;
				}
				else
				{
					sensorIdToIdxY[CurrentFrameRecords[i].id_v] = i;
				}

			}
		}

		if (ReturnScanID == 4)
		{
			using namespace std;
			map<int, int> sensorHitCount;
			map<int, size_t> sensorIdToIdxX;
			map<int, size_t> sensorIdToIdxY;
			for (int i = 0; i < CurRecordNum; i++)
			{
				if (CurrentFrameRecords[i].id_LH == 0)
				{
					continue;
				}
				sensorHitCount[CurrentFrameRecords[i].id_v]++;
				if (CurrentFrameRecords[i].is_x)
				{
					sensorIdToIdxX[CurrentFrameRecords[i].id_v] = i;
				}
				else
				{
					sensorIdToIdxY[CurrentFrameRecords[i].id_v] = i;
				}
			}
		}

		return ReturnScanID;
	}

	int SensorDataInterpretor::ProcessNewIMU(imu_packet & imuReport, irrmath::vector3d* outGyro)
	{
		if (imuReport.type == PACKET_TYPE::IMU_RAWDATA)
		{
			const int IMUdatasize = 2;
			for (int i = 0; i < IMUdatasize; ++i)
			{
				_trackObject->_transAcce(imuReport._imu.rawdata.Samples[i].Acce, imuReport._imu.rawdata.Samples[i].Acce);
				_trackObject->_transGyro(imuReport._imu.rawdata.Samples[i].Gyro, imuReport._imu.rawdata.Samples[i].Gyro);
				for (int j = 0; j < 3; ++j)
				{
					static const double ratioGRYRO = 1 / 32768.0f * 2000.0f / 180 * M_PI;
					int16_t gyrotick = imuReport._imu.rawdata.Samples[i].Gyro[j];
					outGyro[i][j] = gyrotick * ratioGRYRO;
				}
			}
			return 1;
		}
		return -1;
	}

	int SensorDataInterpretor::ProcessNewIMU(imu_packet & imuReport, irrmath::Quatd* outQuat)
	{
		if (imuReport.type == PACKET_TYPE::IMU_QUAT)
		{
			outQuat->x = imuReport._imu.QuatIncrement[0];
			outQuat->y = imuReport._imu.QuatIncrement[1];
			outQuat->z = imuReport._imu.QuatIncrement[2];
			outQuat->w = sqrt(1 - outQuat->x * outQuat->y - outQuat->y * outQuat->y - outQuat->z * outQuat->z);
			return 1;
		}
		return -1;
	}

	double SensorDataInterpretor::TimetickToAngle(uint32_t timetick)
	{
		return ((timetick - 18000.0f) * 60 / (5.25 * 1e6)) * 2 * M_PI + M_PI_2;
	}
	uint32_t SensorDataInterpretor::AngleToTimetick(double angle)
	{
		return uint32_t((angle - M_PI_2) / 2 / M_PI * (5.25 * 1e6) / 60 + 18000.0f);
	}
}
