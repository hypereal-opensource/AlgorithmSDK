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

#include "LighthouseInputProcessor.h"
#include "SensorDataInterpretor.h"
#include "InternalVariables.h"
#include "UdpPacket.hpp"
#include "UdpPacketChannel.h"
#include "internal.h"
#include "LogUtil.h"
#include "TrackObject.h"

namespace HYPEREAL
{
	LighthouseInputProcessor::LighthouseInputProcessor(std::shared_ptr<TrackingDataLoader> dataLoader)
		: dataLoader(dataLoader)
	{
		// pass
	}


	void LighthouseInputProcessor::Stop()
	{
		shallStop = true;
	}

	void LighthouseInputProcessor::loopProcess()
	{
		shallStop = false;
		const uint16_t sendPacketArrivalTimeUdpPort = InternalVariables::GetInstance()->GetInt("udp.sendPacketArrivalTime.port", 0);
		UdpPacketChannel<PlotDataUdpPacket<1>> channel("127.0.0.1", sendPacketArrivalTimeUdpPort);
		bool enableLogging = InternalVariables::GetInstance()->GetInt("logging.enable", 0) == 1;

		SensorDataInterpretor dataInterpretor(64, trackObject->VerNum, trackObject, trackObject->GetVertexMap());
		int preScanID = 0;
		std::shared_ptr<HitRecord> hitRecords[2];
		int hitNums[2] = { 0 };
		while (true)
		{
			{
				if (shallStop)
				{
					debug_print("Stop tracking object: %d\n", trackObject->id);
					break;
					return;
				}
			}
			TrackingDataType rtndata = dataLoader->ProceedData();
			if (TrackingDataType::FAILED == rtndata)
				continue;

			////////////////////////////////////////////////////////
			// IMU Part
			if (TrackingDataType::IMU == rtndata)
			{
				const int IMUdatasize = 2;
				std::shared_ptr<irrmath::vector3d> gyroVec = std::shared_ptr<irrmath::vector3d>(new irrmath::vector3d[IMUdatasize]);
				std::shared_ptr<imu_packet> imuData = std::shared_ptr<imu_packet>(dataLoader->GetIMUData());
				if (!imuData)
					continue;
				if (imuData->type == PACKET_TYPE::IMU_RAWDATA)
				{
					dataInterpretor.ProcessNewIMU(*imuData, gyroVec.get());
					for (int i = 0; i < IMUdatasize; i++)
					{
						{
							irrmath::vector3d vec = irrmath::vector3d(gyroVec.get()[i].x, gyroVec.get()[i].y, gyroVec.get()[i].z);
							irrmath::Quatd q;
							q.fromRotationVector(vec * 0.001);
							q = trackObject->_transIMU(q);
							q.toRotationVector(vec);
							vec *= 1000;
							gyroVec.get()[i] = irrmath::vector3d(vec.x, vec.y, vec.z);
						}
						irrmath::vector3d accVec(TransferAcceUnit(imuData->_imu.rawdata.Samples[i].Acce[0]),
							TransferAcceUnit(imuData->_imu.rawdata.Samples[i].Acce[1]),
							TransferAcceUnit(imuData->_imu.rawdata.Samples[i].Acce[2]));
						uint64_t timestamp = imuData->timestamp - 1000 * (1 - i);
						irrmath::vector3d magVec(imuData->_imu.rawdata.MagneticField[0],
							imuData->_imu.rawdata.MagneticField[1],
							imuData->_imu.rawdata.MagneticField[2]);
						trackObject->imuRotationModel->AddIMUPacket(timestamp, gyroVec.get()[i], accVec, magVec);

						// CALLBACK
						if (imuDataCallback)
						{
							AdditionalIMUData data;
							data.acce = accVec;
							data.gyro = *gyroVec;
							data.isValid = true;
							imuDataCallback(trackObject->imuRotationModel->GetLatestRotation(), data);
						}
					}
				}
				else if (imuData->type == PACKET_TYPE::IMU_QUAT)
				{
					irrmath::Quatd q;
					dataInterpretor.ProcessNewIMU(*imuData, &q);
					q = trackObject->_transIMU(q);
					trackObject->imuRotationModel->AddQuatIncrease(q);

					// CALLBACK
					if (imuDataCallback)
					{
						imuDataCallback(trackObject->imuRotationModel->GetLatestRotation(), AdditionalIMUData() /* invalid */);
					}
				}
				if (sendPacketArrivalTimeUdpPort != 0)
				{
					PlotDataUdpPacket<1> pkt;
					pkt.id = trackObject->trackObjectDeviceType;
					pkt.plotData[0] = (double)imuData->timestamp;
					channel.send(pkt);
				}
				continue;
			}

			////////////////////////////////////////////////////////
			// LightHouse Part
			auto lightSensorReport = dataLoader->GetLightSensorReport();
			if (!lightSensorReport)
				continue;
			int scanID = dataInterpretor.ProcessNewSlice(*lightSensorReport);
			auto lighthouseDataTimestamp = lightSensorReport->timestamp;
			delete lightSensorReport;
			if (-1 == scanID)
			{
				// LIGHTHOUSE_SYNC Data
				continue;
			}

			HitRecord* pScanRecords = nullptr;
			int nHitRecordNum = 0;

			if (1 == preScanID && 2 == scanID)
			{
				nHitRecordNum = dataInterpretor.CurrentFrameRecordsNum[1];
				pScanRecords = dataInterpretor.CurrentFrameRecords;

				hitRecords[0] = std::shared_ptr<HitRecord>(new HitRecord[nHitRecordNum]);
				hitNums[0] = nHitRecordNum;
				memcpy(hitRecords[0].get(), pScanRecords, sizeof(HitRecord) * nHitRecordNum);

				auto hitMap = LighthouseInputProcessor::HitRecordToHitMap(hitRecords[0].get(), nHitRecordNum);

				if (baseStationCallback)
				{
					baseStationCallback(0, lighthouseDataTimestamp, hitMap);
				}
			}
			else if (3 == preScanID && 4 == scanID)
			{
				nHitRecordNum = dataInterpretor.CurrentFrameRecordsNum[3] - dataInterpretor.CurrentFrameRecordsNum[1];
				pScanRecords = dataInterpretor.CurrentFrameRecords + dataInterpretor.CurrentFrameRecordsNum[1];

				hitRecords[1] = std::shared_ptr<HitRecord>(new HitRecord[nHitRecordNum]);
				hitNums[1] = nHitRecordNum;
				memcpy(hitRecords[1].get(), pScanRecords, sizeof(HitRecord) * nHitRecordNum);
				auto hitMap = LighthouseInputProcessor::HitRecordToHitMap(hitRecords[1].get(), nHitRecordNum);

				if (baseStationCallback)
				{
					baseStationCallback(1, lighthouseDataTimestamp, hitMap);
				}
			}

			// For e2e test.
			if (scanID == 4 || scanID == 2)
			{
				if (enableLogging)
				{
					// Log translation
					if (trackObject->res_translation != nullptr)
					{
						double x = trackObject->res_translation->x,
							y = trackObject->res_translation->y,
							z = trackObject->res_translation->z;
						std::stringstream ss;
						ss << std::to_string(x) << "," << std::to_string(y) << "," << std::to_string(z) << std::endl;
						LogUtil::getLogger("translation")->log(ss.str());
					}

					if (trackObject->res_quat != nullptr)
					{
						std::stringstream ss;
						irrmath::Quatd tempQuat = trackObject->imuRotationModel->GetLatestRotation();
						irrmath::vector3d ypr;
						tempQuat.toEuler(ypr);
						ss << std::to_string(ypr.x) << "," << std::to_string(ypr.y) << "," << std::to_string(ypr.z) << std::endl;
						LogUtil::getLogger("YawPitchRoll")->log(ss.str());
					}
				}
			}

			preScanID = scanID;
		}
	}

	std::shared_ptr<HitMap> LighthouseInputProcessor::HitRecordToHitMap(const HitRecord* hitRecords, const int hitRecordsNum)
	{
		auto map = std::make_shared<HitMap>();
		for (int i = 0; i < hitRecordsNum; i++)
		{
			const HitRecord* hit = hitRecords + i;
			const int sensorId = hit->id_v;
			const int lighthouseId = hit->id_LH;
			const bool isX = hit->is_x;
			const double angle = hit->angle;
			const uint32_t time = hit->time;
			if (map->getHitStatus(sensorId, lighthouseId) == NOT_HIT_BOTH)
			{
				SensorHitData hitData;
				hitData.hitStatus = SensorHitStatus::NOT_HIT_BOTH;
				hitData.xAngle = 0;
				hitData.yAngle = 0;
				map->data[std::make_pair(sensorId, lighthouseId)] = hitData;
			}
			SensorHitData& hitData = map->data.at(std::make_pair(sensorId, lighthouseId));
			if (isX)
			{
				hitData.xAngle = angle;
				hitData.hitStatus = hitData.hitStatus == SensorHitStatus::NOT_HIT_X ? SensorHitStatus::ACTIVE : SensorHitStatus::NOT_HIT_Y;
			}
			else
			{
				hitData.yAngle = angle;
				hitData.hitStatus = hitData.hitStatus == SensorHitStatus::NOT_HIT_Y ? SensorHitStatus::ACTIVE : SensorHitStatus::NOT_HIT_X;
			}
		}
		return map;
	}

	const std::pair<std::shared_ptr<HitRecord>, int> LighthouseInputProcessor::HitMapToHitRecords(std::shared_ptr<HitMap> hitMap)
	{
		std::vector<HitRecord> hitRecords;
		for each (auto it in hitMap->data)
		{
			bool hitX = false, hitY = false;
			switch (it.second.hitStatus)
			{
			case ACTIVE:
				hitX = hitY = true;
				break;
			case NOT_HIT_X:
				hitY = true;
				break;
			case NOT_HIT_Y:
				hitX = true;
				break;
			}

			// for x, y
			for (int i = 0; i < 2; ++i)
			{
				bool isX = (i == 0);
				if (isX && (it.second.hitStatus == ACTIVE || it.second.hitStatus == NOT_HIT_Y)
					|| !isX && (it.second.hitStatus == ACTIVE || it.second.hitStatus == NOT_HIT_X))
				{
					HitRecord hit;
					hit.is_x = isX;
					hit.id_v = it.first.first;
					hit.id_LH = it.first.second;
					hit.angle = isX ? it.second.xAngle : it.second.yAngle;
					hit.time = SensorHitData::angle2time(hit.angle);
					hitRecords.push_back(hit);
				}
			}
		}

		HitRecord *result = new HitRecord[hitRecords.size()];
		size_t idx = 0;
		for each (auto const &hit in hitRecords)
		{
			result[idx++] = hit;
		}
		return std::make_pair(std::shared_ptr<HitRecord>(result), hitRecords.size());
	}

	//  Unit mm/s^2
	float LighthouseInputProcessor::TransferAcceUnit(int16_t originAcceVal)
	{
		const float G = 9.8f;
		const float Max_Range = 32768;
		int gScale = 16;
		return float(originAcceVal / Max_Range * G * gScale * 1000);
	}

}
