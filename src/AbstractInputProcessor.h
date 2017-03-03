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

#include <functional>
#include <memory>
#include <atomic>
#include "quaternion.h"
#include "internal.h"


namespace HYPEREAL
{
	class TrackObject;

	// Sensor hit map, containing which point is hit by what base station.
	class HitMap
	{
	public:
		std::map<std::pair<int, int>, SensorHitData> data;
		// Return whether (sensorId, bsId) == ACTIVE
		inline bool isHit(int sensorId, int bsId)
		{
			return getHitStatus(sensorId, bsId) == ACTIVE;
		}

		inline SensorHitStatus getHitStatus(int sensorId, int bsId)
		{
			auto it = data.find(std::make_pair(sensorId, bsId));
			return it == data.cend() ? NOT_HIT_BOTH : it->second.hitStatus;
		}

		inline const SensorHitData& get(int sensorId, int bsId)
		{
			return data.at(std::make_pair(sensorId, bsId));
		}

		// Merge with another HitMap. value with same key will be overridden by others.
		void Merge(const HitMap& other);

		// Clear data specified base station id.
		void ClearBaseStation(int bsId);
	};

	// Unit: acc (mm/s^2), gyro (radian/s).
	struct AdditionalIMUData
	{
		bool isValid = false;
		irrmath::vector3d acce;
		irrmath::vector3d gyro;
	};

	// Callback when new IMU data arrives. quatNow is maintained by trackObject's IMURotationModel.
	typedef std::function<void(const irrmath::Quatd& quatNow, const AdditionalIMUData &imuData)> ImuDataCallback;
	// Callback when new base station (LH or camera) data arrives;
	// Unit: bsId starts from zero. timestamp (micro second).
	typedef std::function<void(int bsId, const uint64_t &timestamp, std::shared_ptr<HitMap> hitMap)> BaseStationCallback;

	// Interface of InputProcessor, which handles raw input, e.g. imu_packet, camera_image_packet.
	// InputProcessor converts raw data into TrackObject-readable data, like points with id, translated 
	// IMU data.
	// Input processor could be register some callbacks, e.g. onImuData, onBaseStationData. Thus trackObjects
	// could use these callbacks to extract new pose.
	class AbstractInputProcessor
	{
	public:
		void inline RegisterImuDataCallback(ImuDataCallback cb)
		{
			imuDataCallback = cb;
		}

		void inline RegisterBaseStationCallback(BaseStationCallback cb)
		{
			baseStationCallback = cb;
		}

		// Let the processor know what trackObject it is serving.
		// Note that trackObject holds inputProcessor, so we use the raw pointer.
		void inline SetTrackObject(TrackObject *trackOjbect)
		{
			this->trackObject = trackOjbect;
		}

		// 
		virtual void loopProcess() = 0;

		// Stop loopRun, usually set shallStop to true, and in loopRun, detect it on each loop.
		// Don't forget to reset shallStop in each begin of loopRun.
		virtual void Stop() = 0;
	protected:
		std::atomic_bool shallStop;
		ImuDataCallback imuDataCallback;
		BaseStationCallback baseStationCallback;
		// Referencing about TO specific operations.
		TrackObject * trackObject;
	};
}
