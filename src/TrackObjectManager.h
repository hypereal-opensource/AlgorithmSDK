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
#ifdef ALGORITHMDLL_EXPORTS
#define ALGORITHMDLL_API __declspec(dllexport)
#else
#define ALGORITHMDLL_API __declspec(dllimport)
#endif
#include <map>
#include <vector>
#include <thread>
#include "ringbuffer.h"
#include "internal.h"
#include "SafeFlag.hpp"

namespace HYPEREAL
{
class TrackObject;

enum PoseResultFlag
{
	invalid = 0,
	quatValid = 1,
	transValid = 2
};

enum CalibrationState
{
	NoCaibrated,
	Calibrating,
	Ready
};
enum TrackObjectDeviceType
{
	HMD
};

// Flags for configuring track objects.
enum TrackObjectFlagsEnum
{
	DEFUALT = 0x00, // The most ordinary mode.
	// XXX = 0x02,
	// XXX = 0x04,
	// XXX = 0x08,
};
struct TrackObjectFlags
{
	uint32_t value = 0;
	TrackObjectFlags(uint32_t v) { value = v; }
	TrackObjectFlags(TrackObjectFlagsEnum e) { value = e; }
	inline bool isSet(TrackObjectFlagsEnum e) { return (value & e) != 0; }
	TrackObjectFlags operator| (TrackObjectFlags a) { return TrackObjectFlags(value | a.value); }
	TrackObjectFlags operator& (TrackObjectFlags a) { return TrackObjectFlags(value & a.value); }
};

enum CalibrationResult
{
	SUCCESS,
	FAILED,
	CALIOBJ_INVALID
};

typedef void(*VOIDFUNC)(CalibrationState);

class TrackObjectManager
{

public:
	static TrackObjectManager singletonInstance;
	static ALGORITHMDLL_API TrackObjectManager * GetInstance();

	// translation: n * 3,  rotation: n * 9
	CalibrationResult ALGORITHMDLL_API CalculateLHPose(int id, double translation[][3], double rotation[][9], VOIDFUNC callback = NULL);

	void ALGORITHMDLL_API SetLHPose(double translation[][3], double rotation[][9]);

	int ALGORITHMDLL_API AddTrackObject(TrackObjectDeviceType trackObjType, ringbuffer<imu_packet> * _imuRingBuffer, ringbuffer<lightsensor_packet> * _lightSensorRingBuffer, TrackObjectFlags flags, uint16_t versionMajor, uint16_t versionMinor);

	// Remove TrackObject. This function will stop the tracking, if applicable, and remove the trackObject's all data.
	void ALGORITHMDLL_API RemoveTrackObject(int trackObjectId);

	PoseResultFlag ALGORITHMDLL_API GetPose(int id, double* quat, double * translation, uint64_t timeStamp = -1);
	PoseResultFlag ALGORITHMDLL_API GetLighthousePoses(int lightHouseId, double* quat, double * translation);
	bool ALGORITHMDLL_API  GetGyroVec(int id, double gyroVec[3]);

	void ALGORITHMDLL_API Track(int id, bool isThreadJoin);

	ALGORITHMDLL_API const std::map<int, int> * GetVertexMap(int id);

	ALGORITHMDLL_API ~TrackObjectManager();

	ALGORITHMDLL_API bool IsDeviceStatic(int id);

	ALGORITHMDLL_API bool IsLHCalibrated();
	ALGORITHMDLL_API void WaitUntilLHCalibrated();
	// Notify TrackObjects to stop tracking, but the TrackObject remains.
	// To resume tracking, just call Track() again.
	ALGORITHMDLL_API void StopTracking();
private:
	SafeFlag hasLHCalibrated;
	CalibrationState _CalibrationState;
	std::map<int, TrackObject *>trackObjects;
	std::map<TrackObject *, std::thread *> caliThreads;
	std::map<TrackObject *, std::thread *> trackThreads;

	TrackObjectManager();

	TrackObject * GetTrackObject(int id);

	/// called after calibration
	void UpdateLightHouse(int lightHouseId, double *LHRotation, double * LHTranslation);

	int CreateTrackObjectID();
	void Init();
};
}
