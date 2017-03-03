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
#include <iostream>
#include <cstdio>
#include <fstream>
#include "TrackObjectManager.h"
#include "TrackObject.h"
#include "LightHouseData.h"
#include "InternalVariables.h"
#include "LogUtil.h"
#include "HVR_Logger.h"

#include "PreSampleDataLoader.h"
#include "RingBufferDataLoader.h"
#include "LighthouseInputProcessor.h"
#include "TO_HMD.h"

namespace HYPEREAL
{

TrackObjectManager TrackObjectManager::singletonInstance;
TrackObjectManager * TrackObjectManager::GetInstance()
{
	return &singletonInstance;
}

// translation: n * 3,  rotation: n * 9
CalibrationResult TrackObjectManager::CalculateLHPose(int id, double translation[][3], double rotation[][9], VOIDFUNC callback)
{
	if (id >= 0)
	{
		double LH1Translation[3], LH2Translation[3];
		double LH1Rotation[9], LH2Rotation[9];
		auto caliObject = trackObjects[id];
		debug_print("Calibration Light House Stage: Please keep Board and Lighthouse devices stable!!\n");
		auto res = caliObject->StartAutoCalibrating(LH1Rotation, LH1Translation, LH2Rotation, LH2Translation);
		for (int i = 0; i < 9; i++)
		{
			rotation[0][i] = LH1Rotation[i];
			rotation[1][i] = LH2Rotation[i];
		}
		for (int i = 0; i < 3; i++)
		{
			translation[0][i] = LH1Translation[i];
			translation[1][i] = LH2Translation[i];
		}
		return res;
	}
	return CalibrationResult::CALIOBJ_INVALID;
}

void TrackObjectManager::SetLHPose(double translation[][3], double rotation[][9])
{
	UpdateLightHouse(0, rotation[0], translation[0]);
	if (!InternalVariables::GetInstance()->GetInt("define.nIsSingleLightHouse", 0))
		UpdateLightHouse(1, rotation[1], translation[1]);
	hasLHCalibrated.set();
}

bool TrackObjectManager::IsDeviceStatic(int id)
{
	auto _trackObject = GetTrackObject(id);
	return _trackObject->IsDeviceStatic();
}


void TrackObjectManager::Init()
{
	// Initialize logging util, tell it workspace and config.
	static bool internalVariablesInitialized = false;
	if (!internalVariablesInitialized)
	{
		LogUtil::initialize(InternalVariables::GetInstance()->GetWorkspace(),
			InternalVariables::GetInstance()->GetInt("logging.enable", 0) == 1);
		internalVariablesInitialized = true;
	}
}

int TrackObjectManager::AddTrackObject(TrackObjectDeviceType trackObjType, ringbuffer<imu_packet> * _imuRingBuffer, ringbuffer<lightsensor_packet> * _lightSensorRingBuffer, TrackObjectFlags flags, uint16_t versionMajor, uint16_t versionMinor)
{
	Init();
	std::map<TrackObjectDeviceType, std::string> typeNameMap = {
		{ HMD, "DataHmd" }
	};

	std::shared_ptr<TrackingDataLoader> dataLoader;
	// Create data loader according to configuration.
	if (InternalVariables::GetInstance()->GetInt("define._SerialData", 2) == 0)
		dataLoader = std::shared_ptr<TrackingDataLoader>(new PresampleDataLoader(typeNameMap[trackObjType].c_str()));
	else
		dataLoader = std::shared_ptr<TrackingDataLoader>(new RingBufferDataLoader(_imuRingBuffer, _lightSensorRingBuffer, typeNameMap[trackObjType].c_str()));
	dataLoader->init();

	std::shared_ptr<AbstractInputProcessor> inputProcessor = std::make_shared<LighthouseInputProcessor>(dataLoader);

	int id = CreateTrackObjectID();
	TrackObject* obj =
		trackObjType == TrackObjectDeviceType::HMD ? (TrackObject *) new TO_HMD(id, flags, inputProcessor) :
		NULL;
	inputProcessor->SetTrackObject(obj);

	if (obj == NULL)
	{
		HVR_LOG_FATAL(alg_TrackObjectManager, "TrackObject's type is invalid: %d\n", trackObjType);
    }
	obj->SetTrackObjectManager(this);
	obj->SetId(id);
	trackObjects[id] = obj;
	trackObjects[id]->init();
	HVR_LOG_INFO(alg_TrackObjectManager, "AddTrackObject: %d\n", id);

	return id;
}


PoseResultFlag TrackObjectManager::GetPose(int id, double * quat, double * translation, uint64_t timeStamp)
{
	auto trackObj = GetTrackObject(id);

	// If not calibrated && with LH mode, return -1.
	if (!hasLHCalibrated.is_set())
	{
		return PoseResultFlag::invalid;
	}
	PoseResultFlag res = trackObj->GetPose(quat, translation, timeStamp);
	return res;
}

PoseResultFlag TrackObjectManager::GetLighthousePoses(int lightHouseId, double * quat, double * translation)
{
	auto lightHouseData = LightHouseData::GetInstance();
	PoseResultFlag res = lightHouseData->GetLightHousePoseWithQuat(lightHouseId, quat, translation);
	return res;
}

// Get Gyro Data
bool TrackObjectManager::GetGyroVec(int id, double gyroVec[3])
{
	auto trackObj = GetTrackObject(id);
	return(trackObj->GetGyroVec(id, gyroVec));
}

void TrackObjectManager::Track(int id, bool isThreadJoin)
{
	auto _trackObject = GetTrackObject(id);
	if (trackThreads.find(_trackObject) != trackThreads.end())
		return;

	trackThreads[_trackObject] = new std::thread([&, _trackObject]{
		_trackObject->Track();
	});
	if (isThreadJoin)
	{
		trackThreads[_trackObject]->join();
	}
}

TrackObjectManager::~TrackObjectManager()
{

}

void TrackObjectManager::RemoveTrackObject(int trackObjectId)
{
	TrackObject *trackObject = GetTrackObject(trackObjectId);
	if (trackObject == nullptr)
	{
		return;
	}
	trackObject->StopTracking();
	auto it = trackThreads.find(trackObject);
	if (it != trackThreads.cend())
	{
		it->second->join();
		delete it->second;
		trackThreads.erase(it);
	}
}

void TrackObjectManager::StopTracking()
{
	for each (auto it in trackObjects)
	{
		RemoveTrackObject(it.first);
	}
}

TrackObjectManager::TrackObjectManager()
{
	HVR_DECLARE_LOGGER(alg_TrackObjectManager);
	_CalibrationState = NoCaibrated;
	debug_print("This is internal version of TrackObjectManager, may contain unpublished features.\n");
}

TrackObject * TrackObjectManager::GetTrackObject(int id)
{
	if (trackObjects.find(id) == trackObjects.end())
	{
		return NULL;
	}
	return trackObjects[id];
}

void TrackObjectManager::UpdateLightHouse(int lightHouseId, double *LHRotation, double * LHTranslation)
{
	LightHouseData::GetInstance()->SetLightHousePose(lightHouseId, LHRotation, LHTranslation);
}

const std::map<int, int> * TrackObjectManager::GetVertexMap(int id)
{
	auto _trackObject = GetTrackObject(id);
	return _trackObject->GetVertexMap();
}

bool TrackObjectManager::IsLHCalibrated()
{
	return hasLHCalibrated.is_set();
}
void TrackObjectManager::WaitUntilLHCalibrated()
{
	hasLHCalibrated.wait();
}

int TrackObjectManager::CreateTrackObjectID()
{
	static int id = 0;
	return id++;
}

}

