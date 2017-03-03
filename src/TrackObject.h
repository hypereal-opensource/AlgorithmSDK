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

#include <iostream>
#include "SensorDataInterpretor.h"
#include <string>
#include "LHTrackingAlgorithm.h"
#include "opencv2/video/tracking.hpp"
#include "internal.h"
#include "DualQuat.h"
#include <map>
#include "TrackObjectManager.h"
#include "MoveDetector.h"
#include "ShiftArray.hpp"
#include "IMURotationModel.h"
//#include "quaternion.h"

namespace HYPEREAL
{

enum CalibrationRes
{
	LH1Valid = 1,
	LH2Valid = 2
};

class TrackObject
{
	friend class AbstractInputProcessor;
	friend class LighthouseInputProcessor;
public:
	enum class DeviceType
	{
		Unknown = -1,
		Wireless = 1,
		Wired
	};

	irrmath::Quatd quatLH;

	TrackObject(int id, TrackObjectFlags flags, std::shared_ptr<AbstractInputProcessor> inputProcessor, DeviceType deviceType);
	virtual ~TrackObject();
	void SetTrackObjectManager(TrackObjectManager* manager);
	void initKalman(const double predictionratio = 5, const double massureratio = 10, const double predictionratioSlow = 5, const double massureratioSlow = 5);
	virtual bool init();

	CalibrationResult StartAutoCalibrating(double* LH1Rotation, double* LH1Translation, double* LH2Rotation, double* LH2Translation);

	virtual void Track();
	// Notify the trackObject to stop looping tracking.
	virtual void StopTracking();

	virtual irrmath::Quatd _transIMU(irrmath::Quatd& quatNow) = 0;

	virtual void _transAcce(int16_t * inAcce, int16_t * outAcce);

	virtual void _transGyro(int16_t *inGyro, int16_t * outGyro);

	virtual void _transGyro(irrmath::vector3d & inGyro, irrmath::vector3d & outGyro);
	virtual irrmath::Quatd _invertTransIMU(irrmath::Quatd& quatNow) = 0;

	int VerNum;
	std::vector<double> vertices;
	std::vector<double> normals;

	std::string sProfileName;

	std::shared_ptr<AbstractInputProcessor> processor;
	SensorDataInterpretor* _dataInterpretor;
	irrmath::vector3d GyrCalibrateParam;

	LHTrackingAlgorithm _LHTrackingAlgorithm;

	cv::KalmanFilter KF_Solver;
	cv::Mat KF_measurement;

	double TrackTranslation[3];
	double TrackRotation[9];

	std::vector<double> _verticesRatioArray;

	const std::map<int, int>* GetVertexMap();

	irrmath::matrix4d array2Mat3d(double* mat);
	irrmath::vector3d array2Vec3d(double* vec);

	// qaut to column-major 3x3 array.
	void quat2Array(const irrmath::Quatd quat, double* mat);
	void vector2Array(const irrmath::vector3d vec3d, double* vec);

	PoseResultFlag GetPose(double * quat, double * translation, uint64_t timeStamp);
	bool GetGyroVec(int id, double gyroVec[3]);

	// return whether the TO is "static" or not
	bool IsDeviceStatic();

	void SetId(int _id);

	int GetId();
	TrackObjectFlags GetFlags();

protected:
	std::map<int, int>* _vertexMap = NULL;
	DeviceType deviceType;
	TrackObjectDeviceType trackObjectDeviceType;

private:
	void TryRecalibration(irrmath::vector3d * avgTrans = NULL);
	virtual void ResetStatus();
	TrackObjectManager* trackObjectManager;
	MoveDetector moveDetector;
	int id;
	int AutoCalibrateLightHouse(double* LH1Rotation, double* LH1Translation, double* LH2Rotation, double* LH2Translation);
	bool IsCalibrationValid(double* LHRotation, double* LHTranslation, std::vector<Vector3> * TranslationRecords);
	TrackObjectFlags flags;
	irrmath::vector3d *KF_translation;
	// quatDiff is initial quat diff between IMU&LH, so quatDiff * quatIMU = quatLH
	irrmath::Quatd quatDiffBetweenIMUAndLH;

	irrmath::vector3d * res_translation;
	irrmath::Quatd * res_quat;

	dualquatd poseHistory[2];
	uint64_t poseHistoryTime[2];
	int poseNum;

	uint64_t lastRecalibrationTime;

	double lastestGyro[2][3];


	std::shared_ptr<IMURotationModel> imuRotationModel;

	bool DoPrediction(const uint64_t timeStamp, irrmath::Quatd& quat, irrmath::vector3d& trans);
	bool DoPredictionWireless(const uint64_t timeStamp, irrmath::Quatd& quat, irrmath::vector3d& trans);
	bool DoPredictionHMD(const uint64_t timeStamp, irrmath::Quatd& quat, irrmath::vector3d& trans);

	std::mutex mtxPose;

	// Convenient method for GetPose.
	PoseResultFlag GetCurrentPose(irrmath::vector3d& translation, irrmath::Quatd& rotation);

	void TrackwithRotFusion();

	bool isSingleLighthouse;

	uint64_t lastTimeStamp;
	// false: the device is moving, true: dev has been placed static for staticTimesThreshold seconds
	bool IsDeviceStaticWhenCalibrating();

	void SetResQuat(const irrmath::Quatd* quat);
};
}
