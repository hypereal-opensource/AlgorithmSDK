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
#include "TrackObject.h"
#include "SensorDataInterpretor.h"

#include <iostream>
#include "LHTrackingAlgorithm.h"
#include <fstream>
#include "TimeManager.h"
#include "LogUtil.h"
#include "InternalVariables.h"
#include "ShiftArray.hpp"
#include "LightHouseData.h"
#include "UdpPacketChannel.h"
#include "AbstractInputProcessor.h"
#include "LighthouseInputProcessor.h"

using namespace std;
namespace HYPEREAL
{

	TrackObject::TrackObject(int id, TrackObjectFlags flags, std::shared_ptr<AbstractInputProcessor> inputProcessor, DeviceType deviceType)
		: KF_Solver(6, 3, 0, CV_64F), id(id), flags(flags), processor(inputProcessor), deviceType(deviceType)
	{
		VerNum = 0;

		poseNum = 0;
		res_quat = NULL;
		res_translation = NULL;

		KF_translation = NULL;

    imuRotationModel = std::make_shared<IMURotationModel>(
			deviceType == DeviceType::Wired ? IMURotationModel::Mode::RAW_IMU : IMURotationModel::Mode::QUAT_INC
		);

		isSingleLighthouse = InternalVariables::GetInstance()->GetInt("define.nIsSingleLightHouse", 0) == 1;

		lastRecalibrationTime = 0;
	}

	bool TrackObject::init()
	{
		_dataInterpretor = new SensorDataInterpretor(64, VerNum, this, _vertexMap);
		return true;
	}

	void TrackObject::ResetStatus()
	{
		memset(TrackTranslation, 0, sizeof(TrackTranslation));
		memset(TrackRotation, 0, sizeof(TrackRotation));
		TrackRotation[0] = TrackRotation[4] = TrackRotation[8] = 1.0;
	}

	void TrackObject::SetTrackObjectManager(TrackObjectManager* manager)
	{
		trackObjectManager = manager;
	}

	void TrackObject::StopTracking()
	{
		processor->Stop();
	}

	TrackObject::~TrackObject()
	{

	}

	CalibrationResult TrackObject::StartAutoCalibrating(double* LH1Rotation, double* LH1Translation, double* LH2Rotation, double* LH2Translation)
	{
		const int LAST_ACCE_SIZE = 2;
		double latestAcce[LAST_ACCE_SIZE][3];
		bool hasNewAcceData = false;
		bool isValid = false;

		auto onImuData = [&](const irrmath::Quatd& imuQuatNow, const AdditionalIMUData &imuData)
		{
			hasNewAcceData = true;
			const irrmath::vector3d &acceVec = imuData.acce;
			const irrmath::vector3d &gyroVec = imuData.gyro;
			for (int j = 0; j < 3; j++)
			{
				// shift the array
				for (int k = 0; k < LAST_ACCE_SIZE - 1; k++)
				{
					latestAcce[k][j] = latestAcce[k + 1][j];
					lastestGyro[k][j] = lastestGyro[k + 1][j];
				}
				latestAcce[LAST_ACCE_SIZE - 1][j] = acceVec[j];
				lastestGyro[LAST_ACCE_SIZE - 1][j] = gyroVec[j];
			}
		};

		auto onOneLightHouseData = [&](int lhId, const uint64_t &lighthouseDataTimestamp, std::shared_ptr<HitMap> hitMap)
		{
			if (!hasNewAcceData)
				return;

			auto result = LighthouseInputProcessor::HitMapToHitRecords(hitMap);
			std::shared_ptr<HitRecord> rawHitRecords = result.first;
			int hitRecordsNum = result.second;

			std::pair<std::shared_ptr<HitRecord>, int> recordsPair = make_pair(rawHitRecords, hitRecordsNum);
			moveDetector.AddAccData(latestAcce[0][0], latestAcce[0][1], latestAcce[0][2], recordsPair);
			hasNewAcceData = false;

			if (IsDeviceStaticWhenCalibrating())
			{
				auto result = AutoCalibrateLightHouse(LH1Rotation, LH1Translation, LH2Rotation, LH2Translation);
				isValid = isSingleLighthouse
					? result == CalibrationRes::LH1Valid
					: result == (CalibrationRes::LH1Valid | CalibrationRes::LH2Valid);
				moveDetector.ClearStaicLightSensorDatas();
				processor->Stop();
			}
		};

		processor->RegisterImuDataCallback(onImuData);
		processor->RegisterBaseStationCallback(onOneLightHouseData);
		processor->loopProcess();

		if (!isValid)
		{
			moveDetector.Reset();
			debug_print("Calibration Failed, Try again!\n");
			return CalibrationResult::FAILED;

		}
		else
		{
			debug_print("Calibration Success!\n");
			return CalibrationResult::SUCCESS;
		}
	}

	bool TrackObject::IsCalibrationValid(double* LHRotation, double* LHTranslation, std::vector<Vector3> * TranslationRecords)
	{
		double minAbsDouble = 0.000000001;
		// 20m
		double maxValidValue = 20000;

		// 1cm
		double stddevThreshold = InternalVariables::GetInstance()->GetDouble("CalibrationStddevThreshold", 3);
		int lhNum = 1;
		if (!isSingleLighthouse)
			lhNum = 2;
		bool isQuatValid = false;
		bool isTransValid = true;
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
			{
				double val = 0;
				if (i == j)
					val = 1;
				if (abs(LHRotation[i + 3 * j] - val) > minAbsDouble)
				{
					isQuatValid = true;
				}
			}
		for (int i = 0; i < 3; i++)
		{
			if (abs(LHTranslation[i]) > maxValidValue)
			{
				isTransValid = false;
			}
		}

		std::vector<double> distanceArray;
		for each (auto trans in *TranslationRecords)
		{
			double x = trans.x;
			double y = trans.y;
			double z = trans.z;
			double distance = sqrt(pow(x - LHTranslation[0], 2) + pow(y - LHTranslation[1], 2) + pow(z - LHTranslation[2], 2));
			distanceArray.push_back(distance);
		}
		cv::Scalar meanScalar, stddevScalar;
		meanStdDev(distanceArray, meanScalar, stddevScalar);
		auto mean = meanScalar[0];
		auto stddev = stddevScalar[0];
		bool isDataStddevScalarValid = true;
		if (stddev > stddevThreshold)
			isDataStddevScalarValid = false;
		debug_print("LH mean = %.2lf, stddev = %.2lf\n", mean, stddev);
		return isTransValid && isQuatValid && isDataStddevScalarValid;
	}

	int TrackObject::AutoCalibrateLightHouse(double* LH1Rotation, double* LH1Translation, double* LH2Rotation, double* LH2Translation)
	{
		irrmath::Quatd LH1Quat(0, 0, 0, 0), LH2Quat(0, 0, 0, 0);
		irrmath::vector3d LH1Vector(0, 0, 0), LH2Vector(0, 0, 0);
		std::vector<Vector3> TranslationRecords1, TranslationRecords2;
		int stageCounterLh1 = 0, stageCounterLh2 = 0;

		auto staicLightSensorDatas = moveDetector.GetStaicLightSensorDatas();
		size_t CALITIME = staicLightSensorDatas.size();

		for (auto it = staicLightSensorDatas.begin(); it != staicLightSensorDatas.end(); it++)
		{
			auto lightSensorReport = *it;
			int nHitRecordNum = lightSensorReport.second;
			auto hitRecords = lightSensorReport.first;
			int lhId = hitRecords->id_LH;


			double* pLHTranslation = LH1Translation;
			double* pLHRotation = LH1Rotation;
			irrmath::Quatd* pLHQuat = &LH1Quat;
			irrmath::vector3d* pLHVector = &LH1Vector;
			std::vector<Vector3>* TranslationRecords = &TranslationRecords1;

			int* stageCounter = nullptr;
			if (lhId == 0)
			{
				stageCounter = &stageCounterLh1;
			}
			else if (lhId == 1)
			{
				pLHTranslation = LH2Translation;
				pLHRotation = LH2Rotation;
				TranslationRecords = &TranslationRecords2;
				pLHQuat = &LH2Quat;
				pLHVector = &LH2Vector;
				stageCounter = &stageCounterLh2;
			}

			if (lhId == 0 || lhId == 1)
			{
				Vector3 solveTranslation;
				irrmath::Quatd& LHQuat = *pLHQuat;
				irrmath::vector3d& LHVector = *pLHVector;
				std::map<int, SensorHitStatus> outliers;
				irrmath::Quatd solveQuat;
				auto hitMap = LighthouseInputProcessor::HitRecordToHitMap(hitRecords.get(), nHitRecordNum);
				double tempTrans[] = { 0,0,0 };
				double tempRot[] = { 1,0,0,0,1,0,0,0,1 };
				_LHTrackingAlgorithm.SolvePnPOrIterative(&vertices[0], VerNum, hitMap, lhId, tempRot, tempTrans, false, solveQuat, solveTranslation.val);

				TranslationRecords->push_back(solveTranslation);
				if (0 == *stageCounter)
				{
					LHQuat = solveQuat;
					LHVector = array2Vec3d(solveTranslation.val);
				}
				else
				{
					solveQuat.normalize();
					solveQuat.ensureSameHemisphere(LHQuat);

					LHQuat.lerp(LHQuat, solveQuat, 1 - (double)*stageCounter / (*stageCounter + 1.0));
					LHVector += (array2Vec3d(solveTranslation.val) - LHVector) / (double)(*stageCounter + 1);

				}
				(*stageCounter)++;
			}
		}

		quat2Array(LH1Quat, LH1Rotation);
		vector2Array(LH1Vector, LH1Translation);
		debug_print("Calibration Finish!!\n");
		debug_print("Light House 1 :\n");
		debug_print("CaliRotation:\t %f %f %f %f\n", LH1Quat.x, LH1Quat.y, LH1Quat.z, LH1Quat.w);
		debug_print("CaliTranslation:\t %f %f %f\n", LH1Vector.x, LH1Vector.y, LH1Vector.z);

		if (!isSingleLighthouse)
		{
			quat2Array(LH2Quat, LH2Rotation);
			vector2Array(LH2Vector, LH2Translation);
			debug_print("Light House 2 :\n");
			debug_print("CaliRotation:\t %f %f %f %f\n", LH2Quat.x, LH2Quat.y, LH2Quat.z, LH2Quat.w);
			debug_print("CaliTranslation:\t %f %f %f\n", LH2Vector.x, LH2Vector.y, LH2Vector.z);
		}

		int res = 0;
		if (IsCalibrationValid(LH1Rotation, LH1Translation, &TranslationRecords1))
			res |= CalibrationRes::LH1Valid;
		if (!isSingleLighthouse && IsCalibrationValid(LH2Rotation, LH2Translation, &TranslationRecords2))
			res |= CalibrationRes::LH2Valid;
		return res;
	}

	void TrackObject::TryRecalibration(irrmath::vector3d * avgTrans)
	{
		auto ls = clock();
		double newLH1Rotation[9];
		double newLH2Rotation[9];
		double newLH1Translation[3];
		double newLH2Translation[3];

		if (TimeManager::GetMicroSeconds() - lastRecalibrationTime > 1000000)
		{
			int res = AutoCalibrateLightHouse(newLH1Rotation, newLH1Translation, newLH2Rotation, newLH2Translation);
			int isLH1Valid = res & CalibrationRes::LH1Valid;
			int isLH2Valid = res & CalibrationRes::LH2Valid;
			if (isLH1Valid)
			{
				irrmath::matrix4d MatRLH;
				for (int i = 0; i < 3; i++)
					for (int j = 0; j < 3; j++)
						MatRLH.at(i, j) = newLH1Rotation[j * 3 + i];
				auto ivMatRLH = MatRLH.inverted();

				irrmath::vector3d matTrans(newLH1Translation[0], newLH1Translation[1], newLH1Translation[2]);

				auto resT = ivMatRLH * matTrans;

				auto rotInHmdCoord = ivMatRLH;
				auto transInHmdCoord = irrmath::vector3d(-resT[0], -resT[1], -resT[2]);

				double hmdTrans[3];
				double hmdQuat[4];
				int res = GetPose(hmdQuat, hmdTrans, TimeManager::GetMicroSeconds());
				if (res == 2)
				{
					irrmath::Quatd _hmdQuat(hmdQuat[0], hmdQuat[1], hmdQuat[2], hmdQuat[3]);
					irrmath::matrix4d MatHmdRot = _hmdQuat.getMatrix();
					auto rotWorld = irrmath::Quatd(MatHmdRot * rotInHmdCoord);

					irrmath::vector3d _hmdTrans(hmdTrans[0], hmdTrans[1], hmdTrans[2]);
					if (avgTrans)
					{
						_hmdTrans = *avgTrans;
					}

					auto transWorld = MatHmdRot * transInHmdCoord + _hmdTrans;
					std::cout << "LH1 in world, position = " << transWorld.x << ", " << transWorld.y << ", " << transWorld.z << "\n";
					std::cout << "rotation = " << rotWorld.x << ", " << rotWorld.y << ", " << rotWorld.z << ", " << rotWorld.w << "\n";
				}
				else
				{
					std::cout << "hmd pose invalid\n";
				}

			}
			if (isLH2Valid)
			{
				irrmath::matrix4d MatRLH;
				for (int i = 0; i < 3; i++)
					for (int j = 0; j < 3; j++)
						MatRLH.at(i, j) = newLH2Rotation[j * 3 + i];
				auto ivMatRLH = MatRLH.inverted();

				irrmath::vector3d matTrans(newLH2Translation[0], newLH2Translation[1], newLH2Translation[2]);

				auto resT = ivMatRLH * matTrans;

				auto rotInHmdCoord = ivMatRLH;
				auto transInHmdCoord = irrmath::vector3d(-resT[0], -resT[1], -resT[2]);

				double hmdTrans[3];
				double hmdQuat[4];
				int res = GetPose(hmdQuat, hmdTrans, TimeManager::GetMicroSeconds());
				if (res == 2)
				{
					irrmath::Quatd _hmdQuat(hmdQuat[0], hmdQuat[1], hmdQuat[2], hmdQuat[3]);
					irrmath::matrix4d MatHmdRot = _hmdQuat.getMatrix();
					auto rotWorld = irrmath::Quatd(MatHmdRot * rotInHmdCoord);

					irrmath::vector3d _hmdTrans(hmdTrans[0], hmdTrans[1], hmdTrans[2]);

					auto transWorld = MatHmdRot * transInHmdCoord + _hmdTrans;
				}
				else
				{
					std::cout << "hmd pose invalid\n";
				}

			}
			if (isLH1Valid || isLH2Valid)
			{
				lastRecalibrationTime = TimeManager::GetMicroSeconds();
			}
		}


		moveDetector.ClearStaicLightSensorDatas();
		moveDetector.Reset();
	}

	void TrackObject::initKalman(const double predictionratio, const double massureratio, const double predictionratioSlow, const double massureratioSlow)
	{
    KF_Solver.transitionMatrix = (cv::Mat_<double>(6, 6) << 1, 0, 0, 1, 0, 0,
      0, 1, 0, 0, 1, 0,
      0, 0, 1, 0, 0, 1,
      0, 0, 0, 1, 0, 0,
      0, 0, 0, 0, 1, 0,
      0, 0, 0, 0, 0, 1);

    KF_measurement = cv::Mat::zeros(3, 1, CV_64F);

    //init value
    for (int i = 0; i < 6; ++i)
    {
      KF_Solver.statePost.at<double>(i) = 0;
    }
    KF_Solver.measurementMatrix = (cv::Mat_<double>(3, 6) <<      // H
      1, 0, 0, 0, 0, 0,
      0, 1, 0, 0, 0, 0,
      0, 0, 1, 0, 0, 0);

    setIdentity(KF_Solver.processNoiseCov, cv::Scalar::all(predictionratio));
    setIdentity(KF_Solver.errorCovPost, cv::Scalar::all(.1));
    setIdentity(KF_Solver.measurementNoiseCov, cv::Scalar::all(massureratio));
	}

	void TrackObject::Track()
	{
		ResetStatus();
		trackObjectManager->WaitUntilLHCalibrated();

		TrackwithRotFusion();
	}

	void TrackObject::SetResQuat(const irrmath::Quatd* quat) {
		if (!res_quat)
			res_quat = new irrmath::Quatd();
		*res_quat = *quat;
	};

	void TrackObject::TrackwithRotFusion()
	{
		debug_print("TrackObject %d started with RotFusion.\n", id);
		irrmath::vector3d gyroDiff(0, 0, 0);
		quatDiffBetweenIMUAndLH = irrmath::Quatd();
		int lhFrameCount = 0;
		ShiftArray<irrmath::vector3d> poseShiftArray(10);

		ShiftArray<irrmath::vector3d> poseHistoryArray(int(InternalVariables::GetInstance()->GetDouble("staticTimesShortThreshold", 250) * 1000 / 16));

		const int LAST_ACCE_SIZE = 2;
		double latestAcce[LAST_ACCE_SIZE][3];
		bool hasNewAcceData = false;
		int recvNewSensorDataNum = 0;

		auto onImuData = [&](const irrmath::Quatd& imuQuatNow, const AdditionalIMUData &imuData)
		{
			double LH1Rotation[9];
			double LH2Rotation[9];
			double LH1Translation[3];
			double LH2Translation[3];
			LightHouseData::GetInstance()->GetLightHousePoseWithRotMat(0, LH1Rotation, LH1Translation);
			LightHouseData::GetInstance()->GetLightHousePoseWithRotMat(1, LH2Rotation, LH2Translation);
			if (imuData.isValid) // PACKET_TYPE == IMU_RAWDATA, type = Wired
			{
				hasNewAcceData = true;
				const irrmath::vector3d &acceVec = imuData.acce;
				const irrmath::vector3d &gyroVec = imuData.gyro;
				for (int j = 0; j < 3; j++)
				{
					// shift the array
					for (int k = 0; k < LAST_ACCE_SIZE - 1; k++)
					{
						latestAcce[k][j] = latestAcce[k + 1][j];
						lastestGyro[k][j] = lastestGyro[k + 1][j];
					}
					latestAcce[LAST_ACCE_SIZE - 1][j] = acceVec[j];
					lastestGyro[LAST_ACCE_SIZE - 1][j] = gyroVec[j];
				}

			}
			std::lock_guard<std::mutex> lock(mtxPose);
			SetResQuat(&imuQuatNow);
		};

		int solvertnLH1 = 0, solvertnLH2 = 0;
		irrmath::Quatd quatLH1(0, 0, 0, 1);
		irrmath::Quatd quatLH2(0, 0, 0, 1);
		double transLH1[3], transLH2[3];
		int lh1HitNum = 0, lh2HitNum = 0;
		std::map<int, SensorHitStatus> lh1Outliers, lh2Outliers;
		auto countHitPoints = [&](std::shared_ptr<HitMap> hitMap, int lhId) -> int {
			int count = 0;
			for (int i = 0; i < VerNum; i++)
			{
				if (hitMap->isHit(i, lhId))
				{
					count++;
				}
			}
			return count;
		};

		int hitPointNumThreshold = 5;
		uint64 lastOneLHTimeStamp = TimeManager::GetMicroSeconds();

		// Forward declaration. Will be defined later.
		std::function<void(int lastBsId, uint64_t timestamp)> Dofusion;
		std::shared_ptr<HitMap> debugHitMap = std::make_shared<HitMap>();
		auto onOneLightHouseData = [&](int lhId, const uint64_t &lighthouseDataTimestamp, std::shared_ptr<HitMap> hitMap)
		{
			auto result = LighthouseInputProcessor::HitMapToHitRecords(hitMap);
			std::shared_ptr<HitRecord> rawHitRecords = result.first;
			int hitRecordsNum = result.second;

			lastOneLHTimeStamp = lighthouseDataTimestamp;

			double LH1Rotation[9];
			double LH2Rotation[9];
			double LH1Translation[3];
			double LH2Translation[3];

			LightHouseData::GetInstance()->GetLightHousePoseWithRotMat(0, LH1Rotation, LH1Translation);
			LightHouseData::GetInstance()->GetLightHousePoseWithRotMat(1, LH2Rotation, LH2Translation);

			// for recalibration
			if (hasNewAcceData)
			{
				std::pair<std::shared_ptr<HitRecord>, int> recordsPair = make_pair(rawHitRecords, hitRecordsNum);
				moveDetector.AddAccData(latestAcce[0][0], latestAcce[0][1], latestAcce[0][2], recordsPair);
				hasNewAcceData = false;
			}

			// For calibrating IMU's orientation by LH PnP.
			Vector3 caliSolveTranslation;
			Matrix33 caliSolveRotation;
			// useThreeLayerStrategy:
			// The strategy will use solvePnP (EPNP) if hit points >= hitPointNumThreshold.
			// Otherwise, it uses PNP_ITERATIVE to solve. If ITERATIVE fails, use SolvePnPWithRotation instead.
			if (lhId == 0)
			{
				solvertnLH1 = 0;
				// If useThreeLayerStrategy, hit points must >= hitPointNumThreshold.
				// Otherwise, always enter the loop.
				if (countHitPoints(hitMap, lhId) >= hitPointNumThreshold)
				{
					lh1HitNum = hitRecordsNum;
					irrmath::Quatd rotation;
					// Use EPNP
					solvertnLH1 = _LHTrackingAlgorithm.SolvePnPOrIterative(&vertices[0], VerNum, hitMap, lhId, LH1Rotation, LH1Translation, false, quatLH1, transLH1);
				}

				if (solvertnLH1 == 0)
				{
					irrmath::vector3d translation;
					irrmath::Quatd rotation;
					PoseResultFlag ret = GetCurrentPose(translation, rotation);
					for (int i = 0; i < 3; i++)
					{
						transLH1[i] = translation[i];
					}
					solvertnLH1 = _LHTrackingAlgorithm.SolvePnPOrIterative(&vertices[0], VerNum, hitMap, lhId, LH1Rotation, LH1Translation, true, rotation, transLH1);
					if (!solvertnLH1)
					{
						quatLH1 = imuRotationModel->GetLatestRotation();
						// SolvePnP failed, use SolvePnPWithRotation instead.
						double pRotation[9];
						quat2Array(imuRotationModel->GetLatestRotation(), pRotation);

						Vector3 outTrans;
						int solvertnLH1 = _LHTrackingAlgorithm.SolvePnPWithRotation(&vertices[0], VerNum, hitMap, lhId, LH1Rotation, LH1Translation, pRotation, outTrans);
						if (solvertnLH1)
						{
							memcpy(transLH1, outTrans.val, sizeof(double) * 3);
						}
					}
				}
			}
			else if (lhId == 1)
			{
				solvertnLH2 = 0;
				// If useThreeLayerStrategy, hit points must >= hitPointNumThreshold.
				// Otherwise, always enter the loop.
				if (countHitPoints(hitMap, lhId) >= hitPointNumThreshold)
				{
					lh2HitNum = hitRecordsNum;
					irrmath::Quatd rotation;
					// Use EPNP
					solvertnLH2 = _LHTrackingAlgorithm.SolvePnPOrIterative(&vertices[0], VerNum, hitMap, lhId, LH2Rotation, LH2Translation, false, quatLH2, transLH2);
				}

				if (solvertnLH2 == 0)
				{
					irrmath::vector3d translation;
					irrmath::Quatd rotation;
					PoseResultFlag ret = GetCurrentPose(translation, rotation);
					for (int i = 0; i < 3; i++)
					{
						transLH2[i] = translation[i];
					}
					solvertnLH2 = _LHTrackingAlgorithm.SolvePnPOrIterative(&vertices[0], VerNum, hitMap, lhId, LH2Rotation, LH2Translation, true, rotation, transLH2);
					if (!solvertnLH2)
					{
						quatLH2 = imuRotationModel->GetLatestRotation();
						// SolvePnP failed, use SolvePnPWithRotation instead.
						double pRotation[9];
						quat2Array(imuRotationModel->GetLatestRotation(), pRotation);
						Vector3 outTrans;
						int solvertnLH1 = _LHTrackingAlgorithm.SolvePnPWithRotation(&vertices[0], VerNum, hitMap, lhId, LH1Rotation, LH1Translation, pRotation, outTrans);
						if (solvertnLH1)
						{
							memcpy(transLH1, outTrans.val, sizeof(double) * 3);
						}
					}
				}
			}

			debugHitMap->ClearBaseStation(lhId);
			debugHitMap->Merge(*hitMap);
			Dofusion(lhId, lighthouseDataTimestamp);
		};

		// The following two params use for New LH Fusion Alg
		int lastUseLHId = -1;
		irrmath::vector3d translationDiff(0, 0, 0);
		uint64_t lastTimestamp = 0;
		bool hasHugeDiffBetweenLH = false;
		Dofusion = [&](int lastBsId, uint64_t lighthouseDataTimestamp)
		{
			double LH1Quat[4];
			double LH1Pos[3];
			double LH2Quat[4];
			double LH2Pos[3];
			LightHouseData::GetInstance()->GetLightHousePoseWithQuat(0, LH1Quat, LH1Pos);
			LightHouseData::GetInstance()->GetLightHousePoseWithQuat(1, LH2Quat, LH2Pos);

			double lastTranslation[3];

			if (res_translation)
			{
				lastTranslation[0] = res_translation->x;
				lastTranslation[1] = res_translation->y;
				lastTranslation[2] = res_translation->z;
			}

			bool getQuatLH = false;
			//irrmath::Quatd quatLH;
			auto setQuatLH = [&](irrmath::Quatd q) {
				quatLH = q;
				getQuatLH = true;
			};
			if (0 != solvertnLH1 || 0 != solvertnLH2)
			{
				if (isSingleLighthouse)
				{
					setQuatLH(quatLH1);
					memcpy(TrackTranslation, transLH1, sizeof(double[3]));
				}
				else
				{
					if (0 != solvertnLH1 && 0 != solvertnLH2)
					{
						quatLH2.ensureSameHemisphere(quatLH1);
						double alpha = (lh1HitNum*lh1HitNum) / double(lh1HitNum*lh1HitNum + lh2HitNum*lh2HitNum);
						if (!InternalVariables::GetInstance()->GetInt("Fusion.UseDualquatFusion", 0))
						{
							irrmath::Quatd q;
							q.lerp(quatLH1, quatLH2, 1 - alpha);
							setQuatLH(q);
							for (int i = 0; i < 3; ++i)
								TrackTranslation[i] = alpha*transLH1[i] + (1 - alpha)*transLH2[i];
						}
						else
						{
							auto tmp = quatd(quatLH1.w, quatLH1.x, quatLH1.y, quatLH1.z);
							auto dq1 = dualquatd(tmp, transLH1[0], transLH1[1], transLH1[2]);
							tmp = quatd(quatLH2.w, quatLH2.x, quatLH2.y, quatLH2.z);
							auto dq2 = dualquatd(tmp, transLH2[0], transLH2[1], transLH2[2]);

							std::vector<double> quatsWeights;
							quatsWeights.push_back(alpha);
							quatsWeights.push_back(1 - alpha);

							std::vector<dualquatd> quatsVec;
							quatsVec.push_back(dq1.N());
							quatsVec.push_back(dq2.N());

							dualquatd dqt = average::DLB(quatsVec, quatsWeights);

							setQuatLH(irrmath::Quatd(dqt.x, dqt.y, dqt.z, dqt.w));
							auto posQuat = dqt.getPositionQuat();
							TrackTranslation[0] = posQuat.x;
							TrackTranslation[1] = posQuat.y;
							TrackTranslation[2] = posQuat.z;
						}
					}
					else if (0 != solvertnLH1 && 0 == solvertnLH2)
					{
						setQuatLH(quatLH1);
						memcpy(TrackTranslation, transLH1, sizeof(double[3]));
					}
					else if (0 == solvertnLH1 && 0 != solvertnLH2)
					{
						setQuatLH(quatLH2);
						memcpy(TrackTranslation, transLH2, sizeof(double[3]));
					}
				}

			}

			if (getQuatLH) {
				// Use quatLH to fix the rotaion model.
				imuRotationModel->FixRotationWithCorrectValue(lighthouseDataTimestamp, quatLH);
			}

			if (!solvertnLH1 && !solvertnLH2)
				return;

			{
				std::lock_guard<std::mutex> lock(mtxPose);
				irrmath::vector3d nowTranslation(TrackTranslation[0], TrackTranslation[1], TrackTranslation[2]);
				if (res_translation)
				{
					double threshold = InternalVariables::GetInstance()->GetDouble("tracking.maxTranlationMargin", 10000);
					if (res_translation->getDistanceFrom(nowTranslation) > threshold)
					{
						return;
					}
				}
			}

			poseShiftArray.PushBack(irrmath::vector3d(TrackTranslation[0], TrackTranslation[1], TrackTranslation[2]));
			lastTimestamp = lighthouseDataTimestamp;


			if (KF_translation)
			{
				delete KF_translation;
				KF_translation = NULL;
			}

			{
				KF_Solver.transitionMatrix = (cv::Mat_<double>(6, 6) <<
					1, 0, 0, 1, 0, 0,
					0, 1, 0, 0, 1, 0,
					0, 0, 1, 0, 0, 1,
					0, 0, 0, 1, 0, 0,
					0, 0, 0, 0, 1, 0,
					0, 0, 0, 0, 0, 1);

				cv::Mat kf_prediction = KF_Solver.predict();
				KF_measurement = (cv::Mat_<double>(3, 1) << TrackTranslation[0], TrackTranslation[1], TrackTranslation[2]);
				cv::Mat kf_estimated = KF_Solver.correct(KF_measurement);
				KF_translation = new irrmath::vector3d(kf_estimated.at<double>(0), kf_estimated.at<double>(1), kf_estimated.at<double>(2));
			}

			{
				std::lock_guard<std::mutex> lock(mtxPose);
				poseNum++;
				poseHistory[0] = poseHistory[1];
				poseHistoryTime[0] = poseHistoryTime[1];
				irrmath::Quatd q = imuRotationModel->GetLatestRotation();
				quatd tmp(q.w, q.x, q.y, q.z);
				poseHistory[1] = dualquatd(tmp, KF_translation->x, KF_translation->y, KF_translation->z);
				poseHistoryTime[1] = lighthouseDataTimestamp;

				if (!res_translation)
					res_translation = new irrmath::vector3d;
				res_translation->x = KF_translation->x;
				res_translation->y = KF_translation->y;
				res_translation->z = KF_translation->z;
			}

			lh1Outliers.clear();
			lh2Outliers.clear();
			lhFrameCount++;
		};
		processor->RegisterImuDataCallback(onImuData);
		processor->RegisterBaseStationCallback(onOneLightHouseData);
		processor->loopProcess();
	}

	const std::map<int, int>* TrackObject::GetVertexMap()
	{
		return _vertexMap;
	}

	irrmath::matrix4d TrackObject::array2Mat3d(double* mat)
	{
		return irrmath::matrix4d(mat[0], mat[3], mat[6],
			mat[1], mat[4], mat[7],
			mat[2], mat[5], mat[8]);
	}

	irrmath::vector3d TrackObject::array2Vec3d(double* vec)
	{
		return irrmath::vector3d(vec[0], vec[1], vec[2]);
	}

	void TrackObject::quat2Array(const irrmath::Quatd quat, double* mat)
	{
		irrmath::matrix4d mat3d = quat.getMatrix();
		for (int row = 0; row < 3; row++)
		{
			for (int col = 0; col < 3; col++)
			{
				mat[col * 3 + row] = mat3d.at(row, col);
			}
		}
	}

	void TrackObject::vector2Array(const irrmath::vector3d vec3d, double* vec)
	{
		vec[0] = vec3d.x;
		vec[1] = vec3d.y;
		vec[2] = vec3d.z;
	}

	bool TrackObject::DoPrediction(const uint64_t timeStamp, irrmath::Quatd& quat, irrmath::vector3d& trans)
	{
		if (deviceType == DeviceType::Wireless)
		{
			return DoPredictionWireless(timeStamp, quat, trans);
		}
		else
		{
			return DoPredictionHMD(timeStamp, quat, trans);
		}
	}

	bool TrackObject::DoPredictionHMD(const uint64_t timeStamp, irrmath::Quatd& quat, irrmath::vector3d& trans)
	{
		uint64_t timestamp_delay = timeStamp;
		if (poseNum < 2)
			return false;
		uint64 maxPredictTime = 500000;
		if (timestamp_delay - poseHistoryTime[1] > maxPredictTime)
			timestamp_delay = maxPredictTime + poseHistoryTime[1];

		std::vector<dualquatd> quatsHistory;
		quatsHistory.push_back(poseHistory[0].N());
		quatsHistory.push_back(poseHistory[1].N());
		std::vector<double> quatsWeights;
		quatsWeights.push_back((double(poseHistoryTime[1]) - double(timestamp_delay)) / double(poseHistoryTime[1] - poseHistoryTime[0]));
		quatsWeights.push_back((double(timestamp_delay) - double(poseHistoryTime[0])) / double(poseHistoryTime[1] - poseHistoryTime[0]));

		dualquatd dqt = average::DLB(quatsHistory, quatsWeights);

		quat = irrmath::Quatd(dqt.x, dqt.y, dqt.z, dqt.w);
		auto posQuat = dqt.getPositionQuat();
		trans = irrmath::vector3d(posQuat.x, posQuat.y, posQuat.z);
		return true;
	}

	bool TrackObject::DoPredictionWireless(const uint64_t timeStamp, irrmath::Quatd& quat, irrmath::vector3d& trans)
	{
		uint64_t timestamp_delay = timeStamp;
		if (poseNum < 2)
			return false;

		std::vector<dualquatd> quatsHistory;
		quatsHistory.push_back(poseHistory[0].N());
		quatsHistory.push_back(poseHistory[1].N());
		std::vector<double> quatsWeights;
		quatsWeights.push_back((double(poseHistoryTime[1]) - double(timestamp_delay)) / double(poseHistoryTime[1] - poseHistoryTime[0]));
		quatsWeights.push_back((double(timestamp_delay) - double(poseHistoryTime[0])) / double(poseHistoryTime[1] - poseHistoryTime[0]));


		dualquatd dqt = average::DLB(quatsHistory, quatsWeights);

		quat = irrmath::Quatd(dqt.x, dqt.y, dqt.z, dqt.w);
		auto posQuat = dqt.getPositionQuat();
		trans = irrmath::vector3d(posQuat.x, posQuat.y, posQuat.z);

		return true;
	}

	PoseResultFlag TrackObject::GetPose(double * quat, double * translation, uint64_t timeStamp)
	{
		PoseResultFlag res = PoseResultFlag::invalid;
		std::lock_guard<std::mutex> lock(mtxPose);
		if (res_translation)
		{
			translation[0] = res_translation->x;
			translation[1] = res_translation->y;
			translation[2] = res_translation->z;
			res = PoseResultFlag(res | PoseResultFlag::transValid);
		}

		if (res_quat)
		{
			quat[0] = res_quat->x;
			quat[1] = res_quat->y;
			quat[2] = res_quat->z;
			quat[3] = res_quat->w;
			res = PoseResultFlag(res | PoseResultFlag::quatValid);
		}

		if (res_quat && timeStamp != 0 && deviceType == DeviceType::Wired)
		{
			irrmath::Quatd predictQuat = imuRotationModel->GetRotationAt(timeStamp);
			quat[0] = predictQuat.x;
			quat[1] = predictQuat.y;
			quat[2] = predictQuat.z;
			quat[3] = predictQuat.w;

			if (InternalVariables::GetInstance()->GetInt("UseHmdTranslationPredict", 1))
			{
				irrmath::Quatd dummyPredictQuat; // Not used for now.
				irrmath::vector3d predictTrans;
				if (DoPrediction(timeStamp, dummyPredictQuat, predictTrans))
				{
					translation[0] = predictTrans.x;
					translation[1] = predictTrans.y;
					translation[2] = predictTrans.z;
				}
			}
			std::stringstream ss;
			for (int i = 0; i < 4; i++)
			{
				ss << quat[i] << "  ";
			}
			ss << std::endl;

			for (int i = 0; i < 3; i++)
			{
				ss << translation[i] << "  ";
			}
			ss << std::endl;
			LogUtil::getLogger("Pose")->log(ss.str());

		}
		else if (timeStamp != 0 && deviceType == DeviceType::Wireless && HYPEREAL::InternalVariables::GetInstance()->GetInt("UseWirelessPredict", 1) == 1)
		{
			irrmath::Quatd predictQuat;
			irrmath::vector3d predictTrans;
			bool bRtn = DoPrediction(timeStamp, predictQuat, predictTrans);
			if (bRtn)
			{
				quat[0] = predictQuat.x;
				quat[1] = predictQuat.y;
				quat[2] = predictQuat.z;
				quat[3] = predictQuat.w;
				translation[0] = predictTrans.x;
				translation[1] = predictTrans.y;
				translation[2] = predictTrans.z;
			}
		}

		if (poseNum >= 1)
		{
			auto deltaTime = timeStamp - poseHistoryTime[1];
			if (deltaTime > InternalVariables::GetInstance()->GetInt("predict.maxPredictionTime", 2000000))
			{
				return PoseResultFlag::quatValid;
			}
		}

		return res;
	}

	bool TrackObject::GetGyroVec(int id, double outGyroVec[3])
	{
		if (deviceType != DeviceType::Wired)
			return false;
		irrmath::vector3d gyroVec(lastestGyro[1][0], lastestGyro[1][1], lastestGyro[1][2]);


		double quat[4];
		double translation[3];

		if (GetPose(quat, translation, TimeManager::GetMicroSeconds()) & PoseResultFlag::quatValid)
		{
			irrmath::Quatd _quat(quat[0], quat[1], quat[2], quat[3]);
			irrmath::matrix4d rotation = _quat.getMatrix();
			gyroVec = rotation * gyroVec;
			for (int i = 0; i < 3; i++)
			{
				outGyroVec[i] = gyroVec[i];
			}
			return true;
		}
		else
			return false;
	}

	bool TrackObject::IsDeviceStaticWhenCalibrating()
	{
		return moveDetector.IsDeviceStatic();
	}

	bool TrackObject::IsDeviceStatic()
	{
		bool isStatic = false;
		irrmath::Quatd nextQuat;
		irrmath::vector3d nextTrans;
		double staticThreshold;
		bool hasPredictTrans = false;
		if (deviceType == DeviceType::Wired)
		{
			staticThreshold = InternalVariables::GetInstance()->GetDouble("MoveDetector.HmdStaticThreshold", 10);
		}
		else if (deviceType == DeviceType::Wireless)
		{
			staticThreshold = InternalVariables::GetInstance()->GetDouble("MoveDetector.WirelessStaticThreshold", 10);
		}

		hasPredictTrans = DoPrediction(TimeManager::GetMicroSeconds(), nextQuat, nextTrans);

		if (!hasPredictTrans)
			return false;

		auto dist = res_translation->getDistanceFrom(nextTrans);
		if (dist < staticThreshold)
			isStatic = true;

		return isStatic;
	}

	void TrackObject::SetId(int _id)
	{
		id = _id;
	}

	int TrackObject::GetId()
	{
		return id;
	}

	TrackObjectFlags TrackObject::GetFlags()
	{
		return flags;
	}

	PoseResultFlag TrackObject::GetCurrentPose(irrmath::vector3d& translation, irrmath::Quatd& rotation)
	{
		Vector4 rotInVector;
		Vector3 transd;
		PoseResultFlag ret = GetPose(rotInVector.val, transd.val, TimeManager::GetMicroSeconds());
		if (ret & PoseResultFlag::quatValid)
		{
			// quat
			rotation.x = rotInVector.x;
			rotation.y = rotInVector.y;
			rotation.z = rotInVector.z;
			rotation.w = rotInVector.w;
		}

		if (ret & PoseResultFlag::transValid)
		{
			// trans
			translation.x = transd.x;
			translation.y = transd.y;
			translation.z = transd.z;
		}

		return ret;
	}

	void TrackObject::_transAcce(int16_t * inAcce, int16_t * outAcce)
	{
		int16_t tmpAcc[3] = { inAcce[0], inAcce[1], inAcce[2] };
		outAcce[0] = tmpAcc[0];
		outAcce[1] = tmpAcc[1];
		outAcce[2] = tmpAcc[2];
	}

	void TrackObject::_transGyro(int16_t *inGyro, int16_t * outGyro)
	{
		int16_t tmpGyro[3] = { inGyro[0], inGyro[1], inGyro[2] };
		outGyro[0] = tmpGyro[0];
		outGyro[1] = tmpGyro[1];
		outGyro[2] = tmpGyro[2];
	}

	void TrackObject::_transGyro(irrmath::vector3d & inGyro, irrmath::vector3d & outGyro)
	{
		irrmath::vector3d tmpGyro(inGyro[0], inGyro[1], inGyro[2]);
		outGyro[0] = tmpGyro[0];
		outGyro[1] = tmpGyro[1];
		outGyro[2] = tmpGyro[2];
	}

}
