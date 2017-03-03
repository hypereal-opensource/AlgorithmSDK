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
#include "LHTrackingAlgorithm.h"
#include <math.h>
#include <string>

#include "opencv2/calib3d.hpp"

#include "f2c.h"
#include "clapack.h"

#include <iostream>

#include <fstream>
#include <memory>
#include "LogUtil.h"

#include "quaternion.h"
#include "InternalVariables.h"

using namespace HYPEREAL;

LHTrackingAlgorithm::LHTrackingAlgorithm()
{
}

LHTrackingAlgorithm::~LHTrackingAlgorithm()
{
}

int LHTrackingAlgorithm::SolvePnPOrIterative(const double* vertices, const int vernum, const std::shared_ptr<HitMap> hitMap, const int lhId,
	const double* rotateLH /* Column Major */, const double* translateLH, bool isIterative,
	/* Input/Output */ irrmath::Quatd& outrotate, double outtranslate[3])
{
	auto projectionPoint = [&](int sensorId, int lhId, bool isX) -> double {
		auto it = hitMap->get(sensorId, lhId);
		double angle = isX ? it.xAngle : it.yAngle;
		return -cos(angle) / sin(angle);
	};

	irrmath::matrix4d lhTransMat(
		rotateLH[0], rotateLH[3], rotateLH[6], translateLH[0],
		rotateLH[1], rotateLH[4], rotateLH[7], translateLH[1],
		rotateLH[2], rotateLH[5], rotateLH[8], translateLH[2],
		0, 0, 0, 1);

	std::map<int, cv::Point3d> sensorObjectPointMap;
	std::map<int, cv::Point2d> sensorImagePointMap;
	for (int i = 0; i < vernum; i++)
	{
		if (!hitMap->isHit(i, lhId))
		{
			continue;
		}
		const double* vertex = vertices + 3 * i;
		cv::Point3d objPoint(vertex[0], vertex[1], vertex[2]);
		cv::Point2d imagePoint(projectionPoint(i, lhId, true), projectionPoint(i, lhId, false));
		sensorObjectPointMap[i] = objPoint;
		sensorImagePointMap[i] = imagePoint;
	}

	if (isIterative && sensorImagePointMap.size() < 1
		|| !isIterative && sensorImagePointMap.size() < 4)
	{
		return 0;
	}
	// Do actual solving.
	std::vector<cv::Point3d> objectPoints;
	std::vector<cv::Point2d> imagePoints;
	cv::Matx33d rotaionMatrix;
	for each (auto entry in sensorImagePointMap)
	{
		int sensorId = entry.first;
		objectPoints.push_back(sensorObjectPointMap[sensorId]);
		imagePoints.push_back(sensorImagePointMap[sensorId]);
	}
	cv::Matx31d solveRotateInVec, solveTranslate;
	if (isIterative)
	{
		irrmath::matrix4d objectTransMat = outrotate.getMatrix();
		objectTransMat.setTranslation(irrmath::vector3d(outtranslate[0], outtranslate[1], outtranslate[2]));
		objectTransMat = lhTransMat * objectTransMat;
		rotaionMatrix = cv::Matx33d(
			objectTransMat.at(0, 0), objectTransMat.at(0, 1), objectTransMat.at(0, 2),
			objectTransMat.at(1, 0), objectTransMat.at(1, 1), objectTransMat.at(1, 2),
			objectTransMat.at(2, 0), objectTransMat.at(2, 1), objectTransMat.at(2, 2));
		cv::Rodrigues(rotaionMatrix, solveRotateInVec);
		solveTranslate = cv::Vec3d(objectTransMat.at(0, 3), objectTransMat.at(1, 3), objectTransMat.at(2, 3));
	}
	bool ret = cv::solvePnP(objectPoints, imagePoints, cv::Matx33d::eye(), cv::Mat(), solveRotateInVec, solveTranslate, isIterative, isIterative ? cv::SOLVEPNP_ITERATIVE : cv::SOLVEPNP_EPNP);
	if (!ret)
	{
		return 0;
	}
	// Translate to output format
	cv::Rodrigues(solveRotateInVec, rotaionMatrix);
	// Finish the output.

	irrmath::matrix4d objectTransMat(
		rotaionMatrix(0, 0), rotaionMatrix(0, 1), rotaionMatrix(0, 2), solveTranslate.val[0],
		rotaionMatrix(1, 0), rotaionMatrix(1, 1), rotaionMatrix(1, 2), solveTranslate.val[1],
		rotaionMatrix(2, 0), rotaionMatrix(2, 1), rotaionMatrix(2, 2), solveTranslate.val[2],
		0, 0, 0, 1);

	objectTransMat = lhTransMat.inverted() * objectTransMat;
	outrotate = irrmath::Quatd(objectTransMat);
	for (int i = 0; i < 3; ++i)
		outtranslate[i] = objectTransMat.at(i, 3);
	return 1;
}

int LHTrackingAlgorithm::SolvePnPWithRotation(const double* vertices, const int vernum, const std::shared_ptr<HitMap> hitMap, const int lhId,
	const double* rotateLH /* Column Major */, const double* translateLH,
	const double* rotateIMU /* Row Major */, HYPEREAL::Vector3& outtranslate)
{
	std::vector<int> hitSensorIdList;
	for (int i = 0; i < vernum; i++)
	{
		if (!hitMap->isHit(i, lhId))
		{
			continue;
		}
		hitSensorIdList.push_back(i);
	}

	if (hitSensorIdList.size() < 2)
	{
		return 0;
	}

	///////////	Use algorithm in paper <On_the_use_of_IMUs_in_the_PnP_problem> //////////
	int pointNum = (int)hitSensorIdList.size();
	irrmath::matrix4d lhTransMat(
		rotateLH[0], rotateLH[3], rotateLH[6], translateLH[0],
		rotateLH[1], rotateLH[4], rotateLH[7], translateLH[1],
		rotateLH[2], rotateLH[5], rotateLH[8], translateLH[2],
		0, 0, 0, 1);
	irrmath::matrix4d objectRot(
		rotateIMU[0], rotateIMU[1], rotateIMU[2],
		rotateIMU[3], rotateIMU[4], rotateIMU[5],
		rotateIMU[6], rotateIMU[7], rotateIMU[8]);

	objectRot = lhTransMat * objectRot; // let object's rotation relative to LH.

	cv::Matx33d r(
		objectRot.at(0, 0), objectRot.at(0, 1), objectRot.at(0, 2),
		objectRot.at(1, 0), objectRot.at(1, 1), objectRot.at(1, 2),
		objectRot.at(2, 0), objectRot.at(2, 1), objectRot.at(2, 2)); // rotation relative to LH

	auto pointVector = [&](int pointIdx) -> cv::Matx31d {
		const double *v = vertices + 3 * hitSensorIdList[pointIdx];
		return cv::Matx31d(v[0], v[1], v[2]) * 0.001; // unit: m
	};

	auto projectionPoint = [&](int pointIdx, bool isX) -> double {
		int sensorId = hitSensorIdList[pointIdx];

		auto it = hitMap->get(sensorId, lhId);
		double angle = isX ? it.xAngle : it.yAngle;
		return -cos(angle) / sin(angle);
	};

	cv::Mat Q(pointNum * 2, 3, CV_64F);
	cv::Mat s(pointNum * 2, 1, CV_64F);
	for (int row = 0; row < pointNum * 2; ++row)
	{
		Q.at<double>(row, 0) = row % 2 == 0 ? -1 : 0;
		Q.at<double>(row, 1) = row % 2 == 0 ? 0 : -1;
		int pointIdx = row / 2;
		Q.at<double>(row, 2) = projectionPoint(pointIdx, row % 2 == 0);

		if (row % 2 == 0)
		{
			auto tmp = r.row(0) * pointVector(pointIdx) - r.row(2) * pointVector(pointIdx) * projectionPoint(pointIdx, true);
			s.at<double>(row, 0) = tmp.val[0];
		}
		else
		{
			auto tmp = r.row(1) * pointVector(pointIdx) - r.row(2) * pointVector(pointIdx) * projectionPoint(pointIdx, false);
			s.at<double>(row, 0) = tmp.val[0];
		}
	}

	// Return norm of inv(Q) * s - t
	auto t = cv::Mat(Q.inv(cv::DecompTypes::DECOMP_SVD) * s);

	double x = t.at<double>(0, 0);
	double y = t.at<double>(1, 0);
	double z = t.at<double>(2, 0);
	irrmath::vector3d objInLhFrame(x * 1000, y * 1000, z * 1000);

	auto objInWorldFrame = lhTransMat.inverted() * objInLhFrame;
	outtranslate.x = objInWorldFrame.x;
	outtranslate.y = objInWorldFrame.y;
	outtranslate.z = objInWorldFrame.z;
	return 1;
}
