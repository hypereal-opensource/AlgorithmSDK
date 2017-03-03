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
#include "quaternion.h"
#include<mutex>
#include<thread>

namespace HYPEREAL
{

class LightHouseData
{
public:

	static LightHouseData * GetInstance()
	{
		static LightHouseData * uniqueInstance = NULL;
		if (!uniqueInstance)
		{
			uniqueInstance = new LightHouseData();
		}
		return uniqueInstance;
	}

	void SetLightHousePose(int lightHouseId, double * rotation, double * translation)
	{
		std::lock_guard<std::mutex> lock(mtx);
		for (int i = 0; i < 9; i++)
			LHRotation[lightHouseId][i] = *(rotation + i);
		for (int i = 0; i < 3; i++)
			LHTranslation[lightHouseId][i] = *(translation + i);
		RefreshData(lightHouseId);
	}

	int GetLightHousePoseWithRotMat(int index, double * rotation, double * translation)
	{
		std::lock_guard<std::mutex> lock(mtx);
		if (!hasLHPoseInit[index])
		{
			return -1;
		}
		for (int i = 0; i < 9; i++)
		{
			rotation[i] = LHRotation[index][i];
		}
		for (int i = 0; i < 3; i++)
		{
			translation[i] = LHTranslation[index][i];
		}
		return 2;
	}

	PoseResultFlag GetLightHousePoseWithQuat(int lightHouseId, double * quat, double * translation)
	{
		std::lock_guard<std::mutex> lock(mtx);
		if (!hasLHPoseInit[lightHouseId])
		{
			return PoseResultFlag::invalid;
		}
		translation[0] = lighthouseTranslation[lightHouseId].x;
		translation[1] = lighthouseTranslation[lightHouseId].y;
		translation[2] = lighthouseTranslation[lightHouseId].z;

		quat[0] = lighthouseRotation[lightHouseId].x;
		quat[1] = lighthouseRotation[lightHouseId].y;
		quat[2] = lighthouseRotation[lightHouseId].z;
		quat[3] = lighthouseRotation[lightHouseId].w;
		return PoseResultFlag(PoseResultFlag::quatValid | PoseResultFlag::transValid);
	}

private:
	std::mutex mtx;
	double LHRotation[2][9];
	double LHTranslation[2][3];
	bool hasLHPoseInit[2];
	irrmath::Quatd worldQuat;
	irrmath::vector3d worldVector;

	irrmath::Quatd lighthouseRotation[2];
	irrmath::vector3d lighthouseTranslation[2];

	LightHouseData()
	{
		hasLHPoseInit[0] = hasLHPoseInit[1] = false;
		worldQuat.makeIdentity();
		worldVector[0] = worldVector[1] = worldVector[2] = 0;
	}

	// modify quatd and vector3d
	void RefreshData(int lightHouseId)
	{
		irrmath::matrix4d MatRLH;
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				MatRLH.at(i, j) = LHRotation[lightHouseId][j * 3 + i];
		auto ivMatRLH = MatRLH.inverted();

		irrmath::vector3d matTrans(LHTranslation[lightHouseId][0], LHTranslation[lightHouseId][1], LHTranslation[lightHouseId][2]);

		auto resT = ivMatRLH * matTrans;
		lighthouseRotation[lightHouseId] = irrmath::Quatd(ivMatRLH);
		lighthouseTranslation[lightHouseId] = irrmath::vector3d(-resT[0], -resT[1], -resT[2]);
		hasLHPoseInit[lightHouseId] = true;
	}

	// column first
	irrmath::matrix4d GetMatrixByTranform(const double * rot)
	{
		irrmath::matrix4d res;
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				res.at(i, j) = rot[i + j * 3];
			}
		}
		return res;
	}
};

}
