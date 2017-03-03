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

#include "IMURotationModel.h"
#include <cassert>
#include "TimeManager.h"

namespace HYPEREAL
{
	IMURotationModel::IMURotationModel(Mode mode) : mode(mode)
	{
	}

	void IMURotationModel::AddQuatIncrease(const irrmath::Quatd &quatInc)
	{
		assert(mode == QUAT_INC);
		imuQuatNow = imuQuatNow * quatInc;
		imuQuatNow.normalize();
	}

	void IMURotationModel::AddIMUPacket(uint64_t timestamp, const irrmath::vector3d &gyroVec, const irrmath::vector3d &accVec, const irrmath::vector3d &magVec)
	{
		assert(mode == RAW_IMU);
		irrmath::Quatd quatInc;
		quatInc.fromRotationVector(irrmath::vector3d(gyroVec.x, gyroVec.y, gyroVec.z) * 0.001);
		imuQuatNow = imuQuatNow * quatInc;
		imuQuatNow.normalize();

	}

	void IMURotationModel::FixRotationWithCorrectValue(uint64_t timestamp, const irrmath::Quatd &quatLH)
	{
		irrmath::Quatd quatDiff = quatLH * imuQuatNow.inversed();
		static double firstTime = -1;
		if (firstTime < 0) {
			firstTime = TimeManager::GetMicroSeconds() * 1e-6;
		}
		double t = TimeManager::GetMicroSeconds() * 1e-6;
		const double fastIMUFusionRatio = 0.2;
		const double slowIMUFusionRatio = 0.015;

		auto quatDiffTooMuch = [](irrmath::Quatd a, irrmath::Quatd b) ->bool {
			const double MAX_QUAT_DIFF = 1;
			bool ret = abs(a.getDiffAngle(b)) > MAX_QUAT_DIFF;
			return ret;
		};

		// At first 3 seconds after the first time scanned by LH, it can be fixed faster than normal
		if (t - firstTime < 3 && quatDiffTooMuch(quatDiff, quatDiffBetweenIMUAndLH))
		{
			quatDiffBetweenIMUAndLH.slerp(quatDiffBetweenIMUAndLH, quatDiff, fastIMUFusionRatio, 0.0001);
		}
		else
		{
			quatDiffBetweenIMUAndLH.slerp(quatDiffBetweenIMUAndLH, quatDiff, slowIMUFusionRatio, 0.0001);
		}
	}

	irrmath::Quatd IMURotationModel::GetLatestRotation()
	{
		irrmath::Quatd out = quatDiffBetweenIMUAndLH * imuQuatNow;
		out.normalize();
		return out;
	}

	irrmath::Quatd IMURotationModel::GetRotationAt(uint64_t timestamp)
	{
		// Fix the result by initial rotation.
		irrmath::Quatd out = quatDiffBetweenIMUAndLH * imuQuatNow;
		out.normalize();
		return out;
	}


	IMURotationModel::~IMURotationModel()
	{

	}
}
