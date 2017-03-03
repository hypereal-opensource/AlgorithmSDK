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

#include <memory>

#include "quaternion.h"

namespace HYPEREAL
{
	// The model contains rotation info.
	// It could be handled with HMD (with predictor) and controller (quat increase).
	class IMURotationModel
	{
	public:
		enum Mode
		{
			RAW_IMU, // HMD, wired controller
			QUAT_INC // Wireless devices.
		};
		IMURotationModel(Mode mode);

		// Add quat inc (only for QUAT_INC mode).
		void AddQuatIncrease(const irrmath::Quatd &quatInc);

		// Add IMU packet. Requires gyro and acc data.
		// (only for RAW_IMU mode).
		// @param timestamp unit microsecond
		// @param gyroVec unit radian/s
		// @param accVec unit m/s^2
		// @param magVec unit uT
		void AddIMUPacket(uint64_t timestamp, const irrmath::vector3d &gyroVec, const irrmath::vector3d &accVec, const irrmath::vector3d &magVec);

		// Tell the model an accurate rotation, it will fix the rotation.
		// maintain quatDiff between IMUQuat & LHQuat, quatDiff * IMUQuat = LHQuat, quatDiff is increasing gradually until object's initial rotation.
		void FixRotationWithCorrectValue(uint64_t timestamp, const irrmath::Quatd &rot);

		// Get the orientation. Note in QUAT_INC mode, it will only return latest value, ignoring timestamp.
		irrmath::Quatd GetRotationAt(uint64_t timestamp);

		// Get the latest orientation, without predictor.
		irrmath::Quatd GetLatestRotation();

		~IMURotationModel();

	private:
		Mode mode;
		// Initial rotation, initial quat diff between IMU&LH, so quatDiff * quatIMU = quatLH
		irrmath::Quatd quatDiffBetweenIMUAndLH;

		// Quat completely integrated by IMu. For both QUAT_INC and RAW_IMU.
		irrmath::Quatd imuQuatNow;
	};
}
