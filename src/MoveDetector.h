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

#include "internal.h"
#include<vector>
#include "quaternion.h"


namespace HYPEREAL
{
class MoveDetector
{
public:
	MoveDetector();
	void AddAccData(double x, double y, double z, std::pair<std::shared_ptr<HitRecord>, int> lightSensorRecords);
	void ClearStaicLightSensorDatas();
	bool IsDeviceStatic();
	bool IsDeviceStaticInShortTime();
	std::vector<std::pair<std::shared_ptr<HitRecord>, int> > & GetStaicLightSensorDatas();
	void Reset();

private:
	// params use in auto calibration and static state judging, unit: micro seconds.
	const double staticTimesThreshold = 3000 * 1000;
	double staticTimesShortThreshold = 500 * 1000;
	double fisrtAccObserverVal[3];
	int64_t fisrtAccObserverTime;
	//  Unit mm/s^2
	double accFloatThreshold;
	const unsigned int maxStaicLightSensorDatasNum = 1000;
	//  end of params use in auto calibration and static state judging
	bool _isDeviceStatic;
	bool _isDeviceStaticInShortTime;

	irrmath::vector3d totalAccObserverVals;
	int totalAccObserverNum = 0;
	std::vector<std::pair<std::shared_ptr<HitRecord>, int> > staicLightSensorDatas;

	std::mutex mtx;

};

}
