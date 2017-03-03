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

#include "MoveDetector.h"
#include"TimeManager.h"
#include "InternalVariables.h"

namespace HYPEREAL
{

MoveDetector::MoveDetector()
{
	//  Unit mm/s^2
	accFloatThreshold = InternalVariables::GetInstance()->GetDouble("StaticAccFloatThreshold", 598.14);
	staticTimesShortThreshold = InternalVariables::GetInstance()->GetDouble("staticTimesShortThreshold", 250) * 1000;
	Reset();
}

void MoveDetector::Reset()
{
	for (int i = 0; i < 3; i++)
	{
		fisrtAccObserverVal[i] = -9999999;
		fisrtAccObserverTime = 0;
	}
	_isDeviceStatic = false;
	_isDeviceStaticInShortTime = false;
}

void MoveDetector::ClearStaicLightSensorDatas()
{
	staicLightSensorDatas.clear();
}

void MoveDetector::AddAccData(double x, double y, double z, std::pair<std::shared_ptr<HitRecord>, int> lightSensorRecords)
{
	std::lock_guard<std::mutex> lock(mtx);
	if (staicLightSensorDatas.size() < maxStaicLightSensorDatasNum)
	{
		staicLightSensorDatas.push_back(lightSensorRecords);
	}
	if (abs(fisrtAccObserverVal[0] - x) > accFloatThreshold || abs(fisrtAccObserverVal[1] - y) > accFloatThreshold \
		|| abs(fisrtAccObserverVal[2] - z) > accFloatThreshold)
	{
		fisrtAccObserverTime = TimeManager::GetMicroSeconds();
		fisrtAccObserverVal[0] = x;
		fisrtAccObserverVal[1] = y;
		fisrtAccObserverVal[2] = z;
		totalAccObserverVals.x = x;
		totalAccObserverVals.y = y;
		totalAccObserverVals.z = z;
		totalAccObserverNum = 1;
		_isDeviceStatic = false;
		_isDeviceStaticInShortTime = false;
		ClearStaicLightSensorDatas();
	}
	else
	{
		totalAccObserverVals.x += x;
		totalAccObserverVals.y += y;
		totalAccObserverVals.z += z;
		totalAccObserverNum++;
		if (TimeManager::GetMicroSeconds() - fisrtAccObserverTime > staticTimesThreshold)
		{
			_isDeviceStatic = true;
		}
		else
		{
			_isDeviceStatic = false;
		}

		if (TimeManager::GetMicroSeconds() - fisrtAccObserverTime > staticTimesShortThreshold)
		{
			_isDeviceStaticInShortTime = true;
		}
		else
		{
			_isDeviceStaticInShortTime = false;
		}
	}
}

bool MoveDetector::IsDeviceStatic()
{
	std::lock_guard<std::mutex> lock(mtx);
	return _isDeviceStatic;
}

bool MoveDetector::IsDeviceStaticInShortTime()
{
	std::lock_guard<std::mutex> lock(mtx);
	return _isDeviceStaticInShortTime;
}

std::vector<std::pair<std::shared_ptr<HitRecord>, int> > & MoveDetector::GetStaicLightSensorDatas()
{
	return staicLightSensorDatas;
}

}
