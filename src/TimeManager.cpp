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

#include "TimeManager.h"

std::atomic<uint64_t> TimeManager::timeStampDelta = 0;
std::atomic<uint64_t> TimeManager::lastStopRealTime = 0;
bool TimeManager::isPaused = false;

#include <cstdio>

uint64_t TimeManager::GetMicroSeconds()
{
	if (isPaused)
	{
		return lastStopRealTime - timeStampDelta;
	}
	auto nowTime = GetRealMicroSeconds();
	return nowTime - timeStampDelta;
}

uint64_t TimeManager::GetRealMicroSeconds()
{
	static LARGE_INTEGER Frequency = { 0 };
	static LARGE_INTEGER initCounter = { 0 };
	LARGE_INTEGER now;
	QueryPerformanceCounter(&now);
	if (Frequency.QuadPart == 0)
	{
		initCounter = now;
		QueryPerformanceFrequency(&Frequency);
	}
	return ((now.QuadPart - initCounter.QuadPart) * 1000000 / Frequency.QuadPart);
}

void TimeManager::StopTimer()
{
	if (isPaused)
	{
		return;
	}
	lastStopRealTime = GetRealMicroSeconds();
	isPaused = true;
}

void TimeManager::StartTimer()
{
	if (!isPaused)
	{
		return;
	}
	timeStampDelta += GetRealMicroSeconds() - lastStopRealTime;
	isPaused = false;
}

void TimeManager::Reset()
{
	isPaused = false;
	timeStampDelta = 0;
}

void TimeManager::Advance(uint64_t microSeconds)
{
	timeStampDelta -= microSeconds;
}
