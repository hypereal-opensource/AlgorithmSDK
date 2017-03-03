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
#ifdef ALGORITHMDLL_EXPORTS
#define ALGORITHMDLL_API __declspec(dllexport) 
#else
#define ALGORITHMDLL_API __declspec(dllimport) 
#endif
#include <stdint.h>
#include <windows.h>
#include <atomic>

class TimeManager
{
public:
	static ALGORITHMDLL_API uint64_t GetMicroSeconds();
	static ALGORITHMDLL_API void StopTimer();
	static ALGORITHMDLL_API void StartTimer();
	// Continue clock and set to current time.
	static ALGORITHMDLL_API void Reset();
	static ALGORITHMDLL_API bool isPaused;
	static ALGORITHMDLL_API void Advance(uint64_t microSeconds);
private:
	static std::atomic<uint64_t> timeStampDelta;
	static std::atomic<uint64_t> lastStopRealTime;
	static uint64_t GetRealMicroSeconds();
};


