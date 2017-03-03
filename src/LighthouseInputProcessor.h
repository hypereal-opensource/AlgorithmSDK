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

#include <memory>
#include "AbstractInputProcessor.h"
#include "TrackingDataLoader.h"

namespace HYPEREAL
{
	class LighthouseInputProcessor : public AbstractInputProcessor
	{
	public:
		LighthouseInputProcessor(std::shared_ptr<TrackingDataLoader> dataLoader);

		virtual void loopProcess() override;
		virtual void Stop() override;
		// Convert raw LH hitRecord to hitMap.
		static std::shared_ptr<HitMap> HitRecordToHitMap(const HitRecord* hitRecords, const int hitRecordsNum);
		// Convert hitMap to raw LH hitRecord.
		static const std::pair<std::shared_ptr<HitRecord>, int> HitMapToHitRecords(std::shared_ptr<HitMap> hitMap);

		// uint16_t to float (mm/s^2)
		static float TransferAcceUnit(int16_t originAcceVal);
	private:
		std::shared_ptr<TrackingDataLoader> dataLoader;
	};
}
