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
#include "TrackingDataLoader.h"


TrackingDataLoader::TrackingDataLoader()
{

}


TrackingDataLoader::~TrackingDataLoader()
{
}

int TrackingDataLoader::search_beginpoint(uint8_t* data, const int buffersize, bool& bIMU)
{
	for (int i = 0; i < buffersize - 4; ++i)
	{
		if (0 == data[i] && 0 == data[i + 1] && 0x3C == data[i + 3])
		{
			uint8_t c = data[i + 2];
			if (1 == c || 2 == c || 3 == c || 4 == c)
			{
				bIMU = false;
				return i;
			}
			else if (0x2B == c)
			{
				bIMU = true;
				return i;
			}
		}
	}
	return -1;
}

int TrackingDataLoader::search_LHdatabegin(uint8_t* data, const int buffersize)
{
	for (int i = 0; i < buffersize - 4; ++i)
	{
		if (0 == data[i] && 0 == data[i + 1] && 0x3C == data[i + 3])
		{
			uint8_t c = data[i + 2];
			if (1 == c || 2 == c || 3 == c || 4 == c)
				return i;
		}
	}
	return -1;
}

int TrackingDataLoader::alignData()
{
	uint8_t sensordata[64];
	int pos = -1;
	for (int i = 0; i < 100; ++i)
	{
		blockReadData(sensordata, 64);	//serial.read(sensordata, 64 * 2);
		pos = search_LHdatabegin(sensordata, 64);
		if (-1 != pos)
			break;
	}

	if (-1 == pos)
	{
		debug_print("ERROR!! Miss Light House Data!!");
		return -1;
	}
	else
	{
		if (0 != pos)
			blockReadData(sensordata, pos); //serial.read(sensordata, pos);
		return 0;
	}
}

TrackingDataType TrackingDataLoader::ProceedData()
{
	bool bIsIMU = false;
	blockReadData(_data, 64);	//serial.read(sensordata, 16);
	int pos = search_beginpoint(_data, 64, bIsIMU);
	if (0 == pos)
	{
		if (bIsIMU)
			return TrackingDataType::IMU;
		else
			return TrackingDataType::LightHouse;
	}
	else
	{
		debug_print("ERROR!! Data Process Error!!");
		return TrackingDataType::FAILED;
	}
}
