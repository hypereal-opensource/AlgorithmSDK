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
#include "TO_HMD.h"
#include "InternalVariables.h"
#include <iostream>

// modified over
namespace HYPEREAL
{

TO_HMD::TO_HMD(int id, TrackObjectFlags flags, std::shared_ptr<AbstractInputProcessor> inputProcessor) : TrackObject(id, flags, inputProcessor, DeviceType::Wired)
{
	sProfileName = std::string("TO_HMD");
	trackObjectDeviceType = TrackObjectDeviceType::HMD;
}


TO_HMD::~TO_HMD()
{
}

bool TO_HMD::init()
{
	const int _VERNUM = 32;
	VerNum = _VERNUM;
	double vertices_tmp[3 * _VERNUM] = {
		0.048416, 0.036757, 0.017365, //0
		0.081359, 0.045010, -0.000500,  //1
		-0.052042, 0.052817, -0.002500, //2
		-0.048416, 0.036757, 0.017365,//3
		0.024452, 0.020629, 0.026250,//4
		0.052042, 0.052817, -0.002500,//5
		-0.081359, 0.045010, -0.000500,//6
		-0.024452, 0.020629, 0.026250,//7
		0.086360, 0.016778, 0.014246,//8
		0.058527, 0.000000, 0.023643,//9
		-0.086360, 0.016778, 0.014246,//10
		-0.058527, 0.000000, 0.023643,//11
		0.091176, 0.028644, -0.003272,//12
		0.048416, -0.036757, 0.017365,//13
		-0.091176, 0.028644, -0.003272,//14
		-0.094372, 0.000000, 0.002750,//15
		0.088042, 0.016129, -0.012660,//16
		0.086360, -0.016778, 0.014246,//17
		-0.088042, 0.016129, -0.012660,//18
		-0.091176, -0.028644, -0.003272,//19
		0.094372, 0.000000, 0.002750,//20
		0.088042, -0.016129, -0.012660,//21
		-0.088042, -0.016129, -0.012660,//22
		-0.086360, -0.016778, 0.014246,//23
		0.091176, -0.028644, -0.003272,//24
		0.081359, -0.045010, -0.000500,//25
		-0.048416, -0.036757, 0.017365,//26
		-0.081359, -0.045010, -0.000500,//27
		0.052042, -0.052817, -0.002500,//28
		0.027890, -0.052122, 0.012061,//29
		-0.027890, -0.052122, 0.012061,//30
		-0.052042, -0.052817, -0.002500,//31
	};
	vertices.resize(_VERNUM * 3);
	for (int i = 0; i < _VERNUM; ++i)
	{
		// with head model
		vertices[i * 3] = -vertices_tmp[i * 3] * 1000;
		vertices[i * 3 + 1] = vertices_tmp[i * 3 + 1] * 1000 - 3;
		vertices[i * 3 + 2] = -vertices_tmp[i * 3 + 2] * 1000 - 64.25;
	}

	double normaltmp[3 * _VERNUM] = {
		0.126900, 0.667800, 0.733400,  // 0
		0.545800, 0.837900, 0.000000,// 1
		-0.586300, 0.810100, 0.000000,// 2
		-0.127200, 0.667800, 0.733400,// 3
		0.080200, 0.289400, 0.953900,// 4
		0.586300, 0.810100, 0.000000,// 5
		-0.545800, 0.837900, 0.000000,//6
		-0.080200, 0.289400, 0.953900,// 7
		0.615100, 0.132500, 0.777200,// 8
		0.457300, 0.000000, 0.889300,// 9
		-0.615100, 0.132500, 0.777200,// 10
		-0.457300, 0.000000, 0.889300,// 11
		0.923200, 0.236300, -0.303000,// 12
		0.127000, -0.667800, 0.733500,// 13
		-0.923200, 0.236300, -0.303000,// 14
		-1.000000, 0.000000, 0.000000,// 15
		0.922100, 0.147400, -0.357700,// 16
		0.615100, -0.132500, 0.777200,// 17
		-0.922100, 0.147400, -0.357700,// 18
		-0.923200, -0.236300, -0.303000,// 19
		1.000000, 0.000000, 0.000000,// 20
		0.922100, -0.147400, -0.357700,// 21
		-0.922100, -0.147400, -0.357700,// 22
		-0.615100, -0.132500, 0.777200,// 23
		0.923200, -0.236300, -0.303000,// 24
		0.545800, -0.837900, 0.000000,// 25
		-0.127000, -0.667800, 0.733500,// 26
		-0.545800, -0.837900, 0.000000,// 27
		0.586300, -0.810100, 0.000000,// 28
		0.121300, -0.602700, 0.788700,// 29
		-0.121300, -0.602700, 0.788700,// 30
		-0.586300, -0.810100, 0.000000// 31
	};

	normals.resize(_VERNUM * 3);

	for (int i = 0; i < _VERNUM; ++i)
	{
		normals[i * 3] = -normaltmp[i * 3];
		normals[i * 3 + 1] = normaltmp[i * 3 + 1];
		normals[i * 3 + 2] = -normaltmp[i * 3 + 2];
	}

	initKalman(5, 10);

	_verticesRatioArray.resize(VerNum, 1.0);

	return TrackObject::init();
}

irrmath::Quatd TO_HMD::_transIMU(irrmath::Quatd& quatNow)
{
	return irrmath::Quatd(quatNow.x, quatNow.y, quatNow.z, quatNow.w);
}

irrmath::Quatd TO_HMD::_invertTransIMU(irrmath::Quatd& quatNow)
{
	return irrmath::Quatd(quatNow.x, quatNow.y, quatNow.z, quatNow.w);
}

}
