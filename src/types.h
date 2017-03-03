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

namespace HYPEREAL
{
	/// A vector with 2 elements.
	union Vector2
	{
		struct
		{
			double x;
			double y;
		};
		double val[2];
		Vector2() : x(0), y(0) {};
		Vector2(double x, double y) : x(x), y(y) {};
		Vector2(const double* p) : x(p[0]), y(p[1]) {};		
	};

	/// A vector with 3 elements.
	union Vector3
	{
		struct
		{
			double x;
			double y;
			double z;
		};
		double val[3];
		Vector3() : x(0), y(0), z(0) {};
		Vector3(double x, double y, double z) : x(x), y(y), z(z) {};
		Vector3(const double* p) : x(p[0]), y(p[1]), z(p[2]) {};
		Vector3 operator+(const Vector3& rhs)
		{
			return Vector3(x + rhs.x, y + rhs.y, z + rhs.z);
		}
		Vector3 operator-(const Vector3& rhs)
		{
			return Vector3(x - rhs.x, y - rhs.y, z - rhs.z);
		}
		Vector3& operator+=(const Vector3& rhs)
		{
			x += rhs.x; y += rhs.y; z += rhs.z;
			return *this;
		}
		Vector3& operator-=(const Vector3& rhs)
		{
			x -= rhs.x; y -= rhs.y; z -= rhs.z;
			return *this;
		}
		Vector3& operator*=(const double& rhs)
		{
			x *= rhs; y *= rhs; z *= rhs;
			return *this;
		}
	};

	/// A vector with 4 elements.
	union Vector4
	{
		struct
		{
			double x;
			double y;
			double z;
			double w;
		};
		double val[4];
		Vector4() : x(0), y(0), z(0), w(0){};
		Vector4(const double* p) : x(p[0]), y(p[1]), z(p[2]), w(p[3]) {};
	};

	/// Translation of a object. 
	/// You can use ->x, ->y or ->z to access its value. Or use ->val[0], ->val[1] and ->val[2].
	union Translation
	{
		struct
		{
			double x;
			double y;
			double z;
		};
		double val[3];
		Translation(Vector3 t) : x(t.x), y(t.y), z(t.z) {};
	};

	/// Quaternion of a object. Usually representing a rotation.
	union Quat
	{
		struct
		{
			double x;
			double y;
			double z;
			double w;
		};
		double val[4];
		Quat(Vector4 t) : x(t.x), y(t.y), z(t.z), w(t.w) {};
	};

	/// A 3x3 matrix.
	struct Matrix33
	{
		double val[9];
		Matrix33() { memset(val, 0, sizeof(val)); };
		Matrix33(double* m) { memcpy(val, m, sizeof(val)); };
	};
 
	/// Pose of a tracking object.
	/// Containing the object's translation and rotation in quaternion.
	struct Pose
	{
		Translation position;
		Quat rotationQuat;
	};
}
