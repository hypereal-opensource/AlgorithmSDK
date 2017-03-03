// Copyright (C) 2002-2012 Nikolaus Gebhardt
// This file is part of the "Irrlicht Engine".
// For conditions of distribution and use, see copyright notice in irrlicht.h

#ifndef __IRR_QUATERNION_H_INCLUDED__
#define __IRR_QUATERNION_H_INCLUDED__

#include "irrTypes.h"
#include "irrMath.h"
#include "matrix4.h"
#include "vector3.h"

// Between Irrlicht 1.7 and Irrlicht 1.8 the quaternion-matrix conversions got fixed.
// This define disables all involved functions completely to allow finding all places
// where the wrong conversions had been in use.
#define IRR_TEST_BROKEN_QUATERNION_USE 0

namespace irr
{
namespace core
{

//! Quaternion class for representing rotations.
/** It provides cheap combinations and avoids gimbal locks.
Also useful for interpolations. */
template <class T>
class quaternion
{
	public:

		//! Default Constructor
		quaternion() : x(0.0), y(0.0), z(0.0), w(1.0) {}

		//! Constructor
		quaternion(T x, T y, T z, T w) : x(x), y(y), z(z), w(w) { }

		//! Constructor which converts euler angles (radians) to a quaternion
		quaternion(T x, T y, T z);

		//! Constructor which converts euler angles (radians) to a quaternion
		quaternion(const vector3<T>& vec);

#if !IRR_TEST_BROKEN_QUATERNION_USE
		//! Constructor which converts a matrix to a quaternion
		quaternion(const matrix4& mat);
#endif

		//! Equalilty operator
		bool operator==(const quaternion<T>& other) const;

		//! inequality operator
		bool operator!=(const quaternion<T>& other) const;

		//! Assignment operator
		inline quaternion<T>& operator=(const quaternion<T>& other);

#if !IRR_TEST_BROKEN_QUATERNION_USE
		//! Matrix assignment operator
		inline quaternion<T>& operator=(const matrix4& other);
#endif

		//! Add operator
		quaternion operator+(const quaternion<T>& other) const;

		//! Multiplication operator ()
		quaternion operator*(const quaternion<T>& other) const;

		//! Multiplication operator with scalar
		quaternion operator*(T s) const;

		//! Multiplication operator with scalar
		quaternion<T>& operator*=(T s);

		//! Multiplication operator
		vector3<T> operator*(const vector3<T>& v) const;

		//! Multiplication operator XXX(maxhe): changed to right side first.
		quaternion<T>& operator*=(const quaternion<T>& other);

		//! Calculates the dot product
		inline T dotProduct(const quaternion<T>& other) const;

		//! Sets new quaternion
		inline quaternion<T>& set(T x, T y, T z, T w);

		//! Sets new quaternion based on euler angles (radians)
		inline quaternion<T>& set(T x, T y, T z);

		//! Sets new quaternion based on euler angles (radians)
		inline quaternion<T>& set(const core::vector3<T>& vec);

		//! Sets new quaternion from other quaternion
		inline quaternion<T>& set(const core::quaternion<T>& quat);

		//! returns if this quaternion equals the other one, taking floating point rounding errors into account
		inline bool equals(const quaternion<T>& other,
				const T tolerance = ROUNDING_ERROR_f64 ) const;

		//! Normalizes the quaternion
		inline quaternion<T>& normalize();

#if !IRR_TEST_BROKEN_QUATERNION_USE
		//! Creates a matrix from this quaternion
		matrix4 getMatrix() const;
#endif

		//! Creates a matrix from this quaternion
		void getMatrix( matrix4 &dest, const core::vector3<T> &translation=core::vector3<T>() ) const;

		/*!
			Creates a matrix from this quaternion
			Rotate about a center point
			shortcut for
			core::quaternion q;
			q.rotationFromTo ( vin[i].Normal, forward );
			q.getMatrixCenter ( lookat, center, newPos );

			core::matrix4 m2;
			m2.setInverseTranslation ( center );
			lookat *= m2;

			core::matrix4 m3;
			m2.setTranslation ( newPos );
			lookat *= m3;

		*/
		void getMatrixCenter( matrix4 &dest, const core::vector3<T> &center, const core::vector3<T> &translation ) const;

		//! Creates a matrix from this quaternion
		inline void getMatrix_transposed( matrix4 &dest ) const;

		//! Inverts this quaternion
		quaternion<T>& makeInverse();

		quaternion<T> inversed() const;

		//! Set this quaternion to the linear interpolation between two quaternions
		/** \param q1 First quaternion to be interpolated.
		\param q2 Second quaternion to be interpolated.
		\param time Progress of interpolation. For time=0 the result is
		q1, for time=1 the result is q2. Otherwise interpolation
		between q1 and q2.
		*/
		quaternion<T>& lerp(quaternion q1, quaternion q2, T time);

		//! Set this quaternion to the result of the spherical interpolation between two quaternions
		/** \param q1 First quaternion to be interpolated.
		\param q2 Second quaternion to be interpolated.
		\param time Progress of interpolation. For time=0 the result is
		q1, for time=1 the result is q2. Otherwise interpolation
		between q1 and q2.
		\param threshold To avoid inaccuracies at the end (time=1) the
		interpolation switches to linear interpolation at some point.
		This value defines how much of the remaining interpolation will
		be calculated with lerp. Everything from 1-threshold up will be
		linear interpolation.
		*/
		quaternion<T>& slerp(quaternion q1, quaternion q2,
				T time, T threshold=.05f);

		//! Create quaternion from rotation angle and rotation axis.
		/** Axis must be unit length.
		The quaternion representing the rotation is
		q = cos(A/2)+sin(A/2)*(x*i+y*j+z*k).
		\param angle Rotation Angle in radians.
		\param axis Rotation axis. */
		quaternion<T>& fromAngleAxis (T angle, const vector3<T>& axis);

		//! Create quaternion from unitRotationAxis * angle
		quaternion<T>& fromRotationVector(const vector3<T>& axis);

		//! Fills an angle (radians) around an axis (unit vector)
		void toAngleAxis (T &angle, core::vector3<T>& axis) const;

		//! Fills an vector, which equals angle (radians) * axis (unit vector)
		void toRotationVector(core::vector3<T>& axis) const;

		//! Output this quaternion to an euler angle (radians)
		void toEuler(vector3<T>& euler) const;

		//! Set quaternion to identity
		quaternion<T>& makeIdentity();

		//! Set quaternion to represent a rotation from one vector to another.
		quaternion<T>& rotationFromTo(const vector3<T>& from, const vector3<T>& to);

		//! Make the quaternion is on the same hemisphere of the other.
		inline void ensureSameHemisphere(const quaternion<T>& o)
		{
			if (dotProduct(o) < 0)
			{
				x = -x;
				y = -y;
				z = -z;
				w = -w;
			}
		}

		//! Get two quaternion's difference in radians.
		inline T getDiffAngle(const quaternion<T>& q) const
		{
			return 2 * acos(abs(dotProduct(q)));
		}


		//! Quaternion elements.
		T x; // vectorial (imaginary) part
		T y;
		T z;
		T w; // real part
};


// Constructor which converts euler angles to a quaternion
template <class T>
inline quaternion<T>::quaternion(T x, T y, T z)
{
	set(x,y,z);
}


// Constructor which converts euler angles to a quaternion
template <class T>
inline quaternion<T>::quaternion(const vector3<T>& vec)
{
	set(vec.x,vec.y,vec.z);
}

#if !IRR_TEST_BROKEN_QUATERNION_USE
// Constructor which converts a matrix to a quaternion
template <class T>
inline quaternion<T>::quaternion(const matrix4& mat)
{
	(*this) = mat;
}
#endif

// equal operator
template <class T>
inline bool quaternion<T>::operator==(const quaternion<T>& other) const
{
	return ((x == other.x) &&
		(y == other.y) &&
		(z == other.z) &&
		(w == other.w));
}

// inequality operator
template <class T>
inline bool quaternion<T>::operator!=(const quaternion<T>& other) const
{
	return !(*this == other);
}

// assignment operator
template <class T>
inline quaternion<T>& quaternion<T>::operator=(const quaternion<T>& other)
{
	x = other.x;
	y = other.y;
	z = other.z;
	w = other.w;
	return *this;
}

#if !IRR_TEST_BROKEN_QUATERNION_USE
// matrix assignment operator
template <class T>
inline quaternion<T>& quaternion<T>::operator=(const matrix4& m)
{
	const T diag = m[0] + m[5] + m[10] + 1;

	if( diag > 0.0 )
	{
		const T scale = sqrt(diag) * 2.0; // get scale from diagonal

		// TODO: speed this up
		x = (m[6] - m[9]) / scale;
		y = (m[8] - m[2]) / scale;
		z = (m[1] - m[4]) / scale;
		w = 0.25f * scale;
	}
	else
	{
		if (m[0]>m[5] && m[0]>m[10])
		{
			// 1st element of diag is greatest value
			// find scale according to 1st element, and double it
			const T scale = sqrt(1.0 + m[0] - m[5] - m[10]) * 2.0;

			// TODO: speed this up
			x = 0.25f * scale;
			y = (m[4] + m[1]) / scale;
			z = (m[2] + m[8]) / scale;
			w = (m[6] - m[9]) / scale;
		}
		else if (m[5]>m[10])
		{
			// 2nd element of diag is greatest value
			// find scale according to 2nd element, and double it
			const T scale = sqrt(1.0 + m[5] - m[0] - m[10]) * 2.0;

			// TODO: speed this up
			x = (m[4] + m[1]) / scale;
			y = 0.25f * scale;
			z = (m[9] + m[6]) / scale;
			w = (m[8] - m[2]) / scale;
		}
		else
		{
			// 3rd element of diag is greatest value
			// find scale according to 3rd element, and double it
			const T scale = sqrt(1.0 + m[10] - m[0] - m[5]) * 2.0;

			// TODO: speed this up
			x = (m[8] + m[2]) / scale;
			y = (m[9] + m[6]) / scale;
			z = 0.25f * scale;
			w = (m[1] - m[4]) / scale;
		}
	}

	return normalize();
}
#endif


// multiplication operator (right side first)
template <class T>
inline quaternion<T> quaternion<T>::operator*(const quaternion<T>& other) const
{
	quaternion tmp;

	tmp.x = (w * other.x) + (x * other.w) + (y * other.z) - (z * other.y);
	tmp.y = (w * other.y) + (y * other.w) + (z * other.x) - (x * other.z);
	tmp.z = (w * other.z) + (z * other.w) + (x * other.y) - (y * other.x);
	tmp.w = (w * other.w) - (x * other.x) - (y * other.y) - (z * other.z);
	return tmp;
}


// multiplication operator
template <class T>
inline quaternion<T> quaternion<T>::operator*(T s) const
{
	return quaternion(s*x, s*y, s*z, s*w);
}


// multiplication operator
template <class T>
inline quaternion<T>& quaternion<T>::operator*=(T s)
{
	x*=s;
	y*=s;
	z*=s;
	w*=s;
	return *this;
}

// multiplication operator
template <class T>
inline quaternion<T>& quaternion<T>::operator*=(const quaternion<T>& other)
{
	return (*this = other * (*this));
}

// add operator
template <class T>
inline quaternion<T> quaternion<T>::operator+(const quaternion<T>& b) const
{
	return quaternion(x+b.x, y+b.y, z+b.z, w+b.w);
}

#if !IRR_TEST_BROKEN_QUATERNION_USE
// Creates a matrix from this quaternion
template <class T>
inline matrix4 quaternion<T>::getMatrix() const
{
	core::matrix4 m;
	getMatrix(m);
	return m;
}
#endif

/*!
	Creates a matrix from this quaternion
*/
template <class T>
inline void quaternion<T>::getMatrix(matrix4 &dest,
		const core::vector3<T> &center) const
{
	dest[0] = 1.0 - 2.0*y*y - 2.0*z*z;
	dest[1] = 2.0*x*y + 2.0*z*w;
	dest[2] = 2.0*x*z - 2.0*y*w;
	dest[3] = 0.0;

	dest[4] = 2.0*x*y - 2.0*z*w;
	dest[5] = 1.0 - 2.0*x*x - 2.0*z*z;
	dest[6] = 2.0*z*y + 2.0*x*w;
	dest[7] = 0.0;

	dest[8] = 2.0*x*z + 2.0*y*w;
	dest[9] = 2.0*z*y - 2.0*x*w;
	dest[10] = 1.0 - 2.0*x*x - 2.0*y*y;
	dest[11] = 0.0;

	dest[12] = center.x;
	dest[13] = center.y;
	dest[14] = center.z;
	dest[15] = 1.0;

	dest.setDefinitelyIdentityMatrix ( false );
}


/*!
	Creates a matrix from this quaternion
	Rotate about a center point
	shortcut for
	core::quaternion q;
	q.rotationFromTo(vin[i].Normal, forward);
	q.getMatrix(lookat, center);

	core::matrix4 m2;
	m2.setInverseTranslation(center);
	lookat *= m2;
*/
template <class T>
inline void quaternion<T>::getMatrixCenter(matrix4 &dest,
					const core::vector3<T> &center,
					const core::vector3<T> &translation) const
{
	dest[0] = 1.0 - 2.0*y*y - 2.0*z*z;
	dest[1] = 2.0*x*y + 2.0*z*w;
	dest[2] = 2.0*x*z - 2.0*y*w;
	dest[3] = 0.0;

	dest[4] = 2.0*x*y - 2.0*z*w;
	dest[5] = 1.0 - 2.0*x*x - 2.0*z*z;
	dest[6] = 2.0*z*y + 2.0*x*w;
	dest[7] = 0.0;

	dest[8] = 2.0*x*z + 2.0*y*w;
	dest[9] = 2.0*z*y - 2.0*x*w;
	dest[10] = 1.0 - 2.0*x*x - 2.0*y*y;
	dest[11] = 0.0;

	dest.setRotationCenter ( center, translation );
}

// Creates a matrix from this quaternion
template <class T>
inline void quaternion<T>::getMatrix_transposed(matrix4 &dest) const
{
	dest[0] = 1.0 - 2.0*y*y - 2.0*z*z;
	dest[4] = 2.0*x*y + 2.0*z*w;
	dest[8] = 2.0*x*z - 2.0*y*w;
	dest[12] = 0.0;

	dest[1] = 2.0*x*y - 2.0*z*w;
	dest[5] = 1.0 - 2.0*x*x - 2.0*z*z;
	dest[9] = 2.0*z*y + 2.0*x*w;
	dest[13] = 0.0;

	dest[2] = 2.0*x*z + 2.0*y*w;
	dest[6] = 2.0*z*y - 2.0*x*w;
	dest[10] = 1.0 - 2.0*x*x - 2.0*y*y;
	dest[14] = 0.0;

	dest[3] = 0.0;
	dest[7] = 0.0;
	dest[11] = 0.0;
	dest[15] = 1.0;

	dest.setDefinitelyIdentityMatrix(false);
}


// Inverts this quaternion
template <class T>
inline quaternion<T>& quaternion<T>::makeInverse()
{
	x = -x; y = -y; z = -z;
	return *this;
}

// Returns a new quaternion, which is inversed.
template <class T>
inline quaternion<T> quaternion<T>::inversed() const
{
	quaternion<T> q(*this);
	q.makeInverse();
	return q;
}

// sets new quaternion
template <class T>
inline quaternion<T>& quaternion<T>::set(T x, T y, T z, T w)
{
	x = x;
	y = y;
	z = z;
	w = w;
	return *this;
}


// sets new quaternion based on euler angles
template <class T>
inline quaternion<T>& quaternion<T>::set(T x, T y, T z)
{
	f64 angle;

	angle = x * 0.5;
	const f64 sr = sin(angle);
	const f64 cr = cos(angle);

	angle = y * 0.5;
	const f64 sp = sin(angle);
	const f64 cp = cos(angle);

	angle = z * 0.5;
	const f64 sy = sin(angle);
	const f64 cy = cos(angle);

	const f64 cpcy = cp * cy;
	const f64 spcy = sp * cy;
	const f64 cpsy = cp * sy;
	const f64 spsy = sp * sy;

	x = (T)(sr * cpcy - cr * spsy);
	y = (T)(cr * spcy + sr * cpsy);
	z = (T)(cr * cpsy - sr * spcy);
	w = (T)(cr * cpcy + sr * spsy);

	return normalize();
}

// sets new quaternion based on euler angles
template <class T>
inline quaternion<T>& quaternion<T>::set(const core::vector3<T>& vec)
{
	return set(vec.x, vec.y, vec.z);
}

// sets new quaternion based on other quaternion
template <class T>
inline quaternion<T>& quaternion<T>::set(const core::quaternion<T>& quat)
{
	return (*this=quat);
}


//! returns if this quaternion equals the other one, taking floating point rounding errors into account
template <class T>
inline bool quaternion<T>::equals(const quaternion<T>& other, const T tolerance) const
{
	return core::equals(x, other.x, tolerance) &&
		core::equals(y, other.y, tolerance) &&
		core::equals(z, other.z, tolerance) &&
		core::equals(w, other.w, tolerance);
}


// normalizes the quaternion
template <class T>
inline quaternion<T>& quaternion<T>::normalize()
{
	const T n = x*x + y*y + z*z + w*w;

	if (n == 1)
		return *this;

	//n = 1.0 / sqrt(n);
	return (*this *= reciprocal_squareroot ( n ));
}


// set this quaternion to the result of the linear interpolation between two quaternions
template <class T>
inline quaternion<T>& quaternion<T>::lerp(quaternion q1, quaternion q2, T time)
{
	const T scale = 1.0 - time;
	*this = (q1*scale) + (q2*time);
	this->normalize();
	return *this;
}


// set this quaternion to the result of the interpolation between two quaternions
template <class T>
inline quaternion<T>& quaternion<T>::slerp(quaternion q1, quaternion q2, T time, T threshold)
{
	T angle = q1.dotProduct(q2);

	// make sure we use the short rotation
	if (angle < 0.0)
	{
		q1 *= -1.0;
		angle *= -1.0;
	}

	if (angle <= (1-threshold)) // spherical interpolation
	{
		const T theta = acos(angle);
		const T invsintheta = reciprocal((T)sin(theta));
		const T scale = (T)sin(theta * (1.0-time)) * invsintheta;
		const T invscale = (T)sin(theta * time) * invsintheta;
		return (*this = (q1*scale) + (q2*invscale));
	}
	else // linear interploation
		return lerp(q1,q2,time);
}


// calculates the dot product
template <class T>
inline T quaternion<T>::dotProduct(const quaternion<T>& q2) const
{
	return (x * q2.x) + (y * q2.y) + (z * q2.z) + (w * q2.w);
}


//! axis could be any length, angle in radians
template <class T>
inline quaternion<T>& quaternion<T>::fromAngleAxis(T angle, const vector3<T>& axis)
{
    vector3<T> normalizedAxis = axis.normalized();
	const T fHalfAngle = 0.5f*angle;
	const T fSin = (T)sin(fHalfAngle);
	w = (T)cos(fHalfAngle);
	x = fSin*normalizedAxis.x;
	y = fSin*normalizedAxis.y;
	z = fSin*normalizedAxis.z;
	return *this;
}

template <class T>
inline quaternion<T>& quaternion<T>::fromRotationVector(const vector3<T>& axis)
{
	return fromAngleAxis(axis.getLength(), axis);
}

template <class T>
inline void quaternion<T>::toAngleAxis(T &angle, core::vector3<T> &axis) const
{
	const T scale = sqrt(x*x + y*y + z*z);

	if (core::iszero(scale) || w > 1.0 || w < -1.0)
	{
		angle = 0.0;
		axis.x = 0.0;
		axis.y = 1.0;
		axis.z = 0.0;
	}
	else
	{
		const T invscale = reciprocal(scale);
		angle = 2.0 * acos(w);
		axis.x = x * invscale;
		axis.y = y * invscale;
		axis.z = z * invscale;
	}
}

template <class T>
inline void quaternion<T>::toRotationVector(core::vector3<T>& axis) const
{
	double angle;
	toAngleAxis(angle, axis);
	axis *= angle;
}

template <class T>
inline void quaternion<T>::toEuler(vector3<T>& euler) const
{
	// XXX(maxhe): To get yaw pitch roll.
	T X = this->y;
	T Y = this->x;
	T Z = this->z;
	T W = this->w;

	const f64 sqw = W*W;
	const f64 sqx = X*X;
	const f64 sqy = Y*Y;
	const f64 sqz = Z*Z;
	const f64 test = 2.0 * (Y*W - X*Z);

	if (core::equals(test, 1.0, 0.000001))
	{
		// heading = rotation about z-axis
		euler.z = (T) (-2.0*atan2(X, W));
		// bank = rotation about x-axis
		euler.x = 0;
		// attitude = rotation about y-axis
		euler.y = (T) (core::PI64/2.0);
	}
	else if (core::equals(test, -1.0, 0.000001))
	{
		// heading = rotation about z-axis
		euler.z = (T) (2.0*atan2(X, W));
		// bank = rotation about x-axis
		euler.x = 0;
		// attitude = rotation about y-axis
		euler.y = (T) (core::PI64/-2.0);
	}
	else
	{
		// heading = rotation about z-axis
		euler.z = (T) atan2(2.0 * (X*Y +Z*W),(sqx - sqy - sqz + sqw));
		// bank = rotation about x-axis
		euler.x = (T) atan2(2.0 * (Y*Z +X*W),(-sqx - sqy + sqz + sqw));
		// attitude = rotation about y-axis
		euler.y = (T) asin( clamp(test, -1.0, 1.0) );
	}
}


template <class T>
inline vector3<T> quaternion<T>::operator* (const vector3<T>& v) const
{
	// nVidia SDK implementation

	vector3<T> uv, uuv;
	vector3<T> qvec(x, y, z);
	uv = qvec.crossProduct(v);
	uuv = qvec.crossProduct(uv);
	uv *= (2.0 * w);
	uuv *= 2.0;

	return v + uv + uuv;
}

// set quaternion to identity
template <class T>
inline core::quaternion<T>& quaternion<T>::makeIdentity()
{
	w = 1.0;
	x = 0.0;
	y = 0.0;
	z = 0.0;
	return *this;
}

template <class T>
inline core::quaternion<T>& quaternion<T>::rotationFromTo(const vector3<T>& from, const vector3<T>& to)
{
	// Based on Stan Melax's article in Game Programming Gems
	// Copy, since cannot modify local
	vector3<T> v0 = from;
	vector3<T> v1 = to;
	v0.normalize();
	v1.normalize();

	const T d = v0.dotProduct(v1);
	if (d >= 1.0) // If dot == 1, vectors are the same
	{
		return makeIdentity();
	}
	else if (d <= -1.0) // exactly opposite
	{
		core::vector3<T> axis(1.0, 0.0, 0.0);
		axis = axis.crossProduct(v0);
		if (axis.getLength()==0)
		{
			axis.set(0.0,1.0,0.0);
			axis = axis.crossProduct(v0);
		}
		// same as fromAngleAxis(core::PI, axis).normalize();
		return set(axis.x, axis.y, axis.z, 0).normalize();
	}

	const T s = sqrt( (1+d)*2 ); // optimize inv_sqrt
	const T invs = 1.0 / s;
	const vector3<T> c = v0.crossProduct(v1)*invs;
	return set(c.x, c.y, c.z, s * 0.5f).normalize();
}

    template <class T>
    using Quat = quaternion<T>;
    typedef Quat<f32> Quatf;
    typedef Quat<f64> Quatd;


} // end namespace core
} // end namespace irr

#endif

