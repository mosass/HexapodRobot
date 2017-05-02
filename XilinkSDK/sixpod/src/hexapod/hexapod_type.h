/*
 * hexapod_type.h
 *
 *  Created on: Apr 3, 2017
 *      Author: Phanomphon Yotchon
 */

#ifndef SRC_HEXAPOD_HEXAPOD_TYPE_H_
#define SRC_HEXAPOD_HEXAPOD_TYPE_H_

#include "math.h"

class Rot3d {
public:
	float y;
	float p;
	float r;

	Rot3d (float y = 0, float p = 0, float r = 0){
		this->y = y;
		this->p = p;
		this->r = r;
	}

	static inline float toDeg(float rad){
		return rad / M_PI * 180.0f;
	}

	static inline float toReg(float deg){
		return deg / 180.0f * M_PI;
	}

	inline Rot3d diff(const Rot3d& R){
		Rot3d L;
		if(this->y > R.y)
			L.y = this->y - R.y;
		else
			L.y = R.y - this->y;

		if(this->p > R.p)
			L.p = this->p - R.p;
		else
			L.p = R.p - this->p;

		if(this->r > R.r)
			L.r = this->r - R.r;
		else
			L.r = R.r - this->r;

		return L;
	}

	inline Rot3d operator+(const Rot3d& R){
		Rot3d L;
		L.y = this->y + R.y;
		L.p = this->p + R.p;
		L.r = this->r + R.r;
		return L;
	}

	inline Rot3d operator-(const Rot3d& R){
		Rot3d L;
		L.y = this->y - R.y;
		L.p = this->p - R.p;
		L.r = this->r - R.r;
		return L;
	}

	inline Rot3d operator+(float R){
		Rot3d L;
		L.y = this->y + R;
		L.p = this->p + R;
		L.r = this->r + R;
		return L;
	}

	inline Rot3d operator-(float R){
		Rot3d L;
		L.y = this->y - R;
		L.p = this->p - R;
		L.r = this->r - R;
		return L;
	}

	inline Rot3d operator-(){
		Rot3d L;
		L.y = -this->y;
		L.p = -this->p;
		L.r = -this->r;
		return L;
	}

	inline Rot3d operator*(const Rot3d& R){
		Rot3d L;
		L.y = this->y * R.y;
		L.p = this->p * R.p;
		L.r = this->r * R.r;
		return L;
	}

	inline Rot3d operator/(const Rot3d& R){
		Rot3d L;
		L.y = this->y / R.y;
		L.p = this->p / R.p;
		L.r = this->r / R.r;
		return L;
	}

	inline Rot3d operator*(float R){
		Rot3d L;
		L.y = this->y * R;
		L.p = this->p * R;
		L.r = this->r * R;
		return L;
	}

	inline Rot3d operator/(float R){
		Rot3d L;
		L.y = this->y / R;
		L.p = this->p / R;
		L.r = this->r / R;
		return L;
	}
};

#endif /* SRC_HEXAPOD_HEXAPOD_TYPE_H_ */
