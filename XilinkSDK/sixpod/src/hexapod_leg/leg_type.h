/*
 * leg_type.h
 *
 *  Created on: Apr 3, 2017
 *      Author: mosas
 */

#ifndef SRC_HEXAPOD_LEG_LEG_TYPE_H_
#define SRC_HEXAPOD_LEG_LEG_TYPE_H_

class FootTip {
public:
	float x;
	float y;
	float z;

	FootTip (float x = 0, float y = 0, float z = 0){
		this->x = x;
		this->y = y;
		this->z = z;
	}

	inline FootTip diff(const FootTip& R){
		FootTip L;
		if(this->x > R.x)
			L.x = this->x - R.x;
		else
			L.x = R.x - this->x;

		if(this->y > R.y)
			L.y = this->y - R.y;
		else
			L.y = R.y - this->y;

		if(this->z > R.z)
			L.z = this->z - R.z;
		else
			L.z = R.z - this->z;

		return L;
	}

	inline FootTip operator+(const FootTip& R){
		FootTip L;
		L.x = this->x + R.x;
		L.y = this->y + R.y;
		L.z = this->z + R.z;
		return L;
	}

	inline FootTip operator-(const FootTip& R){
		FootTip L;
		L.x = this->x - R.x;
		L.y = this->y - R.y;
		L.z = this->z - R.z;
		return L;
	}

	inline FootTip operator-(){
		FootTip L;
		L.x = -this->x;
		L.y = -this->y;
		L.z = -this->z;
		return L;
	}

	inline FootTip operator*(const FootTip& R){
		FootTip L;
		L.x = this->x * R.x;
		L.y = this->y * R.y;
		L.z = this->z * R.z;
		return L;
	}

	inline FootTip operator/(const FootTip& R){
		FootTip L;
		L.x = this->x / R.x;
		L.y = this->y / R.y;
		L.z = this->z / R.z;
		return L;
	}

	inline FootTip operator*(float R){
		FootTip L;
		L.x = this->x * R;
		L.y = this->y * R;
		L.z = this->z * R;
		return L;
	}

	inline FootTip operator/(float R){
		FootTip L;
		L.x = this->x / R;
		L.y = this->y / R;
		L.z = this->z / R;
		return L;
	}
};

class Link3d {
public:
	float a;
	float b;
	float c;

	Link3d (float a = 0, float b = 0, float c = 0){
		this->a = a;
		this->b = b;
		this->c = c;
	}

	inline Link3d diff(const Link3d& R){
		Link3d L;
		if(this->a > R.a)
			L.a = this->a - R.a;
		else
			L.a = R.a - this->a;

		if(this->b > R.b)
			L.b = this->b - R.b;
		else
			L.b = R.b - this->b;

		if(this->c > R.c)
			L.c = this->c - R.c;
		else
			L.c = R.c - this->c;

		return L;
	}

	inline Link3d operator+(const Link3d& R){
		Link3d L;
		L.a = this->a + R.a;
		L.b = this->b + R.b;
		L.c = this->c + R.c;
		return L;
	}

	inline Link3d operator-(const Link3d& R){
		Link3d L;
		L.a = this->a - R.a;
		L.b = this->b - R.b;
		L.c = this->c - R.c;
		return L;
	}

	inline Link3d operator+(float R){
		Link3d L;
		L.a = this->a + R;
		L.b = this->b + R;
		L.c = this->c + R;
		return L;
	}

	inline Link3d operator-(float R){
		Link3d L;
		L.a = this->a - R;
		L.b = this->b - R;
		L.c = this->c - R;
		return L;
	}

	inline Link3d operator-(){
		Link3d L;
		L.a = -this->a;
		L.b = -this->b;
		L.c = -this->c;
		return L;
	}

	inline Link3d operator*(const Link3d& R){
		Link3d L;
		L.a = this->a * R.a;
		L.b = this->b * R.b;
		L.c = this->c * R.c;
		return L;
	}

	inline Link3d operator/(const Link3d& R){
		Link3d L;
		L.a = this->a / R.a;
		L.b = this->b / R.b;
		L.c = this->c / R.c;
		return L;
	}

	inline Link3d operator*(float R){
		Link3d L;
		L.a = this->a * R;
		L.b = this->b * R;
		L.c = this->c * R;
		return L;
	}

	inline Link3d operator/(float R){
		Link3d L;
		L.a = this->a / R;
		L.b = this->b / R;
		L.c = this->c / R;
		return L;
	}
};

#endif /* SRC_HEXAPOD_LEG_LEG_TYPE_H_ */
