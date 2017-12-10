#pragma once

#include <math.h>

#include "matrix.h"

namespace math
{

class quaternion : public vector<4>
{
public:
	/**
	 * trivial ctor
	 */
	quaternion() : vector<4>() {}

	/**
	 * copy ctor
	 */
	quaternion(const quaternion &q) : vector<4>(q) {}

	/**
	 * casting from vector
	 */
	quaternion(const vector<4> &v) : vector<4>(v) {}

	/**
	 * setting ctor
	 */
	quaternion(const float d[4]) : vector<4>(d) {}

	/**
	 * setting ctor
	 */
	quaternion(const float a0, const float b0, const float c0, const float d0): vector<4>(a0, b0, c0, d0) {}

	using vector<4>::operator *;

	/**
	 * multiplication
	 */
	const quaternion operator *(const quaternion &q) const {
		return quaternion(
			       data[0] * q.data[0] - data[1] * q.data[1] - data[2] * q.data[2] - data[3] * q.data[3],
			       data[0] * q.data[1] + data[1] * q.data[0] + data[2] * q.data[3] - data[3] * q.data[2],
			       data[0] * q.data[2] - data[1] * q.data[3] + data[2] * q.data[0] + data[3] * q.data[1],
			       data[0] * q.data[3] + data[1] * q.data[2] - data[2] * q.data[1] + data[3] * q.data[0]);
	}

	/**
	 * derivative
	 */
	const quaternion derivative(const vector<3> &w) {
		float dataQ[] = {
			data[0], -data[1], -data[2], -data[3],
			data[1],  data[0], -data[3],  data[2],
			data[2],  data[3],  data[0], -data[1],
			data[3], -data[2],  data[1],  data[0]
		};
		matrix<4, 4> Q(dataQ);
		vector<4> v(0.0f, w.data[0], w.data[1], w.data[2]);
		return Q * v * 0.5f;
	}

	/**
	 * imaginary part of quaternion
	 */
	vector<3> imag(void) {
		return vector<3>(&data[1]);
	}

	/**
	 * set quaternion to rotation defined by euler angles
	 */
	void from_euler(float roll, float pitch, float yaw) {
		double cosPhi_2 = cos(double(roll) / 2.0);
		double sinPhi_2 = sin(double(roll) / 2.0);
		double cosTheta_2 = cos(double(pitch) / 2.0);
		double sinTheta_2 = sin(double(pitch) / 2.0);
		double cosPsi_2 = cos(double(yaw) / 2.0);
		double sinPsi_2 = sin(double(yaw) / 2.0);
		data[0] = cosPhi_2 * cosTheta_2 * cosPsi_2 + sinPhi_2 * sinTheta_2 * sinPsi_2;
		data[1] = sinPhi_2 * cosTheta_2 * cosPsi_2 - cosPhi_2 * sinTheta_2 * sinPsi_2;
		data[2] = cosPhi_2 * sinTheta_2 * cosPsi_2 + sinPhi_2 * cosTheta_2 * sinPsi_2;
		data[3] = cosPhi_2 * cosTheta_2 * sinPsi_2 - sinPhi_2 * sinTheta_2 * cosPsi_2;
	}

	void from_dcm(const matrix<3, 3> &m) {
		// avoiding singularities by not using division equations
		data[0] = 0.5f * sqrtf(1.0f + m.data[0][0] + m.data[1][1] + m.data[2][2]);
		data[1] = 0.5f * sqrtf(1.0f + m.data[0][0] - m.data[1][1] - m.data[2][2]);
		data[2] = 0.5f * sqrtf(1.0f - m.data[0][0] + m.data[1][1] - m.data[2][2]);
		data[3] = 0.5f * sqrtf(1.0f - m.data[0][0] - m.data[1][1] + m.data[2][2]);
	}

	/**
	 * create rotation matrix for the quaternion
	 */
	matrix<3, 3> to_dcm(void) const {
		matrix<3, 3> R;
		float aSq = data[0] * data[0];
		float bSq = data[1] * data[1];
		float cSq = data[2] * data[2];
		float dSq = data[3] * data[3];
		R.data[0][0] = aSq + bSq - cSq - dSq;
		R.data[0][1] = 2.0f * (data[1] * data[2] - data[0] * data[3]);
		R.data[0][2] = 2.0f * (data[0] * data[2] + data[1] * data[3]);
		R.data[1][0] = 2.0f * (data[1] * data[2] + data[0] * data[3]);
		R.data[1][1] = aSq - bSq + cSq - dSq;
		R.data[1][2] = 2.0f * (data[2] * data[3] - data[0] * data[1]);
		R.data[2][0] = 2.0f * (data[1] * data[3] - data[0] * data[2]);
		R.data[2][1] = 2.0f * (data[0] * data[1] + data[2] * data[3]);
		R.data[2][2] = aSq - bSq - cSq + dSq;
		return R;
	}
};

}

