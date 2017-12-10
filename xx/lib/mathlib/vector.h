#pragma once

#include <stdio.h>
#include <math.h>

#include "arm_math.h"
//#include "platforms/CMSIS/Include/arm_math.h"
//#include <platforms/ros/eigen_math.h>

#include "mathtype.h"

namespace math
{

template <unsigned int N>
class vector;

template <unsigned int N>
class vectorbase
{
public:
	/**
	 * vector data
	 */
	float data[N];

	/**
	 * struct for using arm_math functions, represents column vector
	 */
	arm_matrix_instance_f32 arm_col;


	/**
	 * trivial ctor
	 * initializes elements to zero
	 */
	vectorbase()//:
		//data{},
		//arm_col{N, 1, &data[0]}
	{
        memset(data, 0, sizeof(data));
        arm_col.numRows = N;
        arm_col.numCols = 1;
        arm_col.pData = &data[0];
	}

	virtual ~vectorbase() { };

	/**
	 * copy ctor
	 */
	vectorbase(const vectorbase<N> &v) //:
		//arm_col{N, 1, &data[0]}
	{
        arm_col.numRows = N;
        arm_col.numCols = 1;
        arm_col.pData = &data[0];
		memcpy(data, v.data, sizeof(data));
	}

	/**
	 * setting ctor
	 */
	vectorbase(const float d[N]) //:
		//arm_col{N, 1, &data[0]}
	{
        arm_col.numRows = N;
        arm_col.numCols = 1;
        arm_col.pData = &data[0];
		memcpy(data, d, sizeof(data));
	}

	/**
	 * set data
	 */
	void set(const float d[N]) {
		memcpy(data, d, sizeof(data));
	}

	/**
	 * access to elements by index
	 */
	float &operator()(const unsigned int i) {
		return data[i];
	}

	/**
	 * access to elements by index
	 */
	float operator()(const unsigned int i) const {
		return data[i];
	}

	/**
	 * get vector size
	 */
	unsigned int get_size() const {
		return N;
	}

	/**
	 * test for equality
	 */
	bool operator ==(const vector<N> &v) const {
		for (unsigned int i = 0; i < N; i++)
			if (data[i] != v.data[i])
				return false;

		return true;
	}

	/**
	 * test for inequality
	 */
	bool operator !=(const vector<N> &v) const {
		for (unsigned int i = 0; i < N; i++)
			if (data[i] != v.data[i])
				return true;

		return false;
	}

	/**
	 * set to value
	 */
	const vector<N> &operator =(const vector<N> &v) {
		memcpy(data, v.data, sizeof(data));
		return *static_cast<const vector<N>*>(this);
	}

	/**
	 * negation
	 */
	const vector<N> operator -(void) const {
		vector<N> res;

		for (unsigned int i = 0; i < N; i++)
			res.data[i] = -data[i];

		return res;
	}

	/**
	 * addition
	 */
	const vector<N> operator +(const vector<N> &v) const {
		vector<N> res;

		for (unsigned int i = 0; i < N; i++)
			res.data[i] = data[i] + v.data[i];

		return res;
	}

	/**
	 * subtraction
	 */
	const vector<N> operator -(const vector<N> &v) const {
		vector<N> res;

		for (unsigned int i = 0; i < N; i++)
			res.data[i] = data[i] - v.data[i];

		return res;
	}

	/**
	 * uniform scaling
	 */
	const vector<N> operator *(const float num) const {
		vector<N> res;

		for (unsigned int i = 0; i < N; i++)
			res.data[i] = data[i] * num;

		return res;
	}

	/**
	 * uniform scaling
	 */
	const vector<N> operator /(const float num) const {
		vector<N> res;

		for (unsigned int i = 0; i < N; i++)
			res.data[i] = data[i] / num;

		return res;
	}

	/**
	 * addition
	 */
	const vector<N> &operator +=(const vector<N> &v) {
		for (unsigned int i = 0; i < N; i++)
			data[i] += v.data[i];

		return *static_cast<const vector<N>*>(this);
	}

	/**
	 * subtraction
	 */
	const vector<N> &operator -=(const vector<N> &v) {
		for (unsigned int i = 0; i < N; i++)
			data[i] -= v.data[i];

		return *static_cast<const vector<N>*>(this);
	}

	/**
	 * uniform scaling
	 */
	const vector<N> &operator *=(const float num) {
		for (unsigned int i = 0; i < N; i++)
			data[i] *= num;

		return *static_cast<const vector<N>*>(this);
	}

	/**
	 * uniform scaling
	 */
	const vector<N> &operator /=(const float num) {
		for (unsigned int i = 0; i < N; i++)
			data[i] /= num;

		return *static_cast<const vector<N>*>(this);
	}

	/**
	 * dot product
	 */
	float operator *(const vector<N> &v) const {
		float res = 0.0f;

		for (unsigned int i = 0; i < N; i++)
			res += data[i] * v.data[i];

		return res;
	}

	/**
	 * element by element multiplication
	 */
	const vector<N> emult(const vector<N> &v) const {
		vector<N> res;

		for (unsigned int i = 0; i < N; i++)
			res.data[i] = data[i] * v.data[i];

		return res;
	}

	/**
	 * element by element division
	 */
	const vector<N> edivide(const vector<N> &v) const {
		vector<N> res;

		for (unsigned int i = 0; i < N; i++)
			res.data[i] = data[i] / v.data[i];

		return res;
	}

	/**
	 * gets the length of this vector squared
	 */
	float length_squared() const {
		float res = 0.0f;

		for (unsigned int i = 0; i < N; i++)
			res += data[i] * data[i];

		return res;
	}

	/**
	 * gets the length of this vector
	 */
	float length() const {
		float res = 0.0f;

		for (unsigned int i = 0; i < N; i++)
			res += data[i] * data[i];

		return sqrtf(res);
	}

	/**
	 * normalizes this vector
	 */
	void normalize() {
		*this /= length();
	}

	/**
	 * returns the normalized version of this vector
	 */
	vector<N> normalized() const {
		return *this / length();
	}

	/**
	 * set zero vector
	 */
	void zero(void) {
		memset(data, 0, sizeof(data));
	}

	void print(void) {
		printf("[ ");

		for (unsigned int i = 0; i < N; i++)
			printf("%.3f\t", data[i]);

		printf("]\n");
	}
};

template <unsigned int N>
class vector : public vectorbase<N>
{
public:
	vector() : vectorbase<N>() {}

	vector(const vector<N> &v) : vectorbase<N>(v) {}

	vector(const float d[N]) : vectorbase<N>(d) {}

	/**
	 * set to value
	 */
	const vector<N> &operator =(const vector<N> &v) {
		memcpy(this->data, v.data, sizeof(this->data));
		return *this;
	}
};

template <>
class vector<2> : public vectorbase<2>
{
public:
	vector() : vectorbase<2>() {}

	// simple copy is 1.6 times faster than memcpy
	vector(const vector<2> &v) : vectorbase<2>() {
		data[0] = v.data[0];
		data[1] = v.data[1];
	}

	vector(const float d[2]) : vectorbase<2>() {
		data[0] = d[0];
		data[1] = d[1];
	}

	vector(const float x, const float y) : vectorbase<2>() {
		data[0] = x;
		data[1] = y;
	}

	/**
	 * set data
	 */
	void set(const float d[2]) {
		data[0] = d[0];
		data[1] = d[1];
	}

	/**
	 * set to value
	 */
	const vector<2> &operator =(const vector<2> &v) {
		data[0] = v.data[0];
		data[1] = v.data[1];
		return *this;
	}

	float operator %(const vector<2> &v) const {
		return data[0] * v.data[1] - data[1] * v.data[0];
	}
};

template <>
class vector<3> : public vectorbase<3>
{
public:
	vector() : vectorbase<3>() {}

	// simple copy is 1.6 times faster than memcpy
	vector(const vector<3> &v) : vectorbase<3>() {
		for (unsigned int i = 0; i < 3; i++)
			data[i] = v.data[i];
	}

	vector(const float d[3]) : vectorbase<3>() {
		for (unsigned int i = 0; i < 3; i++)
			data[i] = d[i];
	}

	vector(const float x, const float y, const float z) : vectorbase<3>() {
		data[0] = x;
		data[1] = y;
		data[2] = z;
	}

	/**
	 * set data
	 */
	void set(const float d[3]) {
		for (unsigned int i = 0; i < 3; i++)
			data[i] = d[i];
	}

	/**
	 * set to value
	 */
	const vector<3> &operator =(const vector<3> &v) {
		for (unsigned int i = 0; i < 3; i++)
			data[i] = v.data[i];

		return *this;
	}

	vector<3> operator %(const vector<3> &v) const {
		return vector<3>(
			       data[1] * v.data[2] - data[2] * v.data[1],
			       data[2] * v.data[0] - data[0] * v.data[2],
			       data[0] * v.data[1] - data[1] * v.data[0]
		       );
	}
};

template <>
class vector<4> : public vectorbase<4>
{
public:
	vector() : vectorbase() {}

	vector(const vector<4> &v) : vectorbase<4>() {
		for (unsigned int i = 0; i < 4; i++)
			data[i] = v.data[i];
	}

	vector(const float d[4]) : vectorbase<4>() {
		for (unsigned int i = 0; i < 4; i++)
			data[i] = d[i];
	}

	vector(const float x0, const float x1, const float x2, const float x3) : vectorbase() {
		data[0] = x0;
		data[1] = x1;
		data[2] = x2;
		data[3] = x3;
	}

	/**
	 * set data
	 */
	void set(const float d[4]) {
		for (unsigned int i = 0; i < 4; i++)
			data[i] = d[i];
	}

	/**
	 * set to value
	 */
	const vector<4> &operator =(const vector<4> &v) {
		for (unsigned int i = 0; i < 4; i++)
			data[i] = v.data[i];

		return *this;
	}
};

}

