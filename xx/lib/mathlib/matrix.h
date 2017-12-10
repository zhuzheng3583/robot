#pragma once

#include <stdio.h>
#include <math.h>

#include "arm_math.h"
//#include "platforms/CMSIS/Include/arm_math.h"
//#include <platforms/ros/eigen_math.h>
//#include <Eigen/Eigen>

#include "mathtype.h"

namespace math
{

template<unsigned int M, unsigned int N>
class matrix;

// MxN matrix with float elements
template <unsigned int M, unsigned int N>
class matrixbase
{
public:
	/**
	 * matrix data[row][col]
	 */
	float data[M][N];

	/**
	 * struct for using arm_math functions
	 */
	arm_matrix_instance_f32 arm_mat;

	/**
	 * trivial ctor
	 * Initializes the elements to zero.
	 */
	matrixbase() //:
		//data{},
		//arm_mat{M, N, &data[0][0]}
	{
        memset(data, 0, sizeof(data));
        arm_mat.numRows = M;
        arm_mat.numCols = N;
        arm_mat.pData = &data[0][0];
	}

	virtual ~matrixbase() {};

	/**
	 * copyt ctor
	 */
	matrixbase(const matrixbase<M, N> &m) //:
		//arm_mat{M, N, &data[0][0]}
	{
        arm_mat.numRows = M;
        arm_mat.numCols = N;
        arm_mat.pData = &data[0][0];
		memcpy(data, m.data, sizeof(data));
	}

	matrixbase(const float *d) //:
		//arm_mat{M, N, &data[0][0]}
	{
        arm_mat.numRows = M;
        arm_mat.numCols = N;
        arm_mat.pData = &data[0][0];
		memcpy(data, d, sizeof(data));
	}

	matrixbase(const float d[M][N]) //:
		//arm_mat{M, N, &data[0][0]}
	{
        arm_mat.numRows = M;
        arm_mat.numCols = N;
        arm_mat.pData = &data[0][0];
		memcpy(data, d, sizeof(data));
	}

	/**
	 * set data
	 */
	void set(const float *d) {
		memcpy(data, d, sizeof(data));
	}

	/**
	 * set data
	 */
	void set(const float d[M][N]) {
		memcpy(data, d, sizeof(data));
	}

#if defined(__PX4_ROS)
	/**
	 * set data from boost::array
	 */
	void set(const boost::array<float, 9ul> d) {
	set(static_cast<const float*>(d.data()));
	}
#endif

	/**
	 * access by index
	 */
	float &operator()(const unsigned int row, const unsigned int col) {
		return data[row][col];
	}

	/**
	 * access by index
	 */
	float operator()(const unsigned int row, const unsigned int col) const {
		return data[row][col];
	}

	/**
	 * get rows number
	 */
	unsigned int get_rows() const {
		return M;
	}

	/**
	 * get columns number
	 */
	unsigned int get_cols() const {
		return N;
	}

	/**
	 * test for equality
	 */
	bool operator ==(const matrix<M, N> &m) const {
		for (unsigned int i = 0; i < M; i++)
			for (unsigned int j = 0; j < N; j++)
				if (data[i][j] != m.data[i][j])
					return false;

		return true;
	}

	/**
	 * test for inequality
	 */
	bool operator !=(const matrix<M, N> &m) const {
		for (unsigned int i = 0; i < M; i++)
			for (unsigned int j = 0; j < N; j++)
				if (data[i][j] != m.data[i][j])
					return true;

		return false;
	}

	/**
	 * set to value
	 */
	const matrix<M, N> &operator =(const matrix<M, N> &m) {
		memcpy(data, m.data, sizeof(data));
		return *static_cast<matrix<M, N>*>(this);
	}

	/**
	 * negation
	 */
	matrix<M, N> operator -(void) const {
		matrix<M, N> res;

		for (unsigned int i = 0; i < M; i++)
			for (unsigned int j = 0; j < N; j++)
				res.data[i][j] = -data[i][j];

		return res;
	}

	/**
	 * addition
	 */
	matrix<M, N> operator +(const matrix<M, N> &m) const {
		matrix<M, N> res;

		for (unsigned int i = 0; i < M; i++)
			for (unsigned int j = 0; j < N; j++)
				res.data[i][j] = data[i][j] + m.data[i][j];

		return res;
	}

	matrix<M, N> &operator +=(const matrix<M, N> &m) {
		for (unsigned int i = 0; i < M; i++)
			for (unsigned int j = 0; j < N; j++)
				data[i][j] += m.data[i][j];

		return *static_cast<matrix<M, N>*>(this);
	}

	/**
	 * subtraction
	 */
	matrix<M, N> operator -(const matrix<M, N> &m) const {
		matrix<M, N> res;

		for (unsigned int i = 0; i < M; i++)
			for (unsigned int j = 0; j < N; j++)
				res.data[i][j] = data[i][j] - m.data[i][j];

		return res;
	}

	matrix<M, N> &operator -=(const matrix<M, N> &m) {
		for (unsigned int i = 0; i < M; i++)
			for (unsigned int j = 0; j < N; j++)
				data[i][j] -= m.data[i][j];

		return *static_cast<matrix<M, N>*>(this);
	}

	/**
	 * uniform scaling
	 */
	matrix<M, N> operator *(const float num) const {
		matrix<M, N> res;

		for (unsigned int i = 0; i < M; i++)
			for (unsigned int j = 0; j < N; j++)
				res.data[i][j] = data[i][j] * num;

		return res;
	}

	matrix<M, N> &operator *=(const float num) {
		for (unsigned int i = 0; i < M; i++)
			for (unsigned int j = 0; j < N; j++)
				data[i][j] *= num;

		return *static_cast<matrix<M, N>*>(this);
	}

	matrix<M, N> operator /(const float num) const {
		matrix<M, N> res;

		for (unsigned int i = 0; i < M; i++)
			for (unsigned int j = 0; j < N; j++)
				res[i][j] = data[i][j] / num;

		return res;
	}

	matrix<M, N> &operator /=(const float num) {
		for (unsigned int i = 0; i < M; i++)
			for (unsigned int j = 0; j < N; j++)
				data[i][j] /= num;

		return *static_cast<matrix<M, N>*>(this);
	}

	/**
	 * multiplication by another matrix
	 */
	template <unsigned int P>
	matrix<M, P> operator *(const matrix<N, P> &m) const {
		matrix<M, P> res;
		arm_mat_mult_f32(&arm_mat, &m.arm_mat, &res.arm_mat);
		return res;
	}

	/**
	 * transpose the matrix
	 */
	matrix<N, M> transposed(void) const {
		matrix<N, M> res;
		arm_mat_trans_f32(&this->arm_mat, &res.arm_mat);
		return res;
	}

	/**
	 * invert the matrix
	 */
	matrix<M, N> inversed(void) const {
		matrix<M, N> res;
		arm_mat_inverse_f32(&this->arm_mat, &res.arm_mat);
		return res;
	}

	/**
	 * set zero matrix
	 */
	void zero(void) {
		memset(data, 0, sizeof(data));
	}

	/**
	 * set identity matrix
	 */
	void identity(void) {
		memset(data, 0, sizeof(data));
		unsigned int n = (M < N) ? M : N;

		for (unsigned int i = 0; i < n; i++)
			data[i][i] = 1;
	}

	void print(void) {
		for (unsigned int i = 0; i < M; i++) {
			printf("[ ");

			for (unsigned int j = 0; j < N; j++)
				printf("%.3f\t", data[i][j]);

			printf(" ]\n");
		}
	}
};

template <unsigned int M, unsigned int N>
class matrix : public matrixbase<M, N>
{
public:
	using matrixbase<M, N>::operator *;

	matrix() : matrixbase<M, N>() {}

	matrix(const matrix<M, N> &m) : matrixbase<M, N>(m) {}

	matrix(const float *d) : matrixbase<M, N>(d) {}

	matrix(const float d[M][N]) : matrixbase<M, N>(d) {}

	/**
	 * set to value
	 */
	const matrix<M, N> &operator =(const matrix<M, N> &m) {
		memcpy(this->data, m.data, sizeof(this->data));
		return *this;
	}

	/**
	 * multiplication by a vector
	 */
	vector<M> operator *(const vector<N> &v) const {
		vector<M> res;
		arm_mat_mult_f32(&this->arm_mat, &v.arm_col, &res.arm_col);
		return res;
	}
};

template <>
class matrix<3, 3> : public matrixbase<3, 3>
{
public:
	using matrixbase<3, 3>::operator *;

	matrix() : matrixbase<3, 3>() {}

	matrix(const matrix<3, 3> &m) : matrixbase<3, 3>(m) {}

	matrix(const float *d) : matrixbase<3, 3>(d) {}

	matrix(const float d[3][3]) : matrixbase<3, 3>(d) {}

	/**
	 * set to value
	 */
	const matrix<3, 3> &operator =(const matrix<3, 3> &m) {
		memcpy(this->data, m.data, sizeof(this->data));
		return *this;
	}

	/**
	 * multiplication by a vector
	 */
	vector<3> operator *(const vector<3> &v) const {
		vector<3> res(data[0][0] * v.data[0] + data[0][1] * v.data[1] + data[0][2] * v.data[2],
			      data[1][0] * v.data[0] + data[1][1] * v.data[1] + data[1][2] * v.data[2],
			      data[2][0] * v.data[0] + data[2][1] * v.data[1] + data[2][2] * v.data[2]);
		return res;
	}

	/**
	 * create a rotation matrix from given euler angles
	 * based on http://gentlenav.googlecode.com/files/EulerAngles.pdf
	 */
	void from_euler(float roll, float pitch, float yaw) {
		float cp = cosf(pitch);
		float sp = sinf(pitch);
		float sr = sinf(roll);
		float cr = cosf(roll);
		float sy = sinf(yaw);
		float cy = cosf(yaw);

		data[0][0] = cp * cy;
		data[0][1] = (sr * sp * cy) - (cr * sy);
		data[0][2] = (cr * sp * cy) + (sr * sy);
		data[1][0] = cp * sy;
		data[1][1] = (sr * sp * sy) + (cr * cy);
		data[1][2] = (cr * sp * sy) - (sr * cy);
		data[2][0] = -sp;
		data[2][1] = sr * cp;
		data[2][2] = cr * cp;
	}

	/**
	 * get euler angles from rotation matrix
	 */
	vector<3> to_euler(void) const {
		vector<3> euler;
		euler.data[1] = asinf(-data[2][0]);

		if (fabsf(euler.data[1] - M_PI_2_F) < 1.0e-3f) {
			euler.data[0] = 0.0f;
			euler.data[2] = atan2f(data[1][2] - data[0][1], data[0][2] + data[1][1]) + euler.data[0];

		} else if (fabsf(euler.data[1] + M_PI_2_F) < 1.0e-3f) {
			euler.data[0] = 0.0f;
			euler.data[2] = atan2f(data[1][2] - data[0][1], data[0][2] + data[1][1]) - euler.data[0];

		} else {
			euler.data[0] = atan2f(data[2][1], data[2][2]);
			euler.data[2] = atan2f(data[1][0], data[0][0]);
		}

		return euler;
	}
};

}

