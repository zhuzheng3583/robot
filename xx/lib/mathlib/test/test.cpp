#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include "test.h"

bool equal(float a, float b, float epsilon)
{
	float diff = fabsf(a - b);

	if (diff > epsilon) {
		printf("not equal ->\n\ta: %12.8f\n\tb: %12.8f\n", double(a), double(b));
		return false;

	} else return true;
}

void float2SigExp(
	const float &num,
	float &sig,
	int &exp)
{
	if (isnan(num) || isinf(num)) {
		sig = 0.0f;
		exp = -99;
		return;
	}

	if (fabsf(num) < 1.0e-38f) {
		sig = 0;
		exp = 0;
		return;
	}

	exp = log10f(fabsf(num));

	if (exp > 0) {
		//exp = ceil(exp);

	} else {
		//exp = floor(exp);
	}

	sig = num;

	// cheap power since it is integer
	if (exp > 0) {
		for (int i = 0; i < abs(exp); i++) sig /= 10;

	} else {
		for (int i = 0; i < abs(exp); i++) sig *= 10;
	}
}


