#pragma once

//#include <assert.h>
//#include <time.h>
//#include <stdlib.h>

bool equal(float a, float b, float eps = 1e-5);
void float2SigExp(
	const float &num,
	float &sig,
	int &exp);
