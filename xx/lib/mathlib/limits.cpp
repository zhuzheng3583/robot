#include <math.h>
#include <stdint.h>

#include "limits.h"
#include "mathtype.h"

namespace math {

float min(float val1, float val2)
{
	return (val1 < val2) ? val1 : val2;
}

int min(int val1, int val2)
{
	return (val1 < val2) ? val1 : val2;
}

unsigned min(unsigned val1, unsigned val2)
{
	return (val1 < val2) ? val1 : val2;
}

uint64_t min(uint64_t val1, uint64_t val2)
{
	return (val1 < val2) ? val1 : val2;
}

double min(double val1, double val2)
{
	return (val1 < val2) ? val1 : val2;
}

float max(float val1, float val2)
{
	return (val1 > val2) ? val1 : val2;
}

int max(int val1, int val2)
{
	return (val1 > val2) ? val1 : val2;
}

unsigned max(unsigned val1, unsigned val2)
{
	return (val1 > val2) ? val1 : val2;
}

uint64_t max(uint64_t val1, uint64_t val2)
{
	return (val1 > val2) ? val1 : val2;
}

double max(double val1, double val2)
{
	return (val1 > val2) ? val1 : val2;
}


float constrain(float val, float min, float max)
{
	return (val < min) ? min : ((val > max) ? max : val);
}

int constrain(int val, int min, int max)
{
	return (val < min) ? min : ((val > max) ? max : val);
}

unsigned constrain(unsigned val, unsigned min, unsigned max)
{
	return (val < min) ? min : ((val > max) ? max : val);
}

uint64_t constrain(uint64_t val, uint64_t min, uint64_t max)
{
	return (val < min) ? min : ((val > max) ? max : val);
}

double constrain(double val, double min, double max)
{
	return (val < min) ? min : ((val > max) ? max : val);
}

float radians(float degrees)
{
	return (degrees / 180.0f) * M_PI_F;
}

double radians(double degrees)
{
	return (degrees / 180.0) * M_PI;
}

float degrees(float radians)
{
	return (radians / M_PI_F) * 180.0f;
}

double degrees(double radians)
{
	return (radians / M_PI) * 180.0;
}

}
