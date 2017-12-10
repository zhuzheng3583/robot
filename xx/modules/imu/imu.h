#pragma once
#include "leader_type.h"
#include "leader_misc.h"
#include "vehicle_attitude.h"

#ifdef __cplusplus
extern "C" {
#endif

void imu_update(float gx, float gy, float gz, float ax, float ay, float az, struct vehicle_attitude_s *att);

#ifdef __cplusplus
}
#endif

