/*******************************Copyright (c)***************************
** 
** Porject name:	leader
** Created by:	zhuzheng<happyzhull@163.com>
** Created date:	2016/06/12
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/
#pragma once
#include "leader_type.h"
#include "leader_misc.h"

#include "device.h"

typedef struct {    
    union { 
        float v[3];        
        struct {            
            float x;            
            float y;            
            float z;        
        };        
        struct {            
            float azimuth;            
            float pitch;            
            float roll;        
         };    
    };    
    char status;
    unsigned char reserved[3];
} sensorhub_vec_t;

typedef struct sensorhub_event{
    /* acceleration values are in meter per second per second (m/s^2) */
    sensorhub_vec_t   acceleration;
    /* magnetic vector values are in micro-Tesla (uT) */
    sensorhub_vec_t   magnetic;
    /* orientation values are in degrees */
    sensorhub_vec_t   orientation;
    /* gyroscope values are in rad/s */
    sensorhub_vec_t   gyro;
    /* distance in centimeters */
    float           distance;
    /* light in SI lux units */
    float           light;
} sensorhub_event_t;



/***********************************************************************
** End of file
***********************************************************************/
