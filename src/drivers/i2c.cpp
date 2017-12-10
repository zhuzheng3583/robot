/*******************************Copyright (c)***************************
**
** Porject name:	robot
** Created by:		
** Created date:	
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/
#include "i2c.h"

#define I2C_POLLING_TIMEOUT_MS 		1000

namespace driver {

struct stm32_i2c_hw_table
{
	GPIO_TypeDef  		*GPIOx;
	IRQn_Type 			IRQn;
	I2C_HandleTypeDef	I2C_Handle;
	GPIO_InitTypeDef  	GPIO_Init;
};

static struct stm32_i2c_hw_table i2c_hw_table[] = {
	[0] = { NULL },
    [1] = {
		.GPIOx = GPIOB,
		.IRQn = (IRQn_Type)NULL,
		.I2C_Handle = {
			.Instance = I2C1,
			.Init = {
                .ClockSpeed = 100000,
				.OwnAddress1 =  0xff,//_slave_addr
				.AddressingMode = I2C_ADDRESSINGMODE_7BIT,
				.DualAddressMode = I2C_DUALADDRESS_DISABLE,
				.OwnAddress2 = 0,
				.GeneralCallMode = I2C_GENERALCALL_DISABLE,
				.NoStretchMode = I2C_NOSTRETCH_DISABLE,
			},
		},
		.GPIO_Init = {
			.Pin = (GPIO_PIN_6 | GPIO_PIN_9),
			.Mode = GPIO_MODE_AF_PP, //GPIO_MODE_AF_OD; //
			.Pull = GPIO_PULLDOWN, //GPIO_PULLUP; //;	//
			.Speed = GPIO_SPEED_FREQ_HIGH,
			.Alternate = GPIO_AF4_I2C1,
		},
	},
    [2] = {
        .GPIOx = GPIOB,
        .IRQn = (IRQn_Type)NULL,
        .I2C_Handle = {
            .Instance = I2C2,
            .Init = {
                .ClockSpeed = 400000,
                .OwnAddress1 =  0xff,//_slave_addr
                .AddressingMode = I2C_ADDRESSINGMODE_7BIT,
                .DualAddressMode = I2C_DUALADDRESS_DISABLE,
                .OwnAddress2 = 0,
                .GeneralCallMode = I2C_GENERALCALL_DISABLE,
                .NoStretchMode = I2C_NOSTRETCH_DISABLE,
            },
        },
        .GPIO_Init = {
            .Pin = (GPIO_PIN_10 | GPIO_PIN_11),
            .Mode = GPIO_MODE_AF_OD, //GPIO_MODE_AF_PP, //
            .Pull = GPIO_PULLUP, //GPIO_PULLDOWN, //
            .Speed = GPIO_SPEED_FREQ_HIGH,
            .Alternate = GPIO_AF4_I2C2,
        },
    },

};

i2c::i2c(PCSTR name, s32 id)
{
	_name = name;
	_id = id;
}

i2c::~i2c(void)
{

}

s32 i2c::probe(void)
{
	I2C_HandleTypeDef *hi2c = &i2c_hw_table[_id].I2C_Handle;
	if(HAL_I2C_GetState(hi2c) != HAL_I2C_STATE_RESET)
	{
		ERR("%s: failed HAL_I2C_GetState.\n", _name);
		goto fail0;
	}

  	/*Configure the I2C peripheral*/
	//HAL_I2C_MspDeInit
	switch(_id)
    	{
	case 1:
#if 0
		RCC_PeriphCLKInitTypeDef  RCC_PeriphCLKInitStruct;
		/*##-1- Configure the I2C clock source. The clock is derived from the SYSCLK #*/
	  	RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  		RCC_PeriphCLKInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
		HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);
#endif
		/* Enable SCK SDA and GPIO clocks */
	  	__HAL_RCC_GPIOB_CLK_ENABLE();
	  	/* Enable the I2C clock */
	  	__HAL_RCC_I2C1_CLK_ENABLE();
		break;
	case 2:
        /* Enable SCK SDA and GPIO clocks */
	  	__HAL_RCC_GPIOB_CLK_ENABLE();
	  	/* Enable the I2C clock */
	  	__HAL_RCC_I2C2_CLK_ENABLE();
		break;
	default:
		break;
    	}
	/* I2Cx SDA/SCL pin configuration */
	GPIO_TypeDef  *GPIOx = i2c_hw_table[_id].GPIOx;
	GPIO_InitTypeDef *GPIO_Init= &i2c_hw_table[_id].GPIO_Init;
	HAL_GPIO_Init(GPIOx, GPIO_Init);

	/* Init the I2C */
	hi2c = &i2c_hw_table[_id].I2C_Handle;
	if(HAL_I2C_Init(hi2c) != HAL_OK) {
		ERR("%s: failed to HAL_I2C_Init.\n", _name);
		goto fail1;
	}
	_handle = (u32)hi2c;

	/* Enable the Analog I2C Filter */
	//HAL_I2CEx_ConfigAnalogFilter(hi2c, I2C_ANALOGFILTER_ENABLE);

	INF("%s: probe success.\n", _name);
	return 0;

fail1:
	HAL_GPIO_DeInit(GPIOx, GPIO_Init->Pin);
fail0:
	return -1;
}

s32 i2c::remove(void)
{
	I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)_handle;

	if(HAL_I2C_DeInit((I2C_HandleTypeDef*)_handle) != HAL_OK) {
    		goto fail0;
	}

	//void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c)
    switch(_id)
    	{
	case 1:
		/*1- Reset peripherals */
	  	__HAL_RCC_I2C1_FORCE_RESET();
	  	__HAL_RCC_I2C1_RELEASE_RESET();
		break;
	case 2:
	  	__HAL_RCC_I2C2_FORCE_RESET();
	  	__HAL_RCC_I2C2_RELEASE_RESET();
		break;
	default:
		break;
    	}
	/* 2- Disable peripherals and GPIO Clocks */
	GPIO_TypeDef  *GPIOx = i2c_hw_table[_id].GPIOx;
	uint32_t GPIO_Pin = i2c_hw_table[_id].GPIO_Init.Pin;
	HAL_GPIO_DeInit(GPIOx, GPIO_Pin);

	_handle = NULL;
	return 0;

fail0:
    return -1;
}

s32 i2c::transfer(struct i2c_msg *msgs, int num)
{
	HAL_StatusTypeDef status = HAL_OK;
	I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)_handle;

	u16 addr, flags, len, addr_len;
	u8 *buf;
	u8 reg;

	if (num > 2) {
		WRN("%s: more than 2 i2c messages at a time is not handled yet. TODO.\n", _name);
    }

	if (num == 2) {
		/* start addr reg | start addr data ... stop */
		addr_len = (msgs[0].len == 1 ? I2C_MEMADD_SIZE_8BIT : I2C_MEMADD_SIZE_16BIT);
		reg = msgs[0].buf[0];

		addr = msgs[1].addr;
		len = msgs[1].len;
		buf = (u8 *)(msgs[1].buf);

		if (msgs[1].flags & I2C_M_RD) {
			status = HAL_I2C_Mem_Read(hi2c, addr, (u16)reg, addr_len, buf, len, I2C_POLLING_TIMEOUT_MS);
		} else {
			status = HAL_I2C_Mem_Write(hi2c, addr, (u16)reg, addr_len, buf, len, I2C_POLLING_TIMEOUT_MS);
		}

		if(status != HAL_OK) {
			ERR("%s: failed to i2c transfer£¬num = %d!\n", _name, num);
			return -1;
		}
    } else {
		/* start addr data ... stop */
		addr = msgs[0].addr;
		len = msgs[0].len;
		buf = (u8 *)(msgs[0].buf);

        if (msgs[0].flags & I2C_M_RD) {
			status = HAL_I2C_Master_Receive(hi2c, addr, buf, len, I2C_POLLING_TIMEOUT_MS);
		} else {
			status = HAL_I2C_Master_Transmit(hi2c, addr, buf, len, I2C_POLLING_TIMEOUT_MS);
		}

		if(status != HAL_OK) {
			ERR("%s: failed to i2c transfer£¬num = %d!\n", _name, num);
			return -1;
		}
	}

	return 0;
}

s32 i2c::self_test(void)
{
#define SCCB_ID   			0X42  			//OV7670µÄID
	u8 data = 0x80;
	i2c::write_reg(SCCB_ID, 0x12, &data, 1);
	core::mdelay(50);  
	u8 temp = 0;
    i2c::read_reg(SCCB_ID, 0x0b, &temp, 1);
	if(temp!=0x73)
		return 2;  
    i2c::read_reg(SCCB_ID, 0x0a, &temp, 1);
	if(temp!=0x76)
		return 2;
	//3?¨º??¡¥D¨°¨¢D	  
	//for(i=0;i<sizeof(ov7670_init_reg_tbl)/sizeof(ov7670_init_reg_tbl[0]);i++)
	//{
	 //  	SCCB_WR_Reg(ov7670_init_reg_tbl[i][0],ov7670_init_reg_tbl[i][1]);
  	//}
   	return 0x00; 	//ok

	

	s32 ret = 0;
	s8 wbuf[16] = "hello world!";//{ 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0xAB };

	s8 rbuf[16] = { 0 };

 	u8 who_i = 0; //0x33
 	#define ACC_I2C_ADDRESS                      0x32
	#define LSM303DLHC_WHO_AM_I_ADDR             0x0F  /* Acceleration identification register */
	while (1) {
		who_i = 0;
		i2c::read_reg(ACC_I2C_ADDRESS, LSM303DLHC_WHO_AM_I_ADDR, &who_i, sizeof(who_i));

		INF("%s: who_i = 0x%02x.\n", _name, who_i);

		core::mdelay(500);
	}

	return ret;
}

void i2c::write_reg(u16 addr, u8 reg, u8 *buf, u32 len)
{
#if 0
  	HAL_StatusTypeDef status = HAL_OK;
  	status = HAL_I2C_Mem_Write((I2C_HandleTypeDef *)_handle, addr, (u16)reg, I2C_MEMADD_SIZE_8BIT, buf, len, 5000);
	if(status != HAL_OK) {
		ERR("%s: failed to i2c HAL_I2C_Mem_Write!\n", _name);
	}
#else
	struct i2c_msg msgs[] = {
		[0] = {
			.addr = addr,
			.flags = 0,
			.len = 1,
			.buf = &reg,
		},
		[1] = {
			.addr = addr,
			.flags = 0,
			.len = len,
			.buf = buf,
		},
	};
	if (i2c::transfer(msgs, 2) != 0) {
		ERR("%s: failed to i2c transfer!\n", _name);
	}
#endif
}

void i2c::read_reg(u16 addr, u8 reg, u8 *buf, u32 len)
{
#if 0
	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_I2C_Mem_Read((I2C_HandleTypeDef *)_handle, addr, (u16)reg, I2C_MEMADD_SIZE_8BIT, buf, len, 5000);
	if(status != HAL_OK) {
		ERR("%s: failed to i2c HAL_I2C_Mem_Read!\n", _name);
	}
#else
	struct i2c_msg msgs[] = {
		[0] = {
			.addr = addr,
			.flags = 0,
			.len = 1,
			.buf = &reg,
		},
		[1] = {
			.addr = addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = buf,
		},
	};
	if (i2c::transfer(msgs, 2) != 0) {
		ERR("%s: failed to i2c transfer!\n", _name);
	}
#endif
}

}
/***********************************************************************
** End of file
***********************************************************************/

