/*******************************Copyright (c)***************************
**
** Porject name:	leader
** Created by:		zhuzheng<happyzhull@163.com>
** Created date:	2016/04/05
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/
#include "device.h"

namespace driver {

device::device(PCSTR devname, s32 devid) :
	_name(devname),
    _id(devid),
    _handle(NULL),
    _devname(devname),
    _devid(devid),
    _devhandle(NULL),
    _irq(-1),
    _probed(0)
{
	_lock = new semaphore;
	_lock->create("device_sem", 1);

	_pollset.sem = new semaphore;
	_pollset.sem->create("poll_sem", 1);
	_pollset.events = 0;
	_pollset.revents = 0;
}

device::~device(void)
{
	_lock->s_delete();
	delete _lock;

	_pollset.sem->s_delete();
	delete _pollset.sem;
}

s32 device::probe(void)
{
	if (_probed != 0) {
		ERR("%s: Has been probed.\n", _devname);
		return -1;
	}

	_probed = 1;
	return 0;
}

s32 device::is_probed(void)
{
	if (_probed != 0) {
		return 0;
	}

	ERR("%s: Has no been probed.\n", _devname);
	return -1;
}

s32 device::remove(void)
{
	if (_probed == 0) {
		ERR("%s: Has been removed.\n", _devname);
		return -1;
	}

	_probed = 0;
	_handle = NULL;

	return 0;
}

/*
 * Default implementations of the character device interface
 */
s32 device::open(void)
{
	DEV_DBG("device::open.\n");
	int ret = LD_OK;

	lock();
	/* increment the open count */
	_open_count++;

	if (_open_count == 1) {

		/* first-open callback may decline the open */
		ret = open_first();

		if (ret != LD_OK) {
			_open_count--;
		}
	}

	unlock();

	return ret;
}

s32 device::open_first(void)
{
	DEV_DBG("device::open_first.\n");
	return LD_OK;
}

int device::close(void)
{
	DEV_DBG("device::close.\n");
	int ret = LD_OK;

	lock();

	if (_open_count > 0) {
		/* decrement the open count */
		_open_count--;

		/* callback cannot decline the close */
		if (_open_count == 0) {
			ret = close_last();
		}

	} else {
		ret = -EBADF;
	}

	unlock();

	return ret;
}

int device::close_last(void)
{
	DEV_DBG("device::close_last.\n");
	return LD_OK;
}

s32 device::read(u8 *buf, u32 size)
{
	DEV_DBG("device::read.\n");
	return -ENOSYS;
}

s32 device::write(u8 *buf, u32 size)
{
	DEV_DBG("device::write.\n");
	return -ENOSYS;
}

off_t device::seek(off_t offset, s32 whence)
{
	DEV_DBG("device::seek.\n");
	return -ENOSYS;
}

s32 device::ioctl(s32 cmd, u64 arg)
{
	DEV_DBG("device::ioctl.\n");
	return -ENOSYS;
}

s32 device::tell(void)
{
	DEV_DBG("device::tell.\n");
	return -ENOSYS;
}

s32 device::flush(void)
{
	DEV_DBG("device::flush.\n");
	return -ENOSYS;
}

s32 device::poll(struct pollfd *fds, s32 timeoutms)
{
	s32 ret = LD_OK;
	
    _pollset.events = fds->events;
	_pollset.sem->pend(timeoutms);
	
	lock();
	if (fds->events & _pollset.revents) {
		fds->revents & _pollset.revents;
		ret = LD_OK;
	} else {
		ret = LD_ERROR;
	}
	unlock();
			
	return ret;
}

void device::poll_notify(pollevent_t events)
{
	//DEV_DBG("device::poll_notify events = %0x", events);

	/* lock against poll() as well as other wakeups */
	lock();

	s32 value = -1;
	//px4_sem_getvalue(_pollset->sem, &value);

	/* update the reported event set */
	_pollset.revents |= _pollset.events & events;
	//DEV_DBG(" Events fds=%p %0x %0x %0x %d", _pollset, _pollset.revents, _pollset.events, events, value);

	unlock();
	
	/* if the state is now interesting, wake the waiter if it's still asleep */
	/* XXX semcount check here is a vile hack; counting semphores should not be abused as cvars */
	if ((_pollset.revents != 0) && (value <= 0)) {
		_pollset.sem->post(OS_WAIT_FOREVER);
	}	
}


/**
 * Interrupt dispatch table entry.
 */
struct irq_entry {
	s32	    irq;
	device  *owner;
};

//id = [0,81]
#define STM32F4xx_USER_IRQNUM_MAX	    82
static irq_entry irq_entries[STM32F4xx_USER_IRQNUM_MAX];	/**< interrupt dispatch table (XXX should be a vector) */

/*
 * 初始化中断
 * @return	操作成功返回 0
 */
s32 device::irq_init(void)
{
	/*
	 * 分配抢占优先级和响应优先级位域:
	 * NVIC_PRIORITYGROUP_4: 4位抢占优先级，0位响应优先级
	 */
	/* Set Interrupt Group Priority */
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

	return 0;
}

/*
 * 中断资源请求
 * @param[in]	irq		设备中断逻辑编号
 * @param[in]	handler	设备中断服务程序基类
 * @return	操作成功返回 0
 * @note		填充向量表，绑定中断逻辑编号与中断服务程序
 */
s32 device::request_irq(s32 irq, device *owner)
{
	if (irq >= ARRAYSIZE(irq_entries) || owner == NULL) {
		return -1;
	}

	irq_entries[irq].irq = irq;
	irq_entries[irq].owner = owner;

	HAL_NVIC_SetPriority((IRQn_Type)(irq_entries[irq].irq), configLIBRARY_LOWEST_INTERRUPT_PRIORITY, 0);

	return 0;
}

void device::free_irq(s32 irq)
{
	if (irq < ARRAYSIZE(irq_entries)) {
        irq_entries[irq].irq = -1;
        irq_entries[irq].owner = NULL;
	}
}

void device::enable_irq(s32 irq)
{
	HAL_NVIC_EnableIRQ((IRQn_Type)(irq_entries[irq].irq));
}

void device::disable_irq(s32 irq)
{
	HAL_NVIC_DisableIRQ((IRQn_Type)(irq_entries[irq].irq));
}

void device::enable_all_irq(void)
{

}

void device::disable_all_irq(void)
{

}

void device::isr(void)
{

}

/*
 * 中断向量一级分发表
 * @note TODO:在此添加一级中断向量，二级中断向量在irq_handler的派生类中实现
 */
#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
#if defined(USE_HAL_DRIVER)
    HAL_IncTick();
#if (USE_STM32F4_DEMO)
    /* Call user callback */
    HAL_SYSTICK_IRQHandler();
#endif
#endif
    osSystickHandler();
}

void USART1_IRQHandler(void)
{
	irq_entries[USART1_IRQn].owner->isr();
}
void DMA2_Stream5_IRQHandler(void)
{
	irq_entries[DMA2_Stream5_IRQn].owner->isr();
}
void DMA2_Stream7_IRQHandler(void)
{
	irq_entries[DMA2_Stream7_IRQn].owner->isr();
}

void USART2_IRQHandler(void)
{
	irq_entries[USART2_IRQn].owner->isr();
}
void DMA1_Stream5_IRQHandler(void)
{
	irq_entries[DMA1_Stream5_IRQn].owner->isr();
}
void DMA1_Stream6_IRQHandler(void)
{
	irq_entries[DMA1_Stream6_IRQn].owner->isr();
}

void USART3_IRQHandler(void)
{
	irq_entries[USART3_IRQn].owner->isr();
}
void DMA1_Stream1_IRQHandler(void)
{
	irq_entries[DMA1_Stream1_IRQn].owner->isr();
}
void DMA1_Stream3_IRQHandler(void)
{
	irq_entries[DMA1_Stream3_IRQn].owner->isr();
}

void DMA2_Stream1_IRQHandler(void)
{
	irq_entries[DMA2_Stream1_IRQn].owner->isr();
}
void USART6_IRQHandler(void)
{
	irq_entries[USART6_IRQn].owner->isr();
}


void DMA2_Stream2_IRQHandler(void)
{
	irq_entries[DMA2_Stream2_IRQn].owner->isr();
}
void DMA2_Stream3_IRQHandler(void)
{
	irq_entries[DMA2_Stream3_IRQn].owner->isr();
}


#if 0
//I2C1_DMA_TX
void DMA1_Channel6_IRQHandler(void)
{
	irq_entries[DMA1_Channel6_IRQn].owner->isr();
}
//I2C1_DMA_RX
void DMA1_Channel7_IRQHandler(void)
{
	irq_entries[DMA1_Channel7_IRQn].owner->isr();
}


//EXTI
void EXTI0_IRQHandler(void)
{
	irq_entries[EXTI0_IRQn].owner->isr();
}
void EXTI1_IRQHandler(void)
{
	irq_entries[EXTI1_IRQn].owner->isr();
}
void EXTI2_IRQHandler(void)
{
	irq_entries[EXTI2_TSC_IRQn].owner->isr();
}
void EXTI3_IRQHandler(void)
{
	irq_entries[EXTI3_IRQn].owner->isr();
}
void EXTI4_IRQHandler(void)
{
	irq_entries[EXTI4_IRQn].owner->isr();
}

#endif

void TIM2_IRQHandler(void)
{
	irq_entries[TIM2_IRQn].owner->isr();
}
void TIM3_IRQHandler(void)
{
	irq_entries[TIM3_IRQn].owner->isr();
}

#ifdef __cplusplus
}
#endif


}

/***********************************************************************
** End of file
***********************************************************************/
