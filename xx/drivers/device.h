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
#pragma once
#include "leader_type.h"
#include "leader_misc.h"

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_conf.h"

#include "cmsis_os.h"
#include "errno.h"
#include "driver_type.h"

#include "kernel.h"
#include "semaphore.h"


using namespace os;

#define DEV_ERR(fmt, ...) print(3, ("[DEV_ERR] --%s--(%d)-<%s>: %s: " fmt), \
    __FILE__, __LINE__, __FUNCTION__, _devname, ##__VA_ARGS__)
#define DEV_WRN(fmt, ...) print(2, ("[DEV_WRN]<%s>: %s: " fmt), __FUNCTION__, _devname, ##__VA_ARGS__)
#define DEV_INF(fmt, ...) print(1, ("[DEV_INF]<%s>: %s: " fmt), __FUNCTION__, _devname, ##__VA_ARGS__)
#define DEV_DBG(fmt, ...) print(0, ("[DEV_DBG]<%s>: %s: " fmt), __FUNCTION__, _devname, ##__VA_ARGS__)

#define NO_PARAMS   0

typedef struct file {
	struct file *next;
	struct file *parent;
	char *name;
	int lineno;
	int flags;
} file_t;


/* Poll event definitions:
 *
 *   POLLIN
 *     Data other than high-priority data may be read without blocking.
 *   POLLRDNORM
 *     Normal data may be read without blocking.
 *   POLLRDBAND
 *     Priority data may be read without blocking.
 *   POLLPRI
 *     High priority data may be read without blocking.
 *
 *   POLLOUT
 *     Normal data may be written without blocking.
 *   POLLWRNORM
 *     Equivalent to POLLOUT.
 *   POLLWRBAND
 *     Priority data may be written.
 *
 *   POLLERR
 *     An error has occurred (revents only).
 *   POLLHUP
 *     Device has been disconnected (revents only).
 *   POLLNVAL
 *     Invalid fd member (revents only).
 */

#define POLLIN       (0x01) 
#define POLLRDNORM   (0x01)
#define POLLRDBAND   (0x01)
#define POLLPRI      (0x01)

#define POLLOUT      (0x02) 
#define POLLWRNORM   (0x02)
#define POLLWRBAND   (0x02)

#define POLLERR      (0x04)
#define POLLHUP      (0x08)
#define POLLNVAL     (0x10)


/* This is the Nuttx variant of the standard pollfd structure. */

struct pollfd
{
  //int         fd;       /* The descriptor being polled */
  semaphore *sem;      /* Pointer to semaphore used to post output event */
  pollevent_t events;   /* The input event flags */
  pollevent_t revents;  /* The output event flags */
  //FAR void   *priv;     /* For use by drivers */
};

namespace driver {

#define SEEK_SET_M      1               // SEEK_SET
#define SEEK_CUR_M      2                // SEEK_CUR
#define SEEK_END_M      3                // SEEK_END

#define DIR_READ        0x0000B000
#define DIR_WRITE       0x0000B001

#define WAIT_FOREVER    ~(0)
#define DO_NOT_WAIT     0


class device
{
public:
    device(PCSTR devname, s32 devid);
    ~device(void);

public:
    PCSTR _name;
    s32 _id;
    s32 _irq;
    u32 _handle;

	PCSTR _devname;
	s32 _devid;
	u32 _devhandle;

public:
    s32 probe(void);
    s32 is_probed(void);
    s32 remove(void);

public:
    static s32 irq_init(void);
    static s32 request_irq(s32 irq, device *owner);
    static void free_irq(s32 irq);
    static void enable_irq(s32 irq);
    static void disable_irq(s32 irq);
    static void enable_all_irq(void);
    static void disable_all_irq(void);
    virtual void isr(void);

public:
    semaphore *_lock;
	void    lock(void) { do {} while (_lock->pend(OS_WAIT_FOREVER) != true); }
	void    unlock(void) { _lock->post(OS_WAIT_FOREVER); }
    
    virtual s32 open(void);
    virtual s32 close(void);
    virtual s32 read(u8 *buf, u32 size);
    virtual s32 write(u8 *buf, u32 size);
    virtual off_t seek(off_t offset, s32 whence);
    virtual s32 ioctl(s32 cmd, u64 arg);
    virtual s32 poll(struct pollfd *fds, s32 timeoutms);
    virtual s32 tell(void);
    virtual s32 flush(void);
    bool    is_open(void) { return _open_count > 0; }


public:
    virtual void poll_notify(pollevent_t events);
    virtual s32	open_first(void);
    virtual s32	close_last(void);

    bool	_pub_blocked;		/**< true if publishing should be blocked */

private:
    bool _registered;		/**< true if device name was registered */
    u32 _open_count;		/**< number of successful opens */
    s32 _probed;
    struct pollfd _pollset;


};

}
/***********************************************************************
** End of file
***********************************************************************/
