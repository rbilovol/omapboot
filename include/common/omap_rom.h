/*
 * Copyright (C) 2010 The Android Open Source Project
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#ifndef _ROM_ROM_H_
#define _ROM_ROM_H_

extern u32 public_rom_base;

/* public api */
#define PUBLIC_API_BASE_4430		(0x28400)
#define PUBLIC_API_BASE_4460		(0x30400)

#define PUBLIC_GET_DRIVER_MEM_OFFSET (0x04)
#define PUBLIC_GET_DRIVER_PER_OFFSET (0x08)
#define PUBLIC_GET_DEVICE_MEM_OFFSET (0x80)
#define PUBLIC_GET_DEVICE_PER_OFFSET (0x84)

#define PUBLIC_IRQ_INITIALIZE				(0x40)

#define PUBLIC_HAL_I2C_DRIVER_INIT_OFFSET		(0x60)
#define PUBLIC_HAL_I2C_DRIVER_WRITE_OFFSET		(0x64)
#define PUBLIC_HAL_I2C_DRIVER_READ_OFFSET		(0x68)
#define PUBLIC_HAL_I2C_DRIVER_CLOSE_OFFSET		(0x6C)

#define PUBLIC_HAL_CM_ENABLEMODULECLOCKS_OFFSET		(0xA0)
#define PUBLIC_HAL_CM_DISABLEMODULECLOCKS_OFFSET	(0xA4)
#define PUBLIC_HAL_CTRL_CONFIGUREPADS_OFFSET		(0xA8)

#define DEVICE_NULL	0x40
#define DEVICE_UART1	0x41
#define DEVICE_UART2	0x42
#define DEVICE_UART3	0x43
#define DEVICE_UART4	0x44
#define DEVICE_USB	0x45
#define DEVICE_USBEXT	0x46

#define XFER_MODE_CPU 0
#define XFER_MODE_DMA 1

#define STATUS_OKAY		0
#define STATUS_FAILED		1
#define STATUS_TIMEOUT		2
#define STATUS_BAD_PARAM	3
#define STATUS_WAITING		4
#define STATUS_NO_MEMORY	5
#define STATUS_INVALID_PTR	6

typedef enum {
	HAL_MODULE_NULL = (0x00),
	HAL_MODULE_MMC,
	HAL_MODULE_UART,
	HAL_MODULE_USB,
	HAL_MODULE_I2C,
	HAL_MODULE_DMTIMER1MS,
	HAL_MODULE_WDTIMER,
	HAL_MODULE_ELM,
	HAL_MODULE_GPMC,
	HAL_MODULE_EMIF

} hal_module;

/* Memory ROM interface */
struct read_desc {
	u32 sector_start;
	u32 sector_count;
	void *destination;
};

struct mem_device {
	u32 initialized;
	u8 device_type;
	u8 trials_count;
	u32 xip_device;
	u16 search_size;
	u32 base_address;
	u16 hs_toc_mask;
	u16 gp_toc_mask;
	void *device_data;
	u16 *boot_options;
};

struct mem_driver {
	int (*init)(struct mem_device *md);
	int (*read)(struct mem_device *md, struct read_desc *rd);
	int (*configure)(struct mem_device *md, void *config);
};


/* Peripheral ROM interface */
struct per_handle {
	void *set_to_null;
	void (*callback)(struct per_handle *rh);
	void *data;
	u32 length;
	u16 *options;
	u32 xfer_mode;
	u32 device_type;
	volatile u32 status;
	u16 hs_toc_mask;
	u16 gp_toc_mask;
	u32 config_timeout;
};

struct per_driver {
	int (*init)(struct per_handle *rh);
	int (*read)(struct per_handle *rh);
	int (*write)(struct per_handle *rh);
	int (*close)(struct per_handle *rh);
	int (*config)(struct per_handle *rh, void *x);
};

#define USB_SETCONFIGDESC_ATTRIBUTES      (0)
#define USB_SETCONFIGDESC_MAXPOWER        (1)
#define USB_SETSUSPEND_CALLBACK           (2)
struct per_usb_config {
	u32 configid;
	u32 value;
};

#define API(n) ((void *) (*((u32 *) (n))))
/* ROM API End */

struct usb {
	struct per_handle dread;
	struct per_handle dwrite;
	struct per_driver *io;
};

int usb_open(struct usb *usb);
void usb_close(struct usb *usb);

void usb_queue_read(struct usb *usb, void *data, unsigned len);
int usb_wait_read(struct usb *usb);

void usb_queue_write(struct usb *usb, void *data, unsigned len);
int usb_wait_write(struct usb *usb);

int usb_read(struct usb *usb, void *data, unsigned len);
int usb_write(struct usb *usb, void *data, unsigned len);

/* I2C */
typedef enum {
	HAL_I2C1 = (0x00),
	HAL_I2C2,
	HAL_I2C3,
	HAL_I2C4
} hal_i2c;

int i2c_init(hal_i2c i2c_id);

int i2c_write(hal_i2c i2c_id, u16 slave, u32 count, void *data, u32 start_time,
								u32 timeout);

int i2c_read(hal_i2c i2c_id, u16 slave, u32 count, void *data, u32 start_time,
								u32 timeout);

int i2c_close(hal_i2c i2c_id);

/* rom api prototypes ------------------------------------------------ begin */

/* PUBLIC_GET_DRIVER_MEM */
typedef int (** const sys_getdrivermem_pt)(struct mem_driver **, u32);
#define rom_get_mem_driver(a, b) \
	(*(sys_getdrivermem_pt) \
	(public_rom_base + PUBLIC_GET_DRIVER_MEM_OFFSET))(a, b);

/* PUBLIC_GET_DRIVER_PER */
typedef int (** const sys_getdriverper_pt)(struct per_driver **, u32);
#define rom_get_per_driver(a, b) \
	(*(sys_getdriverper_pt) \
	(public_rom_base + PUBLIC_GET_DRIVER_PER_OFFSET))(a, b);

/* PUBLIC_GET_DEVICE_MEM */
typedef int (** const sys_getdevicedescmem_pt)(struct per_handle **);
#define rom_get_mem_device(a) \
	(*(sys_getdevicedescmem_pt) \
	(public_rom_base + PUBLIC_GET_DEVICE_MEM_OFFSET))(a);

/* PUBLIC_GET_DEVICE_PER */
typedef int (** const sys_getdevicedescper_pt)(struct per_handle **);
#define rom_get_per_device(a) \
	(*(sys_getdevicedescper_pt) \
	(public_rom_base + PUBLIC_GET_DEVICE_PER_OFFSET))(a);

/* PUBLIC_HAL_CTRL_CONFIGUREPADS */
typedef int (** const hal_ctrl_configurepads_pt)(hal_module, u32);
#define rom_hal_ctrl_configurepads(a, b) \
	(*(hal_ctrl_configurepads_pt)\
	(public_rom_base + PUBLIC_HAL_CTRL_CONFIGUREPADS_OFFSET))(a, b);

/* PUBLIC_HAL_CM_ENABLEMODULECLOCKS */
typedef int (** const hal_cm_enablemoduleclocks_pt)(hal_module, u32);
#define rom_hal_cm_enablemoduleclocks(a, b) \
	(*(hal_cm_enablemoduleclocks_pt) \
	(public_rom_base + PUBLIC_HAL_CM_ENABLEMODULECLOCKS_OFFSET))(a, b);

/* PUBLIC_HAL_CM_DISABLEMODULECLOCKS */
typedef int (** const hal_cm_disablemoduleclocks_pt)(hal_module, u32);
#define rom_hal_cm_disablemoduleclocks(a, b) \
	(*(hal_cm_disablemoduleclocks_pt) \
	(public_rom_base + PUBLIC_HAL_CM_DISABLEMODULECLOCKS_OFFSET))(a, b);

/* PUBLIC_IRQ_INITIALIZE */
typedef int (** const irq_initialize_pt)(void);
#define rom_irq_initialize() \
	(*(irq_initialize_pt) \
	(public_rom_base + PUBLIC_IRQ_INITIALIZE))();

/* PUBLIC_HAL_I2C_DRIVER_INITIALIZE */
typedef int (** const hal_i2c_initialize_pt)(hal_i2c);
#define rom_hal_i2c_initialize(a) \
	(*(hal_i2c_initialize_pt) \
	(public_rom_base + PUBLIC_HAL_I2C_DRIVER_INIT_OFFSET))(a);

/* PUBLIC_HAL_I2C_DRIVER_WRITE */
typedef int (** const hal_i2c_write_pt)(hal_i2c, u16, u32, void*, u32, u32);
#define rom_hal_i2c_write(a, b, c, d, e, f) \
	(*(hal_i2c_write_pt) \
	(public_rom_base + PUBLIC_HAL_I2C_DRIVER_WRITE_OFFSET)) \
						(a, b, c, d, e, f);

/* PUBLIC_HAL_I2C_DRIVER_READ */
typedef int (** const hal_i2c_read_pt)(hal_i2c, u16, u32, void*, u32, u32);
#define rom_hal_i2c_read(a, b, c, d, e, f) \
		(*(hal_i2c_read_pt) \
		(public_rom_base + PUBLIC_HAL_I2C_DRIVER_READ_OFFSET)) \
							(a, b, c, d, e, f);

/* PUBLIC_HAL_I2C_DRIVER_CLOSE */
typedef int (** const hal_i2c_close_pt)(hal_i2c);
#define rom_hal_i2c_close(a) \
		(*(hal_i2c_close_pt) \
		(public_rom_base + PUBLIC_HAL_I2C_DRIVER_CLOSE_OFFSET))(a);

/* rom api prototypes -------------------------------------------------- end */

#endif
