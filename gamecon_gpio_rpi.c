/*
 * Modified NES, SNES, N64, PSX, Gamecube gamepad driver for Raspberry Pi
 *
 *  Revised GPIO's used for NES/SNES controllers:
 *  CLOCK = 9,10
 *  LATCH = 11
 *  CONTROLLER3 (i=2) = 8
 *
 *  Copyright (c) 2014	Davis Walsh
 *
 */
/*
 * NES, SNES, N64, PSX, Gamecube gamepad driver for Raspberry Pi
 *
 *  Copyright (c) 2012	Markus Hiienkari
 *
 *  Based on the gamecon driver by Vojtech Pavlik
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/mutex.h>
#include <linux/slab.h>

#include <linux/ioport.h>
#include <asm/io.h>

MODULE_AUTHOR("Davis Walsh");
MODULE_DESCRIPTION("Modified NES, SNES gamepad driver");
MODULE_LICENSE("GPL");

#define GC_MAX_DEVICES		2

#define BCM2708_PERI_BASE        0x20000000
#define GPIO_BASE                (BCM2708_PERI_BASE + 0x200000) /* GPIO controller */

//GPIO SET register mem location
#define GPIO_SET *(gpio+7)

//GPIO CLEAR register mem location
#define GPIO_CLR *(gpio+10)

//This will be a pointer to the mem location of the GPIO base register, each register is 32-bits
static volatile unsigned *gpio;

struct gc_config {
	int args[GC_MAX_DEVICES];
	unsigned int nargs;
};

static struct gc_config gc_cfg __initdata;

module_param_array_named(map, gc_cfg.args, int, &(gc_cfg.nargs), 0);
MODULE_PARM_DESC(map, "Describes the set of pad connections (<GPIO7>,<GPIO8>)");

/* see also gs_psx_delay parameter in PSX support section */

enum gc_type {
	GC_NONE = 0,
	GC_SNES,
	GC_NES,
	GC_SNESMOUSE,
	GC_MAX
};

#define GC_REFRESH_TIME	HZ/100

struct gc_pad {
	struct input_dev *dev;
	enum gc_type type;
	char phys[32];
};

struct gc_nin_gpio {
	unsigned pad_id;
	unsigned cmd_setinputs;
	unsigned cmd_setoutputs;
	unsigned valid_bits;
	unsigned request;
	unsigned request_len;
	unsigned response_len;
	unsigned response_bufsize;
};

struct gc {
	struct gc_pad pads[GC_MAX_DEVICES];
	struct timer_list timer;
	int pad_count[GC_MAX];
	int used;
	struct mutex mutex;
};

struct gc_subdev {
	unsigned int idx;
};

static struct gc *gc_base;

/* GPIO pins 0, 1, 8, 7, 2, 3 */
// 0 : (0x01) : 0000 0001
// 1 : (0x02) : 0000 0010
// 4 : (0x10) : 0001 0000
// 7 : (0x80) : 1000 0000
// 2 : (0x04) : 0000 0100
// 3 : (0x08) : 0000 1000
//Status bit = 2^x, where x = GPIO port, not sure why yet...
static const int gc_status_bit[] = { 0x80, 0x100 };
static const int gc_gpio_ids[] = { 7, 8 };

static const char *gc_names[] = {
	NULL, "SNES pad", "NES pad", "SNES mouse"
};

/*
 * NES/SNES support.
 */

#define GC_NES_DELAY		6	/* Delay between bits - 6us */
#define GC_NES_LENGTH		8	/* The NES pads use 8 bits of data */
#define GC_SNES_LENGTH		12	/* The SNES true length is 16, but the
					   last 4 bits are unused */
#define GC_SNESMOUSE_LENGTH	32	/* The SNES mouse uses 32 bits, the first
					   16 bits are equivalent to a gamepad */

/* clock = gpio9 & gpio10, latch = gpio11 */
//0110 0000 0000
#define GC_NES_CLOCK	0x600
//1000 0000 0000
#define GC_NES_LATCH	0x800

static const unsigned char gc_nes_bytes[] = { 0, 1, 2, 3 };
static const unsigned char gc_snes_bytes[] = { 8, 0, 2, 3, 9, 1, 10, 11 };
static const short gc_snes_btn[] = {
	BTN_A, BTN_B, BTN_SELECT, BTN_START, BTN_X, BTN_Y, BTN_TL, BTN_TR
};

/*
 * gc_nes_read_packet() reads a NES/SNES packet.
 * Each pad uses one bit per byte. So all pads connected to
 * this port are read in parallel.
 */

static void gc_nes_read_packet(struct gc *gc, int length, unsigned short *data)
{
	int i;

	GPIO_SET = GC_NES_CLOCK | GC_NES_LATCH;
	udelay(GC_NES_DELAY * 2);
	GPIO_CLR = GC_NES_LATCH;

	for (i = 0; i < length; i++) {
		udelay(GC_NES_DELAY);
		GPIO_CLR = GC_NES_CLOCK;
		data[i] = ~(*(gpio+13));
		udelay(GC_NES_DELAY);
		GPIO_SET = GC_NES_CLOCK;
	}
}

static void gc_nes_process_packet(struct gc *gc)
{
	unsigned short data[GC_SNESMOUSE_LENGTH];
	struct gc_pad *pad;
	struct input_dev *dev;
	int i, j, s, len;
	char x_rel, y_rel;

	len = gc->pad_count[GC_SNESMOUSE] ? GC_SNESMOUSE_LENGTH :
			(gc->pad_count[GC_SNES] ? GC_SNES_LENGTH : GC_NES_LENGTH);

	gc_nes_read_packet(gc, len, data);

	for (i = 0; i < GC_MAX_DEVICES; i++) {

		pad = &gc->pads[i];
		dev = pad->dev;
		s = gc_status_bit[i];

		switch (pad->type) {

		case GC_NES:

			input_report_abs(dev, ABS_X, !(s & data[6]) - !(s & data[7]));
			input_report_abs(dev, ABS_Y, !(s & data[4]) - !(s & data[5]));

			for (j = 0; j < 4; j++)
				input_report_key(dev, gc_snes_btn[j],
						 s & data[gc_nes_bytes[j]]);
			input_sync(dev);
			break;

		case GC_SNES:

			input_report_abs(dev, ABS_X, !(s & data[6]) - !(s & data[7]));
			input_report_abs(dev, ABS_Y, !(s & data[4]) - !(s & data[5]));

			for (j = 0; j < 8; j++)
				input_report_key(dev, gc_snes_btn[j],
						 s & data[gc_snes_bytes[j]]);
			input_sync(dev);
			break;

		case GC_SNESMOUSE:
			/*
			 * The 4 unused bits from SNES controllers appear
			 * to be ID bits so use them to make sure we are
			 * dealing with a mouse.
			 * gamepad is connected. This is important since
			 * my SNES gamepad sends 1's for bits 16-31, which
			 * cause the mouse pointer to quickly move to the
			 * upper left corner of the screen.
			 */
			if (!(s & data[12]) && !(s & data[13]) &&
			    !(s & data[14]) && (s & data[15])) {
				input_report_key(dev, BTN_LEFT, s & data[9]);
				input_report_key(dev, BTN_RIGHT, s & data[8]);

				x_rel = y_rel = 0;
				for (j = 0; j < 7; j++) {
					x_rel <<= 1;
					if (data[25 + j] & s)
						x_rel |= 1;

					y_rel <<= 1;
					if (data[17 + j] & s)
						y_rel |= 1;
				}

				if (x_rel) {
					if (data[24] & s)
						x_rel = -x_rel;
					input_report_rel(dev, REL_X, x_rel);
				}

				if (y_rel) {
					if (data[16] & s)
						y_rel = -y_rel;
					input_report_rel(dev, REL_Y, y_rel);
				}

				input_sync(dev);
			}
			break;

		default:
			break;
		}
	}
}

/*
 * gc_timer() initiates reads of console pads data.
 */

static void gc_timer(unsigned long private)
{
	struct gc *gc = (void *) private;
		
/*
 * NES and SNES pads or mouse
 */

	if (gc->pad_count[GC_NES] ||
	    gc->pad_count[GC_SNES] ||
	    gc->pad_count[GC_SNESMOUSE]) {
		gc_nes_process_packet(gc);
	}

	mod_timer(&gc->timer, jiffies + GC_REFRESH_TIME);
}

static int gc_open(struct input_dev *dev)
{
	struct gc *gc = input_get_drvdata(dev);
	int err;

	err = mutex_lock_interruptible(&gc->mutex);
	if (err)
		return err;

	if (!gc->used++)
		mod_timer(&gc->timer, jiffies + GC_REFRESH_TIME);

	mutex_unlock(&gc->mutex);
	return 0;
}

static void gc_close(struct input_dev *dev)
{
	struct gc *gc = input_get_drvdata(dev);

	mutex_lock(&gc->mutex);
	if (!--gc->used) {
		del_timer_sync(&gc->timer);
	}
	mutex_unlock(&gc->mutex);
}

static int __init gc_setup_pad(struct gc *gc, int idx, int pad_type)
{
	struct gc_pad *pad = &gc->pads[idx];
	struct input_dev *input_dev;
	int i;
	int err;

	if (pad_type < 1 || pad_type >= GC_MAX) {
		pr_err("Pad type %d unknown\n", pad_type);
		return -EINVAL;
	}

	pad->dev = input_dev = input_allocate_device();
	if (!input_dev) {
		pr_err("Not enough memory for input device\n");
		return -ENOMEM;
	}

	pad->type = pad_type;

	snprintf(pad->phys, sizeof(pad->phys),
		 "input%d", idx);

	input_dev->name = gc_names[pad_type];
	input_dev->phys = pad->phys;
	input_dev->id.bustype = BUS_PARPORT;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = pad_type;
	input_dev->id.version = 0x0100;

	input_set_drvdata(input_dev, gc);

	input_dev->open = gc_open;
	input_dev->close = gc_close;

	if (pad_type != GC_SNESMOUSE) {
		input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);

		for (i = 0; i < 2; i++)
			input_set_abs_params(input_dev, ABS_X + i, -1, 1, 0, 0);
	} else
		input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_REL);

	gc->pad_count[pad_type]++;

	switch (pad_type) {

	case GC_SNESMOUSE:
		__set_bit(BTN_LEFT, input_dev->keybit);
		__set_bit(BTN_RIGHT, input_dev->keybit);
		__set_bit(REL_X, input_dev->relbit);
		__set_bit(REL_Y, input_dev->relbit);
		break;

	case GC_SNES:
		for (i = 4; i < 8; i++)
			__set_bit(gc_snes_btn[i], input_dev->keybit);
	case GC_NES:
		for (i = 0; i < 4; i++)
			__set_bit(gc_snes_btn[i], input_dev->keybit);
		break;
	}

	err = input_register_device(pad->dev);
	if (err)
		goto err_free_dev;
		
				
	/* set data pin to input - uses a mask to set the three bits for the particular gpio to 000 on register0
		e.g. for gpio7 (which uses pins 21-23 on register 0) uses the following mask:
		bit:      31   27   23   19   15   11   7    3  
		value:    1111 1111 0001 1111 1111 1111 1111 1111                                                 */	
	*gpio &= ~(7<<(gc_gpio_ids[idx]*3));
	
	/* enable pull-up on GPIO4 / GPIO7 */
	if ((idx == 0) || (idx == 1)) {
		//enable pull up control by writing 10 to GPPUD register (#37)
		*(gpio+37) = 0x02;
		//wait for setup
		udelay(10);
		//activate this controllers specified gpio port by writing 1 to GPIO specific bit on GPPUDCLK register (#38 for GPIO0-GPIO31, #39 would be GPIO32-53)
		*(gpio+38) = (1 << gc_gpio_ids[idx]);
		//wait for hold
		udelay(10);
		//disable pull up control by writing 00 to GPPUD register
		*(gpio+37) = 0x00;
		//remove the clock from the GPPUDCLK register
		*(gpio+38) = 0x00;
	}
		
	printk("%s data pin connected to GPIO%d\n", gc_names[pad_type], gc_gpio_ids[idx]);

	return 0;

err_free_dev:
	input_free_device(pad->dev);
	pad->dev = NULL;
	return err;
}

static struct gc __init *gc_probe(int *pads, int n_pads)
{
	struct gc *gc;
	int i;
	int count = 0;
	int err;

	gc = kzalloc(sizeof(struct gc), GFP_KERNEL);
	if (!gc) {
		pr_err("Not enough memory\n");
		err = -ENOMEM;
		goto err_out;
	}

	mutex_init(&gc->mutex);
	setup_timer(&gc->timer, gc_timer, (long) gc);

	for (i = 0; i < n_pads && i < GC_MAX_DEVICES; i++) {
		if (!pads[i])
			continue;

		err = gc_setup_pad(gc, i, pads[i]);
		if (err)
			goto err_unreg_devs;

		count++;
	}

	if (count == 0) {
		pr_err("No valid devices specified\n");
		err = -EINVAL;
		goto err_free_gc;
	}
	
	/* setup common pins for each pad type */
	if (gc->pad_count[GC_NES] ||
	    gc->pad_count[GC_SNES] ||
	    gc->pad_count[GC_SNESMOUSE]) {
		
		/* set clk & latch pins to OUTPUT */
		//set GPIO9 as input, SNES controller port pcb has 1 clock for each controller
		//1100 0111 1111 1111 1111 1111 1111 1111
		*gpio &= ~(7<<27);
		//set GPIO9 as output
		//0000 1000 0000 0000 0000 0000 0000 0000
		*gpio |= (1<<27);
		
		//set GPIO10 and 11 as input
		//1100 0000
		*(gpio+1) &= ~0x3f;
		//set GPIO10 and 11 as output
		//0000 1001
		*(gpio+1) |= 0x09;
	}

	return gc;

 err_unreg_devs:
	while (--i >= 0)
		if (gc->pads[i].dev)
			input_unregister_device(gc->pads[i].dev);
 err_free_gc:
	kfree(gc);
 err_out:
	return ERR_PTR(err);
}

static void gc_remove(struct gc *gc)
{
	int i;

	for (i = 0; i < GC_MAX_DEVICES; i++)
		if (gc->pads[i].dev)
			input_unregister_device(gc->pads[i].dev);
	kfree(gc);
}

static int __init gc_init(void)
{
	/* Set up gpio pointer for direct register access */
   	if ((gpio = ioremap(GPIO_BASE, 0xB0)) == NULL) {
   	   	pr_err("io remap failed\n");
   	   	return -EBUSY;
   	}   	

	if (gc_cfg.nargs < 1) {
		pr_err("at least one device must be specified\n");
		return -EINVAL;
	} else {
		gc_base = gc_probe(gc_cfg.args, gc_cfg.nargs);
		if (IS_ERR(gc_base))
			return -ENODEV;
	}

	return 0;
}

static void __exit gc_exit(void)
{
	if (gc_base)
		gc_remove(gc_base);
			
	iounmap(gpio);
}

module_init(gc_init);
module_exit(gc_exit);
