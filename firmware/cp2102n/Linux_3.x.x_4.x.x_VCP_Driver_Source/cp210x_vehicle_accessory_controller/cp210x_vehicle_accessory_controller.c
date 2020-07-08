/*
 * Silicon Laboratories CP210x USB to RS232 serial adaptor driver
 *
 * Copyright (C) 2005 Craig Shelley (craig@microtron.org.uk)
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License version
 *	2 as published by the Free Software Foundation.
 *
 * Support to set flow control line levels using TIOCMGET and TIOCMSET
 * thanks to Karl Hiramoto karl@hiramoto.org. RTSCTS hardware flow
 * control thanks to Munir Nassar nassarmu@real-time.com
 *
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/usb.h>
#include <linux/uaccess.h>
#include <linux/usb/serial.h>

#define DRIVER_DESC "Noctivore Vehicle Accessory Controller UART Bridge"

/*
 * Function Prototypes
 */
static int cp210x_open(struct tty_struct *tty, struct usb_serial_port *);
static void cp210x_close(struct usb_serial_port *);
static int cp210x_ioctl(struct tty_struct *tty,
	unsigned int cmd, unsigned long arg);
static void cp210x_get_termios(struct tty_struct *, struct usb_serial_port *);
static void cp210x_get_termios_port(struct usb_serial_port *port,
	unsigned int *cflagp, unsigned int *baudp);
static void cp210x_change_speed(struct tty_struct *, struct usb_serial_port *,
							struct ktermios *);
static void cp210x_set_termios(struct tty_struct *, struct usb_serial_port *,
							struct ktermios*);
static bool cp210x_tx_empty(struct usb_serial_port *port);
static int cp210x_tiocmget(struct tty_struct *);
static int cp210x_tiocmset(struct tty_struct *, unsigned int, unsigned int);
static int cp210x_tiocmset_port(struct usb_serial_port *port,
		unsigned int, unsigned int);
static void cp210x_break_ctl(struct tty_struct *, int);
static int cp210x_port_probe(struct usb_serial_port *);
static int cp210x_port_remove(struct usb_serial_port *);
static void cp210x_dtr_rts(struct usb_serial_port *p, int on);

static const struct usb_device_id id_table[] = {
	{ USB_DEVICE(0xC0DE, 0xBABE) }, /* Noctivore Vehicle Accessory Controller UART Bridge */
	{ } /* Terminating Entry */
};

MODULE_DEVICE_TABLE(usb, id_table);

struct cp210x_port_private {
	__u8			bPartNumber;
	__u8			bInterfaceNumber;
	bool			has_swapped_line_ctl;
	bool			is_cp2102n_a01;
};

static struct usb_serial_driver cp210x_device = {
	.driver = {
		.owner =	THIS_MODULE,
		.name =		"cp210x",
	},
	.id_table		= id_table,
	.num_ports		= 1,
	.bulk_in_size		= 256,
	.bulk_out_size		= 256,
	.open			= cp210x_open,
	.close			= cp210x_close,
	.ioctl			= cp210x_ioctl,
	.break_ctl		= cp210x_break_ctl,
	.set_termios		= cp210x_set_termios,
	.tx_empty		= cp210x_tx_empty,
	.tiocmget		= cp210x_tiocmget,
	.tiocmset		= cp210x_tiocmset,
	.port_probe		= cp210x_port_probe,
	.port_remove		= cp210x_port_remove,
	.dtr_rts		= cp210x_dtr_rts
};

/* IOCTLs */
#define IOCTL_GPIOGET		0x8000
#define IOCTL_GPIOSET		0x8001

static struct usb_serial_driver * const serial_drivers[] = {
	&cp210x_device, NULL
};

/* Config request types */
#define REQTYPE_HOST_TO_INTERFACE	0x41
#define REQTYPE_INTERFACE_TO_HOST	0xc1
#define REQTYPE_HOST_TO_DEVICE	0x40
#define REQTYPE_DEVICE_TO_HOST	0xc0

/* Config request codes */
#define CP210X_IFC_ENABLE	0x00
#define CP210X_SET_BAUDDIV	0x01
#define CP210X_GET_BAUDDIV	0x02
#define CP210X_SET_LINE_CTL	0x03
#define CP210X_GET_LINE_CTL	0x04
#define CP210X_SET_BREAK	0x05
#define CP210X_IMM_CHAR		0x06
#define CP210X_SET_MHS		0x07
#define CP210X_GET_MDMSTS	0x08
#define CP210X_SET_XON		0x09
#define CP210X_SET_XOFF		0x0A
#define CP210X_SET_EVENTMASK	0x0B
#define CP210X_GET_EVENTMASK	0x0C
#define CP210X_SET_CHAR		0x0D
#define CP210X_GET_CHARS	0x0E
#define CP210X_GET_PROPS	0x0F
#define CP210X_GET_COMM_STATUS	0x10
#define CP210X_RESET		0x11
#define CP210X_PURGE		0x12
#define CP210X_SET_FLOW		0x13
#define CP210X_GET_FLOW		0x14
#define CP210X_EMBED_EVENTS	0x15
#define CP210X_GET_EVENTSTATE	0x16
#define CP210X_SET_CHARS	0x19
#define CP210X_GET_BAUDRATE	0x1D
#define CP210X_SET_BAUDRATE	0x1E
#define CP210X_VENDOR_SPECIFIC	0xFF

/* CP210X_IFC_ENABLE */
#define UART_ENABLE		0x0001
#define UART_DISABLE		0x0000

/* CP210X_(SET|GET)_BAUDDIV */
#define BAUD_RATE_GEN_FREQ	0x384000

/* CP210X_(SET|GET)_LINE_CTL */
#define BITS_DATA_MASK		0X0f00
#define BITS_DATA_5		0X0500
#define BITS_DATA_6		0X0600
#define BITS_DATA_7		0X0700
#define BITS_DATA_8		0X0800
#define BITS_DATA_9		0X0900

#define BITS_PARITY_MASK	0x00f0
#define BITS_PARITY_NONE	0x0000
#define BITS_PARITY_ODD		0x0010
#define BITS_PARITY_EVEN	0x0020
#define BITS_PARITY_MARK	0x0030
#define BITS_PARITY_SPACE	0x0040

#define BITS_STOP_MASK		0x000f
#define BITS_STOP_1		0x0000
#define BITS_STOP_1_5		0x0001
#define BITS_STOP_2		0x0002

/* CP210X_SET_BREAK */
#define BREAK_ON		0x0001
#define BREAK_OFF		0x0000

/* CP210X_(SET_MHS|GET_MDMSTS) */
#define CONTROL_DTR		0x0001
#define CONTROL_RTS		0x0002
#define CONTROL_CTS		0x0010
#define CONTROL_DSR		0x0020
#define CONTROL_RING		0x0040
#define CONTROL_DCD		0x0080
#define CONTROL_WRITE_DTR	0x0100
#define CONTROL_WRITE_RTS	0x0200

/* CP210X_VENDOR_SPECIFIC sub-commands passed in wValue */
#define CP210X_WRITE_LATCH	0x37E1
#define CP210X_READ_LATCH	0x00C2
#define CP210X_GET_PARTNUM	0x370B
#define CP2102N_GET_FW_VERS 0x0010

/* CP210X_GET_PARTNUM returns one of these */
#define CP2101_PARTNUM		0x01
#define CP2102_PARTNUM		0x02
#define CP2103_PARTNUM		0x03
#define CP2104_PARTNUM		0x04
#define CP2105_PARTNUM		0x05
#define CP2108_PARTNUM		0x08
#define CP210x_PARTNUM_CP2102N_QFN28	0x20
#define CP210x_PARTNUM_CP2102N_QFN24	0x21
#define CP210x_PARTNUM_CP2102N_QFN20	0x22

/* CP210X_GET_COMM_STATUS returns these 0x13 bytes */
struct cp210x_comm_status {
	__le32   ulErrors;
	__le32   ulHoldReasons;
	__le32   ulAmountInInQueue;
	__le32   ulAmountInOutQueue;
	u8       bEofReceived;
	u8       bWaitForImmediate;
	u8       bReserved;
} __packed;

/*
 * CP210X_PURGE - 16 bits passed in wValue of USB request.
 * SiLabs app note AN571 gives a strange description of the 4 bits:
 * bit 0 or bit 2 clears the transmit queue and 1 or 3 receive.
 * writing 1 to all, however, purges cp2108 well enough to avoid the hang.
 */
#define PURGE_ALL		0x000f

/* CP210X_GET_FLOW/CP210X_SET_FLOW read/write these 0x10 bytes */
struct cp210x_flow_ctl {
	__le32	ulControlHandshake;
	__le32	ulFlowReplace;
	__le32	ulXonLimit;
	__le32	ulXoffLimit;
} __packed;

/* cp210x_flow_ctl::ulControlHandshake */
#define CP210X_SERIAL_DTR_MASK		GENMASK(1, 0)
#define CP210X_SERIAL_DTR_SHIFT(_mode)	(_mode)
#define CP210X_SERIAL_CTS_HANDSHAKE	BIT(3)
#define CP210X_SERIAL_DSR_HANDSHAKE	BIT(4)
#define CP210X_SERIAL_DCD_HANDSHAKE	BIT(5)
#define CP210X_SERIAL_DSR_SENSITIVITY	BIT(6)

/* values for cp210x_flow_ctl::ulControlHandshake::CP210X_SERIAL_DTR_MASK */
#define CP210X_SERIAL_DTR_INACTIVE	0
#define CP210X_SERIAL_DTR_ACTIVE	1
#define CP210X_SERIAL_DTR_FLOW_CTL	2

/* cp210x_flow_ctl::ulFlowReplace */
#define CP210X_SERIAL_AUTO_TRANSMIT	BIT(0)
#define CP210X_SERIAL_AUTO_RECEIVE	BIT(1)
#define CP210X_SERIAL_ERROR_CHAR	BIT(2)
#define CP210X_SERIAL_NULL_STRIPPING	BIT(3)
#define CP210X_SERIAL_BREAK_CHAR	BIT(4)
#define CP210X_SERIAL_RTS_MASK		GENMASK(7, 6)
#define CP210X_SERIAL_RTS_SHIFT(_mode)	(_mode << 6)
#define CP210X_SERIAL_XOFF_CONTINUE	BIT(31)

/* values for cp210x_flow_ctl::ulFlowReplace::CP210X_SERIAL_RTS_MASK */
#define CP210X_SERIAL_RTS_INACTIVE	0
#define CP210X_SERIAL_RTS_ACTIVE	1
#define CP210X_SERIAL_RTS_FLOW_CTL	2

/*
 * Reads a variable-sized block of CP210X_ registers, identified by req.
 * Returns data into buf in native USB byte order.
 */
static int cp210x_read_reg_block(struct usb_serial_port *port, u8 req,
		void *buf, int bufsize)
{
	struct usb_serial *serial = port->serial;
	struct cp210x_port_private *port_priv = usb_get_serial_port_data(port);
	void *dmabuf;
	int result;

	dmabuf = kmalloc(bufsize, GFP_KERNEL);
	if (!dmabuf) {
		/*
		 * FIXME Some callers don't bother to check for error,
		 * at least give them consistent junk until they are fixed
		 */
		memset(buf, 0, bufsize);
		return -ENOMEM;
	}

	result = usb_control_msg(serial->dev, usb_rcvctrlpipe(serial->dev, 0),
			req, REQTYPE_INTERFACE_TO_HOST, 0,
			port_priv->bInterfaceNumber, dmabuf, bufsize,
			USB_CTRL_SET_TIMEOUT);
	if (result == bufsize) {
		memcpy(buf, dmabuf, bufsize);
		result = 0;
	} else {
		dev_err(&port->dev, "failed get req 0x%x size %d status: %d\n",
				req, bufsize, result);
		if (result >= 0)
			result = -EPROTO;

		/*
		 * FIXME Some callers don't bother to check for error,
		 * at least give them consistent junk until they are fixed
		 */
		memset(buf, 0, bufsize);
	}

	kfree(dmabuf);

	return result;
}

/*
 * Reads any 32-bit CP210X_ register identified by req.
 */
static int cp210x_read_u32_reg(struct usb_serial_port *port, u8 req, u32 *val)
{
	__le32 le32_val;
	int err;

	err = cp210x_read_reg_block(port, req, &le32_val, sizeof(le32_val));
	if (err) {
		/*
		 * FIXME Some callers don't bother to check for error,
		 * at least give them consistent junk until they are fixed
		 */
		*val = 0;
		return err;
	}

	*val = le32_to_cpu(le32_val);

	return 0;
}

/*
 * Reads any 16-bit CP210X_ register identified by req.
 */
static int cp210x_read_u16_reg(struct usb_serial_port *port, u8 req, u16 *val)
{
	__le16 le16_val;
	int err;

	err = cp210x_read_reg_block(port, req, &le16_val, sizeof(le16_val));
	if (err)
		return err;

	*val = le16_to_cpu(le16_val);

	return 0;
}

/*
 * Reads any 8-bit CP210X_ register identified by req.
 */
static int cp210x_read_u8_reg(struct usb_serial_port *port, u8 req, u8 *val)
{
	return cp210x_read_reg_block(port, req, val, sizeof(*val));
}

/*
 * Writes any 16-bit CP210X_ register (req) whose value is passed
 * entirely in the wValue field of the USB request.
 */
static int cp210x_write_u16_reg(struct usb_serial_port *port, u8 req, u16 val)
{
	struct usb_serial *serial = port->serial;
	struct cp210x_port_private *port_priv = usb_get_serial_port_data(port);
	int result;

	result = usb_control_msg(serial->dev, usb_sndctrlpipe(serial->dev, 0),
			req, REQTYPE_HOST_TO_INTERFACE, val,
			port_priv->bInterfaceNumber, NULL, 0,
			USB_CTRL_SET_TIMEOUT);
	if (result < 0) {
		dev_err(&port->dev, "failed set request 0x%x status: %d\n",
				req, result);
	}

	return result;
}

/*
 * Writes a variable-sized block of CP210X_ registers, identified by req.
 * Data in buf must be in native USB byte order.
 */
static int cp210x_write_reg_block(struct usb_serial_port *port, u8 req,
		void *buf, int bufsize)
{
	struct usb_serial *serial = port->serial;
	struct cp210x_port_private *port_priv = usb_get_serial_port_data(port);
	void *dmabuf;
	int result;

	dmabuf = kmalloc(bufsize, GFP_KERNEL);
	if (!dmabuf)
		return -ENOMEM;

	memcpy(dmabuf, buf, bufsize);

	result = usb_control_msg(serial->dev, usb_sndctrlpipe(serial->dev, 0),
			req, REQTYPE_HOST_TO_INTERFACE, 0,
			port_priv->bInterfaceNumber, dmabuf, bufsize,
			USB_CTRL_SET_TIMEOUT);

	kfree(dmabuf);

	if (result == bufsize) {
		result = 0;
	} else {
		dev_err(&port->dev, "failed set req 0x%x size %d status: %d\n",
				req, bufsize, result);
		if (result >= 0)
			result = -EPROTO;
	}

	return result;
}

/*
 * Writes any 32-bit CP210X_ register identified by req.
 */
static int cp210x_write_u32_reg(struct usb_serial_port *port, u8 req, u32 val)
{
	__le32 le32_val;

	le32_val = cpu_to_le32(val);

	return cp210x_write_reg_block(port, req, &le32_val, sizeof(le32_val));
}

/*
 * Detect CP2108 GET_LINE_CTL bug and activate workaround.
 * Write a known good value 0x800, read it back.
 * If it comes back swapped the bug is detected.
 * Preserve the original register value.
 */
static int cp210x_detect_swapped_line_ctl(struct usb_serial_port *port)
{
	struct cp210x_port_private *port_priv = usb_get_serial_port_data(port);
	u16 line_ctl_save;
	u16 line_ctl_test;
	int err;

	err = cp210x_read_u16_reg(port, CP210X_GET_LINE_CTL, &line_ctl_save);
	if (err)
		return err;

	err = cp210x_write_u16_reg(port, CP210X_SET_LINE_CTL, 0x800);
	if (err)
		return err;

	err = cp210x_read_u16_reg(port, CP210X_GET_LINE_CTL, &line_ctl_test);
	if (err)
		return err;

	if (line_ctl_test == 8) {
		port_priv->has_swapped_line_ctl = true;
		line_ctl_save = swab16(line_ctl_save);
	}

	return cp210x_write_u16_reg(port, CP210X_SET_LINE_CTL, line_ctl_save);
}

/*
 * Must always be called instead of cp210x_read_u16_reg(CP210X_GET_LINE_CTL)
 * to workaround cp2108 bug and get correct value.
 */
static int cp210x_get_line_ctl(struct usb_serial_port *port, u16 *ctl)
{
	struct cp210x_port_private *port_priv = usb_get_serial_port_data(port);
	int err;

	err = cp210x_read_u16_reg(port, CP210X_GET_LINE_CTL, ctl);
	if (err)
		return err;

	/* Workaround swapped bytes in 16-bit value from CP210X_GET_LINE_CTL */
	if (port_priv->has_swapped_line_ctl)
		*ctl = swab16(*ctl);

	return 0;
}

/*
 * cp210x_quantise_baudrate
 * Quantises the baud rate as per AN205 Table 1
 */
static unsigned int cp210x_quantise_baudrate(unsigned int baud)
{
	if (baud <= 300)
		baud = 300;
	else if (baud <= 600)      baud = 600;
	else if (baud <= 1200)     baud = 1200;
	else if (baud <= 1800)     baud = 1800;
	else if (baud <= 2400)     baud = 2400;
	else if (baud <= 4000)     baud = 4000;
	else if (baud <= 4803)     baud = 4800;
	else if (baud <= 7207)     baud = 7200;
	else if (baud <= 9612)     baud = 9600;
	else if (baud <= 14428)    baud = 14400;
	else if (baud <= 16062)    baud = 16000;
	else if (baud <= 19250)    baud = 19200;
	else if (baud <= 28912)    baud = 28800;
	else if (baud <= 38601)    baud = 38400;
	else if (baud <= 51558)    baud = 51200;
	else if (baud <= 56280)    baud = 56000;
	else if (baud <= 58053)    baud = 57600;
	else if (baud <= 64111)    baud = 64000;
	else if (baud <= 77608)    baud = 76800;
	else if (baud <= 117028)   baud = 115200;
	else if (baud <= 129347)   baud = 128000;
	else if (baud <= 156868)   baud = 153600;
	else if (baud <= 237832)   baud = 230400;
	else if (baud <= 254234)   baud = 250000;
	else if (baud <= 273066)   baud = 256000;
	else if (baud <= 491520)   baud = 460800;
	else if (baud <= 567138)   baud = 500000;
	else if (baud <= 670254)   baud = 576000;
	else if (baud < 1000000)
		baud = 921600;
	else if (baud > 2000000)
		baud = 2000000;
	return baud;
}

static int cp210x_open(struct tty_struct *tty, struct usb_serial_port *port)
{
	int result;

	result = cp210x_write_u16_reg(port, CP210X_IFC_ENABLE, UART_ENABLE);
	if (result) {
		dev_err(&port->dev, "%s - Unable to enable UART\n", __func__);
		return result;
	}

	/* Configure the termios structure */
	cp210x_get_termios(tty, port);

	/* The baud rate must be initialised on cp2104 */
	if (tty)
		cp210x_change_speed(tty, port, NULL);

	return usb_serial_generic_open(tty, port);
}

static void cp210x_close(struct usb_serial_port *port)
{
	struct cp210x_port_private *port_priv = usb_get_serial_port_data(port);
	
	usb_serial_generic_close(port);

	/* Clear both queues; cp2108 needs this to avoid an occasional hang */
	cp210x_write_u16_reg(port, CP210X_PURGE, PURGE_ALL);

	cp210x_write_u16_reg(port, CP210X_IFC_ENABLE, UART_DISABLE);
	
	if (port_priv->is_cp2102n_a01) {
        int result;

		struct usb_serial *serial = port->serial;

        result = usb_control_msg(serial->dev,
				usb_sndctrlpipe(serial->dev, 0),
				0x09, 
				0,
				0,
				port_priv->bInterfaceNumber, 
				NULL, 
				0,
				USB_CTRL_SET_TIMEOUT);
				
				
		if (result < 0) {
			dev_err(&port->dev, "failed usb_control_msg (1) request 0x09 status: %d\n",
					result);
		}
		
		result = usb_control_msg(serial->dev, 
				usb_sndctrlpipe(serial->dev, 0),
				0x09, 
				0,
				1,
				port_priv->bInterfaceNumber, 
				NULL, 
				0,
				USB_CTRL_SET_TIMEOUT);
				
				
		if (result < 0) {
			dev_err(&port->dev, "failed usb_control_msg (2) request 0x09 status: %d\n",
					result);
		}


		result = usb_control_msg(serial->dev,
				usb_sndctrlpipe(serial->dev, 0),
				0x01, 
				2,
				0,
				0x82,
				NULL, 
				0,
				USB_CTRL_SET_TIMEOUT);
				
				
		if (result < 0) {
			dev_err(&port->dev, "failed usb_control_msg (1) request 0x01 status: %d\n",
					result);
		}
		
		result = usb_control_msg(serial->dev, 
				usb_sndctrlpipe(serial->dev, 0),
				0x01, 
				2,
				0,
				0x02,
				NULL, 
				0,
				USB_CTRL_SET_TIMEOUT);
				
				
		if (result < 0) {
			dev_err(&port->dev, "failed usb_control_msg (2) request 0x01 status: %d\n",
					result);
		}
			
	}
}

/*
 * Reads a variable-sized vendor-specific register identified by wValue
 * Returns data into buf in native USB byte order.
 */
static int cp210x_read_vs_reg_block(struct usb_serial_port *port, u8 bmRequestType,
		u16 wValue, void *buf, int bufsize)
{
	struct usb_serial *serial = port->serial;
	struct cp210x_port_private *port_priv = usb_get_serial_port_data(port);
	void *dmabuf;
	int result;

	dmabuf = kmalloc(bufsize, GFP_KERNEL);
	if (!dmabuf)
		return -ENOMEM;

	result = usb_control_msg(serial->dev, usb_rcvctrlpipe(serial->dev, 0),
			CP210X_VENDOR_SPECIFIC, bmRequestType, wValue,
			port_priv->bInterfaceNumber, dmabuf, bufsize,
			USB_CTRL_SET_TIMEOUT);
	if (result == bufsize) {
		memcpy(buf, dmabuf, bufsize);
		result = 0;
	} else {
		dev_err(&port->dev, "failed get VENDOR_SPECIFIC wValue 0x%x size %d status: %d\n",
				wValue, bufsize, result);
		if (result >= 0)
			result = -EPROTO;
	}

	kfree(dmabuf);

	return result;
}

/* GPIO register read from single-interface CP210x */
static int cp210x_read_device_gpio_u8(struct usb_serial_port *port, u8 *val)
{
	return cp210x_read_vs_reg_block(port, REQTYPE_DEVICE_TO_HOST, CP210X_READ_LATCH, val, 1);
}

/* GPIO register read from CP2105 */
static int cp210x_read_interface_gpio_u8(struct usb_serial_port *port, u8 *val)
{
	return cp210x_read_vs_reg_block(port, REQTYPE_INTERFACE_TO_HOST, CP210X_READ_LATCH, val, 1);
}

/* GPIO register read from CP2108 */
static int cp210x_read_device_gpio_u16(struct usb_serial_port *port, u16 *val)
{
	__le16 le16_val;
	int err = cp210x_read_vs_reg_block(port, REQTYPE_DEVICE_TO_HOST, CP210X_READ_LATCH, &le16_val, 2);
	if (err)
		return err;

	*val = le16_to_cpu(le16_val);
	return 0;
}

/* GPIO register write to single-interface CP210x */
static int cp210x_write_device_gpio_u16(struct usb_serial_port *port, u16 val)
{
	int result;

	result = usb_control_msg(port->serial->dev,
			usb_sndctrlpipe(port->serial->dev, 0),
			CP210X_VENDOR_SPECIFIC,
			REQTYPE_HOST_TO_DEVICE,
			CP210X_WRITE_LATCH, /* wValue */
			val, /* wIndex */
			NULL, 0, USB_CTRL_SET_TIMEOUT);
	if (result != 0) {
		dev_err(&port->dev, "failed set WRITE_LATCH status: %d\n",
				result);
		if (result >= 0)
			result = -EPROTO;
	}
	return result;
}

/*
 * Writes a variable-sized block of CP210X_ registers, identified by req.
 * Data in buf must be in native USB byte order.
 */
static int cp210x_write_gpio_reg_block(struct usb_serial_port *port,
		u8 bmRequestType, void *buf, int bufsize)
{
	struct usb_serial *serial = port->serial;
	struct cp210x_port_private *port_priv = usb_get_serial_port_data(port);
	void *dmabuf;
	int result;

	dmabuf = kmalloc(bufsize, GFP_KERNEL);
	if (!dmabuf)
		return -ENOMEM;

	memcpy(dmabuf, buf, bufsize);

	result = usb_control_msg(serial->dev, usb_sndctrlpipe(serial->dev, 0),
			CP210X_VENDOR_SPECIFIC, bmRequestType,
			CP210X_WRITE_LATCH,  /* wValue */
			port_priv->bInterfaceNumber, /* wIndex */
			dmabuf, bufsize,
			USB_CTRL_SET_TIMEOUT);

	kfree(dmabuf);

	if (result == bufsize) {
		result = 0;
	} else {
		dev_err(&port->dev, "failed set WRITE_LATCH size %d status: %d\n",
				bufsize, result);
		if (result >= 0)
			result = -EPROTO;
	}

	return result;
}

/* GPIO register write to CP2105 */
static int cp210x_write_interface_gpio_u16(struct usb_serial_port *port, u16 val)
{
	__le16 le16_val = cpu_to_le16(val);

	return cp210x_write_gpio_reg_block(port, REQTYPE_HOST_TO_INTERFACE, &le16_val, 2);
}

/* GPIO register write to CP2108 */
static int cp210x_write_device_gpio_u32(struct usb_serial_port *port, u32 val)
{
	__le32 le32_val = cpu_to_le32(val);

	return cp210x_write_gpio_reg_block(port, REQTYPE_HOST_TO_DEVICE, &le32_val, 4);
}

static int cp210x_ioctl(struct tty_struct *tty,
	unsigned int cmd, unsigned long arg)
{
	struct usb_serial_port *port = tty->driver_data;
	struct cp210x_port_private *port_priv = usb_get_serial_port_data(port);

	switch (cmd) {
	case IOCTL_GPIOGET:
		if ((port_priv->bPartNumber == CP2103_PARTNUM) ||
			(port_priv->bPartNumber == CP2104_PARTNUM) ||
			(port_priv->bPartNumber == CP210x_PARTNUM_CP2102N_QFN28) ||
			(port_priv->bPartNumber == CP210x_PARTNUM_CP2102N_QFN24) ||
			(port_priv->bPartNumber == CP210x_PARTNUM_CP2102N_QFN20)) {
			u8 gpio;
			int err = cp210x_read_device_gpio_u8(port, &gpio);
			if (err)
				return err;
			if (copy_to_user((void*)arg, &gpio, sizeof(gpio)))
				return -EFAULT;
			return 0;
		}
		else if (port_priv->bPartNumber == CP2105_PARTNUM) {
			u8 gpio;
			int err = cp210x_read_interface_gpio_u8(port, &gpio);
			if (err)
				return err;
			if (copy_to_user((void*)arg, &gpio, sizeof(gpio)))
				return -EFAULT;
			return 0;
		}
		else if (port_priv->bPartNumber == CP2108_PARTNUM) {
			u16 gpio;
			int err = cp210x_read_device_gpio_u16(port, &gpio);
			if (err)
				return err;
			if (copy_to_user((void*)arg, &gpio, sizeof(gpio)))
				return -EFAULT;
			return 0;
		}
		else {
			return -ENOTSUPP;
		}
		break;
	case IOCTL_GPIOSET:
		if ((port_priv->bPartNumber == CP2103_PARTNUM) ||
			(port_priv->bPartNumber == CP2104_PARTNUM) ||
			(port_priv->bPartNumber == CP210x_PARTNUM_CP2102N_QFN28) ||
			(port_priv->bPartNumber == CP210x_PARTNUM_CP2102N_QFN24) ||
			(port_priv->bPartNumber == CP210x_PARTNUM_CP2102N_QFN20)) {
			u16 gpio;
			if (copy_from_user(&gpio, (void*)arg, sizeof(gpio)))
				return -EFAULT;
			return cp210x_write_device_gpio_u16(port, gpio);
		}
		else if (port_priv->bPartNumber == CP2105_PARTNUM) {
			u16 gpio;
			if (copy_from_user(&gpio, (void*)arg, sizeof(gpio)))
				return -EFAULT;
			return cp210x_write_interface_gpio_u16(port, gpio);
		}
		else if (port_priv->bPartNumber == CP2108_PARTNUM) {
			u32 gpio;
			if (copy_from_user(&gpio, (void*)arg, sizeof(gpio)))
				return -EFAULT;
			return cp210x_write_device_gpio_u32(port, gpio);
		}
		else {
			return -ENOTSUPP;
		}
		break;

	default:
		break;
	}

	return -ENOIOCTLCMD;
}

/*
 * Read how many bytes are waiting in the TX queue.
 */
static int cp210x_get_tx_queue_byte_count(struct usb_serial_port *port,
		u32 *count)
{
	struct usb_serial *serial = port->serial;
	struct cp210x_port_private *port_priv = usb_get_serial_port_data(port);
	struct cp210x_comm_status *sts;
	int result;

	sts = kmalloc(sizeof(*sts), GFP_KERNEL);
	if (!sts)
		return -ENOMEM;

	result = usb_control_msg(serial->dev, usb_rcvctrlpipe(serial->dev, 0),
			CP210X_GET_COMM_STATUS, REQTYPE_INTERFACE_TO_HOST,
			0, port_priv->bInterfaceNumber, sts, sizeof(*sts),
			USB_CTRL_GET_TIMEOUT);
	if (result == sizeof(*sts)) {
		*count = le32_to_cpu(sts->ulAmountInOutQueue);
		result = 0;
	} else {
		dev_err(&port->dev, "failed to get comm status: %d\n", result);
		if (result >= 0)
			result = -EPROTO;
	}

	kfree(sts);

	return result;
}

static bool cp210x_tx_empty(struct usb_serial_port *port)
{
	int err;
	u32 count;

	err = cp210x_get_tx_queue_byte_count(port, &count);
	if (err)
		return true;

	return !count;
}

/*
 * cp210x_get_termios
 * Reads the baud rate, data bits, parity, stop bits and flow control mode
 * from the device, corrects any unsupported values, and configures the
 * termios structure to reflect the state of the device
 */
static void cp210x_get_termios(struct tty_struct *tty,
	struct usb_serial_port *port)
{
	unsigned int baud;

	if (tty) {
		cp210x_get_termios_port(tty->driver_data,
			&tty->termios.c_cflag, &baud);
		tty_encode_baud_rate(tty, baud, baud);
	} else {
		unsigned int cflag;
		cflag = 0;
		cp210x_get_termios_port(port, &cflag, &baud);
	}
}

/*
 * cp210x_get_termios_port
 * This is the heart of cp210x_get_termios which always uses a &usb_serial_port.
 */
static void cp210x_get_termios_port(struct usb_serial_port *port,
	unsigned int *cflagp, unsigned int *baudp)
{
	struct device *dev = &port->dev;
	unsigned int cflag;
	struct cp210x_flow_ctl flow_ctl;
	u32 baud;
	u16 bits;
	u32 ctl_hs;

	cp210x_read_u32_reg(port, CP210X_GET_BAUDRATE, &baud);

	dev_dbg(dev, "%s - baud rate = %d\n", __func__, baud);
	*baudp = baud;

	cflag = *cflagp;

	cp210x_get_line_ctl(port, &bits);
	cflag &= ~CSIZE;
	switch (bits & BITS_DATA_MASK) {
	case BITS_DATA_5:
		dev_dbg(dev, "%s - data bits = 5\n", __func__);
		cflag |= CS5;
		break;
	case BITS_DATA_6:
		dev_dbg(dev, "%s - data bits = 6\n", __func__);
		cflag |= CS6;
		break;
	case BITS_DATA_7:
		dev_dbg(dev, "%s - data bits = 7\n", __func__);
		cflag |= CS7;
		break;
	case BITS_DATA_8:
		dev_dbg(dev, "%s - data bits = 8\n", __func__);
		cflag |= CS8;
		break;
	case BITS_DATA_9:
		dev_dbg(dev, "%s - data bits = 9 (not supported, using 8 data bits)\n", __func__);
		cflag |= CS8;
		bits &= ~BITS_DATA_MASK;
		bits |= BITS_DATA_8;
		cp210x_write_u16_reg(port, CP210X_SET_LINE_CTL, bits);
		break;
	default:
		dev_dbg(dev, "%s - Unknown number of data bits, using 8\n", __func__);
		cflag |= CS8;
		bits &= ~BITS_DATA_MASK;
		bits |= BITS_DATA_8;
		cp210x_write_u16_reg(port, CP210X_SET_LINE_CTL, bits);
		break;
	}

	switch (bits & BITS_PARITY_MASK) {
	case BITS_PARITY_NONE:
		dev_dbg(dev, "%s - parity = NONE\n", __func__);
		cflag &= ~PARENB;
		break;
	case BITS_PARITY_ODD:
		dev_dbg(dev, "%s - parity = ODD\n", __func__);
		cflag |= (PARENB|PARODD);
		break;
	case BITS_PARITY_EVEN:
		dev_dbg(dev, "%s - parity = EVEN\n", __func__);
		cflag &= ~PARODD;
		cflag |= PARENB;
		break;
	case BITS_PARITY_MARK:
		dev_dbg(dev, "%s - parity = MARK\n", __func__);
		cflag |= (PARENB|PARODD|CMSPAR);
		break;
	case BITS_PARITY_SPACE:
		dev_dbg(dev, "%s - parity = SPACE\n", __func__);
		cflag &= ~PARODD;
		cflag |= (PARENB|CMSPAR);
		break;
	default:
		dev_dbg(dev, "%s - Unknown parity mode, disabling parity\n", __func__);
		cflag &= ~PARENB;
		bits &= ~BITS_PARITY_MASK;
		cp210x_write_u16_reg(port, CP210X_SET_LINE_CTL, bits);
		break;
	}

	cflag &= ~CSTOPB;
	switch (bits & BITS_STOP_MASK) {
	case BITS_STOP_1:
		dev_dbg(dev, "%s - stop bits = 1\n", __func__);
		break;
	case BITS_STOP_1_5:
		dev_dbg(dev, "%s - stop bits = 1.5 (not supported, using 1 stop bit)\n", __func__);
		bits &= ~BITS_STOP_MASK;
		cp210x_write_u16_reg(port, CP210X_SET_LINE_CTL, bits);
		break;
	case BITS_STOP_2:
		dev_dbg(dev, "%s - stop bits = 2\n", __func__);
		cflag |= CSTOPB;
		break;
	default:
		dev_dbg(dev, "%s - Unknown number of stop bits, using 1 stop bit\n", __func__);
		bits &= ~BITS_STOP_MASK;
		cp210x_write_u16_reg(port, CP210X_SET_LINE_CTL, bits);
		break;
	}

	cp210x_read_reg_block(port, CP210X_GET_FLOW, &flow_ctl,
			sizeof(flow_ctl));
	ctl_hs = le32_to_cpu(flow_ctl.ulControlHandshake);
	if (ctl_hs & CP210X_SERIAL_CTS_HANDSHAKE) {
		dev_dbg(dev, "%s - flow control = CRTSCTS\n", __func__);
		cflag |= CRTSCTS;
	} else {
		dev_dbg(dev, "%s - flow control = NONE\n", __func__);
		cflag &= ~CRTSCTS;
	}

	*cflagp = cflag;
}

/*
 * CP2101 supports the following baud rates:
 *
 *	300, 600, 1200, 1800, 2400, 4800, 7200, 9600, 14400, 19200, 28800,
 *	38400, 56000, 57600, 115200, 128000, 230400, 460800, 921600
 *
 * CP2102 and CP2103 support the following additional rates:
 *
 *	4000, 16000, 51200, 64000, 76800, 153600, 250000, 256000, 500000,
 *	576000
 *
 * The device will map a requested rate to a supported one, but the result
 * of requests for rates greater than 1053257 is undefined (see AN205).
 *
 * CP2104, CP2105 and CP2110 support most rates up to 2M, 921k and 1M baud,
 * respectively, with an error less than 1%. The actual rates are determined
 * by
 *
 *	div = round(freq / (2 x prescale x request))
 *	actual = freq / (2 x prescale x div)
 *
 * For CP2104 and CP2105 freq is 48Mhz and prescale is 4 for request <= 365bps
 * or 1 otherwise.
 * For CP2110 freq is 24Mhz and prescale is 4 for request <= 300bps or 1
 * otherwise.
 */
static void cp210x_change_speed(struct tty_struct *tty,
		struct usb_serial_port *port, struct ktermios *old_termios)
{
	u32 baud;

	baud = tty->termios.c_ospeed;

	/* This maps the requested rate to a rate valid on cp2102 or cp2103,
	 * or to an arbitrary rate in [1M,2M].
	 *
	 * NOTE: B0 is not implemented.
	 */
	baud = cp210x_quantise_baudrate(baud);

	dev_dbg(&port->dev, "%s - setting baud rate to %u\n", __func__, baud);
	if (cp210x_write_u32_reg(port, CP210X_SET_BAUDRATE, baud)) {
		dev_warn(&port->dev, "failed to set baud rate to %u\n", baud);
		if (old_termios)
			baud = old_termios->c_ospeed;
		else
			baud = 9600;
	}

	tty_encode_baud_rate(tty, baud, baud);
}

static void cp210x_set_termios(struct tty_struct *tty,
		struct usb_serial_port *port, struct ktermios *old_termios)
{
	struct device *dev = &port->dev;
	unsigned int cflag, old_cflag;
	u16 bits;

	cflag = tty->termios.c_cflag;
	old_cflag = old_termios->c_cflag;

	if (tty->termios.c_ospeed != old_termios->c_ospeed)
		cp210x_change_speed(tty, port, old_termios);

	/* If the number of data bits is to be updated */
	if ((cflag & CSIZE) != (old_cflag & CSIZE)) {
		cp210x_get_line_ctl(port, &bits);
		bits &= ~BITS_DATA_MASK;
		switch (cflag & CSIZE) {
		case CS5:
			bits |= BITS_DATA_5;
			dev_dbg(dev, "%s - data bits = 5\n", __func__);
			break;
		case CS6:
			bits |= BITS_DATA_6;
			dev_dbg(dev, "%s - data bits = 6\n", __func__);
			break;
		case CS7:
			bits |= BITS_DATA_7;
			dev_dbg(dev, "%s - data bits = 7\n", __func__);
			break;
		case CS8:
			bits |= BITS_DATA_8;
			dev_dbg(dev, "%s - data bits = 8\n", __func__);
			break;
		/*case CS9:
			bits |= BITS_DATA_9;
			dev_dbg(dev, "%s - data bits = 9\n", __func__);
			break;*/
		default:
			dev_dbg(dev, "cp210x driver does not support the number of bits requested, using 8 bit mode\n");
			bits |= BITS_DATA_8;
			break;
		}
		if (cp210x_write_u16_reg(port, CP210X_SET_LINE_CTL, bits))
			dev_dbg(dev, "Number of data bits requested not supported by device\n");
	}

	if ((cflag     & (PARENB|PARODD|CMSPAR)) !=
	    (old_cflag & (PARENB|PARODD|CMSPAR))) {
		cp210x_get_line_ctl(port, &bits);
		bits &= ~BITS_PARITY_MASK;
		if (cflag & PARENB) {
			if (cflag & CMSPAR) {
				if (cflag & PARODD) {
					bits |= BITS_PARITY_MARK;
					dev_dbg(dev, "%s - parity = MARK\n", __func__);
				} else {
					bits |= BITS_PARITY_SPACE;
					dev_dbg(dev, "%s - parity = SPACE\n", __func__);
				}
			} else {
				if (cflag & PARODD) {
					bits |= BITS_PARITY_ODD;
					dev_dbg(dev, "%s - parity = ODD\n", __func__);
				} else {
					bits |= BITS_PARITY_EVEN;
					dev_dbg(dev, "%s - parity = EVEN\n", __func__);
				}
			}
		}
		if (cp210x_write_u16_reg(port, CP210X_SET_LINE_CTL, bits))
			dev_dbg(dev, "Parity mode not supported by device\n");
	}

	if ((cflag & CSTOPB) != (old_cflag & CSTOPB)) {
		cp210x_get_line_ctl(port, &bits);
		bits &= ~BITS_STOP_MASK;
		if (cflag & CSTOPB) {
			bits |= BITS_STOP_2;
			dev_dbg(dev, "%s - stop bits = 2\n", __func__);
		} else {
			bits |= BITS_STOP_1;
			dev_dbg(dev, "%s - stop bits = 1\n", __func__);
		}
		if (cp210x_write_u16_reg(port, CP210X_SET_LINE_CTL, bits))
			dev_dbg(dev, "Number of stop bits requested not supported by device\n");
	}

	if ((cflag & CRTSCTS) != (old_cflag & CRTSCTS)) {
		struct cp210x_flow_ctl flow_ctl;
		u32 ctl_hs;
		u32 flow_repl;

		cp210x_read_reg_block(port, CP210X_GET_FLOW, &flow_ctl,
				sizeof(flow_ctl));
		ctl_hs = le32_to_cpu(flow_ctl.ulControlHandshake);
		flow_repl = le32_to_cpu(flow_ctl.ulFlowReplace);
		dev_dbg(dev, "%s - read ulControlHandshake=0x%08x, ulFlowReplace=0x%08x\n",
				__func__, ctl_hs, flow_repl);

		ctl_hs &= ~CP210X_SERIAL_DSR_HANDSHAKE;
		ctl_hs &= ~CP210X_SERIAL_DCD_HANDSHAKE;
		ctl_hs &= ~CP210X_SERIAL_DSR_SENSITIVITY;
		ctl_hs &= ~CP210X_SERIAL_DTR_MASK;
		ctl_hs |= CP210X_SERIAL_DTR_SHIFT(CP210X_SERIAL_DTR_ACTIVE);
		if (cflag & CRTSCTS) {
			ctl_hs |= CP210X_SERIAL_CTS_HANDSHAKE;

			flow_repl &= ~CP210X_SERIAL_RTS_MASK;
			flow_repl |= CP210X_SERIAL_RTS_SHIFT(
					CP210X_SERIAL_RTS_FLOW_CTL);
			dev_dbg(dev, "%s - flow control = CRTSCTS\n", __func__);
		} else {
			ctl_hs &= ~CP210X_SERIAL_CTS_HANDSHAKE;

			flow_repl &= ~CP210X_SERIAL_RTS_MASK;
			flow_repl |= CP210X_SERIAL_RTS_SHIFT(
					CP210X_SERIAL_RTS_ACTIVE);
			dev_dbg(dev, "%s - flow control = NONE\n", __func__);
		}

		dev_dbg(dev, "%s - write ulControlHandshake=0x%08x, ulFlowReplace=0x%08x\n",
				__func__, ctl_hs, flow_repl);
		flow_ctl.ulControlHandshake = cpu_to_le32(ctl_hs);
		flow_ctl.ulFlowReplace = cpu_to_le32(flow_repl);
		cp210x_write_reg_block(port, CP210X_SET_FLOW, &flow_ctl,
				sizeof(flow_ctl));
	}

}

static int cp210x_tiocmset(struct tty_struct *tty,
		unsigned int set, unsigned int clear)
{
	struct usb_serial_port *port = tty->driver_data;
	return cp210x_tiocmset_port(port, set, clear);
}

static int cp210x_tiocmset_port(struct usb_serial_port *port,
		unsigned int set, unsigned int clear)
{
	u16 control = 0;

	if (set & TIOCM_RTS) {
		control |= CONTROL_RTS;
		control |= CONTROL_WRITE_RTS;
	}
	if (set & TIOCM_DTR) {
		control |= CONTROL_DTR;
		control |= CONTROL_WRITE_DTR;
	}
	if (clear & TIOCM_RTS) {
		control &= ~CONTROL_RTS;
		control |= CONTROL_WRITE_RTS;
	}
	if (clear & TIOCM_DTR) {
		control &= ~CONTROL_DTR;
		control |= CONTROL_WRITE_DTR;
	}

	dev_dbg(&port->dev, "%s - control = 0x%.4x\n", __func__, control);

	return cp210x_write_u16_reg(port, CP210X_SET_MHS, control);
}

static void cp210x_dtr_rts(struct usb_serial_port *p, int on)
{
	if (on)
		cp210x_tiocmset_port(p, TIOCM_DTR|TIOCM_RTS, 0);
	else
		cp210x_tiocmset_port(p, 0, TIOCM_DTR|TIOCM_RTS);
}

static int cp210x_tiocmget(struct tty_struct *tty)
{
	struct usb_serial_port *port = tty->driver_data;
	u8 control;
	int result;

	cp210x_read_u8_reg(port, CP210X_GET_MDMSTS, &control);

	result = ((control & CONTROL_DTR) ? TIOCM_DTR : 0)
		|((control & CONTROL_RTS) ? TIOCM_RTS : 0)
		|((control & CONTROL_CTS) ? TIOCM_CTS : 0)
		|((control & CONTROL_DSR) ? TIOCM_DSR : 0)
		|((control & CONTROL_RING)? TIOCM_RI  : 0)
		|((control & CONTROL_DCD) ? TIOCM_CD  : 0);

	dev_dbg(&port->dev, "%s - control = 0x%.2x\n", __func__, control);

	return result;
}

static void cp210x_break_ctl(struct tty_struct *tty, int break_state)
{
	struct usb_serial_port *port = tty->driver_data;
	u16 state;

	if (break_state == 0)
		state = BREAK_OFF;
	else
		state = BREAK_ON;
	dev_dbg(&port->dev, "%s - turning break %s\n", __func__,
		state == BREAK_OFF ? "off" : "on");
	cp210x_write_u16_reg(port, CP210X_SET_BREAK, state);
}

static int cp210x_port_probe(struct usb_serial_port *port)
{
	struct usb_serial *serial = port->serial;
	struct usb_host_interface *cur_altsetting;
	struct cp210x_port_private *port_priv;
	int ret;

	port_priv = kzalloc(sizeof(*port_priv), GFP_KERNEL);
	if (!port_priv)
		return -ENOMEM;

	cur_altsetting = serial->interface->cur_altsetting;
	port_priv->bInterfaceNumber = cur_altsetting->desc.bInterfaceNumber;
	port_priv->is_cp2102n_a01 = false;

	usb_set_serial_port_data(port, port_priv);

	ret = cp210x_read_vs_reg_block(port, REQTYPE_DEVICE_TO_HOST,
			CP210X_GET_PARTNUM, &port_priv->bPartNumber, 1);
	if (ret) {
		kfree(port_priv);
		return ret;
	}
	
	if ((port_priv->bPartNumber == CP210x_PARTNUM_CP2102N_QFN28) ||
		(port_priv->bPartNumber == CP210x_PARTNUM_CP2102N_QFN24) ||
		(port_priv->bPartNumber == CP210x_PARTNUM_CP2102N_QFN20)) {
			// check the firmware version on the CP2102N.
			u8 fwversion[3];
			ret = cp210x_read_vs_reg_block(port, REQTYPE_DEVICE_TO_HOST, CP2102N_GET_FW_VERS, fwversion, sizeof(fwversion));
			if (0 == ret) {
				if (1 == fwversion[0] && 0 == fwversion[1] && fwversion[2] < 8) {
					port_priv->is_cp2102n_a01 = true;
				}
			}
			
		}

	ret = cp210x_detect_swapped_line_ctl(port);
	if (ret) {
		kfree(port_priv);
		return ret;
	}

	return 0;
}

static int cp210x_port_remove(struct usb_serial_port *port)
{
	struct cp210x_port_private *port_priv;

	port_priv = usb_get_serial_port_data(port);
	kfree(port_priv);

	return 0;
}

module_usb_serial_driver(serial_drivers, id_table);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
