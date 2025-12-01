// SPDX-License-Identifier: GPL-2.0
/*
 * Defines interfaces for interacting with the Raspberry Pi firmware's
 * property channel.
 *
 * Copyright © 2015 Broadcom
 */

#include <linux/dma-mapping.h>
#include <linux/kref.h>
#include <linux/mailbox_client.h>
#include <linux/mailbox_controller.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/slab.h>
#include <soc/bcm2835/raspberrypi-firmware.h>

#define MBOX_MSG(chan, data28)		(((data28) & ~0xf) | ((chan) & 0xf))
#define MBOX_CHAN(msg)			((msg) & 0xf)
#define MBOX_DATA28(msg)		((msg) & ~0xf)
#define MBOX_CHAN_PROPERTY		8

static struct platform_device *rpi_hwmon;
static struct platform_device *rpi_clk;

struct rpi_firmware {
	struct mbox_client cl;
	struct mbox_chan *chan; /* The property channel. */
	struct completion c;
	u32 enabled;

	struct kref consumers;
};

static struct platform_device *g_pdev;

static DEFINE_MUTEX(transaction_lock);

static void response_callback(struct mbox_client *cl, void *msg)
{
	struct rpi_firmware *fw = container_of(cl, struct rpi_firmware, cl);
	complete(&fw->c);
}

/*
 * Sends a request to the firmware through the BCM2835 mailbox driver,
 * and synchronously waits for the reply.
 */
static int
rpi_firmware_transaction(struct rpi_firmware *fw, u32 chan, u32 data)
{
	u32 message = MBOX_MSG(chan, data);
	int ret;

	WARN_ON(data & 0xf);

	mutex_lock(&transaction_lock);
	reinit_completion(&fw->c);
	ret = mbox_send_message(fw->chan, &message);
	if (ret >= 0) {
		if (wait_for_completion_timeout(&fw->c, 3*HZ)) {
			ret = 0;
		} else {
			ret = -ETIMEDOUT;
		}
	} else {
		dev_err(fw->cl.dev, "mbox_send_message returned %d\n", ret);
	}
	mutex_unlock(&transaction_lock);

	return ret;
}

struct tags_s {u32 t; const char *s;};

static struct tags_s tagnames[] = {
{0, "RPI_FIRMWARE_PROPERTY_END"},
{0x00000001, "RPI_FIRMWARE_GET_FIRMWARE_REVISION"},
{0x00000002, "RPI_FIRMWARE_GET_FIRMWARE_VARIANT"},
{0x00000003, "RPI_FIRMWARE_GET_FIRMWARE_HASH"},

{0x00008010, "RPI_FIRMWARE_SET_CURSOR_INFO"},
{0x00008011, "RPI_FIRMWARE_SET_CURSOR_STATE"},

{0x00010001, "RPI_FIRMWARE_GET_BOARD_MODEL"},
{0x00010002, "RPI_FIRMWARE_GET_BOARD_REVISION"},
{0x00010003, "RPI_FIRMWARE_GET_BOARD_MAC_ADDRESS"},
{0x00010004, "RPI_FIRMWARE_GET_BOARD_SERIAL"},
{0x00010005, "RPI_FIRMWARE_GET_ARM_MEMORY"},
{0x00010006, "RPI_FIRMWARE_GET_VC_MEMORY"},
{0x00010007, "RPI_FIRMWARE_GET_CLOCKS"},
{0x00020001, "RPI_FIRMWARE_GET_POWER_STATE"},
{0x00020002, "RPI_FIRMWARE_GET_TIMING"},
{0x00028001, "RPI_FIRMWARE_SET_POWER_STATE"},
{0x00030001, "RPI_FIRMWARE_GET_CLOCK_STATE"},
{0x00030002, "RPI_FIRMWARE_GET_CLOCK_RATE"},
{0x00030003, "RPI_FIRMWARE_GET_VOLTAGE"},
{0x00030004, "RPI_FIRMWARE_GET_MAX_CLOCK_RATE"},
{0x00030005, "RPI_FIRMWARE_GET_MAX_VOLTAGE"},
{0x00030006, "RPI_FIRMWARE_GET_TEMPERATURE"},
{0x00030007, "RPI_FIRMWARE_GET_MIN_CLOCK_RATE"},
{0x00030008, "RPI_FIRMWARE_GET_MIN_VOLTAGE"},
{0x00030009, "RPI_FIRMWARE_GET_TURBO"},
{0x0003000a, "RPI_FIRMWARE_GET_MAX_TEMPERATURE"},
{0x0003000b, "RPI_FIRMWARE_GET_STC"},
{0x0003000c, "RPI_FIRMWARE_ALLOCATE_MEMORY"},
{0x0003000d, "RPI_FIRMWARE_LOCK_MEMORY"},
{0x0003000e, "RPI_FIRMWARE_UNLOCK_MEMORY"},
{0x0003000f, "RPI_FIRMWARE_RELEASE_MEMORY"},
{0x00030010, "RPI_FIRMWARE_EXECUTE_CODE"},
{0x00030011, "RPI_FIRMWARE_EXECUTE_QPU"},
{0x00030012, "RPI_FIRMWARE_SET_ENABLE_QPU"},
{0x00030014, "RPI_FIRMWARE_GET_DISPMANX_RESOURCE_MEM_HANDLE"},
{0x00030020, "RPI_FIRMWARE_GET_EDID_BLOCK"},
{0x00030021, "RPI_FIRMWARE_GET_CUSTOMER_OTP"},
{0x00030023, "RPI_FIRMWARE_GET_EDID_BLOCK_DISPLAY"},
{0x00030030, "RPI_FIRMWARE_GET_DOMAIN_STATE"},
{0x00030046, "RPI_FIRMWARE_GET_THROTTLED"},
{0x00030047, "RPI_FIRMWARE_GET_CLOCK_MEASURED"},
{0x00030048, "RPI_FIRMWARE_NOTIFY_REBOOT"},
{0x00038001, "RPI_FIRMWARE_SET_CLOCK_STATE"},
{0x00038002, "RPI_FIRMWARE_SET_CLOCK_RATE"},
{0x00038003, "RPI_FIRMWARE_SET_VOLTAGE"},
{0x00038009, "RPI_FIRMWARE_SET_TURBO"},
{0x00038021, "RPI_FIRMWARE_SET_CUSTOMER_OTP"},
{0x00038030, "RPI_FIRMWARE_SET_DOMAIN_STATE"},
{0x00030041, "RPI_FIRMWARE_GET_GPIO_STATE"},
{0x00038041, "RPI_FIRMWARE_SET_GPIO_STATE"},
{0x00038042, "RPI_FIRMWARE_SET_SDHOST_CLOCK"},
{0x00030043, "RPI_FIRMWARE_GET_GPIO_CONFIG"},
{0x00038043, "RPI_FIRMWARE_SET_GPIO_CONFIG"},
{0x00030045, "RPI_FIRMWARE_GET_PERIPH_REG"},
{0x00038045, "RPI_FIRMWARE_SET_PERIPH_REG"},
{0x00030049, "RPI_FIRMWARE_GET_POE_HAT_VAL"},
{0x00038049, "RPI_FIRMWARE_SET_POE_HAT_VAL"},
{0x00030050, "RPI_FIRMWARE_SET_POE_HAT_VAL_OLD"},
{0x00030058, "RPI_FIRMWARE_NOTIFY_XHCI_RESET"},
{0x00030064, "RPI_FIRMWARE_GET_REBOOT_FLAGS"},
{0x00038064, "RPI_FIRMWARE_SET_REBOOT_FLAGS"},
{0x00030066, "RPI_FIRMWARE_NOTIFY_DISPLAY_DONE"},

{0x00040001, "RPI_FIRMWARE_FRAMEBUFFER_ALLOCATE"},
{0x00040002, "RPI_FIRMWARE_FRAMEBUFFER_BLANK"},
{0x00040003, "RPI_FIRMWARE_FRAMEBUFFER_GET_PHYSICAL_WIDTH_HEIGHT"},
{0x00040004, "RPI_FIRMWARE_FRAMEBUFFER_GET_VIRTUAL_WIDTH_HEIGHT"},
{0x00040005, "RPI_FIRMWARE_FRAMEBUFFER_GET_DEPTH"},
{0x00040006, "RPI_FIRMWARE_FRAMEBUFFER_GET_PIXEL_ORDER"},
{0x00040007, "RPI_FIRMWARE_FRAMEBUFFER_GET_ALPHA_MODE"},
{0x00040008, "RPI_FIRMWARE_FRAMEBUFFER_GET_PITCH"},
{0x00040009, "RPI_FIRMWARE_FRAMEBUFFER_GET_VIRTUAL_OFFSET"},
{0x0004000a, "RPI_FIRMWARE_FRAMEBUFFER_GET_OVERSCAN"},
{0x0004000b, "RPI_FIRMWARE_FRAMEBUFFER_GET_PALETTE"},
{0x0004000c, "RPI_FIRMWARE_FRAMEBUFFER_GET_LAYER"},
{0x0004000d, "RPI_FIRMWARE_FRAMEBUFFER_GET_TRANSFORM"},
{0x0004000e, "RPI_FIRMWARE_FRAMEBUFFER_GET_VSYNC"},
{0x0004000f, "RPI_FIRMWARE_FRAMEBUFFER_GET_TOUCHBUF"},
{0x00040010, "RPI_FIRMWARE_FRAMEBUFFER_GET_GPIOVIRTBUF"},
{0x00048001, "RPI_FIRMWARE_FRAMEBUFFER_RELEASE"},
{0x00040016, "RPI_FIRMWARE_FRAMEBUFFER_GET_DISPLAY_ID"},
{0x00048013, "RPI_FIRMWARE_FRAMEBUFFER_SET_DISPLAY_NUM"},
{0x00040013, "RPI_FIRMWARE_FRAMEBUFFER_GET_NUM_DISPLAYS"},
{0x00040014, "RPI_FIRMWARE_FRAMEBUFFER_GET_DISPLAY_SETTINGS"},
{0x00044003, "RPI_FIRMWARE_FRAMEBUFFER_TEST_PHYSICAL_WIDTH_HEIGHT"},
{0x00044004, "RPI_FIRMWARE_FRAMEBUFFER_TEST_VIRTUAL_WIDTH_HEIGHT"},
{0x00044005, "RPI_FIRMWARE_FRAMEBUFFER_TEST_DEPTH"},
{0x00044006, "RPI_FIRMWARE_FRAMEBUFFER_TEST_PIXEL_ORDER"},
{0x00044007, "RPI_FIRMWARE_FRAMEBUFFER_TEST_ALPHA_MODE"},
{0x00044009, "RPI_FIRMWARE_FRAMEBUFFER_TEST_VIRTUAL_OFFSET"},
{0x0004400a, "RPI_FIRMWARE_FRAMEBUFFER_TEST_OVERSCAN"},
{0x0004400b, "RPI_FIRMWARE_FRAMEBUFFER_TEST_PALETTE"},
{0x0004400c, "RPI_FIRMWARE_FRAMEBUFFER_TEST_LAYER"},
{0x0004400d, "RPI_FIRMWARE_FRAMEBUFFER_TEST_TRANSFORM"},
{0x0004400e, "RPI_FIRMWARE_FRAMEBUFFER_TEST_VSYNC"},
{0x00048003, "RPI_FIRMWARE_FRAMEBUFFER_SET_PHYSICAL_WIDTH_HEIGHT"},
{0x00048004, "RPI_FIRMWARE_FRAMEBUFFER_SET_VIRTUAL_WIDTH_HEIGHT"},
{0x00048005, "RPI_FIRMWARE_FRAMEBUFFER_SET_DEPTH"},
{0x00048006, "RPI_FIRMWARE_FRAMEBUFFER_SET_PIXEL_ORDER"},
{0x00048007, "RPI_FIRMWARE_FRAMEBUFFER_SET_ALPHA_MODE"},
{0x00048008, "RPI_FIRMWARE_FRAMEBUFFER_SET_PITCH"},
{0x00048009, "RPI_FIRMWARE_FRAMEBUFFER_SET_VIRTUAL_OFFSET"},
{0x0004800a, "RPI_FIRMWARE_FRAMEBUFFER_SET_OVERSCAN"},
{0x0004800b, "RPI_FIRMWARE_FRAMEBUFFER_SET_PALETTE"},

{0x0004801f, "RPI_FIRMWARE_FRAMEBUFFER_SET_TOUCHBUF"},
{0x00048020, "RPI_FIRMWARE_FRAMEBUFFER_SET_GPIOVIRTBUF"},
{0x0004800e, "RPI_FIRMWARE_FRAMEBUFFER_SET_VSYNC"},
{0x0004800c, "RPI_FIRMWARE_FRAMEBUFFER_SET_LAYER"},
{0x0004800d, "RPI_FIRMWARE_FRAMEBUFFER_SET_TRANSFORM"},
{0x0004800f, "RPI_FIRMWARE_FRAMEBUFFER_SET_BACKLIGHT"},

{0x00048010, "RPI_FIRMWARE_VCHIQ_INIT"},

{0x00048015, "RPI_FIRMWARE_SET_PLANE"},
{0x00040017, "RPI_FIRMWARE_GET_DISPLAY_TIMING"},
{0x00048017, "RPI_FIRMWARE_SET_TIMING"},
{0x00040018, "RPI_FIRMWARE_GET_DISPLAY_CFG"},
{0x00048019, "RPI_FIRMWARE_SET_DISPLAY_POWER"},
{0x00050001, "RPI_FIRMWARE_GET_COMMAND_LINE"},
{0x00060001, "RPI_FIRMWARE_GET_DMA_CHANNELS"},
};

static struct tags_s domainnames[] = {
{0, "RPI_POWER_DOMAIN_I2C0"},
{1, "RPI_POWER_DOMAIN_I2C1"},
{2, "RPI_POWER_DOMAIN_I2C2"},
{3, "RPI_POWER_DOMAIN_VIDEO_SCALER"},
{4, "RPI_POWER_DOMAIN_VPU1"},
{5, "RPI_POWER_DOMAIN_HDMI"},
{6, "RPI_POWER_DOMAIN_USB"},
{7, "RPI_POWER_DOMAIN_VEC"},
{8, "RPI_POWER_DOMAIN_JPEG"},
{9, "RPI_POWER_DOMAIN_H264"},
{10, "RPI_POWER_DOMAIN_V3D"},
{11, "RPI_POWER_DOMAIN_ISP"},
{12, "RPI_POWER_DOMAIN_UNICAM0"},
{13, "RPI_POWER_DOMAIN_UNICAM1"},
{14, "RPI_POWER_DOMAIN_CCP2RX"},
{15, "RPI_POWER_DOMAIN_CSI2"},
{16, "RPI_POWER_DOMAIN_CPI"},
{17, "RPI_POWER_DOMAIN_DSI0"},
{18, "RPI_POWER_DOMAIN_DSI1"},
{19, "RPI_POWER_DOMAIN_TRANSPOSER"},
{20, "RPI_POWER_DOMAIN_CCP2TX"},
{21, "RPI_POWER_DOMAIN_CDP"},
{22, "RPI_POWER_DOMAIN_ARM"},
};

static size_t get_tagname(char *tn, u32 t, struct tags_s *tags, size_t size)
{
	int i;
	for (i=0; i<size; i++)
		if (tags[i].t == t) {
			strcpy(tn, tags[i].s);
			return strlen(tn);
		}
	sprintf(tn, "%x", t);
	return strlen(tn);
}

static char *print_buf(u32 *buf, size_t tag_size, char *u, size_t u_size)
{
	char *p = u;
	int i;
	p += get_tagname(p, buf[2], tagnames, ARRAY_SIZE(tagnames));

	if ((buf[2] == RPI_FIRMWARE_GET_DOMAIN_STATE || buf[2] == RPI_FIRMWARE_SET_DOMAIN_STATE) && buf[5])
	{
		p += sprintf(p, " ");
		p += get_tagname(p, buf[5]-1, domainnames, ARRAY_SIZE(domainnames));
		if (buf[2] == RPI_FIRMWARE_SET_DOMAIN_STATE)
			p += sprintf(p, " %x", buf[6]);
		return u;
	}
	for (i = 0; i<(tag_size>>2)+2; i++)
		if (p + 10 < u + u_size)
			p += sprintf(p, " %x", buf[i]);
	return u;
}

/**
 * rpi_firmware_property_list - Submit firmware property list
 * @fw:		Pointer to firmware structure from rpi_firmware_get().
 * @data:	Buffer holding tags.
 * @tag_size:	Size of tags buffer.
 *
 * Submits a set of concatenated tags to the VPU firmware through the
 * mailbox property interface.
 *
 * The buffer header and the ending tag are added by this function and
 * don't need to be supplied, just the actual tags for your operation.
 * See struct rpi_firmware_property_tag_header for the per-tag
 * structure.
 */
int rpi_firmware_property_list(struct rpi_firmware *fw,
			       void *data, size_t tag_size)
{
	size_t size = tag_size + 12;
	u32 *buf;
	dma_addr_t bus_addr;
	int ret;
	char u[1024];

	/* Packets are processed a dword at a time. */
	if (size & 3)
		return -EINVAL;

	buf = dma_alloc_coherent(fw->chan->mbox->dev, PAGE_ALIGN(size),
				 &bus_addr, GFP_ATOMIC);
	if (!buf)
		return -ENOMEM;

	/* The firmware will error out without parsing in this case. */
	WARN_ON(size >= 1024 * 1024);

	buf[0] = size;
	buf[1] = RPI_FIRMWARE_STATUS_REQUEST;
	memcpy(&buf[2], data, tag_size);
	buf[size / 4 - 1] = RPI_FIRMWARE_PROPERTY_END;
	wmb();

if (buf[2] != RPI_FIRMWARE_GET_THROTTLED) printk("%s: %s\n", __func__, print_buf(buf, tag_size, u, sizeof u));

	ret = rpi_firmware_transaction(fw, MBOX_CHAN_PROPERTY, bus_addr);

if (buf[2] != RPI_FIRMWARE_GET_THROTTLED) printk("%s: %s ret=%d\n", __func__, print_buf(buf, tag_size, u, sizeof u), ret);

	rmb();
	memcpy(data, &buf[2], tag_size);
	if (ret == 0 && buf[1] != RPI_FIRMWARE_STATUS_SUCCESS) {
		/*
		 * The tag name here might not be the one causing the
		 * error, if there were multiple tags in the request.
		 * But single-tag is the most common, so go with it.
		 */
		dev_err(fw->cl.dev, "Request 0x%08x returned status 0x%08x\n",
			buf[2], buf[1]);
		ret = -EINVAL;
	} else if (ret == -ETIMEDOUT) {
		WARN_ONCE(1, "Firmware transaction 0x%08x timeout", buf[2]);
	}

	dma_free_coherent(fw->chan->mbox->dev, PAGE_ALIGN(size), buf, bus_addr);

	return ret;
}
EXPORT_SYMBOL_GPL(rpi_firmware_property_list);

/**
 * rpi_firmware_property - Submit single firmware property
 * @fw:		Pointer to firmware structure from rpi_firmware_get().
 * @tag:	One of enum_mbox_property_tag.
 * @tag_data:	Tag data buffer.
 * @buf_size:	Buffer size.
 *
 * Submits a single tag to the VPU firmware through the mailbox
 * property interface.
 *
 * This is a convenience wrapper around
 * rpi_firmware_property_list() to avoid some of the
 * boilerplate in property calls.
 */
int rpi_firmware_property(struct rpi_firmware *fw,
			  u32 tag, void *tag_data, size_t buf_size)
{
	struct rpi_firmware_property_tag_header *header;
	int ret;

	/* Some mailboxes can use over 1k bytes. Rather than checking
	 * size and using stack or kmalloc depending on requirements,
	 * just use kmalloc. Mailboxes don't get called enough to worry
	 * too much about the time taken in the allocation.
	 */
	void *data = kmalloc(sizeof(*header) + buf_size, GFP_KERNEL);

	if (!data)
		return -ENOMEM;

	header = data;
	header->tag = tag;
	header->buf_size = buf_size;
	header->req_resp_size = 0;
	memcpy(data + sizeof(*header), tag_data, buf_size);

	ret = rpi_firmware_property_list(fw, data, buf_size + sizeof(*header));

	memcpy(tag_data, data + sizeof(*header), buf_size);

	kfree(data);

	return ret;
}
EXPORT_SYMBOL_GPL(rpi_firmware_property);

static int rpi_firmware_notify_reboot(struct notifier_block *nb,
				      unsigned long action,
				      void *data)
{
	struct rpi_firmware *fw;
	struct platform_device *pdev = g_pdev;
	u32 reboot_flags = 0;

	if (!pdev)
		return 0;

	fw = platform_get_drvdata(pdev);
	if (!fw)
		return 0;

	// The partition id is the first parameter followed by zero or
	// more flags separated by spaces indicating the reason for the reboot.
	//
	// 'tryboot': Sets a one-shot flag which is cleared upon reboot and
	//            causes the tryboot.txt to be loaded instead of config.txt
	//            by the bootloader and the start.elf firmware.
	//
	//            This is intended to allow automatic fallback to a known
	//            good image if an OS/FW upgrade fails.
	//
	// N.B. The firmware mechanism for storing reboot flags may vary
	// on different Raspberry Pi models.
	if (data && strstr(data, " tryboot"))
		reboot_flags |= 0x1;

	// The mailbox might have been called earlier, directly via vcmailbox
	// so only overwrite if reboot flags are passed to the reboot command.
	if (reboot_flags)
		(void)rpi_firmware_property(fw, RPI_FIRMWARE_SET_REBOOT_FLAGS,
				&reboot_flags, sizeof(reboot_flags));

	(void)rpi_firmware_property(fw, RPI_FIRMWARE_NOTIFY_REBOOT, NULL, 0);

	return 0;
}

static void
rpi_firmware_print_firmware_revision(struct rpi_firmware *fw)
{
	time64_t date_and_time;
	u32 packet;
	static const char * const variant_strs[] = {
		"unknown",
		"start",
		"start_x",
		"start_db",
		"start_cd",
	};
	const char *variant_str = "cmd unsupported";
	u32 variant;
	int ret = rpi_firmware_property(fw,
					RPI_FIRMWARE_GET_FIRMWARE_REVISION,
					&packet, sizeof(packet));

	if (ret)
		return;

	/* This is not compatible with y2038 */
	date_and_time = packet;

	ret = rpi_firmware_property(fw, RPI_FIRMWARE_GET_FIRMWARE_VARIANT,
				    &variant, sizeof(variant));

	if (!ret) {
		if (variant >= ARRAY_SIZE(variant_strs))
			variant = 0;
		variant_str = variant_strs[variant];
	}

	dev_info(fw->cl.dev,
		 "Attached to firmware from %ptT, variant %s\n",
		 &date_and_time, variant_str);
}

static void
rpi_firmware_print_firmware_hash(struct rpi_firmware *fw)
{
	u32 hash[5];
	int ret = rpi_firmware_property(fw,
					RPI_FIRMWARE_GET_FIRMWARE_HASH,
					hash, sizeof(hash));

	if (ret)
		return;

	dev_info(fw->cl.dev,
		 "Firmware hash is %08x%08x%08x%08x%08x\n",
		 hash[0], hash[1], hash[2], hash[3], hash[4]);
}

static void
rpi_register_hwmon_driver(struct device *dev, struct rpi_firmware *fw)
{
	u32 packet;
	int ret = rpi_firmware_property(fw, RPI_FIRMWARE_GET_THROTTLED,
					&packet, sizeof(packet));

	if (ret)
		return;

	rpi_hwmon = platform_device_register_data(dev, "raspberrypi-hwmon",
						  -1, NULL, 0);
}

static void rpi_register_clk_driver(struct device *dev)
{
	struct device_node *firmware;

	/*
	 * Earlier DTs don't have a node for the firmware clocks but
	 * rely on us creating a platform device by hand. If we do
	 * have a node for the firmware clocks, just bail out here.
	 */
	firmware = of_get_compatible_child(dev->of_node,
					   "raspberrypi,firmware-clocks");
	if (firmware) {
		of_node_put(firmware);
		return;
	}

	rpi_clk = platform_device_register_data(dev, "raspberrypi-clk",
						-1, NULL, 0);
}

unsigned int rpi_firmware_clk_get_max_rate(struct rpi_firmware *fw, unsigned int id)
{
	struct rpi_firmware_clk_rate_request msg =
		RPI_FIRMWARE_CLK_RATE_REQUEST(id);
	int ret;

	ret = rpi_firmware_property(fw, RPI_FIRMWARE_GET_MAX_CLOCK_RATE,
				    &msg, sizeof(msg));
	if (ret)
		/*
		 * If our firmware doesn't support that operation, or fails, we
		 * assume the maximum clock rate is absolute maximum we can
		 * store over our type.
		 */
		 return UINT_MAX;

	return le32_to_cpu(msg.rate);
}
EXPORT_SYMBOL_GPL(rpi_firmware_clk_get_max_rate);

static void rpi_firmware_delete(struct kref *kref)
{
	struct rpi_firmware *fw = container_of(kref, struct rpi_firmware,
					       consumers);

	mbox_free_channel(fw->chan);
	kfree(fw);
}

void rpi_firmware_put(struct rpi_firmware *fw)
{
	kref_put(&fw->consumers, rpi_firmware_delete);
}
EXPORT_SYMBOL_GPL(rpi_firmware_put);

static void devm_rpi_firmware_put(void *data)
{
	struct rpi_firmware *fw = data;

	rpi_firmware_put(fw);
}

static int rpi_firmware_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rpi_firmware *fw;

	/*
	 * Memory will be freed by rpi_firmware_delete() once all users have
	 * released their firmware handles. Don't use devm_kzalloc() here.
	 */
	fw = kzalloc(sizeof(*fw), GFP_KERNEL);
	if (!fw)
		return -ENOMEM;

	fw->cl.dev = dev;
	fw->cl.rx_callback = response_callback;
	fw->cl.tx_block = true;

	fw->chan = mbox_request_channel(&fw->cl, 0);
	if (IS_ERR(fw->chan)) {
		int ret = PTR_ERR(fw->chan);
		kfree(fw);
		return dev_err_probe(dev, ret, "Failed to get mbox channel\n");
	}

	init_completion(&fw->c);
	kref_init(&fw->consumers);

	platform_set_drvdata(pdev, fw);
	g_pdev = pdev;

	rpi_firmware_print_firmware_revision(fw);
	rpi_firmware_print_firmware_hash(fw);
	rpi_register_hwmon_driver(dev, fw);
	rpi_register_clk_driver(dev);

	return 0;
}

static void rpi_firmware_shutdown(struct platform_device *pdev)
{
	struct rpi_firmware *fw = platform_get_drvdata(pdev);

	if (!fw)
		return;

	rpi_firmware_property(fw, RPI_FIRMWARE_NOTIFY_REBOOT, NULL, 0);
}

static void rpi_firmware_remove(struct platform_device *pdev)
{
	struct rpi_firmware *fw = platform_get_drvdata(pdev);

	platform_device_unregister(rpi_hwmon);
	rpi_hwmon = NULL;
	platform_device_unregister(rpi_clk);
	rpi_clk = NULL;

	rpi_firmware_put(fw);
	g_pdev = NULL;
}

static const struct of_device_id rpi_firmware_of_match[] = {
	{ .compatible = "raspberrypi,bcm2835-firmware", },
	{},
};
MODULE_DEVICE_TABLE(of, rpi_firmware_of_match);

struct device_node *rpi_firmware_find_node(void)
{
	return of_find_matching_node(NULL, rpi_firmware_of_match);
}
EXPORT_SYMBOL_GPL(rpi_firmware_find_node);

/**
 * rpi_firmware_get - Get pointer to rpi_firmware structure.
 * @firmware_node:    Pointer to the firmware Device Tree node.
 *
 * The reference to rpi_firmware has to be released with rpi_firmware_put().
 *
 * Returns NULL is the firmware device is not ready.
 */
struct rpi_firmware *rpi_firmware_get(struct device_node *firmware_node)
{
	struct platform_device *pdev = of_find_device_by_node(firmware_node);
	struct rpi_firmware *fw;

	if (!pdev)
		return NULL;

	fw = platform_get_drvdata(pdev);
	if (!fw)
		goto err_put_device;

	if (!kref_get_unless_zero(&fw->consumers))
		goto err_put_device;

	put_device(&pdev->dev);

	return fw;

err_put_device:
	put_device(&pdev->dev);
	return NULL;
}
EXPORT_SYMBOL_GPL(rpi_firmware_get);

/**
 * devm_rpi_firmware_get - Get pointer to rpi_firmware structure.
 * @dev:              The firmware device structure
 * @firmware_node:    Pointer to the firmware Device Tree node.
 *
 * Returns NULL is the firmware device is not ready.
 */
struct rpi_firmware *devm_rpi_firmware_get(struct device *dev,
					   struct device_node *firmware_node)
{
	struct rpi_firmware *fw;

	fw = rpi_firmware_get(firmware_node);
	if (!fw)
		return NULL;

	if (devm_add_action_or_reset(dev, devm_rpi_firmware_put, fw))
		return NULL;

	return fw;
}
EXPORT_SYMBOL_GPL(devm_rpi_firmware_get);

static struct platform_driver rpi_firmware_driver = {
	.driver = {
		.name = "raspberrypi-firmware",
		.of_match_table = rpi_firmware_of_match,
	},
	.probe		= rpi_firmware_probe,
	.shutdown	= rpi_firmware_shutdown,
	.remove		= rpi_firmware_remove,
};

static struct notifier_block rpi_firmware_reboot_notifier = {
	.notifier_call = rpi_firmware_notify_reboot,
};

static int __init rpi_firmware_init(void)
{
	int ret = register_reboot_notifier(&rpi_firmware_reboot_notifier);
	if (ret)
		goto out1;
	ret = platform_driver_register(&rpi_firmware_driver);
	if (ret)
		goto out2;

	return 0;

out2:
	unregister_reboot_notifier(&rpi_firmware_reboot_notifier);
out1:
	return ret;
}
core_initcall(rpi_firmware_init);

static void __init rpi_firmware_exit(void)
{
	platform_driver_unregister(&rpi_firmware_driver);
	unregister_reboot_notifier(&rpi_firmware_reboot_notifier);
}
module_exit(rpi_firmware_exit);

MODULE_AUTHOR("Eric Anholt <eric@anholt.net>");
MODULE_DESCRIPTION("Raspberry Pi firmware driver");
MODULE_LICENSE("GPL v2");
