/*
 * host_platform.c
 *
 * Copyright(c) 1998 - 2009 Texas Instruments. All rights reserved.      
 * All rights reserved.                                                  
 *                                                                       
 * Redistribution and use in source and binary forms, with or without    
 * modification, are permitted provided that the following conditions    
 * are met:                                                              
 *                                                                       
 *  * Redistributions of source code must retain the above copyright     
 *    notice, this list of conditions and the following disclaimer.      
 *  * Redistributions in binary form must reproduce the above copyright  
 *    notice, this list of conditions and the following disclaimer in    
 *    the documentation and/or other materials provided with the         
 *    distribution.                                                      
 *  * Neither the name Texas Instruments nor the names of its            
 *    contributors may be used to endorse or promote products derived    
 *    from this software without specific prior written permission.      
 *                                                                       
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS   
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT     
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR 
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT      
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY 
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT   
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "tidef.h"
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/wifi_tiwlan.h>
#include <linux/completion.h>

#include "host_platform.h"
#include "ioctl_init.h"
#include "WlanDrvIf.h"
#include "Device1273.h"
#include "SdioDrv.h"


#define OS_API_MEM_ADDR  	0x0000000
#define OS_API_REG_ADDR  	0x0300000

#define SDIO_ATTEMPT_LONGER_DELAY_LINUX  150

static DECLARE_COMPLETION(sdio_ready);
static DECLARE_COMPLETION(wifi_ready);

static struct wifi_platform_data *wifi_control_data = NULL;
static struct resource *wifi_irqres = NULL;

static int wifi_probe(struct platform_device *pdev)
{
	struct wifi_platform_data *wifi_ctrl = (struct wifi_platform_data *)(pdev->dev.platform_data);

	wifi_irqres = platform_get_resource_byname(pdev, IORESOURCE_IRQ, "device_wifi_irq");
	if (wifi_irqres) {
		printk(KERN_INFO "%s: got wlan irq %lu\n", __func__, (unsigned long)(wifi_irqres->start));
		printk(KERN_INFO "%s: got wlan irq trigger %s flag\n", __func__, 
				wifi_irqres->flags & IRQF_TRIGGER_FALLING ? "falling" : "unknown");
	}

	wifi_control_data = wifi_ctrl;

	complete(&wifi_ready);

	return 0;
}

static int wifi_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver wifi_device = {
	.probe          = wifi_probe,
	.remove         = wifi_remove,
	.suspend        = NULL,
	.resume         = NULL,
	.driver         = {
		.name   = "device_wifi",
	},
};

static int wifi_add_dev(void)
{
	return platform_driver_register(&wifi_device);
}

static void wifi_del_dev(void)
{
	platform_driver_unregister(&wifi_device);
}

static int unplug_device(int delay)
{
	/* In case hPlatform_DevicePowerOff is called before hPlatform_DevicePowerOn
	 * had the chance to wait for sdio_ready, we need to reinit it */
	INIT_COMPLETION(sdio_ready);

	sdioDrv_ReleaseHost(SDIO_WLAN_FUNC);

	if(wifi_control_data) {
		if(wifi_control_data->set_carddetect)
			wifi_control_data->set_carddetect(0);
		if(wifi_control_data->set_reset)
			wifi_control_data->set_reset(1);
		if(wifi_control_data->set_power)
			wifi_control_data->set_power(0);
	}

	mdelay(delay);

	return 0;
}
int hPlatform_DevicePowerOff (void)
{
	return unplug_device(10);
}

int hPlatform_DevicePowerOffSetLongerDelay(void)
{
	return unplug_device(SDIO_ATTEMPT_LONGER_DELAY_LINUX);
}

int hPlatform_DevicePowerOn (void)
{
	unsigned long timeleft;

	if(!wifi_control_data || !wifi_control_data->set_power)
		return -1;

	wifi_control_data->set_power(1);
	/* New Power Up Sequence */
	mdelay(15);
	wifi_control_data->set_power(0);
	mdelay(1);
	wifi_control_data->set_power(1);
	/* Should not be changed, 50 msec cause failures */
	mdelay(70);

	if(wifi_control_data->set_reset)
		wifi_control_data->set_reset(0);
	if(wifi_control_data->set_carddetect)
		wifi_control_data->set_carddetect(1);

	/* let the mmc core finish enumeration + initialization before we continue */
	timeleft = wait_for_completion_interruptible_timeout(&sdio_ready, msecs_to_jiffies(5000));
	if (!timeleft) {
		printk(KERN_ERR "%s: timeout waiting for sdio init\n", __func__);
		return -1;
	}

	sdioDrv_ClaimHost(SDIO_WLAN_FUNC);

	return 0;
}

static void hPlatform_sdio_ready(void)
{
	complete(&sdio_ready);
}

int hPlatform_Wlan_Hardware_Init(void *tnet_drv)
{
	TWlanDrvIfObj *drv = tnet_drv;
	unsigned long timeleft;

	sdioDrv_Register_Notification(hPlatform_sdio_ready);

	wifi_add_dev();

	/* make sure wifi device finished initialization before we move on */
	timeleft = wait_for_completion_interruptible_timeout(&wifi_ready, msecs_to_jiffies(5000));
	if (!timeleft) {
		printk(KERN_ERR "%s: timeout waiting for wifi device init\n", __func__);
		return -1;
	}

	if (!wifi_irqres)
		return -1;

	drv->irq = wifi_irqres->start;
	drv->irq_flags = wifi_irqres->flags & IRQF_TRIGGER_MASK;

	return 0;
}

int hPlatform_initInterrupt(void *tnet_drv, void* handle_add) 
{
	TWlanDrvIfObj *drv = tnet_drv;
	int rc;

	if (drv->irq == 0 || handle_add == NULL) {
		print_err("hPlatform_initInterrupt() bad param drv->irq=%d handle_add=0x%x !!!\n",drv->irq,(int)handle_add);
		return -EINVAL;
	}

	rc = request_irq(drv->irq, handle_add, drv->irq_flags, drv->netdev->name, drv);
	if (rc) {
		print_err("TIWLAN: Failed to register interrupt handler\n");
		return rc;
	}

	set_irq_wake(drv->irq, 1);	
	return rc;
}

void hPlatform_freeInterrupt(void *tnet_drv)
{
	TWlanDrvIfObj *drv = tnet_drv;

//	set_irq_wake(drv->irq, 0);
	free_irq(drv->irq, drv);
}

void *hPlatform_hwGetRegistersAddr(TI_HANDLE OsContext)
{
	return (void *)OS_API_REG_ADDR;
}

void *hPlatform_hwGetMemoryAddr(TI_HANDLE OsContext)
{
	return (void *)OS_API_MEM_ADDR;
}

void hPlatform_Wlan_Hardware_DeInit(void)
{
	wifi_del_dev();
}

