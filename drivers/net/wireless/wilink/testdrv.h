/***************************************************************************
**+----------------------------------------------------------------------+**
**|                                ****                                  |**
**|                                ****                                  |**
**|                                ******o***                            |**
**|                          ********_///_****                           |**
**|                           ***** /_//_/ ****                          |**
**|                            ** ** (__/ ****                           |**
**|                                *********                             |**
**|                                 ****                                 |**
**|                                  ***                                 |**
**|                                                                      |**
**|     Copyright (c) 1998 - 2009 Texas Instruments Incorporated         |**
**|                        ALL RIGHTS RESERVED                           |**
**|                                                                      |**
**| Permission is hereby granted to licensees of Texas Instruments       |**
**| Incorporated (TI) products to use this computer program for the sole |**
**| purpose of implementing a licensee product based on TI products.     |**
**| No other rights to reproduce, use, or disseminate this computer      |**
**| program, whether in part or in whole, are granted.                   |**
**|                                                                      |**
**| TI makes no representation or warranties with respect to the         |**
**| performance of this computer program, and specifically disclaims     |**
**| any responsibility for any damages, special or consequential,        |**
**| connected with the use of this program.                              |**
**|                                                                      |**
**+----------------------------------------------------------------------+**
***************************************************************************/

#ifndef _MMC_TEST_H_
#define _MMC_TEST_H_

#define TESTDRV_MODULE_NAME "sdio_test"

#ifdef  TESTDRV_CONFIG_MMC_TEST_DEBUG
#define DBG(x...)	printk(x)
#else
#define DBG(x...)	do { } while (0)
#endif

#define TESTDRV_GPIO_OUTPUT		                0

#if 0
#include <asm/arch/hardware.h>
#endif
#include <mach/hardware.h>

#define TXN_FUNC_ID_CTRL         0
#define TXN_FUNC_ID_BT           1
#define TXN_FUNC_ID_WLAN         2

#define TNETW_IRQ                       (OMAP_GPIO_IRQ(IRQ_GPIO))
#define TIWLAN_IRQ_POLL_INTERVAL	    HZ/100
#define HZ_IN_MSEC						HZ/1000
#define TIWLAN_IRQ_POLL_INTERVAL_MS		TIWLAN_IRQ_POLL_INTERVAL/HZ_IN_MSEC

#define TESTDRV_SDIO_FUNC1_OFFSET           	0x1FFC0  /* address of the partition table */

#define SDIO_TEST_FIRST_VALID_DMA_ADDR			(0x00000008)	/* used for escaping addressing invalid DMA Addresses */
#define SDIO_TEST_NO_OF_TRANSACTIONS			(3)

#define TESTDRV_512_SDIO_BLOCK					(512)
#define TESTDRV_MAX_SDIO_BLOCK					(TESTDRV_512_SDIO_BLOCK /* - 4 */)

#define TESTDRV_MAX_PART_SIZE  					0x1F000 	/* 124k	*/	 

#define TESTDRV_CODE_RAM_SIZE  					0x30000		/* 192K	*/
#define TESTDRV_DATA_RAM_SIZE 					0xC000		/* 48K 	*/
#define TESTDRV_PACKET_RAM_SIZE 				0xD000		/* 52K 	*/

#define TESTDRV_REG_PART_START_ADDR 			0x300000
#define TESTDRV_REG_DOWNLOAD_PART_SIZE 			0x8800 		/* 44k	*/ 	
#define TESTDRV_REG_WORKING_PART_SIZE 			0xB000 		/* 44k	*/ 	

#define TESTDRV_CODE_RAM_PART_START_ADDR 		0		
#define TESTDRV_DATA_RAM_PART_START_ADDR 		0x20000000
#define TESTDRV_PACKET_RAM_PART_START_ADDR 		0x40000

/* Partition Size Left for Memory */
#define TESTDRV_MEM_WORKING_PART_SIZE  			(TESTDRV_MAX_PART_SIZE - TESTDRV_REG_WORKING_PART_SIZE) 	 
#define TESTDRV_MEM_DOWNLOAD_PART_SIZE  		(TESTDRV_MAX_PART_SIZE - TESTDRV_REG_DOWNLOAD_PART_SIZE) 	 

#define TESTDRV_TESTING_DATA_LENGTH 512
#endif /* _MMC_TEST_H_ */

