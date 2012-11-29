# Build SDIO and TESTSDIO modules 
#EXTRA_CFLAGS += -D HOST_PLATFORM_OMAP3xxx -D __BYTE_ORDER_LITTLE_ENDIAN -D TIWLAN_BMTRACE -D HOST_COMPILE -D FW_RUNNING_AS_STA -D TNETW1273 -D __LINUX__ -D TI_DBG -D REPORT_LOG
EXTRA_CFLAGS += -D HOST_PLATFORM_OMAP3xxx -D __BYTE_ORDER_LITTLE_ENDIAN -D HOST_COMPILE -D FW_RUNNING_AS_STA -D TNETW1273 -D __LINUX__ -D TI_DBG -D REPORT_LOG

EXTRA_CFLAGS += -DOMAP3_EVM -DCONFIG_SDIO_ADDRESS_MAPPING_BY_APPLICATION

obj-m	:= sdio.o
sdio-objs := SdioDrv.o
# Build TIWLAN driver module
obj-m	+= tiwlan_drv.o
tiwlan_drv-objs := \
		DrvMain.o EvHandler.o CmdHndlr.o \
		CmdDispatcher.o txCtrl.o txCtrlServ.o txCtrlParams.o \
		txPort.o txDataQueue.o TxDataClsfr.o txMgmtQueue.o \
		Ctrl.o GeneralUtil.o TrafficMonitor.o rx.o \
		PowerMgr.o PowerMgrKeepAlive.o PowerMgrDbgPrint.o \
		healthMonitor.o siteMgr.o StaCap.o siteHash.o \
		systemConfig.o templates.o mlmeBuilder.o mlmeParser.o \
		mlmeSm.o assocSM.o authSm.o openAuthSm.o sharedKeyAuthSm.o \
		scr.o ScanCncn.o ScanCncnSm.o ScanCncnSmSpecific.o \
		ScanCncnOsSm.o ScanCncnApp.o scanResultTable.o \
		currBss.o qosMngr.o trafficAdmControl.o SwitchChannel.o \
		measurementMgr.o measurementMgrSM.o spectrumMngmntMgr.o \
		requestHandler.o SoftGemini.o regulatoryDomain.o conn.o \
		connIbss.o connInfra.o sme.o smeSm.o smeSelect.o \
		rsn.o mainSecSm.o mainSecNull.o mainSecKeysOnly.o admCtrl.o \
		admCtrlWep.o admCtrlNone.o admCtrlWpa.o admCtrlWpa2.o \
		apConn.o keyDerive.o keyDeriveWep.o keyParser.o \
		keyDeriveAes.o keyDeriveTkip.o broadcastKeySM.o \
		unicastKeySM.o broadcastKey802_1x.o unicastKey802_1x.o \
		broadcastKeyNone.o unicastKeyNone.o keyParserExternal.o \
		keyParserWep.o mainKeysSm.o externalSec.o scanMngr.o \
		roamingMngr.o roamingMngr_manualSM.o roamingMngr_autoSM.o \
		MacServices.o ScanSrv.o ScanSrvSM.o PowerSrv.o PowerSrvSM.o \
		MeasurementSrv.o MeasurementSrvSM.o measurementSrvDbgPrint.o \
		TWDriver.o TWDriverTx.o TWDriverCtrl.o TWDriverRadio.o \
		CmdQueue.o CmdBld.o CmdBldCfg.o CmdBldCmd.o CmdBldItr.o \
		CmdBldCfgIE.o CmdBldCmdIE.o CmdBldItrIE.o txHwQueue.o \
		txCtrlBlk.o RxQueue.o CmdMBox.o txResult.o txXfer.o \
		FwEvent.o RxXfer.o eventMbox.o HwInit.o fwDebug.o \
		TwIf.o TxnQueue.o SdioBusDrv.o fsm.o GenSM.o report.o \
		queue.o context.o timer.o rate.o freq.o debug.o \
		measurementDbg.o connDebug.o HealthMonitorDbg.o scanCncnDbg.o \
		SoftGeminiDbg.o PowerMgrDebug.o ctrlDbg.o TWD_Debug.o rsnDbg.o \
		siteMgrDebug.o roamingMgrDebug.o fwdriverdebug.o ScanMngrDbg.o \
		MibDbg.o qosMngrDbg.o TwIfDebug.o scrDbg.o smeDebug.o TxDbg.o \
		RxDbg.o osRgstry.o WlanDrvIf.o host_platform.o osapi.o \
		osmemapi.o osRgstry_parser.o ipc_k.o WlanDrvWext.o \
		CmdInterpretWext.o RxBuf.o SdioAdapter.o
