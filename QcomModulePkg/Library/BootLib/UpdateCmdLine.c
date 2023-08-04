/** @file UpdateCmdLine.c
 *
 * Copyright (c) 2009, Google Inc.
 * All rights reserved.
 *
 * Copyright (c) 2009-2021, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of The Linux Foundation nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **/
/*
  * Changes from Qualcomm Innovation Center are provided under the following
  * license:
  * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted (subject to the limitations in the disclaimer
  * below) provided that the following conditions are met:
  *  * Redistributions of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  *  * Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided ?with the distribution.
  *  * Neither the name of Qualcomm Innovation Center, Inc. nor the names of its
  *     contributors may be used to endorse or promote products derived from this
  *     software without specific prior written permission.
  *
  * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED
  * BY THIS LICENSE.
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
  * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
  * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
  * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
  * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
  * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
  * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
  * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  */


#include <Library/BaseLib.h>
#include <Library/BootLinux.h>
#include <Library/PartitionTableUpdate.h>
#include <Library/PrintLib.h>
#include <Library/FdtRw.h>
#include <LinuxLoaderLib.h>
#include <Protocol/EFICardInfo.h>
#include <Protocol/EFIChargerEx.h>
#include <Protocol/EFIChipInfoTypes.h>
#include <Protocol/EFIPmicPon.h>
#include <Protocol/Print2.h>
#include <Library/ShutdownServices.h>
#include "AutoGen.h"
#include <DeviceInfo.h>
#include "UpdateCmdLine.h"
#include "Recovery.h"
#include "LECmdLine.h"
#include "EarlyEthernet.h"

#ifdef ASUS_BUILD
#include "abl.h"
#include "LinuxLoaderLib.h"
#include <Protocol/EFIPmicGpio.h>
#include <Protocol/EFITlmm.h>

/***** ASUS_CMDLINE *****/
// +++ ASUS_BSP : add for ftm mode
extern char cmd_enable_adb_mode[64];
extern char cmd_enable_adb_prop[64];
extern char cmd_ftm_mode[64];
extern char cmd_ftm_mode_prop[64];
extern char cmd_selinux[64];
extern char cmd_selinux_prop[64];
//+++ ASUS_BSP : add for ASUS ID value
extern char cmd_asus_hwid[64];

extern char cmd_soc_id[64];
extern char cmd_prj_id[64];
extern char cmd_stage_id[64];
extern char cmd_sku_id[64];
extern char cmd_rf_id[64];
extern char cmd_fp_id[64];
extern char cmd_ddr_id[64];
extern char cmd_feature_id[64];
extern char cmd_jtag_id[64];
extern char cmd_pcb_id[64];
extern char cmd_nfc_id[64];
extern char cmd_lgf_con_id[64];
extern char cmd_fc_id[64];
extern char cmd_upper_id[64];
extern char cmd_sub_id[64];
extern char cmd_valid_image[64];
// +++ ASUS_BPS : add for read country code
extern char cmd_country_code[64];
extern char cmd_product_name[64];
// +++ ASUS_BSP : add for boot count
extern char cmd_boot_count[64];
// +++ ASUS_BSP : check if have rawdump partition or not
extern char cmd_rawdump_en[32];
// +++ ASUS_BSP : add for reboot reason
extern char cmd_asus_info[64];
extern char cmd_authorized_prop[64];
extern char cmd_unlock[16];
// +++ ASUS_BSP : add for WaterMask unlock
extern char cmd_watermask_unlock[16];
extern char cmd_watermask_unlock_prop[64];
extern char cmd_update_cmdline[64];
extern char cmd_cpuid_hash[64];
extern char cmd_toolid[64];
// +++ ASUS_BSP : add for fuse blow
extern char cmd_fuse_Info[16];
extern char cmd_fuse_prop[64];
// +++ ASUS_BSP : add for logcat-asdf sevices
extern char cmd_enable_logcat_asdf[64];
// +++ ASUS_BSP : add for check fuse with no rpmb
extern char cmd_fuse_no_rpmb_Info[16];
extern char cmd_fuse_no_rpmb_prop[64];
// +++ ASUS_BSP : add for check factory crc
extern char cmd_factory_crc[64];
extern char cmd_retry_count[64];
// +++ ASUS_BSP : add for ddr info
//extern char cmd_ddr_manufacturer[64];
//extern char cmd_ddr_device_type[64];
// +++ ASUS_BSP : add for panel uid
extern char cmd_unique_id[64];
extern char cmd_lgf_id[64];
extern char cmd_uart_status[64];
// +++ ASUS_BSP : add for NFC check hardware sku
extern char cmd_hardware_sku_prop[64];
extern BOOLEAN adb_enter_shipping_mode;

STATIC CONST CHAR8 *RecoveryMode = " androidboot.mode=recovery";
STATIC CONST CHAR8 *ShippingMode = " androidboot.shippingmode=1";
STATIC CONST CHAR8 *bootreason_panic = " androidboot.bootreason=kernel_panic";
STATIC CONST CHAR8 *cmd_boot_reason = NULL;
#endif

#define SIZE_OF_DELIM 2
#define PARAM_DELIM "\n"
#define ADD_PARAM_LEN(BootConfigFlag, ParamLen, CmdLineL, BootConfigL) \
                     do { \
                       if (BootConfigFlag == FALSE) { \
                         CmdLineL += ParamLen; \
                       } else { \
                         BootConfigL += (ParamLen + SIZE_OF_DELIM); \
                         CmdLineL += ParamLen; \
                       }\
                     } while (0);
STATIC CONST CHAR8 *DynamicBootDeviceCmdLine =
                                      " androidboot.boot_devices=soc/";
STATIC CONST CHAR8 *BootDeviceCmdLine = " androidboot.bootdevice=";

STATIC CONST CHAR8 *UsbSerialCmdLine = " androidboot.serialno=";
STATIC CONST CHAR8 *AndroidBootMode = " androidboot.mode=";
STATIC CONST CHAR8 *LogLevel = " quite";
STATIC CONST CHAR8 *BatteryChgPause = " androidboot.mode=charger";
STATIC CONST CHAR8 *MdtpActiveFlag = " mdtp";
STATIC CONST CHAR8 *AlarmBootCmdLine = " androidboot.alarmboot=true";
STATIC CONST CHAR8 *NoPasr = " mem_offline.nopasr=1";

/*Send slot suffix in cmdline with which we have booted*/
STATIC CHAR8 *AndroidSlotSuffix = " androidboot.slot_suffix=";
STATIC CHAR8 *RootCmdLine = " rootwait ro init=";
STATIC CHAR8 *InitCmdline = INIT_BIN;
STATIC CHAR8 *SkipRamFs = " skip_initramfs";

STATIC CHAR8 IPv4AddrBufCmdLine[MAX_IP_ADDR_BUF];
STATIC CHAR8 IPv6AddrBufCmdLine[MAX_IP_ADDR_BUF];
STATIC CHAR8 MacEthAddrBufCmdLine[MAX_IP_ADDR_BUF];
STATIC CHAR8 *ResumeCmdLine = NULL;

/* Display command line related structures */
#define MAX_DISPLAY_CMD_LINE 256
STATIC CHAR8 DisplayCmdLine[MAX_DISPLAY_CMD_LINE];
STATIC UINTN DisplayCmdLineLen = sizeof (DisplayCmdLine);

#define MAX_HW_FENCE_CMD_LINE 32
STATIC CHAR8 HwFenceCmdLine[MAX_HW_FENCE_CMD_LINE];
STATIC UINTN HwFenceCmdLineLen = sizeof (HwFenceCmdLine);

#define MAX_DTBO_IDX_STR 64
STATIC CHAR8 *AndroidBootDtboIdx = " androidboot.dtbo_idx=";
STATIC CHAR8 *AndroidBootDtbIdx = " androidboot.dtb_idx=";

STATIC CONST CHAR8 *AndroidBootForceNormalBoot =
                                      " androidboot.force_normal_boot=1";
STATIC CONST CHAR8 *AndroidBootFstabSuffix =
                                      " androidboot.fstab_suffix=";
STATIC CHAR8 *FstabSuffixEmmc = "emmc";
STATIC CHAR8 *FstabSuffixDefault = "default";

#ifdef ASUS_AI2205_BUILD
BOOLEAN EnterChargingMode = 0;
#endif

STATIC EFI_STATUS
GetBootReasonForCmdline (VOID){
  EFI_STATUS Status = EFI_SUCCESS;
  EFI_PM_PON_REASON_TYPE PONReason;
  EFI_QCOM_PMIC_PON_PROTOCOL *PmicPonProtocol;
  //EFI_QCOM_PMIC_GPIO_PROTOCOL *PmicGpioProtocol;
  BOOLEAN WarmRtStatus;
  extern UINT32 reboot_reason;

  Status = gBS->LocateProtocol (&gQcomPmicPonProtocolGuid, NULL,
                                (VOID **)&PmicPonProtocol);
  if (EFI_ERROR (Status)) {
      DEBUG ((EFI_D_ERROR, "Error locating pmic pon protocol: %r\n", Status));
      return Status;
  }

  SetMem (&PONReason, sizeof(EFI_PM_PON_REASON_TYPE), 0x00);

  /* Passing 0 for PMIC device Index since the protocol infers internally */
  Status = PmicPonProtocol->GetPonReason (0, &PONReason);
  if (EFI_ERROR (Status)) {
      DEBUG ((EFI_D_ERROR, "Error getting pon reason: %r\n", Status));
      return Status;
  }

  Status = PmicPonProtocol->WarmResetStatus (0, &WarmRtStatus);
  if (EFI_ERROR (Status)) {
      DEBUG ((EFI_D_ERROR, "Error getting warm reset status: %r\n", Status));
      return Status;
  }

  DEBUG ((EFI_D_INFO, " PON Reason is 0x%x, %a boot, Reset Reason is 0x%x\n",
          PONReason, WarmRtStatus ? "warm" : "cold", reboot_reason));

  if (reboot_reason == PANIC_REBOOT)
    cmd_boot_reason = bootreason_panic;

  if (cmd_boot_reason)
      DEBUG ((EFI_D_INFO, "[ABL] bootreason: %a\n", cmd_boot_reason));

  return Status;
}


/* Memory offline arguments */
STATIC CHAR8 *MemOff = " mem=";
STATIC CONST CHAR8 *MemHpState = " memhp_default_state=online";
STATIC CONST CHAR8 *MovableNode = " movable_node";

LIST_ENTRY *BootConfigListHead = NULL;
EFI_STATUS
TargetPauseForBatteryCharge (BOOLEAN *BatteryStatus)
{
  EFI_STATUS Status = EFI_SUCCESS;
  EFI_PM_PON_REASON_TYPE PONReason;
  EFI_QCOM_PMIC_PON_PROTOCOL *PmicPonProtocol;
  EFI_CHARGER_EX_PROTOCOL *ChgDetectProtocol;
  BOOLEAN ChgPresent;
  BOOLEAN WarmRtStatus;
  BOOLEAN IsColdBoot;
//ASUS_BSP +++ boot to COS mode in low voltage
#if defined  ASUS_AI2205_BUILD 
  UINT32 BatteryVoltage = 0;
  EFI_TLMM_PROTOCOL *TLMMProtocol;
  UINT32 BTM_OVP_Val = 1;
  UINT32 config = 0;
#endif
//ASUS_BSP --- boot to COS mode in low voltage

#if defined  ASUS_AI2205_BUILD 
  //Enter COS if user enter "adb reboot EnterShippingMode"
  if(adb_enter_shipping_mode){
	DEBUG((EFI_D_ERROR, "[ABL] adb command trigger shipping mode, boot to cos\n"));
	*BatteryStatus = 1;
	return Status;
  }
#endif

  /* Determines whether to pause for batter charge,
   * Serves only performance purposes, defaults to return zero*/
  *BatteryStatus = 0;

  Status = gBS->LocateProtocol (&gChargerExProtocolGuid, NULL,
                                (VOID **)&ChgDetectProtocol);
  if (Status == EFI_NOT_FOUND) {
    DEBUG ((EFI_D_VERBOSE, "Charger Protocol is not available.\n"));
    return EFI_SUCCESS;
  } else if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "Error finding charger protocol: %r\n", Status));
    return Status;
  }

//ASUS_BSP +++ boot to COS mode in low voltage
#if defined  ASUS_AI2205_BUILD
/* Get battery voltage/capacity & CHG_present +++ */
  Status = ChgDetectProtocol->GetBatteryVoltage (&BatteryVoltage);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "Error getting charger info battery voltage: %r\n", Status));
    return Status;
  }
  DEBUG((EFI_D_ERROR, "[ABL] Battery Voltage = %d\n", BatteryVoltage));

  Status = ChgDetectProtocol->GetChargerPresence (&ChgPresent);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "Error getting charger info: %r\n", Status));
    return Status;
  }
  DEBUG((EFI_D_ERROR, "[ABL] ChgPresent = %d\n", ChgPresent));

  Status = gBS->LocateProtocol( &gEfiTLMMProtocolGuid, NULL, (void**)&TLMMProtocol);
  if (EFI_ERROR(Status)) {
    DEBUG((EFI_D_ERROR, "[ABL] TargetPauseForBatteryCharge : Locate TLMMProtocol protocol failed! %r\n", Status));
    return Status;
  }
  config = EFI_GPIO_CFG(85, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_16MA);
  Status = TLMMProtocol->GpioIn(config, &BTM_OVP_Val);
  if (EFI_ERROR(Status)) {
    DEBUG((EFI_D_ERROR, "Error getting TLMMProtocol->GpioIn: %r\n", Status));
  }
  DEBUG((EFI_D_ERROR, "[ABL] BTM_OVP_Val = %d\n", BTM_OVP_Val));

  if ((ChgPresent || !BTM_OVP_Val) && (BatteryVoltage <= 7000)) {
    *BatteryStatus = 1;
    DEBUG((EFI_D_ERROR, "[ABL] Battery Voltage <= 7000, boot to COS\n"));
    return Status;
  }
#endif
/* Get battery voltage/capacity & CHG_present --- */
//ASUS_BSP --- boot to COS mode in low voltage

  /* The new protocol are supported on future chipsets */
  if (ChgDetectProtocol->Revision >= CHARGER_EX_REVISION) {
    Status = ChgDetectProtocol->IsOffModeCharging (BatteryStatus);
    if (EFI_ERROR (Status))
      DEBUG (
          (EFI_D_ERROR, "Error getting off mode charging info: %r\n", Status));

    return Status;
  } else {
    Status = gBS->LocateProtocol (&gQcomPmicPonProtocolGuid, NULL,
                                  (VOID **)&PmicPonProtocol);
    if (EFI_ERROR (Status)) {
      DEBUG ((EFI_D_ERROR, "Error locating pmic pon protocol: %r\n", Status));
      return Status;
    }

    /* Passing 0 for PMIC device Index since the protocol infers internally */
    Status = PmicPonProtocol->GetPonReason (0, &PONReason);
    if (EFI_ERROR (Status)) {
      DEBUG ((EFI_D_ERROR, "Error getting pon reason: %r\n", Status));
      return Status;
    }

    Status = PmicPonProtocol->WarmResetStatus (0, &WarmRtStatus);
    if (EFI_ERROR (Status)) {
      DEBUG ((EFI_D_ERROR, "Error getting warm reset status: %r\n", Status));
      return Status;
    }

    IsColdBoot = !WarmRtStatus;
    Status = ChgDetectProtocol->GetChargerPresence (&ChgPresent);
    if (EFI_ERROR (Status)) {
      DEBUG ((EFI_D_ERROR, "Error getting charger info: %r\n", Status));
      return Status;
    }

    DEBUG ((EFI_D_INFO, " PON Reason is %d cold_boot:%d charger path: %d\n",
            PONReason, IsColdBoot, ChgPresent));
    /* In case of fastboot reboot,adb reboot or if we see the power key
     * pressed we do not want go into charger mode.
     * fastboot/adb reboot is warm boot with PON hard reset bit set.
     */
    if (IsColdBoot && (!(PONReason.HARD_RESET) && (!(PONReason.KPDPWR)) &&
                       (PONReason.PON1 || PONReason.USB_CHG) && (ChgPresent))) {
      *BatteryStatus = 1;
    } else {
      *BatteryStatus = 0;
    }

    return Status;
  }
}

/**
  Check battery status
  @param[out] BatteryPresent  The pointer to battry's presence status.
  @param[out] ChargerPresent  The pointer to battry's charger status.
  @param[out] BatteryVoltage  The pointer to battry's voltage.
  @retval     EFI_SUCCESS     Check battery status successfully.
  @retval     other           Failed to check battery status.
**/
STATIC EFI_STATUS
TargetCheckBatteryStatus (BOOLEAN *BatteryPresent,
                          BOOLEAN *ChargerPresent,
                          UINT32 *BatteryVoltage)
{
  EFI_STATUS Status = EFI_SUCCESS;
  EFI_CHARGER_EX_PROTOCOL *ChgDetectProtocol;

  Status = gBS->LocateProtocol (&gChargerExProtocolGuid, NULL,
                                (void **)&ChgDetectProtocol);
  if (Status == EFI_NOT_FOUND) {
    DEBUG ((EFI_D_VERBOSE, "Charger Protocol is not available.\n"));
    return EFI_SUCCESS;
  } else if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "Error locating charger detect protocol\n"));
    return EFI_PROTOCOL_ERROR;
  }

  Status = ChgDetectProtocol->GetBatteryPresence (BatteryPresent);
  if (EFI_ERROR (Status)) {
    /* Not critical. Hence, loglevel priority is low*/
    DEBUG ((EFI_D_VERBOSE, "Error getting battery presence: %r\n", Status));
    return Status;
  }

  Status = ChgDetectProtocol->GetBatteryVoltage (BatteryVoltage);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "Error getting battery voltage: %r\n", Status));
    return Status;
  }

  Status = ChgDetectProtocol->GetChargerPresence (ChargerPresent);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "Error getting charger presence: %r\n", Status));
    return Status;
  }

  return Status;
}

VOID
ParseVBCmdLine (CHAR8* VBCmdLine, UINT32 Len) {
  for (UINT32 Iter = 0 ; Iter < Len ; Iter++) {
    if (VBCmdLine[Iter] == ' ') {
      VBCmdLine[Iter] = '\n';
    }
  }
}
/**
   Add safeguards such as refusing to flash if the battery levels is lower than
 the min voltage
   or bypass if the battery is not present.
   @param[out] BatteryVoltage  The current voltage of battery
   @retval     BOOLEAN         The value whether the device is allowed to flash
 image.
 **/
BOOLEAN
TargetBatterySocOk (UINT32 *BatteryVoltage)
{
  EFI_STATUS Status = EFI_SUCCESS;
  EFI_CHARGER_EX_PROTOCOL *ChgDetectProtocol = NULL;
  EFI_CHARGER_EX_FLASH_INFO FlashInfo = {0};
  BOOLEAN BatteryPresent = FALSE;
  BOOLEAN ChargerPresent = FALSE;

  *BatteryVoltage = 0;
  Status = gBS->LocateProtocol (&gChargerExProtocolGuid, NULL,
                                (VOID **)&ChgDetectProtocol);
  if (Status == EFI_NOT_FOUND) {
    DEBUG ((EFI_D_VERBOSE, "Charger Protocol is not available.\n"));
    return TRUE;
  } else if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "Error locating charger detect protocol\n"));
    return FALSE;
  }

  /* The new protocol are supported on future chipsets */
  if (ChgDetectProtocol->Revision >= CHARGER_EX_REVISION) {
    Status = ChgDetectProtocol->IsPowerOk (
        EFI_CHARGER_EX_POWER_FLASH_BATTERY_VOLTAGE_TYPE, &FlashInfo);
    if (EFI_ERROR (Status)) {
      /* But be bypassable where the device doesn't even have a battery */
      if (Status == EFI_UNSUPPORTED)
        return TRUE;

      DEBUG ((EFI_D_ERROR, "Error getting the info of charger: %r\n", Status));
      return FALSE;
    }

    *BatteryVoltage = FlashInfo.BattCurrVoltage;
    if (!(FlashInfo.bCanFlash) ||
        (*BatteryVoltage < FlashInfo.BattRequiredVoltage))
    {
      DEBUG ((EFI_D_ERROR, "Error battery voltage: %d "
        "Requireed voltage: %d, can flash: %d\n", *BatteryVoltage,
        FlashInfo.BattRequiredVoltage, FlashInfo.bCanFlash));
      return FALSE;
    }
    return TRUE;
  } else {
    Status = TargetCheckBatteryStatus (&BatteryPresent, &ChargerPresent,
                                       BatteryVoltage);
    if (((Status == EFI_SUCCESS) &&
         (!BatteryPresent ||
          (BatteryPresent && (*BatteryVoltage > BATT_MIN_VOLT)))) ||
        (Status == EFI_UNSUPPORTED)) {
      return TRUE;
    }

    DEBUG ((EFI_D_ERROR, "Error battery check status: %r voltage: %d\n",
        Status, *BatteryVoltage));
    return FALSE;
  }
}

STATIC VOID GetDisplayCmdline (VOID)
{
  EFI_STATUS Status;

  Status = gRT->GetVariable ((CHAR16 *)L"DisplayPanelConfiguration",
                             &gQcomTokenSpaceGuid, NULL, &DisplayCmdLineLen,
                             DisplayCmdLine);
  if (Status != EFI_SUCCESS) {
    DEBUG ((EFI_D_ERROR, "Unable to get Panel Config, %r\n", Status));
  }
}

STATIC VOID GetHwFenceCmdline (VOID)
{
  EFI_STATUS Status;

  Status = gRT->GetVariable ((CHAR16 *)L"HwFenceConfiguration",
                             &gQcomTokenSpaceGuid, NULL, &HwFenceCmdLineLen,
                             HwFenceCmdLine);
  if (Status != EFI_SUCCESS) {
    DEBUG ((EFI_D_ERROR, "Unable to get hw fence Config, %r\n", Status));
  }
}

/*
 * Returns length = 0 when there is failure.
 */
UINT32
GetSystemPath (CHAR8 **SysPath, BOOLEAN MultiSlotBoot, BOOLEAN BootIntoRecovery,
                CHAR16 *ReqPartition, CHAR8 *Key, BOOLEAN FlashlessBoot)
{
  INT32 Index;
  UINT32 Lun;
  CHAR16 PartitionName[MAX_GPT_NAME_SIZE];
  Slot CurSlot = GetCurrentSlotSuffix ();
  CHAR8 LunCharMapping[] = {'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h'};
  CHAR8 RootDevStr[BOOT_DEV_NAME_SIZE_MAX];

  *SysPath = AllocateZeroPool (sizeof (CHAR8) * MAX_PATH_SIZE);
  if (!*SysPath) {
    DEBUG ((EFI_D_ERROR, "Failed to allocated memory for System path query\n"));
    return 0;
  }

  if (FlashlessBoot) {
     AsciiSPrint (*SysPath, MAX_PATH_SIZE,
                     " rootfstype=squashfs root=/dev/ram0");
     return AsciiStrLen (*SysPath);
  }

  if (ReqPartition == NULL ||
      Key == NULL) {
    DEBUG ((EFI_D_ERROR, "Invalid parameters: NULL\n"));
    FreePool (*SysPath);
    *SysPath = NULL;
    return 0;
  }

  if (IsLEVariant () &&
      BootIntoRecovery) {
    StrnCpyS (PartitionName, MAX_GPT_NAME_SIZE, (CONST CHAR16 *)L"recoveryfs",
            StrLen ((CONST CHAR16 *)L"recoveryfs"));
  } else {
    StrnCpyS (PartitionName, MAX_GPT_NAME_SIZE, ReqPartition,
            StrLen (ReqPartition));
  }

  /* Append slot info for A/B Variant */
  if (MultiSlotBoot &&
      NAND != CheckRootDeviceType ()) {
     StrnCatS (PartitionName, MAX_GPT_NAME_SIZE, CurSlot.Suffix,
            StrLen (CurSlot.Suffix));
  }

  Index = GetPartitionIndex (PartitionName);
  if (Index == INVALID_PTN || Index >= MAX_NUM_PARTITIONS) {
    DEBUG ((EFI_D_ERROR, "System partition does not exist\n"));
    FreePool (*SysPath);
    *SysPath = NULL;
    return 0;
  }

  Lun = GetPartitionLunFromIndex (Index);
  GetRootDeviceType (RootDevStr, BOOT_DEV_NAME_SIZE_MAX);
  if (!AsciiStrCmp ("Unknown", RootDevStr)) {
    FreePool (*SysPath);
    *SysPath = NULL;
    return 0;
  }

  if (!AsciiStrCmp ("EMMC", RootDevStr)) {
    AsciiSPrint (*SysPath, MAX_PATH_SIZE, " %a=/dev/mmcblk0p%d", Key, Index);
  } else if (!AsciiStrCmp ("NAND", RootDevStr)) {
    /* NAND is being treated as GPT partition, hence reduce the index by 1 as
     * PartitionIndex (0) should be ignored for correct mapping of partition.
     */
    if (IsNANDSquashFsSupport ()) {
      // The gluebi device that is to be passed to "root=" will be the first one
      // after all "regular" mtd devices have been populated.
      UINT32 PartitionCount = 0;
      UINT32 MtdBlkIndex = 0;
      GetPartitionCount (&PartitionCount);
      if (MultiSlotBoot &&
         (StrnCmp ((CONST CHAR16 *)L"_b", CurSlot.Suffix,
          StrLen (CurSlot.Suffix)) == 0))
         MtdBlkIndex = PartitionCount;
      else
         MtdBlkIndex = PartitionCount - 1;
      AsciiSPrint (*SysPath, MAX_PATH_SIZE,
                   " rootfstype=squashfs root=/dev/mtdblock%d ubi.mtd=%d",
                   MtdBlkIndex, (Index - 1));
    } else {
      AsciiSPrint (*SysPath, MAX_PATH_SIZE,
          " rootfstype=ubifs rootflags=bulk_read root=ubi0:rootfs ubi.mtd=%d",
          (Index - 1));
    }
  } else if (!AsciiStrCmp ("UFS", RootDevStr)) {
    AsciiSPrint (*SysPath, MAX_PATH_SIZE, " %a=/dev/sd%c%d",
                 Key,
                 LunCharMapping[Lun],
                 GetPartitionIdxInLun (PartitionName, Lun));
  } else {
    DEBUG ((EFI_D_ERROR, "Unknown Device type\n"));
    FreePool (*SysPath);
    *SysPath = NULL;
    return 0;
  }
  DEBUG ((EFI_D_VERBOSE, "System Path - %a \n", *SysPath));

  return AsciiStrLen (*SysPath);
}

UINT32
GetResumeCmdLine (CHAR8 **ResumeCmdLine, CHAR16 *ReqPartition)
{
  BOOLEAN MultiSlotBoot;
  UINT32 Len = 0;

  MultiSlotBoot = PartitionHasMultiSlot ((CONST CHAR16 *)L"swap_a");
  Len = GetSystemPath (ResumeCmdLine, MultiSlotBoot, FALSE,
                (CHAR16 *)L"swap_a", (CHAR8 *)"resume", FALSE);
  if (Len == 0) {
     DEBUG ((EFI_D_ERROR, "GetSystemPath failed\n"));
     return 0;
  }
  return Len;
}

STATIC
EFI_STATUS
GetMemoryLimit (VOID *fdt, CHAR8 *MemOffAmt)
{
  UINT64 DdrSize = 0;
  UINT64 MemLimit;
  UINT32 i = 0;
  INT32 MemOfflineOffset;
  UINT64 *MemTable;
  INT32 PropLen;
  EFI_STATUS Status;

  if (IsLEVariant ()) {
    goto Unsupported;
  }

  Status = GetDdrSize (&DdrSize);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "Error getting DDR size %r\n", Status));
    return Status;
  }

  MemLimit = DdrSize;
  MemOfflineOffset = FdtPathOffset (fdt, "/mem-offline");

  if (DdrSize < MEM_OFF_MIN ||
      MemOfflineOffset < 0) {
    goto Unsupported;
  }

  /* get table of offline sizes and subtract the size based off of DDR size */
  MemTable = (UINT64 *)fdt_getprop_w (fdt, MemOfflineOffset, "offline-sizes",
                                      &PropLen);
  if (!MemTable ||
       PropLen < 0) {
    goto Unsupported;
  }

  if (DdrSize >= SwapBytes64 (MemTable[0])) {
    for (i = (PropLen / sizeof (UINT64)) - 2; i >= 0; i -= 2) {
      if (DdrSize >= SwapBytes64 (MemTable[i])) {
        MemLimit -= SwapBytes64 (MemTable[i + 1]);
        break;
      }
    }
  }

  AsciiSPrint (MemOffAmt, MEM_OFF_SIZE, "%luB", MemLimit);

  return EFI_SUCCESS;

Unsupported:
  DEBUG ((EFI_D_INFO, "Offlining Memory Not Supported\n"));
  return EFI_UNSUPPORTED;
}

STATIC
EFI_STATUS
UpdateCmdLineParams (UpdateCmdLineParamList *Param, CHAR8 **FinalCmdLine,
                     BootParamlist *BootParamlistPtr)
{
  CONST CHAR8 *Src;
  CHAR8 *Dst;
  UINT32 MaxCmdLineLen = Param->CmdLineLen;
  BOOLEAN BootConfigFlag = FALSE;

  Dst = AllocateZeroPool (MaxCmdLineLen);
  if (!Dst) {
    DEBUG ((EFI_D_ERROR, "CMDLINE: Failed to allocate destination buffer\n"));
    return EFI_OUT_OF_RESOURCES;
  }

  /* Save start ptr for debug print */
  *FinalCmdLine = Dst;

  if (Param->HaveCmdLine) {
    Src = Param->CmdLine;
    AsciiStrCpyS (Dst, MaxCmdLineLen, Src);
  }

  if (Param->VBCmdLine != NULL) {
    if (Param->HeaderVersion <= BOOT_HEADER_VERSION_THREE) {
        Src = Param->VBCmdLine;
        AsciiStrCatS (Dst, MaxCmdLineLen, Src);
    } else {
        BootConfigFlag = IsAndroidBootParam (Param->VBCmdLine,
                              AsciiStrLen (Param->VBCmdLine),
                                       Param->HeaderVersion);
      ParseVBCmdLine ((CHAR8*) Param->VBCmdLine,
                 AsciiStrLen (Param->VBCmdLine));
      AddtoBootConfigList (BootConfigFlag, Param->VBCmdLine, NULL,
                  BootConfigListHead, AsciiStrLen (Param->VBCmdLine), 0);
    }
  }

  if ((Param->BootDevBuf) &&
      (Param->HeaderVersion <= BOOT_HEADER_VERSION_THREE)) {
    Src = Param->BootDeviceCmdLine;
    AsciiStrCatS (Dst, MaxCmdLineLen, Src);

    Src = Param->BootDevBuf;
    AsciiStrCatS (Dst, MaxCmdLineLen, Src);

    Src = Param->AndroidBootFstabSuffix;
    AsciiStrCatS (Dst, MaxCmdLineLen, Src);

    Src = Param->FstabSuffix;
    AsciiStrCatS (Dst, MaxCmdLineLen, Src);

    /* Dynamic partition append boot_devices for super partition */
    if (IsDynamicPartitionSupport ()) {
      Src = DynamicBootDeviceCmdLine;
      AsciiStrCatS (Dst, MaxCmdLineLen, Src);

      Src = Param->BootDevBuf;
      AsciiStrCatS (Dst, MaxCmdLineLen, Src);
    }

    FreePool (Param->BootDevBuf);
    Param->BootDevBuf = NULL;
  }

  if (Param->HeaderVersion <= BOOT_HEADER_VERSION_THREE) {
    Src = Param->UsbSerialCmdLine;
    AsciiStrCatS (Dst, MaxCmdLineLen, Src);
    Src = Param->StrSerialNum;
    AsciiStrCatS (Dst, MaxCmdLineLen, Src);
  }
  if (Param->FfbmStr &&
      (Param->FfbmStr[0] != '\0')) {
    if (Param->HeaderVersion <= BOOT_HEADER_VERSION_THREE) {
      Src = Param->AndroidBootMode;
      AsciiStrCatS (Dst, MaxCmdLineLen, Src);

      Src = Param->FfbmStr;
      AsciiStrCatS (Dst, MaxCmdLineLen, Src);
    }

    Src = Param->LogLevel;
    AsciiStrCatS (Dst, MaxCmdLineLen, Src);
  } else if (Param->PauseAtBootUp) {
      if (Param->HeaderVersion <= BOOT_HEADER_VERSION_THREE) {
        Src = Param->BatteryChgPause;
        AsciiStrCatS (Dst, MaxCmdLineLen, Src);
      }
  } else if (Param->AlarmBoot) {
      if (Param->HeaderVersion <= BOOT_HEADER_VERSION_THREE) {
        Src = Param->AlarmBootCmdLine;
        AsciiStrCatS (Dst, MaxCmdLineLen, Src);
      }
  }

  if (Param->HeaderVersion <= BOOT_HEADER_VERSION_THREE) {
    Src = BOOT_BASE_BAND;
    AsciiStrCatS (Dst, MaxCmdLineLen, Src);

    gBS->SetMem (Param->ChipBaseBand, CHIP_BASE_BAND_LEN, 0);
    AsciiStrnCpyS (Param->ChipBaseBand, CHIP_BASE_BAND_LEN,
                 BoardPlatformChipBaseBand (),
                 (CHIP_BASE_BAND_LEN - 1));
    ToLower (Param->ChipBaseBand);
    Src = Param->ChipBaseBand;
    AsciiStrCatS (Dst, MaxCmdLineLen, Src);

  }

  Src = Param->DisplayCmdLine;
  AsciiStrCatS (Dst, MaxCmdLineLen, Src);

  Src = Param->HwFenceCmdLine;
  AsciiStrCatS (Dst, MaxCmdLineLen, Src);

  if (Param->MdtpActive) {
    Src = Param->MdtpActiveFlag;
    AsciiStrCatS (Dst, MaxCmdLineLen, Src);
  }

  if (Param->MultiSlotBoot &&
     !IsBootDevImage ()) {
     /* Slot suffix */
    if (Param->HeaderVersion <= BOOT_HEADER_VERSION_THREE) {
      Src = Param->AndroidSlotSuffix;
      AsciiStrCatS (Dst, MaxCmdLineLen, Src);
    }

    UnicodeStrToAsciiStr (GetCurrentSlotSuffix ().Suffix,
                          Param->SlotSuffixAscii);
    if (Param->HeaderVersion <= BOOT_HEADER_VERSION_THREE) {
      Src = Param->SlotSuffixAscii;
      AsciiStrCatS (Dst, MaxCmdLineLen, Src);
    } else {
      BootConfigFlag = IsAndroidBootParam (Param->AndroidSlotSuffix,
                              AsciiStrLen (Param->AndroidSlotSuffix),
                                       Param->HeaderVersion);
      AddtoBootConfigList (BootConfigFlag, Param->AndroidSlotSuffix,
                     Param->SlotSuffixAscii,
                     BootConfigListHead,
                     AsciiStrLen (Param->AndroidSlotSuffix),
                     AsciiStrLen (Param->SlotSuffixAscii));
    }
  }

  if ((IsBuildAsSystemRootImage (BootParamlistPtr) &&
      !Param->MultiSlotBoot) ||
      (Param->MultiSlotBoot &&
      !IsBootDevImage ())) {

       /* Skip Initramfs*/
       if (!IsDynamicPartitionSupport () &&
           !Param->Recovery) {
         Src = Param->SkipRamFs;
         AsciiStrCatS (Dst, MaxCmdLineLen, Src);
       }

     /* Add root command line */
     Src = Param->RootCmdLine;
     AsciiStrCatS (Dst, MaxCmdLineLen, Src);

     /* Add init value*/
     Src = Param->InitCmdline;
     AsciiStrCatS (Dst, MaxCmdLineLen, Src);
   }

  if ((Param->DtboIdxStr != NULL) &&
       (Param->HeaderVersion <= BOOT_HEADER_VERSION_THREE)) {
    Src = Param->DtboIdxStr;
    AsciiStrCatS (Dst, MaxCmdLineLen, Src);
  }

  if ((Param->DtbIdxStr != NULL) &&
       (Param->HeaderVersion <= BOOT_HEADER_VERSION_THREE)) {
    Src = Param->DtbIdxStr;
    AsciiStrCatS (Dst, MaxCmdLineLen, Src);
  }

  if (((IsBuildUseRecoveryAsBoot () ||
      IsRecoveryHasNoKernel ()) &&
      IsDynamicPartitionSupport () &&
      !Param->Recovery) ||
      (!Param->MultiSlotBoot &&
       !IsBuildUseRecoveryAsBoot ())) {
    if (Param->HeaderVersion <= BOOT_HEADER_VERSION_THREE) {
      Src = AndroidBootForceNormalBoot;
      AsciiStrCatS (Dst, MaxCmdLineLen, Src);
    }
  }

  if (Param->LEVerityCmdLine != NULL) {
    Src = Param->LEVerityCmdLine;
    AsciiStrCatS (Dst, MaxCmdLineLen, Src);
    FreePool (Param->LEVerityCmdLine);
    Param->LEVerityCmdLine = NULL;
  }

  if (Param->MemOffAmt != NULL) {
    Src = MemOff;
    AsciiStrCatS (Dst, MaxCmdLineLen, Src);
    Src = Param->MemOffAmt;
    AsciiStrCatS (Dst, MaxCmdLineLen, Src);
    Src = MemHpState;
    AsciiStrCatS (Dst, MaxCmdLineLen, Src);
    Src = MovableNode;
    AsciiStrCatS (Dst, MaxCmdLineLen, Src);
  } else if (Param->NoPasr != NULL) {
    Src = Param->NoPasr;
    AsciiStrCatS (Dst, MaxCmdLineLen, Src);
  }

  if (EarlyEthEnabled ()) {
    Src = Param->EarlyIPv4CmdLine;
    AsciiStrCatS (Dst, MaxCmdLineLen, Src);
    Src = Param->EarlyIPv6CmdLine;
    AsciiStrCatS (Dst, MaxCmdLineLen, Src);
    Src = Param->EarlyEthMacCmdLine;
  }

  if (IsHibernationEnabled ()) {
    Src = Param->ResumeCmdLine;
    AsciiStrCatS (Dst, MaxCmdLineLen, Src);
  }

#ifdef ASUS_BUILD

  if (Param->Recovery){
    Src = Param->RecoveryMode;
    AsciiStrCatS (Dst, MaxCmdLineLen, Src);
  }

  // +++ ASUS_BSP : add for logcat-asdf sevices
  if ((Param->cmd_enable_logcat_asdf!=NULL) &&
		(Param->HeaderVersion <= BOOT_HEADER_VERSION_THREE))
  {
    Src = Param->cmd_enable_logcat_asdf;
    AsciiStrCatS (Dst, MaxCmdLineLen, Src);
  }

  Src = Param->cmd_asus_hwid;
  AsciiStrCatS (Dst, MaxCmdLineLen, Src);

  // +++ ASUS_BSP : add for ftm mode
  Src = Param->cmd_enable_adb_mode;
  AsciiStrCatS (Dst, MaxCmdLineLen, Src);
  
  if(Param->HeaderVersion <= BOOT_HEADER_VERSION_THREE){
    Src = Param->cmd_enable_adb_prop;
    AsciiStrCatS (Dst, MaxCmdLineLen, Src);
  }

  Src = Param->cmd_ftm_mode;
  AsciiStrCatS (Dst, MaxCmdLineLen, Src);
  
  if(Param->HeaderVersion <= BOOT_HEADER_VERSION_THREE){
    Src = Param->cmd_ftm_mode_prop;
    AsciiStrCatS (Dst, MaxCmdLineLen, Src);
  }

  Src = Param->cmd_selinux;
  AsciiStrCatS (Dst, MaxCmdLineLen, Src);
  
  if(Param->HeaderVersion <= BOOT_HEADER_VERSION_THREE){
    Src = Param->cmd_selinux_prop;
    AsciiStrCatS (Dst, MaxCmdLineLen, Src);
  }

  //+++ ASUS_BSP : add for ASUS ID value
  if(Param->HeaderVersion <= BOOT_HEADER_VERSION_THREE){
    Src = Param->cmd_soc_id;
    AsciiStrCatS (Dst, MaxCmdLineLen, Src);
    
    Src = Param->cmd_prj_id;
    AsciiStrCatS (Dst, MaxCmdLineLen, Src);
    
    Src = Param->cmd_stage_id;
    AsciiStrCatS (Dst, MaxCmdLineLen, Src);

    Src = Param->cmd_sku_id;
    AsciiStrCatS (Dst, MaxCmdLineLen, Src);

    Src = Param->cmd_rf_id;
    AsciiStrCatS (Dst, MaxCmdLineLen, Src);

    Src = Param->cmd_ddr_id;
    AsciiStrCatS (Dst, MaxCmdLineLen, Src);

    Src = Param->cmd_feature_id;
    AsciiStrCatS (Dst, MaxCmdLineLen, Src);

    Src = Param->cmd_jtag_id;
    AsciiStrCatS (Dst, MaxCmdLineLen, Src);

    Src = Param->cmd_pcb_id;
    AsciiStrCatS (Dst, MaxCmdLineLen, Src);

    Src = Param->cmd_nfc_id;
    AsciiStrCatS (Dst, MaxCmdLineLen, Src);

    Src = Param->cmd_fp_id;
    AsciiStrCatS (Dst, MaxCmdLineLen, Src);

    Src = Param->cmd_lgf_id;
    AsciiStrCatS (Dst, MaxCmdLineLen, Src);
     
    Src = Param->cmd_lgf_con_id;
    AsciiStrCatS (Dst, MaxCmdLineLen, Src);
  
    Src = Param->cmd_fc_id;
    AsciiStrCatS (Dst, MaxCmdLineLen, Src);
  
    Src = Param->cmd_upper_id;
    AsciiStrCatS (Dst, MaxCmdLineLen, Src);
  
    Src = Param->cmd_sub_id;
    AsciiStrCatS (Dst, MaxCmdLineLen, Src);
    
    Src = Param->cmd_valid_image;
    AsciiStrCatS (Dst, MaxCmdLineLen, Src);

    // +++ ASUS_BPS : add for read country code
    Src = Param->cmd_country_code;
    AsciiStrCatS (Dst, MaxCmdLineLen, Src);

    Src = Param->cmd_product_name;
    AsciiStrCatS (Dst, MaxCmdLineLen, Src);

    // +++ ASUS_BSP : add for boot count
    Src = Param->cmd_boot_count;
    AsciiStrCatS (Dst, MaxCmdLineLen, Src);

    // +++ ASUS_BSP : add for reboot reason
    Src = Param->cmd_boot_reason;
    AsciiStrCatS (Dst, MaxCmdLineLen, Src);

    // +++ ASUS_BSP : check if have rawdump partition or not
    Src = Param->cmd_rawdump_en;
    AsciiStrCatS (Dst, MaxCmdLineLen, Src); 
    
    Src = Param->cmd_authorized_prop;
    AsciiStrCatS (Dst, MaxCmdLineLen, Src);
    
    Src = Param->cmd_cpuid_hash;
    AsciiStrCatS (Dst, MaxCmdLineLen, Src);

    Src = Param->cmd_toolid;
    AsciiStrCatS (Dst, MaxCmdLineLen, Src);
    
    Src = Param->cmd_fuse_prop;
    AsciiStrCatS (Dst, MaxCmdLineLen, Src);
    
    Src = Param->cmd_fuse_no_rpmb_prop;
    AsciiStrCatS (Dst, MaxCmdLineLen, Src);
    
    Src = Param->cmd_hardware_sku_prop;
    AsciiStrCatS (Dst, MaxCmdLineLen, Src);
    
    // +++ ASUS_BSP : add for check factory crc
    Src = Param->cmd_factory_crc;
    AsciiStrCatS (Dst, MaxCmdLineLen, Src);

    Src = Param->cmd_retry_count;
    AsciiStrCatS (Dst, MaxCmdLineLen, Src);

    // +++ ASUS_BSP : add for ddr info
    /*
    Src = Param->cmd_ddr_manufacturer;
    AsciiStrCatS (Dst, MaxCmdLineLen, Src);

    Src = Param->cmd_ddr_device_type;
    AsciiStrCatS (Dst, MaxCmdLineLen, Src);
    */
    
    if(IsAuthorized_3()){
      Src = Param->cmd_watermask_unlock_prop;
      AsciiStrCatS (Dst, MaxCmdLineLen, Src);
    }
  }
  
  if (Param->cmd_uart_status!=NULL)
  {
    Src = Param->cmd_uart_status;
    AsciiStrCatS (Dst, MaxCmdLineLen, Src);
  }

  Src = Param->cmd_asus_info;
  AsciiStrCatS (Dst, MaxCmdLineLen, Src);

  Src = Param->cmd_unlock;
  AsciiStrCatS (Dst, MaxCmdLineLen, Src);

  Src = Param->cmd_update_cmdline;
  AsciiStrCatS (Dst, MaxCmdLineLen, Src);

  Src = Param->cmd_fuse_Info;
  AsciiStrCatS (Dst, MaxCmdLineLen, Src);

  // +++ ASUS_BSP : add for check fuse with no rpmb
  Src = Param->cmd_fuse_no_rpmb_Info;
  AsciiStrCatS (Dst, MaxCmdLineLen, Src);

  // +++ ASUS_BSP : add for panel uid
  Src = Param->cmd_unique_id;
  AsciiStrCatS (Dst, MaxCmdLineLen, Src);

  // +++ ASUS_BSP : add for WaterMask unlock
  if (IsAuthorized_3())
  {
    Src = Param->cmd_watermask_unlock;
    AsciiStrCatS (Dst, MaxCmdLineLen, Src);
  }
#endif

  return EFI_SUCCESS;
}
CHAR8* RemoveSpace (CHAR8* param, UINT32 ParamLen)
{
    UINT32 Iter, NewIter;
    CHAR8 *NewParam = param;

    for (Iter = 0, NewIter = 0; Iter < ParamLen; Iter++, NewIter++)
    {
        if (param[Iter] != ' ') {
          NewParam[NewIter] = param[Iter];
        }
        else {
          NewIter--;
        }
    }
    NewParam[NewIter] = '\0';
    return NewParam;
}
UINT32
ChangeFormattoBootConfig (CHAR8 *Param, UINT32 ParamLen)
{
  Param = RemoveSpace (Param, ParamLen);
  return (AsciiStrLen (Param));
}
struct BootConfigParamNode* AllocateBootConfigNode (UINT32 ParamLen)
{
  struct BootConfigParamNode *Node = NULL;
  Node = (struct BootConfigParamNode *)
         AllocateZeroPool (sizeof (struct BootConfigParamNode));
  if (!Node) {
    return NULL;
  }
  Node->param = (CHAR8 *)
                 AllocateZeroPool (ParamLen + SIZE_OF_DELIM);//to add \n
  if (!Node->param) {
    FreePool (Node);
    return NULL;
  }
  return Node;
}
VOID
AddParamToList (BOOLEAN BootConfigFlag,
                CONST CHAR8 *ParamKey,
                LIST_ENTRY *list,
                UINT32 ParamKeyLen)
{
  return;
}
VOID
AddtoBootConfigList (BOOLEAN BootConfigFlag,
                CONST CHAR8 *ParamKey,
                CONST CHAR8 *ParamValue,
                LIST_ENTRY *list,
                UINT32 ParamKeyLen,
                UINT32 ParamValueLen)
{
  struct BootConfigParamNode* NewNode = NULL;
  if (!BootConfigFlag) {
    return;
  }
  NewNode = (struct BootConfigParamNode *)
               AllocateBootConfigNode (ParamKeyLen + SIZE_OF_DELIM +
               SIZE_OF_DELIM + ParamValueLen);
  gBS->CopyMem (NewNode->param, (CHAR8*)ParamKey, ParamKeyLen);
  if (ParamValue) {
    gBS->CopyMem (&NewNode->param[ParamKeyLen], (CHAR8*)ParamValue,
                  ParamValueLen);
  }
  NewNode->ParamLen = ChangeFormattoBootConfig (NewNode->param,
                                               (ParamKeyLen + ParamValueLen));
  InsertTailList (list, &(NewNode->ListNode));
}

/* IsAndroidBootParam: Checks if the parameter is an androidboot.* kernel
 *                      parameter.
 * @param Param string
 * @param Param length
 * @return true if the parameter is androidboot.*
 */
BOOLEAN IsAndroidBootParam (CONST CHAR8 *param,
                            UINT32 ParamLen,
                            UINT32 HeaderVersion)
{
  if (ParamLen < 12) {
    return FALSE;
  }
  if (HeaderVersion <= BOOT_HEADER_VERSION_THREE) {
    return FALSE;
  }
  if (AsciiStrStr (param, "androidboot.")) {
    return TRUE;
  }
  else {
    return FALSE;
  }
}
EFI_STATUS
UpdateBootConfigParams (LIST_ENTRY *BootConfigListHead,
                        UINT32 BootConfigLen,
                        CHAR8 **FinalBootConfig,
                        UINT32 *FinalBootConfigLen)
{
  CHAR8* Dst = NULL;
  LIST_ENTRY *Link = BootConfigListHead;
  struct BootConfigParamNode* Node = NULL;
  BootConfigLen += SIZE_OF_DELIM;

  if (BootConfigLen == 0) {
    return EFI_D_ERROR;
  }

  if (!BootConfigListHead) {
    return EFI_D_ERROR;
  }

  Dst = (CHAR8 *) AllocateZeroPool (BootConfigLen + SIZE_OF_DELIM);
  if (!Dst) {
    return EFI_OUT_OF_RESOURCES;
  }
  Link = GetFirstNode (BootConfigListHead);
  if (!Link) {
    DEBUG ((EFI_D_INFO, "Error in Node entry \n"));
  }

  gBS->CopyMem (Dst, "\n", SIZE_OF_DELIM);

  while (!IsNull (BootConfigListHead, Link)) {
    Node = BASE_CR (Link, struct BootConfigParamNode, ListNode);
    if (!Node) {
      DEBUG ((EFI_D_INFO, "Unable to read bsae struct \n"));
      return EFI_INVALID_PARAMETER;
    }

    AsciiStrCatS (Dst, BootConfigLen, (CHAR8*)Node->param);
    AsciiStrCatS (Dst, BootConfigLen, "\n");
    Link = GetNextNode (BootConfigListHead, Link);

  }
  Dst[AsciiStrLen (Dst) + 1] = '\0';
  *FinalBootConfig = Dst;
  *FinalBootConfigLen = AsciiStrLen (Dst) + 1;

  return EFI_SUCCESS;
}
VOID
ClearBootConfigList (LIST_ENTRY* BootConfigListHead)
{
  LIST_ENTRY *Link = NULL;
  LIST_ENTRY *NewLink = NULL;
  struct BootConfigParamNode* Node = NULL;

  if (!BootConfigListHead) {
    return;
  }

  Link = GetFirstNode (BootConfigListHead);
  if (!Link) {
    DEBUG ((EFI_D_INFO, "Error in Node entry \n"));
  }

  while (!IsNull (BootConfigListHead, Link)) {

    NewLink = RemoveEntryList (Link);

    Node = BASE_CR (Link, struct BootConfigParamNode, ListNode);

    if (!Node) {
      break;
    }

    if (!Node->param) {
      break;
    }

    FreePool (Node->param);
    FreePool (Node);
    Link = NewLink;

  }
  FreePool (BootConfigListHead);

  return;

}
/*Update command line: appends boot information to the original commandline
 *that is taken from boot image header*/
EFI_STATUS
UpdateCmdLine (BootParamlist *BootParamlistPtr,
               CHAR8 *FfbmStr,
               BOOLEAN Recovery,
               BOOLEAN FlashlessBoot,
               BOOLEAN AlarmBoot,
               CONST CHAR8 *VBCmdLine,
               UINT32 HeaderVersion)
{
  EFI_STATUS Status;
  UINT32 CmdLineLen = 0;
  UINT32 BootConfigLen = 0;
  UINT32 ParamLen = 0;
  UINT32 HaveCmdLine = 0;
  UINT32 PauseAtBootUp = 0;
  CHAR8 SlotSuffixAscii[MAX_SLOT_SUFFIX_SZ];
  BOOLEAN MultiSlotBoot;
  CHAR8 ChipBaseBand[CHIP_BASE_BAND_LEN];
  CHAR8 *BootDevBuf = NULL;
  BOOLEAN BatteryStatus;
  CHAR8 StrSerialNum[SERIAL_NUM_SIZE];
  BOOLEAN MdtpActive = FALSE;
  UpdateCmdLineParamList Param = {0};
  CHAR8 DtboIdxStr[MAX_DTBO_IDX_STR] = "\0";
  CHAR8 DtbIdxStr[MAX_DTBO_IDX_STR] = "\0";
  INT32 DtboIdx = INVALID_PTN;
  INT32 DtbIdx = INVALID_PTN;
  CHAR8 *LEVerityCmdLine = NULL;
  UINT32 LEVerityCmdLineLen = 0;
  CHAR8 RootDevStr[BOOT_DEV_NAME_SIZE_MAX];
  CHAR8 MemOffAmt[MEM_OFF_SIZE];
  BOOLEAN BootConfigFlag = FALSE;

  CONST CHAR8 *CmdLine = BootParamlistPtr->CmdLine;
  CHAR8 **FinalCmdLine = &BootParamlistPtr->FinalCmdLine;
  CHAR8 **FinalBootConfig = &BootParamlistPtr->FinalBootConfig;
  UINT32 *FinalBootConfigLen = &BootParamlistPtr->FinalBootConfigLen;
  VOID *fdt = (VOID *)BootParamlistPtr->DeviceTreeLoadAddr;

  BootConfigListHead = (LIST_ENTRY*) AllocateZeroPool (sizeof (LIST_ENTRY));
  InitializeListHead (BootConfigListHead);

  if (!FlashlessBoot) {
    Status = BoardSerialNum (StrSerialNum, sizeof (StrSerialNum));
    if (Status != EFI_SUCCESS) {
      DEBUG ((EFI_D_ERROR, "Error Finding board serial num: %x\n", Status));
      return Status;
    }
  }

  if (CmdLine && CmdLine[0]) {
    CmdLineLen = AsciiStrLen (CmdLine);
    HaveCmdLine = 1;
  }

  if (FixedPcdGetBool (EnableMdtpSupport)) {
    Status = IsMdtpActive (&MdtpActive);

    if (EFI_ERROR (Status)) {
      DEBUG ((EFI_D_ERROR, "Failed to get activation state for MDTP, "
                           "Status=%r. Considering MDTP as active\n",
              Status));
      MdtpActive = TRUE;
    }
  }

  if (VBCmdLine != NULL) {
    DEBUG ((EFI_D_VERBOSE, "UpdateCmdLine VBCmdLine present len %d\n",
            AsciiStrLen (VBCmdLine)));
    ParamLen = AsciiStrLen (VBCmdLine);
    BootConfigFlag = IsAndroidBootParam (VBCmdLine, ParamLen,
                                              HeaderVersion);
    ADD_PARAM_LEN (BootConfigFlag, ParamLen, CmdLineLen,
                                         BootConfigLen);
  }

  if (HaveCmdLine) {
    if (IsLEVerity ()) {
      Status = GetLEVerityCmdLine (CmdLine, &LEVerityCmdLine,
                                   &LEVerityCmdLineLen);
      if (Status != EFI_SUCCESS) {
        DEBUG ((EFI_D_ERROR, "Failed to get LEVerityCmdLine: %r\n", Status));
      }
      CmdLineLen += LEVerityCmdLineLen;
    }
  }

  BootDevBuf = AllocateZeroPool (sizeof (CHAR8) * BOOT_DEV_MAX_LEN);
  if (BootDevBuf == NULL) {
    DEBUG ((EFI_D_ERROR, "Boot device buffer: Out of resources\n"));
    return EFI_OUT_OF_RESOURCES;
  }

  Status = GetBootDevice (BootDevBuf, BOOT_DEV_MAX_LEN);
  if (Status != EFI_SUCCESS) {
    DEBUG ((EFI_D_ERROR, "Failed to get Boot Device: %r\n", Status));
    FreePool (BootDevBuf);
    BootDevBuf = NULL;
  } else {
    ParamLen = AsciiStrLen (BootDeviceCmdLine);
    BootConfigFlag = IsAndroidBootParam (BootDeviceCmdLine, ParamLen,
                                         HeaderVersion);
    ADD_PARAM_LEN (BootConfigFlag, ParamLen, CmdLineLen, BootConfigLen);
    AddtoBootConfigList (BootConfigFlag, BootDeviceCmdLine, BootDevBuf,
                    BootConfigListHead, ParamLen, AsciiStrLen (BootDevBuf));
    ADD_PARAM_LEN (BootConfigFlag, AsciiStrLen (BootDevBuf),
                   CmdLineLen, BootConfigLen);

    if (IsDynamicPartitionSupport ()) {
      ParamLen = AsciiStrLen (DynamicBootDeviceCmdLine);
      BootConfigFlag = IsAndroidBootParam (DynamicBootDeviceCmdLine,
                                           ParamLen, HeaderVersion);
      ADD_PARAM_LEN (BootConfigFlag, ParamLen, CmdLineLen,
                                           BootConfigLen);
      AddtoBootConfigList (BootConfigFlag, DynamicBootDeviceCmdLine, BootDevBuf,
                 BootConfigListHead, ParamLen, AsciiStrLen (BootDevBuf));
      ADD_PARAM_LEN (BootConfigFlag, AsciiStrLen (BootDevBuf),
                     CmdLineLen, BootConfigLen);
    }
  }

  ParamLen = AsciiStrLen (UsbSerialCmdLine);
  BootConfigFlag = IsAndroidBootParam (UsbSerialCmdLine, ParamLen,
                                       HeaderVersion);
  ADD_PARAM_LEN (BootConfigFlag, ParamLen, CmdLineLen,
                                       BootConfigLen);
  AddtoBootConfigList (BootConfigFlag, UsbSerialCmdLine, StrSerialNum,
                     BootConfigListHead, ParamLen, AsciiStrLen (StrSerialNum));
  ADD_PARAM_LEN (BootConfigFlag, AsciiStrLen (StrSerialNum), CmdLineLen,
                                       BootConfigLen);

  GetBootReasonForCmdline();
  /* Ignore the EFI_STATUS return value as the default Battery Status = 0 and is
   * not fatal */
  TargetPauseForBatteryCharge (&BatteryStatus);

  if (FfbmStr && FfbmStr[0] != '\0') {
    ParamLen = AsciiStrLen (AndroidBootMode);
    BootConfigFlag = IsAndroidBootParam (AndroidBootMode,
                                ParamLen, HeaderVersion);
    ADD_PARAM_LEN (BootConfigFlag, ParamLen, CmdLineLen,
                                         BootConfigLen);
    AddtoBootConfigList (BootConfigFlag, AndroidBootMode, FfbmStr,
                      BootConfigListHead, ParamLen, AsciiStrLen (FfbmStr));
    ADD_PARAM_LEN (BootConfigFlag, AsciiStrLen (FfbmStr), CmdLineLen,
                                         BootConfigLen);
    /* reduce kernel console messages to speed-up boot */
    ParamLen = AsciiStrLen (LogLevel);
    BootConfigFlag = IsAndroidBootParam (LogLevel,
                         ParamLen, HeaderVersion);
    ADD_PARAM_LEN (BootConfigFlag, ParamLen,
                 CmdLineLen, BootConfigLen);
    AddtoBootConfigList (BootConfigFlag, LogLevel, NULL,
               BootConfigListHead, ParamLen, 0);
  } else if (BatteryStatus &&
             IsChargingScreenEnable () &&
             !Recovery) {
    DEBUG ((EFI_D_INFO, "Device will boot into off mode charging mode\n"));
    PauseAtBootUp = 1;
#ifdef ASUS_AI2205_BUILD
    EnterChargingMode = 1;
#endif
    ParamLen = AsciiStrLen (BatteryChgPause);
    BootConfigFlag = IsAndroidBootParam (BatteryChgPause,
                                ParamLen, HeaderVersion);
    ADD_PARAM_LEN (BootConfigFlag, ParamLen, CmdLineLen,
                                         BootConfigLen);
    AddtoBootConfigList (BootConfigFlag, BatteryChgPause, NULL,
                      BootConfigListHead, ParamLen, 0);
                      
    if(adb_enter_shipping_mode){
      ParamLen = AsciiStrLen (ShippingMode);
      BootConfigFlag = IsAndroidBootParam (ShippingMode,
                                ParamLen, HeaderVersion);
      ADD_PARAM_LEN (BootConfigFlag, ParamLen, CmdLineLen,
                                         BootConfigLen);
      AddtoBootConfigList (BootConfigFlag, ShippingMode, NULL,
                      BootConfigListHead, ParamLen, 0);
    }
  } else if (AlarmBoot) {
    ParamLen = AsciiStrLen (AlarmBootCmdLine);
    BootConfigFlag = IsAndroidBootParam (AlarmBootCmdLine,
                                 ParamLen, HeaderVersion);
    ADD_PARAM_LEN (BootConfigFlag, ParamLen, CmdLineLen,
                                         BootConfigLen);
    AddtoBootConfigList (BootConfigFlag, AlarmBootCmdLine, NULL,
                       BootConfigListHead, ParamLen, 0);
  }

  if (NULL == BoardPlatformChipBaseBand ()) {
    DEBUG ((EFI_D_ERROR, "Invalid BaseBand String\n"));
    FreePool (BootDevBuf);
    BootDevBuf = NULL;
    return EFI_NOT_FOUND;
  }

  ParamLen = AsciiStrLen (BOOT_BASE_BAND);
  BootConfigFlag = IsAndroidBootParam (BOOT_BASE_BAND,
                             ParamLen, HeaderVersion);
  ADD_PARAM_LEN (BootConfigFlag, ParamLen, CmdLineLen,
                                       BootConfigLen);
  AddtoBootConfigList (BootConfigFlag, BOOT_BASE_BAND,
                  BoardPlatformChipBaseBand (),
                  BootConfigListHead, ParamLen,
                  AsciiStrLen (BoardPlatformChipBaseBand ()));
  ADD_PARAM_LEN (BootConfigFlag, AsciiStrLen (BoardPlatformChipBaseBand ()),
                 CmdLineLen, BootConfigLen);

  if (MdtpActive) {
    ParamLen = AsciiStrLen (MdtpActiveFlag);
    BootConfigFlag = IsAndroidBootParam (MdtpActiveFlag,
                               ParamLen, HeaderVersion);
    ADD_PARAM_LEN (BootConfigFlag, ParamLen, CmdLineLen,
                                         BootConfigLen);
    AddtoBootConfigList (BootConfigFlag, MdtpActiveFlag, NULL,
                     BootConfigListHead, ParamLen, 0);
  }
  MultiSlotBoot = PartitionHasMultiSlot ((CONST CHAR16 *)L"boot");
  if (MultiSlotBoot &&
     !IsBootDevImage ()) {
    /* Add additional length for slot suffix */
    ParamLen = AsciiStrLen (AndroidSlotSuffix) + MAX_SLOT_SUFFIX_SZ;
    BootConfigFlag = IsAndroidBootParam (AndroidSlotSuffix,
                               ParamLen, HeaderVersion);
    ADD_PARAM_LEN (BootConfigFlag, ParamLen, CmdLineLen,
                                         BootConfigLen);
  }

  if ((IsBuildAsSystemRootImage (BootParamlistPtr) &&
      !MultiSlotBoot) ||
      (MultiSlotBoot &&
      !IsBootDevImage ())) {
    ParamLen = AsciiStrLen (RootCmdLine);
    BootConfigFlag = IsAndroidBootParam (RootCmdLine,
                            ParamLen, HeaderVersion);
    ADD_PARAM_LEN (BootConfigFlag, ParamLen,
                 CmdLineLen, BootConfigLen);
    AddtoBootConfigList (BootConfigFlag, RootCmdLine, NULL,
                  BootConfigListHead, ParamLen, 0);
    ParamLen = AsciiStrLen (InitCmdline);
    BootConfigFlag = IsAndroidBootParam (InitCmdline,
                            ParamLen, HeaderVersion);
    ADD_PARAM_LEN (BootConfigFlag, ParamLen,
                 CmdLineLen, BootConfigLen);
    AddtoBootConfigList (BootConfigFlag, InitCmdline, NULL,
                  BootConfigListHead, ParamLen, 0);

       if (!IsDynamicPartitionSupport () &&
           !Recovery) {
         ParamLen = AsciiStrLen (SkipRamFs);
         BootConfigFlag = IsAndroidBootParam (SkipRamFs,
                               ParamLen, HeaderVersion);
         ADD_PARAM_LEN (BootConfigFlag, ParamLen,
                      CmdLineLen, BootConfigLen);
         AddtoBootConfigList (BootConfigFlag, SkipRamFs, NULL,
                     BootConfigListHead, ParamLen, 0);
       }
  }

  GetDisplayCmdline ();
  ParamLen = AsciiStrLen (DisplayCmdLine);
  BootConfigFlag = IsAndroidBootParam (DisplayCmdLine,
                             ParamLen, HeaderVersion);
  ADD_PARAM_LEN (BootConfigFlag, ParamLen, CmdLineLen,
                                       BootConfigLen);
  AddtoBootConfigList (BootConfigFlag, DisplayCmdLine, NULL,
                   BootConfigListHead, ParamLen, 0);

  GetHwFenceCmdline ();
  ParamLen = AsciiStrLen (HwFenceCmdLine);
  BootConfigFlag = IsAndroidBootParam (HwFenceCmdLine,
                             ParamLen, HeaderVersion);
  ADD_PARAM_LEN (BootConfigFlag, ParamLen, CmdLineLen,
                                       BootConfigLen);
  AddtoBootConfigList (BootConfigFlag, HwFenceCmdLine, NULL,
                   BootConfigListHead, ParamLen, 0);

  if (!IsLEVariant ()) {
    DtboIdx = GetDtboIdx ();
    if (DtboIdx != INVALID_PTN) {
      AsciiSPrint (DtboIdxStr, sizeof (DtboIdxStr),
                   "%a%d", AndroidBootDtboIdx, DtboIdx);
      ParamLen = AsciiStrLen (DtboIdxStr);
      BootConfigFlag = IsAndroidBootParam (DtboIdxStr,
                             ParamLen, HeaderVersion);
      ADD_PARAM_LEN (BootConfigFlag, ParamLen,
                   CmdLineLen, BootConfigLen);
      AddtoBootConfigList (BootConfigFlag, DtboIdxStr, NULL,
                   BootConfigListHead, ParamLen, 0);
    }

    DtbIdx = GetDtbIdx ();
    if (DtbIdx != INVALID_PTN) {
      AsciiSPrint (DtbIdxStr, sizeof (DtbIdxStr),
                   "%a%d", AndroidBootDtbIdx, DtbIdx);
      ParamLen = AsciiStrLen (DtbIdxStr);
      BootConfigFlag = IsAndroidBootParam (DtbIdxStr,
                            ParamLen, HeaderVersion);
      ADD_PARAM_LEN (BootConfigFlag, ParamLen,
                   CmdLineLen, BootConfigLen);
      AddtoBootConfigList (BootConfigFlag, DtbIdxStr, NULL,
                  BootConfigListHead, ParamLen, 0);
    }
  }

  if (((IsBuildUseRecoveryAsBoot () ||
      IsRecoveryHasNoKernel ()) &&
      IsDynamicPartitionSupport () &&
      !Recovery) ||
      (!MultiSlotBoot &&
       !IsBuildUseRecoveryAsBoot ())) { 
    ParamLen = AsciiStrLen (AndroidBootForceNormalBoot);
    BootConfigFlag = IsAndroidBootParam (AndroidBootForceNormalBoot,
                                           ParamLen, HeaderVersion);
    ADD_PARAM_LEN (BootConfigFlag, ParamLen, CmdLineLen,
                                         BootConfigLen);
    AddtoBootConfigList (BootConfigFlag, AndroidBootForceNormalBoot, NULL,
                    BootConfigListHead, ParamLen, 0);
  }

#ifdef ASUS_BUILD
  // +++ ASUS_BSP : add for fuse blow
  get_fuse_status();
  // +++ ASUS_BSP : add for fuse blow

  // +++ ASUS_BSP : add for asus unlock state
  ASUS_Get_UNLOCK_STATE();
  // --- ASUS_BSP : add for asus unlock state

  if (Recovery) {
    CmdLineLen += AsciiStrLen (RecoveryMode);
  }

  CmdLineLen += AsciiStrLen(cmd_asus_hwid);

  // +++ ASUS_BSP : add for ftm mode
  CmdLineLen += AsciiStrLen(cmd_enable_adb_mode);
  CmdLineLen += AsciiStrLen(cmd_ftm_mode);
  CmdLineLen += AsciiStrLen(cmd_selinux);
  
  ParamLen = AsciiStrLen (cmd_enable_adb_prop);
  BootConfigFlag = IsAndroidBootParam (cmd_enable_adb_prop, ParamLen, HeaderVersion);
  ADD_PARAM_LEN (BootConfigFlag, ParamLen, CmdLineLen, BootConfigLen);
  AddtoBootConfigList (BootConfigFlag, cmd_enable_adb_prop, NULL, BootConfigListHead, ParamLen, 0);
  
  ParamLen = AsciiStrLen (cmd_ftm_mode_prop);
  BootConfigFlag = IsAndroidBootParam (cmd_ftm_mode_prop, ParamLen, HeaderVersion);
  ADD_PARAM_LEN (BootConfigFlag, ParamLen, CmdLineLen, BootConfigLen);
  AddtoBootConfigList (BootConfigFlag, cmd_ftm_mode_prop, NULL, BootConfigListHead, ParamLen, 0);
  
  ParamLen = AsciiStrLen (cmd_selinux_prop);
  BootConfigFlag = IsAndroidBootParam (cmd_selinux_prop, ParamLen, HeaderVersion);
  ADD_PARAM_LEN (BootConfigFlag, ParamLen, CmdLineLen, BootConfigLen);
  AddtoBootConfigList (BootConfigFlag, cmd_selinux_prop, NULL, BootConfigListHead, ParamLen, 0);
  
  ParamLen = AsciiStrLen (cmd_prj_id);
  BootConfigFlag = IsAndroidBootParam (cmd_prj_id, ParamLen, HeaderVersion);
  ADD_PARAM_LEN (BootConfigFlag, ParamLen, CmdLineLen, BootConfigLen);
  AddtoBootConfigList (BootConfigFlag, cmd_prj_id, NULL, BootConfigListHead, ParamLen, 0);
  
  ParamLen = AsciiStrLen (cmd_stage_id);
  BootConfigFlag = IsAndroidBootParam (cmd_stage_id, ParamLen, HeaderVersion);
  ADD_PARAM_LEN (BootConfigFlag, ParamLen, CmdLineLen, BootConfigLen);
  AddtoBootConfigList (BootConfigFlag, cmd_stage_id, NULL, BootConfigListHead, ParamLen, 0);
  
  ParamLen = AsciiStrLen (cmd_sku_id);
  BootConfigFlag = IsAndroidBootParam (cmd_sku_id, ParamLen, HeaderVersion);
  ADD_PARAM_LEN (BootConfigFlag, ParamLen, CmdLineLen, BootConfigLen);
  AddtoBootConfigList (BootConfigFlag, cmd_sku_id, NULL, BootConfigListHead, ParamLen, 0);
  
  ParamLen = AsciiStrLen (cmd_rf_id);
  BootConfigFlag = IsAndroidBootParam (cmd_rf_id, ParamLen, HeaderVersion);
  ADD_PARAM_LEN (BootConfigFlag, ParamLen, CmdLineLen, BootConfigLen);
  AddtoBootConfigList (BootConfigFlag, cmd_rf_id, NULL, BootConfigListHead, ParamLen, 0);
  
  ParamLen = AsciiStrLen (cmd_fp_id);
  BootConfigFlag = IsAndroidBootParam (cmd_fp_id, ParamLen, HeaderVersion);
  ADD_PARAM_LEN (BootConfigFlag, ParamLen, CmdLineLen, BootConfigLen);
  AddtoBootConfigList (BootConfigFlag, cmd_fp_id, NULL, BootConfigListHead, ParamLen, 0);

  ParamLen = AsciiStrLen (cmd_ddr_id);
  BootConfigFlag = IsAndroidBootParam (cmd_ddr_id, ParamLen, HeaderVersion);
  ADD_PARAM_LEN (BootConfigFlag, ParamLen, CmdLineLen, BootConfigLen);
  AddtoBootConfigList (BootConfigFlag, cmd_ddr_id, NULL, BootConfigListHead, ParamLen, 0);
  
  ParamLen = AsciiStrLen (cmd_feature_id);
  BootConfigFlag = IsAndroidBootParam (cmd_feature_id, ParamLen, HeaderVersion);
  ADD_PARAM_LEN (BootConfigFlag, ParamLen, CmdLineLen, BootConfigLen);
  AddtoBootConfigList (BootConfigFlag, cmd_feature_id, NULL, BootConfigListHead, ParamLen, 0);
 
  ParamLen = AsciiStrLen (cmd_jtag_id);
  BootConfigFlag = IsAndroidBootParam (cmd_jtag_id, ParamLen, HeaderVersion);
  ADD_PARAM_LEN (BootConfigFlag, ParamLen, CmdLineLen, BootConfigLen);
  AddtoBootConfigList (BootConfigFlag, cmd_jtag_id, NULL, BootConfigListHead, ParamLen, 0);
  
  ParamLen = AsciiStrLen (cmd_pcb_id);
  BootConfigFlag = IsAndroidBootParam (cmd_pcb_id, ParamLen, HeaderVersion);
  ADD_PARAM_LEN (BootConfigFlag, ParamLen, CmdLineLen, BootConfigLen);
  AddtoBootConfigList (BootConfigFlag, cmd_pcb_id, NULL, BootConfigListHead, ParamLen, 0);
  
  ParamLen = AsciiStrLen (cmd_nfc_id);
  BootConfigFlag = IsAndroidBootParam (cmd_nfc_id, ParamLen, HeaderVersion);
  ADD_PARAM_LEN (BootConfigFlag, ParamLen, CmdLineLen, BootConfigLen);
  AddtoBootConfigList (BootConfigFlag, cmd_nfc_id, NULL, BootConfigListHead, ParamLen, 0);
  
  ParamLen = AsciiStrLen (cmd_lgf_id);
  BootConfigFlag = IsAndroidBootParam (cmd_lgf_id, ParamLen, HeaderVersion);
  ADD_PARAM_LEN (BootConfigFlag, ParamLen, CmdLineLen, BootConfigLen);
  AddtoBootConfigList (BootConfigFlag, cmd_lgf_id, NULL, BootConfigListHead, ParamLen, 0);

  ParamLen = AsciiStrLen (cmd_lgf_con_id);
  BootConfigFlag = IsAndroidBootParam (cmd_lgf_con_id, ParamLen, HeaderVersion);
  ADD_PARAM_LEN (BootConfigFlag, ParamLen, CmdLineLen, BootConfigLen);
  AddtoBootConfigList (BootConfigFlag, cmd_lgf_con_id, NULL, BootConfigListHead, ParamLen, 0);  
  
  ParamLen = AsciiStrLen (cmd_fc_id);
  BootConfigFlag = IsAndroidBootParam (cmd_fc_id, ParamLen, HeaderVersion);
  ADD_PARAM_LEN (BootConfigFlag, ParamLen, CmdLineLen, BootConfigLen);
  AddtoBootConfigList (BootConfigFlag, cmd_fc_id, NULL, BootConfigListHead, ParamLen, 0);  
  
  ParamLen = AsciiStrLen (cmd_upper_id);
  BootConfigFlag = IsAndroidBootParam (cmd_upper_id, ParamLen, HeaderVersion);
  ADD_PARAM_LEN (BootConfigFlag, ParamLen, CmdLineLen, BootConfigLen);
  AddtoBootConfigList (BootConfigFlag, cmd_upper_id, NULL, BootConfigListHead, ParamLen, 0);  
  
  ParamLen = AsciiStrLen (cmd_sub_id);
  BootConfigFlag = IsAndroidBootParam (cmd_sub_id, ParamLen, HeaderVersion);
  ADD_PARAM_LEN (BootConfigFlag, ParamLen, CmdLineLen, BootConfigLen);
  AddtoBootConfigList (BootConfigFlag, cmd_sub_id, NULL, BootConfigListHead, ParamLen, 0);
  
  ParamLen = AsciiStrLen (cmd_valid_image);
  BootConfigFlag = IsAndroidBootParam (cmd_valid_image, ParamLen, HeaderVersion);
  ADD_PARAM_LEN (BootConfigFlag, ParamLen, CmdLineLen, BootConfigLen);
  AddtoBootConfigList (BootConfigFlag, cmd_valid_image, NULL, BootConfigListHead, ParamLen, 0); 

  // +++ ASUS_BPS : add for read country code
  ParamLen = AsciiStrLen (cmd_country_code);
  BootConfigFlag = IsAndroidBootParam (cmd_country_code, ParamLen, HeaderVersion);
  ADD_PARAM_LEN (BootConfigFlag, ParamLen, CmdLineLen, BootConfigLen);
  AddtoBootConfigList (BootConfigFlag, cmd_country_code, NULL, BootConfigListHead, ParamLen, 0);  

  ParamLen = AsciiStrLen (cmd_product_name);
  BootConfigFlag = IsAndroidBootParam (cmd_product_name, ParamLen, HeaderVersion);
  ADD_PARAM_LEN (BootConfigFlag, ParamLen, CmdLineLen, BootConfigLen);
  AddtoBootConfigList (BootConfigFlag, cmd_product_name, NULL, BootConfigListHead, ParamLen, 0);

  // +++ ASUS_BSP : add for boot count
  ParamLen = AsciiStrLen (cmd_boot_count);
  BootConfigFlag = IsAndroidBootParam (cmd_boot_count, ParamLen, HeaderVersion);
  ADD_PARAM_LEN (BootConfigFlag, ParamLen, CmdLineLen, BootConfigLen);
  AddtoBootConfigList (BootConfigFlag, cmd_boot_count, NULL, BootConfigListHead, ParamLen, 0); 

  // +++ ASUS_BSP : add for reboot reason
  if (cmd_boot_reason)
  {
    ParamLen = AsciiStrLen (cmd_boot_reason);
    BootConfigFlag = IsAndroidBootParam (cmd_boot_reason, ParamLen, HeaderVersion);
    ADD_PARAM_LEN (BootConfigFlag, ParamLen, CmdLineLen, BootConfigLen);
    AddtoBootConfigList (BootConfigFlag, cmd_boot_reason, NULL, BootConfigListHead, ParamLen, 0);
  }

  // +++ ASUS_BSP : check if have rawdump partition or not
  ParamLen = AsciiStrLen (cmd_rawdump_en);
  BootConfigFlag = IsAndroidBootParam (cmd_rawdump_en, ParamLen, HeaderVersion);
  ADD_PARAM_LEN (BootConfigFlag, ParamLen, CmdLineLen, BootConfigLen);
  AddtoBootConfigList (BootConfigFlag, cmd_rawdump_en, NULL, BootConfigListHead, ParamLen, 0); 

  CmdLineLen += AsciiStrLen(cmd_asus_info);
  
  ParamLen = AsciiStrLen (cmd_authorized_prop);
  BootConfigFlag = IsAndroidBootParam (cmd_authorized_prop, ParamLen, HeaderVersion);
  ADD_PARAM_LEN (BootConfigFlag, ParamLen, CmdLineLen, BootConfigLen);
  AddtoBootConfigList (BootConfigFlag, cmd_authorized_prop, NULL, BootConfigListHead, ParamLen, 0); 

  CmdLineLen += AsciiStrLen(cmd_unlock);
  CmdLineLen += AsciiStrLen(cmd_update_cmdline);
  
  ParamLen = AsciiStrLen (cmd_cpuid_hash);
  BootConfigFlag = IsAndroidBootParam (cmd_cpuid_hash, ParamLen, HeaderVersion);
  ADD_PARAM_LEN (BootConfigFlag, ParamLen, CmdLineLen, BootConfigLen);
  AddtoBootConfigList (BootConfigFlag, cmd_cpuid_hash, NULL, BootConfigListHead, ParamLen, 0); 
  
  ParamLen = AsciiStrLen (cmd_toolid);
  BootConfigFlag = IsAndroidBootParam (cmd_toolid, ParamLen, HeaderVersion);
  ADD_PARAM_LEN (BootConfigFlag, ParamLen, CmdLineLen, BootConfigLen);
  AddtoBootConfigList (BootConfigFlag, cmd_toolid, NULL, BootConfigListHead, ParamLen, 0); 
  
  CmdLineLen += AsciiStrLen(cmd_fuse_Info);
  
  ParamLen = AsciiStrLen (cmd_fuse_prop);
  BootConfigFlag = IsAndroidBootParam (cmd_fuse_prop, ParamLen, HeaderVersion);
  ADD_PARAM_LEN (BootConfigFlag, ParamLen, CmdLineLen, BootConfigLen);
  AddtoBootConfigList (BootConfigFlag, cmd_fuse_prop, NULL, BootConfigListHead, ParamLen, 0); 
  
  ParamLen = AsciiStrLen (cmd_enable_logcat_asdf);
  BootConfigFlag = IsAndroidBootParam (cmd_enable_logcat_asdf, ParamLen, HeaderVersion);
  ADD_PARAM_LEN (BootConfigFlag, ParamLen, CmdLineLen, BootConfigLen);
  AddtoBootConfigList (BootConfigFlag, cmd_enable_logcat_asdf, NULL, BootConfigListHead, ParamLen, 0);

  // +++ ASUS_BSP : add for check fuse with no rpmb
  CmdLineLen += AsciiStrLen(cmd_fuse_no_rpmb_Info);
  
  ParamLen = AsciiStrLen (cmd_fuse_no_rpmb_prop);
  BootConfigFlag = IsAndroidBootParam (cmd_fuse_no_rpmb_prop, ParamLen, HeaderVersion);
  ADD_PARAM_LEN (BootConfigFlag, ParamLen, CmdLineLen, BootConfigLen);
  AddtoBootConfigList (BootConfigFlag, cmd_fuse_no_rpmb_prop, NULL, BootConfigListHead, ParamLen, 0);
  
  // +++ ASUS_BSP : add for NFC check hardware sku
  ParamLen = AsciiStrLen (cmd_hardware_sku_prop);
  BootConfigFlag = IsAndroidBootParam (cmd_hardware_sku_prop, ParamLen, HeaderVersion);
  ADD_PARAM_LEN (BootConfigFlag, ParamLen, CmdLineLen, BootConfigLen);
  AddtoBootConfigList (BootConfigFlag, cmd_hardware_sku_prop, NULL, BootConfigListHead, ParamLen, 0);
  
  // +++ ASUS_BSP : add for check factory crc
  ParamLen = AsciiStrLen (cmd_factory_crc);
  BootConfigFlag = IsAndroidBootParam (cmd_factory_crc, ParamLen, HeaderVersion);
  ADD_PARAM_LEN (BootConfigFlag, ParamLen, CmdLineLen, BootConfigLen);
  AddtoBootConfigList (BootConfigFlag, cmd_factory_crc, NULL, BootConfigListHead, ParamLen, 0);

  ParamLen = AsciiStrLen (cmd_retry_count);
  BootConfigFlag = IsAndroidBootParam (cmd_retry_count, ParamLen, HeaderVersion);
  ADD_PARAM_LEN (BootConfigFlag, ParamLen, CmdLineLen, BootConfigLen);
  AddtoBootConfigList (BootConfigFlag, cmd_retry_count, NULL, BootConfigListHead, ParamLen, 0);

  // +++ ASUS_BSP : add for ddr info
  /*
  ParamLen = AsciiStrLen (cmd_ddr_manufacturer);
  BootConfigFlag = IsAndroidBootParam (cmd_ddr_manufacturer, ParamLen, HeaderVersion);
  ADD_PARAM_LEN (BootConfigFlag, ParamLen, CmdLineLen, BootConfigLen);
  AddtoBootConfigList (BootConfigFlag, cmd_ddr_manufacturer, NULL, BootConfigListHead, ParamLen, 0);
  
  ParamLen = AsciiStrLen (cmd_ddr_device_type);
  BootConfigFlag = IsAndroidBootParam (cmd_ddr_device_type, ParamLen, HeaderVersion);
  ADD_PARAM_LEN (BootConfigFlag, ParamLen, CmdLineLen, BootConfigLen);
  AddtoBootConfigList (BootConfigFlag, cmd_ddr_device_type, NULL, BootConfigListHead, ParamLen, 0);
  */
  
  if (IsAuthorized_3())
  {
    ParamLen = AsciiStrLen (cmd_watermask_unlock_prop);
    BootConfigFlag = IsAndroidBootParam (cmd_watermask_unlock_prop, ParamLen, HeaderVersion);
    ADD_PARAM_LEN (BootConfigFlag, ParamLen, CmdLineLen, BootConfigLen);
    AddtoBootConfigList (BootConfigFlag, cmd_watermask_unlock_prop, NULL, BootConfigListHead, ParamLen, 0);
  }

  // +++ ASUS_BSP : add for panel uid
  CmdLineLen += AsciiStrLen(cmd_unique_id);

  CmdLineLen += AsciiStrLen(cmd_uart_status);

  // +++ ASUS_BSP : add for WaterMask unlock
  if (IsAuthorized_3())
  {
    CmdLineLen += AsciiStrLen(cmd_watermask_unlock);
  }
  
#endif

  ParamLen = AsciiStrLen (AndroidBootFstabSuffix);
  BootConfigFlag = IsAndroidBootParam (AndroidBootFstabSuffix, ParamLen,
                                       HeaderVersion);
  ADD_PARAM_LEN (BootConfigFlag, ParamLen,
                 CmdLineLen,
                 BootConfigLen);
  GetRootDeviceType (RootDevStr, BOOT_DEV_NAME_SIZE_MAX);
  if (!AsciiStriCmp (FstabSuffixEmmc, RootDevStr)) {
    Param.FstabSuffix = FstabSuffixEmmc;
  } else {
    Param.FstabSuffix = FstabSuffixDefault;
  }
  Param.AndroidBootFstabSuffix = AndroidBootFstabSuffix;
  AddtoBootConfigList (BootConfigFlag, AndroidBootFstabSuffix,
                  Param.FstabSuffix,
                  BootConfigListHead, ParamLen,
                  AsciiStrLen (Param.FstabSuffix));
  ADD_PARAM_LEN (BootConfigFlag, AsciiStrLen (Param.FstabSuffix),
                 CmdLineLen,
                 BootConfigLen);

  Status = GetMemoryLimit (fdt, MemOffAmt);
  /* Don't override "mem" argument if coded into boot image */
  if (Status == EFI_SUCCESS &&
      HaveCmdLine) {
    if (AsciiStrStr (CmdLine, MemOff)) {
      ParamLen = AsciiStrLen (NoPasr);
      BootConfigFlag = IsAndroidBootParam (NoPasr, ParamLen, HeaderVersion);
      ADD_PARAM_LEN (BootConfigFlag, ParamLen, CmdLineLen, BootConfigLen);
      Param.NoPasr = NoPasr;
      Param.MemOffAmt = NULL;
    } else {
      ParamLen = AsciiStrLen (MemOff);
      BootConfigFlag = IsAndroidBootParam (MemOff, ParamLen, HeaderVersion);
      ADD_PARAM_LEN (BootConfigFlag, ParamLen, CmdLineLen, BootConfigLen);
      AddtoBootConfigList (BootConfigFlag, MemOff, NULL, BootConfigListHead,
                                                             ParamLen, 0);
      ParamLen = AsciiStrLen (MemOffAmt);
      BootConfigFlag = IsAndroidBootParam (MemOffAmt, ParamLen, HeaderVersion);
      ADD_PARAM_LEN (BootConfigFlag, ParamLen, CmdLineLen, BootConfigLen);
      AddtoBootConfigList (BootConfigFlag, MemOffAmt, NULL,
                 BootConfigListHead, ParamLen, 0);
      ParamLen = AsciiStrLen (MemHpState);
      BootConfigFlag = IsAndroidBootParam (MemHpState, ParamLen, HeaderVersion);
      ADD_PARAM_LEN (BootConfigFlag, ParamLen, CmdLineLen, BootConfigLen);
      AddtoBootConfigList (BootConfigFlag, MemHpState, NULL,
                 BootConfigListHead, ParamLen, 0);
      ParamLen = AsciiStrLen (MovableNode);
      BootConfigFlag = IsAndroidBootParam (MovableNode,
                 ParamLen, HeaderVersion);
      ADD_PARAM_LEN (BootConfigFlag, ParamLen, CmdLineLen, BootConfigLen);
      AddtoBootConfigList (BootConfigFlag, MovableNode, NULL,
                 BootConfigListHead, ParamLen, 0);

      Param.MemOffAmt = MemOffAmt;
    }
  } else {
    Param.MemOffAmt = NULL;
  }

  if (EarlyEthEnabled ()) {
    GetEarlyEthInfoFromPartition (IPv4AddrBufCmdLine,
                                 IPv6AddrBufCmdLine,
                                 MacEthAddrBufCmdLine);
    CmdLineLen += AsciiStrLen (IPv4AddrBufCmdLine);
    CmdLineLen += AsciiStrLen (IPv6AddrBufCmdLine);
    CmdLineLen += AsciiStrLen (MacEthAddrBufCmdLine);
  }

  /* 1 extra byte for NULL */
  CmdLineLen += 1;

  if (IsHibernationEnabled ()) {
    CmdLineLen += GetResumeCmdLine (&ResumeCmdLine, (CHAR16 *)L"swap_a");
  }

  Param.Recovery = Recovery;
  Param.MultiSlotBoot = MultiSlotBoot;
  Param.AlarmBoot = AlarmBoot;
  Param.MdtpActive = MdtpActive;
  Param.CmdLineLen = CmdLineLen;
  Param.HaveCmdLine = HaveCmdLine;
  Param.PauseAtBootUp = PauseAtBootUp;
  Param.StrSerialNum = StrSerialNum;
  Param.SlotSuffixAscii = SlotSuffixAscii;
  Param.ChipBaseBand = ChipBaseBand;
  Param.DisplayCmdLine = DisplayCmdLine;
  Param.HwFenceCmdLine = HwFenceCmdLine;
  Param.CmdLine = CmdLine;
  Param.AlarmBootCmdLine = AlarmBootCmdLine;
  Param.MdtpActiveFlag = MdtpActiveFlag;
  Param.BatteryChgPause = BatteryChgPause;
  Param.UsbSerialCmdLine = UsbSerialCmdLine;
  Param.VBCmdLine = VBCmdLine;
  Param.LogLevel = LogLevel;
  Param.BootDeviceCmdLine = BootDeviceCmdLine;
  Param.AndroidBootMode = AndroidBootMode;
  Param.BootDevBuf = BootDevBuf;
  Param.FfbmStr = FfbmStr;
  Param.AndroidSlotSuffix = AndroidSlotSuffix;
  Param.SkipRamFs = SkipRamFs;
  Param.RootCmdLine = RootCmdLine;
  Param.InitCmdline = InitCmdline;
  Param.DtboIdxStr = DtboIdxStr;
  Param.DtbIdxStr = DtbIdxStr;
  Param.LEVerityCmdLine = LEVerityCmdLine;
#ifdef ASUS_BUILD
  Param.cmd_asus_hwid = cmd_asus_hwid;  
  // +++ ASUS_BSP : add for ftm mode
  Param.cmd_enable_adb_mode = cmd_enable_adb_mode;
  Param.cmd_enable_adb_prop = cmd_enable_adb_prop;
  Param.cmd_ftm_mode = cmd_ftm_mode;
  Param.cmd_ftm_mode_prop = cmd_ftm_mode_prop;
  Param.cmd_selinux = cmd_selinux;
  Param.cmd_selinux_prop = cmd_selinux_prop;
  //+++ ASUS_BSP : add for ASUS ID value
  Param.cmd_prj_id = cmd_prj_id;
  Param.cmd_stage_id = cmd_stage_id;
  Param.cmd_sku_id = cmd_sku_id;
  Param.cmd_rf_id = cmd_rf_id;
  Param.cmd_fp_id = cmd_fp_id;
  Param.cmd_ddr_id = cmd_ddr_id;
  Param.cmd_feature_id = cmd_feature_id;
  Param.cmd_jtag_id = cmd_jtag_id;
  Param.cmd_pcb_id = cmd_pcb_id;
  Param.cmd_nfc_id = cmd_nfc_id;
  Param.cmd_lgf_con_id = cmd_lgf_con_id;
  Param.cmd_fc_id = cmd_fc_id;
  Param.cmd_upper_id = cmd_upper_id;
  Param.cmd_sub_id = cmd_sub_id;
  Param.cmd_valid_image= cmd_valid_image;
  // +++ ASUS_BPS : add for read country code
  Param.cmd_country_code = cmd_country_code;
  Param.cmd_product_name= cmd_product_name;
  // +++ ASUS_BSP : add for boot count
  Param.cmd_boot_count = cmd_boot_count;
  // +++ ASUS_BSP : add for reboot reason
  Param.cmd_boot_reason = cmd_boot_reason;
  // +++ ASUS_BSP : check if have rawdump partition or not
  Param.cmd_rawdump_en = cmd_rawdump_en;
  Param.cmd_asus_info = cmd_asus_info;
  Param.cmd_authorized_prop = cmd_authorized_prop;
  Param.cmd_unlock = cmd_unlock;
  Param.cmd_update_cmdline = cmd_update_cmdline;
  Param.cmd_cpuid_hash = cmd_cpuid_hash;
  Param.cmd_toolid = cmd_toolid;
  Param.cmd_fuse_Info = cmd_fuse_Info;
  Param.cmd_fuse_prop = cmd_fuse_prop;
  Param.cmd_enable_logcat_asdf = cmd_enable_logcat_asdf;
  // +++ ASUS_BSP : add for check fuse with no rpmb
  Param.cmd_fuse_no_rpmb_Info = cmd_fuse_no_rpmb_Info;
  Param.cmd_fuse_no_rpmb_prop = cmd_fuse_no_rpmb_prop;
  // +++ ASUS_BSP : add for check factory crc
  Param.cmd_factory_crc = cmd_factory_crc;
  Param.cmd_retry_count = cmd_retry_count;
  Param.RecoveryMode = RecoveryMode;
  // +++ ASUS_BSP : add for ddr info
  //Param.cmd_ddr_manufacturer = cmd_ddr_manufacturer;
  //Param.cmd_ddr_device_type = cmd_ddr_device_type;
  // +++ ASUS_BSP : add for panel uid
  Param.cmd_unique_id = cmd_unique_id;
  Param.cmd_uart_status = cmd_uart_status;

  Param.cmd_lgf_id = cmd_lgf_id;
  
  if(adb_enter_shipping_mode){
    Param.ShippingMode = ShippingMode;
  }

  // +++ ASUS_BSP : add for WaterMask unlock
  if (IsAuthorized_3())
  {
    Param.cmd_watermask_unlock = cmd_watermask_unlock;
    Param.cmd_watermask_unlock_prop = cmd_watermask_unlock_prop;
  }
  // --- ASUS_BSP : add for WaterMask unlock
  
  // +++ ASUS_BSP : add for NFC check hardware sku
  Param.cmd_hardware_sku_prop = cmd_hardware_sku_prop;

#endif
  Param.HeaderVersion = HeaderVersion;
  Param.HeaderVersion = HeaderVersion;

  if (EarlyEthEnabled ()) {
    Param.EarlyIPv4CmdLine = IPv4AddrBufCmdLine;
    Param.EarlyIPv6CmdLine = IPv6AddrBufCmdLine;
    Param.EarlyEthMacCmdLine = MacEthAddrBufCmdLine;
  }

  if (IsHibernationEnabled ()) {
    Param.ResumeCmdLine = ResumeCmdLine;
  }

  Status = UpdateCmdLineParams (&Param, FinalCmdLine, BootParamlistPtr);
  if (Status != EFI_SUCCESS) {
    return Status;
  }
  Status = UpdateBootConfigParams (BootConfigListHead,
           BootConfigLen,
           FinalBootConfig, FinalBootConfigLen);
  if (Status != EFI_SUCCESS) {
    return Status;
  }

  ClearBootConfigList (BootConfigListHead);
  if (*FinalCmdLine) {
    DEBUG ((EFI_D_INFO, "Cmdline: %a\n", *FinalCmdLine));
  }
  if (*FinalBootConfig) {
    DEBUG ((EFI_D_INFO, "\n"));
    DEBUG ((EFI_D_INFO, "BootConfig: %a\n", *FinalBootConfig));
  }

  return EFI_SUCCESS;
}
