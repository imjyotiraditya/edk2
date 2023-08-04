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
 * Changes from Qualcomm Innovation Center are provided under the following license:
 *
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted (subject to the limitations in the
 *  disclaimer below) provided that the following conditions are met:
 *
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *
 *      * Redistributions in binary form must reproduce the above
 *        copyright notice, this list of conditions and the following
 *        disclaimer in the documentation and/or other materials provided
 *        with the distribution.
 *
 *      * Neither the name of Qualcomm Innovation Center, Inc. nor the names of its
 *        contributors may be used to endorse or promote products derived
 *        from this software without specific prior written permission.
 *
 *  NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
 *  GRANTED BY THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
 *  HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 *   WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 *  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 *  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 *  GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 *  IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 *  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 *  IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __UPDATECMDLINE_H__
#define __UPDATECMDLINE_H__

#include <Library/DebugLib.h>
#include <Library/Debug.h>
#include <Library/DeviceInfo.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiLib.h>
#include <Uefi.h>
#include "EarlyEthernet.h"

#define BOOT_BASE_BAND " androidboot.baseband="
#define BATT_MIN_VOLT 3200

#define MAX_PATH_SIZE 72
#define SERIAL_NUM_SIZE 64

#define MEM_OFF_SIZE 16
/* set minimum size to 6 GB */
#define MEM_OFF_MIN 0x180000000
#define MB_SIZE 0x100000

typedef struct BootInfo BootInfo;
typedef struct BootLinuxParamlist BootParamlist;

typedef struct UpdateCmdLineParamList {
  BOOLEAN Recovery;
  BOOLEAN MultiSlotBoot;
  BOOLEAN AlarmBoot;
  BOOLEAN MdtpActive;
  UINT32 CmdLineLen;
  UINT32 HaveCmdLine;
  UINT32 PauseAtBootUp;
  CHAR8 *StrSerialNum;
  CHAR8 *SlotSuffixAscii;
  CHAR8 *ChipBaseBand;
  CHAR8 *DisplayCmdLine;
  CHAR8 *HwFenceCmdLine;
  CONST CHAR8 *CmdLine;
  CONST CHAR8 *AlarmBootCmdLine;
  CONST CHAR8 *MdtpActiveFlag;
  CONST CHAR8 *BatteryChgPause;
  CONST CHAR8 *UsbSerialCmdLine;
  CONST CHAR8 *VBCmdLine;
  CONST CHAR8 *LogLevel;
  CONST CHAR8 *NoPasr;
  CONST CHAR8 *BootDeviceCmdLine;
  CONST CHAR8 *AndroidBootMode;
  CONST CHAR8 *AndroidBootFstabSuffix;
  CHAR8 *BootDevBuf;
  CHAR8 *FfbmStr;
  CHAR8 *AndroidSlotSuffix;
  CHAR8 *SkipRamFs;
  CHAR8 *RootCmdLine;
  CHAR8 *InitCmdline;
  CHAR8 *DtboIdxStr;
  CHAR8 *DtbIdxStr;
  CHAR8 *LEVerityCmdLine;
  CHAR8 *FstabSuffix;
  UINT32 HeaderVersion;
  CHAR8 *MemOffAmt;
  CHAR8 *EarlyIPv4CmdLine;
  CHAR8 *EarlyIPv6CmdLine;
  CHAR8 *EarlyEthMacCmdLine;
#ifdef ASUS_BUILD
  CHAR8 *cmd_enable_adb_mode;
  CHAR8 *cmd_enable_adb_prop;
  CHAR8 *cmd_ftm_mode;
  CHAR8 *cmd_ftm_mode_prop;
  CHAR8 *cmd_selinux;
  CHAR8 *cmd_selinux_prop;
  CHAR8 *cmd_soc_id;
  CHAR8 *cmd_prj_id;
  CHAR8 *cmd_stage_id;
  CHAR8 *cmd_sku_id;
  CHAR8 *cmd_nfc_id;
  CHAR8 *cmd_rf_id;
  CHAR8 *cmd_fp_id;
  CHAR8 *cmd_ddr_id;
  CHAR8 *cmd_feature_id;
  CHAR8 *cmd_jtag_id;
  CHAR8 *cmd_pcb_id;
  CHAR8 *cmd_country_code;
  CHAR8 *cmd_product_name;
  CHAR8 *cmd_boot_count;
  CHAR8 *cmd_rawdump_en;
  CHAR8 *cmd_asus_info;
  CHAR8 *cmd_authorized_prop;
  CHAR8 *cmd_unlock;
  CHAR8 *cmd_update_cmdline;
  CHAR8 *cmd_fuse_Info;
  CHAR8 *cmd_fuse_prop;
  CONST CHAR8 *RecoveryMode;
//  CHAR8 *cmd_ddr_manufacturer;
//  CHAR8 *cmd_ddr_device_type;
  CHAR8 *cmd_unique_id; // ASUS BSP Display +++ for panel uid
  CHAR8 *cmd_watermask_unlock; // +++ ASUS_BSP : add for WaterMask unlock
  CHAR8 *cmd_watermask_unlock_prop; // +++ ASUS_BSP : add for WaterMask unlock
  CHAR8 *cmd_cpuid_hash;
  CHAR8 *cmd_toolid;
  CHAR8 *cmd_enable_logcat_asdf; // +++ ASUS_BSP : add for logcat-asdf sevices
  CHAR8 *cmd_lgf_id;
  // +++ ASUS_BSP : add for check fuse with no rpmb
  CHAR8 *cmd_fuse_no_rpmb_Info;
  CHAR8 *cmd_fuse_no_rpmb_prop;
  // +++ ASUS_BSP : add for check factory crc
  CHAR8 *cmd_factory_crc;
  CONST CHAR8 *cmd_boot_reason;
  CHAR8 *cmd_retry_count;
  CHAR8 *cmd_asus_hwid;
  CHAR8 *cmd_uart_status;
  CHAR8 *cmd_lgf_con_id;
  CHAR8 *cmd_fc_id;
  CHAR8 *cmd_upper_id;
  CHAR8 *cmd_sub_id;
  CHAR8 *cmd_valid_image;
  // +++ ASUS_BSP : add for NFC check hardware sku
  CHAR8 *cmd_hardware_sku_prop;
  CONST CHAR8 *ShippingMode;
#endif
  CHAR8 *ResumeCmdLine;
} UpdateCmdLineParamList;


typedef struct BootConfigParamNode {
  CHAR8 *param;
  UINT32 ParamLen;
  LIST_ENTRY ListNode;
} BootConfigParamNode;

EFI_STATUS
UpdateCmdLine (BootParamlist *BootParamlistPtr,
               CHAR8 *FfbmStr,
               BOOLEAN Recovery,
               BOOLEAN FlashlessBoot,
               BOOLEAN AlarmBoot,
               CONST CHAR8 *VBCmdLine,
               UINT32 HeaderVersion);
BOOLEAN
TargetBatterySocOk (UINT32 *BatteryVoltage);

UINT32
GetSystemPath (CHAR8 **SysPath,
               BOOLEAN MultiSlotBoot,
               BOOLEAN BootIntoRecovery,
               CHAR16 *ReqPartition,
               CHAR8 *Key,
               BOOLEAN FlashlessBoot);

EFI_STATUS
TargetPauseForBatteryCharge (BOOLEAN *BatteryStatus);
BOOLEAN IsAndroidBootParam (CONST CHAR8 *param,
                            UINT32 ParamLen,
                            UINT32 HeaderVersion);

VOID
AddtoBootConfigList (BOOLEAN BootConfigFlag,
                CONST CHAR8 *ParamKey,
                CONST CHAR8 *ParamValue,
                LIST_ENTRY *list,
                UINT32 ParamKeyLen,
                UINT32 ParamValueLen);

UINT32
GetResumeCmdLine (CHAR8 **ResumeCmdLine, CHAR16 *ReqPartition);
#endif
