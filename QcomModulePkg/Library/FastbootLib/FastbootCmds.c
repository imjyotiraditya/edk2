/** @file

  Copyright (c) 2013-2014, ARM Ltd. All rights reserved.<BR>

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD
License
  which accompanies this distribution.  The full text of the license may be
found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

/*
 * Copyright (c) 2009, Google Inc.
 * All rights reserved.
 *
 * Copyright (c) 2015 - 2021, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 * * Redistributions of source code must retain the above copyright
 *  notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 *  with the distribution.
 *   * Neither the name of The Linux Foundation nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

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

#include <Library/BaseLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/DebugLib.h>
#include <Library/Debug.h>
#include <Library/DeviceInfo.h>
#include <Library/DevicePathLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/MenuKeysDetection.h>
#include <Library/PartitionTableUpdate.h>
#include <Library/PcdLib.h>
#include <Library/PrintLib.h>
#include <Library/ThreadStack.h>
#include <Library/UefiApplicationEntryPoint.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiLib.h>
#include <Library/UefiRuntimeServicesTableLib.h>
#include <Library/UnlockMenu.h>
#include <Library/BootLinux.h>
#include <Uefi.h>

#include <Guid/EventGroup.h>

#include <Protocol/BlockIo.h>
#include <Protocol/DiskIo.h>
#include <Protocol/EFIUsbDevice.h>
#include <Protocol/EFIUbiFlasher.h>
#include <Protocol/SimpleTextIn.h>
#include <Protocol/SimpleTextOut.h>
#include <Protocol/EFIDisplayUtils.h>

#include "AutoGen.h"
#include "BootImage.h"
#include "BootLinux.h"
#include "BootStats.h"
#include "FastbootCmds.h"
#include "FastbootMain.h"
#include "LinuxLoaderLib.h"
#include "MetaFormat.h"
#include "SparseFormat.h"
#include "Recovery.h"

#ifdef ASUS_AI2205_BUILD
#include "libavb/libavb.h"

BOOLEAN AllowParallelDownloadFlash = TRUE;
BOOLEAN g_allow_flash_and_erase_asdf = FALSE;
BOOLEAN ALLOW_FLASH_AFTER_CHECK_SOC = FALSE;

#define SECBOOT_FUSE 0
#define SHK_FUSE 1
#define DEBUG_DISABLED_FUSE 2
#define ANTI_ROLLBACK_FUSE 3
#define FEC_ENABLED_FUSE 4
#define RPMB_ENABLED_FUSE 5
#define DEBUG_RE_ENABLED_FUSE 6
#define MISC_DEBUG_FUSE 7
#define TZ_DEBUG_FUSE 8
#define MSS_DEBUG_FUSE 9
#define CP_DEBUG_FUSE 10

#define CHECK_BIT(var, pos) ((var) & (1 << (pos)))

STATIC CONST CHAR16 *VerifiedBootPartition[] = {
	L"boot",
	L"vendor_boot",
	L"dtbo",
	L"system",
	L"vendor",
	L"vbmeta",
	L"vbmeta_system",
	L"super"
};

STATIC CONST CHAR16 *UserFlashPartition[] = {
	L"asuskey2",
	L"asuskey3",
	L"sig",
	L"sha256",
	L"signature",
	L"partition",
	L"userdata",
	L"asdf",
	L"CRC",
	L"APD",
	L"ADF",
	L"metadata",
	L"apdp",
	L"batinfo",
	L"gpt",
	L"logbuf"
};
STATIC CONST CHAR16 *UserErasePartition[] = {
	L"userdata",
	L"asdf",
	L"super",
	L"ddr",
	L"APD",
	L"ADF",
	L"xrom",
	L"asuskey3",
	L"asuskey5",
	L"metadata",
	L"apdp",
	L"batinfo",
	L"gpt",
	L"spunvm",
	L"logbuf",
	L"misc",
};
#endif

#ifdef ASUS_BUILD
#include "abl.h"

// +++ ASUS_BSP : add for user unlock
#include <Protocol/EFISecRSA.h>
#include "SecRSATestApp.h"
#include "VerifiedBoot.h"
// --- ASUS_BSP : add for user unlock

// +++ ASUS_BSP : add for fastboot permission
#define ASUS_FASTBOOT_PERMISSION (!IsSecureBootEnabled() || IsAuthorized() || IsAuthorized_2())
extern unsigned is_ftm_mode(void);

extern char cmd_stage_id[64]; // +++ ASUS_BSP : add for force hwid
extern char asus_project_info[32]; // +++ ASUS_BSP : update project name
extern char cid_name[32];

STATIC int IsRawFlash = 0;
STATIC int IsRawFlash_gpt = 0;
STATIC BOOLEAN IsASUSRawFlash = FALSE;

// +++ ASUS_BSP : add for ckeck CRC_partiiton:0~6
unsigned int g_calculate_gpt0_crc = 0xffffffff;
unsigned int g_calculate_gpt1_crc = 0xffffffff;
unsigned int g_calculate_gpt2_crc = 0xffffffff;
unsigned int g_calculate_gpt3_crc = 0xffffffff;
unsigned int g_calculate_gpt4_crc = 0xffffffff;
unsigned int g_calculate_gpt5_crc = 0xffffffff;
unsigned int g_calculate_gpt6_crc = 0xffffffff;
unsigned int g_calculate_gpt7_crc = 0xffffffff;
// --- ASUS_BSP : add for ckeck CRC_partiiton:0~6

// +++ ASUS_BSP : add for check crc
unsigned int g_calculate_crc = 0xffffffff;
unsigned int g_flash_crc = 0xffffffff;
// +++ ASUS_BSP : add for check crc

// +++ ASUS_BSP : add for ASUS dongle unlock
#include <Md5.h>
int ASUS_GEN_RANDOM_FLAG = 0;
char rand_number[34]={};
//char *rand_number = "08000260013335480527294877279139";
char calculate_hash_buf[65]={};
char mmc_hash_buf[65]={};
char key[9]="12345678";
char key2[9]="28825252";
int hash_buf_size = 64;

typedef unsigned short uint16;

typedef enum
{
	A68 = 1,
	Undefine_operator1,
	Undefine_operator2,
	Undefine_operator3,
	MAX_Algo
}dongleAlgoType;
// --- ASUS_BSP : add for ASUS dongle unlock

// +++ ASUS_BSP : add for update cmdline
char cmd_update_cmdline[64] = {0};
extern char cmd_prj_id[64];
extern char cmd_stage_id[64];
extern char cmd_sku_id[64];
extern char cmd_ddr_id[64];
extern char cmd_nfc_id[64];
extern char cmd_rf_id[64];
extern char cmd_fp_id[64];
extern char cmd_country_code[128];
extern char cmd_lgf_id[64];
extern char cmd_lgf_con_id[64];
extern char cmd_fc_id[64];
extern char cmd_upper_id[64];
extern char cmd_sub_id[64];
extern char* asus_strtok ( char* str, const char* delimiters);
// --- ASUS_BSP : add for update cmdline

BOOLEAN g_asus_slot_b_enable = FALSE; // +++ ASUS_BSP : add for enable flash raw in slot_b
#endif

#ifdef ASUS_BUILD
STATIC VOID CmdOemSha256(CONST CHAR8 * arg, VOID * data, UINT32 sz); // +++ ASUS_BSP : add for FRP unlck
void random_num_generator(char *rand_num);
STATIC struct GetVarPartitionInfo part_info[] = {
#ifdef F2FS_BUILD
    {"userdata", "partition-size:", "partition-type:", "", "f2fs"},
#else
    {"userdata", "partition-size:", "partition-type:", "", "ext4"},
#endif
    {"cache", "partition-size:", "partition-type:", "", "ext4"},
    {"asdf", "partition-size:", "partition-type:", "", "ext4"},
    {"APD", "partition-size:", "partition-type:", "", "ext4"},
    {"ADF", "partition-size:", "partition-type:", "", "ext4"},
    {"xrom", "partition-size:", "partition-type:", "", "ext4"},
    {"system", "partition-size:", "partition-type:", "", "ext4"},
    {"vendor", "partition-size:", "partition-type:", "", "ext4"},
    {"metadata", "partition-size:", "partition-type:", "", "f2fs"},
    {"batinfo", "partition-size:", "partition-type:", "", "ext4"},
#ifdef ASUS_AI2205_BUILD
    {"logbuf", "partition-size:", "partition-type:", "", "ext4"},
#endif
};
#else
STATIC struct GetVarPartitionInfo part_info[] = {
    {"system", "partition-size:", "partition-type:", "", "ext4"},
    {"userdata", "partition-size:", "partition-type:", "", "ext4"},
    {"cache", "partition-size:", "partition-type:", "", "ext4"},
};
#endif

STATIC struct GetVarPartitionInfo PublishedPartInfo[MAX_NUM_PARTITIONS];

#ifdef ENABLE_UPDATE_PARTITIONS_CMDS
STATIC CONST CHAR16 *CriticalPartitions[] = {
    L"abl",  L"rpm",        L"tz",      L"sdi",       L"xbl",       L"hyp",
    L"pmic", L"bootloader", L"devinfo", L"partition", L"devcfg",    L"ddr",
    L"frp",  L"cdt",        L"cmnlib",  L"cmnlib64",  L"keymaster", L"mdtp",
    L"aop",  L"multiimgoem", L"secdata", L"imagefv",  L"qupfw", L"uefisecapp"};

STATIC BOOLEAN
IsCriticalPartition (CHAR16 *PartitionName);

STATIC CONST CHAR16 *VirtualAbCriticalPartitions[] = {
    L"misc",  L"metadata",  L"userdata",  L"asdf"};

STATIC BOOLEAN
CheckVirtualAbCriticalPartition (CHAR16 *PartitionName);
#endif

STATIC FASTBOOT_VAR *Varlist;
STATIC BOOLEAN Finished = FALSE;
STATIC CHAR8 StrSerialNum[MAX_RSP_SIZE];
STATIC CHAR8 FullProduct[MAX_RSP_SIZE];
STATIC CHAR8 StrVariant[MAX_RSP_SIZE];
STATIC CHAR8 StrBatteryVoltage[MAX_RSP_SIZE];
STATIC CHAR8 StrBatterySocOk[MAX_RSP_SIZE];
STATIC CHAR8 ChargeScreenEnable[MAX_RSP_SIZE];
STATIC CHAR8 OffModeCharge[MAX_RSP_SIZE];
STATIC CHAR8 StrSocVersion[MAX_RSP_SIZE];
STATIC CHAR8 LogicalBlkSizeStr[MAX_RSP_SIZE];
STATIC CHAR8 EraseBlkSizeStr[MAX_RSP_SIZE];
STATIC CHAR8 MaxDownloadSizeStr[MAX_RSP_SIZE];
STATIC CHAR8 SnapshotMergeState[MAX_RSP_SIZE];

struct GetVarSlotInfo {
  CHAR8 SlotSuffix[MAX_SLOT_SUFFIX_SZ];
  CHAR8 SlotSuccessfulVar[SLOT_ATTR_SIZE];
  CHAR8 SlotUnbootableVar[SLOT_ATTR_SIZE];
  CHAR8 SlotRetryCountVar[SLOT_ATTR_SIZE];
  CHAR8 SlotSuccessfulVal[ATTR_RESP_SIZE];
  CHAR8 SlotUnbootableVal[ATTR_RESP_SIZE];
  CHAR8 SlotRetryCountVal[ATTR_RESP_SIZE];
};

STATIC struct GetVarSlotInfo *BootSlotInfo = NULL;
STATIC CHAR8 SlotSuffixArray[SLOT_SUFFIX_ARRAY_SIZE];
STATIC CHAR8 SlotCountVar[ATTR_RESP_SIZE];
STATIC CHAR8 CurrentSlotFB[MAX_SLOT_SUFFIX_SZ];

/*Note: This needs to be used only when Slot already has prefix "_" */
#define SKIP_FIRSTCHAR_IN_SLOT_SUFFIX(Slot)                                    \
  do {                                                                         \
    int i = 0;                                                                 \
    do {                                                                       \
      Slot[i] = Slot[i + 1];                                                   \
      i++;                                                                     \
    } while (i < MAX_SLOT_SUFFIX_SZ - 1);                                      \
  } while (0);

#define MAX_DISPLAY_PANEL_OVERRIDE 256

/*This variable is used to skip populating the FastbootVar
 * When PopulateMultiSlotInfo called while flashing each Lun
 */
STATIC BOOLEAN InitialPopulate = FALSE;
STATIC UINT32 SlotCount;
extern struct PartitionEntry PtnEntries[MAX_NUM_PARTITIONS];

STATIC ANDROID_FASTBOOT_STATE mState = ExpectCmdState;
/* When in ExpectDataState, the number of bytes of data to expect: */
STATIC UINT64 mNumDataBytes;
STATIC UINT64 mFlashNumDataBytes;
/* .. and the number of bytes so far received this data phase */
STATIC UINT64 mBytesReceivedSoFar;
/*  and the buffer to save data into */
STATIC UINT8 *mDataBuffer = NULL;
/*  and the offset for usb to save data into */
STATIC UINT8 *mFlashDataBuffer = NULL;
STATIC UINT8 *mUsbDataBuffer = NULL;

STATIC EFI_KERNEL_PROTOCOL  *KernIntf = NULL;
STATIC BOOLEAN IsMultiThreadSupported = FALSE;
STATIC BOOLEAN IsFlashComplete = TRUE;
STATIC LockHandle *LockDownload;
STATIC LockHandle *LockFlash;

STATIC EFI_STATUS FlashResult = EFI_SUCCESS;
#ifdef ENABLE_UPDATE_PARTITIONS_CMDS
STATIC EFI_EVENT UsbTimerEvent;
#endif

STATIC UINT64 MaxDownLoadSize = 0;

STATIC INT32 Lun = NO_LUN;
STATIC BOOLEAN LunSet;

STATIC FASTBOOT_CMD *cmdlist;
STATIC UINT32 IsAllowUnlock;

STATIC EFI_STATUS
FastbootCommandSetup (VOID *Base, UINT64 Size);
STATIC VOID
AcceptCmd (IN UINT64 Size, IN CHAR8 *Data);
STATIC VOID
AcceptCmdHandler (IN EFI_EVENT Event, IN VOID *Context);

#define NAND_PAGES_PER_BLOCK 64

#define UBI_HEADER_MAGIC "UBI#"
#define UBI_NUM_IMAGES 1
typedef struct UbiHeader {
  CHAR8 HdrMagic[4];
} UbiHeader_t;

typedef struct {
  UINT64 Size;
  VOID *Data;
} CmdInfo;

typedef struct {
  CHAR16 PartitionName[MAX_GPT_NAME_SIZE];
  UINT32 PartitionSize;
  UINT8 *FlashDataBuffer;
  UINT64 FlashNumDataBytes;
} FlashInfo;

STATIC BOOLEAN FlashSplitNeeded;
STATIC BOOLEAN UsbTimerStarted;

BOOLEAN IsUsbTimerStarted (VOID) {
  return UsbTimerStarted;
}

BOOLEAN IsFlashSplitNeeded (VOID)
{
  if (IsUseMThreadParallel ()) {
    return FlashSplitNeeded;
  } else {
    return UsbTimerStarted;
  }
}

BOOLEAN FlashComplete (VOID)
{
  return IsFlashComplete;
}

#ifdef DISABLE_PARALLEL_DOWNLOAD_FLASH
BOOLEAN IsDisableParallelDownloadFlash (VOID)
{
  return TRUE;
}
#else
BOOLEAN IsDisableParallelDownloadFlash (VOID)
{
  return FALSE;
}
#endif

/* Clean up memory for the getvar variables during exit */
STATIC EFI_STATUS FastbootUnInit (VOID)
{
  FASTBOOT_VAR *Var;
  FASTBOOT_VAR *VarPrev = NULL;

  for (Var = Varlist; Var && Var->next; Var = Var->next) {
    if (VarPrev) {
      FreePool (VarPrev);
      VarPrev = NULL;
    }
    VarPrev = Var;
  }
  if (Var) {
    FreePool (Var);
    Var = NULL;
  }

  return EFI_SUCCESS;
}

/* Publish a variable readable by the built-in getvar command
 * These Variables must not be temporary, shallow copies are used.
 */
STATIC VOID
FastbootPublishVar (IN CONST CHAR8 *Name, IN CONST CHAR8 *Value)
{
  FASTBOOT_VAR *Var;
  Var = AllocateZeroPool (sizeof (*Var));
  if (Var) {
    Var->next = Varlist;
    Varlist = Var;
    Var->name = Name;
    Var->value = Value;
  } else {
    DEBUG ((EFI_D_VERBOSE,
            "Failed to publish a variable readable(%a): malloc error!\n",
            Name));
  }
}

/* Returns the Remaining amount of bytes expected
 * This lets us bypass ZLT issues
 */
UINTN GetXfrSize (VOID)
{
  UINTN BytesLeft = mNumDataBytes - mBytesReceivedSoFar;
  if ((mState == ExpectDataState) && (BytesLeft < USB_BUFFER_SIZE))
    return BytesLeft;

  return USB_BUFFER_SIZE;
}

/* Acknowlege to host, INFO, OKAY and FAILURE */
STATIC VOID
FastbootAck (IN CONST CHAR8 *code, CONST CHAR8 *Reason)
{
  if (Reason == NULL)
    Reason = "";

  AsciiSPrint (GetFastbootDeviceData ()->gTxBuffer, MAX_RSP_SIZE, "%a%a", code,
               Reason);
  GetFastbootDeviceData ()->UsbDeviceProtocol->Send (
      ENDPOINT_OUT, AsciiStrLen (GetFastbootDeviceData ()->gTxBuffer),
      GetFastbootDeviceData ()->gTxBuffer);
  DEBUG ((EFI_D_VERBOSE, "Sending %d:%a\n",
          AsciiStrLen (GetFastbootDeviceData ()->gTxBuffer),
          GetFastbootDeviceData ()->gTxBuffer));
}

VOID
FastbootFail (IN CONST CHAR8 *Reason)
{
  FastbootAck ("FAIL", Reason);
}

VOID
FastbootInfo (IN CONST CHAR8 *Info)
{
  FastbootAck ("INFO", Info);
}

VOID
FastbootOkay (IN CONST CHAR8 *info)
{
  FastbootAck ("OKAY", info);
}

VOID PartitionDump (VOID)
{
  EFI_STATUS Status;
  EFI_PARTITION_ENTRY *PartEntry;
  UINT16 i;
  UINT32 j;
  /* By default the LunStart and LunEnd would point to '0' and max value */
  UINT32 LunStart = 0;
  UINT32 LunEnd = GetMaxLuns ();

  /* If Lun is set in the Handle flash command then find the block io for that
   * lun */
  if (LunSet) {
    LunStart = Lun;
    LunEnd = Lun + 1;
  }
  for (i = LunStart; i < LunEnd; i++) {
    for (j = 0; j < Ptable[i].MaxHandles; j++) {
      Status =
          gBS->HandleProtocol (Ptable[i].HandleInfoList[j].Handle,
                               &gEfiPartitionRecordGuid, (VOID **)&PartEntry);
      if (EFI_ERROR (Status)) {
        DEBUG ((EFI_D_VERBOSE, "Error getting the partition record for Lun %d "
                               "and Handle: %d : %r\n",
                i, j, Status));
        continue;
      }
      DEBUG ((EFI_D_INFO, "Name:[%s] StartLba: %u EndLba:%u\n",
              PartEntry->PartitionName, PartEntry->StartingLBA,
              PartEntry->EndingLBA));
    }
  }
}

EFI_STATUS
PartitionGetInfo (IN CHAR16 *PartitionName,
                  OUT EFI_BLOCK_IO_PROTOCOL **BlockIo,
                  OUT EFI_HANDLE **Handle)
{
  EFI_STATUS Status;
  EFI_PARTITION_ENTRY *PartEntry;
  UINT16 i;
  UINT32 j;
  /* By default the LunStart and LunEnd would point to '0' and max value */
  UINT32 LunStart = 0;
  UINT32 LunEnd = GetMaxLuns ();

  /* If Lun is set in the Handle flash command then find the block io for that
   * lun */
  if (LunSet) {
    LunStart = Lun;
    LunEnd = Lun + 1;
  }
  for (i = LunStart; i < LunEnd; i++) {
    for (j = 0; j < Ptable[i].MaxHandles; j++) {
      Status =
          gBS->HandleProtocol (Ptable[i].HandleInfoList[j].Handle,
                               &gEfiPartitionRecordGuid, (VOID **)&PartEntry);
      if (EFI_ERROR (Status)) {
        continue;
      }
      if (!(StrCmp (PartitionName, PartEntry->PartitionName))) {
        *BlockIo = Ptable[i].HandleInfoList[j].BlkIo;
        *Handle = Ptable[i].HandleInfoList[j].Handle;
        return Status;
      }
    }
  }

  DEBUG ((EFI_D_ERROR, "Partition not found : %s\n", PartitionName));
  return EFI_NOT_FOUND;
}

STATIC VOID FastbootPublishSlotVars (VOID)
{
  UINT32 i;
  UINT32 j;
  CHAR8 *Suffix = NULL;
  UINT32 PartitionCount = 0;
  CHAR8 PartitionNameAscii[MAX_GPT_NAME_SIZE];
  UINT32 RetryCount = 0;
  BOOLEAN Set = FALSE;

  GetPartitionCount (&PartitionCount);
  /*Scan through partition entries, populate the attributes*/
  for (i = 0, j = 0; i < PartitionCount && j < SlotCount; i++) {
    UnicodeStrToAsciiStr (PtnEntries[i].PartEntry.PartitionName,
                          PartitionNameAscii);

    if (!(AsciiStrnCmp (PartitionNameAscii, "boot", AsciiStrLen ("boot")))) {
      Suffix = PartitionNameAscii + AsciiStrLen ("boot_");

      AsciiStrnCpyS (BootSlotInfo[j].SlotSuffix, MAX_SLOT_SUFFIX_SZ, Suffix,
                     AsciiStrLen (Suffix));
      AsciiStrnCpyS (BootSlotInfo[j].SlotSuccessfulVar, SLOT_ATTR_SIZE,
                     "slot-successful:", AsciiStrLen ("slot-successful:"));
      Set = PtnEntries[i].PartEntry.Attributes & PART_ATT_SUCCESSFUL_VAL
                ? TRUE
                : FALSE;
      AsciiStrnCpyS (BootSlotInfo[j].SlotSuccessfulVal, ATTR_RESP_SIZE,
                     Set ? "yes" : "no",
                     Set ? AsciiStrLen ("yes") : AsciiStrLen ("no"));
      AsciiStrnCatS (BootSlotInfo[j].SlotSuccessfulVar, SLOT_ATTR_SIZE, Suffix,
                     AsciiStrLen (Suffix));
      FastbootPublishVar (BootSlotInfo[j].SlotSuccessfulVar,
                          BootSlotInfo[j].SlotSuccessfulVal);

      AsciiStrnCpyS (BootSlotInfo[j].SlotUnbootableVar, SLOT_ATTR_SIZE,
                     "slot-unbootable:", AsciiStrLen ("slot-unbootable:"));
      Set = PtnEntries[i].PartEntry.Attributes & PART_ATT_UNBOOTABLE_VAL
                ? TRUE
                : FALSE;
      AsciiStrnCpyS (BootSlotInfo[j].SlotUnbootableVal, ATTR_RESP_SIZE,
                     Set ? "yes" : "no",
                     Set ? AsciiStrLen ("yes") : AsciiStrLen ("no"));
      AsciiStrnCatS (BootSlotInfo[j].SlotUnbootableVar, SLOT_ATTR_SIZE, Suffix,
                     AsciiStrLen (Suffix));
      FastbootPublishVar (BootSlotInfo[j].SlotUnbootableVar,
                          BootSlotInfo[j].SlotUnbootableVal);

      AsciiStrnCpyS (BootSlotInfo[j].SlotRetryCountVar, SLOT_ATTR_SIZE,
                     "slot-retry-count:", AsciiStrLen ("slot-retry-count:"));
      RetryCount =
          (PtnEntries[i].PartEntry.Attributes & PART_ATT_MAX_RETRY_COUNT_VAL) >>
          PART_ATT_MAX_RETRY_CNT_BIT;
      AsciiSPrint (BootSlotInfo[j].SlotRetryCountVal, ATTR_RESP_SIZE, "%llu",
                   RetryCount);
      AsciiStrnCatS (BootSlotInfo[j].SlotRetryCountVar, SLOT_ATTR_SIZE, Suffix,
                     AsciiStrLen (Suffix));
      FastbootPublishVar (BootSlotInfo[j].SlotRetryCountVar,
                          BootSlotInfo[j].SlotRetryCountVal);
      j++;
    }
  }
  FastbootPublishVar ("has-slot:boot", "yes");
  UnicodeStrToAsciiStr (GetCurrentSlotSuffix ().Suffix, CurrentSlotFB);

  /* Here CurrentSlotFB will only have value of "_a" or "_b".*/
  SKIP_FIRSTCHAR_IN_SLOT_SUFFIX (CurrentSlotFB);

  FastbootPublishVar ("current-slot", CurrentSlotFB);
  FastbootPublishVar ("has-slot:system",
                      PartitionHasMultiSlot ((CONST CHAR16 *)L"system") ? "yes"
                                                                        : "no");
  FastbootPublishVar ("has-slot:modem",
                      PartitionHasMultiSlot ((CONST CHAR16 *)L"modem") ? "yes"
                                                                       : "no");
  return;
}

/*Function to populate attribute fields
 *Note: It traverses through the partition entries structure,
 *populates has-slot, slot-successful,slot-unbootable and
 *slot-retry-count attributes of the boot slots.
 */
STATIC VOID PopulateMultislotMetadata (VOID)
{
  UINT32 i;
  UINT32 j;
  UINT32 PartitionCount = 0;
  CHAR8 *Suffix = NULL;
  CHAR8 PartitionNameAscii[MAX_GPT_NAME_SIZE];

  GetPartitionCount (&PartitionCount);
  if (!InitialPopulate) {
    /*Traverse through partition entries,count matching slots with boot */
    for (i = 0; i < PartitionCount; i++) {
      UnicodeStrToAsciiStr (PtnEntries[i].PartEntry.PartitionName,
                            PartitionNameAscii);
      if (!(AsciiStrnCmp (PartitionNameAscii, "boot", AsciiStrLen ("boot")))) {
        SlotCount++;
        Suffix = PartitionNameAscii + AsciiStrLen ("boot");
        if (!AsciiStrStr (SlotSuffixArray, Suffix)) {
          AsciiStrnCatS (SlotSuffixArray, sizeof (SlotSuffixArray), Suffix,
                         AsciiStrLen (Suffix));
          AsciiStrnCatS (SlotSuffixArray, sizeof (SlotSuffixArray), ",",
                         AsciiStrLen (","));
        }
      }
    }

    AsciiSPrint (SlotCountVar, sizeof (SlotCountVar), "%d", SlotCount);
    FastbootPublishVar ("slot-count", SlotCountVar);

    /*Allocate memory for available number of slots*/
    BootSlotInfo = AllocateZeroPool (
                         SlotCount * sizeof (struct GetVarSlotInfo));
    if (BootSlotInfo == NULL) {
      DEBUG ((EFI_D_ERROR, "Unable to allocate memory for BootSlotInfo\n"));
      return;
    }
    FastbootPublishSlotVars ();
    InitialPopulate = TRUE;
  } else {
    /*While updating gpt from fastboot dont need to populate all the variables
     * as above*/
    for (i = 0; i < SlotCount; i++) {
      AsciiStrnCpyS (BootSlotInfo[i].SlotSuccessfulVal,
                     sizeof (BootSlotInfo[i].SlotSuccessfulVal), "no",
                     AsciiStrLen ("no"));
      AsciiStrnCpyS (BootSlotInfo[i].SlotUnbootableVal,
                     sizeof (BootSlotInfo[i].SlotUnbootableVal), "no",
                     AsciiStrLen ("no"));
      AsciiSPrint (BootSlotInfo[i].SlotRetryCountVal,
                   sizeof (BootSlotInfo[j].SlotRetryCountVal), "%d",
                   MAX_RETRY_COUNT);
    }
  }
  return;
}

#ifdef ENABLE_UPDATE_PARTITIONS_CMDS
/* Helper function to write data to disk */
STATIC EFI_STATUS
WriteToDisk (IN EFI_BLOCK_IO_PROTOCOL *BlockIo,
             IN EFI_HANDLE *Handle,
             IN VOID *Image,
             IN UINT64 Size,
             IN UINT64 offset)
{
  return WriteBlockToPartitionNoFlush (BlockIo, Handle, offset, Size, Image);
}

STATIC BOOLEAN
GetPartitionHasSlot (CHAR16 *PartitionName,
                     UINT32 PnameMaxSize,
                     CHAR16 *SlotSuffix,
                     UINT32 SlotSuffixMaxSize)
{
  INT32 Index = INVALID_PTN;
  BOOLEAN HasSlot = FALSE;
  Slot CurrentSlot;

  Index = GetPartitionIndex (PartitionName);
  if (Index == INVALID_PTN) {
    CurrentSlot = GetCurrentSlotSuffix ();
    StrnCpyS (SlotSuffix, SlotSuffixMaxSize, CurrentSlot.Suffix,
              StrLen (CurrentSlot.Suffix));
    StrnCatS (PartitionName, PnameMaxSize, CurrentSlot.Suffix,
              StrLen (CurrentSlot.Suffix));
    HasSlot = TRUE;
  } else {
    /*Check for _a or _b slots, if available then copy to SlotSuffix Array*/
    if (StrStr (PartitionName, (CONST CHAR16 *)L"_a") ||
        StrStr (PartitionName, (CONST CHAR16 *)L"_b")) {
      StrnCpyS (SlotSuffix, SlotSuffixMaxSize,
                (PartitionName + (StrLen (PartitionName) - 2)), 2);
      HasSlot = TRUE;
    }
  }
  return HasSlot;
}

STATIC EFI_STATUS
HandleChunkTypeRaw (sparse_header_t *sparse_header,
        chunk_header_t *chunk_header,
        VOID **Image,
        SparseImgParam *SparseImgData)
{
  EFI_STATUS Status;

  if (sparse_header == NULL ||
      chunk_header == NULL ||
      *Image == NULL ||
      SparseImgData == NULL) {
    DEBUG ((EFI_D_ERROR, "Invalid input Parameters\n"));
    return EFI_INVALID_PARAMETER;
  }

  if ((UINT64)chunk_header->total_sz !=
      ((UINT64)sparse_header->chunk_hdr_sz +
       SparseImgData->ChunkDataSz)) {
    DEBUG ((EFI_D_ERROR, "Bogus chunk size for chunk type Raw\n"));
    return EFI_INVALID_PARAMETER;
  }

  if (CHECK_ADD64 ((UINT64)*Image, SparseImgData->ChunkDataSz)) {
    DEBUG ((EFI_D_ERROR,
            "Integer overflow while adding Image and chunk data sz\n"));
    return EFI_INVALID_PARAMETER;
  }

  if (SparseImgData->ImageEnd < (UINT64)*Image +
      SparseImgData->ChunkDataSz) {
    DEBUG ((EFI_D_ERROR,
            "buffer overreads occured due to invalid sparse header\n"));
    return EFI_INVALID_PARAMETER;
  }

  /* Data is validated, now write to the disk */
  SparseImgData->WrittenBlockCount =
    SparseImgData->TotalBlocks * SparseImgData->BlockCountFactor;
  Status = WriteToDisk (SparseImgData->BlockIo, SparseImgData->Handle,
                        *Image,
                        SparseImgData->ChunkDataSz,
                        SparseImgData->WrittenBlockCount);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "Flash Write Failure\n"));
    return Status;
  }

  if (SparseImgData->TotalBlocks >
       (MAX_UINT32 - chunk_header->chunk_sz)) {
    DEBUG ((EFI_D_ERROR, "Bogus size for RAW chunk Type\n"));
    return EFI_INVALID_PARAMETER;
  }

  SparseImgData->TotalBlocks += chunk_header->chunk_sz;
  *Image += SparseImgData->ChunkDataSz;

  return EFI_SUCCESS;
}

STATIC EFI_STATUS
HandleChunkTypeFill (sparse_header_t *sparse_header,
        chunk_header_t *chunk_header,
        VOID **Image,
        SparseImgParam *SparseImgData)
{
  UINT32 *FillBuf = NULL;
  UINT32 FillVal;
  EFI_STATUS Status = EFI_SUCCESS;
  UINT32 Temp;

  if (sparse_header == NULL ||
      chunk_header == NULL ||
      *Image == NULL ||
      SparseImgData == NULL) {
    DEBUG ((EFI_D_ERROR, "Invalid input Parameters\n"));
    return EFI_INVALID_PARAMETER;
  }

  if (chunk_header->total_sz !=
     (sparse_header->chunk_hdr_sz + sizeof (UINT32))) {
    DEBUG ((EFI_D_ERROR, "Bogus chunk size for chunk type FILL\n"));
    return EFI_INVALID_PARAMETER;
  }

  FillBuf = AllocateZeroPool (sparse_header->blk_sz);
  if (!FillBuf) {
    DEBUG ((EFI_D_ERROR, "Malloc failed for: CHUNK_TYPE_FILL\n"));
    return EFI_OUT_OF_RESOURCES;
  }

  if (CHECK_ADD64 ((UINT64)*Image, sizeof (UINT32))) {
    DEBUG ((EFI_D_ERROR,
              "Integer overflow while adding Image and uint32\n"));
    Status = EFI_INVALID_PARAMETER;
    goto out;
  }

  if (SparseImgData->ImageEnd < (UINT64)*Image + sizeof (UINT32)) {
    DEBUG ((EFI_D_ERROR,
            "Buffer overread occured due to invalid sparse header\n"));
   Status = EFI_INVALID_PARAMETER;
   goto out;
  }

  FillVal = *(UINT32 *)*Image;
  *Image = (CHAR8 *)*Image + sizeof (UINT32);

  for (Temp = 0;
       Temp < (sparse_header->blk_sz / sizeof (FillVal));
       Temp++) {
    FillBuf[Temp] = FillVal;
  }

  for (Temp = 0; Temp < chunk_header->chunk_sz; Temp++) {
    /* Make sure the data does not exceed the partition size */
    if ((UINT64)SparseImgData->TotalBlocks *
         (UINT64)sparse_header->blk_sz +
         sparse_header->blk_sz >
         SparseImgData->PartitionSize) {
      DEBUG ((EFI_D_ERROR, "Chunk data size for fill type "
                            "exceeds partition size\n"));
      Status = EFI_VOLUME_FULL;
      goto out;
    }

    SparseImgData->WrittenBlockCount =
      SparseImgData->TotalBlocks *
        SparseImgData->BlockCountFactor;
    Status = WriteToDisk (SparseImgData->BlockIo,
                          SparseImgData->Handle,
                          (VOID *)FillBuf,
                          sparse_header->blk_sz,
                          SparseImgData->WrittenBlockCount);
    if (EFI_ERROR (Status)) {
      DEBUG ((EFI_D_ERROR, "Flash write failure for FILL Chunk\n"));

    goto out;
    }

    SparseImgData->TotalBlocks++;
  }

  out:
    if (FillBuf) {
    FreePool (FillBuf);
    FillBuf = NULL;
    }
    return Status;
}

STATIC EFI_STATUS
ValidateChunkDataAndFlash (sparse_header_t *sparse_header,
             chunk_header_t *chunk_header,
             VOID **Image,
             SparseImgParam *SparseImgData)
{
  EFI_STATUS Status;

  if (sparse_header == NULL ||
      chunk_header == NULL ||
      *Image == NULL ||
      SparseImgData == NULL) {
    DEBUG ((EFI_D_ERROR, "Invalid input Parameters\n"));
    return EFI_INVALID_PARAMETER;
  }

  switch (chunk_header->chunk_type) {
    case CHUNK_TYPE_RAW:
    Status = HandleChunkTypeRaw (sparse_header,
                                 chunk_header,
                                 Image,
                                 SparseImgData);
    if (EFI_ERROR (Status)) {
      return Status;
    }

    break;

    case CHUNK_TYPE_FILL:
      Status = HandleChunkTypeFill (sparse_header,
                                    chunk_header,
                                    Image,
                                    SparseImgData);

      if (EFI_ERROR (Status)) {
        return Status;
      }

    break;

    case CHUNK_TYPE_DONT_CARE:
      if (SparseImgData->TotalBlocks >
           (MAX_UINT32 - chunk_header->chunk_sz)) {
        DEBUG ((EFI_D_ERROR, "bogus size for chunk DONT CARE type\n"));
        return EFI_INVALID_PARAMETER;
      }
      SparseImgData->TotalBlocks += chunk_header->chunk_sz;
    break;

    case CHUNK_TYPE_CRC:
      if (chunk_header->total_sz != sparse_header->chunk_hdr_sz) {
        DEBUG ((EFI_D_ERROR, "Bogus chunk size for chunk type CRC\n"));
        return EFI_INVALID_PARAMETER;
      }

      if (SparseImgData->TotalBlocks >
           (MAX_UINT32 - chunk_header->chunk_sz)) {
        DEBUG ((EFI_D_ERROR, "Bogus size for chunk type CRC\n"));
        return EFI_INVALID_PARAMETER;
      }

      SparseImgData->TotalBlocks += chunk_header->chunk_sz;


      if (CHECK_ADD64 ((UINT64)*Image, SparseImgData->ChunkDataSz)) {
        DEBUG ((EFI_D_ERROR,
                "Integer overflow while adding Image and chunk data sz\n"));
        return EFI_INVALID_PARAMETER;
      }

      *Image += (UINT32)SparseImgData->ChunkDataSz;
      if (SparseImgData->ImageEnd < (UINT64)*Image) {
        DEBUG ((EFI_D_ERROR, "buffer overreads occured due to "
                              "invalid sparse header\n"));
        return EFI_INVALID_PARAMETER;
      }
    break;

    default:
      DEBUG ((EFI_D_ERROR, "Unknown chunk type: %x\n",
             chunk_header->chunk_type));
      return EFI_INVALID_PARAMETER;
  }
  return EFI_SUCCESS;
}

/* Handle Sparse Image Flashing */
STATIC
EFI_STATUS
HandleSparseImgFlash (IN CHAR16 *PartitionName,
                      IN UINT32 PartitionMaxSize,
                      IN VOID *Image,
                      IN UINT64 sz)
{
  sparse_header_t *sparse_header;
  chunk_header_t *chunk_header;
  EFI_STATUS Status;

  SparseImgParam SparseImgData = {0};

  if (CHECK_ADD64 ((UINT64)Image, sz)) {
    DEBUG ((EFI_D_ERROR, "Integer overflow while adding Image and sz\n"));
    return EFI_INVALID_PARAMETER;
  }

  SparseImgData.ImageEnd = (UINT64)Image + sz;
  /* Caller to ensure that the partition is present in the Partition Table*/
  Status = PartitionGetInfo (PartitionName,
                             &(SparseImgData.BlockIo),
                             &(SparseImgData.Handle));

  if (Status != EFI_SUCCESS)
    return Status;
  if (!SparseImgData.BlockIo) {
    DEBUG ((EFI_D_ERROR, "BlockIo for %a is corrupted\n", PartitionName));
    return EFI_VOLUME_CORRUPTED;
  }
  if (!SparseImgData.Handle) {
    DEBUG ((EFI_D_ERROR, "EFI handle for %a is corrupted\n", PartitionName));
    return EFI_VOLUME_CORRUPTED;
  }
  // Check image will fit on device
  SparseImgData.PartitionSize = GetPartitionSize (SparseImgData.BlockIo);
  if (!SparseImgData.PartitionSize) {
    return EFI_BAD_BUFFER_SIZE;
  }

  if (sz < sizeof (sparse_header_t)) {
    DEBUG ((EFI_D_ERROR, "Input image is invalid\n"));
    return EFI_INVALID_PARAMETER;
  }

  sparse_header = (sparse_header_t *)Image;
  if (((UINT64)sparse_header->total_blks * (UINT64)sparse_header->blk_sz) >
      SparseImgData.PartitionSize) {
    DEBUG ((EFI_D_ERROR, "Image is too large for the partition\n"));
    return EFI_VOLUME_FULL;
  }

  Image += sizeof (sparse_header_t);

  if (sparse_header->file_hdr_sz != sizeof (sparse_header_t)) {
    DEBUG ((EFI_D_ERROR, "Sparse header size mismatch\n"));
    return EFI_BAD_BUFFER_SIZE;
  }

  if (!sparse_header->blk_sz) {
    DEBUG ((EFI_D_ERROR, "Invalid block size in the sparse header\n"));
    return EFI_INVALID_PARAMETER;
  }

  if ((sparse_header->blk_sz) % (SparseImgData.BlockIo->Media->BlockSize)) {
    DEBUG ((EFI_D_ERROR, "Unsupported sparse block size %x\n",
            sparse_header->blk_sz));
    return EFI_INVALID_PARAMETER;
  }

  SparseImgData.BlockCountFactor = (sparse_header->blk_sz) /
                                   (SparseImgData.BlockIo->Media->BlockSize);

  DEBUG ((EFI_D_VERBOSE, "=== Sparse Image Header ===\n"));
  DEBUG ((EFI_D_VERBOSE, "magic: 0x%x\n", sparse_header->magic));
  DEBUG (
      (EFI_D_VERBOSE, "major_version: 0x%x\n", sparse_header->major_version));
  DEBUG (
      (EFI_D_VERBOSE, "minor_version: 0x%x\n", sparse_header->minor_version));
  DEBUG ((EFI_D_VERBOSE, "file_hdr_sz: %d\n", sparse_header->file_hdr_sz));
  DEBUG ((EFI_D_VERBOSE, "chunk_hdr_sz: %d\n", sparse_header->chunk_hdr_sz));
  DEBUG ((EFI_D_VERBOSE, "blk_sz: %d\n", sparse_header->blk_sz));
  DEBUG ((EFI_D_VERBOSE, "total_blks: %d\n", sparse_header->total_blks));
  DEBUG ((EFI_D_VERBOSE, "total_chunks: %d\n", sparse_header->total_chunks));

  /* Start processing the chunks */
  for (SparseImgData.Chunk = 0;
       SparseImgData.Chunk < sparse_header->total_chunks;
       SparseImgData.Chunk++) {

    if (((UINT64)SparseImgData.TotalBlocks * (UINT64)sparse_header->blk_sz) >=
        SparseImgData.PartitionSize) {
      DEBUG ((EFI_D_ERROR, "Size of image is too large for the partition\n"));
      return EFI_VOLUME_FULL;
    }

    /* Read and skip over chunk header */
    chunk_header = (chunk_header_t *)Image;

    if (CHECK_ADD64 ((UINT64)Image, sizeof (chunk_header_t))) {
      DEBUG ((EFI_D_ERROR,
              "Integer overflow while adding Image and chunk header\n"));
      return EFI_INVALID_PARAMETER;
    }
    Image += sizeof (chunk_header_t);

    if (SparseImgData.ImageEnd < (UINT64)Image) {
      DEBUG ((EFI_D_ERROR,
              "buffer overreads occured due to invalid sparse header\n"));
      return EFI_BAD_BUFFER_SIZE;
    }

    DEBUG ((EFI_D_VERBOSE, "=== Chunk Header ===\n"));
    DEBUG ((EFI_D_VERBOSE, "chunk_type: 0x%x\n", chunk_header->chunk_type));
    DEBUG ((EFI_D_VERBOSE, "chunk_data_sz: 0x%x\n", chunk_header->chunk_sz));
    DEBUG ((EFI_D_VERBOSE, "total_size: 0x%x\n", chunk_header->total_sz));

    if (sparse_header->chunk_hdr_sz != sizeof (chunk_header_t)) {
      DEBUG ((EFI_D_ERROR, "chunk header size mismatch\n"));
      return EFI_INVALID_PARAMETER;
    }

    SparseImgData.ChunkDataSz = (UINT64)sparse_header->blk_sz *
                                 chunk_header->chunk_sz;
    /* Make sure that chunk size calculate from sparse image does not exceed the
     * partition size
     */
    if ((UINT64)SparseImgData.TotalBlocks *
        (UINT64)sparse_header->blk_sz +
        SparseImgData.ChunkDataSz >
        SparseImgData.PartitionSize) {
      DEBUG ((EFI_D_ERROR, "Chunk data size exceeds partition size\n"));
      return EFI_VOLUME_FULL;
    }

    Status = ValidateChunkDataAndFlash (sparse_header,
                                        chunk_header,
                                        &Image,
                                        &SparseImgData);

    if (EFI_ERROR (Status)) {
      return Status;
    }
  }

  DEBUG ((EFI_D_INFO, "Wrote %d blocks, expected to write %d blocks\n",
            SparseImgData.TotalBlocks, sparse_header->total_blks));

  if (SparseImgData.TotalBlocks != sparse_header->total_blks) {
    DEBUG ((EFI_D_ERROR, "Sparse Image Write Failure\n"));
    Status = EFI_VOLUME_CORRUPTED;
  } else if (((SparseImgData.BlockIo)->FlushBlocks (SparseImgData.BlockIo))
               != EFI_SUCCESS) {
    DEBUG ((EFI_D_ERROR, "Sparse Image Flush Failure\n"));
    Status = EFI_DEVICE_ERROR;
  }
  return Status;
}

STATIC VOID
FastbootUpdateAttr (CONST CHAR16 *SlotSuffix)
{
  struct PartitionEntry *Ptn_Entries_Ptr = NULL;
  UINT32 j;
  INT32 Index;
  CHAR16 PartName[MAX_GPT_NAME_SIZE];
  CHAR8 SlotSuffixAscii[MAX_SLOT_SUFFIX_SZ];
  UnicodeStrToAsciiStr (SlotSuffix, SlotSuffixAscii);

  StrnCpyS (PartName, StrLen ((CONST CHAR16 *)L"boot") + 1,
            (CONST CHAR16 *)L"boot", StrLen ((CONST CHAR16 *)L"boot"));
  StrnCatS (PartName, MAX_GPT_NAME_SIZE - 1, SlotSuffix, StrLen (SlotSuffix));

  Index = GetPartitionIndex (PartName);
  if (Index == INVALID_PTN) {
    DEBUG ((EFI_D_ERROR, "Error boot partition for slot: %s not found\n",
            SlotSuffix));
    return;
  }
  Ptn_Entries_Ptr = &PtnEntries[Index];
  Ptn_Entries_Ptr->PartEntry.Attributes &=
      (~PART_ATT_SUCCESSFUL_VAL & ~PART_ATT_UNBOOTABLE_VAL);
  Ptn_Entries_Ptr->PartEntry.Attributes |=
      (PART_ATT_PRIORITY_VAL | PART_ATT_MAX_RETRY_COUNT_VAL);

  UpdatePartitionAttributes (PARTITION_ATTRIBUTES);
  for (j = 0; j < SlotCount; j++) {
    if (AsciiStrStr (SlotSuffixAscii, BootSlotInfo[j].SlotSuffix)) {
      AsciiStrnCpyS (BootSlotInfo[j].SlotSuccessfulVal,
                     sizeof (BootSlotInfo[j].SlotSuccessfulVal), "no",
                     AsciiStrLen ("no"));
      AsciiStrnCpyS (BootSlotInfo[j].SlotUnbootableVal,
                     sizeof (BootSlotInfo[j].SlotUnbootableVal), "no",
                     AsciiStrLen ("no"));
      AsciiSPrint (BootSlotInfo[j].SlotRetryCountVal,
                   sizeof (BootSlotInfo[j].SlotRetryCountVal), "%d",
                   MAX_RETRY_COUNT);
    }
  }
}

/* Raw Image flashing */
STATIC
EFI_STATUS
HandleRawImgFlash (IN CHAR16 *PartitionName,
                   IN UINT32 PartitionMaxSize,
                   IN VOID *Image,
                   IN UINT64 Size)
{
  EFI_STATUS Status;
  EFI_BLOCK_IO_PROTOCOL *BlockIo = NULL;
  UINT64 PartitionSize;
  EFI_HANDLE *Handle = NULL;
  CHAR16 SlotSuffix[MAX_SLOT_SUFFIX_SZ];
  BOOLEAN MultiSlotBoot = PartitionHasMultiSlot ((CONST CHAR16 *)L"boot");
  BOOLEAN HasSlot = FALSE;

  /* For multislot boot the partition may not support a/b slots.
   * Look for default partition, if it does not exist then try for a/b
   */
  if (MultiSlotBoot)
    HasSlot = GetPartitionHasSlot (PartitionName, PartitionMaxSize, SlotSuffix,
                                   MAX_SLOT_SUFFIX_SZ);

  Status = PartitionGetInfo (PartitionName, &BlockIo, &Handle);
  if (Status != EFI_SUCCESS)
    return Status;
  if (!BlockIo) {
    DEBUG ((EFI_D_ERROR, "BlockIo for %a is corrupted\n", PartitionName));
    return EFI_VOLUME_CORRUPTED;
  }
  if (!Handle) {
    DEBUG ((EFI_D_ERROR, "EFI handle for %a is corrupted\n", PartitionName));
    return EFI_VOLUME_CORRUPTED;
  }

  /* Check image will fit on device */
  PartitionSize = GetPartitionSize (BlockIo);
  if (PartitionSize < Size ||
      !PartitionSize) {
    DEBUG ((EFI_D_ERROR, "Partition not big enough.\n"));
    DEBUG ((EFI_D_ERROR, "Partition Size:\t%d\nImage Size:\t%d\n",
            PartitionSize, Size));

    return EFI_VOLUME_FULL;
  }

  Status = WriteBlockToPartition (BlockIo, Handle, 0, Size, Image);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "Writing Block to partition Failure\n"));
  }

  if (MultiSlotBoot && HasSlot &&
      !(StrnCmp (PartitionName, (CONST CHAR16 *)L"boot",
                 StrLen ((CONST CHAR16 *)L"boot"))))
    FastbootUpdateAttr (SlotSuffix);

  return Status;
}

/* UBI Image flashing */
STATIC
EFI_STATUS
HandleUbiImgFlash (
  IN CHAR16  *PartitionName,
  IN UINT32 PartitionMaxSize,
  IN VOID   *Image,
  IN UINT64   Size)
{
  EFI_STATUS Status = EFI_SUCCESS;
  EFI_BLOCK_IO_PROTOCOL *BlockIo = NULL;
  UINT32 UbiPageSize;
  UINT32 UbiBlockSize;
  EFI_UBI_FLASHER_PROTOCOL *Ubi;
  UBI_FLASHER_HANDLE UbiFlasherHandle;
  EFI_HANDLE *Handle = NULL;
  CHAR16 SlotSuffix[MAX_SLOT_SUFFIX_SZ];
  BOOLEAN MultiSlotBoot = PartitionHasMultiSlot ((CONST CHAR16 *)L"boot");
  BOOLEAN HasSlot = FALSE;
  CHAR8 PartitionNameAscii[MAX_GPT_NAME_SIZE] = {'\0'};
  UINT64 PartitionSize = 0;

  /* For multislot boot the partition may not support a/b slots.
   * Look for default partition, if it does not exist then try for a/b
   */
  if (MultiSlotBoot) {
    HasSlot =  GetPartitionHasSlot (PartitionName,
                                    PartitionMaxSize,
                                    SlotSuffix,
                                    MAX_SLOT_SUFFIX_SZ);
    DEBUG ((EFI_D_VERBOSE, "Partition has slot=%d\n", HasSlot));
  }

  Status = PartitionGetInfo (PartitionName, &BlockIo, &Handle);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "Unable to get Parition Info\n"));
    return Status;
  }

  /* Check if Image fits into partition */
  PartitionSize = GetPartitionSize (BlockIo);
  if (Size > PartitionSize ||
    !PartitionSize) {
    DEBUG ((EFI_D_ERROR, "Input Size is invalid\n"));
    return EFI_INVALID_PARAMETER;
  }

  Status = gBS->LocateProtocol (&gEfiUbiFlasherProtocolGuid,
                                NULL,
                                (VOID **) &Ubi);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "UBI Image flashing not supported.\n"));
    return Status;
  }

  UnicodeStrToAsciiStr (PartitionName, PartitionNameAscii);
  Status = Ubi->UbiFlasherOpen (PartitionNameAscii,
                                &UbiFlasherHandle,
                                &UbiPageSize,
                                &UbiBlockSize);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "Unable to open UBI Protocol.\n"));
    return Status;
  }

  /* UBI_NUM_IMAGES can replace with number of sparse images being flashed. */
  Status = Ubi->UbiFlasherWrite (UbiFlasherHandle, UBI_NUM_IMAGES, Image, Size);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "Unable to open UBI Protocol.\n"));
    return Status;
  }

  Status = Ubi->UbiFlasherClose (UbiFlasherHandle);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "Unable to close UBI Protocol.\n"));
    return Status;
  }

  return Status;
}

/* Meta Image flashing */
STATIC
EFI_STATUS
HandleMetaImgFlash (IN CHAR16 *PartitionName,
                    IN UINT32 PartitionMaxSize,
                    IN VOID *Image,
                    IN UINT64 Size)
{
  UINT32 i;
  UINT32 images;
  EFI_STATUS Status = EFI_DEVICE_ERROR;
  img_header_entry_t *img_header_entry;
  meta_header_t *meta_header;
  CHAR16 PartitionNameFromMeta[MAX_GPT_NAME_SIZE];
  UINT64 ImageEnd = 0;
  BOOLEAN PnameTerminated = FALSE;
  UINT32 j;

  if (Size < sizeof (meta_header_t)) {
    DEBUG ((EFI_D_ERROR,
            "Error: The size is smaller than the image header size\n"));
    return EFI_INVALID_PARAMETER;
  }

  meta_header = (meta_header_t *)Image;
  img_header_entry = (img_header_entry_t *)(Image + sizeof (meta_header_t));
  images = meta_header->img_hdr_sz / sizeof (img_header_entry_t);
  if (images > MAX_IMAGES_IN_METAIMG) {
    DEBUG (
        (EFI_D_ERROR,
         "Error: Number of images(%u)in meta_image are greater than expected\n",
         images));
    return EFI_INVALID_PARAMETER;
  }

  if (Size <= (sizeof (meta_header_t) + meta_header->img_hdr_sz)) {
    DEBUG (
        (EFI_D_ERROR,
         "Error: The size is smaller than image header size + entry size\n"));
    return EFI_INVALID_PARAMETER;
  }

  if (CHECK_ADD64 ((UINT64)Image, Size)) {
    DEBUG ((EFI_D_ERROR, "Integer overflow detected in %d, %a\n", __LINE__,
            __FUNCTION__));
    return EFI_BAD_BUFFER_SIZE;
  }
  ImageEnd = (UINT64)Image + Size;

  for (i = 0; i < images; i++) {
    PnameTerminated = FALSE;

    if (img_header_entry[i].ptn_name == NULL ||
        img_header_entry[i].start_offset == 0 || img_header_entry[i].size == 0)
      break;

    if (CHECK_ADD64 ((UINT64)Image, img_header_entry[i].start_offset)) {
      DEBUG ((EFI_D_ERROR, "Integer overflow detected in %d, %a\n", __LINE__,
              __FUNCTION__));
      return EFI_BAD_BUFFER_SIZE;
    }
    if (CHECK_ADD64 ((UINT64) (Image + img_header_entry[i].start_offset),
                     img_header_entry[i].size)) {
      DEBUG ((EFI_D_ERROR, "Integer overflow detected in %d, %a\n", __LINE__,
              __FUNCTION__));
      return EFI_BAD_BUFFER_SIZE;
    }
    if (ImageEnd < ((UINT64)Image + img_header_entry[i].start_offset +
                    img_header_entry[i].size)) {
      DEBUG ((EFI_D_ERROR, "Image size mismatch\n"));
      return EFI_INVALID_PARAMETER;
    }

    for (j = 0; j < MAX_GPT_NAME_SIZE; j++) {
      if (!(img_header_entry[i].ptn_name[j])) {
        PnameTerminated = TRUE;
        break;
      }
    }
    if (!PnameTerminated) {
      DEBUG ((EFI_D_ERROR, "ptn_name string not terminated properly\n"));
      return EFI_INVALID_PARAMETER;
    }
    AsciiStrToUnicodeStr (img_header_entry[i].ptn_name, PartitionNameFromMeta);

    if (!IsUnlockCritical () &&
        IsCriticalPartition (PartitionNameFromMeta)) {
      FastbootFail ("Flashing is not allowed for Critical Partitions\n");
      return EFI_INVALID_PARAMETER;
    }

    Status = HandleRawImgFlash (
        PartitionNameFromMeta, ARRAY_SIZE (PartitionNameFromMeta),
        (void *)Image + img_header_entry[i].start_offset,
        img_header_entry[i].size);
    if (Status != EFI_SUCCESS) {
      DEBUG ((EFI_D_ERROR, "Meta Image Write Failure\n"));
      return Status;
    }
  }

  Status = UpdateDevInfo (PartitionName, meta_header->img_version);
  if (Status != EFI_SUCCESS) {
    DEBUG ((EFI_D_ERROR, "Unable to Update DevInfo\n"));
  }
  return Status;
}

/* Erase partition */
STATIC EFI_STATUS
FastbootErasePartition (IN CHAR16 *PartitionName)
{
  EFI_STATUS Status;
  EFI_BLOCK_IO_PROTOCOL *BlockIo = NULL;
  EFI_HANDLE *Handle = NULL;

  Status = PartitionGetInfo (PartitionName, &BlockIo, &Handle);
  if (Status != EFI_SUCCESS)
    return Status;
  if (!BlockIo) {
    DEBUG ((EFI_D_ERROR, "BlockIo for %s is corrupted\n", PartitionName));
    return EFI_VOLUME_CORRUPTED;
  }
  if (!Handle) {
    DEBUG ((EFI_D_ERROR, "EFI handle for %s is corrupted\n", PartitionName));
    return EFI_VOLUME_CORRUPTED;
  }

  Status = ErasePartition (BlockIo, Handle);
  if (Status != EFI_SUCCESS) {
    DEBUG ((EFI_D_ERROR, "Partition Erase failed: %r\n", Status));
    return Status;
  }
  
  //write 0 to partition
  VOID *Buffer;
  UINT64  BufferSize = ALIGNMENT_MASK_4KB;
  Buffer = AllocatePages(ALIGN_PAGES(BufferSize, ALIGNMENT_MASK_4KB));
  memset(Buffer,0,BufferSize);
  Status = WriteBlockToPartition (BlockIo, Handle, 0, BufferSize, Buffer);
  if (Status != EFI_SUCCESS) {
    DEBUG((EFI_D_ERROR, "[ABL] FastbootErasePartition: Partition write 0 failed: %r\n", Status));
    return Status;
  }

  if (!(StrCmp (L"userdata", PartitionName)))
    Status = ResetDeviceState ();

#ifdef ASUS_AI2205_BUILD
  //+++ ASUS_BSP : erase_frp_partition
  if (!(StrCmp(L"frp", PartitionName))) {
    Status = erase_frp_partition();
  }
  //--- ASUS_BSP : erase_frp_partition
  else if (!(StrCmp(L"fsg", PartitionName))) {
    Status = erase_fsg();
  }
  else if (!(StrCmp(L"apdp", PartitionName))) {
    Status = erase_apdp_partition();
  }
  else if (!(StrCmp(L"modemst1", PartitionName))) {
    Status = erase_modemst1();
  }
  else if (!(StrCmp(L"modemst2", PartitionName))) {
    Status = erase_modemst2();
  }

  if (Status != EFI_SUCCESS) {
    DEBUG((EFI_D_ERROR, "[ABL] FastbootErasePartition: Partition Erase failed: %r\n", Status));
    return Status;
  }
#endif

  return Status;
}

INT32 __attribute__ ( (no_sanitize ("safe-stack")))
SparseImgFlashThread (VOID* Arg)
{
  Thread* CurrentThread = KernIntf->Thread->GetCurrentThread ();
  FlashInfo* ThreadFlashInfo = (FlashInfo*) Arg;

  if (!ThreadFlashInfo || !ThreadFlashInfo->FlashDataBuffer) {
    return 0;
  }

  KernIntf->Lock->AcquireLock (LockFlash);
  IsFlashComplete = FALSE;
  FlashSplitNeeded = TRUE;

  HandleSparseImgFlash (ThreadFlashInfo->PartitionName,
          ThreadFlashInfo->PartitionSize,
          ThreadFlashInfo->FlashDataBuffer,
          ThreadFlashInfo->FlashNumDataBytes);

  FlashSplitNeeded = FALSE;
  IsFlashComplete = TRUE;
  KernIntf->Lock->ReleaseLock (LockFlash);

  ThreadStackNodeRemove (CurrentThread);

  FreePool (ThreadFlashInfo);
  ThreadFlashInfo = NULL;

  KernIntf->Thread->ThreadExit (0);

  return 0;
}

EFI_STATUS CreateSparseImgFlashThread (IN FlashInfo* ThreadFlashInfo)
{
  EFI_STATUS Status = EFI_SUCCESS;
  Thread* SparseImgFlashTD = NULL;

  SparseImgFlashTD = KernIntf->Thread->ThreadCreate ("SparseImgFlashThread",
      SparseImgFlashThread, (VOID*)ThreadFlashInfo, UEFI_THREAD_PRIORITY,
      DEFAULT_STACK_SIZE);

  if (SparseImgFlashTD == NULL) {
    return EFI_NOT_READY;
  }

  AllocateUnSafeStackPtr (SparseImgFlashTD);

  Status = KernIntf->Thread->ThreadResume (SparseImgFlashTD);
  return Status;
}

#endif

STATIC VOID WaitForTransferComplete (VOID)
{
  USB_DEVICE_EVENT Msg;
  USB_DEVICE_EVENT_DATA Payload;
  UINTN PayloadSize;

  /* Wait for the transfer to complete */
  while (1) {
    GetFastbootDeviceData ()->UsbDeviceProtocol->HandleEvent (&Msg,
            &PayloadSize, &Payload);
    if (UsbDeviceEventTransferNotification == Msg) {
      if (1 == USB_INDEX_TO_EP (Payload.TransferOutcome.EndpointIndex)) {
        if (USB_ENDPOINT_DIRECTION_IN ==
            USB_INDEX_TO_EPDIR (Payload.TransferOutcome.EndpointIndex))
          break;
      }
    }
  }
}

/* Handle Download Command */
STATIC VOID
CmdDownload (IN CONST CHAR8 *arg, IN VOID *data, IN UINT32 sz)
{
  CHAR8 Response[13] = "DATA";
  UINT32 InitStrLen = AsciiStrLen ("DATA");

  CHAR16 OutputString[FASTBOOT_STRING_MAX_LENGTH];
  CHAR8 *NumBytesString = (CHAR8 *)arg;

  /* Argument is 8-character ASCII string hex representation of number of
   * bytes that will be sent in the data phase.Response is "DATA" + that same
   * 8-character string.
   */

  // Parse out number of data bytes to expect
  mNumDataBytes = AsciiStrHexToUint64 (NumBytesString);
  if (mNumDataBytes == 0) {
    DEBUG (
        (EFI_D_ERROR, "ERROR: Fail to get the number of bytes to download.\n"));
    FastbootFail ("Failed to get the number of bytes to download");
    return;
  }

  if (mNumDataBytes > MaxDownLoadSize) {
    DEBUG ((EFI_D_ERROR,
            "ERROR: Data size (%d) is more than max download size (%d)\n",
            mNumDataBytes, MaxDownLoadSize));
    FastbootFail ("Requested download size is more than max allowed\n");
    return;
  }

  UnicodeSPrint (OutputString, sizeof (OutputString),
                 (CONST CHAR16 *)L"Downloading %d bytes\r\n", mNumDataBytes);

  /* NumBytesString is a 8 bit string, InitStrLen is 4, and the AsciiStrnCpyS()
   * require "DestMax > SourceLen", so init length of Response as 13.
   */
  AsciiStrnCpyS (Response + InitStrLen, sizeof (Response) - InitStrLen,
                 NumBytesString, AsciiStrLen (NumBytesString));

  gBS->CopyMem (GetFastbootDeviceData ()->gTxBuffer, Response,
                sizeof (Response));

  if (IsUseMThreadParallel ()) {
    KernIntf->Lock->AcquireLock (LockDownload);
  }

  mState = ExpectDataState;
  mBytesReceivedSoFar = 0;
  GetFastbootDeviceData ()->UsbDeviceProtocol->Send (
      ENDPOINT_OUT, sizeof (Response), GetFastbootDeviceData ()->gTxBuffer);
  DEBUG ((EFI_D_VERBOSE, "CmdDownload: Send 12 %a\n",
          GetFastbootDeviceData ()->gTxBuffer));
}

#ifdef ENABLE_UPDATE_PARTITIONS_CMDS
/*  Function needed for event notification callback */
STATIC VOID
BlockIoCallback (IN EFI_EVENT Event, IN VOID *Context)
{
}

STATIC VOID
UsbTimerHandler (IN EFI_EVENT Event, IN VOID *Context)
{
  HandleUsbEvents ();
  if (FastbootFatal ())
    DEBUG ((EFI_D_ERROR, "Continue detected, Exiting App...\n"));
}

STATIC EFI_STATUS
HandleUsbEventsInTimer ()
{
  EFI_STATUS Status = EFI_SUCCESS;

  if (UsbTimerEvent)
    return Status;

  Status = gBS->CreateEvent (EVT_TIMER | EVT_NOTIFY_SIGNAL, TPL_CALLBACK,
                             UsbTimerHandler, NULL, &UsbTimerEvent);

  if (!EFI_ERROR (Status)) {
    Status = gBS->SetTimer (UsbTimerEvent, TimerPeriodic, 100000);
  }

  return Status;
}

STATIC VOID StopUsbTimer (VOID)
{
  if (UsbTimerEvent) {
    gBS->SetTimer (UsbTimerEvent, TimerCancel, 0);
    gBS->CloseEvent (UsbTimerEvent);
    UsbTimerEvent = NULL;
  }

  UsbTimerStarted = FALSE;
}
#else
STATIC VOID StopUsbTimer (VOID)
{
  return;
}
#endif

#ifdef ENABLE_UPDATE_PARTITIONS_CMDS
STATIC BOOLEAN
NamePropertyMatches (CHAR8 *Name)
{

  return (BOOLEAN) (
      !AsciiStrnCmp (Name, "has-slot", AsciiStrLen ("has-slot")) ||
      !AsciiStrnCmp (Name, "current-slot", AsciiStrLen ("current-slot")) ||
      !AsciiStrnCmp (Name, "slot-retry-count",
                     AsciiStrLen ("slot-retry-count")) ||
      !AsciiStrnCmp (Name, "slot-unbootable",
                     AsciiStrLen ("slot-unbootable")) ||
      !AsciiStrnCmp (Name, "slot-successful",
                     AsciiStrLen ("slot-successful")) ||
      !AsciiStrnCmp (Name, "slot-suffixes", AsciiStrLen ("slot-suffixes")) ||
      !AsciiStrnCmp (Name, "partition-type:system",
                     AsciiStrLen ("partition-type:system")) ||
      !AsciiStrnCmp (Name, "partition-size:system",
                     AsciiStrLen ("partition-size:system")));
}

STATIC VOID ClearFastbootVarsofAB (VOID)
{
  FASTBOOT_VAR *CurrentList = NULL;
  FASTBOOT_VAR *PrevList = NULL;
  FASTBOOT_VAR *NextList = NULL;

  for (CurrentList = Varlist; CurrentList != NULL; CurrentList = NextList) {
    NextList = CurrentList->next;
    if (!NamePropertyMatches ((CHAR8 *)CurrentList->name)) {
      PrevList = CurrentList;
      continue;
    }

    if (!PrevList)
      Varlist = CurrentList->next;
    else
      PrevList->next = CurrentList->next;

    FreePool (CurrentList);
    CurrentList = NULL;
  }
}

VOID
IsBootPtnUpdated (INT32 Lun, BOOLEAN *BootPtnUpdated)
{
  EFI_STATUS Status;
  EFI_PARTITION_ENTRY *PartEntry;
  UINT32 j;

  *BootPtnUpdated = FALSE;
  if (Lun == NO_LUN)
    Lun = 0;

  for (j = 0; j < Ptable[Lun].MaxHandles; j++) {
    Status =
        gBS->HandleProtocol (Ptable[Lun].HandleInfoList[j].Handle,
                             &gEfiPartitionRecordGuid, (VOID **)&PartEntry);

    if (EFI_ERROR (Status)) {
      DEBUG ((
          EFI_D_VERBOSE,
          "Error getting the partition record for Lun %d and Handle: %d : %r\n",
          Lun, j, Status));
      continue;
    }

    if (!StrnCmp (PartEntry->PartitionName, L"boot", StrLen (L"boot"))) {
      DEBUG ((EFI_D_VERBOSE, "Boot Partition is updated\n"));
      *BootPtnUpdated = TRUE;
      return;
    }
  }
}

#ifdef ASUS_AI2205_BUILD

STATIC BOOLEAN VerifiedBootPartitionCheck(CHAR16 *PartitionName){
  UINT32 i =0;

  if (PartitionName == NULL)
      return FALSE;

  for (i = 0; i < ARRAY_SIZE(VerifiedBootPartition); i++) {
      if (!StrnCmp(PartitionName, VerifiedBootPartition[i], StrLen(VerifiedBootPartition[i])))
          return TRUE;
  }

  return FALSE;
}

STATIC BOOLEAN UserFlashPartitionCheck(CHAR16 *PartitionName){
  UINT32 i =0;

  if (PartitionName == NULL)
      return FALSE;

  for (i = 0; i < ARRAY_SIZE(UserFlashPartition); i++) {
      if (!StrnCmp(PartitionName, UserFlashPartition[i], StrLen(UserFlashPartition[i])))
          return TRUE;
  }

  return FALSE;
}

STATIC BOOLEAN UserErasePartitionCheck(CHAR16 *PartitionName){
  UINT32 i =0;

  if (PartitionName == NULL)
      return FALSE;

  for (i = 0; i < ARRAY_SIZE(UserErasePartition); i++) {
      if (!StrnCmp(PartitionName, UserErasePartition[i], StrLen(UserErasePartition[i])))
          return TRUE;
  }

  return FALSE;
}
#endif

STATIC BOOLEAN
IsCriticalPartition (CHAR16 *PartitionName)
{
  UINT32 i = 0;

  if (PartitionName == NULL)
    return FALSE;

  for (i = 0; i < ARRAY_SIZE (CriticalPartitions); i++) {
    if (!StrnCmp (PartitionName, CriticalPartitions[i],
                  StrLen (CriticalPartitions[i])))
      return TRUE;
  }

  return FALSE;
}

STATIC BOOLEAN
CheckVirtualAbCriticalPartition (CHAR16 *PartitionName)
{
  VirtualAbMergeStatus SnapshotMergeStatus;
  UINT32 Iter = 0;
  EFI_STATUS Status = EFI_SUCCESS;
  
  // +++ ASUS_BSP : set SnapshotMergeStatus to none and skip check partiton if it is raw flash
  if(IsASUSRawFlash && (GetSnapshotMergeStatus () != NONE_MERGE_STATUS)){
	Status = SetSnapshotMergeStatus (NONE_MERGE_STATUS);
    if (Status != EFI_SUCCESS) {
      FastbootFail ("Failed to update snapshot state to NONE");
      return FALSE;
    }  
    return FALSE;
  }
  // --- ASUS_BSP : set SnapshotMergeStatus to none and skip check partiton if it is raw flash

  SnapshotMergeStatus = GetSnapshotMergeStatus ();
  if ((SnapshotMergeStatus == MERGING ||
      SnapshotMergeStatus == SNAPSHOTTED)) {
    for (Iter = 0; Iter < ARRAY_SIZE (VirtualAbCriticalPartitions); Iter++) {
      if (!StrnCmp (PartitionName, VirtualAbCriticalPartitions[Iter],
                  StrLen (VirtualAbCriticalPartitions[Iter])))
        return TRUE;
    }
  }

  return FALSE;
}

STATIC VOID ExchangeFlashAndUsbDataBuf (VOID)
{
  VOID *mTmpbuff;

  if (IsUseMThreadParallel ()) {
    KernIntf->Lock->AcquireLock (LockDownload);
    KernIntf->Lock->AcquireLock (LockFlash);
  }

  mTmpbuff = mUsbDataBuffer;
  mUsbDataBuffer = mFlashDataBuffer;
  mFlashDataBuffer = mTmpbuff;
  mFlashNumDataBytes = mNumDataBytes;

  if (IsUseMThreadParallel ()) {
    KernIntf->Lock->ReleaseLock (LockFlash);
    KernIntf->Lock->ReleaseLock (LockDownload);
  }
}

STATIC EFI_STATUS
ReenumeratePartTable (VOID)
{
  EFI_STATUS Status;
  LunSet = FALSE;
  EFI_EVENT gBlockIoRefreshEvt;
  BOOLEAN MultiSlotBoot = FALSE;
  BOOLEAN BootPtnUpdated = FALSE;

  Status =
    gBS->CreateEventEx (EVT_NOTIFY_SIGNAL, TPL_CALLBACK, BlockIoCallback,
                        NULL, &gBlockIoRefreshGuid, &gBlockIoRefreshEvt);
  if (Status != EFI_SUCCESS) {
    DEBUG ((EFI_D_ERROR, "Error Creating event for Block Io refresh:%x\n",
            Status));
    return Status;
  }

  Status = gBS->SignalEvent (gBlockIoRefreshEvt);
  if (Status != EFI_SUCCESS) {
    DEBUG ((EFI_D_ERROR, "Error Signalling event for Block Io refresh:%x\n",
            Status));
    return Status;
  }
  Status = EnumeratePartitions ();
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "Enumeration of partitions failed\n"));
    return Status;
  }
  UpdatePartitionEntries ();

  IsBootPtnUpdated (Lun, &BootPtnUpdated);
  if (BootPtnUpdated) {
    /*Check for multislot boot support*/
    MultiSlotBoot = PartitionHasMultiSlot (L"boot");
    if (MultiSlotBoot) {
      UpdatePartitionAttributes (PARTITION_ALL);
      FindPtnActiveSlot ();
      PopulateMultislotMetadata ();
      DEBUG ((EFI_D_VERBOSE, "Multi Slot boot is supported\n"));
    } else {
      DEBUG ((EFI_D_VERBOSE, "Multi Slot boot is not supported\n"));
      if (BootSlotInfo == NULL) {
        DEBUG ((EFI_D_VERBOSE, "No change in Ptable\n"));
      } else {
        DEBUG ((EFI_D_VERBOSE, "Nullifying A/B info\n"));
        ClearFastbootVarsofAB ();
        FreePool (BootSlotInfo);
        BootSlotInfo = NULL;
        gBS->SetMem ((VOID *)SlotSuffixArray, SLOT_SUFFIX_ARRAY_SIZE, 0);
        InitialPopulate = FALSE;
      }
    }
  }

  DEBUG ((EFI_D_INFO, "*************** New partition Table Dump Start "
                      "*******************\n"));
  PartitionDump ();
  DEBUG ((EFI_D_INFO, "*************** New partition Table Dump End   "
                      "*******************\n"));
  return Status;
}

STATIC VOID
CheckPartitionFsSignature (IN CHAR16 *PartName,
                           OUT FS_SIGNATURE *FsSignature)
{
  EFI_BLOCK_IO_PROTOCOL *BlockIo = NULL;
  EFI_HANDLE *Handle = NULL;
  EFI_STATUS Status = EFI_SUCCESS;
  UINT32 BlkSz = 0;
  CHAR8 *FsSuperBlk = NULL;
  CHAR8 *FsSuperBlkBuffer = NULL;
  UINT32 SuperBlkLba = 0;

  *FsSignature = UNKNOWN_FS_SIGNATURE;

  Status = PartitionGetInfo (PartName, &BlockIo, &Handle);
  if (Status != EFI_SUCCESS) {
    DEBUG ((EFI_D_ERROR, "Failed to Info for %s partition\n", PartName));
    return;
  }
  if (!BlockIo) {
    DEBUG ((EFI_D_ERROR, "BlockIo for %s is corrupted\n", PartName));
    return;
  }

  BlkSz = BlockIo->Media->BlockSize;
  FsSuperBlkBuffer = AllocateZeroPool (BlkSz);
  if (!FsSuperBlkBuffer) {
    DEBUG ((EFI_D_ERROR, "Failed to allocate buffer for superblock %s\n",
                            PartName));
    return;
  }
  FsSuperBlk = FsSuperBlkBuffer;
  SuperBlkLba = (FS_SUPERBLOCK_OFFSET / BlkSz);

  BlockIo->ReadBlocks (BlockIo, BlockIo->Media->MediaId,
                           SuperBlkLba,
                           BlkSz, FsSuperBlkBuffer);

  /* If superblklba is 0, it means super block is part of first block read */
  if (SuperBlkLba == 0) {
    FsSuperBlk += FS_SUPERBLOCK_OFFSET;
  }

  if (*((UINT16 *)&FsSuperBlk[EXT_MAGIC_OFFSET_SB]) == (UINT16)EXT_FS_MAGIC) {
    DEBUG ((EFI_D_VERBOSE, "%s Found EXT FS type\n", PartName));
    *FsSignature = EXT_FS_SIGNATURE;
  } else if (*((UINT32 *)&FsSuperBlk[F2FS_MAGIC_OFFSET_SB]) ==
              (UINT32)F2FS_FS_MAGIC) {
      DEBUG ((EFI_D_VERBOSE, "%s Found F2FS FS type\n", PartName));
      *FsSignature = F2FS_FS_SIGNATURE;
    } else {
        DEBUG ((EFI_D_VERBOSE, "%s No Known FS type Found\n", PartName));
  }

  if (FsSuperBlkBuffer) {
     FreePool (FsSuperBlkBuffer);
  }
  return;
}

STATIC EFI_STATUS
GetPartitionType (IN CHAR16 *PartName, OUT CHAR8 * PartType)
{
  UINT32 LoopCounter;
  CHAR8 AsciiPartName[MAX_GET_VAR_NAME_SIZE];
  FS_SIGNATURE FsSignature;

  if (PartName == NULL ||
      PartType == NULL) {
    DEBUG ((EFI_D_ERROR, "Invalid parameters to GetPartitionType\n"));
    return EFI_INVALID_PARAMETER;
  }

  /* By default copy raw to response */
  AsciiStrnCpyS (PartType, MAX_GET_VAR_NAME_SIZE,
                  RAW_FS_STR, AsciiStrLen (RAW_FS_STR));
  UnicodeStrToAsciiStr (PartName, AsciiPartName);

  /* Mark partition type for hard-coded partitions only */
  for (LoopCounter = 0; LoopCounter < ARRAY_SIZE (part_info); LoopCounter++) {
    /* Check if its a hardcoded partition type */
    if (!AsciiStrnCmp ((CONST CHAR8 *) AsciiPartName,
                          part_info[LoopCounter].part_name,
                          AsciiStrLen (part_info[LoopCounter].part_name))) {
      /* Check filesystem type present on partition */
      CheckPartitionFsSignature (PartName, &FsSignature);
      switch (FsSignature) {
        case EXT_FS_SIGNATURE:
          AsciiStrnCpyS (PartType, MAX_GET_VAR_NAME_SIZE, EXT_FS_STR,
                          AsciiStrLen (EXT_FS_STR));
          break;
        case F2FS_FS_SIGNATURE:
          AsciiStrnCpyS (PartType, MAX_GET_VAR_NAME_SIZE, F2FS_FS_STR,
                          AsciiStrLen (F2FS_FS_STR));
          break;
        case UNKNOWN_FS_SIGNATURE:
          /* Copy default hardcoded type in case unknown partition type */
          AsciiStrnCpyS (PartType, MAX_GET_VAR_NAME_SIZE,
                          part_info[LoopCounter].type_response,
                          AsciiStrLen (part_info[LoopCounter].type_response));
      }
    }
  }
  return EFI_SUCCESS;
}

STATIC EFI_STATUS
GetPartitionSizeViaName (IN CHAR16 *PartName, OUT CHAR8 * PartSize)
{
  EFI_BLOCK_IO_PROTOCOL *BlockIo = NULL;
  EFI_HANDLE *Handle = NULL;
  EFI_STATUS Status = EFI_INVALID_PARAMETER;
  UINT64 PartitionSize;

  Status = PartitionGetInfo (PartName, &BlockIo, &Handle);
  if (Status != EFI_SUCCESS) {
    return Status;
  }

  if (!BlockIo) {
    DEBUG ((EFI_D_ERROR, "BlockIo for %s is corrupted\n", PartName));
    return EFI_VOLUME_CORRUPTED;
  }

  PartitionSize = GetPartitionSize (BlockIo);
  if (!PartitionSize) {
    return EFI_BAD_BUFFER_SIZE;
  }

  AsciiSPrint (PartSize, MAX_RSP_SIZE, " 0x%llx", PartitionSize);
  return EFI_SUCCESS;

}

STATIC EFI_STATUS
PublishGetVarPartitionInfo (
                            IN struct GetVarPartitionInfo *PublishedPartInfo,
                            IN UINT32 NumParts)
{
  UINT32 PtnLoopCount;
  EFI_STATUS Status = EFI_INVALID_PARAMETER;
  EFI_STATUS RetStatus = EFI_SUCCESS;
  CHAR16 *PartitionNameUniCode = NULL;
  BOOLEAN PublishType;
  BOOLEAN PublishSize;

  /* Clear Published Partition Buffer */
  gBS->SetMem (PublishedPartInfo,
          sizeof (struct GetVarPartitionInfo) * MAX_NUM_PARTITIONS, 0);

  /* Loop will go through each partition entry
     and publish info for all partitions.*/
  for (PtnLoopCount = 1; PtnLoopCount <= NumParts; PtnLoopCount++) {
    PublishType = FALSE;
    PublishSize = FALSE;
    PartitionNameUniCode = PtnEntries[PtnLoopCount].PartEntry.PartitionName;
    /* Skip Null/last partition */
    if (PartitionNameUniCode[0] == '\0') {
      continue;
    }
    UnicodeStrToAsciiStr (PtnEntries[PtnLoopCount].PartEntry.PartitionName,
                          (CHAR8 *)PublishedPartInfo[PtnLoopCount].part_name);

    /* Fill partition size variable and response string */
    AsciiStrnCpyS (PublishedPartInfo[PtnLoopCount].getvar_size_str,
                      MAX_GET_VAR_NAME_SIZE, "partition-size:",
                      AsciiStrLen ("partition-size:"));
    Status = AsciiStrnCatS (PublishedPartInfo[PtnLoopCount].getvar_size_str,
                            MAX_GET_VAR_NAME_SIZE,
                            PublishedPartInfo[PtnLoopCount].part_name,
                            AsciiStrLen (
                              PublishedPartInfo[PtnLoopCount].part_name));
    if (!EFI_ERROR (Status)) {
      Status = GetPartitionSizeViaName (
                            PartitionNameUniCode,
                            PublishedPartInfo[PtnLoopCount].size_response);
      if (Status == EFI_SUCCESS) {
        PublishSize = TRUE;
      }
    }

    /* Fill partition type variable and response string */
    AsciiStrnCpyS (PublishedPartInfo[PtnLoopCount].getvar_type_str,
                    MAX_GET_VAR_NAME_SIZE, "partition-type:",
                    AsciiStrLen ("partition-type:"));
    Status = AsciiStrnCatS (PublishedPartInfo[PtnLoopCount].getvar_type_str,
                              MAX_GET_VAR_NAME_SIZE,
                              PublishedPartInfo[PtnLoopCount].part_name,
                              AsciiStrLen (
                                PublishedPartInfo[PtnLoopCount].part_name));
    if (!EFI_ERROR (Status)) {
      Status = GetPartitionType (
                            PartitionNameUniCode,
                            PublishedPartInfo[PtnLoopCount].type_response);
      if (Status == EFI_SUCCESS) {
        PublishType = TRUE;
      }
    }

    if (PublishSize) {
      FastbootPublishVar (PublishedPartInfo[PtnLoopCount].getvar_size_str,
                              PublishedPartInfo[PtnLoopCount].size_response);
    } else {
        DEBUG ((EFI_D_ERROR, "Error Publishing size info for %s partition\n",
                                                        PartitionNameUniCode));
        RetStatus = EFI_INVALID_PARAMETER;
    }

    if (PublishType) {
      FastbootPublishVar (PublishedPartInfo[PtnLoopCount].getvar_type_str,
                              PublishedPartInfo[PtnLoopCount].type_response);
    } else {
        DEBUG ((EFI_D_ERROR, "Error Publishing type info for %s partition\n",
                                                        PartitionNameUniCode));
        RetStatus = EFI_INVALID_PARAMETER;
    }
  }
  return RetStatus;
}

/* Handle Flash Command */
STATIC VOID
CmdFlash (IN CONST CHAR8 *arg, IN VOID *data, IN UINT32 sz)
{
  EFI_STATUS Status = EFI_SUCCESS;
  sparse_header_t *sparse_header;
  meta_header_t *meta_header;
  UbiHeader_t *UbiHeader;
  CHAR16 PartitionName[MAX_GPT_NAME_SIZE];
  CHAR16 *Token = NULL;
  LunSet = FALSE;
  BOOLEAN MultiSlotBoot = FALSE;
  UINT32 UfsBootLun = 0;
  CHAR8 BootDeviceType[BOOT_DEV_NAME_SIZE_MAX];
  /* For partition info */
  EFI_BLOCK_IO_PROTOCOL *BlockIo = NULL;
  EFI_HANDLE *Handle = NULL;
  BOOLEAN HasSlot = FALSE;
  CHAR16 SlotSuffix[MAX_SLOT_SUFFIX_SZ];
  CHAR8 FlashResultStr[MAX_RSP_SIZE] = "";
  UINT64 PartitionSize = 0;
  UINT32 Ret;
  VirtualAbMergeStatus SnapshotMergeStatus;

  ExchangeFlashAndUsbDataBuf ();
  if (mFlashDataBuffer == NULL) {
    // Doesn't look like we were sent any data
    FastbootFail ("No data to flash");
    return;
  }

  if (AsciiStrLen (arg) >= MAX_GPT_NAME_SIZE) {
    FastbootFail ("Invalid partition name");
    return;
  }
  AsciiStrToUnicodeStr (arg, PartitionName);

#ifdef ASUS_AI2205_BUILD
  CHAR8 DeviceInfo[MAX_RSP_SIZE];

  if ((!StrnCmp (PartitionName, L"vendor", StrLen (L"vendor")) &&
       StrnCmp (PartitionName, L"vendor_boot", StrLen (L"vendor_boot"))) ||
      !StrnCmp (PartitionName, L"system", StrLen (L"system")) ||
      !StrnCmp (PartitionName, L"system_ext", StrLen (L"system_ext")) ||
      !StrnCmp (PartitionName, L"odm", StrLen (L"odm")) ||
      !StrnCmp (PartitionName, L"product", StrLen (L"product"))) {
        FastbootFail ("Please execute: fastboot reboot fastboot");
        return;
  }
  

  // +++ ASUS_BSP : add for fastboot permission
  if(!ASUS_FASTBOOT_PERMISSION && (IsRawFlash < 6) && (IsRawFlash_gpt < 6) && !UserFlashPartitionCheck(PartitionName))
  {
      if(IsUnlocked() && VerifiedBootPartitionCheck(PartitionName))
      {
          DEBUG ((EFI_D_INFO, "[ABL] Only allow flash hlos image in lock state\n"));
      }else if (IsUnlocked() && !VerifiedBootPartitionCheck(PartitionName))
      {
          FastbootFail("Flashing non-hlos image is not allowed in lock state");
          return;
      }
      else
      {
          FastbootFail("Flashing is not allowed in Lock State");
          return;
      }
  }

  if (IsAuthorized_2() && VerifiedBootPartitionCheck(PartitionName))
  {
      if(!IsAuthorized() && !IsUnlocked() && (IsRawFlash < 6) && (IsRawFlash_gpt < 6))
      {
          AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "If flashing un-signed image in lock state,");
          FastbootInfo(DeviceInfo);
          WaitForTransferComplete();
          AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "or flashing boot/dtbo/vendor_boot/system/vendor/super without flashing vbmeta/vbmeta_system,");
          FastbootInfo(DeviceInfo);
          WaitForTransferComplete();
          AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "the device will be unbootable.");
          FastbootInfo(DeviceInfo);
          WaitForTransferComplete();
      }
  }

  /* Handle ASUS ASDF partition */
  if (!StrnCmp (PartitionName, L"asdf", StrLen (L"asdf"))) {
    if(!ASUS_FASTBOOT_PERMISSION && !is_ftm_mode() && !g_allow_flash_and_erase_asdf){
      DEBUG ((EFI_D_INFO, "[ABL] Skip flashing asdf partition\n"));
      AsciiSPrint (DeviceInfo, sizeof(DeviceInfo),"");
      FastbootInfo (DeviceInfo);
      WaitForTransferComplete();
      AsciiSPrint (DeviceInfo, sizeof(DeviceInfo),
                   "ASUS_FASTBOOT_PERMISSION=%a",
                    ASUS_FASTBOOT_PERMISSION? "TRUE" : "FALSE");
      FastbootInfo (DeviceInfo);
      WaitForTransferComplete();
      AsciiSPrint (DeviceInfo, sizeof(DeviceInfo),
                   "FTM_MODE=%a",
                    is_ftm_mode()? "TRUE" : "FALSE");
      FastbootInfo (DeviceInfo);
      WaitForTransferComplete();
      AsciiSPrint (DeviceInfo, sizeof(DeviceInfo),
                   "ASDF_FLAG=%a",
                    g_allow_flash_and_erase_asdf? "TRUE" : "FALSE");
      FastbootInfo (DeviceInfo);
      WaitForTransferComplete();
      AsciiSPrint (DeviceInfo, sizeof(DeviceInfo),
                   "Skip flashing %s partition", PartitionName);
      FastbootInfo (DeviceInfo);
      WaitForTransferComplete();
      FastbootOkay ("");
      return;
    }
  }
  // --- ASUS_BSP : add for fastboot permission
#else
  if ((GetAVBVersion () == AVB_LE) ||
      ((GetAVBVersion () != AVB_LE) &&
      (TargetBuildVariantUser ()))) {
    if (!IsUnlocked ()) {
      FastbootFail ("Flashing is not allowed in Lock State");
      return;
    }

    if (!IsUnlockCritical () && IsCriticalPartition (PartitionName)) {
      FastbootFail ("Flashing is not allowed for Critical Partitions\n");
      return;
    }
  }
#endif

  if (IsDynamicPartitionSupport ()) {
    /* Virtual A/B is enabled by default.*/
#ifdef ASUS_BUILD
    if (CheckVirtualAbCriticalPartition (PartitionName) && !ASUS_FASTBOOT_PERMISSION) {
#else
    if (CheckVirtualAbCriticalPartition (PartitionName)) {
#endif
      AsciiSPrint (FlashResultStr, MAX_RSP_SIZE,
                    "Flashing of %s is not allowed in %a state",
                    PartitionName, SnapshotMergeState);
      FastbootFail (FlashResultStr);
      return;
    }

    SnapshotMergeStatus = GetSnapshotMergeStatus ();
    if (((SnapshotMergeStatus == MERGING) ||
          (SnapshotMergeStatus == SNAPSHOTTED)) &&
          !StrnCmp (PartitionName, L"super", StrLen (L"super"))) {

      Status = SetSnapshotMergeStatus (CANCELLED);
      if (Status != EFI_SUCCESS) {
        FastbootFail ("Failed to update snapshot state to cancel");
        return;
      }

      //updating fbvar snapshot-merge-state
      AsciiSPrint (SnapshotMergeState,
                    AsciiStrLen (VabSnapshotMergeStatus[NONE_MERGE_STATUS]) + 1,
                    "%a", VabSnapshotMergeStatus[NONE_MERGE_STATUS]);
    }
  }

  /* Handle virtual partition avb_custom_key */
  if (!StrnCmp (PartitionName, L"avb_custom_key", StrLen (L"avb_custom_key"))) {
    DEBUG ((EFI_D_INFO, "flashing avb_custom_key\n"));
    Status = StoreUserKey (data, sz);
    if (Status != EFI_SUCCESS) {
      FastbootFail ("Flashing avb_custom_key failed");
    } else {
      FastbootOkay ("");
    }
    return;
  }
  
   /* ASUS BSP: only erase userdata because f2fs format issue */
#ifdef ASUS_AI2205_BUILD
  if(!StrnCmp(PartitionName, L"userdata", StrLen(L"userdata"))){
	DEBUG ((EFI_D_INFO, "only erase userdata because of f2fs format issue.\n"));
	Status = FastbootErasePartition (PartitionName);
	if (EFI_ERROR (Status)) {
	  DEBUG ((EFI_D_ERROR, "erase userdata fail: %r\n", Status));
	}
	FastbootOkay("");
	return;
  }
#endif

  /* Find the lun number from input string */
  Token = StrStr (PartitionName, L":");

  if (Token) {
    /* Skip past ":" to the lun number */
    Token++;
    Lun = StrDecimalToUintn (Token);

    if (Lun >= MAX_LUNS) {
      FastbootFail ("Invalid Lun number passed\n");
      goto out;
    }

    LunSet = TRUE;
  }

  if (!StrnCmp (PartitionName, L"partition", StrLen (L"partition")))
  {
#ifdef ASUS_BUILD
    // +++ ASUS_BSP : add for ckeck CRC_partiiton:0~6
    UINT8 *Buffer = NULL;
    UINT32 temp_crc = 0xFFFFFFFF;

    g_calculate_gpt0_crc = 0xFFFFFFFF;
    g_calculate_gpt1_crc = 0xFFFFFFFF;
    g_calculate_gpt2_crc = 0xFFFFFFFF;
    g_calculate_gpt3_crc = 0xFFFFFFFF;
    g_calculate_gpt4_crc = 0xFFFFFFFF;
    g_calculate_gpt5_crc = 0xFFFFFFFF;
    g_calculate_gpt6_crc = 0xFFFFFFFF;

    DEBUG((EFI_D_INFO, "[ABL]  CmdFlash - check partition:(%d)\n",sz));

    Buffer = AllocatePages(ALIGN_PAGES(mFlashNumDataBytes, ALIGNMENT_MASK_4KB));
    memcpy(Buffer, (UINT8*)mFlashDataBuffer, mFlashNumDataBytes);

    Status = CalculateCrc32(Buffer, mFlashNumDataBytes, &temp_crc);
    DEBUG((EFI_D_INFO, "[ABL]  CmdFlash - calc partition crc = 0x%x \n", temp_crc));

    if (Status == EFI_SUCCESS)
    {
      if(!StrnCmp(PartitionName, L"partition:0", StrLen(L"partition:0")))
      {
        g_calculate_gpt0_crc = (unsigned int) temp_crc;
        DEBUG((EFI_D_INFO,  "[ABL]  CmdFlash - g_calculate_gpt0_crc = 0x%x \n", g_calculate_gpt0_crc));
      }
      else if(!StrnCmp(PartitionName, L"partition:1", StrLen(L"partition:1")))
      {
        g_calculate_gpt1_crc = (unsigned int) temp_crc;
        DEBUG((EFI_D_INFO,  "[ABL]  CmdFlash - g_calculate_gpt1_crc = 0x%x \n", g_calculate_gpt1_crc));
      }
      else if(!StrnCmp(PartitionName, L"partition:2", StrLen(L"partition:2")))
      {
        g_calculate_gpt2_crc = (unsigned int) temp_crc;
        DEBUG((EFI_D_INFO,  "[ABL]  CmdFlash - g_calculate_gpt2_crc = 0x%x \n", g_calculate_gpt2_crc));
      }
      else if(!StrnCmp(PartitionName, L"partition:3", StrLen(L"partition:3")))
      {
        g_calculate_gpt3_crc = (unsigned int) temp_crc;
        DEBUG((EFI_D_INFO,  "[ABL]  CmdFlash - g_calculate_gpt3_crc = 0x%x \n", g_calculate_gpt3_crc));
      }
      else if(!StrnCmp(PartitionName, L"partition:4", StrLen(L"partition:4")))
      {
        g_calculate_gpt4_crc = (unsigned int) temp_crc;
        DEBUG((EFI_D_INFO,  "[ABL]  CmdFlash - g_calculate_gpt4_crc = 0x%x \n", g_calculate_gpt4_crc));
      }
      else if(!StrnCmp(PartitionName, L"partition:5", StrLen(L"partition:5")))
      {
        g_calculate_gpt5_crc = (unsigned int) temp_crc;
        DEBUG((EFI_D_INFO,  "[ABL]  CmdFlash - g_calculate_gpt5_crc = 0x%x \n", g_calculate_gpt5_crc));
      }
      else if(!StrnCmp(PartitionName, L"partition:6", StrLen(L"partition:6")))
      {
        g_calculate_gpt6_crc = (unsigned int) temp_crc;
        DEBUG((EFI_D_INFO,  "[ABL]  CmdFlash - g_calculate_gpt6_crc = 0x%x \n", g_calculate_gpt6_crc));
      }
      IsRawFlash_gpt++;
    }

    FreePages(Buffer, ALIGN_PAGES(mFlashNumDataBytes, ALIGNMENT_MASK_4KB));
    // +++ ASUS_BSP : add for ckeck CRC_partiiton:0~6
#endif

    GetRootDeviceType (BootDeviceType, BOOT_DEV_NAME_SIZE_MAX);

    if ((!StrnCmp (PartitionName, L"partition", StrLen (L"partition"))) ||
         ((!StrnCmp (PartitionName, L"mibib", StrLen (L"mibib"))) &&
         (!AsciiStrnCmp (BootDeviceType, "NAND", AsciiStrLen ("NAND"))))) {
      if (!AsciiStrnCmp (BootDeviceType, "UFS", AsciiStrLen ("UFS"))) {
        UfsGetSetBootLun (&UfsBootLun, TRUE); /* True = Get */
        if (UfsBootLun != 0x1) {
          UfsBootLun = 0x1;
          UfsGetSetBootLun (&UfsBootLun, FALSE); /* False = Set */
        }
      } else if (!AsciiStrnCmp (BootDeviceType, "EMMC", AsciiStrLen ("EMMC"))) {
        Lun = NO_LUN;
        LunSet = FALSE;
      }
      DEBUG ((EFI_D_INFO, "Attemping to update partition table\n"));
      DEBUG ((EFI_D_INFO, "*************** Current partition Table Dump Start "
                          "*******************\n"));
      PartitionDump ();
      DEBUG ((EFI_D_INFO, "*************** Current partition Table Dump End   "
                          "*******************\n"));
      if (!AsciiStrnCmp (BootDeviceType, "NAND", AsciiStrLen ("NAND"))) {
        Ret = PartitionVerifyMibibImage (mFlashDataBuffer);
        if (Ret) {
          FastbootFail ("Error Updating partition Table\n");
          goto out;
        }
        Status = HandleRawImgFlash (PartitionName,
                          ARRAY_SIZE (PartitionName),
                          mFlashDataBuffer, mFlashNumDataBytes);
      }
      else {
          // +++ ASUS_BSP : add for enable flash raw in slot_b
#ifdef ASUS_BUILD          
          if(!StrnCmp (GetCurrentSlotSuffix ().Suffix, L"_b", StrLen (L"_b")) && g_asus_slot_b_enable == TRUE){
            DEBUG((EFI_D_ERROR, "Enable flash slot_b, skip UpdatePartitionTable\n"));
          }else{
            Status = UpdatePartitionTable (mFlashDataBuffer, mFlashNumDataBytes, Lun, Ptable);
          }
#else
          Status = UpdatePartitionTable (mFlashDataBuffer, mFlashNumDataBytes, Lun, Ptable);
#endif
          // --- ASUS_BSP : add for enable flash raw in slot_b
      }
      /* Signal the Block IO to update and reenumerate the parition table */
      if (Status == EFI_SUCCESS)  {
        Status = ReenumeratePartTable ();
        if (Status == EFI_SUCCESS) {
#ifdef ASUS_AI2205_BUILD
          UINT32 PartitionCount = 0;
          GetPartitionCount (&PartitionCount);
          Status = PublishGetVarPartitionInfo (PublishedPartInfo, PartitionCount);
          if (Status != EFI_SUCCESS) {
            DEBUG ((EFI_D_ERROR, "Failed to publish part info for all partitions\n"));
            FastbootFail ("Error Updating partition Table\n");
            goto out;
          } else {
            FastbootOkay ("");
            goto out;
          }
#else
          FastbootOkay ("");
          goto out;
#endif
        }
      }
      FastbootFail ("Error Updating partition Table\n");
      goto out;
    }
  }

#ifdef ASUS_BUILD
  // +++ ASUS_BSP : add for FRP unlock
  if (!StrnCmp(PartitionName, L"sha256", StrLen(PartitionName))){
    DEBUG((EFI_D_INFO, "[ABL]  +++ CmdFlash : sha256 data (%d)\n",sz));

#if 0
    UINT32 i=0;
    DEBUG((EFI_D_INFO, "[ABL] mDataBuffer[%d] = \n",sz));
    for(i=0;i<sz;i++)
    {
      DEBUG((EFI_D_INFO, " %02x",mFlashDataBuffer[i] ));
    }
    DEBUG((EFI_D_INFO, "\n"));
#endif

    CmdOemSha256(NULL, (VOID *) mFlashDataBuffer, mFlashNumDataBytes);

    DEBUG((EFI_D_INFO, "[ABL]  --- CmdFlash : sha256 data\n"));
    FastbootOkay("");

    return;
  }

  if (!StrnCmp(PartitionName, L"sig", StrLen(PartitionName))){
    EFI_STATUS Status = EFI_SUCCESS;

    CHAR8 SSN[16]={0};  // Read SSN
    UINT8 SIG[SIGNATURE_LEN];
    size_t SSN_size = 0;

#if 1
    // 1 Get SN
    GetSSNNum(SSN, sizeof(SSN));
    SSN_size = strlen(SSN);
    DEBUG((EFI_D_ERROR, "[ABL] +++ FRP_UNLOCK(SSN=%a),size=%d\n",SSN,SSN_size));

#else	//test SSN
    AsciiSPrint(SSN, sizeof(SSN), "%a", "G6AZCY03S376EYA");
    DEBUG((EFI_D_ERROR, "[ABL]  +++ FRP_UNLOCK(SSN=%a)\n",SSN));
#endif

    UINT8 hash[32] = {0};
    memset(hash,0,sizeof(hash));

    Status = get_image_hash((UINT8*)SSN,SSN_size,hash,sizeof(hash),VB_SHA256);

    if (Status != EFI_SUCCESS)
    {
      DEBUG((EFI_D_ERROR, "[ABL] (ERROR) get_image_hash : FAIL (%d)\n", Status));
      FastbootFail("SHA256 : FAIL");
      return;
    }

    // Following RFC3447, SHA256 value begins at byte 19,
    // http://www.ietf.org/rfc/rfc3447.txt
    //MD2:	 (0x)30 20 30 0c 06 08 2a 86 48 86 f7 0d 02 02 05 00 04
    //			  10 || H.
    //MD5:	 (0x)30 20 30 0c 06 08 2a 86 48 86 f7 0d 02 05 05 00 04
    //			  10 || H.
    //SHA-1:	 (0x)30 21 30 09 06 05 2b 0e 03 02 1a 05 00 04 14 || H.
    //SHA-256: (0x)30 31 30 0d 06 09 60 86 48 01 65 03 04 02 01 05 00
    //			  04 20 || H.
    //SHA-384: (0x)30 41 30 0d 06 09 60 86 48 01 65 03 04 02 02 05 00
    //			  04 30 || H.
    //SHA-512: (0x)30 51 30 0d 06 09 60 86 48 01 65 03 04 02 03 05 00
    //				 04 40 || H.

    UINT8 sha256_header[19] = {0x30,0x31,0x30,0x0d,0x06,0x09,0x60,0x86,0x48,0x01,
                               0x65,0x03,0x04,0x02,0x01,0x05,0x00,0x04,0x20};
    UINT8 sha256_buf[51] = {0};

    memset(sha256_buf,0,sizeof(sha256_buf));
    memcpy(sha256_buf,sha256_header,19);
    memcpy(&sha256_buf[19],hash,sizeof(hash));

#if 0
    UINT32 i=0;
    UINT32 size=51;

    DEBUG((EFI_D_ERROR, "\n"));
    DEBUG((EFI_D_ERROR, "[ABL] FRP_UNLOCK[%d] = \n",size));
    for(i=0;i<size;i++)
    {
      DEBUG((EFI_D_ERROR, " %02x",sha256_buf[i] ));
    }
    DEBUG((EFI_D_ERROR, "\n"));
#endif

    // 2. Get signature from asuskey partition
    memset(SIG,0,sizeof(SIG));
    memcpy(SIG,mFlashDataBuffer,sizeof(SIG));

#if 0
    UINT32 i=0;
    DEBUG((EFI_D_INFO, "[ABL] SIG[%d] = \n",sz));
    for(i=0;i<sz;i++)
    {
      DEBUG((EFI_D_INFO, " %02x",SIG[i] ));
    }
    DEBUG((EFI_D_INFO, "\n"));
#endif

    // 3. RSA Verify
    Status = SecRSATestAppMain(SIG,sizeof(SIG),(UINT8*)sha256_buf,sizeof(sha256_buf),FRP_UNLOCK);
    if (Status == EFI_SUCCESS)
    {
      // 4. erase FRP partition
      Status = erase_frp_partition();
      if (Status == EFI_SUCCESS)
      {
        FastbootOkay("");
        return;
      }
      else
      {
        FastbootFail("erase frp Failed!!");
        return;
      }
    }
    else
    {
      FastbootFail("Verify FRP Unlock Signature Failed!!");
      return;
    }
    DEBUG((EFI_D_ERROR, "[ABL] --- FRP_UNLOCK(SSN=%a)\n",SSN));
  }
  // --- ASUS_BSP : add for FRP unlock

  // +++ ASUS_BSP : add for ASUS dongle unlock
  if (!StrnCmp(PartitionName, L"asuskey2", StrLen(L"asuskey2")))
  {
    DEBUG((EFI_D_INFO, "[ABL]  +++ CmdFlash : receive asuskey2 data (%d)\n", StrLen(PartitionName)));

    memset(mmc_hash_buf, 0, sizeof(mmc_hash_buf));
    memcpy(mmc_hash_buf, (char*)mFlashDataBuffer, sizeof(mmc_hash_buf) - 1);
    DEBUG((EFI_D_INFO, "[ABL] mmc_hash_buf = %a\n", mmc_hash_buf));

    #if 0
    UINT32 i=0;
    DEBUG((EFI_D_INFO, "[ABL] mmc_hash_buf[%d] = \n",sz));
    for(i=0;i<sz;i++)
    {
    DEBUG((EFI_D_INFO, " %x",mmc_hash_buf[i] ));
    }
    DEBUG((EFI_D_INFO, "\n"));
    #endif

    DEBUG((EFI_D_INFO, "[ABL]  --- CmdFlash : receive asuskey2 data\n"));
    FastbootOkay("");
    goto out;
  }
  // --- ASUS_BSP : add for ASUS dongle unlock

  if (!StrnCmp(PartitionName, L"frp", StrLen(L"frp"))){
    DEBUG((EFI_D_INFO, "[ABL]  +++ CmdFlash : want flash frp\n"));
    if(!ASUS_FASTBOOT_PERMISSION && !is_ftm_mode()){
        FastbootFail("Can not flash frp");
        goto out;
    }
  }

  if (!StrnCmp(PartitionName, L"CRC", StrLen(PartitionName)))
  {
    DEBUG((EFI_D_INFO, "[ABL]  CmdFlash : check CRC\n"));
    char crc_ptr[8]={0};

    DEBUG((EFI_D_INFO, "[ABL]  data = 0x%x\n", &mFlashDataBuffer));
    memcpy(crc_ptr, (char*)mFlashDataBuffer, 8);

    g_flash_crc =  GET_LWORD_FROM_BYTE(crc_ptr);
    DEBUG((EFI_D_INFO, "[ABL]  CmdFlash : get flash CRC = 0x%x\n", g_flash_crc));

    FastbootOkay("");
    goto out;
  }

  if(!StrnCmp(PartitionName, L"CRC_", StrLen(L"CRC_")))
  {
    char crc_ptr[8]={0};
    CHAR16* pt_name=NULL;
    pt_name=(CHAR16 *)(PartitionName);
    pt_name+=4;

    DEBUG((EFI_D_INFO, "[ABL]  CmdFlash : check (%s) partition\n", pt_name));
    DEBUG((EFI_D_INFO, "[ABL]  CmdFlash : data = 0x%x\n", mFlashDataBuffer));
    memcpy(crc_ptr,(char*)mFlashDataBuffer, 8);

    g_flash_crc =  GET_LWORD_FROM_BYTE(mFlashDataBuffer);
    DEBUG((EFI_D_INFO, "[ABL]  CmdFlash : get flash CRC = 0x%x\n", g_flash_crc));

    if(!StrnCmp(pt_name, L"super", StrLen(L"super")))
    {
      IsRawFlash = 0;
      IsRawFlash_gpt = 0;
    }

    // +++ ASUS_BSP : add for ckeck CRC_partiiton:0~6
    if((!StrnCmp(pt_name, L"partition:", StrLen(L"partition:"))) )
    {
      DEBUG((EFI_D_INFO, "[ABL]  CmdFlash : check CRC_(%s) from DDR = 0x%x\n", pt_name));

      if(!StrnCmp(pt_name, L"partition:0", StrLen(L"partition:0")))
      {
        if(g_calculate_gpt0_crc != g_flash_crc)
        {
          DEBUG((EFI_D_INFO, "[ABL]  Calculate DDR CRC (0x%x) != RAW CRC (0x%x)", g_calculate_gpt0_crc, g_flash_crc));
          IsRawFlash = 0;
          FastbootFail("GPT0 CRC ERROR");
          goto out;
        }
      }
      else if(!StrnCmp(pt_name, L"partition:1", StrLen(L"partition:1")))
      {
        if(g_calculate_gpt1_crc != g_flash_crc)
        {
          DEBUG((EFI_D_INFO, "[ABL]  Calculate DDR CRC (0x%x) != RAW CRC (0x%x)", g_calculate_gpt1_crc, g_flash_crc));
          IsRawFlash = 0;
          FastbootFail("GPT1 CRC ERROR");
          goto out;
        }
      }
      else if(!StrnCmp(pt_name, L"partition:2", StrLen(L"partition:2")))
      {
        if(g_calculate_gpt2_crc != g_flash_crc)
        {
          DEBUG((EFI_D_INFO, "[ABL]  Calculate DDR CRC (0x%x) != RAW CRC (0x%x)", g_calculate_gpt2_crc, g_flash_crc));
          IsRawFlash = 0;
          FastbootFail("GPT2 CRC ERROR");
          goto out;
        }
      }
      else if(!StrnCmp(pt_name, L"partition:3", StrLen(L"partition:3")))
      {
        if(g_calculate_gpt3_crc != g_flash_crc)
        {
          DEBUG((EFI_D_INFO, "[ABL]  Calculate DDR CRC (0x%x) != RAW CRC (0x%x)", g_calculate_gpt3_crc, g_flash_crc));
          IsRawFlash = 0;
          FastbootFail("GPT3 CRC ERROR");
          goto out;
        }
      }
      else if(!StrnCmp(pt_name, L"partition:4", StrLen(L"partition:4")))
      {
        if(g_calculate_gpt4_crc != g_flash_crc)
        {
          DEBUG((EFI_D_INFO, "[ABL]  Calculate DDR CRC (0x%x) != RAW CRC (0x%x)", g_calculate_gpt4_crc, g_flash_crc));
          IsRawFlash = 0;
          FastbootFail("GPT4 CRC ERROR");
          goto out;
        }
      }
      else if(!StrnCmp(pt_name, L"partition:5", StrLen(L"partition:5")))
      {
        if(g_calculate_gpt5_crc != g_flash_crc)
        {
          DEBUG((EFI_D_INFO, "[ABL]  Calculate DDR CRC (0x%x) != RAW CRC (0x%x)", g_calculate_gpt5_crc, g_flash_crc));
          IsRawFlash = 0;
          FastbootFail("GPT5 CRC ERROR");
          goto out;
        }
      }
      else if(!StrnCmp(pt_name, L"partition:6", StrLen(L"partition:6")))
      {
        if(g_calculate_gpt6_crc != g_flash_crc)
        {
          DEBUG((EFI_D_INFO, "[ABL]  Calculate DDR CRC (0x%x) != RAW CRC (0x%x)", g_calculate_gpt6_crc, g_flash_crc));
          IsRawFlash = 0;
          FastbootFail("GPT6 CRC ERROR");
          goto out;
        }
      }

      IsRawFlash++;
      if(IsRawFlash == 7 && IsRawFlash_gpt == 7){
          IsASUSRawFlash = TRUE;
          //Set DM verity to enforcing during raw flash
          EnableEnforcingMode (TRUE);
          //This clears the persistent property.
          WritePersistentValue ((const uint8_t *)AVB_NPV_MANAGED_VERITY_MODE, AsciiStrLen(AVB_NPV_MANAGED_VERITY_MODE),0,0);
      }
      FastbootOkay("");
      goto out;

    }
    // --- ASUS_BSP : add for ckeck CRC_partiiton:0~6

    else if (PartitionHasMultiSlot (pt_name))
    {
      if(!StrnCmp (GetCurrentSlotSuffix ().Suffix, L"_a", StrLen (L"_a")))
      {
        StrCat (pt_name, L"_a");
        g_calculate_crc = AsusCalculatePtCrc32(pt_name);
      }else
      {
        StrCat (pt_name, L"_b");
        g_calculate_crc = AsusCalculatePtCrc32(pt_name);
      }
      }else
      {
        g_calculate_crc = AsusCalculatePtCrc32(pt_name);
      }

      DEBUG((EFI_D_INFO, "[ABL]  g_calculate_crc = 0x%x ; g_flash_crc = 0x%x\n", g_calculate_crc, g_flash_crc));
      if(g_calculate_crc != g_flash_crc)
      {
        DEBUG((EFI_D_INFO, "[ABL]  Calculate CRC (0x%x) != RAW CRC (0x%x)", g_calculate_crc, g_flash_crc));

        FastbootFail("CRC ERROR");
        g_calculate_crc = 0xFFFFFFFF;
        g_flash_crc = 0xFFFFFFFF;
        IsRawFlash = 0;
        IsRawFlash_gpt = 0;
        goto out;
      }

      g_calculate_crc = 0xFFFFFFFF;
      g_flash_crc = 0xFFFFFFFF;

      FastbootOkay("");
      goto out;
  }

  if(!StrnCmp(PartitionName, L"signature", StrLen(L"signature")))
  {
      //DEBUG ((EFI_D_ERROR, "[ABL] Attemping to flash signature\n"));
      //DEBUG ((EFI_D_ERROR, "[ABL] Set check_raw_flash = TRUE\n"));
      //check_raw_flash = TRUE;
      FastbootOkay("");
      goto out;
  }
#endif // #ifdef ASUS_BUILD

  sparse_header = (sparse_header_t *)mFlashDataBuffer;
  meta_header = (meta_header_t *)mFlashDataBuffer;
  UbiHeader = (UbiHeader_t *)mFlashDataBuffer;

  /* Send okay for next data sending */
  if (sparse_header->magic == SPARSE_HEADER_MAGIC) {

    MultiSlotBoot = PartitionHasMultiSlot ((CONST CHAR16 *)L"boot");
    if (MultiSlotBoot) {
      HasSlot = GetPartitionHasSlot (PartitionName,
                                     ARRAY_SIZE (PartitionName),
                                     SlotSuffix, MAX_SLOT_SUFFIX_SZ);
      if (HasSlot) {
        DEBUG ((EFI_D_VERBOSE, "Partition %s has slot\n", PartitionName));
      }
    }

    Status = PartitionGetInfo (PartitionName, &BlockIo, &Handle);
    if (EFI_ERROR (Status)) {
      FastbootFail ("Partition not found");
      goto out;
    }

    PartitionSize = GetPartitionSize (BlockIo);
    if (!PartitionSize) {
      FastbootFail ("Partition error size");
      goto out;
    }

    if ((PartitionSize > MaxDownLoadSize) &&
         !IsDisableParallelDownloadFlash ()) {
      if (IsUseMThreadParallel ()) {
        FlashInfo* ThreadFlashInfo = AllocateZeroPool (sizeof (FlashInfo));
        if (!ThreadFlashInfo) {
          DEBUG ((EFI_D_ERROR,
                  "ERROR: Failed to allocate memory for ThreadFlashInfo\n"));
          return ;
        }

        ThreadFlashInfo->FlashDataBuffer = mFlashDataBuffer,
        ThreadFlashInfo->FlashNumDataBytes = mFlashNumDataBytes;

        StrnCpyS (ThreadFlashInfo->PartitionName, MAX_GPT_NAME_SIZE,
                PartitionName, ARRAY_SIZE (PartitionName));
        ThreadFlashInfo->PartitionSize = ARRAY_SIZE (PartitionName);

        Status = CreateSparseImgFlashThread (ThreadFlashInfo);
      } else {
        IsFlashComplete = FALSE;

        Status = HandleUsbEventsInTimer ();
        if (EFI_ERROR (Status)) {
          DEBUG ((EFI_D_ERROR, "Failed to handle usb event: %r\n", Status));
          IsFlashComplete = TRUE;
          StopUsbTimer ();
        } else {
          UsbTimerStarted = TRUE;
        }
      }

      if (!EFI_ERROR (Status)) {
        FastbootOkay ("");
      }
    }

    if (EFI_ERROR (Status) ||
      !IsUseMThreadParallel () ||
      (PartitionSize <= MaxDownLoadSize)) {
      FlashResult = HandleSparseImgFlash (PartitionName,
                                        ARRAY_SIZE (PartitionName),
                                        mFlashDataBuffer, mFlashNumDataBytes);
      IsFlashComplete = TRUE;
      StopUsbTimer ();
    }
  } else if (!AsciiStrnCmp (UbiHeader->HdrMagic, UBI_HEADER_MAGIC, 4)) {
    FlashResult = HandleUbiImgFlash (PartitionName,
                                     ARRAY_SIZE (PartitionName),
                                     mFlashDataBuffer,
                                     mFlashNumDataBytes);
  } else if (meta_header->magic == META_HEADER_MAGIC) {

    FlashResult = HandleMetaImgFlash (PartitionName,
                                      ARRAY_SIZE (PartitionName),
                                      mFlashDataBuffer, mFlashNumDataBytes);
  } else {

    FlashResult = HandleRawImgFlash (PartitionName,
                                     ARRAY_SIZE (PartitionName),
                                     mFlashDataBuffer, mFlashNumDataBytes);
  }

  /*
   * For Non-sparse image: Check flash result and update the result
   * Also, Handle if there is Failure in handling USB events especially for
   * sparse images.
   */
  if ((sparse_header->magic != SPARSE_HEADER_MAGIC) ||
        (PartitionSize < MaxDownLoadSize) ||
        ((PartitionSize > MaxDownLoadSize) &&
        (IsDisableParallelDownloadFlash () ||
        (Status != EFI_SUCCESS)))) {
    if (EFI_ERROR (FlashResult)) {
      if (FlashResult == EFI_NOT_FOUND) {
        AsciiSPrint (FlashResultStr, MAX_RSP_SIZE, "(%s) No such partition",
                     PartitionName);
      } else {
        AsciiSPrint (FlashResultStr, MAX_RSP_SIZE, "%a : %r",
                     "Error flashing partition", FlashResult);
      }

      DEBUG ((EFI_D_ERROR, "%a\n", FlashResultStr));
      FastbootFail (FlashResultStr);

      /* Reset the Flash Result for next flash command */
      FlashResult = EFI_SUCCESS;
      goto out;
    } else {
      DEBUG ((EFI_D_INFO, "flash image status:  %r\n", FlashResult));
      FastbootOkay ("");
    }
  }

out:
  if (!AsciiStrnCmp (arg, "system", AsciiStrLen ("system")) &&
    !IsEnforcing () &&
    (FlashResult == EFI_SUCCESS)) {
     // reset dm_verity mode to enforcing
    Status = EnableEnforcingMode (TRUE);
    if (Status != EFI_SUCCESS) {
      DEBUG ((EFI_D_ERROR, "failed to update verity mode:  %r\n", Status));
    }
  }

  LunSet = FALSE;
}

STATIC VOID
CmdErase (IN CONST CHAR8 *arg, IN VOID *data, IN UINT32 sz)
{
  EFI_STATUS Status;
  CHAR16 OutputString[FASTBOOT_STRING_MAX_LENGTH];
  BOOLEAN HasSlot = FALSE;
  CHAR16 SlotSuffix[MAX_SLOT_SUFFIX_SZ];
  BOOLEAN MultiSlotBoot = PartitionHasMultiSlot (L"boot");
  CHAR16 PartitionName[MAX_GPT_NAME_SIZE];
  CHAR8 EraseResultStr[MAX_RSP_SIZE] = "";
  VirtualAbMergeStatus SnapshotMergeStatus;

  WaitForFlashFinished ();

  if (AsciiStrLen (arg) >= MAX_GPT_NAME_SIZE) {
    FastbootFail ("Invalid partition name");
    return;
  }
  AsciiStrToUnicodeStr (arg, PartitionName);

#ifdef ASUS_AI2205_BUILD
  // +++ ASUS_BSP : add for fastboot permission
  if(!ASUS_FASTBOOT_PERMISSION && !UserErasePartitionCheck(PartitionName))
  {
    if(IsUnlocked() && VerifiedBootPartitionCheck(PartitionName))
    {
        DEBUG ((EFI_D_INFO, "[ABL] Only allow erase hlos image in lock state\n"));
    }else if (IsUnlocked() && !VerifiedBootPartitionCheck(PartitionName))
    {
        FastbootFail("Eraseing non-hlos image is not allowed in lock state");
        return;
    }
    else
    {
        FastbootFail("Eraseing is not allowed in Lock State");
        return;
    }
  }
  
  /* MISC parttion erase permission check */
  if(!StrnCmp (PartitionName, L"misc", StrLen (L"misc"))){
	  if(!ASUS_FASTBOOT_PERMISSION && (!IsASUSRawFlash)){
		  FastbootFail("Erasing misc is not allowed in Lock State");
          return;
	  }
  }

  /* Handle ASUS ASDF partition */
  if (!StrnCmp (PartitionName, L"asdf", StrLen (L"asdf"))) {
    if(!ASUS_FASTBOOT_PERMISSION && !is_ftm_mode() && !g_allow_flash_and_erase_asdf){
      DEBUG ((EFI_D_INFO, "[ABL] Skip erasing asdf partition\n"));
      AsciiSPrint (EraseResultStr, sizeof(EraseResultStr),"");
      FastbootInfo (EraseResultStr);
      WaitForTransferComplete();
      AsciiSPrint (EraseResultStr, sizeof(EraseResultStr),
                   "ASUS_FASTBOOT_PERMISSION=%a",
                    ASUS_FASTBOOT_PERMISSION? "TRUE" : "FALSE");
      FastbootInfo (EraseResultStr);
      WaitForTransferComplete();
      AsciiSPrint (EraseResultStr, sizeof(EraseResultStr),
                   "FTM_MODE=%a",
                    is_ftm_mode()? "TRUE" : "FALSE");
      FastbootInfo (EraseResultStr);
      WaitForTransferComplete();
      AsciiSPrint (EraseResultStr, sizeof(EraseResultStr),
                   "ASDF_FLAG=%a",
                    g_allow_flash_and_erase_asdf? "TRUE" : "FALSE");
      FastbootInfo (EraseResultStr);
      WaitForTransferComplete();
      AsciiSPrint (EraseResultStr, sizeof(EraseResultStr),
                   "Skip erasing %s partition", PartitionName);
      FastbootInfo (EraseResultStr);
      WaitForTransferComplete();
      FastbootOkay ("");
      return;
    }
  }

#else

  if ((GetAVBVersion () == AVB_LE) ||
      ((GetAVBVersion () != AVB_LE) &&
      (TargetBuildVariantUser ()))) {
    if (!IsUnlocked ()) {
      FastbootFail ("Erase is not allowed in Lock State");
      return;
    }

    if (!IsUnlockCritical () && IsCriticalPartition (PartitionName)) {
      FastbootFail ("Erase is not allowed for Critical Partitions\n");
      return;
    }
  }
#endif

  if (IsDynamicPartitionSupport ()) {
    /* Virtual A/B feature is enabled by default. */
#ifdef ASUS_BUILD
    if (CheckVirtualAbCriticalPartition (PartitionName) && !ASUS_FASTBOOT_PERMISSION) {
#else
    if (CheckVirtualAbCriticalPartition (PartitionName)) {
#endif
      AsciiSPrint (EraseResultStr, MAX_RSP_SIZE,
                    "Erase of %s is not allowed in %a state",
                    PartitionName, SnapshotMergeState);
      FastbootFail (EraseResultStr);
      return;
    }

    SnapshotMergeStatus = GetSnapshotMergeStatus ();
    if (((SnapshotMergeStatus == MERGING) ||
          (SnapshotMergeStatus == SNAPSHOTTED)) &&
          !StrnCmp (PartitionName, L"super", StrLen (L"super"))) {

      Status = SetSnapshotMergeStatus (CANCELLED);
      if (Status != EFI_SUCCESS) {
        FastbootFail ("Failed to update snapshot state to cancel");
        return;
      }

      //updating fbvar snapshot-merge-state
      AsciiSPrint (SnapshotMergeState,
                    AsciiStrLen (VabSnapshotMergeStatus[NONE_MERGE_STATUS]) + 1,
                    "%a", VabSnapshotMergeStatus[NONE_MERGE_STATUS]);
    }
  }

  /* Handle virtual partition avb_custom_key */
  if (!StrnCmp (PartitionName, L"avb_custom_key", StrLen (L"avb_custom_key"))) {
    DEBUG ((EFI_D_INFO, "erasing avb_custom_key\n"));
    Status = EraseUserKey ();
    if (Status != EFI_SUCCESS) {
      FastbootFail ("Erasing avb_custom_key failed");
    } else {
      FastbootOkay ("");
    }
    return;
  }

  /* In A/B to have backward compatibility user can still give fastboot flash
   * boot/system/modem etc
   * based on current slot Suffix try to look for "partition"_a/b if not found
   * fall back to look for
   * just the "partition" in case some of the partitions are no included for A/B
   * implementation
   */
  if (MultiSlotBoot)
    HasSlot = GetPartitionHasSlot (PartitionName, ARRAY_SIZE (PartitionName),
                                   SlotSuffix, MAX_SLOT_SUFFIX_SZ);

  // Build output string
  UnicodeSPrint (OutputString, sizeof (OutputString),
                 L"Erasing partition %s\r\n", PartitionName);
  Status = FastbootErasePartition (PartitionName);
  if (EFI_ERROR (Status)) {
    FastbootFail ("Check device console.");
    DEBUG ((EFI_D_ERROR, "Couldn't erase image:  %r\n", Status));
  } else {
    if (MultiSlotBoot && HasSlot &&
        !(StrnCmp (PartitionName, L"boot", StrLen (L"boot"))))
      FastbootUpdateAttr (SlotSuffix);
    FastbootOkay ("");
  }
}

/*Function to set given slot as high priority
 *Arg: slot Suffix
 *Note: increase the priority of slot to max priority
 *at the same time decrease the priority of other
 *slots.
 */
VOID
CmdSetActive (CONST CHAR8 *Arg, VOID *Data, UINT32 Size)
{
  CHAR16 SetActive[MAX_GPT_NAME_SIZE] = L"boot";
  CHAR8 *InputSlot = NULL;
  CHAR16 InputSlotInUnicode[MAX_SLOT_SUFFIX_SZ];
  CHAR16 InputSlotInUnicodetemp[MAX_SLOT_SUFFIX_SZ];
  CONST CHAR8 *Delim = ":";
  UINT16 j = 0;
  BOOLEAN SlotVarUpdateComplete = FALSE;
  UINT32 SlotEnd = 0;
  BOOLEAN MultiSlotBoot = PartitionHasMultiSlot (L"boot");
  Slot NewSlot = {{0}};
  EFI_STATUS Status;

#ifdef ASUS_BUILD
  if(TargetBuildVariantUser () && !ASUS_FASTBOOT_PERMISSION)
  {
    FastbootFail("Slot Change is not allowed in Lock State\n");
    return;
  }
#else
  if (TargetBuildVariantUser () && !IsUnlocked ()) {
    FastbootFail ("Slot Change is not allowed in Lock State\n");
    return;
  }
#endif

  if (!MultiSlotBoot) {
    FastbootFail ("This Command not supported");
    return;
  }

  if (!Arg) {
    FastbootFail ("Invalid Input Parameters");
    return;
  }

  if (IsDynamicPartitionSupport ()) {
    /* Virtual A/B feature is enabled by default.*/
#ifdef ASUS_BUILD
    if (GetSnapshotMergeStatus () == MERGING && !ASUS_FASTBOOT_PERMISSION) {
#else
    if (GetSnapshotMergeStatus () == MERGING) {
#endif
      FastbootFail ("Slot Change is not allowed in merging state");
      return;
    }
  }

  InputSlot = AsciiStrStr (Arg, Delim);
  if (InputSlot) {
    InputSlot++;
    if (AsciiStrLen (InputSlot) >= MAX_SLOT_SUFFIX_SZ) {
      FastbootFail ("Invalid Slot");
      return;
    }
    if (!AsciiStrStr (InputSlot, "_")) {
      AsciiStrToUnicodeStr (InputSlot, InputSlotInUnicodetemp);
      StrnCpyS (InputSlotInUnicode, MAX_SLOT_SUFFIX_SZ, L"_", StrLen (L"_"));
      StrnCatS (InputSlotInUnicode, MAX_SLOT_SUFFIX_SZ, InputSlotInUnicodetemp,
                StrLen (InputSlotInUnicodetemp));
    } else {
      AsciiStrToUnicodeStr (InputSlot, InputSlotInUnicode);
    }

    if ((AsciiStrLen (InputSlot) == MAX_SLOT_SUFFIX_SZ - 2) ||
        (AsciiStrLen (InputSlot) == MAX_SLOT_SUFFIX_SZ - 1)) {
      SlotEnd = AsciiStrLen (InputSlot);
      if ((InputSlot[SlotEnd] != '\0') ||
          !AsciiStrStr (SlotSuffixArray, InputSlot)) {
        DEBUG ((EFI_D_ERROR, "%a Invalid InputSlot Suffix\n", InputSlot));
        FastbootFail ("Invalid Slot Suffix");
        return;
      }
    }
    /*Arg will be either _a or _b, so apppend it to boot*/
    StrnCatS (SetActive, MAX_GPT_NAME_SIZE - 1, InputSlotInUnicode,
              StrLen (InputSlotInUnicode));
  } else {
    FastbootFail ("set_active _a or _b should be entered");
    return;
  }

  StrnCpyS (NewSlot.Suffix, ARRAY_SIZE (NewSlot.Suffix), InputSlotInUnicode,
            StrLen (InputSlotInUnicode));
  Status = SetActiveSlot (&NewSlot, TRUE);
  if (Status != EFI_SUCCESS) {
    FastbootFail ("set_active failed");
    return;
  }

  // Updating fbvar `current-slot'
  UnicodeStrToAsciiStr (GetCurrentSlotSuffix ().Suffix, CurrentSlotFB);

  /* Here CurrentSlotFB will only have value of "_a" or "_b".*/
  SKIP_FIRSTCHAR_IN_SLOT_SUFFIX (CurrentSlotFB);

  do {
    if (AsciiStrStr (BootSlotInfo[j].SlotSuffix, InputSlot)) {
      AsciiStrnCpyS (BootSlotInfo[j].SlotSuccessfulVal, ATTR_RESP_SIZE, "no",
                     AsciiStrLen ("no"));
      AsciiStrnCpyS (BootSlotInfo[j].SlotUnbootableVal, ATTR_RESP_SIZE, "no",
                     AsciiStrLen ("no"));
      AsciiSPrint (BootSlotInfo[j].SlotRetryCountVal,
                   sizeof (BootSlotInfo[j].SlotRetryCountVal), "%d",
                   MAX_RETRY_COUNT);
      SlotVarUpdateComplete = TRUE;
    }
    j++;
  } while (!SlotVarUpdateComplete);

  UpdatePartitionAttributes (PARTITION_ALL);
  FastbootOkay ("");
}
#endif

STATIC VOID
FlashCompleteHandler (IN EFI_EVENT Event, IN VOID *Context)
{
  EFI_STATUS Status = EFI_SUCCESS;

  /* Wait for flash completely before sending okay */
  if (!IsFlashComplete) {
    Status = gBS->SetTimer (Event, TimerRelative, 100000);
    if (EFI_ERROR (Status)) {
      FastbootFail ("Failed to set timer for waiting flash completely");
      goto Out;
    }
    return;
  }

  FastbootOkay ("");
Out:
  gBS->CloseEvent (Event);
  Event = NULL;
}

/* Parallel usb sending data and device writing data
 * It's need to delay to send okay until flashing finished for
 * next command.
 */
STATIC EFI_STATUS FastbootOkayDelay (VOID)
{
  EFI_STATUS Status = EFI_SUCCESS;
  EFI_EVENT FlashCompleteEvent = NULL;

  Status = gBS->CreateEvent (EVT_TIMER | EVT_NOTIFY_SIGNAL, TPL_CALLBACK,
                             FlashCompleteHandler, NULL, &FlashCompleteEvent);
  if (EFI_ERROR (Status)) {
    FastbootFail ("Failed to creat event for waiting flash completely");
    return Status;
  }

  Status = gBS->SetTimer (FlashCompleteEvent, TimerRelative, 100000);
  if (EFI_ERROR (Status)) {
    gBS->CloseEvent (FlashCompleteEvent);
    FlashCompleteEvent = NULL;
    FastbootFail ("Failed to set timer for waiting flash completely");
  }

  return Status;
}

STATIC VOID
AcceptData (IN UINT64 Size, IN VOID *Data)
{
  UINT64 RemainingBytes = mNumDataBytes - mBytesReceivedSoFar;
  UINT32 PageSize = 0;
  UINT32 RoundSize = 0;

  /* Protocol doesn't say anything about sending extra data so just ignore it.*/
  if (Size > RemainingBytes) {
    Size = RemainingBytes;
  }

  mBytesReceivedSoFar += Size;

  /* Either queue the max transfer size 1 MB or only queue the remaining
   * amount of data left to avoid zlt issues
   */
  if (mBytesReceivedSoFar == mNumDataBytes) {
    /* Download Finished */
    DEBUG ((EFI_D_INFO, "Download Finished\n"));
    /* Zero initialized the surplus data buffer. It's risky to access the data
     * buffer which it's not zero initialized, its content might leak
     */
    GetPageSize (&PageSize);
    RoundSize = ROUND_TO_PAGE (mNumDataBytes, PageSize - 1);
    if (RoundSize < MaxDownLoadSize) {
      gBS->SetMem ((VOID *)(Data + mNumDataBytes), RoundSize - mNumDataBytes,
                   0);
    }
    mState = ExpectCmdState;

    if (IsUseMThreadParallel ())  {
      KernIntf->Lock->ReleaseLock (LockDownload);
      FastbootOkay ("");
    } else {
      /* Stop usb timer after data transfer completed */
      StopUsbTimer ();
      /* Postpone Fastboot Okay until flash completed */
      FastbootOkayDelay ();
    }
  } else {
    GetFastbootDeviceData ()->UsbDeviceProtocol->Send (
        ENDPOINT_IN, GetXfrSize (), (Data + mBytesReceivedSoFar));
    DEBUG ((EFI_D_VERBOSE, "AcceptData: Send %d\n", GetXfrSize ()));
  }
}

/* Called based on the event received from USB device protocol:
 */
VOID
DataReady (IN UINT64 Size, IN VOID *Data)
{
  DEBUG ((EFI_D_VERBOSE, "DataReady %d\n", Size));
  if (mState == ExpectCmdState)
    AcceptCmd (Size, (CHAR8 *)Data);
  else if (mState == ExpectDataState)
    AcceptData (Size, Data);
  else {
    DEBUG ((EFI_D_ERROR, "DataReady Unknown status received\r\n"));
    return;
  }
}

STATIC VOID
FatalErrorNotify (IN EFI_EVENT Event, IN VOID *Context)
{
  DEBUG ((EFI_D_ERROR, "Fatal error sending command response. Exiting.\r\n"));
  Finished = TRUE;
}

/* Fatal error during fastboot */
BOOLEAN FastbootFatal (VOID)
{
  return Finished;
}

/* This function must be called to deallocate the USB buffers, as well
 * as the main Fastboot Buffer. Also Frees Variable data Structure
 */
EFI_STATUS
FastbootCmdsUnInit (VOID)
{
  EFI_STATUS Status;

  if (mDataBuffer) {
    Status = GetFastbootDeviceData ()->UsbDeviceProtocol->FreeTransferBuffer (
        (VOID *)mDataBuffer);
    if (Status != EFI_SUCCESS) {
      DEBUG ((EFI_D_ERROR, "Failed to free up fastboot buffer\n"));
      return Status;
    }
  }
  FastbootUnInit ();
  GetFastbootDeviceData ()->UsbDeviceProtocol->Stop ();
  return EFI_SUCCESS;
}

/* This function must be called to check maximum allocatable chunk for
 * Fastboot Buffer.
 */
STATIC VOID
GetMaxAllocatableMemory (
  OUT UINT64 *FreeSize
  )
{
  EFI_MEMORY_DESCRIPTOR       *MemMap;
  EFI_MEMORY_DESCRIPTOR       *MemMapPtr;
  UINTN                       MemMapSize;
  UINTN                       MapKey, DescriptorSize;
  UINTN                       Index;
  UINTN                       MaxFree = 0;
  UINT32                      DescriptorVersion;
  EFI_STATUS                  Status;

  MemMapSize = 0;
  MemMap     = NULL;
  *FreeSize = 0;

  // Get size of current memory map.
  Status = gBS->GetMemoryMap (&MemMapSize, MemMap, &MapKey,
                              &DescriptorSize, &DescriptorVersion);
  /*
    If the MemoryMap buffer is too small, the EFI_BUFFER_TOO_SMALL error
    code is returned and the MemoryMapSize value contains the size of
    the buffer needed to contain the current memory map.
    The actual size of the buffer allocated for the consequent call
    to GetMemoryMap() should be bigger then the value returned in
    MemMapSize, since allocation of the new buffer may
    potentially increase memory map size.
  */
  if (Status != EFI_BUFFER_TOO_SMALL) {
    DEBUG ((EFI_D_ERROR, "ERROR: Undefined response get memory map\n"));
    return;
  }

  /*
    Allocate some additional memory as returned by MemMapSize,
    and query current memory map.
  */
  if (CHECK_ADD64 (MemMapSize, EFI_PAGE_SIZE)) {
    DEBUG ((EFI_D_ERROR, "ERROR: integer Overflow while adding additional"
                         "memory to MemMapSize"));
    return;
  }
  MemMapSize = MemMapSize + EFI_PAGE_SIZE;
  MemMap = AllocateZeroPool (MemMapSize);
  if (!MemMap) {
    DEBUG ((EFI_D_ERROR,
                    "ERROR: Failed to allocate memory for memory map\n"));
    return;
  }

  // Store pointer to be freed later.
  MemMapPtr = MemMap;
  // Get System MemoryMap
  Status = gBS->GetMemoryMap (&MemMapSize, MemMap, &MapKey,
                              &DescriptorSize, &DescriptorVersion);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "ERROR: Failed to query memory map\n"));
    FreePool (MemMapPtr);
    return;
  }

  // Find largest free chunk of unallocated memory available.
  for (Index = 0; Index < MemMapSize / DescriptorSize; Index ++) {
    if (MemMap->Type == EfiConventionalMemory &&
          MaxFree < MemMap->NumberOfPages) {
          MaxFree = MemMap->NumberOfPages;
    }
    MemMap = (EFI_MEMORY_DESCRIPTOR *)((UINTN)MemMap + DescriptorSize);
  }

  *FreeSize = EFI_PAGES_TO_SIZE (MaxFree);
  DEBUG ((EFI_D_VERBOSE, "Free Memory available: %ld\n", *FreeSize));
  FreePool (MemMapPtr);
  return;
}

//Shoud block command until flash finished
VOID WaitForFlashFinished (VOID)
{
  if (!IsFlashComplete &&
    IsUseMThreadParallel ()) {
    KernIntf->Lock->AcquireLock (LockFlash);
    KernIntf->Lock->ReleaseLock (LockFlash);
  }
}

VOID ThreadSleep (TimeDuration Delay)
{
  KernIntf->Thread->ThreadSleep (Delay);
}

BOOLEAN IsUseMThreadParallel (VOID)
{
  if (FixedPcdGetBool (EnableMultiThreadFlash)) {
    return IsMultiThreadSupported;
  }

  return FALSE;
}

VOID InitMultiThreadEnv ()
{
  EFI_STATUS Status = EFI_SUCCESS;

  if (IsDisableParallelDownloadFlash ()) {
    return;
  }

  Status = gBS->LocateProtocol (&gEfiKernelProtocolGuid, NULL,
      (VOID **)&KernIntf);

  if ((Status != EFI_SUCCESS) ||
    (KernIntf == NULL) ||
    KernIntf->Version < EFI_KERNEL_PROTOCOL_VER_UNSAFE_STACK_APIS) {
    DEBUG ((EFI_D_VERBOSE, "Multi thread is not supported.\n"));
    return;
  }

  KernIntf->Lock->InitLock ("DOWNLOAD", &LockDownload);
  if (&LockDownload == NULL) {
     DEBUG ((EFI_D_ERROR, "InitLock LockDownload error \n"));
     return;
  }

  KernIntf->Lock->InitLock ("FLASH", &LockFlash);
  if (&LockFlash == NULL) {
    DEBUG ((EFI_D_ERROR, "InitLock LockFlash error \n"));
    KernIntf->Lock->DestroyLock (LockDownload);
    return;
  }

  //init MultiThreadEnv succeeded, use multi thread to flash
  IsMultiThreadSupported = TRUE;

  DEBUG ((EFI_D_VERBOSE,
          "InitMultiThreadEnv successfully, will use thread to flash \n"));
}

EFI_STATUS
FastbootCmdsInit (VOID)
{
  EFI_STATUS Status;
  EFI_EVENT mFatalSendErrorEvent;
  CHAR8 *FastBootBuffer;

  mDataBuffer = NULL;
  mUsbDataBuffer = NULL;
  mFlashDataBuffer = NULL;

  DEBUG ((EFI_D_INFO, "Fastboot: Initializing...\n"));

  /* Disable watchdog */
  Status = gBS->SetWatchdogTimer (0, 0x10000, 0, NULL);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "Fastboot: Couldn't disable watchdog timer: %r\n",
            Status));
  }

  /* Create event to pass to FASTBOOT_PROTOCOL.Send, signalling a fatal error */
  Status = gBS->CreateEvent (EVT_NOTIFY_SIGNAL, TPL_CALLBACK, FatalErrorNotify,
                             NULL, &mFatalSendErrorEvent);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "Couldn't create Fastboot protocol send event: %r\n",
            Status));
    return Status;
  }

  /* Allocate buffer used to store images passed by the download command */
  GetMaxAllocatableMemory (&MaxDownLoadSize);
  if (!MaxDownLoadSize) {
    DEBUG ((EFI_D_ERROR, "Failed to get free memory for fastboot buffer\n"));
    return EFI_OUT_OF_RESOURCES;
  }

  do {
    // Try allocating 3/4th of free memory available.
    MaxDownLoadSize = EFI_FREE_MEM_DIVISOR (MaxDownLoadSize);
    MaxDownLoadSize = LOCAL_ROUND_TO_PAGE (MaxDownLoadSize, EFI_PAGE_SIZE);
    if (MaxDownLoadSize < MIN_BUFFER_SIZE) {
      DEBUG ((EFI_D_ERROR,
        "ERROR: Allocation fail for minimim buffer for fastboot\n"));
      return EFI_OUT_OF_RESOURCES;
    }

    /* If available buffer on target is more than max buffer size,
       we limit this to max buffer buffer size we support */
    if (MaxDownLoadSize > MAX_BUFFER_SIZE) {
      MaxDownLoadSize = MAX_BUFFER_SIZE;
    }

    Status =
        GetFastbootDeviceData ()->UsbDeviceProtocol->AllocateTransferBuffer (
                                      MaxDownLoadSize,
                                      (VOID **)&FastBootBuffer);
  }while (EFI_ERROR (Status));

  if (Status != EFI_SUCCESS) {
    DEBUG ((EFI_D_ERROR, "Not enough memory to Allocate Fastboot Buffer\n"));
    return Status;
  }

  /* Clear allocated buffer */
  gBS->SetMem ((VOID *)FastBootBuffer, MaxDownLoadSize , 0x0);
  DEBUG ((EFI_D_VERBOSE,
                  "Fastboot Buffer Size allocated: %ld\n", MaxDownLoadSize));

  MaxDownLoadSize = (CheckRootDeviceType () == NAND) ?
                              MaxDownLoadSize : MaxDownLoadSize / 2;

  FastbootCommandSetup ((VOID *)FastBootBuffer, MaxDownLoadSize);

  InitMultiThreadEnv ();

  return EFI_SUCCESS;
}

/* See header for documentation */
VOID
FastbootRegister (IN CONST CHAR8 *prefix,
                  IN VOID (*handle) (CONST CHAR8 *arg, VOID *data, UINT32 sz))
{
  FASTBOOT_CMD *cmd;

  cmd = AllocateZeroPool (sizeof (*cmd));
  if (cmd) {
    cmd->prefix = prefix;
    cmd->prefix_len = AsciiStrLen (prefix);
    cmd->handle = handle;
    cmd->next = cmdlist;
    cmdlist = cmd;
  } else {
    DEBUG ((EFI_D_VERBOSE,
            "Failed to allocate memory to cmd\n"));
  }
}

STATIC VOID
CmdReboot (IN CONST CHAR8 *arg, IN VOID *data, IN UINT32 sz)
{
  DEBUG ((EFI_D_INFO, "rebooting the device\n"));
  FastbootOkay ("");

  RebootDevice (NORMAL_MODE);

  // Shouldn't get here
  FastbootFail ("Failed to reboot");
}

STATIC VOID
CmdRebootRecovery (IN CONST CHAR8 *Arg, IN VOID *Data, IN UINT32 Size)
{
  EFI_STATUS Status = EFI_SUCCESS;

  Status = WriteRecoveryMessage (RECOVERY_BOOT_RECOVERY);
  if (Status != EFI_SUCCESS) {
    FastbootFail ("Failed to reboot to recovery mode");
    return;
  }
  DEBUG ((EFI_D_INFO, "rebooting the device to recovery\n"));
  FastbootOkay ("");

  RebootDevice (NORMAL_MODE);

  // Shouldn't get here
  FastbootFail ("Failed to reboot");
}

STATIC VOID
CmdRebootFastboot (IN CONST CHAR8 *Arg, IN VOID *Data, IN UINT32 Size)
{
  EFI_STATUS Status = EFI_SUCCESS;
  Status = WriteRecoveryMessage (RECOVERY_BOOT_FASTBOOT);
  if (Status != EFI_SUCCESS) {
    FastbootFail ("Failed to reboot to fastboot mode");
    return;
  }
  DEBUG ((EFI_D_INFO, "rebooting the device to fastbootd\n"));
  FastbootOkay ("");

  RebootDevice (NORMAL_MODE);

  // Shouldn't get here
  FastbootFail ("Failed to reboot");
}

STATIC VOID
CmdUpdateSnapshot (IN CONST CHAR8 *Arg, IN VOID *Data, IN UINT32 Size)
{
  CHAR8 *Command = NULL;
  CONST CHAR8 *Delim = ":";
  EFI_STATUS Status = EFI_SUCCESS;

  Command = AsciiStrStr (Arg, Delim);
  if (Command) {
    Command++;

    if (!AsciiStrnCmp (Command, "merge", AsciiStrLen ("merge"))) {
      if (GetSnapshotMergeStatus () == MERGING) {
        CmdRebootFastboot (Arg, Data, Size);
      }
      FastbootOkay ("");
      return;
    } else if (!AsciiStrnCmp (Command, "cancel", AsciiStrLen ("cancel"))) {
#ifdef ASUS_BUILD
      if(!ASUS_FASTBOOT_PERMISSION) {
#else
      if (!IsUnlocked ()) {
#endif
        FastbootFail ("Snapshot Cancel is not allowed in Lock State");
        return;
      }

      Status = SetSnapshotMergeStatus (CANCELLED);
      if (Status != EFI_SUCCESS) {
        FastbootFail ("Failed to update snapshot state to cancel");
        return;
      }

      //updating fbvar snapshot-merge-state
      AsciiSPrint (SnapshotMergeState,
                    AsciiStrLen (VabSnapshotMergeStatus[NONE_MERGE_STATUS]) + 1,
                    "%a", VabSnapshotMergeStatus[NONE_MERGE_STATUS]);
      FastbootOkay ("");
      return;
    }
  }
  FastbootFail ("Invalid snapshot-update command");
  return;
}

STATIC VOID
CmdContinue (IN CONST CHAR8 *Arg, IN VOID *Data, IN UINT32 Size)
{
  EFI_STATUS Status = EFI_SUCCESS;
  CHAR8 Resp[MAX_RSP_SIZE];
  BootInfo Info = {0};

  Info.MultiSlotBoot = PartitionHasMultiSlot ((CONST CHAR16 *)L"boot");
  Status = LoadImageAndAuth (&Info, FALSE, FALSE);
  if (Status != EFI_SUCCESS) {
    AsciiSPrint (Resp, sizeof (Resp), "Failed to load image from partition: %r",
                 Status);
    FastbootFail (Resp);
    return;
  }

  /* Exit keys' detection firstly */
  ExitMenuKeysDetection ();

  FastbootOkay ("");
  FastbootUsbDeviceStop ();
  Finished = TRUE;
  // call start Linux here
  BootLinux (&Info);
}

STATIC VOID UpdateGetVarVariable (VOID)
{
  BOOLEAN BatterySocOk = FALSE;
  UINT32 BatteryVoltage = 0;

  BatterySocOk = TargetBatterySocOk (&BatteryVoltage);
  AsciiSPrint (StrBatteryVoltage, sizeof (StrBatteryVoltage), "%d",
               BatteryVoltage);
  AsciiSPrint (StrBatterySocOk, sizeof (StrBatterySocOk), "%a",
               BatterySocOk ? "yes" : "no");
  AsciiSPrint (ChargeScreenEnable, sizeof (ChargeScreenEnable), "%d",
               IsChargingScreenEnable ());
  AsciiSPrint (OffModeCharge, sizeof (OffModeCharge), "%d",
               IsChargingScreenEnable ());
}

STATIC VOID CmdGetVarAll (VOID)
{
  FASTBOOT_VAR *Var;
  CHAR8 GetVarAll[MAX_RSP_SIZE];

  for (Var = Varlist; Var; Var = Var->next) {
    AsciiStrnCpyS (GetVarAll, sizeof (GetVarAll), Var->name, MAX_RSP_SIZE);
    AsciiStrnCatS (GetVarAll, sizeof (GetVarAll), ":", AsciiStrLen (":"));
    AsciiStrnCatS (GetVarAll, sizeof (GetVarAll), Var->value, MAX_RSP_SIZE);
    FastbootInfo (GetVarAll);
    /* Wait for the transfer to complete */
    WaitForTransferComplete ();
    ZeroMem (GetVarAll, sizeof (GetVarAll));
  }

  FastbootOkay (GetVarAll);
}

STATIC VOID
CmdGetVar (CONST CHAR8 *Arg, VOID *Data, UINT32 Size)
{
  FASTBOOT_VAR *Var;
  Slot CurrentSlot;
  CHAR16 PartNameUniStr[MAX_GPT_NAME_SIZE];
  CHAR8 *Token = AsciiStrStr (Arg, "partition-");
  CHAR8 CurrentSlotAsc[MAX_SLOT_SUFFIX_SZ];

  UpdateGetVarVariable ();

  if (!(AsciiStrCmp ("all", Arg))) {
    CmdGetVarAll ();
    return;
  }

  if (Token) {
    Token = AsciiStrStr (Arg, ":");
    if (Token) {
      Token = Token + AsciiStrLen (":");
      if (AsciiStrLen (Token) >= ARRAY_SIZE (PartNameUniStr)) {
        FastbootFail ("Invalid partition name");
        return;
      }

      AsciiStrToUnicodeStr (Token, PartNameUniStr);

      if (PartitionHasMultiSlot (PartNameUniStr)) {
        CurrentSlot = GetCurrentSlotSuffix ();
        UnicodeStrToAsciiStr (CurrentSlot.Suffix, CurrentSlotAsc);
        AsciiStrnCatS ((CHAR8 *)Arg,
                        MAX_FASTBOOT_COMMAND_SIZE - AsciiStrLen ("getvar:"),
                        CurrentSlotAsc,
                        AsciiStrLen (CurrentSlotAsc));
      }
    }
  }

  for (Var = Varlist; Var; Var = Var->next) {
    if (!AsciiStrCmp (Var->name, Arg)) {
      FastbootOkay (Var->value);
      return;
    }
  }

  FastbootFail ("GetVar Variable Not found");
}

#ifdef ENABLE_BOOT_CMD
STATIC VOID
CmdBoot (CONST CHAR8 *Arg, VOID *Data, UINT32 Size)
{
  boot_img_hdr *hdr = Data;
  boot_img_hdr_v3 *HdrV3 = Data;
  EFI_STATUS Status = EFI_SUCCESS;
  UINT32 ImageSizeActual = 0;
  UINT32 PageSize = 0;
  UINT32 SigActual = SIGACTUAL;
  CHAR8 Resp[MAX_RSP_SIZE];
  BOOLEAN MdtpActive = FALSE;
  BootInfo Info = {0};

  if (FixedPcdGetBool (EnableMdtpSupport)) {
    Status = IsMdtpActive (&MdtpActive);

    if (EFI_ERROR (Status)) {
      FastbootFail (
          "Failed to get MDTP activation state, blocking fastboot boot");
      return;
    }

    if (MdtpActive == TRUE) {
      FastbootFail (
          "Fastboot boot command is not available while MDTP is active");
      return;
    }
  }
  if (!IsUnlocked ()) {
    FastbootFail (
          "Fastboot boot command is not available in locked device");
      return;
  }
  if (Size < sizeof (boot_img_hdr)) {
    FastbootFail ("Invalid Boot image Header");
    return;
  }

  if (hdr->header_version <= BOOT_HEADER_VERSION_TWO) {
    hdr->cmdline[BOOT_ARGS_SIZE - 1] = '\0';
  } else {
    HdrV3->cmdline[BOOT_ARGS_SIZE + BOOT_EXTRA_ARGS_SIZE - 1] = '\0';
  }

  SetBootDevImage ();

  Info.Images[0].ImageBuffer = Data;
  /* The actual image size will be updated in LoadImageAndAuth */
  Info.Images[0].ImageSize = Size;
  Info.Images[0].Name = "boot";
  Info.NumLoadedImages = 1;
  Info.MultiSlotBoot = PartitionHasMultiSlot (L"boot");

  if (Info.MultiSlotBoot) {
    Status = ClearUnbootable ();
    if (Status != EFI_SUCCESS) {
      FastbootFail ("CmdBoot: ClearUnbootable failed");
      goto out;
    }
  }

  Status = LoadImageAndAuth (&Info, FALSE, FALSE);
  if (Status != EFI_SUCCESS) {
    AsciiSPrint (Resp, sizeof (Resp),
                 "Failed to load/authenticate boot image: %r", Status);
    FastbootFail (Resp);
    goto out;
  }

  ImageSizeActual = Info.Images[0].ImageSize;

  if (ImageSizeActual > Size) {
    FastbootFail ("BootImage is Incomplete");
    goto out;
  }
  if ((MaxDownLoadSize - (ImageSizeActual - SigActual)) < PageSize) {
    FastbootFail ("BootImage: Size is greater than boot image buffer can hold");
    goto out;
  }

  /* Exit keys' detection firstly */
  ExitMenuKeysDetection ();

  FastbootOkay ("");
  FastbootUsbDeviceStop ();
  ResetBootDevImage ();
  BootLinux (&Info);

out:
  ResetBootDevImage ();
  return;
}
#endif

STATIC VOID
CmdRebootBootloader (CONST CHAR8 *arg, VOID *data, UINT32 sz)
{
  DEBUG ((EFI_D_INFO, "Rebooting the device into bootloader mode\n"));
  FastbootOkay ("");
  RebootDevice (FASTBOOT_MODE);

  // Shouldn't get here
  FastbootFail ("Failed to reboot");
}

#ifndef ASUS_BUILD
#if (defined(ENABLE_DEVICE_CRITICAL_LOCK_UNLOCK_CMDS) ||                       \
     defined(ENABLE_UPDATE_PARTITIONS_CMDS))
STATIC UINT8
is_display_supported ( VOID )
{
  EFI_STATUS Status = EFI_SUCCESS;
  EfiQcomDisplayUtilsProtocol *pDisplayUtilProtocol;
  EFI_GUID DisplayUtilGUID = EFI_DISPLAYUTILS_PROTOCOL_GUID;
  EFI_DISPLAY_UTILS_PANEL_CONFIG_PARAMS PanelConfig;
  UINT32 Index = 0;
  UINT32 ParamSize = sizeof (PanelConfig);
  PanelConfig.uPanelIndex = Index;

  if (EFI_SUCCESS == gBS->LocateProtocol (&DisplayUtilGUID,
                                    NULL,
                                    (VOID **)&pDisplayUtilProtocol)) {
     Status = pDisplayUtilProtocol->DisplayUtilsGetProperty (
                                     EFI_DISPLAY_UTILS_PANEL_CONFIG,
                                    (VOID*)&PanelConfig, &ParamSize);
     if ( Status == EFI_NOT_FOUND ) {
       DEBUG ((EFI_D_VERBOSE, "Display is not supported\n"));
       return 0;
     }
   }
   DEBUG ((EFI_D_VERBOSE, "Display is enabled\n"));
   return 1;
}

STATIC VOID
SetDeviceUnlock (UINT32 Type, BOOLEAN State)
{
  BOOLEAN is_unlocked = FALSE;
  char response[MAX_RSP_SIZE] = {0};
  EFI_STATUS Status;

  if (Type == UNLOCK)
    is_unlocked = IsUnlocked ();
  else if (Type == UNLOCK_CRITICAL)
    is_unlocked = IsUnlockCritical ();
  if (State == is_unlocked) {
    AsciiSPrint (response, MAX_RSP_SIZE, "\tDevice already : %a",
                 (State ? "unlocked!" : "locked!"));
    FastbootFail (response);
    return;
  }

  /* If State is TRUE that means set the unlock to true */
  if (State && !IsAllowUnlock) {
    FastbootFail ("Flashing Unlock is not allowed\n");
    return;
  }

  if (GetAVBVersion () != AVB_LE &&
      is_display_supported () &&
      IsEnableDisplayMenuFlagSupported ()) {
    Status = DisplayUnlockMenu (Type, State);
    if (Status != EFI_SUCCESS) {
      FastbootFail ("Command not support: the display is not enabled");
      return;
    } else {
      FastbootOkay ("");
    }
  } else {
    Status = SetDeviceUnlockValue (Type, State);
    if (Status != EFI_SUCCESS) {
         AsciiSPrint (response, MAX_RSP_SIZE, "\tSet device %a failed: %r",
                  (State ? "unlocked!" : "locked!"), Status);
         FastbootFail (response);
         return;
    }
    FastbootOkay ("");
    if (GetAVBVersion () != AVB_LE &&
       !IsEnableDisplayMenuFlagSupported ()) {
      RebootDevice (RECOVERY_MODE);
    }
  }
}
#endif
#endif

#ifdef ENABLE_UPDATE_PARTITIONS_CMDS
STATIC VOID
CmdFlashingUnlock (CONST CHAR8 *arg, VOID *data, UINT32 sz)
{
#ifdef ASUS_BUILD

  // +++ ASUS_BSP : add for fastboot permission
  if(!ASUS_FASTBOOT_PERMISSION)
  {
    FastbootFail("permission denied");
    return;
  }
  // --- ASUS_BSP : add for fastboot permission

  if(IsUnlocked() == TRUE)
  {
    FastbootFail("Device already : unlocked!");
    return;
  }

  SetDeviceUnlockValue (UNLOCK, TRUE);
  FastbootOkay("");
  if (IsAuthorized() && !IsAuthorized_2()){
    RebootDevice(RECOVERY_MODE);
  }

#else
  SetDeviceUnlock (UNLOCK, TRUE);
#endif
}

STATIC VOID
CmdFlashingLock (CONST CHAR8 *arg, VOID *data, UINT32 sz)
{
#ifdef ASUS_BUILD
  UINT32 lock_count;

  // +++ ASUS_BSP : add for fastboot permission
  if(!ASUS_FASTBOOT_PERMISSION)
  {
    FastbootFail("permission denied");
    return;
  }
  // --- ASUS_BSP : add for fastboot permission

  if( IsUnlocked() == FALSE)
  {
    FastbootFail("Device already : locked!");
    return;
  }

  SetDeviceUnlockValue (UNLOCK, FALSE);

  lock_count = GetLockCounter();
  lock_count++;
  SetLockCounter(lock_count);

  FastbootOkay("");
  if (IsAuthorized() && !IsAuthorized_2()){
    RebootDevice(RECOVERY_MODE);
  }

#else
  SetDeviceUnlock (UNLOCK, FALSE);
#endif
}
#endif

#ifdef ENABLE_DEVICE_CRITICAL_LOCK_UNLOCK_CMDS
STATIC VOID
CmdFlashingLockCritical (CONST CHAR8 *arg, VOID *data, UINT32 sz)
{
#ifdef ASUS_BUILD

  // +++ ASUS_BSP : add for fastboot permission
  if(!ASUS_FASTBOOT_PERMISSION)
  {
    FastbootFail("permission denied");
    return;
  }
  // --- ASUS_BSP : add for fastboot permission

  if( IsUnlockCritical() == FALSE)
  {
    FastbootFail("Device already : locked!");
    return;
  }
  
  SetDeviceUnlockValue (UNLOCK_CRITICAL, FALSE);
  FastbootOkay("");
  if (IsAuthorized() && !IsAuthorized_2()){
    RebootDevice(RECOVERY_MODE);
  }

#else
  SetDeviceUnlock (UNLOCK_CRITICAL, FALSE);
#endif
}

STATIC VOID
CmdFlashingUnLockCritical (CONST CHAR8 *arg, VOID *data, UINT32 sz)
{
#ifdef ASUS_BUILD

  // +++ ASUS_BSP : add for fastboot permission
  if(!ASUS_FASTBOOT_PERMISSION)
  {
    FastbootFail("permission denied");
    return;
  }
  // --- ASUS_BSP : add for fastboot permission

  if( IsUnlockCritical() == TRUE)
  {
    FastbootFail("Device already : unlocked!");
    return;
  }

  SetDeviceUnlockValue (UNLOCK_CRITICAL, TRUE);
  FastbootOkay("");
  if (IsAuthorized() && !IsAuthorized_2()){
    RebootDevice(RECOVERY_MODE);
  }

#else
  SetDeviceUnlock (UNLOCK_CRITICAL, TRUE);
#endif
}
#endif

STATIC VOID
CmdOemEnableChargerScreen (CONST CHAR8 *Arg, VOID *Data, UINT32 Size)
{
  EFI_STATUS Status;

#ifdef ASUS_BUILD
  // +++ ASUS_BSP : add for fastboot permission
  if(!ASUS_FASTBOOT_PERMISSION && !IsUnlocked())
  {
    FastbootFail("permission denied");
    return;
  }
  // --- ASUS_BSP : add for fastboot permission
#endif

  DEBUG ((EFI_D_INFO, "Enabling Charger Screen\n"));

  Status = EnableChargingScreen (TRUE);
  if (Status != EFI_SUCCESS) {
    FastbootFail ("Failed to enable charger screen");
  } else {
    FastbootOkay ("");
  }
}

STATIC VOID
CmdOemDisableChargerScreen (CONST CHAR8 *Arg, VOID *Data, UINT32 Size)
{
  EFI_STATUS Status;

#ifdef ASUS_BUILD
  // +++ ASUS_BSP : add for fastboot permission
  if(!ASUS_FASTBOOT_PERMISSION && !IsUnlocked())
  {
    FastbootFail("permission denied");
    return;
  }
  // --- ASUS_BSP : add for fastboot permission
#endif

  DEBUG ((EFI_D_INFO, "Disabling Charger Screen\n"));

  Status = EnableChargingScreen (FALSE);
  if (Status != EFI_SUCCESS) {
    FastbootFail ("Failed to disable charger screen");
  } else {
    FastbootOkay ("");
  }
}

STATIC VOID
CmdOemOffModeCharger (CONST CHAR8 *Arg, VOID *Data, UINT32 Size)
{
  CHAR8 *Ptr = NULL;
  CONST CHAR8 *Delim = " ";
  EFI_STATUS Status;
  BOOLEAN IsEnable = FALSE;
  CHAR8 Resp[MAX_RSP_SIZE] = "Set off mode charger: ";

#ifdef ASUS_BUILD
  // +++ ASUS_BSP : add for fastboot permission
  if(!ASUS_FASTBOOT_PERMISSION && !IsUnlocked())
  {
    FastbootFail("permission denied");
    return;
  }
  // --- ASUS_BSP : add for fastboot permission
#endif

  if (Arg) {
    Ptr = AsciiStrStr (Arg, Delim);
    if (Ptr) {
      Ptr++;
      if (!AsciiStrCmp (Ptr, "0"))
        IsEnable = FALSE;
      else if (!AsciiStrCmp (Ptr, "1"))
        IsEnable = TRUE;
      else {
        FastbootFail ("Invalid input entered");
        return;
      }
    } else {
      FastbootFail ("Enter fastboot oem off-mode-charge 0/1");
      return;
    }
  } else {
    FastbootFail ("Enter fastboot oem off-mode-charge 0/1");
    return;
  }

  AsciiStrnCatS (Resp, sizeof (Resp), Arg, AsciiStrLen (Arg));
  /* update charger_screen_enabled value for getvar command */
  Status = EnableChargingScreen (IsEnable);
  if (Status != EFI_SUCCESS) {
    AsciiStrnCatS (Resp, sizeof (Resp), ": failed", AsciiStrLen (": failed"));
    FastbootFail (Resp);
  } else {
    AsciiStrnCatS (Resp, sizeof (Resp), ": done", AsciiStrLen (": done"));
    FastbootOkay (Resp);
  }
}

STATIC EFI_STATUS
DisplaySetVariable (CHAR16 *VariableName, VOID *VariableValue, UINTN DataSize)
{
  EFI_STATUS Status = EFI_SUCCESS;
  BOOLEAN RTVariable = FALSE;
  EfiQcomDisplayUtilsProtocol *pDisplayUtilsProtocol = NULL;

  Status = gBS->LocateProtocol (&gQcomDisplayUtilsProtocolGuid,
                                NULL,
                                (VOID **)&pDisplayUtilsProtocol);
  if ((EFI_ERROR (Status)) ||
      (pDisplayUtilsProtocol == NULL)) {
    RTVariable = TRUE;
  } else if (pDisplayUtilsProtocol->Revision <  0x20000) {
    RTVariable = TRUE;
  } else {
    /* The display utils version for 0x20000 and above can support
       display protocol to get and set variable */
    Status = pDisplayUtilsProtocol->DisplayUtilsSetVariable (
          VariableName,
          (UINT8 *)VariableValue,
          DataSize,
          0);
  }

  if (RTVariable) {
    Status = gRT->SetVariable (VariableName,
                               &gQcomTokenSpaceGuid,
                               EFI_VARIABLE_RUNTIME_ACCESS |
                               EFI_VARIABLE_BOOTSERVICE_ACCESS |
                               EFI_VARIABLE_NON_VOLATILE,
                               DataSize,
                               (VOID *)VariableValue);
  }

  if (Status == EFI_NOT_FOUND) {
    // EFI_NOT_FOUND is not an error for retail case.
    Status = EFI_SUCCESS;
  } else if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_VERBOSE,
        "Display set variable failed with status(%d)!\n", Status));
  }

  return Status;
}

STATIC EFI_STATUS
DisplayGetVariable (CHAR16 *VariableName, VOID *VariableValue, UINTN *DataSize)
{
  EFI_STATUS Status = EFI_SUCCESS;
  BOOLEAN RTVariable = FALSE;
  EfiQcomDisplayUtilsProtocol *pDisplayUtilsProtocol = NULL;

  Status = gBS->LocateProtocol (&gQcomDisplayUtilsProtocolGuid,
                                NULL,
                                (VOID **)&pDisplayUtilsProtocol);
  if ((EFI_ERROR (Status)) ||
      (pDisplayUtilsProtocol == NULL)) {
    RTVariable = TRUE;
  } else if (pDisplayUtilsProtocol->Revision <  0x20000) {
    RTVariable = TRUE;
  } else {
    /* The display utils version for 0x20000 and above can support
       display protocol to get and set variable */
    Status = pDisplayUtilsProtocol->DisplayUtilsGetVariable (
          VariableName,
          (UINT8 *)VariableValue,
          DataSize,
          0);
  }

  if (RTVariable) {
    Status = gRT->GetVariable (VariableName,
                               &gQcomTokenSpaceGuid,
                               NULL,
                               DataSize,
                               (VOID *)VariableValue);
  }

  if (Status == EFI_NOT_FOUND) {
    // EFI_NOT_FOUND is not an error for retail case.
    Status = EFI_SUCCESS;
  } else if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_VERBOSE,
        "Display get variable failed with status(%d)!\n", Status));
  }

  return Status;
}

STATIC VOID
CmdOemSetHwFenceValue (CONST CHAR8 *arg, VOID *data, UINT32 Size)
{
  EFI_STATUS Status;
  CHAR8 Resp[MAX_RSP_SIZE] = "Set HW fence value: ";
  CHAR8 HwFenceValue[MAX_DISPLAY_PANEL_OVERRIDE] = " msm_hw_fence.enable=";
  INTN Pos = 0;

  for (Pos = 0; Pos < AsciiStrLen (arg); Pos++) {
    if (arg[Pos] == ' ') {
      arg++;
      Pos--;
    } else {
      break;
    }
  }

  AsciiStrnCatS (HwFenceValue,
                 MAX_DISPLAY_PANEL_OVERRIDE,
                 arg,
                 AsciiStrLen (arg));

  Status = gRT->SetVariable ((CHAR16 *)L"HwFenceConfiguration",
                               &gQcomTokenSpaceGuid,
                               EFI_VARIABLE_RUNTIME_ACCESS |
                               EFI_VARIABLE_BOOTSERVICE_ACCESS |
                               EFI_VARIABLE_NON_VOLATILE,
                               AsciiStrLen (HwFenceValue),
                               (VOID *)HwFenceValue);

  if (EFI_ERROR (Status)) {
    AsciiStrnCatS (Resp, sizeof (Resp), ": failed!", AsciiStrLen (": failed!"));
    FastbootFail (Resp);
  } else {
    AsciiStrnCatS (Resp, sizeof (Resp), ": done", AsciiStrLen (": done"));
    FastbootOkay (Resp);
  }
}

STATIC VOID
CmdOemSelectDisplayPanel (CONST CHAR8 *arg, VOID *data, UINT32 sz)
{
  EFI_STATUS Status;
  CHAR8 resp[MAX_RSP_SIZE] = "Selecting Panel: ";
  CHAR8 DisplayPanelStr[MAX_DISPLAY_PANEL_OVERRIDE] = "";
  CHAR8 DisplayPanelStrExist[MAX_DISPLAY_PANEL_OVERRIDE] = "";
  INTN Pos = 0;
  UINTN CurStrLen = 0;
  UINTN TotalStrLen = 0;
  BOOLEAN Append = FALSE;

  for (Pos = 0; Pos < AsciiStrLen (arg); Pos++) {
    if (arg[Pos] == ' ') {
      arg++;
      Pos--;
    } else if (arg[Pos] == ':') {
      Append = TRUE;
    } else {
      break;
    }
  }

  if (Append) {
    CurStrLen = sizeof (DisplayPanelStrExist) / sizeof (CHAR8);

    Status = DisplayGetVariable ((CHAR16 *)L"DisplayPanelOverride",
                                 (VOID *)DisplayPanelStrExist,
                                 &CurStrLen);
    TotalStrLen = CurStrLen + AsciiStrLen (arg);

    if ((EFI_SUCCESS == Status) &&
        (0 != CurStrLen) &&
        (TotalStrLen < MAX_DISPLAY_PANEL_OVERRIDE)) {
      AsciiStrnCatS (DisplayPanelStr,
                     MAX_DISPLAY_PANEL_OVERRIDE,
                     DisplayPanelStrExist,
                     CurStrLen);
      DEBUG ((EFI_D_INFO, "existing panel name (%a)\n", DisplayPanelStr));
    }
  }

  AsciiStrnCatS (DisplayPanelStr,
                 MAX_DISPLAY_PANEL_OVERRIDE,
                 arg,
                 AsciiStrLen (arg));

  /* Update the environment variable with the selected panel */
  Status = DisplaySetVariable ((CHAR16 *)L"DisplayPanelOverride",
                               (VOID *)DisplayPanelStr,
                               AsciiStrLen (DisplayPanelStr));

  if (EFI_ERROR (Status)) {
    AsciiStrnCatS (resp, sizeof (resp), ": failed", AsciiStrLen (": failed"));
    FastbootFail (resp);
  } else {
    AsciiStrnCatS (resp, sizeof (resp), ": done", AsciiStrLen (": done"));
    FastbootOkay (resp);
  }
}

#ifdef ENABLE_UPDATE_PARTITIONS_CMDS
STATIC VOID
CmdFlashingGetUnlockAbility (CONST CHAR8 *arg, VOID *data, UINT32 sz)
{
  CHAR8 UnlockAbilityInfo[MAX_RSP_SIZE];

  AsciiSPrint (UnlockAbilityInfo, sizeof (UnlockAbilityInfo),
               "get_unlock_ability: %d", IsAllowUnlock);
  FastbootInfo (UnlockAbilityInfo);
  WaitForTransferComplete ();
  FastbootOkay ("");
}
#endif

#ifdef ASUS_BUILD
// +++ ASUS_BSP : add for check_unbootable
int check_unbootable(int i, int j)
{
  EFI_PARTITION_ENTRY *PartEntry;
  UINT32 Unbootable_Value=0;
  EFI_STATUS Status;

  UpdatePartitionEntries();

  Status = gBS->HandleProtocol(Ptable[i].HandleInfoList[j].Handle, &gEfiPartitionRecordGuid, (VOID **)&PartEntry);
  if (EFI_SUCCESS != Status) {
    DEBUG ((EFI_D_INFO, "[ABL] check_unbootable: HandleProtocol gEfiPartitionRecordGuid failed, status = %r\n", Status));
  }
  Unbootable_Value= (PartEntry->Attributes & PART_ATT_UNBOOTABLE_VAL) >> PART_ATT_UNBOOTABLE_BIT;

  return Unbootable_Value;
}
// --- ASUS_BSP : add for check_unbootable
#endif

#if HIBERNATION_SUPPORT_NO_AES
STATIC VOID
CmdGoldenSnapshot (CONST CHAR8 *Arg, VOID *Data, UINT32 Size)
{
  EFI_STATUS Status;
  CHAR8 *Ptr = NULL;
  CONST CHAR8 *Delim = " ";

  if (Arg) {
    /* Expect a string "enable" or "disable" */
    if (((AsciiStrLen (Arg)) < 7)
        ||
        ((AsciiStrLen (Arg)) > 8) ) {
      FastbootFail ("Invalid input entered");
      return;
    }
    Ptr = AsciiStrStr (Arg, Delim);
    Ptr++;
  } else {
    FastbootFail ("Invalid input entered");
    return;
  }

  if (!AsciiStrCmp (Ptr, "enable")) {
    /* Set a magic value 200 to denote if it is golden image */
    Status = SetSnapshotGolden (200);
  }
  else if (!AsciiStrCmp (Ptr, "disable")) {
    Status = SetSnapshotGolden (0);
  }
  else {
    FastbootFail ("Invalid input entered");
    return;
  }

  if (Status != EFI_SUCCESS) {
    FastbootFail ("Failed to update golden-snapshot flag");
  }
  else {
    FastbootOkay ("Golden-snapshot flag updated");
  }
   return;
}
#endif

STATIC VOID
CmdOemDevinfo (CONST CHAR8 *arg, VOID *data, UINT32 sz)
{
  CHAR8 DeviceInfo[MAX_RSP_SIZE];
#ifdef ASUS_BUILD
  CHAR8 TempChar[MAX_RSP_SIZE];
  UINT32 TempNum;
//  VirtualAbMergeStatus SnapshotMergeStatus;
#endif

  AsciiSPrint (DeviceInfo, sizeof (DeviceInfo), "Verity mode: %a",
               IsEnforcing () ? "true" : "false");
  FastbootInfo (DeviceInfo);
  WaitForTransferComplete ();
  AsciiSPrint (DeviceInfo, sizeof (DeviceInfo), "Device unlocked: %a",
               IsUnlocked () ? "true" : "false");
  FastbootInfo (DeviceInfo);
  WaitForTransferComplete ();
  AsciiSPrint (DeviceInfo, sizeof (DeviceInfo), "Device critical unlocked: %a",
               IsUnlockCritical () ? "true" : "false");
  FastbootInfo (DeviceInfo);
  WaitForTransferComplete ();
  AsciiSPrint (DeviceInfo, sizeof (DeviceInfo), "Charger screen enabled: %a",
               IsChargingScreenEnable () ? "true" : "false");
  FastbootInfo (DeviceInfo);
  WaitForTransferComplete ();

#ifdef ASUS_BUILD
  // +++ ASUS_BSP : add for ASUS dongle unlock
  AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "Device authorized: %a", IsAuthorized()? "true" : "false");
  FastbootInfo(DeviceInfo);
  WaitForTransferComplete();

  AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "Device authorized2: %a", IsAuthorized_2()? "true" : "false");
  FastbootInfo(DeviceInfo);
  WaitForTransferComplete();
  // --- ASUS_BSP : add for ASUS dongle unlock

  // +++ ASUS_BSP : add for WaterMask unlock
  AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "WaterMask unlock: %a", IsAuthorized_3()? "Y" : "N");
  FastbootInfo(DeviceInfo);
  WaitForTransferComplete();
  // --- ASUS_BSP : add for WaterMask unlock

  // +++ ASUS_BSP : Add for xbl info
  GetXBLVersion(TempChar, sizeof(TempChar));
  AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "XBL: %a", TempChar);
  FastbootInfo(DeviceInfo);
  WaitForTransferComplete();
  // --- ASUS_BSP : Add for xbl info

  // +++ ASUS_BSP : Add for abl info
  GetBootloaderVersion(TempChar, sizeof(TempChar));
  AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "ABL: %a", TempChar);
  FastbootInfo(DeviceInfo);
  WaitForTransferComplete();
  // --- ASUS_BSP : Add for abl info

  // +++ ASUS_BSP : Add for ssn info (from asuskey4)
  GetSSNNum(TempChar, sizeof(TempChar));
  AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "SSN: %a", TempChar);
  FastbootInfo(DeviceInfo);
  WaitForTransferComplete();
  // --- ASUS_BSP : Add for ssn info

  // +++ ASUS_BSP : Add for isn info (from asuskey4)
  GetISNNum(TempChar, sizeof(TempChar));
  AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "ISN: %a", TempChar);
  FastbootInfo(DeviceInfo);
  WaitForTransferComplete();
  // --- ASUS_BSP : Add for isn info

  // +++ ASUS_BSP : Add for imei info (from asuskey4)
  GetIMEINum(TempChar, sizeof(TempChar));
  AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "IMEI: %a", TempChar);
  FastbootInfo(DeviceInfo);
  WaitForTransferComplete();
  // --- ASUS_BSP : Add for imei info

  // +++ ASUS_BSP : Add for imei2 info (from asuskey4)
  GetIMEI2Num(TempChar, sizeof(TempChar));
  AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "IMEI2: %a", TempChar);
  FastbootInfo(DeviceInfo);
  WaitForTransferComplete();
  // --- ASUS_BSP : Add for imei2 info

  // +++ ASUS_BSP : Add for cid info (from asuskey4)
  GetCIDName(TempChar, sizeof(TempChar));
  AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "CID: %a", TempChar);
  FastbootInfo(DeviceInfo);
  WaitForTransferComplete();
  // --- ASUS_BSP : Add for cid info

  // +++ ASUS_BSP : Add for read country code (from asuskey4)
  GetCountryCode(TempChar, sizeof(TempChar));
  AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "COUNTRY: %a", TempChar);
  FastbootInfo(DeviceInfo);
  WaitForTransferComplete();
  // --- ASUS_BSP : Add for read country code

  // +++ ASUS_BPS : add for force hwid
  GetProjName(TempChar, sizeof(TempChar));
  AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "Project: %a", TempChar);
  FastbootInfo(DeviceInfo);
  WaitForTransferComplete();

  AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "DT ID: %d", GetDeviceTreeID());
  FastbootInfo(DeviceInfo);
  WaitForTransferComplete();
  // --- ASUS_BPS : add for force hwid

  // +++ ASUS_BSP : Add for get Feature ID
  TempNum = Get_FEATURE_ID();

  if (TempNum == 0x6){
      AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "Feature ID: %x (%a)", TempNum, "001-AB-SM8550_ES1");
  }else if(TempNum == 0x0){
      AsciiSPrint(DeviceInfo, sizeof(DeviceInfo),"Feature ID: %x (%a)", TempNum, "002-AB-SM8550_ES2");
  }else if(TempNum == 0x8){
      AsciiSPrint(DeviceInfo, sizeof(DeviceInfo),"Feature ID: %x (%a)", TempNum, "002-AB-SM8550P_ES");
  }else{
      AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "Feature ID: %x (%a)", TempNum, "Unknown");
  }
  
  FastbootInfo(DeviceInfo);
  WaitForTransferComplete();
  // --- ASUS_BSP : Add for get Feature ID

  // +++ ASUS_BSP : Add for get JTAG ID
  TempNum = (Get_JTAG_ID() & 0xF000) >> 12;
  AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "JTAG ID: %x (0x%x)", TempNum, Get_JTAG_ID());

  FastbootInfo(DeviceInfo);
  WaitForTransferComplete();
  // --- ASUS_BSP : Add for get JTAG ID

  // +++ ASUS_BSP : add for fuse blow
  if(IsSecureBootEnabled())
  {
    AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "SB: Y");
    FastbootInfo(DeviceInfo);
    WaitForTransferComplete();
  }
  else
  {
    AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "SB: N");
    FastbootInfo(DeviceInfo);
    WaitForTransferComplete();
  }
  // --- ASUS_BSP : add for fuse blow

  // +++ ASUS_BSP : add for check fuse with no rpmb
  BOOLEAN SecureDeviceNoRpmb = FALSE;
  IsSecureDeviceNoCheckRpmb(&SecureDeviceNoRpmb);
  if(SecureDeviceNoRpmb)
  {
    AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "SBNR: Y");
    FastbootInfo(DeviceInfo);
    WaitForTransferComplete();
  }
  else
  {
    AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "SBNR: N");
    FastbootInfo(DeviceInfo);
    WaitForTransferComplete();
  }
  // --- ASUS_BSP : add for check fuse with no rpmb

  // +++ ASUS_BSP : add for ddr info
  /*
  AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "DDR Manufacturer ID: %x", Get_DDR_Manufacturer_ID());
  FastbootInfo(DeviceInfo);
  WaitForTransferComplete();

  AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "DDR Device Type: %x", Get_DDR_Device_Type());
  FastbootInfo(DeviceInfo);
  WaitForTransferComplete();
  */
  // --- ASUS_BSP : add for ddr info

  //+++ ASUS_BSP : add for dm-verity counter
  AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "TotalDMCounter: %d", GetTotalDmVerityCounter());
  FastbootInfo(DeviceInfo);
  WaitForTransferComplete();

  AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "DMCounter: %d", GetDmVerityCounter());
  FastbootInfo(DeviceInfo);
  WaitForTransferComplete();
  //+++ ASUS_BSP : add for dm-verity counter

  //+++ ASUS_BSP : add for unbootable_counter and retry_counter
  AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "SlotARetryCounter: %d", GetSlotARetryCounter());
  FastbootInfo(DeviceInfo);
  WaitForTransferComplete();

  AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "SlotAUnbootableCounter: %d", GetSlotAUnbootableCounter());
  FastbootInfo(DeviceInfo);
  WaitForTransferComplete();

  AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "SlotBRetryCounter: %d", GetSlotBRetryCounter());
  FastbootInfo(DeviceInfo);
  WaitForTransferComplete();

  AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "SlotBUnbootableCounter: %d", GetSlotBUnbootableCounter());
  FastbootInfo(DeviceInfo);
  WaitForTransferComplete();
  //--- ASUS_BSP : add for unbootable_counter and retry_counter

  // +++ ASUS_BSP : add for boot count
  AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "BOOT_COUNT: %d", GetBootCounter());
  FastbootInfo(DeviceInfo);
  WaitForTransferComplete();
  // --- ASUS_BSP : add for boot count

  // +++ ASUS_BSP : add for lock count
  AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "LOCK_COUNT: %d", GetLockCounter());
  FastbootInfo(DeviceInfo);
  WaitForTransferComplete();
  // --- ASUS_BSP : add for lock count

  // +++ ASUS_BSP : add for check apdp partition
  AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "Check_APDP: %d", GetAPDP());
  FastbootInfo(DeviceInfo);
  WaitForTransferComplete();
  // +++ ASUS_BSP : add for check apdp partition

  // +++ ASUS_BSP : check if have rawdump partiton or not
  AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "rawdump_en: %a", check_rawdump_partition()? "true" : "false");
  FastbootInfo(DeviceInfo);
  WaitForTransferComplete();
  // --- ASUS_BSP : check if have rawdump partiton or not

  // +++ ASUS_BSP : add for logcat-asdf sevices
  AsciiSPrint (DeviceInfo, sizeof (DeviceInfo), "force start logcat-asdf: %a", GetLogcatAsdfOn() ? "true" : "false");
  FastbootInfo(DeviceInfo);
  WaitForTransferComplete();
  // --- ASUS_BSP : add for logcat-asdf sevices

  // +++ ASUS_BSP : add for get current slot
  UnicodeStrToAsciiStr (GetCurrentSlotSuffix ().Suffix, TempChar);
  SKIP_FIRSTCHAR_IN_SLOT_SUFFIX (TempChar);
  AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "Current Slot: %a", TempChar);
  FastbootInfo(DeviceInfo);
  WaitForTransferComplete();
  // --- ASUS_BSP : add for get current slot

  // +++ ASUS_BSP : add for check if current slot is bootable
  if(!StrnCmp (GetCurrentSlotSuffix ().Suffix, L"_a", StrLen (L"_a"))){
    AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "Slot_a bootable: %a", check_unbootable(4, 11)? "Unbootable" : "Bootable");
    FastbootInfo(DeviceInfo);
    WaitForTransferComplete();
  }else{
    AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "Slot_b bootable: %a", check_unbootable(4, 36)? "Unbootable" : "Bootable");
    FastbootInfo(DeviceInfo);
    WaitForTransferComplete();
  }
  // --- ASUS_BSP : add for check if boot is bootable

  // +++ ASUS_BSP : add for AVB Verity
  AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "AVB Verity: %a", GetAvbVerity()? "Disable" : "Enable");
  FastbootInfo(DeviceInfo);
  WaitForTransferComplete();
  // --- ASUS_BSP : add for AVB Verity

  // +++ ASUS_BSP : add for verify_vbmeta_ret
  AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "Verify vbmeta ret: %d", GetVerifyVbmetaRet());
  FastbootInfo(DeviceInfo);
  WaitForTransferComplete();
  // --- ASUS_BSP : add for verify_vbmeta_ret

  AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "ABL_FTM: %a", is_ftm_mode()? "TRUE" : "FALSE");
  FastbootInfo(DeviceInfo);
  WaitForTransferComplete();
  
  // add for uart status +++
  AsciiSPrint (DeviceInfo, sizeof (DeviceInfo), "UART-ON Status: %a", GetUartStatus() ? "true" : "false");
  FastbootInfo(DeviceInfo);
  WaitForTransferComplete();
  // add for uart status ---
/*
  if (IsVirtualAbOtaSupported ()) {
    SnapshotMergeStatus = GetSnapshotMergeStatus ();

    switch (SnapshotMergeStatus) {
      case SNAPSHOTTED:
        SnapshotMergeStatus = SNAPSHOTTED;
        break;
      case MERGING:
        SnapshotMergeStatus = MERGING;
        break;
      default:
        SnapshotMergeStatus = NONE_MERGE_STATUS;
        break;
    }

    AsciiSPrint (SnapshotMergeState,
                  AsciiStrLen (VabSnapshotMergeStatus[SnapshotMergeStatus]) + 1,
                  "%a", VabSnapshotMergeStatus[SnapshotMergeStatus]);

    AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "SNAPSHOT-UPDATE-STATUS: %a", SnapshotMergeState);
    FastbootInfo(DeviceInfo);
    WaitForTransferComplete();
  }
*/  
  // add for debug unlock status +++
  AsciiSPrint (DeviceInfo, sizeof (DeviceInfo), "DEBUG UNLOCK Status: %a", IsDebugUnlocked() ? "true" : "false");
  FastbootInfo(DeviceInfo);
  WaitForTransferComplete();
  // add for debug unlock status ---
  memset(DeviceInfo, 0, sizeof(DeviceInfo));
#endif

  if (IsHibernationEnabled ()) {
    AsciiSPrint (DeviceInfo, sizeof (DeviceInfo), "Erase swap on restore: %a",
                 IsSnapshotGolden () ? "true" : "false");
    FastbootInfo (DeviceInfo);
    WaitForTransferComplete ();
  }

  FastbootOkay ("");
}

STATIC EFI_STATUS
AcceptCmdTimerInit (IN UINT64 Size, IN CHAR8 *Data)
{
  EFI_STATUS Status = EFI_SUCCESS;
  CmdInfo *AcceptCmdInfo = NULL;
  EFI_EVENT CmdEvent = NULL;

  AcceptCmdInfo = AllocateZeroPool (sizeof (CmdInfo));
  if (!AcceptCmdInfo)
    return EFI_OUT_OF_RESOURCES;

  AcceptCmdInfo->Size = Size;
  AcceptCmdInfo->Data = Data;

  Status = gBS->CreateEvent (EVT_TIMER | EVT_NOTIFY_SIGNAL, TPL_CALLBACK,
                             AcceptCmdHandler, AcceptCmdInfo, &CmdEvent);

  if (!EFI_ERROR (Status)) {
    Status = gBS->SetTimer (CmdEvent, TimerRelative, 100000);
  }

  if (EFI_ERROR (Status)) {
    FreePool (AcceptCmdInfo);
    AcceptCmdInfo = NULL;
  }

  return Status;
}

STATIC VOID
AcceptCmdHandler (IN EFI_EVENT Event, IN VOID *Context)
{
  CmdInfo *AcceptCmdInfo = Context;

  if (Event) {
    gBS->SetTimer (Event, TimerCancel, 0);
    gBS->CloseEvent (Event);
  }

  AcceptCmd (AcceptCmdInfo->Size, AcceptCmdInfo->Data);
  FreePool (AcceptCmdInfo);
  AcceptCmdInfo = NULL;
}

STATIC VOID
AcceptCmd (IN UINT64 Size, IN CHAR8 *Data)
{
  EFI_STATUS Status = EFI_SUCCESS;
  FASTBOOT_CMD *cmd;
  UINT32 BatteryVoltage = 0;
  STATIC BOOLEAN IsFirstEraseFlash;
  CHAR8 FlashResultStr[MAX_RSP_SIZE] = "\0";

  if (!Data) {
    FastbootFail ("Invalid input command");
    return;
  }
  if (Size > MAX_FASTBOOT_COMMAND_SIZE)
    Size = MAX_FASTBOOT_COMMAND_SIZE;
  Data[Size] = '\0';

  DEBUG ((EFI_D_INFO, "Handling Cmd: %a\n", Data));

  if (!IsDisableParallelDownloadFlash ()) {
    /* Wait for flash finished before next command */
    if (AsciiStrnCmp (Data, "download", AsciiStrLen ("download"))) {
      StopUsbTimer ();
      if (!IsFlashComplete &&
          !IsUseMThreadParallel ()) {
        Status = AcceptCmdTimerInit (Size, Data);
        if (Status == EFI_SUCCESS) {
          return;
        }
      }
    }

    /* Check last flash result */
    if (FlashResult != EFI_SUCCESS) {
      AsciiSPrint (FlashResultStr, MAX_RSP_SIZE, "%a : %r",
                 "Error: Last flash failed", FlashResult);

      DEBUG ((EFI_D_ERROR, "%a\n", FlashResultStr));
      if (!AsciiStrnCmp (Data, "flash", AsciiStrLen ("flash")) ||
          !AsciiStrnCmp (Data, "download", AsciiStrLen ("download"))) {
        FastbootFail (FlashResultStr);
        FlashResult = EFI_SUCCESS;
        return;
      }
    }
  }

  if (FixedPcdGetBool (EnableBatteryVoltageCheck)) {
    /* Check battery voltage before erase or flash image
     * It gets partition type once when to flash or erase image,
     * for sparse image, it calls flash command more than once, it's
     * no need to check the battery voltage at every time, it's risky
     * to stop the update when the image is half-flashed.
     */
    if (IsFirstEraseFlash) {
      if (!AsciiStrnCmp (Data, "erase", AsciiStrLen ("erase")) ||
          !AsciiStrnCmp (Data, "flash", AsciiStrLen ("flash"))) {
        if (!TargetBatterySocOk (&BatteryVoltage)) {
          DEBUG ((EFI_D_VERBOSE, "fastboot: battery voltage: %d\n",
                  BatteryVoltage));
          FastbootFail ("Warning: battery's capacity is very low\n");
          return;
        }
        IsFirstEraseFlash = FALSE;
      }
    } else if (!AsciiStrnCmp (Data, "getvar:partition-type",
                              AsciiStrLen ("getvar:partition-type"))) {
      IsFirstEraseFlash = TRUE;
    }
  }

  for (cmd = cmdlist; cmd; cmd = cmd->next) {
    if (AsciiStrnCmp (Data, cmd->prefix, cmd->prefix_len))
      continue;

    cmd->handle ((CONST CHAR8 *)Data + cmd->prefix_len, (VOID *)mUsbDataBuffer,
                 (UINT32)mBytesReceivedSoFar);
    return;
  }
  DEBUG ((EFI_D_ERROR, "\nFastboot Send Fail\n"));
  FastbootFail ("unknown command");
}

STATIC EFI_STATUS
ReadAllowUnlockValue (UINT32 *IsAllowUnlock)
{
  EFI_STATUS Status;
  EFI_BLOCK_IO_PROTOCOL *BlockIo = NULL;
  EFI_HANDLE *Handle = NULL;
  UINT8 *Buffer;

  Status = PartitionGetInfo ((CHAR16 *)L"frp", &BlockIo, &Handle);
  if (Status != EFI_SUCCESS)
    return Status;

  if (!BlockIo)
    return EFI_NOT_FOUND;

  Buffer = AllocateZeroPool (BlockIo->Media->BlockSize);
  if (!Buffer) {
    DEBUG ((EFI_D_ERROR, "Failed to allocate memory for unlock value \n"));
    return EFI_OUT_OF_RESOURCES;
  }

  Status = BlockIo->ReadBlocks (BlockIo, BlockIo->Media->MediaId,
                                BlockIo->Media->LastBlock,
                                BlockIo->Media->BlockSize, Buffer);
  if (Status != EFI_SUCCESS)
    goto Exit;

  /* IsAllowUnlock value stored at the LSB of last byte*/
  *IsAllowUnlock = Buffer[BlockIo->Media->BlockSize - 1] & 0x01;

Exit:
  FreePool (Buffer);
  return Status;
}

#ifdef ASUS_BUILD
BOOLEAN CheckAsusVerifiedState;
extern BOOLEAN AsusVbmetaVerified;
extern BOOLEAN AsusBootVerified;
extern BOOLEAN AsusDtboVerified;
extern BOOLEAN AsusVendorBootVerified;
extern BOOLEAN AsusInitBootVerified;
STATIC VOID CmdOemAsusVerifiedState(CONST CHAR8 *arg, VOID *data, UINT32 sz)
{
	EFI_STATUS Status = EFI_SUCCESS;
	CHAR8 DeviceInfo[MAX_RSP_SIZE];
	BOOLEAN AsusVerifiedState = FALSE;

	CheckAsusVerifiedState = TRUE;

	BootInfo Info = {0};
	Info.MultiSlotBoot = TRUE;
	Info.BootIntoRecovery = FALSE;
	Info.BootReasonAlarm = FALSE;
	Status = LoadImageAndAuth (&Info, FALSE, FALSE);
	if (Status != EFI_SUCCESS)
	{
		FastbootFail("Failed to LoadImageAndAuth.");
	}

	AsusVerifiedState = AsusVbmetaVerified && 
	                    AsusBootVerified && 
	                    AsusDtboVerified && 
	                    AsusVendorBootVerified &&
	                    AsusInitBootVerified;
	
	BOOLEAN DMVeritySate = (GetDmVerityCounter() == 0) ? TRUE: FALSE;
	BOOLEAN DMVerityMergeState = FALSE;
	if(GetSnapshotMergeStatus() == MERGING){
	    DMVerityMergeState = (GetSnapshotCheckCounter() == 0) ? TRUE: FALSE;
	}else{
	    DMVerityMergeState = TRUE;
	}
	
	AsusVerifiedState = AsusVerifiedState && 
	                    DMVeritySate && 
	                    DMVerityMergeState;

	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "=== Result ===");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "AsusVerifiedState = %a \n", AsusVerifiedState? "PASS" : "FAIL" );
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "=== Info ===");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "AsusVbmetaVerified = %a", AsusVbmetaVerified? "PASS" : "FAIL" );
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "AsusBootVerified = %a", AsusBootVerified? "PASS" : "FAIL" );
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "AsusDtboVerified = %a", AsusDtboVerified? "PASS" : "FAIL" );
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "AsusVendorBootVerified = %a", AsusVendorBootVerified? "PASS" : "FAIL" );
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();
	
	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "AsusInitBootVerified = %a", AsusInitBootVerified? "PASS" : "FAIL" );
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();
	
	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "DMVeritySate = %a", DMVeritySate? "PASS" : "FAIL" );
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();
	
	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "DMVerityMergeState = %a", DMVerityMergeState? "PASS" : "FAIL" );
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	CheckAsusVerifiedState = FALSE;
	FastbootOkay("");

	RebootDevice (FASTBOOT_MODE);
}

// +++ ASUS_BSP : add for verify_vbmeta_ret
STATIC VOID CmdOemGetVerifyVbmetaRet(CONST CHAR8 * arg, VOID * data, UINT32 sz)
{
	CHAR8 DeviceInfo[MAX_RSP_SIZE];
	DEBUG((EFI_D_ERROR, "[ABL] +++ CmdOemGetVerifyVbmetaRet()\n"));
	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "Verify Vbmeta Ret = %d \n", GetVerifyVbmetaRet());
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "=== Ret Info List ===");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();
	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " 0 : AVB_SLOT_VERIFY_RESULT_OK");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();
	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " 1 : AVB_SLOT_VERIFY_RESULT_ERROR_OOM");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();
	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " 2 : AVB_SLOT_VERIFY_RESULT_ERROR_IO");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();
	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " 3 : AVB_SLOT_VERIFY_RESULT_ERROR_VERIFICATION");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();
	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " 4 : AVB_SLOT_VERIFY_RESULT_ERROR_ROLLBACK_INDEX");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();
	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " 5 : AVB_SLOT_VERIFY_RESULT_ERROR_PUBLIC_KEY_REJECTED");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();
	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " 6 : AVB_SLOT_VERIFY_RESULT_ERROR_INVALID_METADATA");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();
	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " 7 : AVB_SLOT_VERIFY_RESULT_ERROR_UNSUPPORTED_VERSION");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();
	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " 8 : AVB_SLOT_VERIFY_RESULT_ERROR_INVALID_ARGUMENT");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	DEBUG((EFI_D_ERROR, "[ABL] --- CmdOemGetVerifyVbmetaRet()\n"));
	FastbootOkay("");
}
// --- ASUS_BSP : add for verify_vbmeta_ret

// +++ ASUS_BSP : add for enable flash raw in slot_b
STATIC VOID CmdOemSlotbEnable(CONST CHAR8 *arg, VOID *data, UINT32 sz)
{
	g_asus_slot_b_enable = TRUE;
	FastbootOkay("");
}
// --- ASUS_BSP : add for enable flash raw in slot_b

#ifdef ASUS_AI2205_BUILD
STATIC VOID CmdOemFEASDF(CONST CHAR8 *arg, VOID *data, UINT32 sz)
{
	CHAR8 DeviceInfo[MAX_RSP_SIZE];

	g_allow_flash_and_erase_asdf = TRUE;

	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "ASDF Flag = %a \n", g_allow_flash_and_erase_asdf? "PASS" : "FAIL" );
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	FastbootOkay("");
}

#define TOTAL_LOG_NUM 50

STATIC VOID CmdOemRecordInfo(CONST CHAR8 * arg, VOID * data, UINT32 sz)
{
	CHAR8 DeviceInfo[MAX_RSP_SIZE];
	sys_info sysinfo;
	int i = 0;
	DEBUG((EFI_D_ERROR, "[ABL] +++ CmdOemRecordInfo()\n"));
	if(read_sysinfo(&sysinfo) == EFI_SUCCESS)
	{
		AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " ===================================================");
		FastbootInfo(DeviceInfo);
		WaitForTransferComplete();
		AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " Asus Falling Record Info count: %d", sysinfo.falling_count);
		FastbootInfo(DeviceInfo);
		WaitForTransferComplete();
		AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " ---------------------------------------------------");
		FastbootInfo(DeviceInfo);
		WaitForTransferComplete();

		for (i = 0; i < sysinfo.falling_count && i < TOTAL_LOG_NUM; i++)
		{
			AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " Asus Falling Record Time: No %a", sysinfo.falling_time[i]);
			FastbootInfo(DeviceInfo);
			WaitForTransferComplete();
		}
		AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " ===================================================");
		FastbootInfo(DeviceInfo);
		WaitForTransferComplete();
		AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " Asus Hit Record Info count: %d", sysinfo.hit_count);
		FastbootInfo(DeviceInfo);
		WaitForTransferComplete();
		AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " ---------------------------------------------------");
		FastbootInfo(DeviceInfo);
		WaitForTransferComplete();

		for (i = 0; i < sysinfo.hit_count && i < TOTAL_LOG_NUM; i++)
		{
			AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " Asus Hit Record Time: No %a", sysinfo.hit_time[i]);
			FastbootInfo(DeviceInfo);
			WaitForTransferComplete();
		}
		AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " ===================================================");
		FastbootInfo(DeviceInfo);
		WaitForTransferComplete();
		AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " Asus Thump Record Info count: %d", sysinfo.thump_count);
		FastbootInfo(DeviceInfo);
		WaitForTransferComplete();
		AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " ---------------------------------------------------");
		FastbootInfo(DeviceInfo);
		WaitForTransferComplete();

		for (i = 0; i < sysinfo.thump_count && i < TOTAL_LOG_NUM; i++)
		{
			AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " Asus Thump Record Time: No %a", sysinfo.thump_time[i]);
			FastbootInfo(DeviceInfo);
			WaitForTransferComplete();
		}
		AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " ===================================================");
		FastbootInfo(DeviceInfo);
		WaitForTransferComplete();
		FastbootOkay("");
	}
	else
	{
		FastbootFail("Failed to read sysinfo");
	}

	DEBUG((EFI_D_ERROR, "[ABL] --- CmdOemRecordInfo()\n"));
}
#endif
// +++ ASUS_BSP : add for restore factory data
STATIC VOID CmdOemRestoreFac(CONST CHAR8 *arg, VOID *data, UINT32 sz)
{
#ifdef ASUS_AI2205_BUILD

	CHAR8 DeviceInfo[MAX_RSP_SIZE];
	unsigned long calculatedCRC_factory = 0xFFFFFFFF;
	unsigned long calculatedCRC_persist = 0xFFFFFFFF;
	unsigned long rpmbCRC_factory = 0xFFFFFFFF;

	// +++ ASUS_BSP : add for fastboot permission
	if(!is_ftm_mode())
	{
		if (!IsAuthorized() && !IsAuthorized_2())
		{
			FastbootFail("Permission denied");
			return;
		}
	}
	// --- ASUS_BSP : add for fastboot permission

	DEBUG((EFI_D_ERROR, "[ABL] +++ CmdOemRestoreFac()\n"));

	calculatedCRC_factory = AsusCalculatePtCrc32(L"factory");
	DEBUG((EFI_D_INFO, "[ABL] Calculated factory CRC = 0x%x\n", calculatedCRC_factory));

	rpmbCRC_factory = GetFactoryCRC();
	DEBUG((EFI_D_INFO, "[ABL] RPMB factory CRC = 0x%x\n", rpmbCRC_factory));

	if ((calculatedCRC_factory != rpmbCRC_factory) && !is_ftm_mode())
	{
		AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "Restore failed, CRC check error!");
		FastbootInfo(DeviceInfo);
		WaitForTransferComplete();
		AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "Calculated factory CRC = 0x%x", calculatedCRC_factory);
		FastbootInfo(DeviceInfo);
		WaitForTransferComplete();
		AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "RPMB factory CRC = 0x%x", rpmbCRC_factory);
		FastbootInfo(DeviceInfo);
		WaitForTransferComplete();

		FastbootFail("Restore factory to persist failed!");
		return;
	}

	restore_factory_to_persist();

	calculatedCRC_persist = AsusCalculatePtCrc32(L"persist");
	DEBUG((EFI_D_INFO, "[ABL] Calculated persist CRC = 0x%x\n", calculatedCRC_persist));

	if (calculatedCRC_persist != calculatedCRC_factory)
	{
		AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "Restore failed, CRC mismatch!");
		FastbootInfo(DeviceInfo);
		WaitForTransferComplete();
		AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "Calculated persist CRC = 0x%x", calculatedCRC_persist);
		FastbootInfo(DeviceInfo);
		WaitForTransferComplete();
		AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "Calculated factory CRC = 0x%x", calculatedCRC_factory);
		FastbootInfo(DeviceInfo);
		WaitForTransferComplete();

		FastbootFail("Restore factory to persist failed!");
		return;
	}

	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "Restore factory to persist successfully!");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	FastbootOkay("");

	DEBUG((EFI_D_ERROR, "[ABL] --- CmdOemRestoreFac()\n"));
#else
	FastbootOkay("");
#endif
}
// --- ASUS_BSP : add for restore factory data

// +++ ASUS_BSP : add for backup factory data
STATIC VOID CmdOemBackupFac(CONST CHAR8 *arg, VOID *data, UINT32 sz)
{
#ifdef ASUS_AI2205_BUILD
	CHAR8 DeviceInfo[MAX_RSP_SIZE];
	unsigned long calculatedCRC_factory = 0xFFFFFFFF;
	unsigned long calculatedCRC_persist = 0xFFFFFFFF;
	unsigned long rpmbCRC_factory = 0xFFFFFFFF;

	// +++ ASUS_BSP : add for fastboot permission
	if(!is_ftm_mode())
	{
		if (!IsAuthorized() && !IsAuthorized_2())
		{
			FastbootFail("Permission denied");
			return;
		}
	}
	// --- ASUS_BSP : add for fastboot permission

	DEBUG((EFI_D_ERROR, "[ABL] +++ CmdOemBackupFac()\n"));

	backup_persist_to_factory();

	calculatedCRC_persist = AsusCalculatePtCrc32(L"persist");
	DEBUG((EFI_D_INFO, "[ABL] Calculated persist CRC = 0x%x\n", calculatedCRC_persist));

	calculatedCRC_factory = AsusCalculatePtCrc32(L"factory");
	DEBUG((EFI_D_INFO, "[ABL] Calculated factory CRC = 0x%x\n", calculatedCRC_factory));

	if (calculatedCRC_factory != calculatedCRC_persist) {
		AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "Backup failed, CRC mismatch!");
		FastbootInfo(DeviceInfo);
		WaitForTransferComplete();
		AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "Calculated persist CRC = 0x%x", calculatedCRC_persist);
		FastbootInfo(DeviceInfo);
		WaitForTransferComplete();
		AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "Calculated factory CRC = 0x%x", calculatedCRC_factory);
		FastbootInfo(DeviceInfo);
		WaitForTransferComplete();

		FastbootFail("Backup persist to factory failed");
		return;
	}else {
		AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "Calculated factory CRC = 0x%x", calculatedCRC_factory);
		FastbootInfo(DeviceInfo);
		WaitForTransferComplete();

		SetFactoryCRC(calculatedCRC_factory);
		rpmbCRC_factory = GetFactoryCRC();

		if ((rpmbCRC_factory != calculatedCRC_factory))
		{
			AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "Backup failed, CRC check error!");
			FastbootInfo(DeviceInfo);
			WaitForTransferComplete();
			AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "Calculated factory CRC = 0x%x", calculatedCRC_factory);
			FastbootInfo(DeviceInfo);
			WaitForTransferComplete();
			AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "RPMB factory CRC = 0x%x", rpmbCRC_factory);
			FastbootInfo(DeviceInfo);
			WaitForTransferComplete();

			FastbootFail("Backup persist to factory failed!");
			return;
		}

		AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "Backup persist to factory successfully!");
		FastbootInfo(DeviceInfo);
		WaitForTransferComplete();

		FastbootOkay("");
	}

	DEBUG((EFI_D_ERROR, "[ABL] --- CmdOemBackupFac()\n"));
#else
	FastbootOkay("");
#endif
}
// --- ASUS_BSP : add for backup factory data

// +++ ASUS_BSP : add for update cmdline
STATIC VOID CmdOemUpdateCmdline(CONST CHAR8 *arg, VOID *data, UINT32 sz)
{
	CHAR8  DeviceInfo[MAX_RSP_SIZE];
	CHAR16 Cmdline[MAX_RSP_SIZE];

	AsciiStrToUnicodeStr(arg, Cmdline);

	DEBUG((EFI_D_INFO, "[ABL] +++ CmdOemUpdateCmdline()\n"));

	if (!StrnCmp(Cmdline, L"androidboot.id.prj=", StrLen(L"androidboot.id.prj="))){
		AsciiSPrint(cmd_prj_id, sizeof(cmd_prj_id), " %s", Cmdline);
	}else if (!StrnCmp(Cmdline, L"androidboot.id.stage=", StrLen(L"androidboot.id.stage="))){
		AsciiSPrint(cmd_stage_id, sizeof(cmd_stage_id), " %s", Cmdline);
	}else if (!StrnCmp(Cmdline, L"androidboot.id.sku=", StrLen(L"androidboot.id.sku="))){
		AsciiSPrint(cmd_sku_id, sizeof(cmd_sku_id), " %s", Cmdline);
	}else if (!StrnCmp(Cmdline, L"androidboot.id.ddr=", StrLen(L"androidboot.id.ddr="))){
		AsciiSPrint(cmd_ddr_id, sizeof(cmd_ddr_id), " %s", Cmdline);
	}else if (!StrnCmp(Cmdline, L"androidboot.id.nfc=", StrLen(L"androidboot.id.nfc="))){
		AsciiSPrint(cmd_nfc_id, sizeof(cmd_nfc_id), " %s", Cmdline);
	}else if (!StrnCmp(Cmdline, L"androidboot.id.rf=", StrLen(L"androidboot.id.rf="))){
		AsciiSPrint(cmd_rf_id, sizeof(cmd_rf_id), " %s", Cmdline);
	}else if (!StrnCmp(Cmdline, L"androidboot.id.fp=", StrLen(L"androidboot.id.fp="))){
		AsciiSPrint(cmd_fp_id, sizeof(cmd_fp_id), " %s", Cmdline);
	}else if (!StrnCmp(Cmdline, L"androidboot.id.lgfcon=", StrLen(L"androidboot.id.lgfcon="))){
		AsciiSPrint(cmd_lgf_con_id, sizeof(cmd_lgf_con_id), " %s", Cmdline);
	}else if (!StrnCmp(Cmdline, L"androidboot.id.fc=", StrLen(L"androidboot.id.fc="))){
		AsciiSPrint(cmd_fc_id, sizeof(cmd_fc_id), " %s", Cmdline);
	}else if (!StrnCmp(Cmdline, L"androidboot.id.upper=", StrLen(L"androidboot.id.upper="))){
		AsciiSPrint(cmd_upper_id, sizeof(cmd_upper_id), " %s", Cmdline);
	}else if (!StrnCmp(Cmdline, L"androidboot.id.sub=", StrLen(L"androidboot.id.sub="))){
		AsciiSPrint(cmd_sub_id, sizeof(cmd_sub_id), " %s", Cmdline);
	}else if (!StrnCmp(Cmdline, L"androidboot.country_code=", StrLen(L"androidboot.country_code="))){
		AsciiSPrint(cmd_country_code, sizeof(cmd_country_code), " %s", Cmdline);
	}else{
		AsciiSPrint(cmd_update_cmdline, sizeof(cmd_update_cmdline), " %s", Cmdline);
	}

	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "Update cmdline : %a\n", (char*)arg);
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "Use following cmd to continue boot:");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "fastboot set_active a");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "fastboot continue \n");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	DEBUG((EFI_D_INFO, "[ABL] --- CmdOemUpdateCmdline()\n"));
	FastbootOkay("");
}
// --- ASUS_BSP : add for update cmdline

// +++ ASUS_BSP : add for reset vbmeta_system rollback value
STATIC VOID CmdOemResetRollback(CONST CHAR8 *arg, VOID *data, UINT32 sz)
{
#if 0
	// +++ ASUS_BSP : add for fastboot permission
	if(!ASUS_FASTBOOT_PERMISSION)
	{
		FastbootFail("permission denied");
		return;
	}
	// --- ASUS_BSP : add for fastboot permission
#endif

	DEBUG((EFI_D_ERROR, "[ABL] +++ CmdOemResetRollback()\n"));
	WriteRollbackIndex (0x2, 0x0);
	DEBUG((EFI_D_ERROR, "[ABL] --- CmdOemResetRollback()\n"));

	FastbootOkay("");
}
// --- ASUS_BSP : add for reset vbmeta_system rollback value

// +++ ASUS_BSP : add for read vbmeta_system rollback value
STATIC VOID CmdOemReadRollback(CONST CHAR8 *arg, VOID *data, UINT32 sz)
{
#if 0
		// +++ ASUS_BSP : add for fastboot permission
		if(!ASUS_FASTBOOT_PERMISSION)
		{
			FastbootFail("permission denied");
			return;
		}
		// --- ASUS_BSP : add for fastboot permission
#endif

	CHAR8 DeviceInfo[MAX_RSP_SIZE];
	UINT64 RollbackIndex = 0x0;

	DEBUG((EFI_D_ERROR, "[ABL] +++ CmdOemReadRollback()\n"));
	ReadRollbackIndex (0x2, &RollbackIndex);

	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "Vbmeta System rollback = %x \n", RollbackIndex);
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();
	DEBUG((EFI_D_ERROR, "[ABL] --- CmdOemReadRollback()\n"));

	FastbootOkay("");
}
// --- ASUS_BSP : add for read vbmeta_system rollback value

// +++ ASUS_BSP : add for read vbmeta magic
STATIC VOID CmdOemReadVbmeta(CONST CHAR8 *arg, VOID *data, UINT32 sz)
{
	CHAR8 DeviceInfo[MAX_RSP_SIZE];
	DEBUG((EFI_D_ERROR, "[ABL] +++ CmdOemReadVbmeta()\n"));

	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "Vbmeta Magic = %a \n", read_vbmeta_magic());
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	DEBUG((EFI_D_ERROR, "[ABL] --- CmdOemReadVbmeta()\n"));

	FastbootOkay("");
}
// --- ASUS_BSP : add for read vbmeta magic

// +++ ASUS_BSP : add for enable vbmeta magic
STATIC VOID CmdOemEnableVbmeta(CONST CHAR8 *arg, VOID *data, UINT32 sz)
{
	// +++ ASUS_BSP : add for fastboot permission
	if(!ASUS_FASTBOOT_PERMISSION)
	{
		FastbootFail("permission denied");
		return;
	}
	// --- ASUS_BSP : add for fastboot permission

	DEBUG((EFI_D_ERROR, "[ABL] +++ CmdOemEnableVbmeta()\n"));
	enable_vbmeta_magic();
	DEBUG((EFI_D_ERROR, "[ABL] --- CmdOemEnableVbmeta()\n"));

	FastbootOkay("");
}
// --- ASUS_BSP : add for enable vbmeta magic

// +++ ASUS_BSP : add for enable verity
STATIC VOID CmdOemEnableVerity(CONST CHAR8 *arg, VOID *data, UINT32 sz)
{
	// +++ ASUS_BSP : add for fastboot permission
	if(!ASUS_FASTBOOT_PERMISSION)
	{
		FastbootFail("permission denied");
		return;
	}
	// --- ASUS_BSP : add for fastboot permission

	DEBUG((EFI_D_ERROR, "[ABL] +++ CmdOemEnableVerity()\n"));
	enable_verity();
	DEBUG((EFI_D_ERROR, "[ABL] --- CmdOemEnableVerity()\n"));

	FastbootOkay("");
}
// --- ASUS_BSP : add for enable verity

// +++ ASUS_BSP : add for disable verity
STATIC VOID CmdOemDisableVerity(CONST CHAR8 *arg, VOID *data, UINT32 sz)
{
	// +++ ASUS_BSP : add for fastboot permission
	if(!ASUS_FASTBOOT_PERMISSION)
	{
		FastbootFail("permission denied");
		return;
	}
	// --- ASUS_BSP : add for fastboot permission

	DEBUG((EFI_D_ERROR, "[ABL] +++ CmdOemDisableVerity()\n"));
	disable_verity();
	DEBUG((EFI_D_ERROR, "[ABL] --- CmdOemDisableVerity()\n"));

	FastbootOkay("");
}
// --- ASUS_BSP : add for disable verity

// +++ ASUS_BPS : add for reset WaterMask unlock
STATIC VOID CmdOemResetAuth3 (CONST CHAR8 *arg, VOID *data, UINT32 sz)
{
	EFI_STATUS Status = EFI_SUCCESS ;

	// +++ ASUS_BSP : add for fastboot permission
	if(!IsAuthorized_3())
	{
		FastbootFail("The watermark is already enabled.");
		return;
	}
	// --- ASUS_BSP : add for fastboot permission

	DEBUG((EFI_D_ERROR, "[ABL] +++ CmdOemResetAuth3()\n"));

	Status = SetAuthorized3Value(FALSE);
	DEBUG((EFI_D_ERROR, "[ABL] SetAuthorized3Value(FALSE) : %d\n", IsAuthorized_3()));

	if (Status != EFI_SUCCESS) {
		DEBUG((EFI_D_ERROR, "[ABL] CmdOemResetDevInfo : Failed to SetAuthorized3Value, Status\n", Status));
		FastbootFail ("Failed to SetAuthorized3Value");
	}

	DEBUG((EFI_D_ERROR, "[ABL] --- CmdOemResetAuth3()\n"));

	FastbootOkay("");
}
// --- ASUS_BPS : add for reset WaterMask unlock

// +++ ASUS_BPS : add for reset dev info
STATIC VOID CmdOemResetAuth2 (CONST CHAR8 *arg, VOID *data, UINT32 sz)
{
	EFI_STATUS Status = EFI_SUCCESS ;

	// +++ ASUS_BSP : add for fastboot permission
	if(!ASUS_FASTBOOT_PERMISSION)
	{
		FastbootFail("permission denied");
		return;
	}
	// --- ASUS_BSP : add for fastboot permission

	DEBUG((EFI_D_ERROR, "[ABL] +++ CmdOemResetAuth2()\n"));

	Status = SetAuthorized2Value(FALSE);
	DEBUG((EFI_D_ERROR, "[ABL] SetAuthorized2Value(FALSE) : %d\n", IsAuthorized_2()));

	if (Status != EFI_SUCCESS) {
		DEBUG((EFI_D_ERROR, "[ABL] CmdOemResetDevInfo : Failed to SetAuthorized2Value, Status\n", Status));
		FastbootFail ("Failed to SetAuthorized2Value");
	}

	DEBUG((EFI_D_ERROR, "[ABL] --- CmdOemResetAuth2()\n"));

	FastbootOkay("");
}

STATIC VOID CmdOemResetDevInfo(CONST CHAR8 *arg, VOID *data, UINT32 sz)
{
	EFI_STATUS Status = EFI_SUCCESS;
	UINT32 ResetDMCounter = 0;//+++ ASUS_BSP : add for dm-verity counter

	DEBUG((EFI_D_ERROR, "[ABL] +++ CmdOemResetDevInfo()\n"));

#if 0 // for ATD request
	// +++ ASUS_BSP : add for fastboot permission
	if(!ASUS_FASTBOOT_PERMISSION)
	{
		FastbootFail("permission denied");
		return;
	}
	// --- ASUS_BSP : add for fastboot permission
#endif

	Status = EnableEnforcingMode(TRUE);// TRUE = enforcing, FALSE = logging
	DEBUG((EFI_D_ERROR, "[ABL]  EnableEnforcingMode(TRUE) : %d\n", IsEnforcing()));

	if (Status != EFI_SUCCESS) {
		DEBUG((EFI_D_ERROR, "[ABL] CmdOemResetDevInfo : Failed to EnableEnforcingMode, Status\n", Status));
		FastbootFail ("Failed to EnableEnforcingMode");
	}

	// +++ ASUS_BSP : lock ASUS dongle unlock flag
	Status = SetAuthorizedValue(FALSE);
	DEBUG((EFI_D_ERROR, "[ABL] SetAuthorizedValue(FALSE) : %d\n", IsAuthorized()));

	if (Status != EFI_SUCCESS) {
		DEBUG((EFI_D_ERROR, "[ABL] CmdOemResetDevInfo : Failed to SetAuthorizedValue, Status\n", Status));
		FastbootFail ("Failed to SetAuthorizedValue");
	}

	Status = SetAuthorized2Value(FALSE);
	DEBUG((EFI_D_ERROR, "[ABL] SetAuthorized2Value(FALSE) : %d\n", IsAuthorized_2()));

	if (Status != EFI_SUCCESS) {
		DEBUG((EFI_D_ERROR, "[ABL] CmdOemResetDevInfo : Failed to SetAuthorized2Value, Status\n", Status));
		FastbootFail ("Failed to SetAuthorized2Value");
	}

#ifdef ASUS_AI2205_BUILD
	// --- ASUS_BSP : lock ASUS dongle unlock flag
	Status = SetDeviceDebugUnlockValue(FALSE);
	DEBUG((EFI_D_ERROR, "[ABL] SetDeviceDebugUnlockValue(FALSE) : %d\n", IsDebugUnlocked()));

	if (Status != EFI_SUCCESS) {
		DEBUG((EFI_D_ERROR, "[ABL] CmdOemResetDevInfo : Failed to SetDeviceDebugUnlockValue, Status\n", Status));
		FastbootFail ("Failed to SetDeviceDebugUnlockValue");
	}
	// +++ ASUS_BSP : lock ASUS debug unlock flag
#endif
	
	// --- ASUS_BSP : lock ASUS debug unlock flag

	//+++ ASUS_BSP : add for dm-verity counter
	SetTotalDmVerityCounter(0);
	GetTotalDmVerityCounter(&ResetDMCounter);
	DEBUG((EFI_D_ERROR, "[ABL] SetTotalDmVerityCounter(0) : %d\n", ResetDMCounter));
	SetDmVerityCounter(0);
	GetDmVerityCounter(&ResetDMCounter);
	DEBUG((EFI_D_ERROR, "[ABL] SetDmVerityCounter(0) : %d\n", ResetDMCounter));
	//--- ASUS_BSP : add for dm-verity counter
	
	Status = SetUartStatus(FALSE);
	if (Status != EFI_SUCCESS) {
		DEBUG((EFI_D_ERROR, "[ABL] CmdOemResetDevInfo : Failed to SetUartStatus, Status\n", Status));
		FastbootFail ("Failed to SetUartStatus");
	}

	DEBUG((EFI_D_ERROR, "[ABL]  --- CmdOemResetDevInfo()\n"));

	FastbootOkay("");
}
// --- ASUS_BPS : add for reset dev info

// +++ ASUS_BSP : add for force hwid
STATIC VOID CmdOemForceHwId(CONST CHAR8 *arg, VOID *data, UINT32 sz)
{
	CHAR8  DeviceInfo[MAX_RSP_SIZE];
	CHAR16 HwStage[MAX_RSP_SIZE];
	UINT32 Force_DTID = 0;

	AsciiStrToUnicodeStr(arg, HwStage);

	// +++ ASUS_BSP : add for fastboot permission
	if(!ASUS_FASTBOOT_PERMISSION && !is_ftm_mode())
	{
		FastbootFail("permission denied");
		return;
	}
	// --- ASUS_BSP : add for fastboot permission

	DEBUG((EFI_D_INFO, "[ABL] +++ CmdOemForceHwId()\n"));

	Force_DTID = AsciiStrHexToUint64(arg);

	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " force hwid : %a (DT_ID=%d)\n", (char*)arg, Force_DTID);
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "Use following cmd to continue boot:");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "fastboot set_active a");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "fastboot continue \n");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	SetDeviceTreeID(Force_DTID);
	if (GetDeviceTreeID() != Force_DTID)
	{
		DEBUG((EFI_D_ERROR, "[ABL] Unable set the device_tree_id : %d\n", Force_DTID));
		FastbootFail("");
		return;
	}

	DEBUG((EFI_D_INFO, "[ABL] --- CmdOemForceHwId()\n"));
	FastbootOkay("");
}
// --- ASUS_BSP : add for force hwid

// add for uart control +++
STATIC VOID CmdOemUartOn(CONST CHAR8 * arg, VOID * data, UINT32 sz)
{
    DEBUG((EFI_D_ERROR, "[ABL] +++ CmdOemUartOn()\n"));
    SetUartStatus(TRUE);
    FastbootOkay("");
}

STATIC VOID CmdOemUartOff(CONST CHAR8 * arg, VOID * data, UINT32 sz)
{
    DEBUG((EFI_D_ERROR, "[ABL] +++ CmdOemUartOff()\n"));
    SetUartStatus(FALSE);
    FastbootOkay("");
}
// add for uart control ---

// +++ ASUS_BSP : add for unbootable_counter and retry_counter
STATIC VOID CmdOemResetSlotBUnbootableCount(CONST CHAR8 *arg, VOID *data, UINT32 sz)
{
	EFI_STATUS Status = EFI_SUCCESS;

	// +++ ASUS_BSP : add for fastboot permission
	if(!ASUS_FASTBOOT_PERMISSION)
	{
		FastbootFail("permission denied");
		return;
	}
	// --- ASUS_BSP : add for fastboot permission

	Status = SetSlotBUnbootableCounter(0);

	if (Status != EFI_SUCCESS)
	{
		FastbootFail("Failed to reset slot_b inbootable counter");
	}
	else
	{
		FastbootOkay("");
	}
}

STATIC VOID CmdOemResetSlotBRetryCount(CONST CHAR8 *arg, VOID *data, UINT32 sz)
{
	EFI_STATUS Status = EFI_SUCCESS;

	// +++ ASUS_BSP : add for fastboot permission
	if(!ASUS_FASTBOOT_PERMISSION)
	{
		FastbootFail("permission denied");
		return;
	}
	// --- ASUS_BSP : add for fastboot permission

	Status = SetSlotBRetryCounter(0);

	if (Status != EFI_SUCCESS)
	{
		FastbootFail("Failed to reset slot_b retry counter");
	}
	else
	{
		FastbootOkay("");
	}
}

STATIC VOID CmdOemResetSlotAUnbootableCount(CONST CHAR8 *arg, VOID *data, UINT32 sz)
{
	EFI_STATUS Status = EFI_SUCCESS;

	// +++ ASUS_BSP : add for fastboot permission
	if(!ASUS_FASTBOOT_PERMISSION)
	{
		FastbootFail("permission denied");
		return;
	}
	// --- ASUS_BSP : add for fastboot permission

	Status = SetSlotAUnbootableCounter(0);

	if (Status != EFI_SUCCESS)
	{
		FastbootFail("Failed to reset slot_a unbootable counter");
	}
	else
	{
		FastbootOkay("");
	}
}

STATIC VOID CmdOemResetSlotARetryCount(CONST CHAR8 *arg, VOID *data, UINT32 sz)
{
	EFI_STATUS Status = EFI_SUCCESS;

	// +++ ASUS_BSP : add for fastboot permission
	if(!ASUS_FASTBOOT_PERMISSION)
	{
		FastbootFail("permission denied");
		return;
	}
	// --- ASUS_BSP : add for fastboot permission

	Status = SetSlotARetryCounter(0);

	if (Status != EFI_SUCCESS)
	{
		FastbootFail("Failed to reset slot_a retry counter");
	}
	else
	{
		FastbootOkay("");
	}
}
// --- ASUS_BSP : add for unbootable_counter and retry_counter

// +++ ASUS_BSP : add for lock count
STATIC VOID CmdOemResetLockCount(CONST CHAR8 *arg, VOID *data, UINT32 sz)
{
	EFI_STATUS Status = EFI_SUCCESS;

	// +++ ASUS_BSP : add for fastboot permission
	if(!ASUS_FASTBOOT_PERMISSION)
	{
		FastbootFail("permission denied");
		return;
	}
	// --- ASUS_BSP : add for fastboot permission

	Status = SetLockCounter(0);

	if (Status != EFI_SUCCESS)
	{
		FastbootFail("Failed to reset lock count");
	}
	else
	{
		FastbootOkay("");
	}
}
// --- ASUS_BSP : add for lock count

// +++ ASUS_BSP : add for boot count
STATIC VOID CmdOemResetBootCount(CONST CHAR8 *arg, VOID *data, UINT32 sz)
{
	EFI_STATUS Status = EFI_SUCCESS;

	// +++ ASUS_BSP : add for fastboot permission
	if(!ASUS_FASTBOOT_PERMISSION)
	{
		FastbootFail("permission denied");
		return;
	}
	// --- ASUS_BSP : add for fastboot permission

	Status = SetBootCounter(0);

	if (Status != EFI_SUCCESS)
	{
		FastbootFail("Failed to reset boot count");
	}
	else
	{
		FastbootOkay("");
	}
}
// --- ASUS_BSP : add for boot count

// +++ ASUS_BSP : add for write pmic reg value
STATIC VOID CmdOemWritePmicReg(CONST CHAR8 *arg, VOID *data, UINT32 sz){
	CHAR8 DeviceInfo[MAX_RSP_SIZE];
	CHAR16 RegAddress[MAX_GPT_NAME_SIZE];
	UINT64 RegValue;
	UINT64 PmicIndex;
	UINT64 WriteValue;
	EFI_STATUS Status = EFI_SUCCESS;
	AsciiStrToUnicodeStr((CHAR8 *)arg, RegAddress);

	PmicIndex = AsciiStrHexToUint64(arg) >>24;
	RegValue = (AsciiStrHexToUint64(arg) >>8 ) & 0xFFFF;
	WriteValue = AsciiStrHexToUint64(arg) & 0xFF;

	// +++ ASUS_BSP : add for fastboot permission
	if(!ASUS_FASTBOOT_PERMISSION)
	{
		FastbootFail("permission denied");
		return;
	}
	// --- ASUS_BSP : add for fastboot permission

	DEBUG((EFI_D_ERROR, "[ABL] +++ CmdOemWritePmicReg : RegAddress = %s, PmicIndex = %x, RegValue = %x, WriteValue = %x\n", RegAddress, PmicIndex, RegValue, WriteValue));
	Status = WritePmicReg((UINT32)PmicIndex, (UINT32)RegValue, (UINT32)WriteValue);

	if (Status != EFI_SUCCESS) {
	DEBUG((EFI_D_ERROR, "[ABL] CmdOemWritePmicReg : Failed to write pmic reg value\n"));
		FastbootFail ("Failed to write pmic reg value");
	} else {
	DEBUG((EFI_D_ERROR, "[ABL] CmdOemWritePmicReg : WritePmicReg EFI_SUCCESS\n"));
		AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "PmicIndex = %x , RegValue = %x , WriteValue = %x", PmicIndex, RegValue, WriteValue);
		FastbootInfo(DeviceInfo);
		WaitForTransferComplete();
		FastbootOkay ("");
	}

	DEBUG((EFI_D_ERROR, "[ABL]  --- CmdOemWritePmicReg()\n\n"));
}
// --- ASUS_BSP : add for write pmic reg value

// +++ ASUS_BSP : add for get pmic reg value
STATIC VOID CmdOemGetPmicReg(CONST CHAR8 *arg, VOID *data, UINT32 sz){
	CHAR8 DeviceInfo[MAX_RSP_SIZE];
	CHAR16 RegAddress[MAX_GPT_NAME_SIZE];
	UINT64 RegValue;
	UINT64 PmicIndex;
	EFI_STATUS Status = EFI_SUCCESS;
	UINT8 value = 0;
	AsciiStrToUnicodeStr((CHAR8 *)arg, RegAddress);

	RegValue = AsciiStrHexToUint64(arg) & 0xFFFF;
	PmicIndex = AsciiStrHexToUint64(arg) >> 16;

	// +++ ASUS_BSP : add for fastboot permission
	if(!ASUS_FASTBOOT_PERMISSION)
	{
		FastbootFail("permission denied");
		return;
	}
	// --- ASUS_BSP : add for fastboot permission

	DEBUG((EFI_D_ERROR, "[ABL] +++ CmdOemGetPmicReg : RegAddress = %s, RegValue = %x, PmicIndex = %x\n", RegAddress, RegValue, PmicIndex));

	Status = GetPmicReg((UINT32)PmicIndex, (UINT32)RegValue, &value);

	if (Status != EFI_SUCCESS) {
		DEBUG((EFI_D_ERROR, "[ABL] CmdOemGetPmicReg : Failed to get pmic reg value\n"));
		FastbootFail ("Failed to get pmic reg value");
	} else {
		DEBUG((EFI_D_ERROR, "[ABL] CmdOemGetPmicReg : value = %x\n", value));
		AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "PmicIndex = %x , RegAddress %x = %x", PmicIndex, RegValue, value);
		FastbootInfo(DeviceInfo);
		WaitForTransferComplete();
		FastbootOkay("");
	}

	DEBUG((EFI_D_ERROR, "[ABL]  --- CmdOemGetPmicReg()\n\n"));
}
// --- ASUS_BSP : add for get pmic reg value

// +++ ASUS_BSP : add for ASUS dongle unlock
/* Convert character to lowercase */
STATIC int tolower (int c)
{
	if (('A' <= (c)) && ((c) <= 'Z'))
	{
		return (c - ('A' - 'a'));
	}
	return (c);
}

UINT16 shitfLeft(UINT16 p, int num)
{
	uint16 A , B , C;
	A = p << num;
	B = p >> (16 - num);
	C = A | B ;
	return C;
}

int algo1_A68(UINT16 *p1,UINT16 *p2,UINT16 *p3,UINT16 *p4)
{
	uint16 A,B,C,D;

	A = *p1;
	B = *p2;
	C = *p3;
	D = *p4;

	A = A+D;
	D = D+C;
	C = C+B;
	B = B+A;

	A = shitfLeft(A,2);
	B = shitfLeft(B,9);
	C = shitfLeft(C,1);
	D = shitfLeft(D,4);

	A = A+6;
	B = B+7;
	C = C+0;
	D = D+0;

	A = A^B;
	B = B^C;
	C = C^D;
	D = D^A;

	*p1 =A;
	*p2 =B;
	*p3 =C;
	*p4 =D;

	return 0;
}

int algo2_A68(UINT16 *p1,UINT16 *p2,UINT16 *p3,UINT16 *p4)
{
	uint16 A,B,C,D;

	A = *p1;
	B = *p2;
	C = *p3;
	D = *p4;

	A = A+D;
	D = D+C;
	C = C+B;
	B = B+A;

	A = shitfLeft(A,6);
	B = shitfLeft(B,0);
	C = shitfLeft(C,8);
	D = shitfLeft(D,3);

	A = A+3;
	B = B+9;
	C = C+2;
	D = D+3;

	A = A^B;
	B = B^C;
	C = C^D;
	D = D^A;

	*p1 =A;
	*p2 =B;
	*p3 =C;
	*p4 =D;

	return 0;
}

//call by cmd_oem_gen_hash()
void main_algo(dongleAlgoType SIMLockAuthAlgoType, char *data,char *final_data)
{
	uint16 p1,p2,p3,p4;
	unsigned char RecvData[33];
	int i =0, j=0;
	char AuthData[65];
//	int m=0;

	memset(RecvData, 0, sizeof(RecvData));
	memcpy(RecvData, data, 32);
	//RecvData[32]='\0';

	while(i<32)
	{
		memcpy(&p1, RecvData+i, 2);
		i+=2;
		memcpy(&p2, RecvData+i, 2);
		i+=2;
		memcpy(&p3, RecvData+i, 2);
		i+=2;
		memcpy(&p4, RecvData+i, 2);
		i+=2;

		if(SIMLockAuthAlgoType == A68)
		{
			algo1_A68(&p1, &p2, &p3, &p4);
		}
		else{}

		// Combine variables
		memcpy(RecvData+j, &p4, 2);
		j+=2;
		memcpy(RecvData+j, &p3, 2);
		j+=2;
		memcpy(RecvData+j, &p2, 2);
		j+=2;
		memcpy(RecvData+j, &p1, 2);
		j+=2;

		// Dispatch Calculate variables
		memcpy(&p1, RecvData+i, 2);
		i+=2;
		memcpy(&p2, RecvData+i, 2);
		i+=2;
		memcpy(&p3, RecvData+i, 2);
		i+=2;
		memcpy(&p4, RecvData+i, 2);
		i+=2;

		if(SIMLockAuthAlgoType == A68)
		{
			algo2_A68(&p1, &p2, &p3, &p4);
		}
		else{}

		// Combine variables
		memcpy(RecvData+j, &p4, 2);
		j+=2;
		memcpy(RecvData+j, &p3, 2);
		j+=2;
		memcpy(RecvData+j, &p2, 2);
		j+=2;
		memcpy(RecvData+j, &p1, 2);
		j+=2;
	}

	memset(AuthData,0,sizeof(AuthData));

	//for( m = 0; m < 32; m++ )
	{
		//sprintf( AuthData + m * 2, "%02x", RecvData[m] );
		AsciiSPrint(AuthData,sizeof(AuthData) ,"%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x",
					RecvData[0],RecvData[1],RecvData[2],RecvData[3],RecvData[4],RecvData[5],RecvData[6],RecvData[7],
					RecvData[8],RecvData[9],RecvData[10],RecvData[11],RecvData[12],RecvData[13],RecvData[14],RecvData[15],
					RecvData[16],RecvData[17],RecvData[18],RecvData[19],RecvData[20],RecvData[21],RecvData[22],RecvData[23],
					RecvData[24],RecvData[25],RecvData[26],RecvData[27],RecvData[28],RecvData[29],RecvData[30],RecvData[31]);

	}

	#if 0
	DEBUG((EFI_D_ERROR, "[ABL] main_algo : AuthData=%a\n",AuthData));
	DEBUG((EFI_D_ERROR, "[ABL] AuthData[65] = "));
	for( i = 0; i < 65; i++ )
	{
		DEBUG((EFI_D_ERROR, " 0x%02x",AuthData[i]));
	}
	DEBUG((EFI_D_ERROR, "\n"));
	#endif

	//char final_data[65];
	memcpy(final_data, AuthData ,64);
}

void cmd_oem_gen_hash(char * rand_buf, int key_select)
{
	//char response[256];
	char final_data[65];
	//struct MD5Context ctx;
	MD5_CTX ctx;
	unsigned char md5sum[16];
	char output[33];
	//char mmc_buf[512];
	char buf_rand_key[41];
	unsigned int i=0;

	DEBUG((EFI_D_ERROR, "\n"));
	DEBUG((EFI_D_ERROR, "[ABL] +++ cmd_oem_gen_hash (key_select=%d)\n", key_select));

	#if 0	//dannio test
	char rand_number[33]="08000260013335480527294877279139";
	char key[9]="12345678";
	sprintf(buf_rand_key, "%s%s", rand_number, key);
	DEBUG((EFI_D_ERROR, "[ABL] buf_rand_key[%d] = %s\n", sizeof(buf_rand_key), buf_rand_key));
	#endif

	memset(buf_rand_key, 0, sizeof(buf_rand_key));
	memcpy(buf_rand_key, rand_number, 32);

	if(key_select == 1)
		memcpy(&buf_rand_key[32], key, 9);
	else if(key_select == 2)
		memcpy(&buf_rand_key[32], key2, 9);

	DEBUG((EFI_D_ERROR, "[ABL] buf_rand_key[%d] = %a\n", sizeof(buf_rand_key), buf_rand_key));

	#if 0
	DEBUG((EFI_D_ERROR, "[ABL] buf_rand_key[%d] = \n", sizeof(buf_rand_key)));
	for(i = 0; i < sizeof(buf_rand_key); i++)
	{
		DEBUG((EFI_D_ERROR, " 0x%02x", buf_rand_key[i]));
	}
	DEBUG((EFI_D_ERROR, "\n"));
	#endif

	MD5Init(&ctx);
	MD5Update(&ctx, (char*)buf_rand_key, (UINTN)strlen(buf_rand_key));
	MD5Final(&ctx, md5sum);

	#if 0
	i=0;
	DEBUG((EFI_D_ERROR, "[ABL] md5sum[%d] = ", sizeof(md5sum)));
	for(i = 0; i < sizeof(md5sum); i++)
	{
		DEBUG((EFI_D_ERROR," 0x%02x", md5sum[i]));
	}
	DEBUG((EFI_D_ERROR,"\n"));
	#endif

	memset(output,0,33);
	//for(i = 0; i < 16; i++)
	{
		//sprintf( output + i * 2, "%02x", md5sum[i]);
		AsciiSPrint(output, sizeof(output), "%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x",
					md5sum[0],md5sum[1],md5sum[2],md5sum[3],md5sum[4],md5sum[5],md5sum[6],md5sum[7],md5sum[8],
					md5sum[9],md5sum[10],md5sum[11],md5sum[12],md5sum[13],md5sum[14],md5sum[15]);
	}

	DEBUG((EFI_D_ERROR, "[ABL] output = %a\n", output));

	#if 0
	i=0;
	DEBUG((EFI_D_ERROR,"[ABL] output[%d] = \n", sizeof(output)));
	for(i = 0; i < sizeof(output); i++)
	{
		DEBUG((EFI_D_ERROR," 0x%02x", output[i]));
	}
	DEBUG((EFI_D_ERROR,"\n"));
	#endif

	//DEBUG((EFI_D_ERROR, "[ABL] lower : output[33] = "));

	for(i = 0; i < 33; i++ )
	{
		output[i] = (char)tolower((int)output[i]);
		//DEBUG((EFI_D_ERROR, " 0x%02x", output[i]));
	}

	DEBUG((EFI_D_ERROR, "[ABL] lower : output = %a\n", output));

	#if 0
	for(i = 0; i < 33; i++ )
	{
		DEBUG((EFI_D_ERROR, " 0x%02x", output[i]));
	}
	DEBUG((EFI_D_ERROR, "\n"));
	#endif

	memset(final_data , 0 ,65);
	main_algo(A68, output, final_data);

	#if 0
	DEBUG((EFI_D_ERROR,"[ABL] final_data[%d] = \n", sizeof(final_data)));
	for(i = 0; i < sizeof(final_data); i++)
	{
		DEBUG((EFI_D_ERROR,"%d 0x%02x\n", i, final_data[i]));
	}
	DEBUG((EFI_D_ERROR,"\n"));
	#endif

	for( i = 0; i < 65; i++ )
	{
		final_data[i] = (char)tolower((int)final_data[i]);
	}

	#if 0
	for(i = 0; i < 65; i++ )
	{
		DEBUG((EFI_D_ERROR,"%d 0x%02x\n", i, final_data[i]));
	}
	DEBUG((EFI_D_ERROR, "\n"));
	#endif

	DEBUG((EFI_D_ERROR,"[ABL] before cmd_oem_gen_hash : calculate hash_buf [%d] = %a \n", sizeof(calculate_hash_buf), calculate_hash_buf));
	memcpy(calculate_hash_buf, final_data, hash_buf_size);
	DEBUG((EFI_D_ERROR,"[ABL] cmd_oem_gen_hash : final_data [%d] = %a \n", sizeof(final_data), final_data));
	DEBUG((EFI_D_ERROR,"[ABL] cmd_oem_gen_hash : calculate hash_buf [%d] = %a \n", sizeof(calculate_hash_buf), calculate_hash_buf));
	DEBUG((EFI_D_ERROR,"[ABL] -- cmd_oem_gen_hash()\n"));
	DEBUG((EFI_D_ERROR,"\n"));
}

// +++ ASUS_BSP : add for WaterMask unlock
STATIC VOID CmdOemGetIMEIAuth(CONST CHAR8 *arg, VOID *data, UINT32 sz)
{
	CHAR8 resp[MAX_RSP_SIZE];
	char TempChar[16]={};

	DEBUG((EFI_D_ERROR, "\n"));
	DEBUG((EFI_D_ERROR, "[ABL] +++ CmdOemGetIMEIAuth()\n"));

	ASUS_GEN_RANDOM_FLAG = 0;

	memset(rand_number, 0, sizeof(rand_number));

	GetIMEINum(TempChar, sizeof(TempChar));
	memcpy(rand_number, TempChar, 15);

	GetIMEI2Num(TempChar, sizeof(TempChar));
	memcpy(&rand_number[15], TempChar, 15);
	memcpy(&rand_number[30], "00", 2);

	AsciiSPrint(resp, sizeof(resp), "==================================");
	FastbootInfo(resp);
	WaitForTransferComplete();

	AsciiSPrint(resp, sizeof(resp), "IMEI Auth = %a ", rand_number);
	FastbootInfo(resp);
	WaitForTransferComplete();

	AsciiSPrint(resp, sizeof(resp), "==================================");
	FastbootInfo(resp);
	WaitForTransferComplete();

	ASUS_GEN_RANDOM_FLAG = 1;

	DEBUG((EFI_D_ERROR, "[ABL] --- CmdOemGetIMEIAuth()\n"));
	DEBUG((EFI_D_ERROR, "\n"));

	FastbootOkay("");
}

STATIC VOID CmdOemAuthHash3(CONST CHAR8 *arg, VOID *data, UINT32 sz)
{
	CHAR8 resp[MAX_RSP_SIZE];
	DEBUG((EFI_D_ERROR, "\n"));
	DEBUG((EFI_D_ERROR, "[ABL] +++ CmdOemAuthHash3()\n"));

	char TempChar[16]={};

	memset(rand_number, 0, sizeof(rand_number));

	GetIMEINum(TempChar, sizeof(TempChar));
	memcpy(rand_number, TempChar, 15);

	GetIMEI2Num(TempChar, sizeof(TempChar));
	memcpy(&rand_number[15], TempChar, 15);
	memcpy(&rand_number[30], "00", 2);

	if (ASUS_GEN_RANDOM_FLAG)
	{
		cmd_oem_gen_hash(rand_number, 2);

		AsciiSPrint(resp, sizeof(resp), "==================================");
		FastbootInfo(resp);
		WaitForTransferComplete();

		AsciiSPrint(resp, sizeof(resp), "\tCalculate hash =");
		FastbootInfo(resp);
		WaitForTransferComplete();

		AsciiSPrint(resp, sizeof(resp), "\t %s " ,&calculate_hash_buf[0]);
		FastbootInfo(resp);
		WaitForTransferComplete();

		AsciiSPrint(resp, sizeof(resp), "\t %s ", &calculate_hash_buf[16]);
		FastbootInfo(resp);
		WaitForTransferComplete();

		AsciiSPrint(resp, sizeof(resp), "\t %s ", &calculate_hash_buf[32]);
		FastbootInfo(resp);
		WaitForTransferComplete();

		AsciiSPrint(resp, sizeof(resp), "==================================");
		FastbootInfo(resp);
		WaitForTransferComplete();

		AsciiSPrint(resp, sizeof(resp), "==================================");
		FastbootInfo(resp);
		WaitForTransferComplete();

		AsciiSPrint(resp, sizeof(resp), "\tReceiver hash =");
		FastbootInfo(resp);
		WaitForTransferComplete();

		AsciiSPrint(resp, sizeof(resp), "\t %s ", &mmc_hash_buf[0]);
		FastbootInfo(resp);
		WaitForTransferComplete();

		AsciiSPrint(resp, sizeof(resp), "\t %s ", &mmc_hash_buf[16]);
		FastbootInfo(resp);
		WaitForTransferComplete();

		AsciiSPrint(resp, sizeof(resp), "\t %s ", &mmc_hash_buf[32]);
		FastbootInfo(resp);
		WaitForTransferComplete();

		AsciiSPrint(resp, sizeof(resp), "==================================");
		FastbootInfo(resp);
		WaitForTransferComplete();

		if(AsciiStrnCmp(calculate_hash_buf, mmc_hash_buf, hash_buf_size) == 0)
		{
			DEBUG((EFI_D_ERROR,"[ABL] cmd_oem_auth_hash - Authorized hash : PASS \n"));

			SetAuthorized3Value(TRUE);
			IsAllowUnlock = 1;

			if (!(IsAuthorized_3()))
			{
				DEBUG((EFI_D_ERROR, "[ABL] Unable to Write Device Info : IsAuthorized_3 = %d\n", IsAuthorized_3()));
				FastbootFail("");
				return;
			}

			IsAllowUnlock = 1;
			AsciiSPrint(resp, sizeof(resp), "\tAuthorized Result : PASS");

			FastbootInfo(resp);
			WaitForTransferComplete();
		}
		else
		{
			DEBUG((EFI_D_ERROR, "[ABL] (ERROR) cmd_oem_auth_hash - Authorized hash : FAILED \n"));

			SetAuthorized3Value(FALSE);
			if (IsAuthorized_3())
			{
				DEBUG((EFI_D_ERROR, "[ABL] Unable to Write Device Info : IsAuthorized_3 = %r\n", IsAuthorized_3()));
				FastbootFail("");
				return;
			}

			AsciiSPrint(resp, sizeof(resp), "\tAuthorized Result : FAILED");
			FastbootInfo(resp);
			WaitForTransferComplete();
		}
	}
	else
	{
		AsciiSPrint(resp, sizeof(resp), "fastboot oem get-imeiauth, first");
		FastbootInfo(resp);
		WaitForTransferComplete();
		FastbootFail("fastboot oem get-imeiauth, first");
	}

	// clean flag
	ASUS_GEN_RANDOM_FLAG = 0;

	FastbootOkay("");

	DEBUG((EFI_D_ERROR, "[ABL] --- CmdOemAuthHash3()\n"));
	DEBUG((EFI_D_ERROR, "\n"));
}
// --- ASUS_BSP : add for WaterMask unlock

STATIC VOID CmdOemAuthHash2(CONST CHAR8 *arg, VOID *data, UINT32 sz)
{
	CHAR8 resp[MAX_RSP_SIZE];
	DEBUG((EFI_D_ERROR, "\n"));
	DEBUG((EFI_D_ERROR, "[ABL] +++ CmdOemAuthHash2()\n"));

	if (ASUS_GEN_RANDOM_FLAG)
	{
		cmd_oem_gen_hash(rand_number, 1);

		AsciiSPrint(resp, sizeof(resp), "==================================");
		FastbootInfo(resp);
		WaitForTransferComplete();

		AsciiSPrint(resp, sizeof(resp), "\tCalculate hash =");
		FastbootInfo(resp);
		WaitForTransferComplete();

		AsciiSPrint(resp, sizeof(resp), "\t %s " ,&calculate_hash_buf[0]);
		FastbootInfo(resp);
		WaitForTransferComplete();

		AsciiSPrint(resp, sizeof(resp), "\t %s ", &calculate_hash_buf[16]);
		FastbootInfo(resp);
		WaitForTransferComplete();

		AsciiSPrint(resp, sizeof(resp), "\t %s ", &calculate_hash_buf[32]);
		FastbootInfo(resp);
		WaitForTransferComplete();

		AsciiSPrint(resp, sizeof(resp), "==================================");
		FastbootInfo(resp);
		WaitForTransferComplete();

		AsciiSPrint(resp, sizeof(resp), "==================================");
		FastbootInfo(resp);
		WaitForTransferComplete();

		AsciiSPrint(resp, sizeof(resp), "\tReceiver hash =");
		FastbootInfo(resp);
		WaitForTransferComplete();

		AsciiSPrint(resp, sizeof(resp), "\t %s ", &mmc_hash_buf[0]);
		FastbootInfo(resp);
		WaitForTransferComplete();

		AsciiSPrint(resp, sizeof(resp), "\t %s ", &mmc_hash_buf[16]);
		FastbootInfo(resp);
		WaitForTransferComplete();

		AsciiSPrint(resp, sizeof(resp), "\t %s ", &mmc_hash_buf[32]);
		FastbootInfo(resp);
		WaitForTransferComplete();

		AsciiSPrint(resp, sizeof(resp), "==================================");
		FastbootInfo(resp);
		WaitForTransferComplete();

		if(AsciiStrnCmp(calculate_hash_buf, mmc_hash_buf, hash_buf_size) == 0)
		{
			DEBUG((EFI_D_ERROR,"[ABL] cmd_oem_auth_hash - Authorized hash : PASS \n"));

			SetAuthorized2Value(TRUE);
			IsAllowUnlock = 1;

			if (!(IsAuthorized_2()))
			{
				DEBUG((EFI_D_ERROR, "[ABL] Unable to Write Device Info : IsAuthorized_2 = %d\n", IsAuthorized_2()));
				FastbootFail("");
				return;
			}

			IsAllowUnlock = 1;
			AsciiSPrint(resp, sizeof(resp), "\tAuthorized Result : PASS");

			FastbootInfo(resp);
			WaitForTransferComplete();
		}
		else
		{
			DEBUG((EFI_D_ERROR, "[ABL] (ERROR) cmd_oem_auth_hash - Authorized hash : FAILED \n"));

			SetAuthorized2Value(FALSE);
			if (IsAuthorized_2())
			{
				DEBUG((EFI_D_ERROR, "[ABL] Unable to Write Device Info : IsAuthorized_2 = %r\n", IsAuthorized_2()));
				FastbootFail("");
				return;
			}

			AsciiSPrint(resp, sizeof(resp), "\tAuthorized Result : FAILED");
			FastbootInfo(resp);
			WaitForTransferComplete();
		}
	}
	else
	{
		AsciiSPrint(resp, sizeof(resp), "fastboot oem gen-random, first");
		FastbootInfo(resp);
		WaitForTransferComplete();
		FastbootFail("fastboot oem gen-random, first");
	}

	// clean flag
	ASUS_GEN_RANDOM_FLAG = 0;

	FastbootOkay("");

	DEBUG((EFI_D_ERROR, "[ABL] --- CmdOemAuthHash2()\n"));
	DEBUG((EFI_D_ERROR, "\n"));
}

STATIC VOID CmdOemAuthHash(CONST CHAR8 *arg, VOID *data, UINT32 sz)
{
	CHAR8 resp[MAX_RSP_SIZE];
	DEBUG((EFI_D_ERROR, "\n"));
	DEBUG((EFI_D_ERROR, "[ABL] +++ CmdOemAuthHash()\n"));
	
	// gen random numbers
	random_num_generator(rand_number);

	// gen hash numbers
	cmd_oem_gen_hash(rand_number, 1);

	AsciiSPrint(resp, sizeof(resp), "==================================");
	FastbootInfo(resp);
	WaitForTransferComplete();

	AsciiSPrint(resp, sizeof(resp), "\tCalculate hash =");
	FastbootInfo(resp);
	WaitForTransferComplete();

	AsciiSPrint(resp, sizeof(resp), "\t %s ", &calculate_hash_buf[0]);
	FastbootInfo(resp);
	WaitForTransferComplete();

	AsciiSPrint(resp, sizeof(resp), "\t %s ", &calculate_hash_buf[16]);
	FastbootInfo(resp);
	WaitForTransferComplete();

	AsciiSPrint(resp, sizeof(resp), "\t %s ", &calculate_hash_buf[32]);
	FastbootInfo(resp);
	WaitForTransferComplete();

	AsciiSPrint(resp, sizeof(resp), "==================================");
	FastbootInfo(resp);
	WaitForTransferComplete();

	//cmd_oem_mmc_read_hash("asuskey2");

	AsciiSPrint(resp, sizeof(resp), "==================================");
	FastbootInfo(resp);
	WaitForTransferComplete();

	AsciiSPrint(resp, sizeof(resp), "\tReceiver hash =");
	FastbootInfo(resp);
	WaitForTransferComplete();

	AsciiSPrint(resp, sizeof(resp), "\t %s ", &mmc_hash_buf[0]);
	FastbootInfo(resp);
	WaitForTransferComplete();

	AsciiSPrint(resp, sizeof(resp), "\t %s ", &mmc_hash_buf[16]);
	FastbootInfo(resp);
	WaitForTransferComplete();

	AsciiSPrint(resp, sizeof(resp), "\t %s ", &mmc_hash_buf[32]);
	FastbootInfo(resp);
	WaitForTransferComplete();

	AsciiSPrint(resp, sizeof(resp), "==================================");
	FastbootInfo(resp);
	WaitForTransferComplete();

	// verify hash file
	if(AsciiStrnCmp(calculate_hash_buf, mmc_hash_buf, hash_buf_size) == 0)
	{
		DEBUG((EFI_D_ERROR,"[ABL] cmd_oem_auth_hash - Authorized hash : PASS \n"));

		SetAuthorizedValue(TRUE);
		IsAllowUnlock = 1;

		if (!(IsAuthorized()))
		{
			DEBUG((EFI_D_ERROR, "[ABL] Unable to Write Device Info : IsAuthorized = %d\n", IsAuthorized()));
			FastbootFail("");
			return;
		}

#ifdef ASUS_AI2205_BUILD		
		SetDeviceDebugUnlockValue(TRUE);
		if(!(IsDebugUnlocked())){
			DEBUG((EFI_D_ERROR, "[ABL] Unable to Write Device Info : IsDebugUnlocked = %d\n", IsDebugUnlocked()));
			FastbootFail("");
			return;
		}
#endif

		IsAllowUnlock = 1;
		AsciiSPrint(resp, sizeof(resp), "\tAuthorized Result : PASS");

		FastbootInfo(resp);
		WaitForTransferComplete();
	}
	else
	{
		DEBUG((EFI_D_ERROR, "[ABL] (ERROR) cmd_oem_auth_hash - Authorized hash : FAILED \n"));

		SetAuthorizedValue(FALSE);
		if (IsAuthorized())
		{
			DEBUG((EFI_D_ERROR, "[ABL] Unable to Write Device Info : IsAuthorized = %r\n", IsAuthorized()));
			FastbootFail("");
			return;
		}

#ifdef ASUS_AI2205_BUILD
		SetDeviceDebugUnlockValue(FALSE);
		if(IsDebugUnlocked()){
			DEBUG((EFI_D_ERROR, "[ABL] Unable to Write Device Info : IsDebugUnlocked = %d\n", IsDebugUnlocked()));
			FastbootFail("");
			return;
		}
#endif

		AsciiSPrint(resp, sizeof(resp), "\tAuthorized Result : FAILED");
		FastbootInfo(resp);
		WaitForTransferComplete();
	}

	// clean flag
	ASUS_GEN_RANDOM_FLAG = 0;

	FastbootOkay("");

	DEBUG((EFI_D_ERROR, "[ABL] --- CmdOemAuthHash()\n"));
	DEBUG((EFI_D_ERROR, "\n"));
}

void random_num_generator(char *rand_num)
{
	EFI_STATUS Status = EFI_SUCCESS;
	UINT32 TimerCount = 0;
	UINT32 UUID = 0;
	UINT32 tmp = 0;
	char time_buf[32];
	MD5_CTX Md5Ctx;
	UINT8 md5sum[16];
	int i=0;
	char output[33];

	DEBUG((EFI_D_ERROR, "[ABL] +++ random_num_generator()\n"));

#ifdef AI2201_USE_RANDON_NUM_UNLOCK
	TimerCount = (UINT32) GetPerformanceCounter();
#endif
	Status = GetSerialNum(&UUID);

	if (Status != EFI_SUCCESS)
	{
		DEBUG((EFI_D_ERROR, "[ABL] Failed to GetSerialNum() \n"));
		return;
	}
	else
	{
		DEBUG((EFI_D_ERROR, "[ABL] CmdOemGenRNG : UUID : %x\n", UUID));
		DEBUG((EFI_D_ERROR, "[ABL] CmdOemGenRNG : TimerCount : %d\n", TimerCount));

		tmp = UUID + TimerCount;
		DEBUG((EFI_D_ERROR, "[ABL] CmdOemGenRNG : tmp : %d\n", tmp));

		AsciiSPrint(time_buf, sizeof(time_buf), "%x", tmp);

		DEBUG((EFI_D_ERROR, "[ABL] CmdOemGenRNG : time_buf : %a\n", time_buf));

		Status = MD5Init (&Md5Ctx);
		if (Status != EFI_SUCCESS)
		{
			DEBUG((EFI_D_ERROR, "[ABL] Failed to MD5Init() \n"));
			return;
		}

		MD5Update(&Md5Ctx, (char *) time_buf, (UINTN)strlen( time_buf ));
		MD5Final(&Md5Ctx, md5sum);

		#if 0
		DEBUG((EFI_D_ERROR, "[ABL] CmdOemGenRNG : md5sum=%a\n", md5sum));
		DEBUG((EFI_D_ERROR, "[ABL] md5sum[16]="));
		for( i = 0; i < 16; i++ )
		{
			DEBUG((EFI_D_ERROR, " 0x%02x", md5sum[i]));
		}
		DEBUG((EFI_D_ERROR, "\n"));
		#endif

		memset(output, 0, sizeof(output));

		//for( i = 0; i < 16; i++ )
		{
			//sprintf(output + i * 2, "%02x", md5sum[i] );
			AsciiSPrint(output, sizeof(output), "%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x",
						md5sum[0],md5sum[1],md5sum[2],md5sum[3],md5sum[4],md5sum[5],md5sum[6],md5sum[7],md5sum[8],
						md5sum[9],md5sum[10],md5sum[11],md5sum[12],md5sum[13],md5sum[14],md5sum[15]);
		}

		DEBUG((EFI_D_ERROR, "[ABL] CmdOemGenRNG : output=%a\n", output));

		#if 0
		DEBUG((EFI_D_ERROR, "[ABL] output[33] = "));
		for( i = 0; i < 33; i++ )
		{
			DEBUG((EFI_D_ERROR, " 0x%02x", output[i]));
		}
		#endif

		memset(rand_num, 0, 32);

		//rand_number[]
		memcpy(rand_num, output, 32);
		rand_num[32]='\0';

		for( i = 0; i < 33; i++ )
		{
			rand_num[i] = (char)tolower((int)rand_num[i]);
			//DEBUG((EFI_D_ERROR, " 0x%02x",rand_num[i]));
		}

		DEBUG((EFI_D_ERROR, "[ABL] CmdOemGenRNG : rand_num=%a\n", rand_num));

		#if 0
		DEBUG((EFI_D_ERROR, "[ABL] CmdOemGenRNG : rand_num=%a\n", rand_num));
		DEBUG((EFI_D_ERROR, "rand_num[32]="));
		for( i = 0; i < 16; i++ )
		{
			DEBUG((EFI_D_ERROR, "%02x",rand_num[i]));
		}
		DEBUG((EFI_D_ERROR, "\n"));
		#endif
	}

	DEBUG((EFI_D_ERROR, "[ABL] --- random_num_generator()\n"));
	return;
}

STATIC VOID CmdOemGenRNG(CONST CHAR8 *arg, VOID *data, UINT32 sz)
{
	CHAR8 resp[MAX_RSP_SIZE];

	DEBUG((EFI_D_ERROR, "\n"));
	DEBUG((EFI_D_ERROR, "[ABL] +++ CmdOemGenRNG()\n"));

	ASUS_GEN_RANDOM_FLAG = 0;

	random_num_generator(rand_number);

	AsciiSPrint(resp, sizeof(resp), "==================================");
	FastbootInfo(resp);
	WaitForTransferComplete();

	AsciiSPrint(resp, sizeof(resp), "Random Num = %a ", rand_number);
	FastbootInfo(resp);
	WaitForTransferComplete();

	AsciiSPrint(resp, sizeof(resp), "==================================");
	FastbootInfo(resp);
	WaitForTransferComplete();

	ASUS_GEN_RANDOM_FLAG = 1;

	DEBUG((EFI_D_ERROR, "[ABL] --- CmdOemGenRNG()\n"));
	DEBUG((EFI_D_ERROR, "\n"));

	FastbootOkay("");
}
// --- ASUS_BSP : add for ASUS dongle unlock

// +++ ASUS_BSP : add for sha256 function
STATIC VOID CmdOemSha256(CONST CHAR8 * arg, VOID * data, UINT32 sz)
{
	EFI_STATUS Status = EFI_SUCCESS;
	CHAR8 DeviceInfo[MAX_RSP_SIZE];

	DEBUG((EFI_D_ERROR, "[ABL]  +++ CmdOemSha256(%d)\n",sz));
	UINT8 hash[32] = {0};
	memset(hash,0,sizeof(hash));

	#if 1
	UINT32 i=0;
	UINT8 * buf = NULL;
	buf = (UINT8*)data;

	DEBUG((EFI_D_ERROR, "\n"));
	DEBUG((EFI_D_ERROR, "[ABL] CmdOemSha256 - data[%d] = \n",sz));
	for(i=0;i<sz;i++)
	{
		DEBUG((EFI_D_ERROR, " %02x",buf[i] ));
	}
	DEBUG((EFI_D_ERROR, "\n"));
	#endif

	#if 0
	CHAR8 asus_key_info[16]={0};	// Get SSN

	AsciiSPrint(asus_key_info, sizeof(asus_key_info), "%a", "G6AZCY03S376EYA");
	DEBUG((EFI_D_ERROR, "[ABL]  CmdOemSha256(SSN=%a)\n",asus_key_info));
	Status = get_image_hash((UINT8*)asus_key_info,15,hash,sizeof(hash),VB_SHA256);

	#else
	Status = get_image_hash((UINT8*)data,(UINTN)sz,hash,sizeof(hash),VB_SHA256);
	#endif

	if (Status != EFI_SUCCESS)
	{
		DEBUG((EFI_D_ERROR, "[ABL] (ERROR) get_image_hash : FAIL (%d)\n", Status));
		FastbootFail("SHA256 : FAIL");
		return;
	}

	//0~31
	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x",
										   hash[0],hash[1],hash[2],hash[3],hash[4],hash[5],hash[6],hash[7],hash[8],hash[9],
										   hash[10],hash[11],hash[12],hash[13],hash[14],hash[15],hash[15]);
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	// 32~64
	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x",
										   hash[16],hash[17],hash[18],hash[19],hash[20],hash[21],hash[22],hash[23],hash[24],
										   hash[25],hash[26],hash[27],hash[28],hash[29],hash[30],hash[31]);
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();
	//FastbootOkay("");
	DEBUG((EFI_D_ERROR, "[ABL]  --- CmdOemSha256()\n"));
}
// --- ASUS_BSP : add for sha256 function

// +++ ASUS_BSP : add for get partition hash
STATIC VOID CmdOemCalculatePtCrc32(CONST CHAR8 *arg, VOID *data, UINT32 sz){
	CHAR16 PartitionName[MAX_GPT_NAME_SIZE];
	unsigned long calculatedCRC=0xFFFFFFFF;
	CHAR8 DeviceInfo[MAX_RSP_SIZE];

	DEBUG((EFI_D_ERROR, "[ABL] +++ CmdOemCalculatePtCrc32()\n"));
	AsciiStrToUnicodeStr((CHAR8 *)arg, PartitionName);
	calculatedCRC = AsusCalculatePtCrc32((CHAR16 *)&PartitionName);
	DEBUG((EFI_D_INFO, "calculatedCRC = %x\n", calculatedCRC));

	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "%x", calculatedCRC);
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();
	DEBUG((EFI_D_ERROR, "[ABL]  --- CmdOemCalculatePtCrc32()\n"));
	FastbootOkay("");
}
// --- ASUS_BSP : add for get partition hash

// +++ ASUS_BSP : add for user unlock
STATIC VOID CmdRsaTest(CONST CHAR8 * arg, VOID * data, UINT32 sz)
{
	CHAR8 DeviceInfo[MAX_RSP_SIZE];
	UINT32 crypt_type = 0;

	crypt_type = (UINT32)AsciiStrHexToUint64(arg)&0xFFFF;
	DEBUG((EFI_D_ERROR, "[ABL] +++ CmdRsaTest(%d)\n", crypt_type));

	if(crypt_type > RAW_PKG_LOCK)
	{
		AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "NON-SUPPORT CRYPT TYPE : %d", crypt_type);
		FastbootInfo(DeviceInfo);
		WaitForTransferComplete();
		FastbootFail("Verify : FAIL");
		goto out;
	}

	if(is_unlock(crypt_type))
	{
		AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "Verify : PASS");
		FastbootInfo(DeviceInfo);
		WaitForTransferComplete();
		FastbootOkay("");
	}
	else
	{
		FastbootFail("Verify : FAIL");
	}

out:
	DEBUG((EFI_D_ERROR, "[ABL]  --- CmdRsaTest()\n"));
}
// --- ASUS_BSP : add for user unlock

// +++ ASUS_BSP : add for oem ASUS CSC lock device
STATIC VOID CmdOemAsusCscLk(CONST CHAR8 *arg, VOID *data, UINT32 sz)
{
	UINT32 lock_count;

	if( IsUnlocked() == FALSE)
	{
		FastbootFail("Device already : locked!");
		return;
	}

	SetDeviceUnlockValue(UNLOCK, FALSE);
	SetDeviceUnlockValue(UNLOCK_CRITICAL, FALSE);

	lock_count = GetLockCounter();
	lock_count++;
	SetLockCounter(lock_count);

	FastbootOkay("");
}
// --- ASUS_BSP : add for oem ASUS CSC lock device

// +++ ASUS_BSP : add for Check Setup Wizard
STATIC VOID CmdOemCheckSetupWizard(CONST CHAR8 * arg, VOID * data, UINT32 sz)
{
	CHAR8 DeviceInfo[MAX_RSP_SIZE];
	sys_info sysinfo;
	char sysinfo_setupwizard[6];
	DEBUG((EFI_D_ERROR, "[ABL] +++ CmdOemCheckSetupWizard()\n"));

	if(read_sysinfo(&sysinfo) == EFI_SUCCESS)
	{
		AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "setupwizard: %a", sysinfo.setupwizard);
		FastbootInfo(DeviceInfo);
		WaitForTransferComplete();

		AsciiSPrint(sysinfo_setupwizard, sizeof(sysinfo_setupwizard), "%a", sysinfo.setupwizard);
		if(!AsciiStrnCmp(sysinfo_setupwizard, "true", StrLen (L"true")))
		{
			DEBUG((EFI_D_ERROR, "[ABL] CmdOemCheckSetupWizard() -- setupwizard is true\n"));
			FastbootOkay("");
		}
		else
		{
			DEBUG((EFI_D_ERROR, "[ABL] CmdOemCheckSetupWizard() -- setupwizard is false\n"));
			FastbootOkay("");
		}
	}
	else
	{
		FastbootFail("read sysinfo fail");
	}
	DEBUG((EFI_D_ERROR, "[ABL] --- CmdOemCheckSetupWizard()\n"));
}

BOOLEAN CheckSetupWizard(void)
{
	sys_info sysinfo;
	char sysinfo_setupwizard[6];
	BOOLEAN check_setupwizard = FALSE;

	DEBUG((EFI_D_ERROR, "[ABL] +++ CheckSetupWizard()\n"));

	if(read_sysinfo(&sysinfo) == EFI_SUCCESS)
	{
		AsciiSPrint(sysinfo_setupwizard, sizeof(sysinfo_setupwizard), "%a", sysinfo.setupwizard);
		if(!AsciiStrnCmp(sysinfo_setupwizard, "true", StrLen (L"true")))
		{
			DEBUG((EFI_D_ERROR, "[ABL] CheckSetupWizard() -- setupwizard is true\n"));
			check_setupwizard = TRUE;
		}
		else
		{
			DEBUG((EFI_D_ERROR, "[ABL] CheckSetupWizard() -- setupwizard is false\n"));
			check_setupwizard = FALSE;
		}
	}
	else
	{
		DEBUG((EFI_D_ERROR, "[ABL] CheckSetupWizard() -- read sysinfo fail!!\n"));
		check_setupwizard = FALSE;
	}

	DEBUG((EFI_D_ERROR, "[ABL] --- CheckSetupWizard()\n"));
	return check_setupwizard;
}
// --- ASUS_BSP : add for Check Setup Wizard

// +++ ASUS_BSP : add for draw barcode
STATIC VOID CmdOemDrawBarcode(CONST CHAR8 *arg, VOID *data, UINT32 sz)
{
	// TODO
	//	It should show ISN number
	//
	CHAR8 TempCHAR[MAX_RSP_SIZE];
	GetISNNum(TempCHAR, sizeof(TempCHAR)); // +++ ASUS_BSP : Add for isn info

	// +++ ASUS_BSP : add for Check Setup Wizard
	if(CheckSetupWizard() == TRUE){
		DrawBarCode(TempCHAR);
		FastbootOkay("");
	}else{
		FastbootFail("Setupwizard is false!!");
	}
	// --- ASUS_BSP : add for Check Setup Wizard
}
//--- ASUS_BSP : add for draw barcode 

// +++ ASUS_BSP : add for get fuse info
STATIC VOID CmdOemFuseInfo(CONST CHAR8 *arg, VOID *data, UINT32 sz)
{
	CHAR8 DeviceInfo[MAX_RSP_SIZE];
	UINT32 SecureState = 0;

	SecureState = ReadSecurityState ();
	if (SecureState == ERROR_SECURITY_STATE) {
		DEBUG ((EFI_D_ERROR, "ReadSecurityState failed!\n"));
		FastbootFail("ReadSecurityState failed!");
	}

	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "=== FUSE Info ===");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "Secure_state = %x\n", SecureState);
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "Bit#0_SECBOOT_FUSE = %x (must be 0)", (CHECK_BIT(SecureState, SECBOOT_FUSE) >> (SECBOOT_FUSE)));
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "Bit#1 SHK_FUSE = %x (must be 0)", (CHECK_BIT(SecureState, SHK_FUSE) >> (SHK_FUSE)));
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "Bit#2 DEBUG_DISABLED_FUSE = %x ", (CHECK_BIT(SecureState, DEBUG_DISABLED_FUSE) >> (DEBUG_DISABLED_FUSE)));
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "Bit#3 ANTI_ROLLBACK_FUSE = %x ", (CHECK_BIT(SecureState, ANTI_ROLLBACK_FUSE) >> (ANTI_ROLLBACK_FUSE)));
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "Bit#4 FEC_ENABLED_FUSE = %x ", (CHECK_BIT(SecureState, FEC_ENABLED_FUSE) >> (FEC_ENABLED_FUSE)));
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "Bit#5 RPMB_ENABLED_FUS = %x (must be 0)", (CHECK_BIT(SecureState, RPMB_ENABLED_FUSE) >> (RPMB_ENABLED_FUSE)));
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "Bit#6 DEBUG_RE_ENABLED_FUSE = %x (must be 1)", (CHECK_BIT(SecureState, DEBUG_RE_ENABLED_FUSE) >> (DEBUG_RE_ENABLED_FUSE)));
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "Bit#7 FEC_ENABLED_FUSE = %x ", (CHECK_BIT(SecureState, FEC_ENABLED_FUSE) >> (FEC_ENABLED_FUSE)));
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "Bit#8 TZ_DEBUG_FUSE = %x (must be 0)", (CHECK_BIT(SecureState, TZ_DEBUG_FUSE) >> (TZ_DEBUG_FUSE)));
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "Bit#9 MSS_DEBUG_FUSE = %x (must be 0)", (CHECK_BIT(SecureState, MSS_DEBUG_FUSE) >> (MSS_DEBUG_FUSE)));
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "Bit#10 CP_DEBUG_FUSE = %x (must be 0)", (CHECK_BIT(SecureState, CP_DEBUG_FUSE) >> (CP_DEBUG_FUSE)));
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	FastbootOkay("");
}
// --- ASUS_BSP : add for get fuse info

// +++ ASUS_BSP : add for check fuse
STATIC VOID CmdOemCheckFuse(CONST CHAR8 *arg, VOID *data, UINT32 sz)
{
	CHAR8 DeviceInfo[MAX_RSP_SIZE];

	if(IsSecureBootEnabled())
	{
		AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "FUSED");
	}
	else
	{
		AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "NON-FUSED");
	}

	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();
	FastbootOkay("");
}
// --- ASUS_BSP : add for check fuse

// +++ ASUS_BSP : add for check fuse with no rpmb
STATIC VOID CmdOemCheckFuseNoRpmb(CONST CHAR8 *arg, VOID *data, UINT32 sz)
{
#ifdef ASUS_AI2205_BUILD
	CHAR8 DeviceInfo[MAX_RSP_SIZE];
	EFI_STATUS Status = EFI_SUCCESS;
	BOOLEAN SecureDeviceNoRpmb = FALSE;

	Status = IsSecureDeviceNoCheckRpmb(&SecureDeviceNoRpmb);
	if (Status != EFI_SUCCESS) {
		DEBUG ((EFI_D_ERROR, "VB: Failed read device state: %r\n", Status));
	}

	if(SecureDeviceNoRpmb)
	{
		AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "FUSED (no check rpmb)");
	}
	else
	{
		AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "NON-FUSED");
	}

	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();
#endif
	FastbootOkay("");
}
// --- ASUS_BSP : add for check fuse with no rpmb

// +++ ASUS_BSP : add for enter shipping mode
STATIC VOID CmdOemEnterShippingMode(CONST CHAR8 * arg, VOID * data, UINT32 sz)
{
	CHAR8 DeviceInfo[MAX_RSP_SIZE];
	EFI_STATUS Status = EFI_SUCCESS;

	Status = EnterShippingMode();

	if (Status != EFI_SUCCESS)
	{
		FastbootFail("Failed to enter shipmode");
	}
	else
	{
		AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "EnterShippingMode() is PASS");
		FastbootInfo(DeviceInfo);
		WaitForTransferComplete();
		FastbootOkay("");

	}
}
// --- ASUS_BSP : add for enter shipping mode

// +++ ASUS_BSP : add for check s3 reset type cmd
STATIC VOID CmdOemCheckS3(CONST CHAR8 *arg, VOID *data, UINT32 sz)
{
#if 0
	EFI_STATUS Status = EFI_SUCCESS;
#endif
	CHAR8 DeviceInfo[MAX_RSP_SIZE];
	UINT8 value = 0;
	UINT8 HW_ID = 0;

	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "======== Check S3 pm_app_pon_reset_source_type ========\n");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	GetPmicReg(0x0, 0x874, &value);
#ifdef ASUS_AI2205_BUILD
	HW_ID = Get_HW_ID();
#endif

	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "HW Stage : %x", HW_ID);
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "Reg_0x874 : %x\n", value);
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();
	FastbootOkay("");
}
// --- ASUS_BSP : add for check s3 reset type cmd

void fastboot_send_data(char *code, size_t Reason)
{
	EFI_STATUS Status;
	if (Reason > 0)
	{
	  	Status=GetFastbootDeviceData ()->UsbDeviceProtocol->Send (
		  	ENDPOINT_OUT, Reason,code);
		DEBUG ((EFI_D_ERROR, "data sending\n"));
		if (Status != EFI_SUCCESS) {
			DEBUG ((EFI_D_ERROR, "data send error\n"));
			return ;
		}
	}
}

void fastboot_data(size_t code)
{
  	AsciiSPrint (GetFastbootDeviceData ()->gTxBuffer, MAX_RSP_SIZE, "DATA%08x", code);
	DEBUG ((EFI_D_ERROR, "Data size =0x %08x\n",code,code));
  	GetFastbootDeviceData ()->UsbDeviceProtocol->Send (
      	ENDPOINT_OUT, AsciiStrLen (GetFastbootDeviceData ()->gTxBuffer),
      	GetFastbootDeviceData ()->gTxBuffer);
}
//use rules:fastboot oem partition name size   
//name : partition name
//size : this parameter can be NULL
//		 if imagesize is NULL, it will be equate to partition size 
STATIC VOID CmdPartition(CONST CHAR8 * arg, VOID * data, UINT32 sz)
{
	int flag = 0;
	uint64_t result = 0;
	uint64_t len_r = 0;
	uint64_t len = 0;
	uint64_t block_size;
	uint64_t size_per_read=0;
	uint64_t remainder = 0;
	UINT64 Offset=0;
	CHAR16 SlotSuffix[MAX_SLOT_SUFFIX_SZ];
	BOOLEAN HasSlot = FALSE;
	UINTN PartitionSize=0;
	EFI_STATUS Status;
  	EFI_BLOCK_IO_PROTOCOL *BlockIo = NULL;
 	EFI_HANDLE *Handle = NULL;
	BOOLEAN MultiSlotBoot = PartitionHasMultiSlot ((CONST CHAR16 *)L"boot");
	CHAR16 PartitionName[MAX_GPT_NAME_SIZE];
	int image_len = 0;
	CHAR8 *Ptr = NULL;
	int max_usb_data_sz = 1024 * 1024 * 1024;
	Ptr=AsciiStrStr(arg+1, " ");
	if(Ptr != NULL)	
		image_len=strtoul(Ptr,0,0);
	DEBUG((EFI_D_ERROR, "input size =%d\n",image_len));

	asus_strtok((char *)arg+1, " ");
	AsciiStrToUnicodeStr (arg+1, PartitionName);//字符串
	if (!PartitionName)
	{
		DEBUG((EFI_D_ERROR, "Param is NULL, Please Input Partition Param\n"));
		FastbootFail("Param is NULL, Please Input Partition Param");
		return;
	}

	if (MultiSlotBoot)
	{
		HasSlot = GetPartitionHasSlot (PartitionName, ARRAY_SIZE (PartitionName), SlotSuffix,
		                               MAX_SLOT_SUFFIX_SZ);
	}
	
	if(HasSlot==FALSE)
	{
		DEBUG ((EFI_D_ERROR, " %s no _a or _b\n", PartitionName));
	}
  	Status = PartitionGetInfo (PartitionName, &BlockIo, &Handle);//获得handle，blockio

 	if (Status != EFI_SUCCESS) 
	{	
		DEBUG ((EFI_D_ERROR, "partiton %s found error \n", PartitionName));
		FastbootFail("Param not found, Please Input Partition Param");
    	return ;
	}

	if (!BlockIo) 
	{
		DEBUG ((EFI_D_ERROR, "BlockIo for %s is corrupted\n", PartitionName));
		return ;
  	}

	if (!Handle) 
	{
		DEBUG ((EFI_D_ERROR, "EFI handle for %s is corrupted\n", PartitionName));
		return ;
  	}

	if(image_len)
	{
		PartitionSize = image_len;
	}
	else
	{
		PartitionSize = (BlockIo->Media->LastBlock + 1) * (BlockIo->Media->BlockSize);
		DEBUG ((EFI_D_ERROR, "Partition size = %llu\n", PartitionSize));
	}

	if(PartitionSize <= 0)
	{	
		DEBUG ((EFI_D_ERROR, " %s size error\n", PartitionName));
		return ;
	}
	block_size = BlockIo->Media->BlockSize;
	size_per_read = block_size*1024*8;//4096*1024*8=32MB
	len = PartitionSize;

	remainder = len % block_size;
    if (remainder)
	{
		len += block_size - remainder;
	}

	len_r = len;
	if(len_r)
		DEBUG ((EFI_D_ERROR, "will read size %d",len_r));

	if(!StrCmp(PartitionName,L"super"))
	{
		while(PartitionSize>max_usb_data_sz){
			len_r=max_usb_data_sz;
			fastboot_data(len_r);
			WaitForTransferComplete();
			while(len_r >= size_per_read)
			{
				Status = BlockIo->ReadBlocks (BlockIo,
				                         BlockIo->Media->MediaId,
				                         Offset+result,
				                         size_per_read,
				                         mDataBuffer);
			 	if (Status != EFI_SUCCESS)
				{	
					DEBUG ((EFI_D_ERROR, "partiton %s read error \n", PartitionName));
					FastbootFail("Partition read error, Please Input correctly size");
					return ;
				}
				len_r -= size_per_read;
				result += size_per_read / BlockIo->Media->BlockSize;
				DEBUG ((EFI_D_ERROR, "Send size=%d data\n", size_per_read));
				fastboot_send_data((void *)mDataBuffer,size_per_read);
				WaitForTransferComplete();
			}
			if(len_r>0)
			{
				Status = BlockIo->ReadBlocks (BlockIo,
				                         BlockIo->Media->MediaId,
				                         result,
				                         len_r,
				                         mDataBuffer);
			 	if (Status != EFI_SUCCESS)
				{	
					DEBUG ((EFI_D_ERROR, "partiton %s read error \n", PartitionName));
					FastbootFail("Partition read error, Please Input correctly size");
					return ;
				}
				
				if (remainder)
					len_r -= block_size - remainder;

				result += len_r / BlockIo->Media->BlockSize;
				DEBUG ((EFI_D_ERROR, "Send size=%d data\n", len_r));
				fastboot_send_data((void *)mDataBuffer, len_r);
				WaitForTransferComplete();

				if (Status != EFI_SUCCESS)
				{	
					DEBUG ((EFI_D_ERROR, "error\n"));
					return ;
				}
			}
			PartitionSize -= max_usb_data_sz;
		}
    	len_r=PartitionSize;
	}	

	if(len_r >= size_per_read)
	{
		flag=1;
		fastboot_data(len_r);
		WaitForTransferComplete();
	}
	while(len_r >= size_per_read)
	{
		Status = BlockIo->ReadBlocks (BlockIo,
                                 BlockIo->Media->MediaId,
                                 Offset+result,
                                 size_per_read,
                                 mDataBuffer);
	 	if (Status != EFI_SUCCESS)
		{	
			DEBUG ((EFI_D_ERROR, "partiton %s read error \n", PartitionName));
			FastbootFail("Partition read error, Please Input correctly size");
			return ;
		}
		len_r -= size_per_read;
		result += size_per_read / BlockIo->Media->BlockSize;
		DEBUG ((EFI_D_ERROR, "Send size=%d data\n", size_per_read));
		fastboot_send_data((void *)mDataBuffer,size_per_read);
		WaitForTransferComplete();
	}

    if(len_r>0)
	{
		Status = BlockIo->ReadBlocks (BlockIo,
                                 BlockIo->Media->MediaId,
                                 result,
                                 len_r,
                                 mDataBuffer);
	 	if (Status != EFI_SUCCESS)
		{	
			DEBUG ((EFI_D_ERROR, "partiton %s read errors%d \n", PartitionName,len_r));
			FastbootFail("Partition read error, Please Input correctly size");
			return ;
		}

		if (remainder)
			len_r -= block_size - remainder;
		if(flag == 0)
		{
			fastboot_data(len_r);
			WaitForTransferComplete();
		}
		DEBUG ((EFI_D_ERROR, "Send size=%d data\n", len_r));
		fastboot_send_data((void *)mDataBuffer, len_r);
		WaitForTransferComplete();
	}
	DEBUG ((EFI_D_ERROR, "cmd success\n"));
    FastbootOkay("");
}

// +++ ASUS_BSP : add for enter emergency download mode cmd
STATIC VOID CmdOemEnterDLoadMode(CONST CHAR8 *arg, VOID *data, UINT32 sz)
{
	if(!ASUS_FASTBOOT_PERMISSION)
	{
		FastbootFail("permission denied");
		return;
	}
	
	DEBUG((EFI_D_INFO, "[ABL] Rebooting the device into emergency dload mode\n"));
	FastbootOkay("");
	RebootDevice(EMERGENCY_DLOAD);

	// Shouldn't get here
	FastbootFail("Failed to reboot");
}
// --- ASUS_BSP : add for enter emergency download mode cmd

// +++ ASUS_BSP : add for logcat-asdf sevices
STATIC VOID CmdOemLogcatAsdfOn(CONST CHAR8 * arg, VOID * data, UINT32 sz)
{
    DEBUG((EFI_D_ERROR, "[ABL] +++ CmdOemLogcatAsdfOn()\n"));
    SetLogcatAsdfOn(TRUE);
    FastbootOkay("");
}

STATIC VOID CmdOemLogcatAsdfOff(CONST CHAR8 * arg, VOID * data, UINT32 sz)
{
    DEBUG((EFI_D_ERROR, "[ABL] +++ CmdOemLogcatAsdfOff()\n"));
    SetLogcatAsdfOn(FALSE);
    FastbootOkay("");
}
// --- ASUS_BSP : add for logcat-asdf sevices

// +++ ASUS_BSP : add for set permissive cmdline
STATIC VOID CmdOemSetPermissive(CONST CHAR8 *arg, VOID *data, UINT32 sz)
{
	CHAR8 DeviceInfo[MAX_RSP_SIZE];

#ifndef ABL_FTM
	DEBUG((EFI_D_INFO, "[ABL] Rebooting the device into permissive mode\n"));

	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "Rebooting the device into permissive mode now!\n");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	FastbootOkay("");
	RebootDevice(SET_PERMISSIVE_MODE);

	// Shouldn't get here
	FastbootFail("Failed to reboot");
#else
	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "FTM mode, Already set permissive!\n");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	FastbootOkay("");
#endif
}
// --- ASUS_BSP : add for set permissive cmdline

// +++ ASUS_BSP : add for wipe-data by recovery and enter fastboot mode
STATIC VOID CmdOemFactoryReset2(CONST CHAR8 *arg, VOID *data, UINT32 sz)
{
	EFI_STATUS Status;

	DEBUG((EFI_D_INFO, "Wipe-data and rebooting the device into fastboot mode \n"));

	Status = SetOemFactoryReset2Flag();
	if (Status != EFI_SUCCESS)
	{
		FastbootFail("Write FactoryReset2Flag deviceinfo fail");
	}

	Status = FactoryResetFromRecovery();

	if (Status != EFI_SUCCESS)
	{
		FastbootFail("Write misc partition fail");
	}
	else
	{
		FastbootOkay("");
		RebootDevice(RECOVERY_MODE);
	}

}
// --- ASUS_BSP : add for wipe-data by recovery and enter fastboot mode

// +++ ASUS_BSP : add for wipe-data by recovery
STATIC VOID CmdOemFactoryReset(CONST CHAR8 *arg, VOID *data, UINT32 sz)
{
	EFI_STATUS Status = EFI_SUCCESS;

	DEBUG((EFI_D_INFO, "[ABL] Rebooting the device into recovery mode and wipe-data\n"));

	Status = FactoryResetFromRecovery();

	if (Status != EFI_SUCCESS)
	{
		FastbootFail("[ABL] Write misc partition fail");
	}
	else
	{
		FastbootOkay("");
		RebootDevice(RECOVERY_MODE);
	}

	// Shouldn't get here
	FastbootFail ("[ABL] Failed to reboot");
}
// --- ASUS_BSP : add for wipe-data by recovery

// +++ ASUS_BSP : add for get BootCount
STATIC VOID CmdOemGetBootCount(CONST CHAR8 * arg, VOID * data, UINT32 sz)
{
	CHAR8 DeviceInfo[MAX_RSP_SIZE];
	DEBUG((EFI_D_ERROR, "[ABL] +++ CmdOemGetBootCount()\n"));
	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "[ABL] BootCount = %d", GetBootCounter());
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();
	DEBUG((EFI_D_ERROR, "[ABL]  --- CmdOemGetBootCount()\n"));
	FastbootOkay("");
}
// --- ASUS_BSP : add for get BootCount

// +++ ASUS_BSP : add for get bat vol
STATIC VOID CmdOemGetBatVol(CONST CHAR8 * arg, VOID * data, UINT32 sz)
{
	CHAR8 DeviceInfo[MAX_RSP_SIZE];
	UINT32 BatteryVoltage = 0;

	DEBUG((EFI_D_ERROR, "[ABL] +++ CmdOemGetBatVol()\n"));

	if(!TargetBatterySocOk(&BatteryVoltage))
	{
		FastbootFail ("Failed to get bat vol");
	}
	else {
		AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "bat vol = %d mV", BatteryVoltage);
		FastbootInfo(DeviceInfo);
		WaitForTransferComplete();
		FastbootOkay("");
	}

	DEBUG((EFI_D_ERROR, "[ABL] --- CmdOemGetBatVol()\n"));
}
// --- ASUS_BSP : add for get bat vol

// +++ ASUS_BSP : add for get bat cap
STATIC VOID CmdOemGetBatCap(CONST CHAR8 * arg, VOID * data, UINT32 sz)
{
	DEBUG((EFI_D_ERROR, "[ABL] +++ CmdOemGetBatCap()\n"));

#if 0
	CHAR8 DeviceInfo[MAX_RSP_SIZE];
	UINT32 BatteryVoltage = 0;
	UINT32 BatteryCap = 0;
	
	if(!TargetBatterySocOk(&BatteryVoltage))
	{
		FastbootFail ("Failed to get bat vol");
	}
	else {
		//Scale to 99%
		BatteryCap = (BatteryVoltage - FG_STUB_BATT_VOLTAGE_MIN_MV_2S) * 99;
		//Rounding
		BatteryCap = (BatteryCap + (FG_STUB_CHARGE_VOLTAGE_MAX_MV_2S - FG_STUB_BATT_VOLTAGE_MIN_MV_2S)/2)/(FG_STUB_CHARGE_VOLTAGE_MAX_MV_2S - FG_STUB_BATT_VOLTAGE_MIN_MV_2S);
		AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "bat cap = %d%%", BatteryCap);
		FastbootInfo(DeviceInfo);
		WaitForTransferComplete();
		FastbootOkay("");
	}
#endif

	DEBUG((EFI_D_ERROR, "[ABL] --- CmdOemGetBatCap()\n"));
}
// --- ASUS_BSP : add for get bat cap

// +++ ASUS_BSP : add for get build version
STATIC VOID CmdOemGetBuildVersion(CONST CHAR8 * arg, VOID * data, UINT32 sz)
{
	CHAR8 DeviceInfo[MAX_RSP_SIZE];
	sys_info sysinfo;

	DEBUG((EFI_D_ERROR, "[ABL] +++ CmdOemGetBuildVersion()\n"));

	if(read_sysinfo(&sysinfo) == EFI_SUCCESS)
	{
		AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "BUILD_VERION = %a", sysinfo.csc_build_version);
		FastbootInfo(DeviceInfo);
		WaitForTransferComplete();
		FastbootOkay("");
	}
	else
	{
		FastbootFail("Failed to get build version");
	}

	DEBUG((EFI_D_ERROR, "[ABL] --- CmdOemGetBuildVersion()\n"));

}
// --- ASUS_BSP : add for get build version

// +++ ASUS_BSP : add for system info
STATIC VOID CmdOemSystemInfo(CONST CHAR8 * arg, VOID * data, UINT32 sz)
{
	CHAR8 DeviceInfo[MAX_RSP_SIZE];
	sys_info sysinfo;

	DEBUG((EFI_D_ERROR, "[ABL] +++ CmdOemSystemInfo()\n"));

	if(read_sysinfo(&sysinfo) == EFI_SUCCESS)
	{
		AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " board_info: %a", sysinfo.board_info);
		FastbootInfo(DeviceInfo);
		WaitForTransferComplete();

		AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " mem_info: %a", sysinfo.mem_info);
		FastbootInfo(DeviceInfo);
		WaitForTransferComplete();

		AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " cpu_freq: %a", sysinfo.cpu_freq);
		FastbootInfo(DeviceInfo);
		WaitForTransferComplete();

		AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " product_name: %a", sysinfo.product_name);
		FastbootInfo(DeviceInfo);
		WaitForTransferComplete();
/*
		AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " product_locale: %a", sysinfo.product_locale);
		FastbootInfo(DeviceInfo);
		WaitForTransferComplete();
*/
		AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " product_carrier: %a", sysinfo.product_carrier);
		FastbootInfo(DeviceInfo);
		WaitForTransferComplete();

		AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " csc_build_version: %a", sysinfo.csc_build_version);
		FastbootInfo(DeviceInfo);
		WaitForTransferComplete();

		AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " bt_mac: %a", sysinfo.bt_mac);
		FastbootInfo(DeviceInfo);
		WaitForTransferComplete();

		AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " wifi_mac: %a", sysinfo.wifi_mac);
		FastbootInfo(DeviceInfo);
		WaitForTransferComplete();

		AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " wifi_mac_2: %a", sysinfo.wifi_mac_2);
		FastbootInfo(DeviceInfo);
		WaitForTransferComplete();

		AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " imei: %a", sysinfo.imei);
		FastbootInfo(DeviceInfo);
		WaitForTransferComplete();

		AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " imei2: %a", sysinfo.imei2);
		FastbootInfo(DeviceInfo);
		WaitForTransferComplete();

		AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " ssn: %a", sysinfo.ssn);
		FastbootInfo(DeviceInfo);
		WaitForTransferComplete();

		AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " isn: %a", sysinfo.isn);
		FastbootInfo(DeviceInfo);
		WaitForTransferComplete();

		AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " color: %a", sysinfo.color);
		FastbootInfo(DeviceInfo);
		WaitForTransferComplete();
/*
		AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " versatility: %a", sysinfo.versatility);
		FastbootInfo(DeviceInfo);
		WaitForTransferComplete();
*/
		AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " country: %a", sysinfo.country);
		FastbootInfo(DeviceInfo);
		WaitForTransferComplete();

		AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " customer: %a", sysinfo.customer);
		FastbootInfo(DeviceInfo);
		WaitForTransferComplete();

		AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " ufs: %a", sysinfo.ufs_info);
		FastbootInfo(DeviceInfo);
		WaitForTransferComplete();

		AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " setupwizard: %a", sysinfo.setupwizard);
		FastbootInfo(DeviceInfo);
		WaitForTransferComplete();

		AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " toolid_check: %a", sysinfo.toolid_check);
		FastbootInfo(DeviceInfo);
		WaitForTransferComplete();

		FastbootOkay("");
	}
	else
	{
		FastbootFail("Failed to read sysinfo");
	}

	DEBUG((EFI_D_ERROR, "[ABL] --- CmdOemSystemInfo()\n"));
}
// --- ASUS_BSP : add for system info

// +++ ASUS_BSP : add for ssn info
STATIC VOID CmdOemSsnInfo(CONST CHAR8 * arg, VOID * data, UINT32 sz)
{
	CHAR8 NowChar[MAX_RSP_SIZE];
	CHAR8 DeviceInfo[MAX_RSP_SIZE];

	DEBUG((EFI_D_ERROR, "[ABL] +++ CmdOemSsnInfo()\n"));
	GetSSNNum(NowChar, sizeof(NowChar));
	DEBUG((EFI_D_ERROR, "[ABL] SSN_NUM = %a\n", NowChar));
	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "SSN_NUM = %a", NowChar);
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();
	DEBUG((EFI_D_ERROR, "[ABL] --- CmdOemSsnInfo()\n"));

	FastbootOkay("");
}
// --- ASUS_BSP : add for ssn info

// +++ ASUS_BSP : add for isn info
STATIC VOID CmdOemIsnInfo(CONST CHAR8 * arg, VOID * data, UINT32 sz)
{
	CHAR8 NowChar[MAX_RSP_SIZE];
	CHAR8 DeviceInfo[MAX_RSP_SIZE];

	DEBUG((EFI_D_ERROR, "[ABL] +++ CmdOemIsnInfo()\n"));
	GetISNNum(NowChar, sizeof(NowChar));
	DEBUG((EFI_D_ERROR, "[ABL] ISN_NUM = %a\n", NowChar));
	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "ISN_NUM = %a", NowChar);
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();
	DEBUG((EFI_D_ERROR, "[ABL] --- CmdOemIsnInfo()\n"));

	FastbootOkay("");
}
// --- ASUS_BSP : add for isn info

// +++ ASUS_BSP : add for read TOOLID
STATIC VOID CmdOemGetToolId(CONST CHAR8 * arg, VOID * data, UINT32 sz)
{
	CHAR8 NowChar[MAX_RSP_SIZE];
	CHAR8 DeviceInfo[MAX_RSP_SIZE];

	DEBUG((EFI_D_ERROR, "[ABL] +++ CmdOemGetToolId()\n"));
	GetTOOLID(NowChar, sizeof(NowChar));
	DEBUG((EFI_D_ERROR, "[ABL] TOOLID = %a\n", NowChar));
	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "TOOLID = %a", NowChar);
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();
	DEBUG((EFI_D_ERROR, "[ABL] --- CmdOemGetToolId()\n"));

	FastbootOkay("");
}
// --- ASUS_BSP : add for read TOOLID

// +++ ASUS_BSP : add for get cpuid hash
STATIC VOID CmdOemGetCpuidHash(CONST CHAR8 * arg, VOID * data, UINT32 sz)
{
	CHAR8 NowChar[MAX_RSP_SIZE];
	CHAR8 DeviceInfo[MAX_RSP_SIZE];

	DEBUG((EFI_D_ERROR, "[ABL] +++ CmdOemGetCpuidHash()\n"));
	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "CPUID = 0x%lx", Get_CPU_ID());

	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();
	GetCpuidHash(NowChar, sizeof(NowChar));
	DEBUG((EFI_D_ERROR, "[ABL] CPUID HASH = %a\n", NowChar));
	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "CPUID HASH = %a", NowChar);
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();
	DEBUG((EFI_D_ERROR, "[ABL] --- CmdOemGetCpuidHash()\n"));
	FastbootOkay("");
}
// --- ASUS_BSP :  add for get cpuid hash

// +++ ASUS_BSP :  Add for get LGF ID
STATIC VOID CmdOemGetLGFID(CONST CHAR8 * arg, VOID * data, UINT32 sz)
{
	CHAR8 DeviceInfo[MAX_RSP_SIZE];

	DEBUG((EFI_D_ERROR, "[ABL] +++ CmdOemGetLGFID()\n"));
	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "LGF ID = %x", Get_LGF_ID());
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "=== ID Info List ===");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "-1 : UNKNOWN");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();
	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " 0 : Entry");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();
	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " 1 : Pro");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();
	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " 2 : Ultimate");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "UNKNOWN");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	DEBUG((EFI_D_ERROR, "[ABL] --- CmdOemGetLGFID()\n"));
	FastbootOkay("");
}
// --- ASUS_BSP :  Add for get BCID

// +++ ASUS_BSP :  Add for get JTAGID
STATIC VOID CmdOemGetJTAGID(CONST CHAR8 * arg, VOID * data, UINT32 sz)
{
	CHAR8 DeviceInfo[MAX_RSP_SIZE];
	DEBUG((EFI_D_ERROR, "[ABL] +++ CmdOemGetJTAGID()\n"));

	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "JTAG ID = %x (0x%x)", (Get_JTAG_ID() & 0xF000) >> 12, Get_JTAG_ID());
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "=== ID Info List ===");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();
	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "SM8550:0x1870E1");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();
	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "SM8550P:0x1880E1");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	DEBUG((EFI_D_ERROR, "[ABL] --- CmdOemGetJTAGID()\n"));
	FastbootOkay("");
}
// --- ASUS_BSP :  Add for get JTAGID

// +++ ASUS_BSP : Add for get FEATUREID
STATIC VOID CmdOemGetFeatureID(CONST CHAR8 * arg, VOID * data, UINT32 sz)
{
	CHAR8 DeviceInfo[MAX_RSP_SIZE];
	DEBUG((EFI_D_ERROR, "[ABL] +++ CmdOemGetFeatureID()\n"));


	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "FeatureID = %x", Get_FEATURE_ID());
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "=== ID Info List ===");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "0x0 002-AB-SM8550_ES2");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();
	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "0x6 001-AB-SM8550_ES1");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();
	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "0x8 002-AB-SM8550P_ES");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	DEBUG((EFI_D_ERROR, "[ABL] --- CmdOemGetFeatureID()\n"));
	FastbootOkay("");
}
// --- ASUS_BSP : Add for get FEATUREID

// +++ ASUS_BSP : Add for get RFID
STATIC VOID CmdOemGetRFID(CONST CHAR8 * arg, VOID * data, UINT32 sz)
{
	CHAR8 DeviceInfo[MAX_RSP_SIZE];
	DEBUG((EFI_D_ERROR, "[ABL] +++ CmdOemGetRFID()\n"));

	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "RFID = %d", Get_RF_ID());
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "=== ID Info List ===");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "-1 : UNKNOWN");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();
	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " 0 : WW_H");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();
	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " 1 : WW_L");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();
	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " 2 : CN");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();
	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " 3 : no RF board");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	DEBUG((EFI_D_ERROR, "[ABL] --- CmdOemGetRFID()\n"));
	FastbootOkay("");
}
// --- ASUS_BSP : Add for get RFID

// +++ ASUS_BSP : Add for get SKUID
STATIC VOID CmdOemGetSKUID(CONST CHAR8 * arg, VOID * data, UINT32 sz)
{
	CHAR8 DeviceInfo[MAX_RSP_SIZE];
	DEBUG((EFI_D_ERROR, "[ABL] +++ CmdOemGetSKUID()\n"));

	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "SKUID = %d", Get_SKU_ID());
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "=== ID Info List ===");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "-1 : UNKNOWN");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();
	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " 0 : UFS3.1/128GB");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();
	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " 1 : UFS3.1/256GB");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();
	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " 2 : UFS3.1/512GB");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();
	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " 3 : UFS3.1/1TB");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();
	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " 4");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();
	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " 5");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();
	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " 6");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();
	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " 7");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	DEBUG((EFI_D_ERROR, "[ABL] --- CmdOemGetSKUID()\n"));
	FastbootOkay("");
}
// --- ASUS_BSP : Add for get SKUID

// +++ ASUS_BSP : Add for get PRJID
STATIC VOID CmdOemGetPRJID(CONST CHAR8 * arg, VOID * data, UINT32 sz)
{
	CHAR8 DeviceInfo[MAX_RSP_SIZE];
	DEBUG((EFI_D_ERROR, "[ABL] +++ CmdOemGetPRJID()\n"));

	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "PRJID = %d \n", Get_PJ_ID());
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "=== ID Info List ===");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "-1 : UNKNOWN");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();
	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " 0 : Entry");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();
	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " 1 : Pro");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();
	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " 2 : Ultimate");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	DEBUG((EFI_D_ERROR, "[ABL] --- CmdOemGetPRJID()\n"));
	FastbootOkay("");
}
// --- ASUS_BSP : Add for get PRJID

// +++ ASUS_BSP : Add for get HWID
STATIC VOID CmdOemGetHWID(CONST CHAR8 * arg, VOID * data, UINT32 sz)
{
	CHAR8 DeviceInfo[MAX_RSP_SIZE];
	DEBUG((EFI_D_ERROR, "[ABL] +++ CmdOemGetHWID()\n"));

	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "HWID = %d \n", Get_HW_ID());
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "=== ID Info List ===");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "-1 : UNKNOWN");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();
	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " 0xD : EVB");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();
	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " 0x1 : SR1");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();
	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " 0x2 : SR2");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();
	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " 0x4 : ER1");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();
	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " 0x8 : ER2");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();
	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " 0x9 : PR");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();
	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " 0xB : MP");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	DEBUG((EFI_D_ERROR, "[ABL] --- CmdOemGetHWID()\n"));
	FastbootOkay("");
}
// --- ASUS_BSP : Add for get HWID

// +++ ASUS_BSP : Add for get DTID
STATIC VOID CmdOemGetDTID(CONST CHAR8 * arg, VOID * data, UINT32 sz)
{
	CHAR8 DeviceInfo[MAX_RSP_SIZE];
	DEBUG((EFI_D_ERROR, "[ABL] +++ CmdOemGetDTID()\n"));
	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "DTID = %d\n", GetDeviceTreeID());
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "=== ID Info List ===");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " -1 : UNKNOWN");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();
	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " 30 : EVB");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();
	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " 40 : SR");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();
	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " 50 : TBD");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();
	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " 60 : TBD");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();
	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " 70 : TBD");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();
	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " 80 : TBD");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();
	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), " 90 : TBD");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();
	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "100 : TBD");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();


	DEBUG((EFI_D_ERROR, "[ABL] --- CmdOemGetDTID()\n"));
	FastbootOkay("");
}
// --- ASUS_BSP : Add for get DTID

// +++ ASUS_BSP : add for oem device shutdown
STATIC VOID CmdOemShutDown()
{
	EFI_STATUS Status = EFI_SUCCESS;
	BOOLEAN ChargerPresent = TRUE;
	CHAR8 DeviceInfo[MAX_RSP_SIZE];
	UINT8 plug_value = 0;
	UINT8 pull_value = 0;

	DEBUG((EFI_D_ERROR, "[ABL] +++ CmdOemShutDown()\n"));

	Status = GetPmicReg(0x7, 0x2910, &plug_value);

	if (Status != EFI_SUCCESS) {
		FastbootFail ("Failed to write pmic reg value");
	}
	else {
		DEBUG((EFI_D_ERROR, "[ABL] ASUS_ShutdownDevice() : Status=%r, plug_value=%x\n", Status, plug_value));
		AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "call CmdOemShutDown()");
		FastbootInfo(DeviceInfo);
		WaitForTransferComplete();
		FastbootOkay("");
	}

	while(ChargerPresent == TRUE){
		GetPmicReg(0x7, 0x2910, &pull_value);
		if(plug_value != pull_value) {
			//DEBUG((EFI_D_ERROR, "[ABL] ASUS_ShutdownDevice() : Status=%r, pull_value=%x\n", Status, pull_value));
			ChargerPresent = FALSE;
			//DEBUG((EFI_D_ERROR, "waitting for cable out ....\n"));
			MicroSecondDelay(3000000);
		}
	}

	DEBUG((EFI_D_ERROR, "[ABL] --- CmdOemShutDown()\n"));
	ShutdownDevice();
	// Shouldn't get here
	FastbootFail("Failed to shutdown");
}
// --- ASUS_BSP : add for oem device shutdown

// +++ ASUS_BSP : add for adb enable
STATIC VOID CmdEnableAdb(CONST CHAR8 *arg, VOID *data, UINT32 sz)
{
	DEBUG((EFI_D_INFO, "Rebooting the device into adb mode\n"));
	FastbootOkay("");
	RebootDevice(ENABLE_ADB_MODE);

	// Shouldn't get here
	FastbootFail("Failed to reboot");
}
// --- ASUS_BSP : add for adb enable

// +++ ASUS_BSP : add for gpt-info
STATIC VOID CmdOemGptinfo(CONST CHAR8 *arg, VOID *data, UINT32 sz)
{
	CHAR8                    DeviceInfo[MAX_RSP_SIZE];
	EFI_STATUS               Status;
	CHAR8                    PartitionNameAscii[MAX_GPT_NAME_SIZE];
	EFI_PARTITION_ENTRY     *PartEntry;
	UINT16                   i;
	UINT32                   j;
	/* By default the LunStart and LunEnd would point to '0' and max value */
	UINT32 LunStart = 0;
	UINT32 LunEnd = GetMaxLuns();

	UINT32 Priority_Value = 0;
	UINT32 ActiveValue =0;
	UINT32 RetryCount =0;
	UINT32 Success_Value=0;
	UINT32 Unbootable_Value=0;

	DEBUG((EFI_D_ERROR, "[ABL] +++ PartitionDump : LunEnd = %d\n", LunEnd));

	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "/////////////////// GPT INFO ///////////////////");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "UFS MAX LUN : %d", LunEnd);
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	/* If Lun is set in the Handle flash command then find the block io for that lun */
	if (LunSet)
	{
		LunStart = Lun;
		LunEnd = Lun + 1;
	}

	for (i = LunStart; i < LunEnd; i++)
	{
		AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "/////////////////// LUN %d ///////////////////",i);
		FastbootInfo(DeviceInfo);
		WaitForTransferComplete();

		for (j = 0; j < Ptable[i].MaxHandles; j++)
		{
			Status = gBS->HandleProtocol(Ptable[i].HandleInfoList[j].Handle, &gEfiPartitionRecordGuid, (VOID **)&PartEntry);
			if (EFI_ERROR (Status))
			{
				DEBUG((EFI_D_VERBOSE, "Error getting the partition record for Lun %d and Handle: %d : %r\n", i, j,Status));
				continue;
			}

			UnicodeStrToAsciiStr(PartEntry->PartitionName, PartitionNameAscii);
			DEBUG((EFI_D_INFO, "LUN:[%d] PARTITION_NUM:[%d] Name:[%a] StartLba: %u EndLba:%u\n", i,j,PartitionNameAscii, PartEntry->StartingLBA, PartEntry->EndingLBA));

			AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "PARTITION_NUM : %d", j);
			FastbootInfo(DeviceInfo);
			WaitForTransferComplete();

			AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "PARTITION_NAME : %a", PartitionNameAscii);
			FastbootInfo(DeviceInfo);
			WaitForTransferComplete();

			Priority_Value= (PartEntry->Attributes & PART_ATT_PRIORITY_VAL) >> PART_ATT_PRIORITY_BIT;
			DEBUG((EFI_D_ERROR,"[AB] FindPtnActiveSlot : Priority_Value = %d\n",Priority_Value));

			ActiveValue = (PartEntry->Attributes & PART_ATT_ACTIVE_VAL) >> PART_ATT_ACTIVE_BIT;
			DEBUG((EFI_D_ERROR,"[AB] FindPtnActiveSlot : ActiveValue = %d\n",ActiveValue));

			RetryCount = (PartEntry->Attributes & PART_ATT_MAX_RETRY_COUNT_VAL) >> PART_ATT_MAX_RETRY_CNT_BIT;
			DEBUG((EFI_D_ERROR,"[AB] FindPtnActiveSlot : RetryCount = %llu\n",RetryCount));

			Success_Value= (PartEntry->Attributes & PART_ATT_SUCCESSFUL_VAL) >> PART_ATT_SUCCESS_BIT;
			DEBUG((EFI_D_ERROR,"[AB] FindPtnActiveSlot : Success_Value = %d\n",Success_Value));

			Unbootable_Value= (PartEntry->Attributes & PART_ATT_UNBOOTABLE_VAL) >> PART_ATT_UNBOOTABLE_BIT;
			DEBUG((EFI_D_ERROR,"[AB] FindPtnActiveSlot : Unbootable_Value = %d\n",Unbootable_Value));

			AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "-----------------------------");
			FastbootInfo(DeviceInfo);
			WaitForTransferComplete();

			AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "PARTITION ATTR.Priority: %d", Priority_Value);
			FastbootInfo(DeviceInfo);
			WaitForTransferComplete();

			AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "PARTITION ATTR.Active: %d", ActiveValue);
			FastbootInfo(DeviceInfo);
			WaitForTransferComplete();

			AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "PARTITION ATTR.RetryCount: %d", RetryCount);
			FastbootInfo(DeviceInfo);
			WaitForTransferComplete();

			AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "PARTITION ATTR.Success: %d", Success_Value);
			FastbootInfo(DeviceInfo);
			WaitForTransferComplete();

			AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "PARTITION ATTR.Unbootable: %d", Unbootable_Value);
			FastbootInfo(DeviceInfo);

			WaitForTransferComplete();
			AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "-----------------------------");
			FastbootInfo(DeviceInfo);
			WaitForTransferComplete();

			AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "START_LBA : %u", PartEntry->StartingLBA);
			FastbootInfo(DeviceInfo);
			WaitForTransferComplete();

			AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "END_LBA : %u", PartEntry->EndingLBA);
			FastbootInfo(DeviceInfo);
			WaitForTransferComplete();

			AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "PARTITION_SIZE (4K BLOCK): %u", ((PartEntry->EndingLBA)-(PartEntry->StartingLBA)+1));
			FastbootInfo(DeviceInfo);
			WaitForTransferComplete();

			AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "==================================");
			FastbootInfo(DeviceInfo);
			WaitForTransferComplete();

		}
	}
	DEBUG((EFI_D_ERROR, "[ABL] --- PartitionDump : LunEnd = %d\n", LunEnd));

	AsciiSPrint(DeviceInfo, sizeof(DeviceInfo), "///////////////////////////////////////////////");
	FastbootInfo(DeviceInfo);
	WaitForTransferComplete();

	FastbootOkay("");
}
// --- ASUS_BSP : add for gpt-info
////////////////////////////////////////////////////////////////////////

// +++ ASUS_BSP : add for xts unlock
STATIC VOID CmdOemXtsUnlock(){
	CHAR8 resp[MAX_RSP_SIZE];
	DEBUG((EFI_D_ERROR, "\n"));
	DEBUG((EFI_D_ERROR, "[ABL] +++ CmdOemXtsUnlock()\n"));
	
	// gen random numbers
	random_num_generator(rand_number);

	// gen hash numbers
	cmd_oem_gen_hash(rand_number, 1);

	// verify hash file
	if(AsciiStrnCmp(calculate_hash_buf, mmc_hash_buf, hash_buf_size) == 0)
	{
		DEBUG((EFI_D_ERROR,"[ABL] cmd_oem_auth_hash - Authorized hash : PASS \n"));

		SetAuthorizedValue(TRUE);

		if (!(IsAuthorized()))
		{
			DEBUG((EFI_D_ERROR, "[ABL] Unable to Write Device Info : IsAuthorized = %d\n", IsAuthorized()));
			FastbootFail("");
			return;
		}
		
		SetDeviceUnlockValue (UNLOCK, TRUE);
		if(!(IsUnlocked())){
			DEBUG((EFI_D_ERROR, "[ABL] Unable to Write Device Info : IsUnlocked = %d\n", IsUnlocked()));
			FastbootFail("");
			return;
		}

		AsciiSPrint(resp, sizeof(resp), "\tAuthorized Result : PASS");
		FastbootInfo(resp);
		WaitForTransferComplete();
	}
	else
	{
		DEBUG((EFI_D_ERROR, "[ABL] (ERROR) cmd_oem_auth_hash - Authorized hash : FAILED \n"));
		AsciiSPrint(resp, sizeof(resp), "\tAuthorized Result : FAILED");
		FastbootInfo(resp);
		WaitForTransferComplete();
	}

	FastbootOkay("");

	DEBUG((EFI_D_ERROR, "[ABL] --- CmdOemXtsUnlock()\n"));
	DEBUG((EFI_D_ERROR, "\n"));
}

// +++ ASUS_BSP : add for lock device
STATIC VOID
CmdOemLock (CONST CHAR8 *Arg, VOID *Data, UINT32 Size)
{
    CHAR16 Cmdinfo[32]={0};
    CHAR8 CmdlockStr[64]={0};
    
    // +++ ASUS_BSP : add for fastboot permission
    if(!ASUS_FASTBOOT_PERMISSION)
    {
        FastbootFail("permission denied");
        return;
    }

    if (Arg == NULL) {
        FastbootFail("Invalid parameter !");
        return;
    }

    AsciiStrToUnicodeStr((CHAR8 *)Arg, Cmdinfo);
    DEBUG((EFI_D_ERROR, "[ABL] +++ CmdOemLock() : Cmdinfo=%s\n", Cmdinfo));
   
	FastbootInfo("Try to lock device");
	WaitForTransferComplete();

	SetAuthorizedValue(FALSE);
	SetDeviceUnlockValue (UNLOCK, FALSE);
	SetDeviceDebugUnlockValue(FALSE);
	
	if(!IsUnlocked() && !IsDebugUnlocked() && !IsAuthorized()){
		AsciiSPrint(CmdlockStr, sizeof(CmdlockStr), "Lock device successfully.");
	}else{
		AsciiSPrint(CmdlockStr, sizeof(CmdlockStr), "Lock device failed.");
	}
	
	FastbootInfo(CmdlockStr);
	WaitForTransferComplete();
	
	FastbootOkay("");
	DEBUG((EFI_D_ERROR, "[ABL] --- CmdOemLock()\n"));
}

#endif

/* Registers all Stock commands, Publishes all stock variables
 * and partitiion sizes. base and size are the respective parameters
 * to the Fastboot Buffer used to store the downloaded image for flashing
 */
STATIC EFI_STATUS
FastbootCommandSetup (IN VOID *Base, IN UINT64 Size)
{
  EFI_STATUS Status;
  CHAR8 HWPlatformBuf[MAX_RSP_SIZE] = "\0";
  CHAR8 DeviceType[MAX_RSP_SIZE] = "\0";
  BOOLEAN BatterySocOk = FALSE;
  UINT32 BatteryVoltage = 0;
  UINT32 PartitionCount = 0;
  BOOLEAN MultiSlotBoot = PartitionHasMultiSlot ((CONST CHAR16 *)L"boot");
  MemCardType Type = UNKNOWN;
  VirtualAbMergeStatus SnapshotMergeStatus;

  mDataBuffer = Base;
  mNumDataBytes = Size;
  mFlashNumDataBytes = Size;
  mUsbDataBuffer = Base;

  mFlashDataBuffer = (CheckRootDeviceType () == NAND) ?
                           Base : (Base + MaxDownLoadSize);

  /* Find all Software Partitions in the User Partition */
  UINT32 i;
  UINT32 BlkSize = 0;
  DeviceInfo *DevInfoPtr = NULL;

  struct FastbootCmdDesc cmd_list[] = {
      /* By Default enable list is empty */
      {"", NULL},
/*CAUTION(High): Enabling these commands will allow changing the partitions
 *like system,userdata,cachec etc...
 */
#ifdef ENABLE_UPDATE_PARTITIONS_CMDS
      {"flash:", CmdFlash},
      {"erase:", CmdErase},
      {"set_active", CmdSetActive},
      {"flashing get_unlock_ability", CmdFlashingGetUnlockAbility},
      {"flashing unlock", CmdFlashingUnlock},
      {"flashing lock", CmdFlashingLock},
#endif
/*
 *CAUTION(CRITICAL): Enabling these commands will allow changes to bootimage.
 */
#ifdef ENABLE_DEVICE_CRITICAL_LOCK_UNLOCK_CMDS
      {"flashing unlock_critical", CmdFlashingUnLockCritical},
      {"flashing lock_critical", CmdFlashingLockCritical},
#endif
/*
 *CAUTION(CRITICAL): Enabling this command will allow boot with different
 *bootimage.
 */
#ifdef ENABLE_BOOT_CMD
      {"boot", CmdBoot},
#endif
      {"oem enable-charger-screen", CmdOemEnableChargerScreen},
      {"oem disable-charger-screen", CmdOemDisableChargerScreen},
      {"oem off-mode-charge", CmdOemOffModeCharger},
      {"oem select-display-panel", CmdOemSelectDisplayPanel},
      {"oem set-hw-fence-value", CmdOemSetHwFenceValue},
      {"oem device-info", CmdOemDevinfo},
#if HIBERNATION_SUPPORT_NO_AES
      {"oem golden-snapshot", CmdGoldenSnapshot},
#endif
      {"continue", CmdContinue},
      {"reboot", CmdReboot},
      {"reboot-bootloader", CmdRebootBootloader},
      {"getvar:", CmdGetVar},
      {"download:", CmdDownload},
#ifdef ASUS_BUILD
      {"oem partition", CmdPartition},
      {"oem gpt-info", CmdOemGptinfo},// +++ ASUS_BSP : add for gpt-info
      {"oem adb_enable", CmdEnableAdb},// +++ ASUS_BSP : add for adb enable
      {"oem logcat-asdf-on", CmdOemLogcatAsdfOn},// +++ ASUS_BSP : add for logcat-asdf sevices
      {"oem logcat-asdf-off", CmdOemLogcatAsdfOff},// +++ ASUS_BSP : add for logcat-asdf sevices
      {"oem shutdown", CmdOemShutDown},// +++ ASUS_BSP : add for oem device shutdown
      {"oem get-dtid", CmdOemGetDTID},// +++ ASUS_BSP : Add for get DTID
      {"oem get-hwid", CmdOemGetHWID},// +++ ASUS_BSP : Add for get HWID
      {"oem get-prjid", CmdOemGetPRJID},// +++ ASUS_BSP : Add for get PRJID
      {"oem get-skuid", CmdOemGetSKUID},// +++ ASUS_BSP : Add for get SKUID
      {"oem get-rfid", CmdOemGetRFID},// +++ ASUS_BSP : Add for get RFID
      {"oem get-featureid", CmdOemGetFeatureID},// +++ ASUS_BSP : Add for get FEATUREID
      {"oem get-jtagid", CmdOemGetJTAGID},// +++ ASUS_BSP : Add for get JTAGID
      {"oem get-lgfid", CmdOemGetLGFID},// +++ ASUS_BSP : Add for get LGF ID
      {"oem get-cpuidhash", CmdOemGetCpuidHash},// +++ ASUS_BSP : add for get cpuid hash
      {"oem get-toolid", CmdOemGetToolId},// +++ ASUS_BSP : add for read TOOLID
      {"oem isn-info", CmdOemIsnInfo},// +++ ASUS_BSP : add for isn info
      {"oem ssn-info", CmdOemSsnInfo},// +++ ASUS_BSP : add for ssn info
      {"oem system-info", CmdOemSystemInfo},// +++ ASUS_BSP : add for system info
      {"oem get_build_version", CmdOemGetBuildVersion},// +++ ASUS_BSP : add for get build version
      {"oem get-batcap", CmdOemGetBatCap},// +++ ASUS_BSP : add for get bat cap
      {"oem get-batvol", CmdOemGetBatVol},// +++ ASUS_BSP : add for get bat vol
      {"oem get-bootcount", CmdOemGetBootCount},// +++ ASUS_BSP : add for boot count
      {"oem factory-reset", CmdOemFactoryReset},// +++ ASUS_BSP : add for wipe-data by recovery
      {"oem factory-reset2", CmdOemFactoryReset2},// +++ ASUS_BSP : add for wipe-data by recovery and enter fastboot mode
      {"oem reboot-recovery", CmdRebootRecovery},// +++ ASUS_BSP : add for reboot recovery
      {"oem set-permissive", CmdOemSetPermissive},// +++ ASUS_BSP : add for set permissive cmdline
      {"oem enter-dload", CmdOemEnterDLoadMode},// +++ ASUS_BSP : add for enter emergency download mode cmd
      {"oem check-s3", CmdOemCheckS3},// +++ ASUS_BSP : add for check s3 reset type cmd
      {"oem EnterShippingMode", CmdOemEnterShippingMode},// +++ ASUS_BSP : add for shipping mode
      {"oem check-nrfuse", CmdOemCheckFuseNoRpmb},// +++ ASUS_BSP : add for check fuse with no rpmb
      {"oem check-fuse", CmdOemCheckFuse},//+++ ASUS_BSP : add for check fuse
      {"oem fuse-info", CmdOemFuseInfo},//+++ ASUS_BSP : add for get fuse info
      {"oem show-barcode", CmdOemDrawBarcode},// +++ ASUS_BSP : add for draw barcode
      {"oem checksetupwizard", CmdOemCheckSetupWizard},// +++ ASUS_BSP : add for Check Setup Wizard
      {"oem asus-csc_lk", CmdOemAsusCscLk},// +++ ASUS_BSP : add for oem ASUS CSC lock device
      {"oem rsa_test_", CmdRsaTest},// +++ ASUS_BSP : add for user unlock
      {"oem crc32_", CmdOemCalculatePtCrc32},// +++ ASUS_BSP : add for get partition hash
      {"oem hash_", CmdOemCalculatePtCrc32},// +++ ASUS_BSP : add for get partition hash
      {"oem gen-random", CmdOemGenRNG},// +++ ASUS_BSP : add for ASUS dongle unlock
      {"oem auth-hash", CmdOemAuthHash},// +++ ASUS_BSP : add for ASUS dongle unlock
      {"oem auth-hash_2", CmdOemAuthHash2},// +++ ASUS_BSP : add for ASUS dongle unlock
      {"oem auth-hash_3", CmdOemAuthHash3},// +++ ASUS_BSP : add for WaterMask unlock
      {"oem get-imeiauth", CmdOemGetIMEIAuth},// +++ ASUS_BSP : add for WaterMask unlock
      {"oem slot_b_enable", CmdOemSlotbEnable},// +++ ASUS_BSP : add for enable flash raw in slot_b
      {"oem get-verify_vbmeta_ret", CmdOemGetVerifyVbmetaRet},// +++ ASUS_BSP : add for verify_vbmeta_ret
      {"oem update-cmdline_", CmdOemUpdateCmdline},// +++ ASUS_BSP : add for update cmdline
      {"oem backup-fac",CmdOemBackupFac},// +++ ASUS_BSP : add for backup factory data
      {"oem restore-fac",CmdOemRestoreFac},// +++ ASUS_BSP : add for restore factory data
      {"oem record-info", CmdOemRecordInfo},// +++ ASUS_BSP : falling & hit & thump record info
      {"oem asus-erase-asdf", CmdOemFEASDF},
      // +++ ASUS_BSP : Need ASUS_FASTBOOT_PERMISSION
      {"oem get-pmic-reg_", CmdOemGetPmicReg},// +++ ASUS_BSP : add for get_pmic_reg value
      {"oem write-pmic-reg_", CmdOemWritePmicReg},// +++ ASUS_BSP : add for write_pmic_reg value
      {"oem reset-boot_count", CmdOemResetBootCount},// +++ ASUS_BSP : add for boot count
      {"oem reset-lock_count", CmdOemResetLockCount},// +++ ASUS_BSP : add for lock count
      {"oem reset-a_retry_count", CmdOemResetSlotARetryCount},// +++ ASUS_BSP : add for unbootable_counter and retry_counter
      {"oem reset-a_unbootable_count", CmdOemResetSlotAUnbootableCount},// +++ ASUS_BSP : add for unbootable_counter and retry_counter
      {"oem reset-b_retry_count", CmdOemResetSlotBRetryCount},// +++ ASUS_BSP : add for unbootable_counter and retry_counter
      {"oem reset-b_unbootable_count", CmdOemResetSlotBUnbootableCount},// +++ ASUS_BSP : add for unbootable_counter and retry_counter
      {"oem force-dtid_", CmdOemForceHwId},// +++ ASUS_BSP : add for force dtid
      {"oem reset-dev_info", CmdOemResetDevInfo},// +++ ASUS_BPS : add for reset dev info
      {"oem reset-auth2", CmdOemResetAuth2},// +++ ASUS_BPS : add for reset dev info
      {"oem reset-auth3", CmdOemResetAuth3},// +++ ASUS_BPS : add for reset WaterMask unlock
      {"oem disable-verity", CmdOemDisableVerity},// +++ ASUS_BSP : add for disable verity
      {"oem enable-verity", CmdOemEnableVerity},// +++ ASUS_BSP : add for enable verity
      {"oem enable-vbmeta", CmdOemEnableVbmeta},// +++ ASUS_BSP : add for enable vbmeta magic
      {"oem read-vbmeta", CmdOemReadVbmeta},// +++ ASUS_BSP : add for read vbmeta magic
      {"oem read-rollback", CmdOemReadRollback},// +++ ASUS_BSP : add for read vbmeta_system rollback value
      {"oem reset-rollback", CmdOemResetRollback},// +++ ASUS_BSP : add for reset vbmeta_system rollback value
      {"oem uart-on", CmdOemUartOn},// +++ add for uart on
      {"oem uart-off", CmdOemUartOff},// +++ add for uart off
      {"oem xts-unlock", CmdOemXtsUnlock},// +++ ASUS_BSP : Add for xts unlock
      {"oem asus-lock", CmdOemLock}, // +++ ASUS_BSP :Lock device
      {"oem check-avb", CmdOemAsusVerifiedState}
#endif
  };

  /* Register the commands only for non-user builds */
  Status = BoardSerialNum (StrSerialNum, sizeof (StrSerialNum));
  if (Status != EFI_SUCCESS) {
    DEBUG ((EFI_D_ERROR, "Error Finding board serial num: %x\n", Status));
    return Status;
  }
  /* Publish getvar variables */
  FastbootPublishVar ("kernel", "uefi");
  AsciiSPrint (MaxDownloadSizeStr,
                  sizeof (MaxDownloadSizeStr), "%ld", MaxDownLoadSize);
  FastbootPublishVar ("max-download-size", MaxDownloadSizeStr);

  if (IsDynamicPartitionSupport ()) {
    FastbootPublishVar ("is-userspace", "no");
  }

  AsciiSPrint (FullProduct, sizeof (FullProduct), "%a", PRODUCT_NAME);
  FastbootPublishVar ("product", FullProduct);
  FastbootPublishVar ("serialno", StrSerialNum);
  FastbootPublishVar ("secure", IsSecureBootEnabled () ? "yes" : "no");
  if (MultiSlotBoot) {
    /*Find ActiveSlot, bydefault _a will be the active slot
     *Populate MultiSlotMeta data will publish fastboot variables
     *like slot_successful, slot_unbootable,slot_retry_count and
     *CurrenSlot, these can modified using fastboot set_active command
     */
    FindPtnActiveSlot ();
    PopulateMultislotMetadata ();
    DEBUG ((EFI_D_VERBOSE, "Multi Slot boot is supported\n"));
  }

  GetPartitionCount (&PartitionCount);
  Status = PublishGetVarPartitionInfo (PublishedPartInfo, PartitionCount);
  if (Status != EFI_SUCCESS)
    DEBUG ((EFI_D_ERROR, "Failed to publish part info for all partitions\n"));
  BoardHwPlatformName (HWPlatformBuf, sizeof (HWPlatformBuf));
  GetRootDeviceType (DeviceType, sizeof (DeviceType));
  AsciiSPrint (StrVariant, sizeof (StrVariant), "%a %a", HWPlatformBuf,
               DeviceType);
  FastbootPublishVar ("variant", StrVariant);
  GetPageSize (&BlkSize);
  AsciiSPrint (LogicalBlkSizeStr, sizeof (LogicalBlkSizeStr), " 0x%x", BlkSize);
  FastbootPublishVar ("logical-block-size", LogicalBlkSizeStr);
  Type = CheckRootDeviceType ();
  if (Type == NAND) {
    BlkSize = NAND_PAGES_PER_BLOCK * BlkSize;
  }

  AsciiSPrint (EraseBlkSizeStr, sizeof (EraseBlkSizeStr), " 0x%x", BlkSize);
  FastbootPublishVar ("erase-block-size", EraseBlkSizeStr);
  GetDevInfo (&DevInfoPtr);
  FastbootPublishVar ("version-bootloader", DevInfoPtr->bootloader_version);
  FastbootPublishVar ("version-baseband", DevInfoPtr->radio_version);
  BatterySocOk = TargetBatterySocOk (&BatteryVoltage);
  AsciiSPrint (StrBatteryVoltage, sizeof (StrBatteryVoltage), "%d",
               BatteryVoltage);
  FastbootPublishVar ("battery-voltage", StrBatteryVoltage);
  AsciiSPrint (StrBatterySocOk, sizeof (StrBatterySocOk), "%a",
               BatterySocOk ? "yes" : "no");
  FastbootPublishVar ("battery-soc-ok", StrBatterySocOk);
  AsciiSPrint (ChargeScreenEnable, sizeof (ChargeScreenEnable), "%d",
               IsChargingScreenEnable ());
  FastbootPublishVar ("charger-screen-enabled", ChargeScreenEnable);
  AsciiSPrint (OffModeCharge, sizeof (OffModeCharge), "%d",
               IsChargingScreenEnable ());
  FastbootPublishVar ("off-mode-charge", ChargeScreenEnable);
  FastbootPublishVar ("unlocked", IsUnlocked () ? "yes" : "no");

  AsciiSPrint (StrSocVersion, sizeof (StrSocVersion), "%x",
                BoardPlatformChipVersion ());
  FastbootPublishVar ("hw-revision", StrSocVersion);

  if (IsDisableParallelDownloadFlash()) {
    FastbootPublishVar ("parallel-download-flash", "no");
  } else {
    FastbootPublishVar ("parallel-download-flash", "yes");
  }

  /* Register handlers for the supported commands*/
  UINT32 FastbootCmdCnt = sizeof (cmd_list) / sizeof (cmd_list[0]);
  for (i = 1; i < FastbootCmdCnt; i++)
    FastbootRegister (cmd_list[i].name, cmd_list[i].cb);

  if (IsDynamicPartitionSupport ()) {
    FastbootRegister ("reboot-recovery", CmdRebootRecovery);
    FastbootRegister ("reboot-fastboot", CmdRebootFastboot);
    FastbootRegister ("snapshot-update", CmdUpdateSnapshot);

    SnapshotMergeStatus = GetSnapshotMergeStatus ();

    switch (SnapshotMergeStatus) {
      case SNAPSHOTTED:
        SnapshotMergeStatus = SNAPSHOTTED;
        break;
      case MERGING:
        SnapshotMergeStatus = MERGING;
        break;
      default:
        SnapshotMergeStatus = NONE_MERGE_STATUS;
        break;
    }

    AsciiSPrint (SnapshotMergeState,
                  AsciiStrLen (VabSnapshotMergeStatus[SnapshotMergeStatus]) + 1,
                  "%a", VabSnapshotMergeStatus[SnapshotMergeStatus]);
    FastbootPublishVar ("snapshot-update-status", SnapshotMergeState);
  }

#ifdef ASUS_BUILD
  FastbootPublishVar("project", asus_project_info);
  FastbootPublishVar ("max-download-size", "536870912");// +++ ASUS_BSP : change max-download-size (512*1024*1024)
  FastbootPublishVar("cpuid", "135");
  FastbootPublishVar("cid", cid_name);
#endif

#ifdef ASUS_AI2205_BUILD
  FastbootPublishVar("project", "ROG_PHONE7");
#endif

  // Read Allow Ulock Flag
  Status = ReadAllowUnlockValue (&IsAllowUnlock);
  DEBUG ((EFI_D_VERBOSE, "IsAllowUnlock is %d\n", IsAllowUnlock));

  if (Status != EFI_SUCCESS) {
    DEBUG ((EFI_D_ERROR, "Error Reading FRP partition: %r\n", Status));
    return Status;
  }

  return EFI_SUCCESS;
}

VOID *FastbootDloadBuffer (VOID)
{
  return (VOID *)mUsbDataBuffer;
}

ANDROID_FASTBOOT_STATE FastbootCurrentState (VOID)
{
  return mState;
}
