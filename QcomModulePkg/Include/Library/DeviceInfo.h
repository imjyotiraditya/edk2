/*
 * Copyright (c) 2011,2014-2015,2021 The Linux Foundation. All rights
 * reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of The Linux Foundation nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
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
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the
 * disclaimer below) provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *
 *     * Neither the name of Qualcomm Innovation Center, Inc. nor the names of
 *       its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
 * GRANTED BY THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
 * HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef _DEVINFO_H_
#define _DEVINFO_H_

#include <Protocol/EFIVerifiedBoot.h>
#define DEVICE_MAGIC "ANDROID-BOOT!"
#define DEVICE_MAGIC_SIZE 13
#define MAX_VERSION_LEN 64
#define MAX_VB_PARTITIONS 32
#define MAX_USER_KEY_SIZE 2048
#define MAX_NAME_SIZE      56
#define MAX_VALUE_SIZE     32
#define MAX_ENTRY_SIZE     8

enum unlock_type {
  UNLOCK = 0,
  UNLOCK_CRITICAL,
};

typedef struct {
  UINT16  in_use;
  UINT16  name_size;
  UINT16  value_size;
  UINT8   name[MAX_NAME_SIZE];
  UINT8   value[MAX_VALUE_SIZE];
} persistent_value_type;

typedef struct device_info {
  CHAR8 magic[DEVICE_MAGIC_SIZE];
  BOOLEAN is_unlocked;
  BOOLEAN is_unlock_critical;
  BOOLEAN is_charger_screen_enabled;
  CHAR8 bootloader_version[MAX_VERSION_LEN];
  CHAR8 radio_version[MAX_VERSION_LEN];
  BOOLEAN verity_mode; // TRUE = enforcing, FALSE = logging
  UINT32 user_public_key_length;
  CHAR8 user_public_key[MAX_USER_KEY_SIZE];
  UINT64 rollback_index[MAX_VB_PARTITIONS];
  persistent_value_type  persistent_value[MAX_ENTRY_SIZE];
#ifdef ASUS_BUILD
  CHAR8 project_name[MAX_VERSION_LEN]; // +++ ASUS_BSP : add for project name
  CHAR8 hw_stage[MAX_VERSION_LEN]; // +++ ASUS_BSP : add for HW stage
  UINT32 device_tree_id; // +++ ASUS_BSP : add for device tree id
  CHAR8 ssn_num[16]; // +++ ASUS_BSP : add for ssn info
  CHAR8 isn_num[18]; // +++ ASUS_BSP : add for isn info
  CHAR8 imei_num[16]; // +++ ASUS_BSP : add for iemi info
  CHAR8 imei2_num[16]; // +++ ASUS_BSP : add for iemi2 info
  CHAR8 cid_name[18]; // +++ ASUS_BSP : add for cid info
  CHAR8 country_code[18]; // +++ ASUS_BPS : add for read country code
  BOOLEAN is_authorized; // +++ ASUS_BSP : add for ASUS dongle unlock
  BOOLEAN is_authorized_2; // +++ ASUS_BSP : add for ASUS dongle unlock
  BOOLEAN Avb_Verity; // +++ ASUS_BSP : add for AVB Verity
  BOOLEAN Flash_APDP; // +++ ASUS_BSP : add for check apdp partition
  UINT32 factory_reset2; // +++ ASUS_BSP : add for wipe-data by recovery and enter fastboot mode
  UINT32 slot_a_unbootable_counter;// +++ ASUS_BSP : add for unbootable_counter and retry_counter
  UINT32 slot_b_unbootable_counter;// +++ ASUS_BSP : add for unbootable_counter and retry_counter
  UINT32 slot_a_retry_counter;// +++ ASUS_BSP : add for unbootable_counter and retry_counter
  UINT32 slot_b_retry_counter;// +++ ASUS_BSP : add for unbootable_counter and retry_counter
  UINT32 boot_count;// +++ ASUS_BSP : add for boot count
  UINT32 lock_count;// +++ ASUS_BSP : add for lock count
  UINT32 total_dm_verity_counter;// +++ ASUS_BSP : add for dm-verity counter
  UINT32 dm_verity_counter;// +++ ASUS_BSP : add for dm-verity counter
  UINT32 verify_vbmeta_ret;// +++ ASUS_BSP : add for verify_vbmeta_ret
  CHAR8 xbl_version[MAX_VERSION_LEN];// +++ ASUS_BSP : add for xbl info
  BOOLEAN is_authorized_3; // +++ ASUS_BSP : add for WaterMask unlock
  unsigned long factory_crc; // +++ ASUS_BSP : add for check factory crc
  CHAR8 ssn_num_new[21]; // +++ ASUS_BSP : add for ssn info
  CHAR8 isn_num_new[21]; // +++ ASUS_BSP : add for isn info
  CHAR8 cpuid_hash[34];// +++ ASUS_BSP : add for get cpuid hash
  CHAR8 toolid[66]; // +++ ASUS_BSP : add for read TOOLID
  BOOLEAN is_logcat_asdf_on; // +++ ASUS_BSP : add for logcat-asdf sevices
  BOOLEAN is_uart_on; // uart on/off
  BOOLEAN is_debug_unlocked; // +++ ASUS_BSP : add for ASUS keymaster status control
#endif
  UINTN GoldenSnapshot;
  UINT32 snapshot_check_counter;
} DeviceInfo;

struct verified_boot_verity_mode {
  BOOLEAN verity_mode_enforcing;
  CHAR8 *name;
};

struct verified_boot_state_name {
  boot_state_t boot_state;
  CHAR8 *name;
};

BOOLEAN IsSnapshotGolden (VOID);
BOOLEAN IsUnlocked (VOID);
BOOLEAN IsUnlockCritical (VOID);
BOOLEAN IsEnforcing (VOID);
BOOLEAN IsChargingScreenEnable (VOID);
VOID GetBootloaderVersion (CHAR8 *BootloaderVersion, UINT32 Len);
VOID GetRadioVersion (CHAR8 *RadioVersion, UINT32 Len);
EFI_STATUS EnableChargingScreen (BOOLEAN IsEnabled);
EFI_STATUS EnableEnforcingMode (BOOLEAN IsEnabled);
EFI_STATUS SetDeviceUnlockValue (UINT32 Type, BOOLEAN State);
EFI_STATUS DeviceInfoInit (VOID);
EFI_STATUS ReadRollbackIndex (UINT32 Loc, UINT64 *RollbackIndex);
EFI_STATUS WriteRollbackIndex (UINT32 Loc, UINT64 RollbackIndex);
EFI_STATUS StoreUserKey (CHAR8 *UserKey, UINT32 UserKeySize);
EFI_STATUS GetUserKey (CHAR8 **UserKey, UINT32 *UserKeySize);
EFI_STATUS EraseUserKey (VOID);
EFI_STATUS ReadPersistentValue (CONST UINT8 *Name, UINTN NameSize,
                                UINT8 *Value, UINTN *ValueSize);
EFI_STATUS WritePersistentValue (CONST UINT8 *Name, UINTN NameSize,
                                 CONST UINT8 *Value, UINTN ValueSize);

#ifdef ASUS_BUILD
////////////////////////////////////////////////////////////////////////
// +++ ROG6 : PROJECT COMMON FUNCTION
////////////////////////////////////////////////////////////////////////
// +++ ASUS_BSP : add for xbl info
VOID GetXBLVersion (CHAR8* XBLVersion, UINT32 Len);
EFI_STATUS SetXBLVersion(CHAR8* XBLVersion, UINT32 Len);
// --- ASUS_BSP : add for xbl info

// +++ ASUS_BSP : add for abl info
EFI_STATUS SetBootloaderVersion(CHAR8* BootloaderVersion, UINT32 Len);
// --- ASUS_BSP : add for abl info

// +++ ASUS_BSP : add for project name
VOID GetProjName(CHAR8* ProjName, UINT32 Len);
EFI_STATUS SetProjName(CHAR8* ProjName, UINT32 Len);
// --- ASUS_BSP : add for project name

// +++ ASUS_BSP : add for HW stage
VOID GetHWStage(CHAR8* HWStage, UINT32 Len);
EFI_STATUS SetHWStage(CHAR8* HWStage, UINT32 Len);
// --- ASUS_BSP : add for HW stage

// +++ ASUS_BSP : add for device tree id
UINT32 GetDeviceTreeID();
EFI_STATUS SetDeviceTreeID(UINT32 device_tree_id);
// --- ASUS_BSP : add for device tree id

// +++ ASUS_BSP : add for ssn info
VOID GetSSNNum(CHAR8* SSN_Num, UINT32 Len);
EFI_STATUS SetSSNNum(CHAR8* SSN_Num, UINT32 Len);
// --- ASUS_BSP : add for ssn info

// +++ ASUS_BSP : add for isn info
VOID GetISNNum(CHAR8* ISN_Num, UINT32 Len);
EFI_STATUS SetISNNum(CHAR8* ISN_Num, UINT32 Len);
// --- ASUS_BSP : add for isn info

// +++ ASUS_BSP : add for imei info
VOID GetIMEINum(CHAR8* IMEI_Num, UINT32 Len);
EFI_STATUS SetIMEINum(CHAR8* IMEI_Num, UINT32 Len);
// --- ASUS_BSP : add for imei info

// +++ ASUS_BSP : add for imei2 info
VOID GetIMEI2Num(CHAR8* IMEI2_Num, UINT32 Len);
EFI_STATUS SetIMEI2Num(CHAR8* IMEI2_Num, UINT32 Len);
// --- ASUS_BSP : add for imei2 info

// +++ ASUS_BSP : add for cid info
VOID GetCIDName(CHAR8* CID_Name, UINT32 Len);
EFI_STATUS SetCIDName(CHAR8* CID_Name, UINT32 Len);
// --- ASUS_BSP : add for cid info

// +++ ASUS_BPS : add for read country code
VOID GetCountryCode(CHAR8* Country_Code, UINT32 Len);
EFI_STATUS SetCountryCode(CHAR8* Country_Code, UINT32 Len);
// --- ASUS_BPS : add for read country code

// +++ ASUS_BSP : add for get cpuid hash
VOID GetCpuidHash(CHAR8* Cpuid_Hash, UINT32 Len);
EFI_STATUS SetCpuidHash(CHAR8* CpuidHash, UINT32 Len);
// --- ASUS_BSP : add for get cpuid hash

// +++ ASUS_BSP : add for read TOOLID
VOID GetTOOLID(CHAR8* ToodId, UINT32 Len);
EFI_STATUS SetTOOLID(CHAR8* ToodId, UINT32 Len);
// --- ASUS_BSP : add for read TOOLID

// +++ ASUS_BSP : add for ASUS dongle unlock
BOOLEAN IsAuthorized(VOID);
BOOLEAN IsAuthorized_2(VOID);
EFI_STATUS SetAuthorizedValue(BOOLEAN State);
EFI_STATUS SetAuthorized2Value(BOOLEAN State);
// --- ASUS_BSP : add for ASUS dongle unlock

// +++ ASUS_BSP : add for WaterMask unlock
BOOLEAN IsAuthorized_3(VOID);
EFI_STATUS SetAuthorized3Value(BOOLEAN State);
// --- ASUS_BSP : add for WaterMask unlock

// +++ ASUS_BSP : add for AVB Verity
BOOLEAN GetAvbVerity();
EFI_STATUS SetAvbVerity(BOOLEAN Flag);
// --- ASUS_BSP : add for AVB Verity

// +++ ASUS_BSP : add for check apdp partition
BOOLEAN GetAPDP();
EFI_STATUS SetAPDP(BOOLEAN Flag);
// +++ ASUS_BSP : add for check apdp partition

// +++ ASUS_BSP : add for wipe-data by recovery
EFI_STATUS FactoryResetFromRecovery();
// --- ASUS_BSP : add for wipe-data by recovery

// +++ ASUS_BSP : add for wipe-data by recovery and enter fastboot mode
BOOLEAN GetOemFactoryReset2Flag();
EFI_STATUS SetOemFactoryReset2Flag();
// --- ASUS_BSP : add for wipe-data by recovery and enter fastboot mode

// +++ ASUS_BSP : add for unbootable_counter and retry_counter
UINT32 GetSlotAUnbootableCounter();
EFI_STATUS SetSlotAUnbootableCounter(UINT32 counter);

UINT32 GetSlotARetryCounter();
EFI_STATUS SetSlotARetryCounter(UINT32 counter);

UINT32 GetSlotBUnbootableCounter();
EFI_STATUS SetSlotBUnbootableCounter(UINT32 counter);

UINT32 GetSlotBRetryCounter();
EFI_STATUS SetSlotBRetryCounter(UINT32 counter);
// --- ASUS_BSP : add for unbootable_counter and retry_counter

// +++ ASUS_BSP : add for boot count
UINT32 GetBootCounter();
EFI_STATUS SetBootCounter(UINT32 counter);
// --- ASUS_BSP : add for boot count

// +++ ASUS_BSP : add for lock count
UINT32 GetLockCounter();
EFI_STATUS SetLockCounter(UINT32 counter);
// --- ASUS_BSP : add for lock count

// +++ ASUS_BSP : add for dm-verity counter
UINT32 GetTotalDmVerityCounter();
EFI_STATUS SetTotalDmVerityCounter(UINT32 counter);
UINT32 GetDmVerityCounter();
EFI_STATUS SetDmVerityCounter(UINT32 counter);
// --- ASUS_BSP : add for dm-verity counter

// +++ ASUS_BSP : add for verify_vbmeta_ret
UINT32 GetVerifyVbmetaRet();
EFI_STATUS SetVerifyVbmetaRet(UINT32 ret);
// --- ASUS_BSP : add for verify_vbmeta_ret

// +++ ASUS_BSP : add for user build menu
BOOLEAN GetRebootMode(VOID);
VOID SetRebootMode(BOOLEAN status);
// --- ASUS_BSP : add for user build menu

// +++ ASUS_BSP : add for check factory crc
unsigned long GetFactoryCRC(VOID);
EFI_STATUS SetFactoryCRC(unsigned long crc_value);
// --- ASUS_BSP : add for check factory crc

// +++ ASUS_BSP : add for logcat-asdf sevices
BOOLEAN GetLogcatAsdfOn();
EFI_STATUS SetLogcatAsdfOn(BOOLEAN State);
// --- ASUS_BSP : add for logcat-asdf sevices

// +++ ASUS_BSP : add for uart control
BOOLEAN GetUartStatus(VOID);
EFI_STATUS SetUartStatus(BOOLEAN uart_status);
// --- ASUS_BSP : add for uart control

BOOLEAN IsDebugUnlocked(VOID);
EFI_STATUS SetDeviceDebugUnlockValue(BOOLEAN State);

// +++ ASUS_BSP : add for snapshot check counter
UINT32 GetSnapshotCheckCounter();
EFI_STATUS SetSnapshotCheckCounter(UINT32 counter);
// --- ASUS_BSP : add for snapshot check counter

////////////////////////////////////////////////////////////////////////
// --- ROG6 : PROJECT COMMON FUNCTION
////////////////////////////////////////////////////////////////////////
#endif


EFI_STATUS
SetSnapshotGolden (UINTN Val);
#endif
