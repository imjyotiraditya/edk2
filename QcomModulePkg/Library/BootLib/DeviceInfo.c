/* Copyright (c) 2016-2018, 2021 The Linux Foundation. All rights reserved.
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
#include "AutoGen.h"
#include "LinuxLoaderLib.h"
#include "Board.h"
#include <FastbootLib/FastbootCmds.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/PartitionTableUpdate.h>
#include <Library/Recovery.h>
#include <Library/StackCanary.h>

#include "abl.h"

DeviceInfo DevInfo;
#define GOLDEN_SNAPSHOT_MAGIC 0x575757

STATIC BOOLEAN FirstReadDevInfo = TRUE;

BOOLEAN IsSnapshotGolden (VOID)
{
  return (DevInfo.GoldenSnapshot == GOLDEN_SNAPSHOT_MAGIC) ? TRUE : FALSE;
}

BOOLEAN IsUnlocked (VOID)
{
  return DevInfo.is_unlocked;
}

BOOLEAN IsUnlockCritical (VOID)
{
  return DevInfo.is_unlock_critical;
}

BOOLEAN IsEnforcing (VOID)
{
  return DevInfo.verity_mode;
}

BOOLEAN IsChargingScreenEnable (VOID)
{
#if defined ABL_FTM
  return FALSE;
#else
  return DevInfo.is_charger_screen_enabled;
#endif
}

VOID
GetDevInfo (DeviceInfo **DevInfoPtr)
{
  *DevInfoPtr = &DevInfo;
}
VOID
GetBootloaderVersion (CHAR8 *BootloaderVersion, UINT32 Len)
{
  AsciiSPrint (BootloaderVersion, Len, "%a", DevInfo.bootloader_version);
}

VOID
GetRadioVersion (CHAR8 *RadioVersion, UINT32 Len)
{
  AsciiSPrint (RadioVersion, Len, "%a", DevInfo.radio_version);
}

EFI_STATUS
EnableChargingScreen (BOOLEAN IsEnabled)
{
  EFI_STATUS Status = EFI_SUCCESS;

  if (IsChargingScreenEnable () != IsEnabled) {
    DevInfo.is_charger_screen_enabled = IsEnabled;
    Status = ReadWriteDeviceInfo (WRITE_CONFIG, &DevInfo, sizeof (DevInfo));
    if (Status != EFI_SUCCESS) {
      DEBUG ((EFI_D_ERROR, "Error %a charger screen: %r\n",
              (IsEnabled ? "Enabling" : "Disabling"), Status));
      return Status;
    }
  }

  return Status;
}

EFI_STATUS
EnableEnforcingMode (BOOLEAN IsEnabled)
{
  EFI_STATUS Status = EFI_SUCCESS;

  if (IsEnforcing () != IsEnabled) {
    DevInfo.verity_mode = IsEnabled;
    Status = ReadWriteDeviceInfo (WRITE_CONFIG, &DevInfo, sizeof (DevInfo));
    if (Status != EFI_SUCCESS) {
      DEBUG ((EFI_D_ERROR, "VBRwDeviceState Returned error: %r\n", Status));
      return Status;
    }
  }

  return Status;
}

STATIC EFI_STATUS
SetUnlockValue (BOOLEAN State)
{
  EFI_STATUS Status = EFI_SUCCESS;

  if (IsUnlocked () != State) {
    DevInfo.is_unlocked = State;
    Status = ReadWriteDeviceInfo (WRITE_CONFIG, &DevInfo, sizeof (DevInfo));
    if (Status != EFI_SUCCESS) {
      DEBUG ((EFI_D_ERROR, "Unable set the unlock value: %r\n", Status));
      return Status;
    }
  }

  return Status;
}

STATIC EFI_STATUS
SetUnlockCriticalValue (BOOLEAN State)
{
  EFI_STATUS Status = EFI_SUCCESS;

  if (IsUnlockCritical () != State) {
    DevInfo.is_unlock_critical = State;
    Status = ReadWriteDeviceInfo (WRITE_CONFIG, &DevInfo, sizeof (DevInfo));
    if (Status != EFI_SUCCESS) {
      DEBUG (
          (EFI_D_ERROR, "Unable set the unlock critical value: %r\n", Status));
      return Status;
    }
  }
  return Status;
}

EFI_STATUS
SetDeviceUnlockValue (UINT32 Type, BOOLEAN State)
{
  EFI_STATUS Status = EFI_SUCCESS;
  struct RecoveryMessage Msg;
  EFI_GUID Ptype = gEfiMiscPartitionGuid;
  MemCardType CardType = UNKNOWN;

  switch (Type) {
  case UNLOCK:
    Status = SetUnlockValue (State);
    break;
  case UNLOCK_CRITICAL:
    Status = SetUnlockCriticalValue (State);
    break;
  default:
    Status = EFI_UNSUPPORTED;
    break;
  }
  if (Status != EFI_SUCCESS)
    return Status;

  Status = ResetDeviceState ();
  if (Status != EFI_SUCCESS) {
    if (Type == UNLOCK)
      SetUnlockValue (!State);
    else if (Type == UNLOCK_CRITICAL)
      SetUnlockCriticalValue (!State);

    DEBUG ((EFI_D_ERROR, "Unable to set the Value: %r", Status));
    return Status;
  }

  gBS->SetMem ((VOID *)&Msg, sizeof (Msg), 0);
  Status = AsciiStrnCpyS (Msg.recovery, sizeof (Msg.recovery),
                          RECOVERY_WIPE_DATA, AsciiStrLen (RECOVERY_WIPE_DATA));
  if (Status == EFI_SUCCESS) {
    CardType = CheckRootDeviceType ();
    if (CardType == NAND) {
      Status = GetNandMiscPartiGuid (&Ptype);
      if (Status != EFI_SUCCESS) {
        return Status;
      }
    }

    Status = WriteToPartition (&Ptype, &Msg, sizeof (Msg));
  }

  return Status;
}

EFI_STATUS
UpdateDevInfo (CHAR16 *Pname, CHAR8 *ImgVersion)
{
  EFI_STATUS Status = EFI_SUCCESS;

  if (!StrCmp ((CONST CHAR16 *)Pname, (CONST CHAR16 *)L"bootloader")) {
    AsciiStrnCpyS (DevInfo.bootloader_version, MAX_VERSION_LEN, PRODUCT_NAME,
                   AsciiStrLen (PRODUCT_NAME));
    AsciiStrnCatS (DevInfo.bootloader_version, MAX_VERSION_LEN, "-",
                   AsciiStrLen ("-"));
    AsciiStrnCatS (DevInfo.bootloader_version, MAX_VERSION_LEN, ImgVersion,
                   AsciiStrLen (ImgVersion));
  } else {
    AsciiStrnCpyS (DevInfo.radio_version, MAX_VERSION_LEN, PRODUCT_NAME,
                   AsciiStrLen (PRODUCT_NAME));
    AsciiStrnCatS (DevInfo.radio_version, MAX_VERSION_LEN, "-",
                   AsciiStrLen ("-"));
    AsciiStrnCatS (DevInfo.radio_version, MAX_VERSION_LEN, ImgVersion,
                   AsciiStrLen (ImgVersion));
  }

  Status =
      ReadWriteDeviceInfo (WRITE_CONFIG, (VOID *)&DevInfo, sizeof (DevInfo));
  if (Status != EFI_SUCCESS) {
    DEBUG ((EFI_D_ERROR, "Unable to Write Device Info: %r\n", Status));
  }
  return Status;
}


#ifdef ASUS_BUILD
////////////////////////////////////////////////////////////////////////
// +++ ROG7  PROJECT COMMON FUNCTION
////////////////////////////////////////////////////////////////////////
// +++ ASUS_BSP : add for xbl info
VOID GetXBLVersion (CHAR8* XBLVersion, UINT32 Len)
{
	AsciiSPrint (XBLVersion, Len, "%a", DevInfo.xbl_version);
}

EFI_STATUS SetXBLVersion(CHAR8* XBLVersion, UINT32 Len)
{
	EFI_STATUS Status = EFI_SUCCESS;
	if (DevInfo.xbl_version!= XBLVersion) {
		AsciiStrnCpyS(DevInfo.xbl_version, sizeof(DevInfo.xbl_version), XBLVersion, MAX_RSP_SIZE);
		Status = ReadWriteDeviceInfo(WRITE_CONFIG, &DevInfo, sizeof(DevInfo));
		if (Status != EFI_SUCCESS)
		{
			DEBUG((EFI_D_ERROR, "[ABL] Unable to set the xbl_version : %r\n", Status));
			return Status;
		}
	}
	return Status;
}
// --- ASUS_BSP : add for xbl info

// +++ ASUS_BSP : add for abl info
EFI_STATUS SetBootloaderVersion(CHAR8* BootloaderVersion, UINT32 Len)
{
	EFI_STATUS Status = EFI_SUCCESS;
	if (DevInfo.bootloader_version!= BootloaderVersion) {
		AsciiStrnCpyS(DevInfo.bootloader_version, sizeof(DevInfo.bootloader_version), BootloaderVersion, MAX_RSP_SIZE);
		Status = ReadWriteDeviceInfo(WRITE_CONFIG, &DevInfo, sizeof(DevInfo));
		if (Status != EFI_SUCCESS)
		{
			DEBUG((EFI_D_ERROR, "[ABL] Unable to set the bootloader_version : %r\n", Status));
			return Status;
		}
	}
	return Status;
}
// --- ASUS_BSP : add for abl info

// +++ ASUS_BSP : add for project name
VOID GetProjName(CHAR8* ProjName, UINT32 Len)
{
	AsciiSPrint(ProjName, Len, "%a", DevInfo.project_name);
}

EFI_STATUS SetProjName(CHAR8* ProjName, UINT32 Len)
{
	EFI_STATUS Status = EFI_SUCCESS;
	if (DevInfo.project_name!= ProjName) {
		AsciiStrnCpyS(DevInfo.project_name, sizeof(DevInfo.project_name),ProjName, sizeof(ProjName));
		Status = ReadWriteDeviceInfo(WRITE_CONFIG, &DevInfo, sizeof(DevInfo));
		if (Status != EFI_SUCCESS)
		{
			DEBUG((EFI_D_ERROR, "[ABL] Unable set the project_name : %r\n", Status));
			return Status;
		}
	}
	return Status;
}
// --- ASUS_BSP : add for project name

// +++ ASUS_BSP : add for HW stage
VOID GetHWStage(CHAR8* HWStage, UINT32 Len)
{
	AsciiSPrint(HWStage, Len, "%a", DevInfo.hw_stage);
}

EFI_STATUS SetHWStage(CHAR8* HWStage, UINT32 Len)
{
	EFI_STATUS Status = EFI_SUCCESS;
	if (DevInfo.hw_stage!= HWStage) {
		AsciiStrnCpyS(DevInfo.hw_stage, sizeof(DevInfo.hw_stage),HWStage, sizeof(HWStage));
		Status = ReadWriteDeviceInfo(WRITE_CONFIG, &DevInfo, sizeof(DevInfo));
		if (Status != EFI_SUCCESS)
		{
			DEBUG((EFI_D_ERROR, "[ABL] Unable set the hw_stage : %r\n", Status));
			return Status;
		}
	}
	return Status;
}
// --- ASUS_BSP : add for HW stage

// +++ ASUS_BSP : add for device tree id
UINT32 GetDeviceTreeID()
{
	return DevInfo.device_tree_id;
}

EFI_STATUS SetDeviceTreeID(UINT32 DeviceTreeID)
{
	EFI_STATUS Status = EFI_SUCCESS;
	if (DevInfo.device_tree_id!= DeviceTreeID) {
		DevInfo.device_tree_id = DeviceTreeID;
		Status = ReadWriteDeviceInfo(WRITE_CONFIG, &DevInfo, sizeof(DevInfo));
		if (Status != EFI_SUCCESS)
		{
			DEBUG((EFI_D_ERROR, "[ABL] Unable set the device_tree_id : %r\n", Status));
			return Status;
		}
	}
	return Status;
}
// --- ASUS_BSP : add for device tree id

// +++ ASUS_BSP : add for ssn info
VOID GetSSNNum(CHAR8* SSN_Num, UINT32 Len)
{
	AsciiSPrint(SSN_Num, Len, "%a", DevInfo.ssn_num_new);
}

EFI_STATUS SetSSNNum(CHAR8* SSN_Num, UINT32 Len)
{
	EFI_STATUS Status = EFI_SUCCESS;
	if (DevInfo.ssn_num_new!= SSN_Num) {
		AsciiStrnCpyS(DevInfo.ssn_num_new, sizeof(DevInfo.ssn_num_new),SSN_Num, MAX_RSP_SIZE);
		Status = ReadWriteDeviceInfo(WRITE_CONFIG, &DevInfo, sizeof(DevInfo));
		if (Status != EFI_SUCCESS)
		{
			DEBUG((EFI_D_ERROR, "[ABL] Unable set the ssn_num_new : %r\n", Status));
			return Status;
		}
	}
	return Status;
}
// --- ASUS_BSP : add for ssn info

// +++ ASUS_BSP : add for isn info
VOID GetISNNum(CHAR8* ISN_Num, UINT32 Len)
{
	AsciiSPrint(ISN_Num, Len, "%a", DevInfo.isn_num_new);
}

EFI_STATUS SetISNNum(CHAR8* ISN_Num, UINT32 Len)
{
	EFI_STATUS Status = EFI_SUCCESS;
	if (DevInfo.isn_num_new!= ISN_Num) {
		AsciiStrnCpyS(DevInfo.isn_num_new, sizeof(DevInfo.isn_num_new),ISN_Num, MAX_RSP_SIZE);
		Status = ReadWriteDeviceInfo(WRITE_CONFIG, &DevInfo, sizeof(DevInfo));
		if (Status != EFI_SUCCESS)
		{
			DEBUG((EFI_D_ERROR, "[ABL] Unable set the isn_num : %r\n", Status));
			return Status;
		}
	}
	return Status;
}
// --- ASUS_BSP : add for isn info

// +++ ASUS_BSP : add for imei info
VOID GetIMEINum(CHAR8* IMEI_Num, UINT32 Len)
{
	AsciiSPrint(IMEI_Num, Len, "%a", DevInfo.imei_num);
}

EFI_STATUS SetIMEINum(CHAR8* IMEI_Num, UINT32 Len)
{
	EFI_STATUS Status = EFI_SUCCESS;
	if (DevInfo.imei_num!= IMEI_Num) {
		AsciiStrnCpyS(DevInfo.imei_num, sizeof(DevInfo.imei_num),IMEI_Num, MAX_RSP_SIZE);
		Status = ReadWriteDeviceInfo(WRITE_CONFIG, &DevInfo, sizeof(DevInfo));
		if (Status != EFI_SUCCESS)
		{
			DEBUG((EFI_D_ERROR, "[ABL] Unable set the imei_num : %r\n", Status));
			return Status;
		}
	}
	return Status;
}
// --- ASUS_BSP : add for imei info

// +++ ASUS_BSP : add for imei2 info
VOID GetIMEI2Num(CHAR8* IMEI2_Num, UINT32 Len)
{
	AsciiSPrint(IMEI2_Num, Len, "%a", DevInfo.imei2_num);
}

EFI_STATUS SetIMEI2Num(CHAR8* IMEI2_Num, UINT32 Len)
{
	EFI_STATUS Status = EFI_SUCCESS;
	if (DevInfo.imei2_num!= IMEI2_Num) {
		AsciiStrnCpyS(DevInfo.imei2_num, sizeof(DevInfo.imei2_num),IMEI2_Num, MAX_RSP_SIZE);
		Status = ReadWriteDeviceInfo(WRITE_CONFIG, &DevInfo, sizeof(DevInfo));
		if (Status != EFI_SUCCESS)
		{
			DEBUG((EFI_D_ERROR, "[ABL] Unable set the imei2_num : %r\n", Status));
			return Status;
		}
	}
	return Status;
}
// --- ASUS_BSP : add for imei2 info

// +++ ASUS_BSP : add for cid info
VOID GetCIDName(CHAR8* CID_Name, UINT32 Len)
{
	AsciiSPrint(CID_Name, Len, "%a", DevInfo.cid_name);
}

EFI_STATUS SetCIDName(CHAR8* CID_Name, UINT32 Len)
{
	EFI_STATUS Status = EFI_SUCCESS;
	if (DevInfo.cid_name!= CID_Name) {
		AsciiStrnCpyS(DevInfo.cid_name, sizeof(DevInfo.cid_name),CID_Name, MAX_RSP_SIZE);
		Status = ReadWriteDeviceInfo(WRITE_CONFIG, &DevInfo, sizeof(DevInfo));
		if (Status != EFI_SUCCESS)
		{
			DEBUG((EFI_D_ERROR, "[ABL] Unable set the cid_num : %r\n", Status));
			return Status;
		}
	}
	return Status;
}
// --- ASUS_BSP : add for cid info

// +++ ASUS_BPS : add for read country code
VOID GetCountryCode(CHAR8* Country_Code, UINT32 Len)
{
	AsciiSPrint(Country_Code, Len, "%a", DevInfo.country_code);
}

EFI_STATUS SetCountryCode(CHAR8* Country_Code, UINT32 Len)
{
	EFI_STATUS Status = EFI_SUCCESS;
	if (DevInfo.country_code!= Country_Code) {
		AsciiStrnCpyS(DevInfo.country_code, sizeof(DevInfo.country_code),Country_Code, MAX_RSP_SIZE);
		Status = ReadWriteDeviceInfo(WRITE_CONFIG, &DevInfo, sizeof(DevInfo));
		if (Status != EFI_SUCCESS)
		{
			DEBUG((EFI_D_ERROR, "[ABL] Unable set the country_code : %r\n", Status));
			return Status;
		}
	}
	return Status;
}
// --- ASUS_BPS : add for read country code

// +++ ASUS_BSP : add for get cpuid hash
VOID GetCpuidHash(CHAR8* Cpuid_Hash, UINT32 Len){
	AsciiSPrint(Cpuid_Hash, Len, "%a", DevInfo.cpuid_hash);
}

EFI_STATUS SetCpuidHash(CHAR8* CpuidHash, UINT32 Len){
	EFI_STATUS Status = EFI_SUCCESS;
	if (DevInfo.cpuid_hash!= CpuidHash) {
		AsciiStrnCpyS(DevInfo.cpuid_hash, sizeof(DevInfo.cpuid_hash),CpuidHash, MAX_RSP_SIZE);
		Status = ReadWriteDeviceInfo(WRITE_CONFIG, &DevInfo, sizeof(DevInfo));
		if (Status != EFI_SUCCESS)
		{
			DEBUG((EFI_D_ERROR, "[ABL] Unable set the cpuid_hash : %r\n", Status));
			return Status;
		}
	}
	return Status;
}
// --- ASUS_BSP : add for get cpuid hash

// +++ ASUS_BSP : add for read TOOLID
VOID GetTOOLID(CHAR8* ToodId, UINT32 Len){
	AsciiSPrint(ToodId, Len, "%a", DevInfo.toolid);
}

EFI_STATUS SetTOOLID(CHAR8* ToodId, UINT32 Len){
	EFI_STATUS Status = EFI_SUCCESS;
	if (DevInfo.toolid!= ToodId) {
		AsciiStrnCpyS(DevInfo.toolid, sizeof(DevInfo.toolid),ToodId, MAX_RSP_SIZE);
		Status = ReadWriteDeviceInfo(WRITE_CONFIG, &DevInfo, sizeof(DevInfo));
		if (Status != EFI_SUCCESS)
		{
			DEBUG((EFI_D_ERROR, "[ABL] Unable set the toolid : %r\n", Status));
			return Status;
		}
	}
	return Status;
}
// --- ASUS_BSP : add for read TOOLID

// +++ ASUS_BSP : add for ASUS dongle unlock
BOOLEAN IsAuthorized(VOID)
{
	return DevInfo.is_authorized;
}

BOOLEAN IsAuthorized_2(VOID)
{
	return DevInfo.is_authorized_2;
}

// +++ ASUS_BSP : add for WaterMask unlock
BOOLEAN IsAuthorized_3(VOID)
{
	return DevInfo.is_authorized_3;
}
// --- ASUS_BSP : add for WaterMask unlock

EFI_STATUS SetAuthorizedValue(BOOLEAN State)
{
	EFI_STATUS Status = EFI_SUCCESS;

	if (IsAuthorized() != State) {
		DevInfo.is_authorized = State;
		Status = ReadWriteDeviceInfo(WRITE_CONFIG, &DevInfo, sizeof(DevInfo));
		if (Status != EFI_SUCCESS)
		{
			DEBUG((EFI_D_ERROR, "[ABL] Unable set the DevInfo.is_authorized value: %r\n", Status));
			return Status;
		}
	}
	return Status;
}

EFI_STATUS SetAuthorized2Value(BOOLEAN State)
{
	EFI_STATUS Status = EFI_SUCCESS;

	if (IsAuthorized_2() != State) {
		DevInfo.is_authorized_2 = State;
		Status = ReadWriteDeviceInfo(WRITE_CONFIG, &DevInfo, sizeof(DevInfo));
		if (Status != EFI_SUCCESS)
		{
			DEBUG((EFI_D_ERROR, "[ABL] Unable set the DevInfo.is_authorized_2 value: %r\n", Status));
			return Status;
		}
	}
	return Status;
}
// --- ASUS_BSP : add for ASUS dongle unlock

// +++ ASUS_BSP : add for WaterMask unlock
EFI_STATUS SetAuthorized3Value(BOOLEAN State)
{
	EFI_STATUS Status = EFI_SUCCESS;

	if (IsAuthorized_3() != State) {
		DevInfo.is_authorized_3 = State;
		Status = ReadWriteDeviceInfo(WRITE_CONFIG, &DevInfo, sizeof(DevInfo));
		if (Status != EFI_SUCCESS)
		{
			DEBUG((EFI_D_ERROR, "[ABL] Unable set the DevInfo.is_authorized_3 value: %r\n", Status));
			return Status;
		}
	}
	return Status;
}
// --- ASUS_BSP : add for WaterMask unlock

// +++ ASUS_BSP : add for AVB Verity
BOOLEAN GetAvbVerity()
{
	check_verity();
	return DevInfo.Avb_Verity;
}

EFI_STATUS SetAvbVerity(BOOLEAN Flag)
{
	EFI_STATUS Status = EFI_SUCCESS;
	DevInfo.Avb_Verity = Flag;
	Status = ReadWriteDeviceInfo(WRITE_CONFIG, &DevInfo, sizeof(DevInfo));
	DEBUG((EFI_D_ERROR, "[ABL] device_info.Avb_Verity = %a\n", Flag? "Disable" : "Enable"));
	return Status;
}
// --- ASUS_BSP : add for AVB Verity

// +++ ASUS_BSP : add for check apdp partition
BOOLEAN GetAPDP()
{
	check_apdp();
	return DevInfo.Flash_APDP;
}

EFI_STATUS SetAPDP(BOOLEAN Flag)
{
	EFI_STATUS Status = EFI_SUCCESS;
	DevInfo.Flash_APDP = Flag;
	Status = ReadWriteDeviceInfo(WRITE_CONFIG, &DevInfo, sizeof(DevInfo));
	DEBUG((EFI_D_ERROR, "[ABL] device_info.Flash_APDP = %a\n", Flag? "TRUE" : "FALSE"));
	return Status;
}
// +++ ASUS_BSP : add for check apdp partition

// +++ ASUS_BSP : add for wipe-data by recovery
EFI_STATUS FactoryResetFromRecovery()
{
	EFI_STATUS Status = EFI_SUCCESS;
	struct RecoveryMessage Msg;

	SetMem((VOID *)&Msg, sizeof(Msg), 0);
	AsciiStrnCpyS(Msg.recovery, sizeof(Msg.recovery), RECOVERY_WIPE_DATA, AsciiStrLen(RECOVERY_WIPE_DATA));
	Status = WriteToPartition(&gEfiMiscPartitionGuid, &Msg, sizeof (Msg));

	if (Status != EFI_SUCCESS)
		return Status;

	return Status;
}
// --- ASUS_BSP : add for wipe-data by recovery

// +++ ASUS_BSP : add for wipe-data by recovery and enter fastboot mode
BOOLEAN GetOemFactoryReset2Flag()
{
	EFI_STATUS Status = EFI_SUCCESS;

	if(DevInfo.factory_reset2 == 1)
	{
		DevInfo.factory_reset2 = 2;
		Status = ReadWriteDeviceInfo(WRITE_CONFIG, &DevInfo, sizeof(DevInfo));
		if (Status != EFI_SUCCESS)
		{
			DEBUG((EFI_D_ERROR, "[ABL] Unable set the DevInfo.factory_reset2 value: %r\n", Status));
		}
		return FALSE;
	}
	else if(DevInfo.factory_reset2 == 2)
	{
		DevInfo.factory_reset2 = 0;
		Status = ReadWriteDeviceInfo(WRITE_CONFIG, &DevInfo, sizeof(DevInfo));
		if (Status != EFI_SUCCESS)
		{
			DEBUG((EFI_D_ERROR, "[ABL] Unable set the DevInfo.factory_reset2 value: %r\n", Status));
		}
		return TRUE;
	}
	return FALSE;
}

EFI_STATUS SetOemFactoryReset2Flag()
{
	EFI_STATUS Status = EFI_SUCCESS;

	DevInfo.factory_reset2 = 1;
	Status = ReadWriteDeviceInfo(WRITE_CONFIG, &DevInfo, sizeof(DevInfo));
	if (Status != EFI_SUCCESS)
	{
		DEBUG((EFI_D_ERROR, "[ABL] Unable set the DevInfo.factory_reset2 value: %r\n", Status));
	}
	return Status;
}
// --- ASUS_BSP : add for wipe-data by recovery and enter fastboot mode

// +++ ASUS_BSP : add for unbootable_counter and retry_counter
UINT32 GetSlotAUnbootableCounter(){
	return DevInfo.slot_a_unbootable_counter;
}

EFI_STATUS SetSlotAUnbootableCounter(UINT32 counter){
	EFI_STATUS Status = EFI_SUCCESS;
	DevInfo.slot_a_unbootable_counter = counter;
	Status = ReadWriteDeviceInfo(WRITE_CONFIG, &DevInfo, sizeof(DevInfo));
	DEBUG((EFI_D_ERROR, "[ABL] DevInfo.slot_a_unbootable_counter = %d\n", counter));
	return Status;
}

UINT32 GetSlotARetryCounter(){
	return DevInfo.slot_a_retry_counter;
}

EFI_STATUS SetSlotARetryCounter(UINT32 counter){
	EFI_STATUS Status = EFI_SUCCESS;
	DevInfo.slot_a_retry_counter = counter;
	Status = ReadWriteDeviceInfo(WRITE_CONFIG, &DevInfo, sizeof(DevInfo));
	DEBUG((EFI_D_ERROR, "[ABL] DevInfo.slot_a_retry_counter = %d\n", counter));
	return Status;
}

UINT32 GetSlotBUnbootableCounter(){
	return DevInfo.slot_b_unbootable_counter;
}

EFI_STATUS SetSlotBUnbootableCounter(UINT32 counter){
	EFI_STATUS Status = EFI_SUCCESS;
	DevInfo.slot_b_unbootable_counter = counter;
	Status = ReadWriteDeviceInfo(WRITE_CONFIG, &DevInfo, sizeof(DevInfo));
	DEBUG((EFI_D_ERROR, "[ABL] DevInfo.slot_b_unbootable_counter = %d\n", counter));
	return Status;
}

UINT32 GetSlotBRetryCounter(){
	return DevInfo.slot_b_retry_counter;
}

EFI_STATUS SetSlotBRetryCounter(UINT32 counter){
	EFI_STATUS Status = EFI_SUCCESS;
	DevInfo.slot_b_retry_counter = counter;
	Status = ReadWriteDeviceInfo(WRITE_CONFIG, &DevInfo, sizeof(DevInfo));
	DEBUG((EFI_D_ERROR, "[ABL] DevInfo.slot_b_retry_counter = %d\n", counter));
	return Status;
}
// --- ASUS_BSP : add for unbootable_counter and retry_counter

// +++ ASUS_BSP : add for boot count
UINT32 GetBootCounter(){
	return DevInfo.boot_count;
}

EFI_STATUS SetBootCounter(UINT32 counter){
	EFI_STATUS Status = EFI_SUCCESS;
	DevInfo.boot_count= counter;
	Status = ReadWriteDeviceInfo(WRITE_CONFIG, &DevInfo, sizeof(DevInfo));
	DEBUG((EFI_D_ERROR, "[ABL] DevInfo.boot_count = %d\n", counter));
	return Status;
}
// --- ASUS_BSP : add for boot count

// +++ ASUS_BSP : add for lock count
UINT32 GetLockCounter(){
	return DevInfo.lock_count;
}

EFI_STATUS SetLockCounter(UINT32 counter){
	EFI_STATUS Status = EFI_SUCCESS;
	DevInfo.lock_count= counter;
	Status = ReadWriteDeviceInfo(WRITE_CONFIG, &DevInfo, sizeof(DevInfo));
	DEBUG((EFI_D_ERROR, "[ABL] DevInfo.lock_count = %d\n", counter));
	return Status;
}
// --- ASUS_BSP : add for lock count

// +++ ASUS_BSP : add for dm-verity counter
UINT32 GetTotalDmVerityCounter(){
	return DevInfo.total_dm_verity_counter;
}

EFI_STATUS SetTotalDmVerityCounter(UINT32 counter){
	EFI_STATUS Status = EFI_SUCCESS;
	DevInfo.total_dm_verity_counter = counter;
	Status = ReadWriteDeviceInfo(WRITE_CONFIG, &DevInfo, sizeof(DevInfo));
	DEBUG((EFI_D_ERROR, "[ABL] DevInfo.total_dm_verity_counter = %d\n", counter));
	return Status;
}

UINT32 GetDmVerityCounter(){
	return DevInfo.dm_verity_counter;
}

EFI_STATUS SetDmVerityCounter(UINT32 counter){
	EFI_STATUS Status = EFI_SUCCESS;
	DevInfo.dm_verity_counter = counter;
	Status = ReadWriteDeviceInfo(WRITE_CONFIG, &DevInfo, sizeof(DevInfo));
	DEBUG((EFI_D_ERROR, "[ABL] DevInfo.dm_verity_counter = %d\n", counter));
	return Status;
}
// --- ASUS_BSP : add for dm-verity counter

// +++ ASUS_BSP : add for verify_vbmeta_ret
UINT32 GetVerifyVbmetaRet(){
	return DevInfo.verify_vbmeta_ret;
}

EFI_STATUS SetVerifyVbmetaRet(UINT32 ret){
	EFI_STATUS Status = EFI_SUCCESS;
	DevInfo.verify_vbmeta_ret = ret;
	Status = ReadWriteDeviceInfo(WRITE_CONFIG, &DevInfo, sizeof(DevInfo));
	DEBUG((EFI_D_ERROR, "[ABL] DevInfo.verify_vbmeta_ret = %d\n", ret));
	return Status;
}
// --- ASUS_BSP : add for verify_vbmeta_ret

// +++ ASUS_BSP : add for user build menu
BOOLEAN is_reboot_mode = FALSE;
BOOLEAN GetRebootMode(VOID)
{
	DEBUG((EFI_D_ERROR, "[ABL] GetRebootMode : is_reboot_mode = %d\n", is_reboot_mode));
	return is_reboot_mode;
}

VOID SetRebootMode(BOOLEAN status)
{
	is_reboot_mode = status;
	DEBUG((EFI_D_ERROR, "[ABL] SetRebootMode : is_reboot_mode = %d\n", is_reboot_mode));
}
// --- ASUS_BSP : add for user build menu

// +++ ASUS_BSP : add for check factory crc
unsigned long GetFactoryCRC(VOID){
	return DevInfo.factory_crc;
}

EFI_STATUS SetFactoryCRC(unsigned long crc_value){
	EFI_STATUS Status = EFI_SUCCESS;
	DevInfo.factory_crc = crc_value;
	Status = ReadWriteDeviceInfo(WRITE_CONFIG, &DevInfo, sizeof(DevInfo));
	DEBUG((EFI_D_ERROR, "[ABL] DevInfo.factory_crc = %d\n", crc_value));
	return Status;
}
// --- ASUS_BSP : add for check factory crc

// +++ ASUS_BSP : add for logcat-asdf sevices
BOOLEAN GetLogcatAsdfOn()
{
       return DevInfo.is_logcat_asdf_on;
}

EFI_STATUS SetLogcatAsdfOn(BOOLEAN State)
{
       EFI_STATUS Status = EFI_SUCCESS;

       DevInfo.is_logcat_asdf_on = State;
       Status = ReadWriteDeviceInfo(WRITE_CONFIG, &DevInfo, sizeof(DevInfo));
       if (Status != EFI_SUCCESS)
       {
               DEBUG((EFI_D_ERROR, "Unable set the DevInfo.is_logcat_asdf_on value: %r\n", Status));
               return Status;
       }
       return Status;
}
// --- ASUS_BSP : add for logcat-asdf sevices

BOOLEAN IsDebugUnlocked(VOID){
	return DevInfo.is_debug_unlocked;
}

EFI_STATUS SetDeviceDebugUnlockValue(BOOLEAN State){
	EFI_STATUS Status = EFI_SUCCESS;
	if(IsDebugUnlocked() != State){
		DevInfo.is_debug_unlocked = State;
		Status = ReadWriteDeviceInfo(WRITE_CONFIG, &DevInfo, sizeof(DevInfo));
		if(Status != EFI_SUCCESS) {
			DEBUG((EFI_D_ERROR, "[ABL] Unable set debug unlock value: %r\n", Status));
			return Status;
		}
	}
	
	return Status;
}

BOOLEAN GetUartStatus(VOID)
{
        return DevInfo.is_uart_on;
}

EFI_STATUS SetUartStatus(BOOLEAN uart_status)
{
        EFI_STATUS Status = EFI_SUCCESS;
        DevInfo.is_uart_on = uart_status;
        Status = ReadWriteDeviceInfo(WRITE_CONFIG, &DevInfo, sizeof(DevInfo));
        DEBUG((EFI_D_ERROR, "[ABL] Set UART-ON = %d\n", DevInfo.is_uart_on));
        return Status;
}

// +++ ASUS_BSP : add for snapshot check counter
UINT32 GetSnapshotCheckCounter(){
	return DevInfo.snapshot_check_counter;
}

EFI_STATUS SetSnapshotCheckCounter(UINT32 counter){
	EFI_STATUS Status = EFI_SUCCESS;
	DevInfo.snapshot_check_counter = counter;
	Status = ReadWriteDeviceInfo(WRITE_CONFIG, &DevInfo, sizeof(DevInfo));
	DEBUG((EFI_D_ERROR, "[ABL] DevInfo.snapshot_check_counter = %d\n", counter));
	return Status;
}
// --- ASUS_BSP : add for snapshot check counter

////////////////////////////////////////////////////////////////////////
// --- ROG7 : PROJECT COMMON FUNCTION
////////////////////////////////////////////////////////////////////////
#endif




EFI_STATUS DeviceInfoInit (VOID)
{
  EFI_STATUS Status = EFI_SUCCESS;

  if (FirstReadDevInfo) {
    Status =
        ReadWriteDeviceInfo (READ_CONFIG, (VOID *)&DevInfo, sizeof (DevInfo));
    if (Status != EFI_SUCCESS) {
      DEBUG ((EFI_D_ERROR, "Unable to Read Device Info: %r\n", Status));
      return Status;
    }

    FirstReadDevInfo = FALSE;
  }

  if (CompareMem (DevInfo.magic, DEVICE_MAGIC, DEVICE_MAGIC_SIZE)) {
    DEBUG ((EFI_D_ERROR, "Device Magic does not match\n"));
    gBS->SetMem (&DevInfo, sizeof (DevInfo), 0);
    gBS->CopyMem (DevInfo.magic, DEVICE_MAGIC, DEVICE_MAGIC_SIZE);
    DevInfo.user_public_key_length = 0;
    gBS->SetMem (DevInfo.rollback_index, sizeof (DevInfo.rollback_index), 0);
    gBS->SetMem (DevInfo.user_public_key, sizeof (DevInfo.user_public_key), 0);
    if (IsSecureBootEnabled ()) {
      DevInfo.is_unlocked = FALSE;
      DevInfo.is_unlock_critical = FALSE;
    } else {
      DevInfo.is_unlocked = TRUE;
      DevInfo.is_unlock_critical = TRUE;
    }
    DevInfo.is_charger_screen_enabled = FALSE;
    DevInfo.verity_mode = TRUE;
    Status =
        ReadWriteDeviceInfo (WRITE_CONFIG, (VOID *)&DevInfo, sizeof (DevInfo));
    if (Status != EFI_SUCCESS) {
      DEBUG ((EFI_D_ERROR, "Unable to Write Device Info: %r\n", Status));
      return Status;
    }
  }

  return Status;
}

EFI_STATUS
ReadRollbackIndex (UINT32 Loc, UINT64 *RollbackIndex)
{
  EFI_STATUS Status = EFI_SUCCESS;

  if (FirstReadDevInfo) {
    Status = EFI_NOT_STARTED;
    DEBUG ((EFI_D_ERROR, "ReadRollbackIndex DeviceInfo not initalized \n"));
    return Status;
  }

  if (Loc >= ARRAY_SIZE (DevInfo.rollback_index)) {
    Status = EFI_INVALID_PARAMETER;
    DEBUG ((EFI_D_ERROR, "ReadRollbackIndex Loc out of range, "
                         "index: %d, array len: %d\n",
            Loc, ARRAY_SIZE (DevInfo.rollback_index)));
    return Status;
  }

  *RollbackIndex = DevInfo.rollback_index[Loc];
  return Status;
}

EFI_STATUS
WriteRollbackIndex (UINT32 Loc, UINT64 RollbackIndex)
{
  EFI_STATUS Status = EFI_SUCCESS;

  if (FirstReadDevInfo) {
    Status = EFI_NOT_STARTED;
    DEBUG ((EFI_D_ERROR, "WriteRollbackIndex DeviceInfo not initalized \n"));
    return Status;
  }

  if (Loc >= ARRAY_SIZE (DevInfo.rollback_index)) {
    Status = EFI_INVALID_PARAMETER;
    DEBUG ((EFI_D_ERROR, "WriteRollbackIndex Loc out of range, "
                         "index: %d, array len: %d\n",
            Loc, ARRAY_SIZE (DevInfo.rollback_index)));
    return Status;
  }

  DevInfo.rollback_index[Loc] = RollbackIndex;
  Status =
      ReadWriteDeviceInfo (WRITE_CONFIG, (VOID *)&DevInfo, sizeof (DevInfo));
  if (Status != EFI_SUCCESS) {
    DEBUG ((EFI_D_ERROR, "Unable to Write Device Info: %r\n", Status));
    return Status;
  }
  return Status;
}

EFI_STATUS
StoreUserKey (CHAR8 *UserKey, UINT32 UserKeySize)
{
  EFI_STATUS Status = EFI_SUCCESS;

  if (FirstReadDevInfo) {
    Status = EFI_NOT_STARTED;
    DEBUG ((EFI_D_ERROR, "StoreUserKey DeviceInfo not initalized \n"));
    return Status;
  }

  if (UserKeySize > ARRAY_SIZE (DevInfo.user_public_key)) {
    DEBUG ((EFI_D_ERROR, "StoreUserKey, UserKeySize too large!\n"));
    return EFI_OUT_OF_RESOURCES;
  }

  gBS->CopyMem (DevInfo.user_public_key, UserKey, UserKeySize);
  DevInfo.user_public_key_length = UserKeySize;
  Status =
      ReadWriteDeviceInfo (WRITE_CONFIG, (VOID *)&DevInfo, sizeof (DevInfo));
  if (Status != EFI_SUCCESS) {
    DEBUG ((EFI_D_ERROR, "Unable to Write Device Info: %r\n", Status));
    return Status;
  }
  return Status;
}

EFI_STATUS EraseUserKey (VOID)
{
  EFI_STATUS Status = EFI_SUCCESS;

  if (FirstReadDevInfo) {
    Status = EFI_NOT_STARTED;
    DEBUG ((EFI_D_ERROR, "EraseUserKey DeviceInfo not initalized \n"));
    return Status;
  }

  gBS->SetMem (DevInfo.user_public_key, sizeof (DevInfo.user_public_key), 0);
  DevInfo.user_public_key_length = 0;
  Status =
      ReadWriteDeviceInfo (WRITE_CONFIG, (VOID *)&DevInfo, sizeof (DevInfo));
  if (Status != EFI_SUCCESS) {
    DEBUG ((EFI_D_ERROR, "Unable to Write Device Info: %r\n", Status));
    return Status;
  }
  return Status;
}

EFI_STATUS
GetUserKey (CHAR8 **UserKey, UINT32 *UserKeySize)
{
  if (FirstReadDevInfo) {
    DEBUG ((EFI_D_ERROR, "GetUserKey DeviceInfo not initalized \n"));
    return EFI_NOT_STARTED;
  }

  *UserKey = DevInfo.user_public_key;
  *UserKeySize = DevInfo.user_public_key_length;
  return EFI_SUCCESS;
}


EFI_STATUS ReadPersistentValue (CONST UINT8 *Name, UINTN NameSize,
                                UINT8 *Value, UINTN *ValueSize)
{
   BOOLEAN  NameFound = FALSE;
   UINT32   i;

   if (FirstReadDevInfo) {
      DEBUG ((EFI_D_ERROR, "ReadPersistentValue DeviceInfo not initalized \n"));
      return EFI_NOT_STARTED;
   }

   if ( Name == NULL ||
        NameSize == 0 ||
        ValueSize == NULL  )
      return EFI_INVALID_PARAMETER;

   for ( i = 0; i < MAX_ENTRY_SIZE; ++i )
   {
      if ( DevInfo.persistent_value[i].in_use == 1 &&
          NameSize == DevInfo.persistent_value[i].name_size &&
          CompareMem ( Name, DevInfo.persistent_value[i].name, NameSize ) == 0 )
      {
         NameFound = TRUE;
         break;
      }
   }

   if ( NameFound == FALSE ) {
      return EFI_NOT_FOUND;
   }

   if ( DevInfo.persistent_value[i].value_size == 0 ) {
      return EFI_NOT_FOUND;
   }

   if ( *ValueSize < DevInfo.persistent_value[i].value_size )
   {
      *ValueSize =  DevInfo.persistent_value[i].value_size;
      return EFI_BUFFER_TOO_SMALL;
   }

   if ( Value == NULL ) {
      return EFI_INVALID_PARAMETER;
   }

   gBS->CopyMem ( Value,
                  DevInfo.persistent_value[i].value,
                  DevInfo.persistent_value[i].value_size);
   *ValueSize =  DevInfo.persistent_value[i].value_size;

   return EFI_SUCCESS;
}


EFI_STATUS WritePersistentValue (CONST UINT8 *Name, UINTN NameSize,
                                 CONST UINT8 *Value, UINTN ValueSize)
{
   EFI_STATUS Status = EFI_SUCCESS;
   BOOLEAN    NameFound = FALSE;
   BOOLEAN    SlotFound = FALSE;
   UINT32     NameFoundIndex;
   UINT32     SlotFoundIndex;
   UINT32     i;

   if (FirstReadDevInfo) {
      DEBUG ((EFI_D_ERROR, "ReadPersistentValue DeviceInfo not initalized \n"));
      return EFI_NOT_STARTED;
   }

   if ( Name == NULL ||
        NameSize == 0 ) {
      return EFI_INVALID_PARAMETER;
   }

   if ( NameSize > MAX_NAME_SIZE ) {
      return EFI_NOT_FOUND;
   }

   if ( ValueSize > MAX_VALUE_SIZE ) {
      return EFI_BAD_BUFFER_SIZE;
   }

   for ( i = 0; i < MAX_ENTRY_SIZE; ++i )
   {
      if ( DevInfo.persistent_value[i].in_use == 1 )
      {
          if ( NameSize == DevInfo.persistent_value[i].name_size &&
               CompareMem ( Name, DevInfo.persistent_value[i].name, NameSize )
               == 0 )
          {
             NameFound = TRUE;
             NameFoundIndex = i;
             break;
          }
       }
       else
       {
          if ( SlotFound == FALSE )
          {
             SlotFound = TRUE;
             SlotFoundIndex = i;
          }
       }
   }

   DEBUG ((EFI_D_ERROR, "NameFound %d NameIndex %d SlotFound %d SlotIndex %d\n",
           NameFound, NameFoundIndex, SlotFound, SlotFoundIndex ));

   if ( NameFound == TRUE )
   {
      if ( ValueSize == 0 )
      {
         gBS->SetMem ( &DevInfo.persistent_value[NameFoundIndex],
                       sizeof (persistent_value_type),
                       0 );
      }
      else
      {
         if ( Value == NULL ) {
            return EFI_INVALID_PARAMETER;
         }

         gBS->CopyMem ( (void *)DevInfo.persistent_value[NameFoundIndex].value,
                        (void *)Value,
                         ValueSize);
         DevInfo.persistent_value[NameFoundIndex].value_size =(UINT16)ValueSize;
      }
   }
   else if ( SlotFound == TRUE )
   {
      if ( Value == NULL ) {
         return EFI_INVALID_PARAMETER;
      }

      if ( ValueSize == 0 ) {
         return EFI_SUCCESS;
      }

      gBS->CopyMem ( (void *)DevInfo.persistent_value[SlotFoundIndex].name,
                     (void *)Name,
                      NameSize);
      DevInfo.persistent_value[SlotFoundIndex].name_size = NameSize;
      gBS->CopyMem ( (void *)DevInfo.persistent_value[SlotFoundIndex].value,
                     (void *)Value,
                     ValueSize);
      DevInfo.persistent_value[SlotFoundIndex].value_size = (UINT16)ValueSize;
      DevInfo.persistent_value[SlotFoundIndex].in_use = 1;
   }
   else
   {
      DEBUG ((EFI_D_ERROR, "No more slot available\n"));
      return EFI_INVALID_PARAMETER;
   }

   Status = ReadWriteDeviceInfo ( WRITE_CONFIG,
                                  (VOID *)&DevInfo,
                                  sizeof (DevInfo) );
   if (Status != EFI_SUCCESS) {
      DEBUG ((EFI_D_ERROR, "Unable to Write Device Info: 0x%x\n", Status));
   }

   return Status;
}

EFI_STATUS
SetSnapshotGolden (UINTN Val)
{
  EFI_STATUS Status = EFI_SUCCESS;

  if (FirstReadDevInfo) {
    Status = EFI_NOT_STARTED;
    DEBUG ((EFI_D_ERROR, "Erase swap on restore DeviceInfo not initalized \n"));
    return Status;
  }

  DevInfo.GoldenSnapshot = Val;
  Status = ReadWriteDeviceInfo (WRITE_CONFIG,
                        (VOID *)&DevInfo, sizeof (DevInfo));
  if (Status != EFI_SUCCESS) {
    DEBUG ((EFI_D_ERROR, "Unable to Write Device Info: %r\n", Status));
  }
  return Status;
}
