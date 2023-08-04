/*
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
 */

/*
 *  Changes from Qualcomm Innovation Center are provided under the following license:
 *
 *  Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
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
 *  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
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

#include "AutoGen.h"
#include "BootLinux.h"
#include "BootStats.h"
#include "KeyPad.h"
#include "LinuxLoaderLib.h"
#include <FastbootLib/FastbootMain.h>
#include <Library/DeviceInfo.h>
#include <Library/DrawUI.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/PartitionTableUpdate.h>
#include <Library/ShutdownServices.h>
#include <Library/StackCanary.h>
#include "Library/ThreadStack.h"
#include <Library/HypervisorMvCalls.h>
#include <Library/UpdateCmdLine.h>
#include <Protocol/EFICardInfo.h>

#define MAX_APP_STR_LEN 64
#define MAX_NUM_FS 10
#define DEFAULT_STACK_CHK_GUARD 0xc0c0c0c0

#if HIBERNATION_SUPPORT_NO_AES
VOID BootIntoHibernationImage (BootInfo *Info, BOOLEAN *SetRotAndBootState);
#endif

STATIC BOOLEAN BootReasonAlarm = FALSE;
STATIC BOOLEAN BootIntoFastboot = FALSE;
STATIC BOOLEAN BootIntoRecovery = FALSE;
UINT64 FlashlessBootImageAddr = 0;
#ifdef ASUS_BUILD
#include "abl.h"
#include "ShutdownServices.h"
#include <Library/VerifiedBootMenu.h>
#include "SecRSATestApp.h" // +++ ASUS_BSP : add for user unlock
#include "Recovery.h"

// +++ ASUS_BSP : add for reboot reason
UINT32 reboot_reason = NORMAL_MODE;
char cmd_asus_info[64]       = {0};
// +++ ASUS_BSP : add for set permissive cmdline
extern char cmd_selinux[64];
//STATIC CHAR8 SnapshotMergeState[MAX_RSP_SIZE];
#endif

#ifdef ASUS_AI2205_BUILD
extern BOOLEAN EnterChargingMode;
#endif

// This function is used to Deactivate MDTP by entering recovery UI
STATIC EFI_STATUS MdtpDisable (VOID)
{
  BOOLEAN MdtpActive = FALSE;
  EFI_STATUS Status = EFI_SUCCESS;
  QCOM_MDTP_PROTOCOL *MdtpProtocol;

  if (FixedPcdGetBool (EnableMdtpSupport)) {
    Status = IsMdtpActive (&MdtpActive);

    if (EFI_ERROR (Status))
      return Status;

    if (MdtpActive) {
      Status = gBS->LocateProtocol (&gQcomMdtpProtocolGuid, NULL,
                                    (VOID **)&MdtpProtocol);
      if (EFI_ERROR (Status)) {
        DEBUG ((EFI_D_ERROR, "Failed to locate MDTP protocol, Status=%r\n",
                Status));
        return Status;
      }
      /* Perform Local Deactivation of MDTP */
      Status = MdtpProtocol->MdtpDeactivate (MdtpProtocol, FALSE);
    }
  }

  return Status;
}

STATIC UINT8
GetRebootReason (UINT32 *ResetReason)
{
  EFI_RESETREASON_PROTOCOL *RstReasonIf;
  EFI_STATUS Status;

  Status = gBS->LocateProtocol (&gEfiResetReasonProtocolGuid, NULL,
                                (VOID **)&RstReasonIf);
  if (Status != EFI_SUCCESS) {
    DEBUG ((EFI_D_ERROR, "Error locating the reset reason protocol\n"));
    return Status;
  }

  RstReasonIf->GetResetReason (RstReasonIf, ResetReason, NULL, NULL);
  if (RstReasonIf->Revision >= EFI_RESETREASON_PROTOCOL_REVISION)
    RstReasonIf->ClearResetReason (RstReasonIf);
  return Status;
}

BOOLEAN IsABRetryCountUpdateRequired (VOID)
{
  BOOLEAN BatteryStatus;

  /* Check power off charging */
  TargetPauseForBatteryCharge (&BatteryStatus);

  /* Do not decrement bootable retry count in below states:
     * fastboot, fastbootd, charger, recovery
     */
  if ((BatteryStatus &&
       IsChargingScreenEnable ()) ||
       BootIntoFastboot ||
       BootIntoRecovery) {
    return FALSE;
  }
  return TRUE;
}

/**
  Linux Loader Application EntryPoint

  @param[in] ImageHandle    The firmware allocated handle for the EFI image.
  @param[in] SystemTable    A pointer to the EFI System Table.

  @retval EFI_SUCCESS       The entry point is executed successfully.
  @retval other             Some error occurs when executing this entry point.

 **/

EFI_STATUS EFIAPI  __attribute__ ( (no_sanitize ("safe-stack")))
LinuxLoaderEntry (IN EFI_HANDLE ImageHandle, IN EFI_SYSTEM_TABLE *SystemTable)
{
  EFI_STATUS Status;

  UINT32 BootReason = NORMAL_MODE;
  UINT32 KeyPressed = SCAN_NULL;
  UINT32 dm_verity_boot_count=0;
  UINT32 DM_VERITY_RETRY_MAX=5;
  UINT32 SnapshotCheckCounter = 0;
  /* MultiSlot Boot */
  BOOLEAN MultiSlotBoot = FALSE;
  /* Flashless Boot */
  BOOLEAN FlashlessBoot = FALSE;
  EFI_MEM_CARDINFO_PROTOCOL *CardInfo = NULL;
  /* set ROT and BootSatte only once per boot*/
  BOOLEAN SetRotAndBootState = FALSE;

  DEBUG ((EFI_D_INFO, "Loader Build Info: %a %a\n", __DATE__, __TIME__));
  DEBUG ((EFI_D_VERBOSE, "LinuxLoader Load Address to debug ABL: 0x%llx\n",
         (UINTN)LinuxLoaderEntry & (~ (0xFFF))));
  DEBUG ((EFI_D_VERBOSE, "LinuxLoaderEntry Address: 0x%llx\n",
         (UINTN)LinuxLoaderEntry));

  Status = InitThreadUnsafeStack ();

  if (Status != EFI_SUCCESS) {
    DEBUG ((EFI_D_ERROR, "Unable to Allocate memory for Unsafe Stack: %r\n",
            Status));
    goto stack_guard_update_default;
  }

  StackGuardChkSetup ();

  BootStatsSetTimeStamp (BS_BL_START);

  /* Check if memory card is present; goto flashless if not */
  Status = gBS->LocateProtocol (&gEfiMemCardInfoProtocolGuid, NULL,
                                  (VOID **)&CardInfo);
  if (EFI_ERROR (Status)) {
    FlashlessBootImageAddr = BASE_ADDRESS;
    FlashlessBoot = TRUE;
    /* In flashless boot avoid all access to secondary storage during boot */
    goto flashless_boot;
  }

  // Initialize verified boot & Read Device Info
  Status = DeviceInfoInit ();
  if (Status != EFI_SUCCESS) {
    DEBUG ((EFI_D_ERROR, "Initialize the device info failed: %r\n", Status));
    goto stack_guard_update_default;
  }

#ifdef ASUS_BUILD
  /***** ASUS_ABL_INIT *****/
  Status = ASUS_ABL_INIT();
  if (Status != EFI_SUCCESS)
  {
    DEBUG((EFI_D_ERROR, "[ABL] LinuxLoaderEntry- abl_init failed : %r\n", Status));
    return Status;
  }
#endif

  Status = EnumeratePartitions ();

  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "LinuxLoader: Could not enumerate partitions: %r\n",
            Status));
    goto stack_guard_update_default;
  }

  UpdatePartitionEntries ();
  /*Check for multislot boot support*/
  MultiSlotBoot = PartitionHasMultiSlot ((CONST CHAR16 *)L"boot");
  if (MultiSlotBoot) {
    DEBUG ((EFI_D_VERBOSE, "Multi Slot boot is supported\n"));
    FindPtnActiveSlot ();
  }
  

#ifdef ASUS_BUILD
  // ASUS_BSP: check if app unlocked device +++
  if(IsSecureBootEnabled() == TRUE && IsUnlocked() == TRUE && IsAuthorized()==FALSE && IsDebugUnlocked()==FALSE){
    copy_asuskey3_to_devcfg();
  }
  // ASUS_BSP: check if app unlocked device ---
  
  // +++ ASUS_BSP : add for wipe-data by recovery and enter fastboot mode
  if(GetOemFactoryReset2Flag())
  {
    BootIntoFastboot = TRUE;
  }
  // --- ASUS_BSP : add for wipe-data by recovery and enter fastboot mode
#endif

  Status = GetKeyPress (&KeyPressed);
  if (Status == EFI_SUCCESS) {
    if (KeyPressed == SCAN_UP)
      BootIntoFastboot = TRUE;
    if (KeyPressed == SCAN_HOME) //vol+ and pwr key
      BootIntoFastboot = TRUE;
#ifndef ASUS_BUILD
    if (KeyPressed == SCAN_UP)
      BootIntoRecovery = TRUE;
#endif
    if ((KeyPressed == SCAN_ESC) && !TargetBuildVariantUser ())
      RebootDevice (EMERGENCY_DLOAD);
  } else if (Status == EFI_DEVICE_ERROR) {
    DEBUG ((EFI_D_ERROR, "Error reading key status: %r\n", Status));
    goto stack_guard_update_default;
  }

  // check for reboot mode
  Status = GetRebootReason (&BootReason);
  if (Status != EFI_SUCCESS) {
    DEBUG ((EFI_D_ERROR, "Failed to get Reboot reason: %r\n", Status));
    goto stack_guard_update_default;
  }

#ifdef ASUS_BUILD
  reboot_reason = BootReason;

  DEBUG((EFI_D_INFO, "\n"));
  DEBUG((EFI_D_INFO, "======================================================= \n"));
  DEBUG((EFI_D_INFO, "  ASUS Reboot Reason : %u \n",BootReason));
  DEBUG((EFI_D_INFO, "======================================================= \n"));
#endif

  //ASUS BSP++++ dm verity fail retry
  switch (BootReason) {
  case DM_VERITY_LOGGING:
  //case DM_VERITY_ENFORCING:
  //case DM_VERITY_KEYSCLEAR:
    dm_verity_boot_count=GetDmVerityCounter();
    DEBUG((EFI_D_ERROR, "[ABSP] dm_verity_boot_count=%d\n", dm_verity_boot_count));
    if(dm_verity_boot_count < DM_VERITY_RETRY_MAX){
        dm_verity_boot_count++;
        SetDmVerityCounter(dm_verity_boot_count);
    }else if(dm_verity_boot_count == DM_VERITY_RETRY_MAX){
        /*
        if (IsVirtualAbOtaSupported ()) {
            switch (GetSnapshotMergeStatus ()) {
              case SNAPSHOTTED:
                DEBUG((EFI_D_ERROR, "[ABSP] SnapshotMergeStatus:SNAPSHOTTED\n"));
                //Status = DisplayVerifiedBootMenu (DISPLAY_MENU_RED_FOTA_SNAP);
                break;
              case MERGING:
                DEBUG((EFI_D_ERROR, "[ABSP] SnapshotMergeStatus:MERGING\n"));
                //Status = DisplayVerifiedBootMenu (DISPLAY_MENU_RED_FOTA_MERG);
                break;
              default:
                DEBUG((EFI_D_ERROR, "[ABSP] SnapshotMergeStatus:NONE\n"));
                //Status = DisplayVerifiedBootMenu (DISPLAY_MENU_RED_FOTA_NONE);
                break;
            }
        }*/
        Status = DisplayVerifiedBootMenu (DISPLAY_MENU_RED_DM_VERITY);
        if (Status != EFI_SUCCESS) {
            DEBUG ((EFI_D_INFO,
                  "Your device is corrupt. It can't be trusted and will not boot."
                  "\nYour device will shutdown in 10s\n"));
        }
        SetDmVerityCounter(0);
        DEBUG((EFI_D_ERROR, "[ABSP] dm verity fail, shutdown after 10sec\n"));
        MicroSecondDelay (10000000);
        ShutdownDevice ();
    }else{
        SetDmVerityCounter(0);
    }
    break;
  default:
    SetDmVerityCounter(0);
    break;
  }
  //ASUS BSP+++

  switch (BootReason) {
  case FASTBOOT_MODE:
    BootIntoFastboot = TRUE;
#ifdef ASUS_BUILD
    SetRebootMode(TRUE); // +++ ASUS_BSP : add for user build menu
#endif
    break;
  case RECOVERY_MODE:
    BootIntoRecovery = TRUE;
    break;
  case ALARM_BOOT:
    BootReasonAlarm = TRUE;
    break;
  case DM_VERITY_ENFORCING:
    // write to device info
    Status = EnableEnforcingMode (TRUE);
    if (Status != EFI_SUCCESS)
      goto stack_guard_update_default;
    break;
  case DM_VERITY_LOGGING:
    /* Disable MDTP if it's Enabled through Local Deactivation */
    Status = MdtpDisable ();
    if (EFI_ERROR (Status) && Status != EFI_NOT_FOUND) {
      DEBUG ((EFI_D_ERROR, "MdtpDisable Returned error: %r\n", Status));
      goto stack_guard_update_default;
    }
    // write to device info
    Status = EnableEnforcingMode (FALSE);
    if (Status != EFI_SUCCESS)
      goto stack_guard_update_default;

    break;
  case DM_VERITY_KEYSCLEAR:
    Status = ResetDeviceState ();
    if (Status != EFI_SUCCESS) {
      DEBUG ((EFI_D_ERROR, "VB Reset Device State error: %r\n", Status));
      goto stack_guard_update_default;
    }
    break;

#ifdef ASUS_BUILD
  // +++ ASUS_BSP : add for adb enable
  case ENABLE_ADB_MODE:
    DEBUG((EFI_D_VERBOSE, "+++++ ENABLE_ADB_MODE +++++\n"));
    AsciiSPrint(cmd_asus_info, sizeof(cmd_asus_info), " ADB=Y androidboot.adb.enable=1");
    DEBUG((EFI_D_INFO, "cmd_asus_info = %a\n", cmd_asus_info));
    DEBUG((EFI_D_VERBOSE, "----- ENABLE_ADB_MODE -----\n"));
    break;
  // --- ASUS_BSP : add for adb enable

  // +++ ASUS_BSP : add for adb reboot shutdown
  case ASUS_SHUTDOWN:
    DEBUG((EFI_D_ERROR, "+++++ ADBSHUTDOWN +++++\n"));
    DEBUG((EFI_D_ERROR, " call ASUS_Shutdown function \n"));
    ASUS_ShutdownDevice();
    DEBUG((EFI_D_ERROR, "----- ADBSHUTDOWN -----\n"));
    break;
  // --- ASUS_BSP :  add for adb reboot shutdown

  // +++ ASUS_BSP : add for enter shipping mode
  case ASUS_SHIPMODE:
    DEBUG((EFI_D_ERROR, "+++++ ADBSHIPMODE +++++\n"));
    DEBUG((EFI_D_ERROR, " call EnterShippingMode function \n"));
    EnterShippingMode();
    DEBUG((EFI_D_ERROR, "----- ADBSHIPMODE -----\n"));
    break;
  // --- ASUS_BSP : add for enter shipping mode

  // +++ ASUS_BSP : add for user unlock
  case ASUS_UNLOCK:
    DEBUG((EFI_D_ERROR, "+++++ ASUS_UNLOCK +++++\n"));
    if(is_unlock(USER_UNLOCK))
    {
      SetDeviceUnlockValue(UNLOCK, TRUE);
      copy_asuskey3_to_devcfg();
//#if defined ASUS_AI2205_BUILD	
//      SetDeviceDebugUnlockValue(TRUE);
//#endif
      DEBUG((EFI_D_INFO, "\n"));
      DEBUG((EFI_D_INFO, "============================\n"));
      DEBUG((EFI_D_INFO, "DEVICE UNLOCK (FACTORY REST): %d \n",IsUnlocked()));
      DEBUG((EFI_D_INFO, "============================\n"));
      DEBUG((EFI_D_INFO, "\n"));
      RebootDevice(RECOVERY_MODE);
    }
    else
    {
      DEBUG((EFI_D_INFO, "\n"));
      DEBUG((EFI_D_INFO, "============================\n"));
      DEBUG((EFI_D_INFO, "DEVICE UNLOCK FAIL : %d \n",IsUnlocked()));
      DEBUG((EFI_D_INFO, "============================\n"));
      DEBUG((EFI_D_INFO, "\n"));
    }
    DEBUG((EFI_D_ERROR, "----- ASUS_UNLOCK -----\n"));
    break;

  // +++ ASUS_BSP : re-partition from gpt to partition:0 for add rawdump partition
  case ASUS_RE_PARTITION:
    RePartition();
    RebootDevice(RECOVERY_MODE);
    break;
  // --- ASUS_BSP : re-partition from gpt to partition:0 for add rawdump partition

  // +++ ASUS_BSP : add for set permissive cmdline
  case SET_PERMISSIVE_MODE:
    DEBUG((EFI_D_VERBOSE, "+++++ SET_PERMISSIVE_MODE +++++\n"));
    AsciiSPrint(cmd_selinux, sizeof(cmd_selinux), " androidboot.selinux=permissive selinux=0");
    DEBUG((EFI_D_INFO, "cmd_selinux = %a\n", cmd_selinux));
    DEBUG((EFI_D_VERBOSE, "----- SET_PERMISSIVE_MODE -----\n"));
    break;
  // --- ASUS_BSP : add for set permissive cmdline
#endif

  default:
    if (BootReason != NORMAL_MODE) {
      DEBUG ((EFI_D_ERROR,
             "Boot reason: 0x%x not handled, defaulting to Normal Boot\n",
             BootReason));
    }
    break;
  }

  Status = RecoveryInit (&BootIntoRecovery);
  if (Status != EFI_SUCCESS)
    DEBUG ((EFI_D_VERBOSE, "RecoveryInit failed ignore: %r\n", Status));

flashless_boot:
  /* Populate board data required for fastboot, dtb selection and cmd line */
  Status = BoardInit ();
  if (Status != EFI_SUCCESS) {
    DEBUG ((EFI_D_ERROR, "Error finding board information: %r\n", Status));
    return Status;
  }

  DEBUG ((EFI_D_INFO, "KeyPress:%u, BootReason:%u\n", KeyPressed, BootReason));
  DEBUG ((EFI_D_INFO, "Fastboot=%d, Recovery:%d\n",
                                          BootIntoFastboot, BootIntoRecovery));
  if (!GetVmData ()) {
    DEBUG ((EFI_D_ERROR, "VM Hyp calls not present\n"));
  }

  if (BootIntoFastboot) {
      goto fastboot;
  }
  else {
	//ABSP++++
    //if (IsVirtualAbOtaSupported ()){
        if (GetSnapshotMergeStatus() == MERGING) {
             SnapshotCheckCounter = GetSnapshotCheckCounter();
             if(SnapshotCheckCounter >= 7) {
                  DEBUG((EFI_D_ERROR, "[ABL] SnapshotCheckCounter >= 7, shutdown after 10s!!\n"));
                  DisplayVerifiedBootMenu(DISPLAY_MENU_RED_SNAPSHOT_CHECK);
                  MicroSecondDelay(10000000);
                  Status = SetSnapshotCheckCounter(0);
                  ShutdownDevice();
             } else {
                  DEBUG((EFI_D_ERROR, "[ABL] SnapshotMergeStatus==MERGING, SnapshotCheckCounter++\n"));
                  SnapshotCheckCounter++;
                  Status = SetSnapshotCheckCounter(SnapshotCheckCounter);
             }
        } else {
             Status = SetSnapshotCheckCounter(0);
        }
    //}
    //ABSP++
    BootInfo Info = {0};
    Info.MultiSlotBoot = MultiSlotBoot;
    Info.BootIntoRecovery = BootIntoRecovery;
    Info.BootReasonAlarm = BootReasonAlarm;
    Info.FlashlessBoot = FlashlessBoot;
  #if HIBERNATION_SUPPORT_NO_AES
    BootIntoHibernationImage (&Info, &SetRotAndBootState);
  #endif
    Status = LoadImageAndAuth (&Info, FALSE, SetRotAndBootState);
    if (Status != EFI_SUCCESS) {
      DEBUG ((EFI_D_ERROR, "LoadImageAndAuth failed: %r\n", Status));
      goto fastboot;
    }

    BootLinux (&Info);
  }

fastboot:
  if (FlashlessBoot) {
    DEBUG ((EFI_D_ERROR, "No fastboot support for flashless chipsets,"
                               " Infinte loop\n"));
    while (1);
  }
  DEBUG ((EFI_D_INFO, "Launching fastboot\n"));
  Status = FastbootInitialize ();
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "Failed to Launch Fastboot App: %d\n", Status));
    goto stack_guard_update_default;
  }

stack_guard_update_default:
  /*Update stack check guard with defualt value then return*/
  __stack_chk_guard = DEFAULT_STACK_CHK_GUARD;

  return Status;
}
