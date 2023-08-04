/* Copyright (c) 2016-2020, The Linux Foundation. All rights reserved.
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

#include "AutoGen.h"
#include <Library/BaseLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/DebugLib.h>
#include <Library/Debug.h>
#include <Library/DeviceInfo.h>
#include <Library/DrawUI.h>
#include <Library/FastbootMenu.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/MenuKeysDetection.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UpdateDeviceTree.h>
#include <Library/BootLinux.h>
#include <Protocol/EFIVerifiedBoot.h>
#include <Uefi.h>

STATIC OPTION_MENU_INFO gMenuInfo;

#ifdef ASUS_BUILD
#include "abl.h"
#include <Protocol/EFIBMP.h>

#if defined ASUS_AI2205_BUILD
#include "libavb/libavb.h"
#endif

extern int check_unbootable(int i, int j);

#define SKIP_FIRSTCHAR_IN_SLOT_SUFFIX(Slot)                                    \
  do {                                                                         \
    int i = 0;                                                                 \
    do {                                                                       \
      Slot[i] = Slot[i + 1];                                                   \
      i++;                                                                     \
    } while (i < MAX_SLOT_SUFFIX_SZ - 1);                                      \
  } while (0);

STATIC MENU_MSG_INFO mFastbootOptionTitle[] = {
    {{"START"},
     BIG_FACTOR,
     BGR_GREEN,
     BGR_BLACK,
     OPTION_ITEM,
     0,
     RESTART},
    {{"Restart bootloader"},
     BIG_FACTOR,
     BGR_RED,
     BGR_BLACK,
     OPTION_ITEM,
     0,
     FASTBOOT},
    {{"Recovery mode"},
     BIG_FACTOR,
     BGR_RED,
     BGR_BLACK,
     OPTION_ITEM,
     0,
     RECOVER},
    {{"Power off"},
     BIG_FACTOR,
     BGR_WHITE,
     BGR_BLACK,
     OPTION_ITEM,
     0,
     POWEROFF},
};

STATIC MENU_MSG_INFO mASUSOptionTitle[] = {
    {{"ASUS INFO"},
     BIG_FACTOR,
     BGR_GREEN,
     BGR_BLACK,
     OPTION_ITEM,
     0,
     RESTART},
    {{"SET ACTIVE A"},
     BIG_FACTOR,
     BGR_CYAN,
     BGR_BLACK,
     OPTION_ITEM,
     0,
     SET_ACTIVE_A},
    {{"SET ACTIVE B"},
     BIG_FACTOR,
     BGR_CYAN,
     BGR_BLACK,
     OPTION_ITEM,
     0,
     SET_ACTIVE_B},
     {{"CHECK AVB"},
     BIG_FACTOR,
     BGR_ORANGE,
     BGR_BLACK,
     OPTION_ITEM,
     0,
     CHECK_AVB},
};

STATIC MENU_MSG_INFO mFastbootCommonMsgInfo[] = {
    {{"\nPress volume key to select, "
      "and press power key to select\n\n"},
     COMMON_FACTOR,
     BGR_WHITE,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
    {{"FastBoot Mode"},
     COMMON_FACTOR,
     BGR_RED,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
    {{"PROJECT NAME - "},
     COMMON_FACTOR,
     BGR_WHITE,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
    {{"HW STAGE - "},
     COMMON_FACTOR,
     BGR_WHITE,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
    {{"XBL VERSION - "},
     COMMON_FACTOR,
     BGR_WHITE,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
    {{"ABL VERSION - "},
     COMMON_FACTOR,
     BGR_WHITE,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
    {{"CSC VERSION - "},
     COMMON_FACTOR,
     BGR_WHITE,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
    {{"SSN - "},
     COMMON_FACTOR,
     BGR_WHITE,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
     {{"ISN - "},
     COMMON_FACTOR,
     BGR_WHITE,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
     {{"IMEI - "},
     COMMON_FACTOR,
     BGR_WHITE,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
     {{"IMEI2 - "},
     COMMON_FACTOR,
     BGR_WHITE,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
     {{"CID - "},
     COMMON_FACTOR,
     BGR_WHITE,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
     {{"COUNTRY - "},
     COMMON_FACTOR,
     BGR_WHITE,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
     {{"SECURE BOOT - "},
     COMMON_FACTOR,
     BGR_WHITE,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
     {{"FEATURE ID - "},
     COMMON_FACTOR,
     BGR_WHITE,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
     {{"JTAG ID - "},
     COMMON_FACTOR,
     BGR_WHITE,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
     {{"FTM MODE - "},
     COMMON_FACTOR,
     BGR_WHITE,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
     {{"AVB VERITY - "},
     COMMON_FACTOR,
     BGR_WHITE,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
     {{"CURRENT SLOT - "},
     COMMON_FACTOR,
     BGR_WHITE,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
     {{"BATTERY VOLTAGE - "},
     COMMON_FACTOR,
     BGR_WHITE,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
     {{"UART STATUS - "},
     COMMON_FACTOR,
     BGR_WHITE,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
     {{"DUNGLE UNLOCK - "},
     COMMON_FACTOR,
     BGR_RED,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
    {{"DEVICE STATE - "},
     COMMON_FACTOR,
     BGR_RED,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
    {{"WATERMASK UNLOCK - "},
     COMMON_FACTOR,
     BGR_RED,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
    {{"DEBUG UNLOCK - "},
     COMMON_FACTOR,
     BGR_RED,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
};

STATIC MENU_MSG_LEN mFastbootCommonMsgLen[ARRAY_SIZE (mFastbootCommonMsgInfo)];

// +++ ASUS_BSP : add for ASUS HOT KEY MENU
STATIC MENU_MSG_INFO mAsusCommonMsgInfo[] = {
     {{"\nLong Press Vol down + Power key to Shutdown Device >>>\n"},
     COMMON_FACTOR, BGR_WHITE, BGR_BLACK, COMMON, 0, NOACTION},// 0
     {{"PROJECT NAME - "},//1
     COMMON_FACTOR,
     BGR_WHITE,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
     {{"HW STAGE - "},//2
     COMMON_FACTOR,
     BGR_WHITE,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
     {{"XBL VERSION - "},//3
     COMMON_FACTOR,
     BGR_WHITE,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
     {{"ABL VERSION - "},//4
     COMMON_FACTOR,
     BGR_WHITE,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
     {{"CSC VERSION - "},//5
     COMMON_FACTOR,
     BGR_WHITE,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
     {{"MEM INFO - "},//6
     COMMON_FACTOR,
     BGR_WHITE,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
     {{"CPU FREQ - "},//7
     COMMON_FACTOR,
     BGR_WHITE,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
     {{"SSN - "},//8
     COMMON_FACTOR,
     BGR_WHITE,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},//9
     {{"ISN - "},
     COMMON_FACTOR,
     BGR_WHITE,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
     {{"IMEI - "},//10
     COMMON_FACTOR,
     BGR_WHITE,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
     {{"IMEI2 - "},//11
     COMMON_FACTOR,
     BGR_WHITE,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
     {{"WIFI MAC - "},//12
     COMMON_FACTOR,
     BGR_WHITE,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
     {{"WIFI MAC2 - "},//13
     COMMON_FACTOR,
     BGR_WHITE,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
     {{"BT MAC - "},//14
     COMMON_FACTOR,
     BGR_WHITE,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
     {{"CID - "},//15
     COMMON_FACTOR,
     BGR_WHITE,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
     {{"COUNTRY - "},//16
     COMMON_FACTOR,
     BGR_WHITE,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
     {{"SECURE BOOT - "},//17
     COMMON_FACTOR,
     BGR_WHITE,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
     {{"FEATURE ID - "},//18
     COMMON_FACTOR,
     BGR_WHITE,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
     {{"JTAG ID - "},//19
     COMMON_FACTOR,
     BGR_WHITE,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
     {{"FTM MODE - "},//20
     COMMON_FACTOR,
     BGR_WHITE,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
     {{"AVB VERITY - "},//21
     COMMON_FACTOR,
     BGR_WHITE,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
     {{"CURRENT SLOT - "},//22
     COMMON_FACTOR,
     BGR_WHITE,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
     {{"BATTERY VOLTAGE - "},//23
     COMMON_FACTOR,
     BGR_WHITE,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
      {{"UART STATUS - "},//24
     COMMON_FACTOR,
     BGR_WHITE,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
     {{"DM-VERITY COUNTER - "},//25
     COMMON_FACTOR,
     BGR_WHITE,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
     {{"SLOT A RETRY COUNTER - "},//26
     COMMON_FACTOR,
     BGR_WHITE,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
     {{"SLOT A UNBOOTABLE COUNTER - "},//27
     COMMON_FACTOR,
     BGR_WHITE,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
     {{"SLOT B RETRY COUNTER - "},//28
     COMMON_FACTOR,
     BGR_WHITE,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
     {{"SLOT B UNBOOTABLE COUNTER - "},//29
     COMMON_FACTOR,
     BGR_WHITE,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
     {{"AUTHORIZED2 STATUS - "},//30
     COMMON_FACTOR,
     BGR_WHITE,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
     {{"SETUP WIZARD - "},//31
     COMMON_FACTOR,
     BGR_WHITE,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
     {{"DUNGLE UNLOCK - "},//32
     COMMON_FACTOR,
     BGR_RED,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
     {{"DEVICE STATE - "},//33
     COMMON_FACTOR,
     BGR_RED,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
     {{"WATERMASK UNLOCK - "},//34
     COMMON_FACTOR,
     BGR_RED,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
     {{"DEBUG UNLOCK - "},//35
     COMMON_FACTOR,
     BGR_RED,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
};
// --- ASUS_BSP : add for ASUS HOT KEY MENU

STATIC MENU_MSG_INFO mAsusVerifiedStateMsgInfo[] = {
     {{"\nPress power key to reboot device\n\n"},
     COMMON_FACTOR, BGR_WHITE, BGR_BLACK, COMMON, 0, NOACTION},// 0
     {{"AsusVerifiedState - "},//1
     COMMON_FACTOR,
     BGR_RED,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
     {{"AsusVbmetaVerified - "},//2
     COMMON_FACTOR,
     BGR_SILVER,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
     {{"AsusBootVerified - "},//3
     COMMON_FACTOR,
     BGR_SILVER,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
     {{"AsusDtboVerified - "},//4
     COMMON_FACTOR,
     BGR_SILVER,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
     {{"AsusVendorBootVerified - "},//5
     COMMON_FACTOR,
     BGR_SILVER,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
     {{"AsusInitBootVerified - "},//6
     COMMON_FACTOR,
     BGR_SILVER,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
     {{"DMVeritySate - "},//7
     COMMON_FACTOR,
     BGR_SILVER,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
     {{"DMVerityMergeState - "},//8
     COMMON_FACTOR,
     BGR_SILVER,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
};

STATIC MENU_MSG_LEN mAsusVerifiedStateMsgLen[ARRAY_SIZE (mAsusVerifiedStateMsgInfo)];

BOOLEAN FirstDrawFastbootMenu = TRUE;
BOOLEAN FirstDrawAsusVerifiedStateMenu = TRUE;

#else
STATIC MENU_MSG_INFO mFastbootOptionTitle[] = {
    {{"START"},
     BIG_FACTOR,
     BGR_GREEN,
     BGR_BLACK,
     OPTION_ITEM,
     0,
     RESTART},
    {{"Restart bootloader"},
     BIG_FACTOR,
     BGR_RED,
     BGR_BLACK,
     OPTION_ITEM,
     0,
     FASTBOOT},
    {{"Recovery mode"},
     BIG_FACTOR,
     BGR_RED,
     BGR_BLACK,
     OPTION_ITEM,
     0,
     RECOVER},
    {{"Power off"},
     BIG_FACTOR,
     BGR_WHITE,
     BGR_BLACK,
     OPTION_ITEM,
     0,
     POWEROFF},
    {{"Boot to FFBM"},
     BIG_FACTOR,
     BGR_YELLOW,
     BGR_BLACK,
     OPTION_ITEM,
     0,
     FFBM},
    {{"Boot to QMMI"},
     BIG_FACTOR,
     BGR_YELLOW,
     BGR_BLACK,
     OPTION_ITEM,
     0,
     QMMI},
};

STATIC MENU_MSG_INFO mFastbootCommonMsgInfo[] = {
    {{"\nPress volume key to select, "
      "and press power key to select\n\n"},
     COMMON_FACTOR,
     BGR_WHITE,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
    {{"FastBoot Mode"},
     COMMON_FACTOR,
     BGR_RED,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
    {{"PRODUCT_NAME - "},
     COMMON_FACTOR,
     BGR_WHITE,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
    {{"VARIANT - "},
     COMMON_FACTOR,
     BGR_WHITE,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
    {{"BOOTLOADER VERSION - "},
     COMMON_FACTOR,
     BGR_WHITE,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
    {{"BASEBAND VERSION - "},
     COMMON_FACTOR,
     BGR_WHITE,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
    {{"SERIAL NUMBER - "},
     COMMON_FACTOR,
     BGR_WHITE,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
    {{"SECURE BOOT - "},
     COMMON_FACTOR,
     BGR_WHITE,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
    {{"DEVICE STATE - "},
     COMMON_FACTOR,
     BGR_RED,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
};
#endif

/**
  Update the fastboot option item
  @param[in] OptionItem  The new fastboot option item
  @param[out] pLocation  The pointer of the location
  @retval EFI_SUCCESS	 The entry point is executed successfully.
  @retval other		 Some error occurs when executing this entry point.
 **/
EFI_STATUS
UpdateFastbootOptionItem (UINT32 OptionItem, UINT32 *pLocation)
{
  EFI_STATUS Status = EFI_SUCCESS;
  UINT32 Location = 0;
  UINT32 Height = 0;
  MENU_MSG_INFO *FastbootLineInfo = NULL;

  FastbootLineInfo = AllocateZeroPool (sizeof (MENU_MSG_INFO));
  if (FastbootLineInfo == NULL) {
    DEBUG ((EFI_D_ERROR, "Failed to allocate zero pool.\n"));
    return EFI_OUT_OF_RESOURCES;
  }

  SetMenuMsgInfo (FastbootLineInfo, "__________", COMMON_FACTOR,
                  mFastbootOptionTitle[OptionItem].FgColor,
                  mFastbootOptionTitle[OptionItem].BgColor, LINEATION, Location,
                  NOACTION);
  Status = DrawMenu (FastbootLineInfo, &Height);
  if (Status != EFI_SUCCESS)
    goto Exit;
  Location += Height;

  mFastbootOptionTitle[OptionItem].Location = Location;
  Status = DrawMenu (&mFastbootOptionTitle[OptionItem], &Height);
  if (Status != EFI_SUCCESS)
    goto Exit;
  Location += Height;

  FastbootLineInfo->Location = Location;
  Status = DrawMenu (FastbootLineInfo, &Height);
  if (Status != EFI_SUCCESS)
    goto Exit;
  Location += Height;

Exit:
  FreePool (FastbootLineInfo);
  FastbootLineInfo = NULL;

  if (pLocation != NULL)
    *pLocation = Location;

  return Status;
}

#ifdef ASUS_BUILD
extern EFI_GUID gEfiImageFvNameGuid;
STATIC BOOLEAN ImageFvLoadedFlag = FALSE;
EFI_QCOM_BMP_PROTOCOL *BMPProtocol = NULL;

EFI_STATUS ASUSDrawGraphicMenu(UINT32 OptionItem)
{
  EFI_STATUS Status = EFI_SUCCESS;
  EFI_PIL_PROTOCOL  *PILProtocol = NULL;
  char   *str = NULL;  
  BMPLIB_OPTION pic_opt={0};
  BMPLIB_OPTION_LOCATION_PARAMS Location_Params={0};
  
  //Read country code
  CHAR8 StrTemp[MAX_RSP_SIZE] = "";
  ZeroMem(StrTemp, sizeof(StrTemp));
  GetCountryCode(StrTemp, sizeof(StrTemp));
  
  switch(OptionItem){
    case start_up:
      if(!strcmp("CN", StrTemp)){
        str = AllocatePool (AsciiStrLen(CN_START_UP) + 1);
        AsciiStrnCpyS(str, AsciiStrLen(CN_START_UP)+1 , CN_START_UP, AsciiStrLen(CN_START_UP) + 1);
      }else if(!strcmp("TW", StrTemp)){
        str = AllocatePool (AsciiStrLen(TW_START_UP) + 1);
        AsciiStrnCpyS(str, AsciiStrLen(TW_START_UP)+1 , TW_START_UP, AsciiStrLen(TW_START_UP) + 1);
      }else{
        str = AllocatePool (AsciiStrLen(ENG_START_UP) + 1);
        AsciiStrnCpyS(str, AsciiStrLen(ENG_START_UP)+1 , ENG_START_UP, AsciiStrLen(ENG_START_UP) + 1);
      }      
      break;
    case reboot_bootloader:
      if(!strcmp("CN", StrTemp)){
        str = AllocatePool (AsciiStrLen(CN_REBOOT_bootloader) + 1);
        AsciiStrnCpyS(str, AsciiStrLen(CN_REBOOT_bootloader)+1 , CN_REBOOT_bootloader, AsciiStrLen(CN_REBOOT_bootloader) + 1);
      }else if(!strcmp("TW", StrTemp)){
        str = AllocatePool (AsciiStrLen(TW_REBOOT_bootloader) + 1);
        AsciiStrnCpyS(str, AsciiStrLen(TW_REBOOT_bootloader)+1 , TW_REBOOT_bootloader, AsciiStrLen(TW_REBOOT_bootloader) + 1);
      }else{
        str = AllocatePool (AsciiStrLen(ENG_REBOOT_bootloader) + 1);
        AsciiStrnCpyS(str, AsciiStrLen(ENG_REBOOT_bootloader)+1 , ENG_REBOOT_bootloader, AsciiStrLen(ENG_REBOOT_bootloader) + 1);
      }    
      break;
    case recovery_mode:
      if(!strcmp("CN", StrTemp)){
        str = AllocatePool (AsciiStrLen(CN_RECOVERY_MODE) + 1);
        AsciiStrnCpyS(str, AsciiStrLen(CN_RECOVERY_MODE)+1 , CN_RECOVERY_MODE, AsciiStrLen(CN_RECOVERY_MODE) + 1);
      }else if(!strcmp("TW", StrTemp)){
        str = AllocatePool (AsciiStrLen(TW_RECOVERY_MODE) + 1);
        AsciiStrnCpyS(str, AsciiStrLen(TW_RECOVERY_MODE)+1 , TW_RECOVERY_MODE, AsciiStrLen(TW_RECOVERY_MODE) + 1);
      }else{
        str = AllocatePool (AsciiStrLen(ENG_RECOVERY_MODE) + 1);
        AsciiStrnCpyS(str, AsciiStrLen(ENG_RECOVERY_MODE)+1 , ENG_RECOVERY_MODE, AsciiStrLen(ENG_RECOVERY_MODE) + 1);
      }
      break;
    case power_off:
      if(!strcmp("CN", StrTemp)){
        str = AllocatePool (AsciiStrLen(CN_POWER_OFF) + 1);
        AsciiStrnCpyS(str, AsciiStrLen(CN_POWER_OFF)+1 , CN_POWER_OFF, AsciiStrLen(CN_POWER_OFF) + 1);
      }else if(!strcmp("TW", StrTemp)){
        str = AllocatePool (AsciiStrLen(TW_POWER_OFF) + 1);
        AsciiStrnCpyS(str, AsciiStrLen(TW_POWER_OFF)+1 , TW_POWER_OFF, AsciiStrLen(TW_POWER_OFF) + 1);
      }else{
        str = AllocatePool (AsciiStrLen(ENG_POWER_OFF) + 1);
        AsciiStrnCpyS(str, AsciiStrLen(ENG_POWER_OFF)+1 , ENG_POWER_OFF, AsciiStrLen(ENG_POWER_OFF) + 1);
      }
      break;
    default:
      str = AllocatePool (AsciiStrLen(ENG_START_UP) + 1);
      AsciiStrnCpyS(str, AsciiStrLen(ENG_START_UP)+1 , ENG_START_UP, AsciiStrLen(ENG_START_UP) + 1);
      break;
  }
 
  DEBUG(( EFI_D_ERROR, "ASUSDrawGraphicMenu:: show pic %a \r\n", str));
 
  if(ImageFvLoadedFlag == FALSE) {
    Status = gBS->LocateProtocol (&gEfiPilProtocolGuid,NULL, (VOID **) &PILProtocol);
    if ((EFI_SUCCESS != Status) || (NULL == PILProtocol)) {
       DEBUG(( EFI_D_ERROR, "ASUSDrawGraphicMenu:: locateProtocol failed, %a %r \r\n", __FUNCTION__, Status));
       return Status;
    }

    Status = PILProtocol->ProcessPilImage(L"ImageFv");
    if ((EFI_SUCCESS != Status)) {
       DEBUG(( EFI_D_ERROR, "ASUSDrawGraphicMenu:: process image failed, %a %r \r\n", __FUNCTION__, Status));
       return Status;
    }
    ImageFvLoadedFlag = TRUE;
  }
  
  /* Clear the screen before launch the verified boot menu */
  //gST->ConOut->ClearScreen (gST->ConOut);
  //DrawMenuInit ();
  
  if(NULL == BMPProtocol) {
    Status |= gBS->LocateProtocol( &gEfiBmpProtocolGuid, NULL, (VOID **)&BMPProtocol);
    if( Status == EFI_SUCCESS ){
      DEBUG((EFI_D_ERROR, "[ABL] get BMPProtocol ok\n"));
    }else{
      DEBUG((EFI_D_ERROR, "[ABL] get BMPProtocol fail\n"));
      return Status;
    }
  }
  
  //draw graphic images
  Location_Params.Location = BmplibOptionLocationCustom;
  Location_Params.DestX = 0;
  Location_Params.DestY = 0;
  pic_opt.Type = BmplibOptionTypeLocation;
  pic_opt.Params = &Location_Params;
  
  Status=BMPProtocol->DrawBmpFile(str, &pic_opt, 1, &gEfiImageFvNameGuid);
 
  if(NULL != str ) {
    FreePool(str);
    str = NULL;
  }

  return Status;
}

EFI_STATUS
UpdateASUSOptionItem (UINT32 OptionItem, UINT32 *pLocation)
{
  EFI_STATUS Status = EFI_SUCCESS;
  UINT32 Location = 0;
  UINT32 Height = 0;
  MENU_MSG_INFO *FastbootLineInfo = NULL;

  FastbootLineInfo = AllocateZeroPool (sizeof (MENU_MSG_INFO));
  if (FastbootLineInfo == NULL) {
    DEBUG ((EFI_D_ERROR, "Failed to allocate zero pool.\n"));
    return EFI_OUT_OF_RESOURCES;
  }

  SetMenuMsgInfo (FastbootLineInfo, "__________", COMMON_FACTOR,
                  mASUSOptionTitle[OptionItem].FgColor,
                  mASUSOptionTitle[OptionItem].BgColor, LINEATION, Location,
                  NOACTION);
  Status = DrawMenu (FastbootLineInfo, &Height);
  if (Status != EFI_SUCCESS)
    goto Exit;
  Location += Height;

  mASUSOptionTitle[OptionItem].Location = Location;
  Status = DrawMenu (&mASUSOptionTitle[OptionItem], &Height);
  if (Status != EFI_SUCCESS)
    goto Exit;
  Location += Height;

  FastbootLineInfo->Location = Location;
  Status = DrawMenu (FastbootLineInfo, &Height);
  if (Status != EFI_SUCCESS)
    goto Exit;
  Location += Height;

Exit:
  FreePool (FastbootLineInfo);
  FastbootLineInfo = NULL;

  if (pLocation != NULL)
    *pLocation = Location;

  return Status;
}

EFI_STATUS
UpdateASUSGraphicOptionItem (UINT32 OptionItem, UINT32 *pLocation)
{
  EFI_STATUS Status = EFI_SUCCESS;
  DEBUG ((EFI_D_ERROR, "[ABL] OptionItem = %d\n", OptionItem));
  
  switch(OptionItem){
	  case 0:
	    DEBUG ((EFI_D_ERROR, "[ABL] show start option\n"));
	    break;
	  case 1:
	    DEBUG ((EFI_D_ERROR, "[ABL] show restart bootloader\n"));
	    break;
	  case 2:
	    DEBUG ((EFI_D_ERROR, "[ABL] show recoverty mode\n"));
	    break;
	  case 3:
	    DEBUG ((EFI_D_ERROR, "[ABL] power off\n"));
	    break;
	  default:
	    break;
  }
  
  Status = ASUSDrawGraphicMenu(OptionItem);
  if(Status != EFI_SUCCESS)
	 DEBUG ((EFI_D_ERROR, "[ABL] draw menu failed, %a %r \r\n", __FUNCTION__, Status));
  
  return Status;
}
#endif

/**
  Draw the fastboot menu
  @param[out] OptionMenuInfo  Fastboot option info
  @retval     EFI_SUCCESS     The entry point is executed successfully.
  @retval     other           Some error occurs when executing this entry point.
 **/
STATIC EFI_STATUS
FastbootMenuShowScreen (OPTION_MENU_INFO *OptionMenuInfo)
{
  EFI_STATUS Status = EFI_SUCCESS;
  UINT32 Location = 0;
  UINT32 OptionItem = 0;
  UINT32 Height = 0;
  UINT32 i = 0;
  CHAR8 StrTemp[MAX_RSP_SIZE] = "";
#ifndef ASUS_BUILD
  CHAR8 StrTemp1[MAX_RSP_SIZE] = "";
#endif
  CHAR8 VersionTemp[MAX_VERSION_LEN] = "";
#ifdef ASUS_BUILD
  sys_info sysinfo;
  UINT32 TempNum = 0;
  UINT32 BatteryVoltage = 0;
#endif
  ZeroMem (&OptionMenuInfo->Info, sizeof (MENU_OPTION_ITEM_INFO));

  /* Update fastboot option title */
  OptionMenuInfo->Info.MsgInfo = mFastbootOptionTitle;
  for (i = 0; i < ARRAY_SIZE (mFastbootOptionTitle); i++) {
    OptionMenuInfo->Info.OptionItems[i] = i;
  }
  OptionItem =
      OptionMenuInfo->Info.OptionItems[OptionMenuInfo->Info.OptionIndex];
  Status = UpdateFastbootOptionItem (OptionItem, &Location);
  if (Status != EFI_SUCCESS)
    return Status;

#ifdef ASUS_BUILD
  if (FirstDrawFastbootMenu == TRUE) {
    for (i = 0; i < ARRAY_SIZE (mFastbootCommonMsgInfo); i++) {
      mFastbootCommonMsgLen[i].Length = strlen(mFastbootCommonMsgInfo[i].Msg);
    }
    FirstDrawFastbootMenu = FALSE;
  }

  /* Update fastboot common message */
  for (i = 0; i < ARRAY_SIZE (mFastbootCommonMsgInfo); i++) {
    switch (i) {
    case 0:
    case 1:
      break;
    case 2:
      /* Get PROJECT NAME */
      ZeroMem(StrTemp, sizeof(StrTemp));
      GetProjName(StrTemp, sizeof(StrTemp));
      mFastbootCommonMsgInfo[i].Msg[mFastbootCommonMsgLen[i].Length] = 0;
      AsciiStrnCatS (mFastbootCommonMsgInfo[i].Msg,
                     sizeof(mFastbootCommonMsgInfo[i].Msg), StrTemp, sizeof (StrTemp));
      break;
    case 3:
      /* Get HW STAGE */
      ZeroMem(StrTemp, sizeof(StrTemp));
      GetHWStage(StrTemp, sizeof(StrTemp));
      mFastbootCommonMsgInfo[i].Msg[mFastbootCommonMsgLen[i].Length] = 0;
      AsciiStrnCatS (mFastbootCommonMsgInfo[i].Msg,
                     sizeof(mFastbootCommonMsgInfo[i].Msg), StrTemp, sizeof (StrTemp));
      break;
    case 4:
      /* Get XBL VERSION */
      ZeroMem(VersionTemp, sizeof(VersionTemp));
      GetXBLVersion (VersionTemp, sizeof (VersionTemp));
      mFastbootCommonMsgInfo[i].Msg[mFastbootCommonMsgLen[i].Length] = 0;
      AsciiStrnCatS (mFastbootCommonMsgInfo[i].Msg,
                     sizeof(mFastbootCommonMsgInfo[i].Msg), VersionTemp, sizeof (VersionTemp));
      break;
    case 5:
      /* Get ABL VERSION */
      ZeroMem(VersionTemp, sizeof(VersionTemp));
      GetBootloaderVersion (VersionTemp, sizeof (VersionTemp));
      mFastbootCommonMsgInfo[i].Msg[mFastbootCommonMsgLen[i].Length] = 0;
      AsciiStrnCatS (mFastbootCommonMsgInfo[i].Msg,
                     sizeof(mFastbootCommonMsgInfo[i].Msg), VersionTemp, sizeof (VersionTemp));
      break;
    case 6:
      /* Get CSC VERSION */
      ZeroMem(VersionTemp, sizeof(VersionTemp));
      if(read_sysinfo(&sysinfo) == EFI_SUCCESS)
      {
        AsciiSPrint(VersionTemp, sizeof(VersionTemp), "%a", sysinfo.csc_build_version);
      }
      mFastbootCommonMsgInfo[i].Msg[mFastbootCommonMsgLen[i].Length] = 0;
      AsciiStrnCatS (mFastbootCommonMsgInfo[i].Msg,
                     sizeof(mFastbootCommonMsgInfo[i].Msg), VersionTemp, sizeof (VersionTemp));
      break;
    case 7:
      /* Get SSN */
      ZeroMem(StrTemp, sizeof(StrTemp));
      GetSSNNum(StrTemp, sizeof(StrTemp));
      mFastbootCommonMsgInfo[i].Msg[mFastbootCommonMsgLen[i].Length] = 0;
      AsciiStrnCatS(mFastbootCommonMsgInfo[i].Msg,
                    sizeof(mFastbootCommonMsgInfo[i].Msg), StrTemp, sizeof (StrTemp));
      break;
    case 8:
      /* Get ISN */
      ZeroMem(StrTemp, sizeof(StrTemp));
      GetISNNum(StrTemp, sizeof(StrTemp));
      mFastbootCommonMsgInfo[i].Msg[mFastbootCommonMsgLen[i].Length] = 0;
      AsciiStrnCatS(mFastbootCommonMsgInfo[i].Msg,
                    sizeof(mFastbootCommonMsgInfo[i].Msg), StrTemp, sizeof (StrTemp));
      break;
    case 9:
      /* Get IMEI */
      ZeroMem(StrTemp, sizeof(StrTemp));
      GetIMEINum(StrTemp, sizeof(StrTemp));
      mFastbootCommonMsgInfo[i].Msg[mFastbootCommonMsgLen[i].Length] = 0;
      AsciiStrnCatS(mFastbootCommonMsgInfo[i].Msg,
                    sizeof(mFastbootCommonMsgInfo[i].Msg), StrTemp, sizeof (StrTemp));
      break;
    case 10:
      /* Get IMEI2 */
      ZeroMem(StrTemp, sizeof(StrTemp));
      GetIMEI2Num(StrTemp, sizeof(StrTemp));
      mFastbootCommonMsgInfo[i].Msg[mFastbootCommonMsgLen[i].Length] = 0;
      AsciiStrnCatS(mFastbootCommonMsgInfo[i].Msg,
                    sizeof(mFastbootCommonMsgInfo[i].Msg), StrTemp, sizeof (StrTemp));
      break;
    case 11:
      /* Get CID */
      ZeroMem(StrTemp, sizeof(StrTemp));
      GetCIDName(StrTemp, sizeof(StrTemp));
      mFastbootCommonMsgInfo[i].Msg[mFastbootCommonMsgLen[i].Length] = 0;
      AsciiStrnCatS(mFastbootCommonMsgInfo[i].Msg,
                    sizeof(mFastbootCommonMsgInfo[i].Msg), StrTemp, sizeof (StrTemp));
      break;
    case 12:
      /* Get COUNTRY */
      ZeroMem(StrTemp, sizeof(StrTemp));
      GetCountryCode(StrTemp, sizeof(StrTemp));
      mFastbootCommonMsgInfo[i].Msg[mFastbootCommonMsgLen[i].Length] = 0;
      AsciiStrnCatS(mFastbootCommonMsgInfo[i].Msg,
                    sizeof(mFastbootCommonMsgInfo[i].Msg), StrTemp, sizeof (StrTemp));
      break;
    case 13:
      /* Get SECURE BOOT */
      mFastbootCommonMsgInfo[i].Msg[mFastbootCommonMsgLen[i].Length] = 0;
      AsciiStrnCatS (
          mFastbootCommonMsgInfo[i].Msg, sizeof(mFastbootCommonMsgInfo[i].Msg),
          IsSecureBootEnabled () ? "SB=Y " : "SB=N ",
          IsSecureBootEnabled () ? AsciiStrLen ("SB=Y ") : AsciiStrLen ("SB=N "));
      BOOLEAN SecureDeviceNoRpmb = FALSE;
      IsSecureDeviceNoCheckRpmb(&SecureDeviceNoRpmb);
      AsciiStrnCatS (
          mFastbootCommonMsgInfo[i].Msg, sizeof(mFastbootCommonMsgInfo[i].Msg),
          SecureDeviceNoRpmb ? "/ SBNR=Y" : "/ SBNR=N",
          SecureDeviceNoRpmb ? AsciiStrLen ("/ SBNR=Y") : AsciiStrLen ("/ SBNR=N"));
      break;
    case 14:
      /* Get FEATURE ID */
      TempNum = Get_FEATURE_ID();
      mFastbootCommonMsgInfo[i].Msg[mFastbootCommonMsgLen[i].Length] = 0;
        if (TempNum == 0x6){
          AsciiStrnCatS (mFastbootCommonMsgInfo[i].Msg,
                         sizeof (mFastbootCommonMsgInfo[i].Msg), "001-AB-SM8550_ES1", AsciiStrLen ("001-AB-SM8550_ES1"));
        }else if(TempNum == 0x0){
          AsciiStrnCatS (mFastbootCommonMsgInfo[i].Msg,
                         sizeof (mFastbootCommonMsgInfo[i].Msg), "002-AB-SM8550_ES2", AsciiStrLen ("002-AB-SM8550_ES2"));
        }else if(TempNum == 0x8){
          AsciiStrnCatS (mFastbootCommonMsgInfo[i].Msg,
                         sizeof (mFastbootCommonMsgInfo[i].Msg), "002-AB-SM8550P_ES", AsciiStrLen ("002-AB-SM8550P_ES"));
        }else{
          AsciiStrnCatS (mFastbootCommonMsgInfo[i].Msg,
                         sizeof (mFastbootCommonMsgInfo[i].Msg), "Unknown", AsciiStrLen ("Unknown"));
        }
      break;
    case 15:
      /* Get JTAG ID */
      ZeroMem (StrTemp, sizeof (StrTemp));
      AsciiSPrint (StrTemp, sizeof (StrTemp), "%x (0x%x)", (Get_JTAG_ID() & 0xF000) >> 12, Get_JTAG_ID());
      mFastbootCommonMsgInfo[i].Msg[mFastbootCommonMsgLen[i].Length] = 0;
      AsciiStrnCatS(mFastbootCommonMsgInfo[i].Msg,
                    sizeof(mFastbootCommonMsgInfo[i].Msg), StrTemp, sizeof (StrTemp));
      break;
    case 16:
      /* Check FTM MODE */
      mFastbootCommonMsgInfo[i].Msg[mFastbootCommonMsgLen[i].Length] = 0;
      AsciiStrnCatS (
          mFastbootCommonMsgInfo[i].Msg, sizeof(mFastbootCommonMsgInfo[i].Msg),
          is_ftm_mode() ? "FTM mode" : "Non FTM mode",
          is_ftm_mode() ? AsciiStrLen ("FTM mode") : AsciiStrLen ("Non FTM mode"));
      break;
    case 17:
      /* Check AVB VERITY */
      mFastbootCommonMsgInfo[i].Msg[mFastbootCommonMsgLen[i].Length] = 0;
      AsciiStrnCatS (
          mFastbootCommonMsgInfo[i].Msg, sizeof(mFastbootCommonMsgInfo[i].Msg),
          GetAvbVerity() ? "Disable" : "Enable",
          GetAvbVerity() ? AsciiStrLen ("Disable") : AsciiStrLen ("Enable"));
      break;
    case 18:
      /* Get CURRENT SLOT */
      ZeroMem (StrTemp, sizeof (StrTemp));
      UnicodeStrToAsciiStr (GetCurrentSlotSuffix ().Suffix, StrTemp);
      SKIP_FIRSTCHAR_IN_SLOT_SUFFIX (StrTemp);
      if(!StrnCmp (GetCurrentSlotSuffix ().Suffix, L"_a", StrLen (L"_a"))){
        AsciiStrnCatS (StrTemp, sizeof (StrTemp),
          check_unbootable(4,11)? " (Unbootable)":" (Bootable)",
          check_unbootable(4,11)? AsciiStrLen (" (Unbootable)"):AsciiStrLen (" (Bootable)"));
      }else{
        AsciiStrnCatS (StrTemp, sizeof (StrTemp),
          check_unbootable(4,36)? " (Unbootable)":" (Bootable)",
          check_unbootable(4,36)? AsciiStrLen (" (Unbootable)"):AsciiStrLen (" (Bootable)"));
      }
      mFastbootCommonMsgInfo[i].Msg[mFastbootCommonMsgLen[i].Length] = 0;
      AsciiStrnCatS (mFastbootCommonMsgInfo[i].Msg,
                     sizeof(mFastbootCommonMsgInfo[i].Msg), StrTemp, AsciiStrLen (StrTemp));
      break;
    case 19:
      /* Get BATTERY VOLTAGE */
      ZeroMem (StrTemp, sizeof (StrTemp));
      TargetBatterySocOk (&BatteryVoltage);
      AsciiSPrint (StrTemp, sizeof (StrTemp), "%d", BatteryVoltage);
      mFastbootCommonMsgInfo[i].Msg[mFastbootCommonMsgLen[i].Length] = 0;
      AsciiStrnCatS (mFastbootCommonMsgInfo[i].Msg,
                     sizeof(mFastbootCommonMsgInfo[i].Msg), StrTemp, AsciiStrLen (StrTemp));
      break;
    case 20:
      /* Get UART STATUS */
      mFastbootCommonMsgInfo[i].Msg[mFastbootCommonMsgLen[i].Length] = 0;
      AsciiStrnCatS (
          mFastbootCommonMsgInfo[i].Msg, sizeof(mFastbootCommonMsgInfo[i].Msg),
          GetUartStatus() ? "ON" : "OFF",
          GetUartStatus() ? AsciiStrLen ("ON") : AsciiStrLen ("OFF"));
      break;
    case 21:
      /* Get DUNGLE UNLOCK */
      mFastbootCommonMsgInfo[i].Msg[mFastbootCommonMsgLen[i].Length] = 0;
      AsciiStrnCatS (
          mFastbootCommonMsgInfo[i].Msg, sizeof(mFastbootCommonMsgInfo[i].Msg),
          IsAuthorized() ? "unlocked" : "locked",
          IsAuthorized() ? AsciiStrLen ("unlocked") : AsciiStrLen ("locked"));
      break;
    case 22:
      /* Get DEVICE STATE */
      mFastbootCommonMsgInfo[i].Msg[mFastbootCommonMsgLen[i].Length] = 0;
      AsciiStrnCatS (
          mFastbootCommonMsgInfo[i].Msg, sizeof(mFastbootCommonMsgInfo[i].Msg),
          IsUnlocked () ? "unlocked" : "locked",
          IsUnlocked () ? AsciiStrLen ("unlocked") : AsciiStrLen ("locked"));
      break;
    case 23:
      /* Get WATERMASK UNLOCK STATE */
      mFastbootCommonMsgInfo[i].Msg[mFastbootCommonMsgLen[i].Length] = 0;
      AsciiStrnCatS (
          mFastbootCommonMsgInfo[i].Msg, sizeof(mFastbootCommonMsgInfo[i].Msg),
          IsAuthorized_3() ? "Y" : "N",
          IsAuthorized_3() ? AsciiStrLen ("Y") : AsciiStrLen ("N"));
      break;
    case 24:
      /* Get DEBUG UNLOCK STATE */
      mFastbootCommonMsgInfo[i].Msg[mFastbootCommonMsgLen[i].Length] = 0;
      AsciiStrnCatS (
          mFastbootCommonMsgInfo[i].Msg, sizeof(mFastbootCommonMsgInfo[i].Msg),
          IsDebugUnlocked() ? "Y" : "N",
          IsDebugUnlocked() ? AsciiStrLen ("Y") : AsciiStrLen ("N"));
      break;
    }

    mFastbootCommonMsgInfo[i].Location = Location;
    Status = DrawMenu (&mFastbootCommonMsgInfo[i], &Height);
    if (Status != EFI_SUCCESS)
      return Status;
    Location += Height;
  }
#else
  /* Update fastboot common message */
  for (i = 0; i < ARRAY_SIZE (mFastbootCommonMsgInfo); i++) {
    switch (i) {
    case 0:
    case 1:
      break;
    case 2:
      /* Get product name */
      AsciiStrnCatS (mFastbootCommonMsgInfo[i].Msg,
        sizeof (mFastbootCommonMsgInfo[i].Msg), PRODUCT_NAME,
        AsciiStrLen (PRODUCT_NAME));
      break;
    case 3:
      /* Get variant value */
      BoardHwPlatformName (StrTemp, sizeof (StrTemp));
      GetRootDeviceType (StrTemp1, sizeof (StrTemp1));

      AsciiStrnCatS (mFastbootCommonMsgInfo[i].Msg,
                     sizeof (mFastbootCommonMsgInfo[i].Msg), StrTemp,
                     sizeof (StrTemp));
      AsciiStrnCatS (mFastbootCommonMsgInfo[i].Msg,
                     sizeof (mFastbootCommonMsgInfo[i].Msg), " ",
                     AsciiStrLen (" "));
      AsciiStrnCatS (mFastbootCommonMsgInfo[i].Msg,
                     sizeof (mFastbootCommonMsgInfo[i].Msg), StrTemp1,
                     sizeof (StrTemp1));
      break;
    case 4:
      /* Get bootloader version */
      GetBootloaderVersion (VersionTemp, sizeof (VersionTemp));
      AsciiStrnCatS (mFastbootCommonMsgInfo[i].Msg,
                     sizeof (mFastbootCommonMsgInfo[i].Msg), VersionTemp,
                     sizeof (VersionTemp));
      break;
    case 5:
      /* Get baseband version */
      ZeroMem (VersionTemp, sizeof (VersionTemp));
      GetRadioVersion (VersionTemp, sizeof (VersionTemp));
      AsciiStrnCatS (mFastbootCommonMsgInfo[i].Msg,
                     sizeof (mFastbootCommonMsgInfo[i].Msg), VersionTemp,
                     sizeof (VersionTemp));
      break;
    case 6:
      /* Get serial number */
      ZeroMem (StrTemp, sizeof (StrTemp));
      BoardSerialNum (StrTemp, MAX_RSP_SIZE);
      AsciiStrnCatS (mFastbootCommonMsgInfo[i].Msg,
                     sizeof (mFastbootCommonMsgInfo[i].Msg), StrTemp,
                     sizeof (StrTemp));
      break;
    case 7:
      /* Get secure boot value */
      AsciiStrnCatS (
          mFastbootCommonMsgInfo[i].Msg, sizeof (mFastbootCommonMsgInfo[i].Msg),
          IsSecureBootEnabled () ? "yes" : "no",
          IsSecureBootEnabled () ? AsciiStrLen ("yes") : AsciiStrLen ("no"));
      break;
    case 8:
      /* Get device status */
      AsciiStrnCatS (
          mFastbootCommonMsgInfo[i].Msg, sizeof (mFastbootCommonMsgInfo[i].Msg),
          IsUnlocked () ? "unlocked" : "locked",
          IsUnlocked () ? AsciiStrLen ("unlocked") : AsciiStrLen ("locked"));
      break;
    }

    mFastbootCommonMsgInfo[i].Location = Location;
    Status = DrawMenu (&mFastbootCommonMsgInfo[i], &Height);
    if (Status != EFI_SUCCESS)
      return Status;
    Location += Height;
  }
#endif

  OptionMenuInfo->Info.MenuType = DISPLAY_MENU_FASTBOOT;
  OptionMenuInfo->Info.OptionNum = ARRAY_SIZE (mFastbootOptionTitle);

  return Status;
}

#ifdef ASUS_BUILD
//+++ ASUS_BSP : add for user build menu
STATIC MENU_MSG_INFO mTitleFastbootInfo[] = {
     {{"\nPress volume key to select the menu, and press power key to enter the function.\n\n"},
     COMMON_FACTOR, BGR_SILVER, BGR_BLACK, ALIGN_LEFT, 0, NOACTION},
};

/**
  Draw the user build fastboot menu
 **/
EFI_STATUS UserBuildShowScreen(OPTION_MENU_INFO *OptionMenuInfo)
{
  EFI_STATUS Status = EFI_SUCCESS;
  UINT32 Location = 0;
  UINT32 Height = 0;
  UINT32 i = 0;
  UINT32 OptionItem = 0;

  ZeroMem (&OptionMenuInfo->Info, sizeof (MENU_OPTION_ITEM_INFO));

  /* Update fastboot option title */
  OptionMenuInfo->Info.MsgInfo = mFastbootOptionTitle;
  for (i = 0; i < ARRAY_SIZE (mFastbootOptionTitle); i++) {
    OptionMenuInfo->Info.OptionItems[i] = i;
  }
  OptionItem =
      OptionMenuInfo->Info.OptionItems[OptionMenuInfo->Info.OptionIndex];
  Status = UpdateFastbootOptionItem (OptionItem, &Location);
  if (Status != EFI_SUCCESS)
    return Status;

  /* Clear the screen before launch the verified boot menu */
  //gST->ConOut->ClearScreen (gST->ConOut);

  for (i = 0; i < ARRAY_SIZE(mTitleFastbootInfo); i++) {
    mTitleFastbootInfo[i].Location = Location;
    Status = DrawMenu(&mTitleFastbootInfo[i], &Height);
    if (Status != EFI_SUCCESS)
        return Status;
    Location += Height;
  }

  OptionMenuInfo->Info.MenuType = DISPLAY_MENU_FASTBOOT;
  OptionMenuInfo->Info.OptionNum = ARRAY_SIZE (mFastbootOptionTitle);

  return Status;
}

/**
  Draw the graphic user build fastboot menu
 **/
EFI_STATUS GraphicUserBuildShowScreen(OPTION_MENU_INFO *OptionMenuInfo)
{
  EFI_STATUS Status = EFI_SUCCESS;
  UINT32 Location = 0;
  UINT32 i = 0;
  UINT32 OptionItem = 0;

  ZeroMem (&OptionMenuInfo->Info, sizeof (MENU_OPTION_ITEM_INFO));

  /* Update fastboot option title */
  OptionMenuInfo->Info.MsgInfo = mFastbootOptionTitle;
  for (i = 0; i < ARRAY_SIZE (mFastbootOptionTitle); i++) {
    OptionMenuInfo->Info.OptionItems[i] = i;
  }
  OptionItem =
      OptionMenuInfo->Info.OptionItems[OptionMenuInfo->Info.OptionIndex];
  //Status = UpdateFastbootOptionItem (OptionItem, &Location);
  Status = UpdateASUSGraphicOptionItem (OptionItem, &Location);
  if (Status != EFI_SUCCESS)
    return Status;

  /* Clear the screen before launch the verified boot menu */
  //gST->ConOut->ClearScreen (gST->ConOut);
/*
  for (i = 0; i < ARRAY_SIZE(mTitleFastbootInfo); i++) {
    mTitleFastbootInfo[i].Location = Location;
    Status = DrawMenu(&mTitleFastbootInfo[i], &Height);
    if (Status != EFI_SUCCESS)
        return Status;
    Location += Height;
  }
*/
/*
  if(NULL == BMPProtocol) {
    Status |= gBS->LocateProtocol( &gEfiBmpProtocolGuid, NULL, (VOID **)&BMPProtocol);
    if( Status == EFI_SUCCESS ){
      DEBUG((EFI_D_ERROR, "[ABL] get BMPProtocol ok\n"));
      ASUSDrawGraphicMenu(start_up);
    }else{
      DEBUG((EFI_D_ERROR, "[ABL] get BMPProtocol fail\n"));
    }
  }else{
    ASUSDrawGraphicMenu(start_up);
  }
*/  
  OptionMenuInfo->Info.MenuType = DISPLAY_MENU_FASTBOOT;
  OptionMenuInfo->Info.OptionNum = ARRAY_SIZE (mFastbootOptionTitle);

  return Status;
}
//--- ASUS_BSP : add for user build menu

// +++ ASUS_BSP : add for fastboot unbootable show screen
STATIC MENU_MSG_INFO mUnbootableMsgInfo[] = {
     {{"\nBATTERY VOLTAGE - "},
     COMMON_FACTOR,
     BGR_GREEN,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
     {{"DUNGLE UNLOCK - "},
     COMMON_FACTOR,
     BGR_GREEN,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
    {{"DEVICE STATE - "},
     COMMON_FACTOR,
     BGR_GREEN,
     BGR_BLACK,
     COMMON,
     0,
     NOACTION},
};

STATIC MENU_MSG_INFO mTitleUnbootable[] = {
     {{"\nAVB VERIFY FAIL!!!"},
      BIG_FACTOR, BGR_RED, BGR_BLACK, ALIGN_LEFT, 0, NOACTION},
     {{"Error verifying image, or low battery voltage. Your device will shutdown in 30s\n"},
      COMMON_FACTOR, BGR_SILVER, BGR_BLACK, ALIGN_LEFT, 0, NOACTION},
};

EFI_STATUS UnbootableShowScreen()
{
  EFI_STATUS Status = EFI_SUCCESS;
  UINT32 Location = 0;
  UINT32 Height = 0;
  UINT32 i = 0;
  CHAR8 StrTemp[MAX_RSP_SIZE] = "";
  UINT32 BatteryVoltage = 0;

  /* Clear the screen before launch the verified boot menu */
  //gST->ConOut->ClearScreen (gST->ConOut);
  DrawMenuInit ();

  for (i = 0; i < ARRAY_SIZE(mTitleUnbootable); i++) {
    mTitleUnbootable[i].Location = Location;
    Status = DrawMenu(&mTitleUnbootable[i], &Height);
    if (Status != EFI_SUCCESS)
      return Status;
    Location += Height;
  }

  /* Update fastboot common message */
  for (i = 0; i < ARRAY_SIZE (mUnbootableMsgInfo); i++) {
    switch (i) {
    case 0:
      /*Show barrery voltage on fastboot menu*/
      ZeroMem (StrTemp, sizeof (StrTemp));
      TargetBatterySocOk (&BatteryVoltage);
      AsciiSPrint (StrTemp, sizeof (StrTemp), "%d", BatteryVoltage);
      AsciiStrnCatS (mUnbootableMsgInfo[i].Msg,
                     sizeof(mUnbootableMsgInfo[i].Msg), StrTemp, AsciiStrLen (StrTemp));
      break;
    case 1:
      /* Get device status */
      AsciiStrnCatS (
                     mUnbootableMsgInfo[i].Msg, sizeof(mUnbootableMsgInfo[i].Msg),
                     IsAuthorized() ? "unlocked" : "locked",
                     IsAuthorized() ? AsciiStrLen ("unlocked") : AsciiStrLen ("locked"));
      break;
    case 2:
      /* Get device status */
      AsciiStrnCatS (
                     mUnbootableMsgInfo[i].Msg, sizeof(mUnbootableMsgInfo[i].Msg),
                     IsUnlocked () ? "unlocked" : "locked",
                     IsUnlocked () ? AsciiStrLen ("unlocked") : AsciiStrLen ("locked"));
      break;
    }

    mUnbootableMsgInfo[i].Location = Location;
    Status = DrawMenu (&mUnbootableMsgInfo[i], &Height);
    if (Status != EFI_SUCCESS)
      return Status;
    Location += Height;
  }

  return Status;
}
// --- ASUS_BSP : add for fastboot unbootable show screen

// +++ ASUS_BSP : add for fastboot slot_a and slot_b unbootable show screen
STATIC MENU_MSG_INFO mTitleSlotAUnbootable[] = {
     {{"\nSlot _a is unbootable"},
      BIG_FACTOR, BGR_RED, BGR_BLACK, ALIGN_LEFT, 0, NOACTION},
     {{"Error verifying image, Your device will shutdown in 30s\n"},
      COMMON_FACTOR, BGR_SILVER, BGR_BLACK, ALIGN_LEFT, 0, NOACTION},
};

/**
  Draw the Slot_a is unbootable fastboot menu
 **/
EFI_STATUS SlotAUnbootableShowScreen()
{
  EFI_STATUS Status = EFI_SUCCESS;
  UINT32 Location = 0;
  UINT32 Height = 0;
  UINT32 i = 0;

  /* Clear the screen before launch the verified boot menu */
  //gST->ConOut->ClearScreen (gST->ConOut);
  DrawMenuInit ();

  for (i = 0; i < ARRAY_SIZE(mTitleSlotAUnbootable); i++) {
    mTitleSlotAUnbootable[i].Location = Location;
    Status = DrawMenu(&mTitleSlotAUnbootable[i], &Height);
    if (Status != EFI_SUCCESS)
      return Status;
    Location += Height;
  }

  return Status;
}

STATIC MENU_MSG_INFO mTitleSlotBUnbootable[] = {
     {{"\nSlot _b is unbootable"},
      BIG_FACTOR, BGR_RED, BGR_BLACK, ALIGN_LEFT, 0, NOACTION},
     {{"Error verifying image, Your device will shutdown in 30s\n"},
      COMMON_FACTOR, BGR_SILVER, BGR_BLACK, ALIGN_LEFT, 0, NOACTION},
};

/**
  Draw the Slot_b is unbootable fastboot menu
 **/
EFI_STATUS SlotBUnbootableShowScreen()
{
  EFI_STATUS Status = EFI_SUCCESS;
  UINT32 Location = 0;
  UINT32 Height = 0;
  UINT32 i = 0;

  /* Clear the screen before launch the verified boot menu */
  //gST->ConOut->ClearScreen (gST->ConOut);
  DrawMenuInit ();

  for (i = 0; i < ARRAY_SIZE(mTitleSlotBUnbootable); i++) {
    mTitleSlotBUnbootable[i].Location = Location;
    Status = DrawMenu(&mTitleSlotBUnbootable[i], &Height);
    if (Status != EFI_SUCCESS)
      return Status;
    Location += Height;
  }

  return Status;
}
// --- ASUS_BSP : add for fastboot slot_a and slot_b unbootable show screen

// +++ ASUS_BSP : add for ASUS HOT KEY MENU
EFI_STATUS AsusHotKeyShowScreen(OPTION_MENU_INFO *OptionMenuInfo)
{
  EFI_STATUS Status = EFI_SUCCESS;
  UINT32 Location = 0;
  UINT32 OptionItem = 0;//set mFastbootOptionTitle number
  UINT32 Height = 0;
  UINT32 i = 0;
  CHAR8 StrTemp[MAX_RSP_SIZE] = "";
  CHAR8 VersionTemp[MAX_VERSION_LEN] = "";
  UINT32 TempNum = 0;
  UINT32 BatteryVoltage = 0;
  sys_info sysinfo;

  /* Clear the screen before launch the asus info menu */
  gST->ConOut->ClearScreen (gST->ConOut);
  ZeroMem (&OptionMenuInfo->Info, sizeof (MENU_OPTION_ITEM_INFO));

  /* Update fastboot option title */
  OptionMenuInfo->Info.MsgInfo = mASUSOptionTitle;
  for (i = 0; i < ARRAY_SIZE (mASUSOptionTitle); i++) {
    OptionMenuInfo->Info.OptionItems[i] = i;
  }
  OptionItem =
      OptionMenuInfo->Info.OptionItems[OptionMenuInfo->Info.OptionIndex];

  Status = UpdateASUSOptionItem(OptionItem, &Location);
  if (Status != EFI_SUCCESS)
    return Status;

  /* Update fastboot option title */
  OptionMenuInfo->Info.MsgInfo = mASUSOptionTitle;

  if(read_sysinfo(&sysinfo) != EFI_SUCCESS)
  {
      DEBUG((EFI_D_ERROR, "[ABL] (ERROR) Get system info fail !!\n"));
  }

  /* Update fastboot common message */
  for (i = 0; i < ARRAY_SIZE(mAsusCommonMsgInfo); i++) {
    switch (i) {
    case 0:
      break;
    case 1:
      /* Get PROJECT NAME */
      ZeroMem(StrTemp, sizeof(StrTemp));
      GetProjName(StrTemp, sizeof(StrTemp));
      AsciiStrnCatS (mAsusCommonMsgInfo[i].Msg, sizeof(mAsusCommonMsgInfo[i].Msg), StrTemp, sizeof (StrTemp));
      break;
    case 2:
      /* Get HW STAGE */
      ZeroMem(StrTemp, sizeof(StrTemp));
      GetHWStage(StrTemp, sizeof(StrTemp));
      AsciiStrnCatS (mAsusCommonMsgInfo[i].Msg, sizeof(mAsusCommonMsgInfo[i].Msg), StrTemp, sizeof (StrTemp));
      break;
    case 3:
      /* Get XBL VERSION */
      ZeroMem(VersionTemp, sizeof(VersionTemp));
      GetXBLVersion (VersionTemp, sizeof (VersionTemp));
      AsciiStrnCatS (mAsusCommonMsgInfo[i].Msg, sizeof(mAsusCommonMsgInfo[i].Msg), VersionTemp, sizeof (VersionTemp));
      break;
    case 4:
      /* Get ABL VERSION */
      ZeroMem(VersionTemp, sizeof(VersionTemp));
      GetBootloaderVersion (VersionTemp, sizeof (VersionTemp));
      AsciiStrnCatS (mAsusCommonMsgInfo[i].Msg, sizeof(mAsusCommonMsgInfo[i].Msg), VersionTemp, sizeof (VersionTemp));
      break;
    case 5:
      /* Get CSC VERSION */
      AsciiStrnCatS (mAsusCommonMsgInfo[i].Msg, sizeof(mAsusCommonMsgInfo[i].Msg), sysinfo.csc_build_version, MAX_RSP_SIZE);
      break;
    case 6:
      /* Get MEM INFO */
      AsciiStrnCatS(mAsusCommonMsgInfo[i].Msg, sizeof(mAsusCommonMsgInfo[i].Msg), sysinfo.mem_info, MAX_RSP_SIZE);
      break;
    case 7:
      /* Get CPU FREQ */
      AsciiStrnCatS (mAsusCommonMsgInfo[i].Msg, sizeof(mAsusCommonMsgInfo[i].Msg), sysinfo.cpu_freq, MAX_RSP_SIZE);
      break;
    case 8:
      /* Get SSN */
      ZeroMem(StrTemp, sizeof(StrTemp));
      GetSSNNum(StrTemp, sizeof(StrTemp));
      AsciiStrnCatS (mAsusCommonMsgInfo[i].Msg, sizeof(mAsusCommonMsgInfo[i].Msg), StrTemp, sizeof (StrTemp));
      break;
    case 9:
      /* Get ISN */
      ZeroMem(StrTemp, sizeof(StrTemp));
      GetISNNum(StrTemp, sizeof(StrTemp));
      AsciiStrnCatS (mAsusCommonMsgInfo[i].Msg, sizeof(mAsusCommonMsgInfo[i].Msg), StrTemp, sizeof (StrTemp));
      break;
    case 10:
      /* Get IMEI */
      ZeroMem(StrTemp, sizeof(StrTemp));
      GetIMEINum(StrTemp, sizeof(StrTemp));
      AsciiStrnCatS (mAsusCommonMsgInfo[i].Msg, sizeof(mAsusCommonMsgInfo[i].Msg), StrTemp, sizeof (StrTemp));
      break;
    case 11:
      /* Get IMEI2 */
      ZeroMem(StrTemp, sizeof(StrTemp));
      GetIMEI2Num(StrTemp, sizeof(StrTemp));
      AsciiStrnCatS (mAsusCommonMsgInfo[i].Msg, sizeof(mAsusCommonMsgInfo[i].Msg), StrTemp, sizeof (StrTemp));
      break;
    case 12:
      /* Get WIFI MAC */
      AsciiStrnCatS (mAsusCommonMsgInfo[i].Msg, sizeof(mAsusCommonMsgInfo[i].Msg), sysinfo.wifi_mac, MAX_RSP_SIZE);
      break;
    case 13:
      /* Get WIFI MAC2 */
      AsciiStrnCatS (mAsusCommonMsgInfo[i].Msg, sizeof(mAsusCommonMsgInfo[i].Msg), sysinfo.wifi_mac_2, MAX_RSP_SIZE);
      break;
    case 14:
      /* Get BT MAC */
      AsciiStrnCatS (mAsusCommonMsgInfo[i].Msg, sizeof(mAsusCommonMsgInfo[i].Msg), sysinfo.bt_mac, MAX_RSP_SIZE);
      break;
    case 15:
      /* Get CID */
      ZeroMem(StrTemp, sizeof(StrTemp));
      GetCIDName(StrTemp, sizeof(StrTemp));
      AsciiStrnCatS (mAsusCommonMsgInfo[i].Msg, sizeof(mAsusCommonMsgInfo[i].Msg), StrTemp, sizeof (StrTemp));
      break;
    case 16:
      /* Get COUNTRY */
      ZeroMem(StrTemp, sizeof(StrTemp));
      GetCountryCode(StrTemp, sizeof(StrTemp));
      AsciiStrnCatS (mAsusCommonMsgInfo[i].Msg, sizeof(mAsusCommonMsgInfo[i].Msg), StrTemp, sizeof (StrTemp));
      break;
    case 17:
      /* Get SECURE BOOT */
      AsciiStrnCatS (
          mAsusCommonMsgInfo[i].Msg, sizeof(mAsusCommonMsgInfo[i].Msg) , 
          IsSecureBootEnabled () ? "SB=Y " : "SB=N ",
          IsSecureBootEnabled () ? AsciiStrLen ("SB=Y ") : AsciiStrLen ("SB=N "));
      BOOLEAN SecureDeviceNoRpmb = FALSE;
      IsSecureDeviceNoCheckRpmb(&SecureDeviceNoRpmb);
      AsciiStrnCatS (
          mAsusCommonMsgInfo[i].Msg, sizeof(mAsusCommonMsgInfo[i].Msg),
          SecureDeviceNoRpmb ? "/ SBNR=Y" : "/ SBNR=N",
          SecureDeviceNoRpmb ? AsciiStrLen ("/ SBNR=Y") : AsciiStrLen ("/ SBNR=N"));

      break;
    case 18:
      /* Get FEATURE ID */
      TempNum = Get_FEATURE_ID();

      if (TempNum == 0x0){
        AsciiStrnCatS (mFastbootCommonMsgInfo[i].Msg,
                       sizeof (mFastbootCommonMsgInfo[i].Msg), "100-AC-SM8475_ES", AsciiStrLen ("100-AC-SM8475_ES"));
      }else{
        AsciiStrnCatS (mFastbootCommonMsgInfo[i].Msg,
                       sizeof (mFastbootCommonMsgInfo[i].Msg), "Unknown", AsciiStrLen ("Unknown"));
      }
      break;
    case 19:
      /* Get JTAG ID */
      ZeroMem (StrTemp, sizeof (StrTemp));
      AsciiSPrint (StrTemp, sizeof (StrTemp), "%x (0x%x)", (Get_JTAG_ID() & 0xF000) >> 12, Get_JTAG_ID());
      AsciiStrnCatS (mAsusCommonMsgInfo[i].Msg, sizeof(mAsusCommonMsgInfo[i].Msg), StrTemp, sizeof (StrTemp));
      break;
    case 20:
      /* Check FTM MODE */
      AsciiStrnCatS (
                     mAsusCommonMsgInfo[i].Msg, sizeof(mAsusCommonMsgInfo[i].Msg),
                     is_ftm_mode() ? "FTM mode" : "Non FTM mode",
                     is_ftm_mode() ? AsciiStrLen ("FTM mode") : AsciiStrLen ("Non FTM mode"));
      break;
    case 21:
      /* Check AVB VERITY */
      AsciiStrnCatS(
                     mAsusCommonMsgInfo[i].Msg, sizeof(mAsusCommonMsgInfo[i].Msg),
                     GetAvbVerity() ? "Disable" : "Enable",
                     GetAvbVerity() ? AsciiStrLen ("Disable") : AsciiStrLen ("Enable"));
      break;
    case 22:
      /* Get CURRENT SLOT */
      ZeroMem (StrTemp, sizeof (StrTemp));
      UnicodeStrToAsciiStr (GetCurrentSlotSuffix ().Suffix, StrTemp);
      SKIP_FIRSTCHAR_IN_SLOT_SUFFIX (StrTemp);
      if(!StrnCmp (GetCurrentSlotSuffix ().Suffix, L"_a", StrLen (L"_a"))){
        AsciiStrnCatS (StrTemp, sizeof (StrTemp),
          check_unbootable(4,11)? " (Unbootable)":" (Bootable)",
          check_unbootable(4,11)? AsciiStrLen (" (Unbootable)"):AsciiStrLen (" (Bootable)"));
      }else{
        AsciiStrnCatS (StrTemp, sizeof (StrTemp),
          check_unbootable(4,36)? " (Unbootable)":" (Bootable)",
          check_unbootable(4,36)? AsciiStrLen (" (Unbootable)"):AsciiStrLen (" (Bootable)"));
      }
      AsciiStrnCatS (mAsusCommonMsgInfo[i].Msg, sizeof(mAsusCommonMsgInfo[i].Msg), StrTemp, AsciiStrLen (StrTemp));
      break;
    case 23:
      /* Get BATTERY VOLTAGE */
      ZeroMem (StrTemp, sizeof (StrTemp));
      TargetBatterySocOk (&BatteryVoltage);
      AsciiSPrint (StrTemp, sizeof (StrTemp), "%d", BatteryVoltage);
      AsciiStrnCatS (mAsusCommonMsgInfo[i].Msg, sizeof(mAsusCommonMsgInfo[i].Msg), StrTemp, AsciiStrLen (StrTemp));
      break;
    case 24:
      /* Get UART STATUS */
      AsciiStrnCatS (
          mAsusCommonMsgInfo[i].Msg, sizeof(mAsusCommonMsgInfo[i].Msg),
          GetUartStatus() ? "ON" : "OFF",
          GetUartStatus() ? AsciiStrLen ("ON") : AsciiStrLen ("OFF"));
      break;
    case 25:
      /* Get DM-Verity Counter */
      ZeroMem(StrTemp, sizeof(StrTemp));
      AsciiSPrint(StrTemp, sizeof(StrTemp), "%d", GetTotalDmVerityCounter());
      AsciiStrnCatS (mAsusCommonMsgInfo[i].Msg, sizeof(mAsusCommonMsgInfo[i].Msg), StrTemp, AsciiStrLen (StrTemp));
      break;
    case 26:
      /* Get Slot A Retry Unbootable Counter */
      ZeroMem(StrTemp, sizeof(StrTemp));
      AsciiSPrint(StrTemp, sizeof(StrTemp), "%d", GetSlotARetryCounter());
      AsciiStrnCatS (mAsusCommonMsgInfo[i].Msg, sizeof(mAsusCommonMsgInfo[i].Msg), StrTemp, AsciiStrLen (StrTemp));
      break;
    case 27:
      /* Get Slot A Unbootable Counter */
      ZeroMem(StrTemp, sizeof(StrTemp));
      AsciiSPrint(StrTemp, sizeof(StrTemp), "%d", GetSlotAUnbootableCounter());
      AsciiStrnCatS (mAsusCommonMsgInfo[i].Msg, sizeof(mAsusCommonMsgInfo[i].Msg), StrTemp, AsciiStrLen (StrTemp));
      break;
    case 28:
      /* Get Slot B Retry Counter */
      ZeroMem(StrTemp, sizeof(StrTemp));
      AsciiSPrint(StrTemp, sizeof(StrTemp), "%d", GetSlotBRetryCounter());
      AsciiStrnCatS (mAsusCommonMsgInfo[i].Msg, sizeof(mAsusCommonMsgInfo[i].Msg), StrTemp, AsciiStrLen (StrTemp));
      break;
    case 29:
      /* Get Slot B Unbootable Counter */
      ZeroMem(StrTemp, sizeof(StrTemp));
      AsciiSPrint(StrTemp, sizeof(StrTemp), "%d", GetSlotBUnbootableCounter());
      AsciiStrnCatS (mAsusCommonMsgInfo[i].Msg, sizeof(mAsusCommonMsgInfo[i].Msg), StrTemp, AsciiStrLen (StrTemp));
      break;
    case 30:
      /* Get ASUS Unlock 2 */
      AsciiStrnCatS (
          mAsusCommonMsgInfo[i].Msg, sizeof(mAsusCommonMsgInfo[i].Msg),
          IsAuthorized_2() ? "unlocked" : "locked",
          IsAuthorized_2() ? AsciiStrLen ("unlocked") : AsciiStrLen ("locked"));
      break;
    case 31:
      /* Get SETUP WIZARD */
      AsciiStrnCatS (mAsusCommonMsgInfo[i].Msg, sizeof(mAsusCommonMsgInfo[i].Msg), sysinfo.setupwizard, MAX_RSP_SIZE);
      break;
    case 32:
      /* Get DUNGLE UNLOCK */
      AsciiStrnCatS (
          mAsusCommonMsgInfo[i].Msg, sizeof(mAsusCommonMsgInfo[i].Msg),
          IsAuthorized() ? "unlocked" : "locked",
          IsAuthorized() ? AsciiStrLen ("unlocked") : AsciiStrLen ("locked"));
      break;
    case 33:
      /* Get DEVICE STATE */
      AsciiStrnCatS (
          mAsusCommonMsgInfo[i].Msg, sizeof(mAsusCommonMsgInfo[i].Msg),
          IsUnlocked () ? "unlocked" : "locked",
          IsUnlocked () ? AsciiStrLen ("unlocked") : AsciiStrLen ("locked"));
      break;
    case 34:
      /* Get WATERMASK UNLOCK STATE */
      AsciiStrnCatS (
          mAsusCommonMsgInfo[i].Msg, sizeof(mAsusCommonMsgInfo[i].Msg),
          IsAuthorized_3() ? "Y" : "N",
          IsAuthorized_3() ? AsciiStrLen ("Y") : AsciiStrLen ("N"));
      break;
    case 35:
      /* Get DEBUG UNLOCK STATE */
      AsciiStrnCatS (
          mAsusCommonMsgInfo[i].Msg, sizeof(mAsusCommonMsgInfo[i].Msg),
          IsDebugUnlocked() ? "Y" : "N",
          IsDebugUnlocked() ? AsciiStrLen ("Y") : AsciiStrLen ("N"));
      break;
    }

    mAsusCommonMsgInfo[i].Location = Location;
    Status = DrawMenu(&mAsusCommonMsgInfo[i], &Height);
    if (Status != EFI_SUCCESS)
        return Status;
        Location += Height;
  }

  OptionMenuInfo->Info.MenuType = DISPLAY_MENU_FASTBOOT;
  OptionMenuInfo->Info.OptionNum = ARRAY_SIZE(mASUSOptionTitle);

  return Status;
}
// --- ASUS_BSP : add for ASUS HOT KEY MENU

extern BOOLEAN AsusVbmetaVerified;
extern BOOLEAN AsusBootVerified;
extern BOOLEAN AsusDtboVerified;
extern BOOLEAN AsusVendorBootVerified;
extern BOOLEAN AsusInitBootVerified;
EFI_STATUS AsusVerifiedStateShowScreen(OPTION_MENU_INFO *OptionMenuInfo)
{
  EFI_STATUS Status = EFI_SUCCESS;
  UINT32 Location = 0;
  UINT32 OptionItem = 0;
  UINT32 Height = 0;
  UINT32 i = 0;
  BOOLEAN AsusVerifiedState = FALSE;

  /* Clear the screen before launch the AsusVerifiedStateShowScreen */
  gST->ConOut->ClearScreen (gST->ConOut);
  ZeroMem (&OptionMenuInfo->Info, sizeof (MENU_OPTION_ITEM_INFO));

  /* Update fastboot option title */
  OptionMenuInfo->Info.MsgInfo = mASUSOptionTitle;
  for (i = 0; i < ARRAY_SIZE (mASUSOptionTitle); i++) {
    OptionMenuInfo->Info.OptionItems[i] = i;
  }

  OptionItem = 3;

  Status = UpdateASUSOptionItem(OptionItem, &Location);
  if (Status != EFI_SUCCESS)
    return Status;

  /* Update fastboot option title */
  OptionMenuInfo->Info.MsgInfo = mASUSOptionTitle;

  if (FirstDrawAsusVerifiedStateMenu == TRUE) {
    for (i = 0; i < ARRAY_SIZE (mAsusVerifiedStateMsgInfo); i++) {
      mAsusVerifiedStateMsgLen[i].Length = strlen(mAsusVerifiedStateMsgInfo[i].Msg);
    }
    FirstDrawAsusVerifiedStateMenu = FALSE;
  }

  for (i = 0; i < ARRAY_SIZE(mAsusVerifiedStateMsgInfo); i++) {
    switch (i) {
    case 0:
      break;
    case 1:
      /* AsusVerifiedState */
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
      AsusVerifiedState = AsusVerifiedState && DMVeritySate && DMVerityMergeState;

      mAsusVerifiedStateMsgInfo[i].Msg[mAsusVerifiedStateMsgLen[i].Length] = 0;
      AsciiStrCatS (mAsusVerifiedStateMsgInfo[i].Msg,
                   sizeof(mAsusVerifiedStateMsgInfo[i].Msg), AsusVerifiedState? "PASS" : "FAIL");
      break;
    case 2:
      /* AsusVbmetaVerified */
      mAsusVerifiedStateMsgInfo[i].Msg[mAsusVerifiedStateMsgLen[i].Length] = 0;
      AsciiStrCatS (mAsusVerifiedStateMsgInfo[i].Msg,
                   sizeof(mAsusVerifiedStateMsgInfo[i].Msg), AsusVbmetaVerified? "PASS" : "FAIL");
      break;
    case 3:
      /* AsusBootVerified */
      mAsusVerifiedStateMsgInfo[i].Msg[mAsusVerifiedStateMsgLen[i].Length] = 0;
      AsciiStrCatS (mAsusVerifiedStateMsgInfo[i].Msg,
                   sizeof(mAsusVerifiedStateMsgInfo[i].Msg), AsusBootVerified? "PASS" : "FAIL");
      break;
    case 4:
      /* AsusDtboVerified */
      mAsusVerifiedStateMsgInfo[i].Msg[mAsusVerifiedStateMsgLen[i].Length] = 0;
      AsciiStrCatS (mAsusVerifiedStateMsgInfo[i].Msg,
                   sizeof(mAsusVerifiedStateMsgInfo[i].Msg), AsusDtboVerified? "PASS" : "FAIL");
      break;
    case 5:
      /* AsusVendorBootVerified */
      mAsusVerifiedStateMsgInfo[i].Msg[mAsusVerifiedStateMsgLen[i].Length] = 0;
      AsciiStrCatS (mAsusVerifiedStateMsgInfo[i].Msg,
                   sizeof(mAsusVerifiedStateMsgInfo[i].Msg), AsusVendorBootVerified? "PASS" : "FAIL");
      break;
    case 6:
      /* AsusInitBootVerified */
      mAsusVerifiedStateMsgInfo[i].Msg[mAsusVerifiedStateMsgLen[i].Length] = 0;
      AsciiStrCatS (mAsusVerifiedStateMsgInfo[i].Msg,
                   sizeof(mAsusVerifiedStateMsgInfo[i].Msg), AsusInitBootVerified? "PASS" : "FAIL");
      break;
    case 7:
      /* DMVeritySate */
      mAsusVerifiedStateMsgInfo[i].Msg[mAsusVerifiedStateMsgLen[i].Length] = 0;
      AsciiStrCatS (mAsusVerifiedStateMsgInfo[i].Msg,
                   sizeof(mAsusVerifiedStateMsgInfo[i].Msg), DMVeritySate? "PASS" : "FAIL");
      break;
    case 8:
      /* DMVerityMergeState */
      mAsusVerifiedStateMsgInfo[i].Msg[mAsusVerifiedStateMsgLen[i].Length] = 0;
      AsciiStrCatS (mAsusVerifiedStateMsgInfo[i].Msg,
                  sizeof(mAsusVerifiedStateMsgInfo[i].Msg), DMVerityMergeState? "PASS" : "FAIL");
      break;
    }

    mAsusVerifiedStateMsgInfo[i].Location = Location;
    Status = DrawMenu(&mAsusVerifiedStateMsgInfo[i], &Height);
    if (Status != EFI_SUCCESS)
        return Status;
        Location += Height;
  }

  OptionMenuInfo->Info.MenuType = DISPLAY_MENU_FASTBOOT;
  OptionMenuInfo->Info.OptionNum = ARRAY_SIZE(mASUSOptionTitle);

  return Status;
}
#endif

/* Draw the fastboot menu and start to detect the key's status */
VOID DisplayFastbootMenu (VOID)
{
  EFI_STATUS Status = EFI_SUCCESS;
  OPTION_MENU_INFO *OptionMenuInfo;

  if (IsEnableDisplayMenuFlagSupported ()) {
    OptionMenuInfo = &gMenuInfo;
#ifdef ASUS_BUILD
    if (TargetBuildVariantUser())
    {
        DEBUG((EFI_D_ERROR, "[ABL] User build Menu\n"));
        DrawMenuInit ();
        OptionMenuInfo->LastMenuType = OptionMenuInfo->Info.MenuType;
        Status = GraphicUserBuildShowScreen(OptionMenuInfo);
        if(Status != EFI_SUCCESS){
            Status = FastbootMenuShowScreen (OptionMenuInfo);
        }
    }else {
        DEBUG((EFI_D_ERROR, "[ABL] Userdebug build Menu\n"));
        DrawMenuInit ();
        OptionMenuInfo->LastMenuType = OptionMenuInfo->Info.MenuType;
        Status = FastbootMenuShowScreen (OptionMenuInfo);
    }
#else
    DrawMenuInit ();
    OptionMenuInfo->LastMenuType = OptionMenuInfo->Info.MenuType;
    Status = FastbootMenuShowScreen (OptionMenuInfo);
#endif

    if (Status != EFI_SUCCESS) {
      DEBUG ((EFI_D_ERROR, "Unable to show fastboot menu on screen: %r\n",
              Status));
      return;
    }

    MenuKeysDetectionInit (OptionMenuInfo);
    DEBUG ((EFI_D_VERBOSE, "Creating fastboot menu keys detect event\n"));
  } else {
    DEBUG ((EFI_D_INFO, "Display menu is not enabled!\n"));
  }
}
