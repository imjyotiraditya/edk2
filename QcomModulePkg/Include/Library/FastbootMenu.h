/* Copyright (c) 2016-2017, The Linux Foundation. All rights reserved.
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

#ifndef _FASTBOOTMENU_H_
#define _FASTBOOTMENU_H_

VOID DisplayFastbootMenu (VOID);
EFI_STATUS
UpdateFastbootOptionItem (UINT32 OptionItem, UINT32 *pLocation);

#ifdef ASUS_BUILD
#include <Library/DrawUI.h>
#include <Protocol/EFIPIL.h>

enum ASUSGraphicMenuOpt {
    start_up = 0,
    reboot_bootloader,
    recovery_mode,
    power_off
};

//WW Graphic images
#define ENG_START_UP          "eng_start.bmp"
#define ENG_REBOOT_bootloader "eng_restart_bootloader.bmp"
#define ENG_RECOVERY_MODE     "eng_recovery_mode.bmp"
#define ENG_POWER_OFF         "eng_power_off.bmp"

//CN Graphic images
#define CN_START_UP          "cn_start.bmp"
#define CN_REBOOT_bootloader "cn_restart_bootloader.bmp"
#define CN_RECOVERY_MODE     "cn_recovery_mode.bmp"
#define CN_POWER_OFF         "cn_power_off.bmp"

//TW Graphic images
#define TW_START_UP          "tw_start.bmp"
#define TW_REBOOT_bootloader "tw_restart_bootloader.bmp"
#define TW_RECOVERY_MODE     "tw_recovery_mode.bmp"
#define TW_POWER_OFF         "tw_power_off.bmp"

EFI_STATUS SlotAUnbootableShowScreen();
EFI_STATUS SlotBUnbootableShowScreen();
EFI_STATUS UnbootableShowScreen();
EFI_STATUS AsusHotKeyShowScreen(OPTION_MENU_INFO *OptionMenuInfo);
EFI_STATUS UpdateASUSOptionItem (UINT32 OptionItem, UINT32 *pLocation);
EFI_STATUS UpdateASUSGraphicOptionItem (UINT32 OptionItem, UINT32 *pLocation);
EFI_STATUS AsusVerifiedStateShowScreen(OPTION_MENU_INFO *OptionMenuInfo);
#endif
#endif
