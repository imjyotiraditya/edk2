/* Copyright (c) 2019, 2021 The Linux Foundation. All rights reserved.
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

#include <Uefi.h>
#include <Library/UefiLib.h>
#include <Library/PrintLib.h>
#include <Library/DebugLib.h>
#include <Library/BootLinux.h>
#include <EarlyEthernet.h>
#include "LinuxLoaderLib.h"
#include "AutoGen.h"

/*
 * Read a page from NAND into a page in RAM, parse
 * ip address string and fill into caller supplied buffer.
 */
EFI_STATUS
GetEarlyEthInfoFromPartition (CHAR8 *ipv4buf, CHAR8 *ipv6buf, CHAR8 *macbuf)
{
  EFI_STATUS Status;
  VOID *Buffer;
  CHAR8 *rawbuf;
  UINT32 DataSize = 0;
  UINT32 Pidx;
  UINT32 Qidx;
  CHAR8 BootDeviceType[BOOT_DEV_NAME_SIZE_MAX];

  memset (ipv4buf, '\0', MAX_IP_ADDR_BUF);
  AsciiStrnCpyS (ipv4buf, MAX_IP_ADDR_BUF, " eipv4=", 7);
  memset (ipv6buf, '\0', MAX_IP_ADDR_BUF);
  AsciiStrnCpyS (ipv6buf, MAX_IP_ADDR_BUF, " eipv6=", 7);
  memset (macbuf, '\0', MAX_IP_ADDR_BUF);
  AsciiStrnCpyS (macbuf, MAX_IP_ADDR_BUF, " ermac=", 7);

  GetRootDeviceType (BootDeviceType, BOOT_DEV_NAME_SIZE_MAX);

  if (!AsciiStrnCmp (BootDeviceType, "EMMC", AsciiStrLen ("EMMC"))) {
    DataSize = BOOT_IMG_MAX_PAGE_SIZE;
  } else {
    GetPageSize (&DataSize);
  }

  if (!ADD_OF (DataSize, ALIGNMENT_MASK_4KB - 1)) {
    DEBUG ((EFI_D_ERROR, "Integer overflow: in alignment mask 4k addition\n"));
    return EFI_BAD_BUFFER_SIZE;
  }

  Buffer = AllocatePages (ALIGN_PAGES (DataSize, ALIGNMENT_MASK_4KB));
  if (!Buffer) {
    DEBUG ((EFI_D_ERROR, "Failed to allocate memory for early ip address\n"));
    return EFI_BAD_BUFFER_SIZE;
  }

  Status = LoadImageFromPartition (Buffer, &DataSize, (CHAR16 *)L"emac");
  if (Status != EFI_SUCCESS) {
    FreePages (Buffer, 1);
    DEBUG ((EFI_D_ERROR, "Failed to load data for early ip address:%u\n",
                                                                  Status));
    return Status;
  }

  rawbuf = (CHAR8 *)Buffer;

  /* Extract ipv4 address string */
  Pidx = IP_ADDR_STR_OFFSET;
  Qidx = 0;
  while (((CHAR8)rawbuf[Pidx] !=
         EARLY_ADDR_TERMINATOR) &&
         (Qidx < 16)) {
    if ((rawbuf[Pidx] == '.') ||
       ((rawbuf[Pidx] > 47) &&
       (rawbuf[Pidx] < 58))) {
      ipv4buf[Qidx + 7] = rawbuf[Pidx];
      Pidx++;
      Qidx++;
    } else {
        FreePages (Buffer, 1);
        DEBUG ((EFI_D_VERBOSE, "Invalid char for early ipv4 0x%x at %d\n",
                                                     rawbuf[Pidx], Pidx));
        return EFI_INVALID_PARAMETER;
    }
  }

  /* Extract ipv6 address string */
  ++Pidx;
  Qidx = 0;
  while ((CHAR8)rawbuf[Pidx] !=
          EARLY_ADDR_TERMINATOR) {
     if ((rawbuf[Pidx] == '.') ||
        (rawbuf[Pidx] == ':') ||
        (rawbuf[Pidx] == '/') ||
        ((rawbuf[Pidx] > 47) &&
        (rawbuf[Pidx] < 58)) ||
        ((rawbuf[Pidx] > 96) &&
        (rawbuf[Pidx] < 103)) ||
        ((rawbuf[Pidx] > 64) &&
        (rawbuf[Pidx] < 71))) {
       ipv6buf[Qidx + 7] = rawbuf[Pidx];
       Pidx++;
       Qidx++;
    } else {
        FreePages (Buffer, 1);
        DEBUG ((EFI_D_VERBOSE, "Invalid char for early ipv6 0x%x at %d\n",
                                                     rawbuf[Pidx], Pidx));
        return EFI_INVALID_PARAMETER;
    }
  }

  /* Extract mac address string */
  ++Pidx;
  Qidx = 0;
  while (((CHAR8)rawbuf[Pidx] !=
         EARLY_ADDR_TERMINATOR) &&
         (Qidx < MAC_ADDR_LEN)) {
    if ((rawbuf[Pidx] == ':') ||
       ((rawbuf[Pidx] > 47) &&
       (rawbuf[Pidx] < 58)) ||
       ((rawbuf[Pidx] > 96) &&
       (rawbuf[Pidx] < 103)) ||
       ((rawbuf[Pidx] > 64) &&
       (rawbuf[Pidx] < 71))) {
       macbuf[Qidx + 7] = rawbuf[Pidx];
       Pidx++;
       Qidx++;
    } else {
        FreePages (Buffer, 1);
        DEBUG ((EFI_D_VERBOSE, "Invalid char for early mac 0x%x at %d\n",
                                                    rawbuf[Pidx], Pidx));
        return EFI_INVALID_PARAMETER;
    }
  }

  FreePages (Buffer, 1);
  return EFI_SUCCESS;
}

/*
 * Return 1 if build has early ethernet feature enabled otherwise 0.
 * Applicable for both Linux and Android builds.
 */
BOOLEAN
EarlyEthEnabled (VOID)
{
#if EARLY_ETH_ENABLED
  return 1;
#else
  return 0;
#endif
}
