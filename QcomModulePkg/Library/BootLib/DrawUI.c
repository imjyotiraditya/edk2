/* Copyright (c) 2016-2019, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of The Linux Foundation, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
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
#include <Library/DebugLib.h>
#include <Library/Debug.h>
#include <Library/DrawUI.h>
#include <Library/Fonts.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UpdateDeviceTree.h>
#include <Protocol/GraphicsOutput.h>
#include <Uefi.h>
#include <Protocol/HiiFont.h>

#if defined  ASUS_AI2205_BUILD && !CN_BUILD
#include <Library/AsusLogo.h>
#include <Library/AsusLogo_charger.h>
#include <Library/AsusAndroid.h>
#endif

#if defined  ASUS_AI2205_BUILD && CN_BUILD
#include <Library/AsusAndroid.h>
#include <Library/AsusLogoCN.h>
#include <Library/AsusLogo_charger.h>
#include <Library/AsusAndroidCN.h>
extern char cid_name[32];
#endif

STATIC EFI_GRAPHICS_OUTPUT_PROTOCOL *GraphicsOutputProtocol;
STATIC EFI_GRAPHICS_OUTPUT_BLT_PIXEL *LogoBlt;
STATIC EFI_HII_FONT_PROTOCOL  *gHiiFont = NULL;

STATIC CHAR16 *mFactorName[] = {
        [1] = (CHAR16 *)L"",        [2] = (CHAR16 *)SYSFONT_2x,
        [3] = (CHAR16 *)SYSFONT_3x, [4] = (CHAR16 *)SYSFONT_4x,
        [5] = (CHAR16 *)SYSFONT_5x, [6] = (CHAR16 *)SYSFONT_6x,
        [7] = (CHAR16 *)SYSFONT_7x, [8] = (CHAR16 *)SYSFONT_8x,
};

STATIC EFI_GRAPHICS_OUTPUT_BLT_PIXEL mColors[] = {
        [BGR_WHITE] = {0xff, 0xff, 0xff, 0x00},
        [BGR_BLACK] = {0x00, 0x00, 0x00, 0x00},
        [BGR_ORANGE] = {0x00, 0xa5, 0xff, 0x00},
        [BGR_YELLOW] = {0x00, 0xff, 0xff, 0x00},
        [BGR_RED] = {0x00, 0x00, 0x98, 0x00},
        [BGR_GREEN] = {0x00, 0xff, 0x00, 0x00},
        [BGR_BLUE] = {0xff, 0x00, 0x00, 0x00},
        [BGR_CYAN] = {0xff, 0xff, 0x00, 0x00},
        [BGR_SILVER] = {0xc0, 0xc0, 0xc0, 0x00},
};

STATIC UINT32 GetResolutionWidth (VOID)
{
  STATIC UINT32 Width;
  EFI_HANDLE ConsoleHandle = (EFI_HANDLE)NULL;

  /* Get the width from the protocal at the first time */
  if (Width)
    return Width;

  if (GraphicsOutputProtocol == NULL) {
    ConsoleHandle = gST->ConsoleOutHandle;
    if (ConsoleHandle == NULL) {
      DEBUG (
          (EFI_D_ERROR,
           "Failed to get the handle for the active console input device.\n"));
      return 0;
    }

    gBS->HandleProtocol (ConsoleHandle, &gEfiGraphicsOutputProtocolGuid,
                         (VOID **)&GraphicsOutputProtocol);
    if (GraphicsOutputProtocol == NULL) {
      DEBUG ((EFI_D_ERROR, "Failed to get the graphics output protocol.\n"));
      return 0;
    }
  }
  Width = GraphicsOutputProtocol->Mode->Info->HorizontalResolution;
  if (!Width)
    DEBUG ((EFI_D_ERROR, "Failed to get the width of the screen.\n"));

  return Width;
}

STATIC UINT32 GetResolutionHeight (VOID)
{
  STATIC UINT32 Height;
  EFI_HANDLE ConsoleHandle = (EFI_HANDLE)NULL;

  /* Get the height from the protocal at the first time */
  if (Height)
    return Height;

  if (GraphicsOutputProtocol == NULL) {
    ConsoleHandle = gST->ConsoleOutHandle;
    if (ConsoleHandle == NULL) {
      DEBUG (
          (EFI_D_ERROR,
           "Failed to get the handle for the active console input device.\n"));
      return 0;
    }

    gBS->HandleProtocol (ConsoleHandle, &gEfiGraphicsOutputProtocolGuid,
                         (VOID **)&GraphicsOutputProtocol);
    if (GraphicsOutputProtocol == NULL) {
      DEBUG ((EFI_D_ERROR, "Failed to get the graphics output protocol.\n"));
      return 0;
    }
  }
  Height = GraphicsOutputProtocol->Mode->Info->VerticalResolution;
  if (!Height)
    DEBUG ((EFI_D_ERROR, "Failed to get the height of the screen.\n"));

  return Height;
}

EFI_STATUS BackUpBootLogoBltBuffer (VOID)
{
  EFI_STATUS Status;
  UINT32 Width;
  UINT32 Height;
  UINT64 BufferSize;

  /* Return directly if it's already backed up the boot logo blt buffer */
  if (LogoBlt)
    return EFI_SUCCESS;

  Width = GetResolutionWidth ();
  Height = GetResolutionHeight ();
  if (!Width || !Height) {
    DEBUG ((EFI_D_ERROR, "Failed to get width or height\n"));
    return EFI_UNSUPPORTED;
  }

  /* Ensure the Height * Width doesn't overflow */
  if (Height > DivU64x64Remainder ((UINTN)~0, Width, NULL)) {
    DEBUG ((EFI_D_ERROR, "Height * Width overflow\n"));
    return EFI_UNSUPPORTED;
  }
  BufferSize = MultU64x64 (Width, Height);

  /* Ensure the BufferSize * sizeof (EFI_GRAPHICS_OUTPUT_BLT_PIXEL) doesn't
   * overflow */
  if (BufferSize >
      DivU64x32 ((UINTN)~0, sizeof (EFI_GRAPHICS_OUTPUT_BLT_PIXEL))) {
    DEBUG ((EFI_D_ERROR,
            "BufferSize * sizeof (EFI_GRAPHICS_OUTPUT_BLT_PIXEL) overflow\n"));
    return EFI_UNSUPPORTED;
  }

  LogoBlt = AllocateZeroPool ((UINTN)BufferSize *
                              sizeof (EFI_GRAPHICS_OUTPUT_BLT_PIXEL));
  if (LogoBlt == NULL) {
    return EFI_OUT_OF_RESOURCES;
  }

  Status = GraphicsOutputProtocol->Blt (
      GraphicsOutputProtocol, LogoBlt, EfiBltVideoToBltBuffer, 0, 0, 0, 0,
      Width, Height, Width * sizeof (EFI_GRAPHICS_OUTPUT_BLT_PIXEL));

  if (Status != EFI_SUCCESS) {
    FreePool (LogoBlt);
    LogoBlt = NULL;
  }

  return Status;
}

// This function would restore the boot logo if the display on the screen is
// changed.
VOID RestoreBootLogoBitBuffer (VOID)
{
  EFI_STATUS Status;
  UINT32 Width;
  UINT32 Height;

  /* Return directly if the boot logo bit buffer is null */
  if (!LogoBlt) {
    return;
  }

  Width = GetResolutionWidth ();
  Height = GetResolutionHeight ();
  if (!Width || !Height) {
    DEBUG ((EFI_D_ERROR, "Failed to get width or height\n"));
    return;
  }

  /* Ensure the Height * Width doesn't overflow */
  if (Height > DivU64x64Remainder ((UINTN)~0, Width, NULL)) {
    DEBUG ((EFI_D_ERROR, "Height * Width overflow\n"));
    return;
  }

  Status = GraphicsOutputProtocol->Blt (
      GraphicsOutputProtocol, LogoBlt, EfiBltBufferToVideo, 0, 0, 0, 0, Width,
      Height, Width * sizeof (EFI_GRAPHICS_OUTPUT_BLT_PIXEL));

  if (Status != EFI_SUCCESS) {
    FreePool (LogoBlt);
    LogoBlt = NULL;
  }
}

VOID FreeBootLogoBltBuffer (VOID)
{
  if (LogoBlt) {
    FreePool (LogoBlt);
    LogoBlt = NULL;
  }
}

STATIC UINT32 GetDisplayMode  (VOID)
{
  if (GetResolutionWidth () < GetResolutionHeight ()) {
    return PORTRAIT_MODE;
  }

  return HORIZONTAL_MODE;
}

/* Get max row */
STATIC UINT32 GetMaxRow (VOID)
{
  EFI_STATUS Status;
  UINT32 FontBaseHeight = EFI_GLYPH_HEIGHT;
  UINT32 MaxRow = 0;
  EFI_IMAGE_OUTPUT *Blt = NULL;

  Status = gBS->LocateProtocol (&gEfiHiiFontProtocolGuid, NULL,
                               (VOID **) &gHiiFont);
  if (EFI_ERROR (Status)) {
    return MaxRow;
  }

  Status = gHiiFont->GetGlyph (gHiiFont, 'a', NULL, &Blt, NULL);
  if (!EFI_ERROR (Status)) {
    if (Blt) {
      FontBaseHeight = Blt->Height;
    }
  }
  MaxRow = GetResolutionHeight() / FontBaseHeight;
  return MaxRow;
}

/* Get Max font count per row */
STATIC UINT32 GetMaxFontCount (VOID)
{
  EFI_STATUS Status;
  UINT32 FontBaseWidth = EFI_GLYPH_WIDTH;
  UINT32 max_count = 0;
  EFI_IMAGE_OUTPUT *Blt = NULL;

  Status = gBS->LocateProtocol (&gEfiHiiFontProtocolGuid, NULL,
                               (VOID **) &gHiiFont);
  if (EFI_ERROR (Status)) {
    return max_count;
  }

  Status = gHiiFont->GetGlyph (gHiiFont, 'a', NULL, &Blt, NULL);
  if (!EFI_ERROR (Status)) {
    if (Blt)
      FontBaseWidth = Blt->Width;
  }
  max_count = GetResolutionWidth () / FontBaseWidth;
  return max_count;
}

/**
  Get Font's scale factor
  @param[in] ScaleFactorType The type of the scale factor.
  @retval    ScaleFactor     Get the suitable scale factor base on the
                             scale factor's type.
 **/
STATIC UINT32
GetFontScaleFactor (UINT32 ScaleFactorType)
{
  UINT32 NumPerRow = 0;
  UINT32 ScaleFactor = 0;
  UINT32 ScaleFactor1 = 0;
  UINT32 ScaleFactor2 = 0;
  UINT32 MaxRow = 0;

  NumPerRow = CHAR_NUM_PERROW_POR;
  MaxRow = MAX_ROW_FOR_POR;
  if (GetDisplayMode () ==  HORIZONTAL_MODE) {
    NumPerRow = CHAR_NUM_PERROW_HOR;
    MaxRow = MAX_ROW_FOR_HOR;
  }
  ScaleFactor1 = GetMaxFontCount () / NumPerRow;
  ScaleFactor2 = GetMaxRow () / MaxRow;

  ScaleFactor = ScaleFactor1 > ScaleFactor2 ? ScaleFactor2 : ScaleFactor1;
  if (ScaleFactor < 2) {
    ScaleFactor = 1;
  } else if (ScaleFactor > ((ARRAY_SIZE (mFactorName) - 1) / MAX_FACTORTYPE)) {
    ScaleFactor = (ARRAY_SIZE (mFactorName) - 1) / MAX_FACTORTYPE;
  }

  return ScaleFactor * ScaleFactorType;
}

/* Get factor name base on the scale factor type */
STATIC CHAR16 *
GetFontFactorName (UINT32 ScaleFactorType)
{
  UINT32 ScaleFactor = GetFontScaleFactor (ScaleFactorType);

  if (ScaleFactor <= (ARRAY_SIZE (mFactorName) - 1)) {
    return mFactorName[ScaleFactor];
  } else {
    return (CHAR16 *)SYSFONT_3x;
  }
}

STATIC VOID
SetBltBuffer (EFI_IMAGE_OUTPUT *BltBuffer)
{
  BltBuffer->Width = (UINT16)GetResolutionWidth ();
  BltBuffer->Height = (UINT16)GetResolutionHeight ();
  BltBuffer->Image.Screen = GraphicsOutputProtocol;
}

STATIC VOID
SetDisplayInfo (MENU_MSG_INFO *TargetMenu,
                EFI_FONT_DISPLAY_INFO *FontDisplayInfo)
{
  /* Foreground */
  FontDisplayInfo->ForegroundColor.Blue = mColors[TargetMenu->FgColor].Blue;
  FontDisplayInfo->ForegroundColor.Green = mColors[TargetMenu->FgColor].Green;
  FontDisplayInfo->ForegroundColor.Red = mColors[TargetMenu->FgColor].Red;
  /* Background */
  FontDisplayInfo->BackgroundColor.Blue = mColors[TargetMenu->BgColor].Blue;
  FontDisplayInfo->BackgroundColor.Green = mColors[TargetMenu->BgColor].Green;
  FontDisplayInfo->BackgroundColor.Red = mColors[TargetMenu->BgColor].Red;

  /* Set font name */
  FontDisplayInfo->FontInfoMask =
      EFI_FONT_INFO_ANY_SIZE | EFI_FONT_INFO_ANY_STYLE;
  gBS->CopyMem (&FontDisplayInfo->FontInfo.FontName,
                GetFontFactorName (TargetMenu->ScaleFactorType),
                StrSize (GetFontFactorName (TargetMenu->ScaleFactorType)));
}

STATIC VOID
StrAlignRight (CHAR8 *Msg, CHAR8 *FilledChar, UINT32 ScaleFactorType)
{
  UINT32 i = 0;
  UINT32 diff = 0;
  CHAR8 *StrSourceTemp = NULL;
  UINT32 Max_x = GetMaxFontCount ();
  UINT32 factor = GetFontScaleFactor (ScaleFactorType);

  if (Max_x / factor > AsciiStrLen (Msg)) {
    diff = Max_x / factor - AsciiStrLen (Msg);
    StrSourceTemp = AllocateZeroPool (MAX_MSG_SIZE);
    if (StrSourceTemp == NULL) {
      DEBUG ((EFI_D_ERROR,
             "Failed to allocate zero pool for StrSourceTemp.\n"));
      return;
    }

    for (i = 0; i < diff; i++) {
      AsciiStrnCatS (StrSourceTemp, MAX_MSG_SIZE, FilledChar, 1);
    }
    AsciiStrnCatS (StrSourceTemp, MAX_MSG_SIZE, Msg, Max_x / factor);
    gBS->CopyMem (Msg, StrSourceTemp, AsciiStrSize (StrSourceTemp));
    FreePool (StrSourceTemp);
  }
}

STATIC VOID
StrAlignLeft (CHAR8 *Msg,
              UINT32 MaxMsgSize,
              CHAR8 *FilledChar,
              UINT32 ScaleFactorType)
{
  UINT32 i = 0;
  UINT32 diff = 0;
  CHAR8 *StrSourceTemp = NULL;
  UINT32 Max_x = GetMaxFontCount ();
  UINT32 factor = GetFontScaleFactor (ScaleFactorType);

  if (Max_x / factor > AsciiStrLen (Msg)) {
    diff = Max_x / factor - AsciiStrLen (Msg);
    StrSourceTemp = AllocateZeroPool (MAX_MSG_SIZE);
    if (StrSourceTemp == NULL) {
      DEBUG ((EFI_D_ERROR,
             "Failed to allocate zero pool for StrSourceTemp.\n"));
      return;
    }

    for (i = 0; i < diff; i++) {
      AsciiStrnCatS (StrSourceTemp, MAX_MSG_SIZE, FilledChar, 1);
    }
    AsciiStrnCatS (Msg, MaxMsgSize,
                   StrSourceTemp, AsciiStrLen (StrSourceTemp));
    FreePool (StrSourceTemp);
  }
}

/* Message string manipulation base on the attribute of the message
 * LINEATION:   Fill a string with "_", for drawing a line
 * ALIGN_RIGHT: String align right and fill this string with " "
 * ALIGN_LEFT:  String align left and fill this string with " "
 * OPTION_ITEM: String align left and fill this string with " ",
 *              for updating the whole line's background
 */
STATIC VOID
ManipulateMenuMsg (MENU_MSG_INFO *TargetMenu)
{
  switch (TargetMenu->Attribute) {
  case LINEATION:
    StrAlignLeft (TargetMenu->Msg, sizeof (TargetMenu->Msg), "_",
                  TargetMenu->ScaleFactorType);
    break;
  case ALIGN_RIGHT:
    StrAlignRight (TargetMenu->Msg, " ", TargetMenu->ScaleFactorType);
    break;
  case ALIGN_LEFT:
  case OPTION_ITEM:
    StrAlignLeft (TargetMenu->Msg, sizeof (TargetMenu->Msg), " ",
                  TargetMenu->ScaleFactorType);
    break;
  }
}

/**
  Draw menu on the screen
  @param[in] TargetMenu    The message info.
  @param[in, out] pHeight  The Pointer for increased height.
  @retval EFI_SUCCESS      The entry point is executed successfully.
  @retval other            Some error occurs when executing this entry point.
**/
EFI_STATUS
DrawMenu (MENU_MSG_INFO *TargetMenu, UINT32 *pHeight)
{
  EFI_STATUS Status = EFI_SUCCESS;
  EFI_FONT_DISPLAY_INFO *FontDisplayInfo = NULL;
  EFI_IMAGE_OUTPUT *BltBuffer = NULL;
  EFI_HII_ROW_INFO *RowInfoArray = NULL;
  UINTN RowInfoArraySize;
  CHAR16 FontMessage[MAX_MSG_SIZE];
  UINT32 Height = GetResolutionHeight ();
  UINT32 Width = GetResolutionWidth ();

  if (!Height || !Width) {
    Status = EFI_OUT_OF_RESOURCES;
    goto Exit;
  }

  if (TargetMenu->Location >= Height) {
    DEBUG ((EFI_D_ERROR, "Error: Check the CHAR_NUM_PERROW: Y-axis(%d)"
                         " is larger than Y-max(%d)\n",
            TargetMenu->Location, Height));
    Status = EFI_ABORTED;
    goto Exit;
  }

  BltBuffer = AllocateZeroPool (sizeof (EFI_IMAGE_OUTPUT));
  if (BltBuffer == NULL) {
    DEBUG ((EFI_D_ERROR, "Failed to allocate zero pool for BltBuffer.\n"));
    Status = EFI_OUT_OF_RESOURCES;
    goto Exit;
  }
  SetBltBuffer (BltBuffer);

  FontDisplayInfo = AllocateZeroPool (sizeof (EFI_FONT_DISPLAY_INFO) + 100);
  if (FontDisplayInfo == NULL) {
    DEBUG (
        (EFI_D_ERROR, "Failed to allocate zero pool for FontDisplayInfo.\n"));
    Status = EFI_OUT_OF_RESOURCES;
    goto Exit;
  }
  SetDisplayInfo (TargetMenu, FontDisplayInfo);

  ManipulateMenuMsg (TargetMenu);
  AsciiStrToUnicodeStr (TargetMenu->Msg, FontMessage);

  Status = gBS->LocateProtocol (&gEfiHiiFontProtocolGuid, NULL,
                               (VOID **) &gHiiFont);
  if (EFI_ERROR (Status)) {
    goto Exit;
  }

  Status = gHiiFont->StringToImage (
      gHiiFont,
      /* Set to 0 for Bitmap mode */
      EFI_HII_DIRECT_TO_SCREEN | EFI_HII_OUT_FLAG_WRAP, FontMessage,
      FontDisplayInfo, &BltBuffer, 0, /* BltX */
      TargetMenu->Location,           /* BltY */
      &RowInfoArray, &RowInfoArraySize, NULL);
  if (Status != EFI_SUCCESS) {
    DEBUG ((EFI_D_ERROR, "Failed to render a string to the display: %r\n",
            Status));
    goto Exit;
  }

  if (pHeight && RowInfoArraySize && RowInfoArray) {
    *pHeight = RowInfoArraySize * RowInfoArray[0].LineHeight;
  }

  /* For Bitmap mode, use EfiBltBufferToVideo, and set DestX,DestY as needed */
  GraphicsOutputProtocol->Blt (GraphicsOutputProtocol, BltBuffer->Image.Bitmap,
                               EfiBltVideoToVideo, 0, /* SrcX */
                               0,                     /* SrcY */
                               0,                     /* DestX */
                               0,                     /* DestY */
                               BltBuffer->Width, BltBuffer->Height,
                               BltBuffer->Width *
                                   sizeof (EFI_GRAPHICS_OUTPUT_BLT_PIXEL));

Exit:
  if (RowInfoArray) {
    FreePool (RowInfoArray);
    RowInfoArray = NULL;
  }

  if (BltBuffer) {
    FreePool (BltBuffer);
    BltBuffer = NULL;
  }

  if (FontDisplayInfo) {
    FreePool (FontDisplayInfo);
    FontDisplayInfo = NULL;
  }
  return Status;
}

/**
  Set the message info
  @param[in]  Msg              Message.
  @param[in]  ScaleFactorType  The scale factor type of the message.
  @param[in]  FgColor          The foreground color of the message.
  @param[in]  BgColor          The background color of the message.
  @param[in]  Attribute        The attribute of the message.
  @param[in]  Location         The location of the message.
  @param[in]  Action           The action of the message.
  @param[out] MenuMsgInfo      The message info.
**/
VOID
SetMenuMsgInfo (MENU_MSG_INFO *MenuMsgInfo,
                CHAR8 *Msg,
                UINT32 ScaleFactorType,
                UINT32 FgColor,
                UINT32 BgColor,
                UINT32 Attribute,
                UINT32 Location,
                UINT32 Action)
{
  gBS->CopyMem (&MenuMsgInfo->Msg, Msg, AsciiStrSize (Msg));
  MenuMsgInfo->ScaleFactorType = ScaleFactorType;
  MenuMsgInfo->FgColor = FgColor;
  MenuMsgInfo->BgColor = BgColor;
  MenuMsgInfo->Attribute = Attribute;
  MenuMsgInfo->Location = Location;
  MenuMsgInfo->Action = Action;
}

#ifdef ASUS_BUILD
#include <Library/barcode.h>

// ASUS_BSP +++
EFI_STATUS DrawBarCode(CHAR8 *ISNstr)
{
    EFI_STATUS     Status = EFI_SUCCESS;
    void           *pBitmapImage = NULL;
    UINT32         Height = GetResolutionHeight();
    UINT32         Width = GetResolutionWidth();
    UINTN          centerX;
    UINTN          centerY;
    UINT32         isnLen = strlen(ISNstr);
    UINT32         bitmapHeight = (isnLen+2)*BARCODE_WIDTH_PIXEL+BARCODE_EDGE_WIDTH_PIXEL*2;
    UINT32         bitmapSize = (isnLen+2)*BARCODE_WIDTH_PIXEL*BARCODE_HEIGHT_PIXEL*PIXEL_SIZE +
                                BARCODE_EDGE_WIDTH_PIXEL*BARCODE_HEIGHT_PIXEL*PIXEL_SIZE*2;

    pBitmapImage = AllocateZeroPool(bitmapSize);

    if (pBitmapImage == NULL) {
        DEBUG((EFI_D_ERROR, "DrawBarCode: AllocateZeroPool failed\n"));
        goto Exit;
    } else {

        UINT32  pattern[2] = {0x00000000, 0xFFFFFFF};
        UINT32  *pData     = (UINT32*)pBitmapImage;
        UINT32  i, j, x, y;
        UINT32  barcode;
        UINT32  mask;
        UINT32  pixelWidth;
        CHAR8   c;

        DEBUG((EFI_D_VERBOSE, "DrawBarCode: start fill isn barcode bitmap\n"));

        //1. fill front edge bar
        for (y = 0; y < BARCODE_EDGE_WIDTH_PIXEL; y++)
            for (x = 0; x < BARCODE_HEIGHT_PIXEL; x++)
                *pData++ = pattern[BARCODE_WHITE];

        //2. fill barcode
        for (i = 0; i < (isnLen+2); i++) {
            if (i == 0 || i == isnLen+1)
                c = '*';
            else
                c = ISNstr[i-1];
            if (c >=  0x30 && c <= 0x39) {
                barcode = mBarCode[(int)(c-0x30)];
            } else if (c >= 0x41 && c <= 0x5A) {
                barcode = mBarCode[(int)(c-0x41+10)];
            } else if (c >= 0x61 && c <= 0x7A) {
                c -= 32;
                barcode = mBarCode[(int)(c-0x41+10)];
            } else if (c == '*') {
                barcode = BARCODE_START_END;
            } else {
                barcode = 0;
                DEBUG((EFI_D_ERROR, "unrecognize char, empty it\n"));
            }

            for (j = 0; j < BARCODE_CODE_SIZE; j++) {

                pixelWidth = BARCODE_BAR_PIXEL;
                mask = 0x0001;
                mask = mask << (BARCODE_CODE_SIZE - j - 1);
                if (barcode & mask)
                    pixelWidth = BARCODE_BAR_PIXEL * 2;

                for (y = 0; y < pixelWidth; y++)
                    for (x = 0; x < BARCODE_HEIGHT_PIXEL; x++)
                        *pData++ = pattern[(j % 2) ? BARCODE_WHITE : BARCODE_BLACK];
            }

            //fill gap between each barcode
            for (y = 0; y < BARCODE_BAR_PIXEL; y++)
                for (x = 0; x < BARCODE_HEIGHT_PIXEL; x++)
                    *pData++ = pattern[BARCODE_WHITE];
        }

        DEBUG((EFI_D_VERBOSE, "DrawBarCode: fill barcode OK!\n"));

        //3. fill end edge bar
        for (y = 0; y < BARCODE_EDGE_WIDTH_PIXEL; y++)
            for (x = 0; x < BARCODE_HEIGHT_PIXEL; x++)
                *pData++ = pattern[1];
    }

    /* Set image position */
    centerX = Width - BARCODE_HEIGHT_PIXEL;
    centerY = (Height>>1) - (bitmapHeight>>1);

    if (EFI_SUCCESS != (Status = GraphicsOutputProtocol->Blt(
                                                    GraphicsOutputProtocol,
                                                    (EFI_GRAPHICS_OUTPUT_BLT_PIXEL*)pBitmapImage,
                                                    EfiBltBufferToVideo,
                                                    0, 0,
                                                    centerX, centerY,
                                                    BARCODE_HEIGHT_PIXEL,
                                                    bitmapHeight,
                                                    0)))
    {
        DEBUG((EFI_D_ERROR, "DisplayBltOperationTest: Blt(EfiBltBufferToVideo) failed.\n"));
      goto Exit;
    }

Exit:
    if (pBitmapImage) {
        FreePool(pBitmapImage);
        pBitmapImage = NULL;
    }
    return Status;
}
// ASUS_BSP ---

EFI_STATUS DrawAndroidLogo()
{
    EFI_STATUS     Status = EFI_SUCCESS;
    void           *pBitmapImage = NULL;
    UINT32         Height = GetResolutionHeight();
    UINT32         Width = GetResolutionWidth();
    UINTN          centerX, centerY;
    UINT32         bitmapSize = SPLASH_ANDROID_IMAGE_WIDTH * PIXEL_SIZE * SPLASH_ANDROID_IMAGE_HEIGHT;

    if (!Width || !Height) {
      DEBUG ((EFI_D_ERROR, "Failed to get width or height\n"));
      return EFI_UNSUPPORTED;
    }

    pBitmapImage = AllocateZeroPool(bitmapSize);

    if (pBitmapImage == NULL) {
        DEBUG((EFI_D_ERROR, "DrawAndroidLogo: AllocateZeroPool failed\n"));
        goto Exit;
    } else {
        UINT32  *pData     = (UINT32*)pBitmapImage;
        UINT32  x;
        UINT32  tmpRed, tmpBlue;
        UINT32  finalRed;
        gST->ConOut->ClearScreen (gST->ConOut);

        for (x = 0; x < (SPLASH_ANDROID_IMAGE_HEIGHT*SPLASH_ANDROID_IMAGE_WIDTH); x++) {
            tmpRed  = ((AndroidLogo[x] & 0xFF) << 16);
            tmpBlue = (AndroidLogo[x] >> 16);
            finalRed  = (tmpRed | (AndroidLogo[x] & 0x00FFFF));
            AndroidLogo[x] = (tmpBlue | (finalRed & 0xFFFF00));
            *pData++ = (AndroidLogo[x] | 0xFF000000);
        }
    }

    /* Set image position */
    centerX = (Width>>1) - (SPLASH_ANDROID_IMAGE_HEIGHT>>1);
    centerY = 1068;

    if (EFI_SUCCESS != (Status = GraphicsOutputProtocol->Blt(
                                GraphicsOutputProtocol,
                                (EFI_GRAPHICS_OUTPUT_BLT_PIXEL*)pBitmapImage,
                                EfiBltBufferToVideo,
                                0, 0,
                                centerX, centerY,
                                SPLASH_ANDROID_IMAGE_HEIGHT, //X position
                                SPLASH_ANDROID_IMAGE_WIDTH, //Y position
                                0)))
    {
      DEBUG((EFI_D_ERROR, "DisplayBltOperationTest: Blt(EfiBltBufferToVideo) failed.\n"));
      goto Exit;
    }

Exit:
    if (pBitmapImage) {
        FreePool(pBitmapImage);
        pBitmapImage = NULL;
    }

    return Status;
}
#endif

//ASUS BSP Display +++
#if defined  ASUS_AI2205_BUILD && CN_BUILD 
// for CN
EFI_STATUS DrawTencentLogo()
{
    EFI_STATUS     Status = EFI_SUCCESS;
    void           *pBitmapImage = NULL;
    UINT32         Height = GetResolutionHeight();
    UINT32         Width = GetResolutionWidth();
    UINTN          centerX, centerY;
    UINT32         bitmapSize = Tencent_Width * PIXEL_SIZE * Tencent_Height;

    if (!Width || !Height) {
      DEBUG ((EFI_D_ERROR, "Failed to get width or height\n"));
      return EFI_UNSUPPORTED;
    }

    pBitmapImage = AllocateZeroPool(bitmapSize);

    if (pBitmapImage == NULL) {
        DEBUG((EFI_D_ERROR, "DrawCNLogo: AllocateZeroPool failed\n"));
        goto Exit;
    } else {
        UINT32  *pData     = (UINT32*)pBitmapImage;
        UINT32  x;
        UINT32  tmpRed, tmpBlue;
        UINT32  finalRed;
        DEBUG((EFI_D_VERBOSE, "DrawTencentLogo: start fill Tencent Logo bitmap\n"));

        for (x = 0; x < (Tencent_Height*Tencent_Width); x++) {
            tmpRed  = ((Tencent[x] & 0xFF) << 16);
            tmpBlue = (Tencent[x] >> 16);
            finalRed  = (tmpRed | (Tencent[x] & 0x00FFFF));
            Tencent[x] = (tmpBlue | (finalRed & 0xFFFF00));
            *pData++ = (Tencent[x] | 0xFF000000);
         }
        DEBUG((EFI_D_VERBOSE, "DrawTencentLogo: fill CN Logo OK!\n"));
    }

    /* Set image position */
    centerX = (Width>>1) - (Tencent_Height>>1);
    centerY = 1960;

    if (EFI_SUCCESS != (Status = GraphicsOutputProtocol->Blt(
                                GraphicsOutputProtocol,
                                (EFI_GRAPHICS_OUTPUT_BLT_PIXEL*)pBitmapImage,
                                EfiBltBufferToVideo,
                                0, 0,
                                centerX, centerY,
                                Tencent_Height, //X position
                                Tencent_Width, //Y position
                                0)))
    {
      DEBUG((EFI_D_ERROR, "DisplayBltOperationTest: Blt(EfiBltBufferToVideo) failed.\n"));
      goto Exit;
    }

Exit:
    if (pBitmapImage) {
        FreePool(pBitmapImage);
        pBitmapImage = NULL;
    }

    return Status;
}


EFI_STATUS DrawASUSCNLogo()
{
    EFI_STATUS     Status = EFI_SUCCESS;
    void           *pBitmapImage = NULL;
    UINT32         Height = GetResolutionHeight();
    UINT32         Width = GetResolutionWidth();
    UINTN          centerX, centerY;
    UINT32         bitmapSize = AsusLogoCN_Width * PIXEL_SIZE * AsusLogoCN_Height;

    if (!Width || !Height) {
      DEBUG ((EFI_D_ERROR, "Failed to get width or height\n"));
      return EFI_UNSUPPORTED;
    }

    pBitmapImage = AllocateZeroPool(bitmapSize);

    if (pBitmapImage == NULL) {
        DEBUG((EFI_D_ERROR, "DrawCNLogo: AllocateZeroPool failed\n"));
        goto Exit;
    } else {
        UINT32  *pData     = (UINT32*)pBitmapImage;
        UINT32  x;
        UINT32  tmpRed, tmpBlue;
        UINT32  finalRed;
        DEBUG((EFI_D_VERBOSE, "DrawCNLogo: start fill CN Logo bitmap\n"));
        gST->ConOut->ClearScreen (gST->ConOut);
        for (x = 0; x < (AsusLogoCN_Height*AsusLogoCN_Width); x++) {
            tmpRed  = ((AsusLogoCN[x] & 0xFF) << 16);
            tmpBlue = (AsusLogoCN[x] >> 16);
            finalRed  = (tmpRed | (AsusLogoCN[x] & 0x00FFFF));
            AsusLogoCN[x] = (tmpBlue | (finalRed & 0xFFFF00));
            *pData++ = (AsusLogoCN[x] | 0xFF000000);
         }
        DEBUG((EFI_D_VERBOSE, "DrawCNLogo: fill CN Logo OK!\n"));
    }

    /* Set image position */
    centerX = (Width>>1) - (AsusLogoCN_Height>>1);
    centerY = 1038;

    if (EFI_SUCCESS != (Status = GraphicsOutputProtocol->Blt(
                                GraphicsOutputProtocol,
                                (EFI_GRAPHICS_OUTPUT_BLT_PIXEL*)pBitmapImage,
                                EfiBltBufferToVideo,
                                0, 0,
                                centerX, centerY,
                                AsusLogoCN_Height, //X position
                                AsusLogoCN_Width, //Y position
                                0)))
    {
      DEBUG((EFI_D_ERROR, "DisplayBltOperationTest: Blt(EfiBltBufferToVideo) failed.\n"));
      goto Exit;
    }
    
    //Don't show tencent logo when CID = DIABLO
    if ((AsciiStrCmp ("DIABLO", cid_name)) ){
        DrawTencentLogo();
    }
    
Exit:
    if (pBitmapImage) {
        FreePool(pBitmapImage);
        pBitmapImage = NULL;
    }

    return Status;
}

EFI_STATUS DrawEliteCNLogo(UINT32 EWidth, UINT32 EHeight, UINT32 EliteCN[], int pos)
{
    EFI_STATUS     Status = EFI_SUCCESS;
    void           *pBitmapImage = NULL;
    UINT32         Height = GetResolutionHeight();
    UINT32         Width = GetResolutionWidth();
    UINTN          centerX, centerY;
    UINT32         bitmapSize = EWidth * PIXEL_SIZE * EHeight;

    if (!Width || !Height) {
      DEBUG ((EFI_D_ERROR, "Failed to get width or height\n"));
      return EFI_UNSUPPORTED;
    }

    pBitmapImage = AllocateZeroPool(bitmapSize);

    if (pBitmapImage == NULL) {
        DEBUG((EFI_D_ERROR, "DrawCNLogo: AllocateZeroPool failed\n"));
        goto Exit;
    } else {
        UINT32  *pData     = (UINT32*)pBitmapImage;
        UINT32  x;
        UINT32  tmpRed, tmpBlue;
        UINT32  finalRed;
        DEBUG((EFI_D_VERBOSE, "DrawEliteCNLogo: start fill EliteCN_Top Logo bitmap\n"));
        //clear display
        gST->ConOut->ClearScreen (gST->ConOut);
        for (x = 0; x < (EHeight*EWidth); x++) {
            tmpRed  = ((EliteCN[x] & 0xFF) << 16);
            tmpBlue = (EliteCN[x] >> 16);
            finalRed  = (tmpRed | (EliteCN[x] & 0x00FFFF));
            EliteCN[x] = (tmpBlue | (finalRed & 0xFFFF00));
            *pData++ = (EliteCN[x] | 0xFF000000);
         }
        DEBUG((EFI_D_VERBOSE, "DrawEliteCNLogo: fill EliteCN_Top Logo OK!\n"));
    }

    /* Set image position */
    centerX = (Width>>1) - (EHeight>>1);
    centerY = pos;

    if (EFI_SUCCESS != (Status = GraphicsOutputProtocol->Blt(
                                GraphicsOutputProtocol,
                                (EFI_GRAPHICS_OUTPUT_BLT_PIXEL*)pBitmapImage,
                                EfiBltBufferToVideo,
                                0, 0,
                                centerX, centerY,
                                EHeight, //X position
                                EWidth, //Y position
                                0)))
    {
      DEBUG((EFI_D_ERROR, "DisplayBltOperationTest: Blt(EfiBltBufferToVideo) failed.\n"));
      goto Exit;
    }

Exit:
    if (pBitmapImage) {
        FreePool(pBitmapImage);
        pBitmapImage = NULL;
    }

    return Status;
}

EFI_STATUS DrawAndroidCNLogo()
{
    EFI_STATUS     Status = EFI_SUCCESS;
    void           *pBitmapImage = NULL;
    UINT32         Height = GetResolutionHeight();
    UINT32         Width = GetResolutionWidth();
    UINTN          centerX, centerY;
    UINT32         bitmapSize = AndroidCN_Width * PIXEL_SIZE * AndroidCN_Height;

    if (!Width || !Height) {
      DEBUG ((EFI_D_ERROR, "Failed to get width or height\n"));
      return EFI_UNSUPPORTED;
    }

    pBitmapImage = AllocateZeroPool(bitmapSize);

    if (pBitmapImage == NULL) {
        DEBUG((EFI_D_ERROR, "DrawCNLogo: AllocateZeroPool failed\n"));
        goto Exit;
    } else {
        UINT32  *pData     = (UINT32*)pBitmapImage;
        UINT32  x;
        UINT32  tmpRed, tmpBlue;
        UINT32  finalRed;
        DEBUG((EFI_D_VERBOSE, "DrawAndroidCNLogo: start fill AndroidCN Logo bitmap\n"));
        gST->ConOut->ClearScreen (gST->ConOut);
        for (x = 0; x < (AndroidCN_Height*AndroidCN_Width); x++) {
            tmpRed  = ((AndroidCN[x] & 0xFF) << 16);
            tmpBlue = (AndroidCN[x] >> 16);
            finalRed  = (tmpRed | (AndroidCN[x] & 0x00FFFF));
            AndroidCN[x] = (tmpBlue | (finalRed & 0xFFFF00));
            *pData++ = (AndroidCN[x] | 0xFF000000);
         }
        DEBUG((EFI_D_VERBOSE, "DrawAndroidCNLogo: fill AndroidCN Logo OK!\n"));
    }

    /* Set image position */
    centerX = (Width>>1) - (AndroidCN_Height>>1);
    centerY = 2104;

    if (EFI_SUCCESS != (Status = GraphicsOutputProtocol->Blt(
                                GraphicsOutputProtocol,
                                (EFI_GRAPHICS_OUTPUT_BLT_PIXEL*)pBitmapImage,
                                EfiBltBufferToVideo,
                                0, 0,
                                centerX, centerY,
                                AndroidCN_Height, //X position
                                AndroidCN_Width, //Y position
                                0)))
    {
      DEBUG((EFI_D_ERROR, "DisplayBltOperationTest: Blt(EfiBltBufferToVideo) failed.\n"));
      goto Exit;
    }

  //DrawEliteCNLogo(EliteCN_Bot_Width,EliteCN_Bot_Height,EliteCN_Bot, 1171);
  DrawEliteCNLogo(EliteCN_Top_Width,EliteCN_Top_Height,EliteCN_Top, 899);

Exit:
    if (pBitmapImage) {
        FreePool(pBitmapImage);
        pBitmapImage = NULL;
    }

    return Status;
}
void DrawLogo() {

  DrawASUSCNLogo();

}
void DrawAndroid() {
  DrawAndroidLogo();
  MicroSecondDelay(500000);
  DrawEliteCNLogo(EliteCN_Top_Width,EliteCN_Top_Height,EliteCN_Top, 899);
}

#endif

#if defined  ASUS_AI2205_BUILD && !CN_BUILD
EFI_STATUS DrawEliteLogo()
{
    EFI_STATUS     Status = EFI_SUCCESS;
    void           *pBitmapImage = NULL;
    UINT32         Height = GetResolutionHeight();
    UINT32         Width = GetResolutionWidth();
    UINTN          centerX, centerY;
    UINT32         bitmapSize = SPLASH_ELITE_IMAGE_WIDTH * PIXEL_SIZE * SPLASH_ELITE_IMAGE_HEIGHT;

    if (!Width || !Height) {
      DEBUG ((EFI_D_ERROR, "Failed to get width or height\n"));
      return EFI_UNSUPPORTED;
    }

    pBitmapImage = AllocateZeroPool(bitmapSize);

    if (pBitmapImage == NULL) {
        DEBUG((EFI_D_ERROR, "DrawEliteLogo: AllocateZeroPool failed\n"));
        goto Exit;
    } else {
        UINT32  *pData     = (UINT32*)pBitmapImage;
        UINT32  x;
        UINT32  tmpRed, tmpBlue;
        UINT32  finalRed;
        DEBUG((EFI_D_VERBOSE, "DrawEliteLogo: start fill Elite Logo bitmap\n"));

        for (x = 0; x < (SPLASH_ELITE_IMAGE_HEIGHT*SPLASH_ELITE_IMAGE_WIDTH); x++) {
            tmpRed  = ((EliteLogo[x] & 0xFF) << 16);
            tmpBlue = (EliteLogo[x] >> 16);
            finalRed  = (tmpRed | (EliteLogo[x] & 0x00FFFF));
            EliteLogo[x] = (tmpBlue | (finalRed & 0xFFFF00));
            *pData++ = (EliteLogo[x] | 0xFF000000);
         }
        DEBUG((EFI_D_VERBOSE, "DrawEliteLogo: fill Elite Logo OK!\n"));
    }

    /* Set image position */
    centerX = (Width>>1) - (SPLASH_ELITE_IMAGE_HEIGHT>>1);
    centerY = 2165;

    if (EFI_SUCCESS != (Status = GraphicsOutputProtocol->Blt(
                                GraphicsOutputProtocol,
                                (EFI_GRAPHICS_OUTPUT_BLT_PIXEL*)pBitmapImage,
                                EfiBltBufferToVideo,
                                0, 0,
                                centerX, centerY,
                                SPLASH_ELITE_IMAGE_HEIGHT, //X position
                                SPLASH_ELITE_IMAGE_WIDTH, //Y position
                                0)))
    {
      DEBUG((EFI_D_ERROR, "DisplayBltOperationTest: Blt(EfiBltBufferToVideo) failed.\n"));
      goto Exit;
    }

Exit:
    if (pBitmapImage) {
        FreePool(pBitmapImage);
        pBitmapImage = NULL;
    }

    return Status;
}

EFI_STATUS DrawAsusLogo()
{
    EFI_STATUS     Status = EFI_SUCCESS;
    void           *pBitmapImage = NULL;
    UINT32         Height = GetResolutionHeight();
    UINT32         Width = GetResolutionWidth();
    UINTN          centerX, centerY;
    UINT32         bitmapSize = SPLASH_IMAGE_WIDTH * PIXEL_SIZE * SPLASH_IMAGE_HEIGHT;

    if (!Width || !Height) {
      DEBUG ((EFI_D_ERROR, "Failed to get width or height\n"));
      return EFI_UNSUPPORTED;
    }

    pBitmapImage = AllocateZeroPool(bitmapSize);

    if (pBitmapImage == NULL) {
        DEBUG((EFI_D_ERROR, "DrawAsusLogo: AllocateZeroPool failed\n"));
        goto Exit;
    } else {
        UINT32  *pData     = (UINT32*)pBitmapImage;
        UINT32  x;
        UINT32  tmpRed, tmpBlue;
        UINT32  finalRed;
        gST->ConOut->ClearScreen (gST->ConOut);
        DEBUG((EFI_D_VERBOSE, "DrawAsusLogo: start fill Asus Logo bitmap\n"));

        for (x = 0; x < (SPLASH_IMAGE_HEIGHT*SPLASH_IMAGE_WIDTH); x++) {
            tmpRed  = ((AsusLogo[x] & 0xFF) << 16);
            tmpBlue = (AsusLogo[x] >> 16);
            finalRed  = (tmpRed | (AsusLogo[x] & 0x00FFFF));
            AsusLogo[x] = (tmpBlue | (finalRed & 0xFFFF00));
            *pData++ = (AsusLogo[x] | 0xFF000000);
        }

        DEBUG((EFI_D_VERBOSE, "DrawAsusLogo: fill Asus Logo OK!\n"));
    }

    /* Set image position */
    centerX = (Width>>1) - (SPLASH_IMAGE_HEIGHT>>1);
    centerY = 1028;

    if (EFI_SUCCESS != (Status = GraphicsOutputProtocol->Blt(
                                GraphicsOutputProtocol,
                                (EFI_GRAPHICS_OUTPUT_BLT_PIXEL*)pBitmapImage,
                                EfiBltBufferToVideo,
                                0, 0,
                                centerX, centerY,
                                SPLASH_IMAGE_HEIGHT, //X position
                                SPLASH_IMAGE_WIDTH, //Y position
                                0)))
    {
      DEBUG((EFI_D_ERROR, "DisplayBltOperationTest: Blt(EfiBltBufferToVideo) failed.\n"));
      goto Exit;
    }
    DrawEliteLogo();
Exit:
    if (pBitmapImage) {
        FreePool(pBitmapImage);
        pBitmapImage = NULL;
    }

    return Status;
}

void DrawLogo() {

  DrawAsusLogo();

}
void DrawAndroid() {

  DrawAndroidLogo();

}
#endif
#if defined  ASUS_AI2205_BUILD
EFI_STATUS DrawAsusLogo_Charger()
{
    EFI_STATUS     Status = EFI_SUCCESS;
    void           *pBitmapImage = NULL;
    UINT32         Height = GetResolutionHeight();
    UINT32         Width = GetResolutionWidth();
    UINTN          centerX;
    UINTN          centerY;
    UINT32         bitmapHeight = SPLASH_IMAGE_WIDTH_CHARGER;
    UINT32         bitmapSize = SPLASH_IMAGE_WIDTH_CHARGER* PIXEL_SIZE * SPLASH_IMAGE_HEIGHT_CHARGER;

    pBitmapImage = AllocateZeroPool(bitmapSize);

    if (pBitmapImage == NULL) {
        DEBUG((EFI_D_ERROR, "DrawAsusLogo: AllocateZeroPool failed\n"));
        goto Exit;
    } else {

        UINT32  *pData     = (UINT32*)pBitmapImage;
        UINT32  x;
        gST->ConOut->ClearScreen (gST->ConOut);
        DEBUG((EFI_D_VERBOSE, "DrawAsusLogo: start fill Asus Logo bitmap\n"));

        for (x = 0; x < (SPLASH_IMAGE_HEIGHT_CHARGER*bitmapHeight); x++)
            *pData++ = (AsusLogo_charger[x] | 0xFF000000);

        DEBUG((EFI_D_VERBOSE, "DrawAsusLogo: fill Asus Logo OK!\n"));
    }

    /* Set image position */
    centerX = (Width>>1) - (SPLASH_IMAGE_HEIGHT_CHARGER>>1);
    centerY = (Height>>1) - (bitmapHeight>>1);

    if (EFI_SUCCESS != (Status = GraphicsOutputProtocol->Blt(
                                GraphicsOutputProtocol,
                                (EFI_GRAPHICS_OUTPUT_BLT_PIXEL*)pBitmapImage,
                                EfiBltBufferToVideo,
                                0, 0,
                                centerX, centerY,
                                SPLASH_IMAGE_HEIGHT_CHARGER, //X position
                                SPLASH_IMAGE_WIDTH_CHARGER, //Y position
                                0)))
    {
      DEBUG((EFI_D_ERROR, "DisplayBltOperationTest: Blt(EfiBltBufferToVideo) failed.\n"));
      goto Exit;
    }

Exit:
    if (pBitmapImage) {
        FreePool(pBitmapImage);
        pBitmapImage = NULL;
    }

    return Status;
}
#endif
//ASUS BSP Display  ---

/**
  Update the background color of the message
  @param[in]  MenuMsgInfo The message info.
  @param[in]  NewBgColor  The new background color of the message.
  @retval EFI_SUCCESS     The entry point is executed successfully.
  @retval other           Some error occurs when executing this entry point.
**/
EFI_STATUS
UpdateMsgBackground (MENU_MSG_INFO *MenuMsgInfo, UINT32 NewBgColor)
{
  MENU_MSG_INFO *target_msg_info = NULL;

  target_msg_info = AllocateZeroPool (sizeof (MENU_MSG_INFO));
  if (target_msg_info == NULL) {
    DEBUG ((EFI_D_ERROR, "Failed to allocate zero pool for message info.\n"));
    return EFI_OUT_OF_RESOURCES;
  }

  SetMenuMsgInfo (target_msg_info, MenuMsgInfo->Msg,
                  MenuMsgInfo->ScaleFactorType, MenuMsgInfo->FgColor,
                  NewBgColor, MenuMsgInfo->Attribute, MenuMsgInfo->Location,
                  MenuMsgInfo->Action);
  DrawMenu (target_msg_info, NULL);

  FreePool (target_msg_info);
  target_msg_info = NULL;

  return EFI_SUCCESS;
}

VOID DrawMenuInit (VOID)
{
  EFI_STATUS Status = EFI_SUCCESS;

  /* Backup the boot logo blt buffer before the screen gets changed */
  Status = BackUpBootLogoBltBuffer ();
  if (Status != EFI_SUCCESS)
    DEBUG ((EFI_D_VERBOSE, "Backup the boot logo blt buffer failed: %r\n",
            Status));

  /* Clear the screen before start drawing menu */
  gST->ConOut->ClearScreen (gST->ConOut);
}
