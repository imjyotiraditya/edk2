#include <Library/BMPLib.h>

#define EFI_DISPLAYUTILS_PROTOCOL_GUID \
    {0xEB13EAE7, 0x7B4B, 0x4F26, { 0xA3, 0xDD, 0xEC, 0xA7, 0xCB, 0xDE, 0x25, 0x5B } }

extern EFI_GUID gEfiBmpProtocolGuid;

typedef struct _EFI_QCOM_BMP_PROTOCOL EFI_QCOM_BMP_PROTOCOL;

typedef
EFI_STATUS (EFIAPI *EFI_DRAW_BMP_FILE)(
  IN   CHAR8               *FileName,
  IN   BMPLIB_OPTION       Opts[]         OPTIONAL,
  IN   UINTN               NumOpts,
  IN   EFI_GUID            *FvNameGuid        OPTIONAL
);

struct _EFI_QCOM_BMP_PROTOCOL
{ 
  EFI_DRAW_BMP_FILE  DrawBmpFile;
};
