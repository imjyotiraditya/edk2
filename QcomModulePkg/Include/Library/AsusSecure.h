#ifndef __ASUSSECURE_H__
#define __ASUSSECURE_H__

#include <Uefi.h>
#include <Library/DebugLib.h>
#include <Library/PrintLib.h>
#include <Library/DeviceInfo.h>
#include <Library/BaseMemoryLib.h>

/* for user unlock reboot_mode value */
#define SECURITY_UNLOCK_SUCCESS                0
#define SECURITY_UNLOCK_FAILURE                1
#define SECURITY_SIG_SUCCESS	               0
#define SECURITY_SIG_FAILURE	               1
#define NOT_ASUS_IMAGE			       2
#include "Board.h"


#define SIGNATURE_SIZE	256
#define MAX_RSP_SIZE	 64
#define IMEI_SIZE 15

#define USE_CPUID 	1

#ifdef USE_CPUID
#define	ID_SIZE		8
#else
#define	ID_SIZE		16
#endif

#define ISN_SIZE		15

#ifdef ENABLE_NONSPARSE_IMG_SECURITY_FLASH_FUNC
#define ASUS_FOOTER_SIZE 2048 // for boot.img if signed for security flashing
#endif

#ifdef ENABLE_SPARSE_IMG_SECURITY_FLASH_FUNC
/* for raw image */
#define ASUS_SHA256_SIZE 32
#define ASUS_SPARSE_MAGIC	"ASUS MAGIC!"  // 11 chars in 28 space
#define ASUS_SPARSE_MAGIC_WW	"ASUS WW_SKU"  // 11 chars in 28 space
#define ASUS_SPARSE_MAGIC_CN	"ASUS CN_SKU"  // 11 chars in 28 space
#define ASUS_SPARSE_MAGIC_STRING_SIZE		11
#define ASUS_SPARSE_MAGIC_WW_STRING_SIZE		11
#define ASUS_SPARSE_MAGIC_CN_STRING_SIZE		11
#define ASUS_SPARSE_MAGIC_BLOCK_SIZE		28

/*The values here must match with the signing script */
#define PACKAGE_10MB 10*1024*1024
#define SPARSE_HEADER_SIZE 28
#define SIGNATURE_CHUNK_HEADER_LEN 12
#define SIGNATURE_CHUNK_DATA_LEN 4096
#define SIGNATURE_CHUNK_SIZE (SIGNATURE_CHUNK_HEADER_LEN + SIGNATURE_CHUNK_DATA_LEN)

#define ASUS_SPARSE_FLASH_DEBUG 1

int verify_asus_sparse_img(UINT8 *buf, int buf_size, UINT8 *signature, int signature_size);

#endif //#ifdef ENABLE_SPARSE_IMG_SECURITY_FLASH_FUNC

UINT8 GetUnlockedStatus(VOID);
int verify_asuskey(UINT8 *, int);
#ifdef ENABLE_NONSPARSE_IMG_SECURITY_FLASH_FUNC
int verify_asus_img(UINT8 *, int);
#endif //#ifdef ENABLE_NONSPARSE_IMG_SECURITY_FLASH_FUNC
int verify_official_unlock(void);
int verify_RD_unlock(void);
int verify_RT_unlock(void);
int verify_WM_unlock(void);
BOOLEAN has_asus_footer_magic(UINT8 *buf, int size);
int FRP_verify(unsigned char *FRP_ptr,
		unsigned int FRP_size,
		unsigned char *signature_ptr,
		unsigned int signature_size
		);
BOOLEAN
EFIAPI
X509_decrypt_sig(
  IN   CONST UINT8  *Cert,
  IN   UINTN        CertSize,
  IN   CONST UINT8  *signature_ptr,
  IN   UINTN        signature_size,
  OUT  UINT8        **plain_text
  );

#endif
