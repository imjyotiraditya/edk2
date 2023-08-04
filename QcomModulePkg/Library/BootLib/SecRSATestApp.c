/**
                SecRSATestApp

  Testing SecRSA protocol.

  Copyright (c) 2016 Qualcomm Technologies, Inc.  All Rights Reserved.
  Qualcomm Technologies Proprietary and Confidential.

**/

/*=============================================================================
                              EDIT HISTORY
  This section contains comments describing changes made to the module.
  Notice that changes are listed in reverse chronological order.



 when       who      what, where, why
 --------   ---     ----------------------------------------------------------
 08/26/16   SA      Initial version

=============================================================================*/

#include <Uefi.h>
#include <Library/UefiLib.h>
#include <Library/BaseLib.h>
#include <Library/DebugLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include "SecRSATestApp.h"
#include <Protocol/EFISecRSA.h>
#include <Protocol/EFIASN1X509.h>

#include "oem_certificate.h"
#include "VerifiedBoot.h"
#include <Protocol/Hash2.h>

#define EFI_FAILURE (80)

#define SHA256_DIGEST_LENGTH	32

//#define HASH_ENABLE 1

STATIC CERTIFICATE oem_certificate = {0};

QcomSecRsaProtocol *pEfiQcomSecRSAProtocol = NULL;

/* Global varaible to keep Android Verified Boot Signature (appended to the image) */
//STATIC VERIFIED_BOOT_SIG embedded_vb_signature = {0};

/**

	Read the OEM certificate from the embedded header file
	and verify whether it follows the ASN1 standard format

	@param[out] 	*certificate	Verified certificate (output)

	@retval         status          Indicates whether reading certificate and verifying its format was successful

**/
//call by RsaVerify()
EFI_STATUS vb_read_oem_certificate(QcomAsn1x509Protocol *pEfiQcomASN1X509Protocol, CERTIFICATE *certificate,UINT32 decrypt_type)
{
  	EFI_STATUS status = EFI_FAILURE;
  	UINT8 *input = NULL;
  	UINTN len = 0;
	
  	switch (decrypt_type) 
	{
  		case USER_UNLOCK:
		input = USER_UNLOCK_CERTIFICATE;
		len = sizeof(USER_UNLOCK_CERTIFICATE);
		break;

		case FRP_UNLOCK:
		input = FRP_CERTIFICATE;
		len = sizeof(FRP_CERTIFICATE);
    		break;

		case RAW_PKG_LOCK:
		input = RAW_CERTIFICATE;
		len = sizeof(RAW_CERTIFICATE);
    		break;
						
  		default:
    		status = EFI_UNSUPPORTED;
		DEBUG((EFI_D_ERROR, "[ABL] (ERROR) vb_read_oem_certificate : UNSUPPORT decrypt  type = %d\n",decrypt_type));	
    		goto exit;
  	}
	
	DEBUG((EFI_D_ERROR, "[ABL] vb_read_oem_certificate : OEM_CERTIFICATE type = %d, len = %d\n",decrypt_type, len));

  	status = pEfiQcomASN1X509Protocol->ASN1X509VerifyOEMCertificate(pEfiQcomASN1X509Protocol, input, len, certificate);
  	if (status != EFI_SUCCESS) 
	{
		DEBUG((EFI_D_ERROR, "[ABL] (ERROR) vb_read_oem_certificate : ASN1X509VerifyOEMCertificate fail : %r\n", status));

    		goto exit;
  	}
  	status = EFI_SUCCESS;
	
exit:
  	return status;
}

//call by vb_verify_hash_oem_certificate()
EFI_STATUS VerifySignature(
    	CONST UINT8 *signature_ptr,
    	UINT32 signature_len,
    	UINT8 *hash,
    	UINT32 hash_sz,
    	VB_HASH hash_algorithm,//VB_SHA256
    	CONST UINT8 *modulus,
    	UINT32 modulus_len,
    	CONST UINT8 *public_exp,
    	UINT32 public_exp_len)
{

  	EFI_STATUS status = EFI_FAILURE;
  	CE_RSA_KEY key;
  	BigInt modulus_bi;
  	BigInt public_exp_bi;
  	INT32 hashidx;
  	INT32 hash_len;
  	UINT32 padding_type;
  	VOID *padding_info = NULL;
  	QcomSecRsaProtocol *pEfiQcomSecRSAProtocol = NULL;
  	SetMem(&key, sizeof(CE_RSA_KEY), 0);

	DEBUG((EFI_D_ERROR, "[ABL] +++ VerifySignature ()\n"));

  	switch (hash_algorithm) 
	{
  		case VB_SHA256:
		DEBUG((EFI_D_ERROR, "[ABL] VerifySignature : VB_SHA256\n"));


    		hashidx = CE_HASH_IDX_SHA256;
    		hash_len = VB_SHA256_SIZE;


		break;

		case VB_UNSUPPORTED_HASH:
		DEBUG((EFI_D_ERROR, "[ABL] VerifySignature : VB_UNSUPPORTED_HASH\n"));

    		hashidx = CE_HASH_IDX_NULL;
    		hash_len = (INT32) hash_sz;			
    		break;
			
  		default:
    		DEBUG((EFI_D_ERROR, "[ABL] (ERROR) VerifySignature: Hash algorithm not supported\n"));
    		status = EFI_UNSUPPORTED;
    		goto exit;
  	}

	DEBUG((EFI_D_ERROR, "[ABL] VerifySignature : hashidx = %d,hash_len = %d\n",hashidx,hash_len));

  	// 1. alocate (CE_RSA_KEY) key-> modulus 
  	key.N = AllocatePool(sizeof(S_BIGINT));
  	if (key.N == NULL) 
	{
    		DEBUG((EFI_D_ERROR, "[ABL] (ERROR) VerifySignature: mem allocation err for key.N\n"));
    		goto exit;
  	}

	// 2. alocate (CE_RSA_KEY) key-> public exponent 	
  	key.e = AllocatePool(sizeof(S_BIGINT));
  	if (key.e == NULL) {
    		DEBUG((EFI_D_ERROR, "[ABL] (ERROR) VerifySignature: mem allocation err for key.e\n"));
    		goto exit;
  	}

  	// 3. load gEfiQcomSecRSAProtocolGuid
  	status = gBS->LocateProtocol(&gEfiQcomSecRSAProtocolGuid, NULL, (VOID **) &pEfiQcomSecRSAProtocol);
  	if ( status != EFI_SUCCESS)
  	{
     		DEBUG((EFI_D_ERROR, "[ABL] (ERROR) VerifySignature: LocateProtocol failed, status = %x\n", status));
    		goto exit;
  	}

  	// 4. get modulus_bigInt
  	status = pEfiQcomSecRSAProtocol->SecRSABigIntReadBin(pEfiQcomSecRSAProtocol, modulus, modulus_len, &modulus_bi);
  	if ( status != EFI_SUCCESS)
  	{
    		DEBUG((EFI_D_ERROR, "[ABL] (ERROR) VerifySignature: SecRSABigIntReadBin for modulus failed!, ret = %x\n", status));
    		goto exit;
  	}

	// 4. get public_exp_bigInt
  	status = pEfiQcomSecRSAProtocol->SecRSABigIntReadBin(pEfiQcomSecRSAProtocol, public_exp, public_exp_len,  &public_exp_bi);
  	if ( status != EFI_SUCCESS)
  	{
		DEBUG((EFI_D_ERROR, "[ABL] (ERROR) VerifySignature: SecRSABigIntReadBin for modulus failed!, ret = %x\n", status));
		goto exit;
  	}

  	key.N->Bi = modulus_bi;
  	key.e->Bi = public_exp_bi;
  	key.e->Sign = S_BIGINT_POS;
  	key.Type = CE_RSA_KEY_PUBLIC;
  	padding_type = CE_RSA_PAD_PKCS1_V1_5_SIG;

	DEBUG((EFI_D_ERROR, "[ABL] VerifySignature : call SecRSAVerifySig()\n"));

	#if 0
	UINT32 i=0;
	DEBUG((EFI_D_ERROR, "[ABL] signature_ptr[%d] = \n",signature_len));
	for(i=0;i<signature_len;i++)
	{
			DEBUG((EFI_D_ERROR, " %02x",signature_ptr[i] ));
	}
	DEBUG((EFI_D_ERROR, "\n"));

	DEBUG((EFI_D_ERROR, "[ABL] hash[%d] = \n",hash_len));
	for(i=0;i<32;i++)
	{
			DEBUG((EFI_D_ERROR, " %02x",hash[i] ));
	}
	DEBUG((EFI_D_ERROR, "\n"));
	
	#endif
		
  	status = pEfiQcomSecRSAProtocol->SecRSAVerifySig(pEfiQcomSecRSAProtocol, 
  												&key, 
  												padding_type,
  												padding_info,
  												hashidx,
                                   							hash, 
                                   							hash_len,
                                   							(UINT8*)signature_ptr,
                                   							signature_len);
  	if (status != EFI_SUCCESS) 
	{
		DEBUG((EFI_D_ERROR, "[ABL] VerifySignature : SecRSAVerifySig() : FAIL !!\n"));

    		goto exit;
  	}

	DEBUG((EFI_D_ERROR, "[ABL] VerifySignature : SecRSAVerifySig() : PASS !!\n"));

  	status = EFI_SUCCESS;
	
exit:
  	if (key.N != NULL) {
    		FreePool(key.N);
  	}
  	if (key.e != NULL) {
    		FreePool(key.e);
  	}

	DEBUG((EFI_D_ERROR, "[ABL] -- VerifySignature - return %d\n",status));

  	return status;
}

UINTN CopyMemS
(  
  IN VOID   *Destination,
  IN UINTN  DestLength, 
  IN const VOID   *Source,
  IN UINTN  SrcLength
) 
{
   if(DestLength >= SrcLength) {
      CopyMem(Destination, Source, SrcLength);
      return SrcLength;
    }
  
    CopyMem(Destination, Source, DestLength);
    return DestLength;
}

EFI_STATUS get_image_hash(
	UINT8 *img, 
	UINTN img_len,
       UINT8 *out_hash,
       UINTN out_hash_size, 
       VB_HASH hash_algorithm) //=VB_SHA256
{

  	EFI_STATUS status = EFI_FAILURE;
  	EFI_GUID *HashAlgorithm;
  	UINTN DigestSize = 0;
  	EFI_HASH2_OUTPUT Hash2Output;
  	EFI_HASH2_PROTOCOL *pEfiHash2Protocol = NULL;


	DEBUG((EFI_D_ERROR, "\n"));

	DEBUG((EFI_D_ERROR, "[ABL] +++ get_image_hash : img = %a,img_len = %d\n", img,img_len));

  	switch (hash_algorithm) 
	{
  		case VB_SHA256:
    		HashAlgorithm = &gEfiHashAlgorithmSha256Guid;
    		break;
			
  		default:			
    		DEBUG((EFI_D_ERROR, "[ABL] (ERROR) get_image_hash: not supported algorithm: %d \n", hash_algorithm));
    		status = EFI_UNSUPPORTED;
    		goto exit;
  	}

  	if (pEfiHash2Protocol == NULL) 
	{
    		status = gBS->LocateProtocol(&gEfiHash2ProtocolGuid,
                                 NULL,
                                 (VOID **)&pEfiHash2Protocol);
    		if (status != EFI_SUCCESS) 
		{

      			DEBUG((EFI_D_ERROR, "[ABL] (ERROR) get_image_hash: LocateProtocol unsuccessful! status: %d\n", status));
      			goto exit;
    		}
  	}
	
  	status = pEfiHash2Protocol->GetHashSize(pEfiHash2Protocol, HashAlgorithm,&DigestSize);
	
  	if (status != EFI_SUCCESS) 
	{

    		DEBUG((EFI_D_ERROR, "[ABL] (ERROR) get_image_hash: GetHashSize unsuccessful! status: %d\n", status));
    		goto exit;
  	}

	DEBUG((EFI_D_ERROR, "[ABL] get_image_hash : DigestSize = %d\n", DigestSize));

  	if (out_hash_size != DigestSize) 
	{
    		DEBUG((EFI_D_ERROR, "[ABL] (ERROR) get_image_hash: Invalid size! out_hash_size: %d, DigestSize: %d\n", out_hash_size, DigestSize));
    		status = EFI_FAILURE;
    		goto exit;
  	}
	
  	status = pEfiHash2Protocol->HashInit(pEfiHash2Protocol, HashAlgorithm);
	
  	if (status != EFI_SUCCESS) 
	{
    		DEBUG((EFI_D_ERROR, "[ABL] (ERROR) get_image_hash: HashInit unsuccessful! status: %d\n", status));
    		goto exit;
  	}
	
  	status = pEfiHash2Protocol->HashUpdate(pEfiHash2Protocol, img, img_len);
  	if (EFI_SUCCESS != status) 
	{

    		DEBUG((EFI_D_ERROR, "[ABL] (ERROR) get_image_hash: HashUpdate unsuccessful(img)! status: %d\n", status));
    		goto exit;
  	}

  	status = pEfiHash2Protocol->HashFinal(pEfiHash2Protocol, &Hash2Output);

  	if (EFI_SUCCESS != status) 
	{

    		DEBUG((EFI_D_ERROR, "[ABL] (ERROR) get_image_hash: HashFinal unsuccessful! status = %d\n", status));
    		goto exit;
  	}
	
  	CopyMemS(out_hash, out_hash_size, (CONST UINT8 *)&Hash2Output, DigestSize);

	#if 0
	UINT32 i=0;
	UINT32 sz=DigestSize;

	DEBUG((EFI_D_ERROR, "\n"));
	DEBUG((EFI_D_ERROR, "[ABL] out_hash[%d] = \n",sz));
	for(i=0;i<sz;i++)
	{
		DEBUG((EFI_D_ERROR, " %02x",out_hash[i] ));
	}
	DEBUG((EFI_D_ERROR, "\n"));
	#endif  

	DEBUG((EFI_D_ERROR, "[ABL] --- get_image_hash : img = %a,img_len = %d\n", img,img_len));

  	status = EFI_SUCCESS;

exit:
  	return status;
}

EFI_STATUS RsaVerify(UINT8 *signature_ptr, UINT32 signature_len, UINT8 *plain_text, UINT32 plain_text_len,UINT32 decrypt_type)
{
  	EFI_STATUS status = EFI_FAILURE;

  	RSA key = {0};
  	secasn1_data_type modulus = {0};
  	secasn1_data_type public_exp = {0};
  	//secasn1_data_type signature = {0};

  	VB_HASH hash_algorithm = VB_UNSUPPORTED_HASH;

	DEBUG((EFI_D_ERROR, "[ABL] +++ RsaVerify : signature_len = %d,plain_text_len = %d\n", signature_len,plain_text_len));

	/* ASN1X509 Protocol */
	STATIC QcomAsn1x509Protocol *pEfiQcomASN1X509Protocol = NULL;

	status = gBS->LocateProtocol(&gEfiQcomASN1X509ProtocolGuid, NULL,(VOID **)&pEfiQcomASN1X509Protocol);

	if (status != EFI_SUCCESS)
	{
		DEBUG((EFI_D_ERROR, "[ABL] (ERROR) RsaVerify : locateProtocolo fail : %r\n", status));
		return status ;
	}	

  	if (vb_read_oem_certificate(pEfiQcomASN1X509Protocol, &oem_certificate ,decrypt_type) != EFI_SUCCESS) 
	{
		DEBUG((EFI_D_ERROR, "[ABL] (ERROR) RsaVerify : vb_read_oem_certificate fail : %r\n", status));
		return status ;
  	}

  	// 1. get key from  oem_certificate[] : OEM_CERTIFICATE[]
  	status = pEfiQcomASN1X509Protocol->ASN1X509GetRSAFromCert(pEfiQcomASN1X509Protocol, &oem_certificate, &key);

	if (status != EFI_SUCCESS) 
	{
		DEBUG((EFI_D_ERROR, "[ABL] (ERROR) RsaVerify : ASN1X509GetRSAFromCert fail : %r\n", status));
		return status ;
	}

  	// 2. Get Key material from key : output : modulus, public_exp
  	status = pEfiQcomASN1X509Protocol->ASN1X509GetKeymaterial(pEfiQcomASN1X509Protocol, &key, &modulus, &public_exp);
  	if (status != EFI_SUCCESS) 
	{
		DEBUG((EFI_D_ERROR, "[ABL] (ERROR) RsaVerify : ASN1X509GetRSAFromCert fail : %r\n", status));
		return status ;
	}

	#if 0
	UINT32 i=0;
	UINT32 sz=modulus.len;

	DEBUG((EFI_D_ERROR, "[ABL] modulus[%d].data = \n",sz));
	for(i=0;i<sz;i++)
	{
		DEBUG((EFI_D_ERROR, "%02x:",modulus.data[i] ));
	}
	DEBUG((EFI_D_ERROR, "\n"));
	#endif  
	
  	status = VerifySignature(signature_ptr,//signature.data,
  						signature_len,//signature.len,
  						plain_text, 			//img_hash : uuid id 
  						plain_text_len,
  						hash_algorithm,//VB_SHA256
                           			modulus.data,
                           			modulus.Len,
                           			public_exp.data,
		                            public_exp.Len);

  	if (status != EFI_SUCCESS) 
	{
		DEBUG((EFI_D_ERROR, "[ABL] (ERROR) RsaVerify : VerifySignature fail : %r\n", status));
		return status ;
	}	
	else
	{
		DEBUG((EFI_D_ERROR, "[ABL] RsaVerify : VerifySignature PASS\n"));
	}
	
	return status;
}

//call by CmdRsaTest()
EFI_STATUS
EFIAPI
SecRSATestAppMain(
	UINT8 *signature_ptr, 
	UINT32 signature_len, 
	UINT8 *plain_text, 
	UINT32 plain_text_len,
	UINT32 decrypt_type)
{
  	EFI_STATUS Status = EFI_FAILURE;

	DEBUG((EFI_D_ERROR, "[ABL] SecRSATestAppMain (decrypt_type = %d): plain_text (%a,sz=%d)\n",decrypt_type, plain_text,plain_text_len));

	Status = RsaVerify(signature_ptr,signature_len,plain_text,plain_text_len,decrypt_type);
	
  	if (Status != EFI_SUCCESS) 
	{
    		DEBUG((EFI_D_ERROR, "[ABL] (ERROR) VerifyImage : FAIL (%d)\n", Status));
		return Status;
	}	

  	return Status;
}

