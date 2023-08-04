/* Copyright (c) 2016, The Linux Foundation. All rights reserved.
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
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiRuntimeServicesTableLib.h>
#include <Library/UefiLib.h>
#include <Library/DebugLib.h>
#include <Library/BaseLib.h>
#include <Protocol/EFIPmicGpio.h> //+++ ASUS_BSP : add for get & write pmic reg value
#include "BootLinux.h"
#include "DeviceInfo.h"
#include "abl.h"
#include "ShutdownServices.h"
#include "SecRSATestApp.h" // +++ ASUS_BSP : add for user unlock
#include <Protocol/EFIDDRGetConfig.h>
#include <UpdateDeviceTree.h>
#include <Md5.h>
#include "libavb/libavb.h"

#ifdef ASUS_BUILD
////////////////////////////////////////////////////////////////////////
// ROG7 : PROJECT COMMON DEFINITION & STRUCTURE
////////////////////////////////////////////////////////////////////////
extern DeviceInfo DevInfo;

char asus_project_info[32]    = {0};
char asus_stage_info[32]      = {0};
char cid_name[32]             = {0};

/***** ASUS_CMDLINE *****/
// +++ ASUS_BSP : add for ftm mode
char cmd_enable_adb_mode[64]  = {0};
char cmd_enable_adb_prop[64]  = {0};
char cmd_ftm_mode[64]         = {0};
char cmd_ftm_mode_prop[64]    = {0};
char cmd_selinux[64]          = {0};
char cmd_selinux_prop[64]     = {0};
//+++ ASUS_BSP : add for ASUS ID value
char cmd_prj_id[64]           = {0};
char cmd_stage_id[64]         = {0};
char cmd_sku_id[64]           = {0};
char cmd_ddr_id[64]           = {0};
char cmd_nfc_id[64]           = {0};
char cmd_rf_id[64]            = {0};
char cmd_fp_id[64]            = {0};
char cmd_feature_id[64]       = {0};
char cmd_jtag_id[64]          = {0};
char cmd_pcb_id[64]           = {0};
char cmd_lgf_con_id[64]       = {0};
char cmd_fc_id[64]            = {0};
char cmd_upper_id[64]         = {0};
char cmd_sub_id[64]           = {0};
// +++ ASUS_BSP : add for boot count
char cmd_boot_count[64]       = {0};
// +++ ASUS_BSP : add for read country code
char cmd_country_code[64]     = {0};
char cmd_product_name[64]     = {0};
char cmd_toolid[64]           = {0};
// +++ ASUS_BSP : check if have rawdump partition or not
char cmd_rawdump_en[32]       = {0};
// +++ ASUS_BSP : add for ASUS dongle unlock
char cmd_authorized_prop[64]  = {0};
// +++ ASUS_BSP : add for unlock state
char cmd_unlock[16]           = {0};
// +++ ASUS_BSP : add for WaterMask unlock
char cmd_watermask_unlock[16] = {0};
char cmd_watermask_unlock_prop[64] = {0};
// +++ ASUS_BSP : add for fuse blow
char cmd_fuse_Info[16]        = {0};
char cmd_fuse_prop[64]        = {0};
// +++ ASUS_BSP : add for logcat-asdf sevices
char cmd_enable_logcat_asdf[64]={0};

char cmd_fuse_no_rpmb_Info[16] = {0};
char cmd_fuse_no_rpmb_prop[64] = {0};
// +++ ASUS_BSP : add for check factory crc
char cmd_factory_crc[64]       = {0};

// +++ ASUS_BSP : add for ddr info
//char cmd_ddr_manufacturer[64] = {0};
//char cmd_ddr_device_type[64]  = {0};
char cmd_cpuid_hash[64]       = {0};
// +++ ASUS_BSP : add for panel uid
char cmd_unique_id[64]={0};
char cmd_lgf_id[64]            = {0};
char cmd_asus_hwid[64]         = {0};
char cmd_uart_status[64]={0};
// +++ ASUS_BSP : add for NFC check hardware sku
char cmd_hardware_sku_prop[64]    = {0};
BOOLEAN adb_enter_shipping_mode = FALSE;
/***** ASUS_CRC_32 *****/
UINT32  mCrcTable[256] = {
  0x00000000, 0x77073096, 0xEE0E612C, 0x990951BA, 0x076DC419, 0x706AF48F, 0xE963A535, 0x9E6495A3,
  0x0EDB8832, 0x79DCB8A4, 0xE0D5E91E, 0x97D2D988, 0x09B64C2B, 0x7EB17CBD, 0xE7B82D07, 0x90BF1D91,
  0x1DB71064, 0x6AB020F2, 0xF3B97148, 0x84BE41DE, 0x1ADAD47D, 0x6DDDE4EB, 0xF4D4B551, 0x83D385C7,
  0x136C9856, 0x646BA8C0, 0xFD62F97A, 0x8A65C9EC, 0x14015C4F, 0x63066CD9, 0xFA0F3D63, 0x8D080DF5,
  0x3B6E20C8, 0x4C69105E, 0xD56041E4, 0xA2677172, 0x3C03E4D1, 0x4B04D447, 0xD20D85FD, 0xA50AB56B,
  0x35B5A8FA, 0x42B2986C, 0xDBBBC9D6, 0xACBCF940, 0x32D86CE3, 0x45DF5C75, 0xDCD60DCF, 0xABD13D59,
  0x26D930AC, 0x51DE003A, 0xC8D75180, 0xBFD06116, 0x21B4F4B5, 0x56B3C423, 0xCFBA9599, 0xB8BDA50F,
  0x2802B89E, 0x5F058808, 0xC60CD9B2, 0xB10BE924, 0x2F6F7C87, 0x58684C11, 0xC1611DAB, 0xB6662D3D,
  0x76DC4190, 0x01DB7106, 0x98D220BC, 0xEFD5102A, 0x71B18589, 0x06B6B51F, 0x9FBFE4A5, 0xE8B8D433,
  0x7807C9A2, 0x0F00F934, 0x9609A88E, 0xE10E9818, 0x7F6A0DBB, 0x086D3D2D, 0x91646C97, 0xE6635C01,
  0x6B6B51F4, 0x1C6C6162, 0x856530D8, 0xF262004E, 0x6C0695ED, 0x1B01A57B, 0x8208F4C1, 0xF50FC457,
  0x65B0D9C6, 0x12B7E950, 0x8BBEB8EA, 0xFCB9887C, 0x62DD1DDF, 0x15DA2D49, 0x8CD37CF3, 0xFBD44C65,
  0x4DB26158, 0x3AB551CE, 0xA3BC0074, 0xD4BB30E2, 0x4ADFA541, 0x3DD895D7, 0xA4D1C46D, 0xD3D6F4FB,
  0x4369E96A, 0x346ED9FC, 0xAD678846, 0xDA60B8D0, 0x44042D73, 0x33031DE5, 0xAA0A4C5F, 0xDD0D7CC9,
  0x5005713C, 0x270241AA, 0xBE0B1010, 0xC90C2086, 0x5768B525, 0x206F85B3, 0xB966D409, 0xCE61E49F,
  0x5EDEF90E, 0x29D9C998, 0xB0D09822, 0xC7D7A8B4, 0x59B33D17, 0x2EB40D81, 0xB7BD5C3B, 0xC0BA6CAD,
  0xEDB88320, 0x9ABFB3B6, 0x03B6E20C, 0x74B1D29A, 0xEAD54739, 0x9DD277AF, 0x04DB2615, 0x73DC1683,
  0xE3630B12, 0x94643B84, 0x0D6D6A3E, 0x7A6A5AA8, 0xE40ECF0B, 0x9309FF9D, 0x0A00AE27, 0x7D079EB1,
  0xF00F9344, 0x8708A3D2, 0x1E01F268, 0x6906C2FE, 0xF762575D, 0x806567CB, 0x196C3671, 0x6E6B06E7,
  0xFED41B76, 0x89D32BE0, 0x10DA7A5A, 0x67DD4ACC, 0xF9B9DF6F, 0x8EBEEFF9, 0x17B7BE43, 0x60B08ED5,
  0xD6D6A3E8, 0xA1D1937E, 0x38D8C2C4, 0x4FDFF252, 0xD1BB67F1, 0xA6BC5767, 0x3FB506DD, 0x48B2364B,
  0xD80D2BDA, 0xAF0A1B4C, 0x36034AF6, 0x41047A60, 0xDF60EFC3, 0xA867DF55, 0x316E8EEF, 0x4669BE79,
  0xCB61B38C, 0xBC66831A, 0x256FD2A0, 0x5268E236, 0xCC0C7795, 0xBB0B4703, 0x220216B9, 0x5505262F,
  0xC5BA3BBE, 0xB2BD0B28, 0x2BB45A92, 0x5CB36A04, 0xC2D7FFA7, 0xB5D0CF31, 0x2CD99E8B, 0x5BDEAE1D,
  0x9B64C2B0, 0xEC63F226, 0x756AA39C, 0x026D930A, 0x9C0906A9, 0xEB0E363F, 0x72076785, 0x05005713,
  0x95BF4A82, 0xE2B87A14, 0x7BB12BAE, 0x0CB61B38, 0x92D28E9B, 0xE5D5BE0D, 0x7CDCEFB7, 0x0BDBDF21,
  0x86D3D2D4, 0xF1D4E242, 0x68DDB3F8, 0x1FDA836E, 0x81BE16CD, 0xF6B9265B, 0x6FB077E1, 0x18B74777,
  0x88085AE6, 0xFF0F6A70, 0x66063BCA, 0x11010B5C, 0x8F659EFF, 0xF862AE69, 0x616BFFD3, 0x166CCF45,
  0xA00AE278, 0xD70DD2EE, 0x4E048354, 0x3903B3C2, 0xA7672661, 0xD06016F7, 0x4969474D, 0x3E6E77DB,
  0xAED16A4A, 0xD9D65ADC, 0x40DF0B66, 0x37D83BF0, 0xA9BCAE53, 0xDEBB9EC5, 0x47B2CF7F, 0x30B5FFE9,
  0xBDBDF21C, 0xCABAC28A, 0x53B39330, 0x24B4A3A6, 0xBAD03605, 0xCDD70693, 0x54DE5729, 0x23D967BF,
  0xB3667A2E, 0xC4614AB8, 0x5D681B02, 0x2A6F2B94, 0xB40BBE37, 0xC30C8EA1, 0x5A05DF1B, 0x2D02EF8D
};
////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////
// +++ ROG7 : PROJECT FUNCTION
////////////////////////////////////////////////////////////////////////
/***** ASUS_GET_ID_VALUE *****/
UINT32 Get_HW_ID()
{
	EFI_STATUS Status;
	UINT32 ID_VALUE = 0;
	UINTN t_ID_SIZE = sizeof(ID_VALUE);

	Status = gRT->GetVariable(
			L"HW_ID",
			&gQcomTokenSpaceGuid,
			NULL,
			&t_ID_SIZE,
			&ID_VALUE);

	if (Status != EFI_SUCCESS)
		DEBUG((EFI_D_ERROR, "[ABL] (ERROR) Get_HW_ID failed : %r \n", Status));

	return ID_VALUE;
}

UINT32 Get_LGF_ID()
{
	EFI_STATUS Status;
	UINT32 ID_VALUE = 0;
	UINTN t_ID_SIZE = sizeof(ID_VALUE);

	Status = gRT->GetVariable(
			L"LGF_ID",
			&gQcomTokenSpaceGuid,
			NULL,
			&t_ID_SIZE,
			&ID_VALUE);

	if (Status != EFI_SUCCESS)
		DEBUG((EFI_D_ERROR, "[ABL] (ERROR) Get_LGF_ID failed : %r \n", Status));

	return ID_VALUE;
}

UINT32 Get_SKU_ID()
{
	EFI_STATUS Status;
	UINT32 ID_VALUE = 0;
	UINTN t_ID_SIZE = sizeof(ID_VALUE);

	Status = gRT->GetVariable(
			L"SKU_ID",
			&gQcomTokenSpaceGuid,
			NULL,
			&t_ID_SIZE,
			&ID_VALUE);

	if (Status != EFI_SUCCESS)
		DEBUG((EFI_D_ERROR, "[ABL] (ERROR) Get_SKU_ID failed : %r \n", Status));

	return ID_VALUE;
}

UINT32 Get_DDR_ID()
{
	EFI_STATUS Status;
	UINT32 ID_VALUE = 0;
	UINTN t_ID_SIZE = sizeof(ID_VALUE);

	Status = gRT->GetVariable(
			L"DDR_ID",
			&gQcomTokenSpaceGuid,
			NULL,
			&t_ID_SIZE,
			&ID_VALUE);

	if (Status != EFI_SUCCESS)
		DEBUG((EFI_D_ERROR, "[ABL] (ERROR) Get_DDR_ID failed : %r \n", Status));

	return ID_VALUE;
}

UINT32 Get_NFC_ID()
{
	EFI_STATUS Status;
	UINT32 ID_VALUE = 0;
	UINTN t_ID_SIZE = sizeof(ID_VALUE);

	Status = gRT->GetVariable(
			L"NFC_ID",
			&gQcomTokenSpaceGuid,
			NULL,
			&t_ID_SIZE,
			&ID_VALUE);

	if (Status != EFI_SUCCESS)
		DEBUG((EFI_D_ERROR, "[ABL] (ERROR) Get_NFC_ID failed : %r \n", Status));

	return ID_VALUE;
}

UINT32 Get_RF_ID()
{
	EFI_STATUS Status;
	UINT32 ID_VALUE = 0;
	UINTN t_ID_SIZE = sizeof(ID_VALUE);

	Status = gRT->GetVariable(
			L"RF_ID",
			&gQcomTokenSpaceGuid,
			NULL,
			&t_ID_SIZE,
			&ID_VALUE);

	if (Status != EFI_SUCCESS)
		DEBUG((EFI_D_ERROR, "[ABL] (ERROR) Get_RF_ID failed : %r \n", Status));

	return ID_VALUE;
}

UINT32 Get_FP_ID()
{
	EFI_STATUS Status;
	UINT32 ID_VALUE = 0;
	UINTN t_ID_SIZE = sizeof(ID_VALUE);

	Status = gRT->GetVariable(
			L"FP_ID",
			&gQcomTokenSpaceGuid,
			NULL,
			&t_ID_SIZE,
			&ID_VALUE);

	if (Status != EFI_SUCCESS)
		DEBUG((EFI_D_ERROR, "[ABL] (ERROR) Get_FP_ID failed : %r \n", Status));

	return ID_VALUE;
}

UINT32 Get_FEATURE_ID()
{
	EFI_STATUS Status;
	UINT32 ID_VALUE = 0;
	UINTN t_ID_SIZE = sizeof(ID_VALUE);

	Status = gRT->GetVariable(
			L"FEATURE_ID",
			&gQcomTokenSpaceGuid,
			NULL,
			&t_ID_SIZE,
			&ID_VALUE);

	if (Status != EFI_SUCCESS)
		DEBUG((EFI_D_ERROR, "[ABL] (ERROR) Get_FEATURE_ID failed : %r \n", Status));

	return ID_VALUE;
}

UINT32 Get_JTAG_ID()
{
	EFI_STATUS Status;
	UINT32 ID_VALUE = 0;
	UINTN t_ID_SIZE = sizeof(ID_VALUE);

	Status = gRT->GetVariable(
			L"JTAG_ID",
			&gQcomTokenSpaceGuid,
			NULL,
			&t_ID_SIZE,
			&ID_VALUE);

	if (Status != EFI_SUCCESS)
		DEBUG((EFI_D_ERROR, "[ABL] (ERROR) Get_JTAG_ID failed : %r \n", Status));

	return ID_VALUE;
}

UINT64 Get_CPU_ID()
{
	EFI_STATUS Status;
	UINT64 ID_VALUE = 0;
	UINTN t_ID_SIZE = sizeof(ID_VALUE);

	Status = gRT->GetVariable(
			L"CPU_ID",
			&gQcomTokenSpaceGuid,
			NULL,
			&t_ID_SIZE,
			&ID_VALUE);

	if (Status != EFI_SUCCESS)
		DEBUG((EFI_D_ERROR, "[ABL] (ERROR) Get_CPU_ID failed : %r \n", Status));

	return ID_VALUE;
}

UINT32 Get_LGF_CON_ID()
{
	EFI_STATUS Status;
	UINT32 ID_VALUE = 0;
	UINTN t_ID_SIZE = sizeof(ID_VALUE);

	Status = gRT->GetVariable(
			L"LGF_CON_ID",
			&gQcomTokenSpaceGuid,
			NULL,
			&t_ID_SIZE,
			&ID_VALUE);

	if (Status != EFI_SUCCESS)
		DEBUG((EFI_D_ERROR, "[ABL] (ERROR) Get_LGF_CON_ID failed : %r \n", Status));

	return ID_VALUE;
}

UINT32 Get_FC_ID()
{
	EFI_STATUS Status;
	UINT32 ID_VALUE = 0;
	UINTN t_ID_SIZE = sizeof(ID_VALUE);

	Status = gRT->GetVariable(
			L"FC_ID",
			&gQcomTokenSpaceGuid,
			NULL,
			&t_ID_SIZE,
			&ID_VALUE);

	if (Status != EFI_SUCCESS)
		DEBUG((EFI_D_ERROR, "[ABL] (ERROR) Get_FC_ID failed : %r \n", Status));

	return ID_VALUE;
}

UINT32 Get_UPPER_ID()
{
	EFI_STATUS Status;
	UINT32 ID_VALUE = 0;
	UINTN t_ID_SIZE = sizeof(ID_VALUE);

	Status = gRT->GetVariable(
			L"UPPER_ID",
			&gQcomTokenSpaceGuid,
			NULL,
			&t_ID_SIZE,
			&ID_VALUE);

	if (Status != EFI_SUCCESS)
		DEBUG((EFI_D_ERROR, "[ABL] (ERROR) Get_UPPER_ID failed : %r \n", Status));

	return ID_VALUE;
}

UINT32 Get_SUB_ID()
{
	EFI_STATUS Status;
	UINT32 ID_VALUE = 0;
	UINTN t_ID_SIZE = sizeof(ID_VALUE);

	Status = gRT->GetVariable(
			L"SUB_ID",
			&gQcomTokenSpaceGuid,
			NULL,
			&t_ID_SIZE,
			&ID_VALUE);

	if (Status != EFI_SUCCESS)
		DEBUG((EFI_D_ERROR, "[ABL] (ERROR) Get_SUB_ID failed : %r \n", Status));

	return ID_VALUE;
}


void Get_PCBID(void)
{
	UINT32 PCBID        = 0;
	UINT32 PJ_ID        = 0;
	UINT32 HW_ID        = 0;
	UINT32 LGF_ID       = 0;
	UINT32 SKU_ID       = 0;
	UINT32 DDR_ID       = 0;
	UINT32 NFC_ID       = 0;
	UINT32 RF_ID        = 0;
	UINT32 FP_ID        = 0;
	UINT32 FEATURE_ID   = 0;
	UINT32 JTAG_ID      = 0;
	UINT32 LGF_CON_ID   = 0;
	UINT32 FC_ID        = 0;
	UINT32 UPPER_ID     = 0;
	UINT32 SUB_ID       = 0;
	unsigned long FACTORY_CRC   = 0;


	PJ_ID        = Get_PJ_ID();
	HW_ID        = Get_HW_ID();
	LGF_ID       = Get_LGF_ID();
	SKU_ID       = Get_SKU_ID();
	DDR_ID       = Get_DDR_ID();
	NFC_ID       = Get_NFC_ID();
	RF_ID        = Get_RF_ID();
	FP_ID        = Get_FP_ID();
	FEATURE_ID   = Get_FEATURE_ID();
	JTAG_ID      = Get_JTAG_ID();
	
	LGF_CON_ID   = Get_LGF_CON_ID();
	FC_ID        = Get_FC_ID();
	UPPER_ID     = Get_UPPER_ID();
	SUB_ID       = Get_SUB_ID();

	FACTORY_CRC          = GetFactoryCRC();

	PCBID        = (PJ_ID << 12)
						 | (HW_ID << 8)
						 | (SKU_ID << 4)
						 | (RF_ID << 0);

	DEBUG((EFI_D_ERROR, "[ABL] PCBID = %x\n", PCBID));

	AsciiSPrint(cmd_prj_id, sizeof(cmd_prj_id), " androidboot.id.prj=%d", PJ_ID);
	AsciiSPrint(cmd_stage_id, sizeof(cmd_stage_id), " androidboot.id.stage=%d", HW_ID);
	AsciiSPrint(cmd_lgf_id, sizeof(cmd_lgf_id), " androidboot.id.lgf=%d", LGF_ID);
	AsciiSPrint(cmd_sku_id, sizeof(cmd_sku_id), " androidboot.id.sku=%d", SKU_ID);
	AsciiSPrint(cmd_ddr_id, sizeof(cmd_ddr_id), " androidboot.id.ddr=%d", DDR_ID);
	AsciiSPrint(cmd_nfc_id, sizeof(cmd_nfc_id), " androidboot.id.nfc=%d", NFC_ID);
	AsciiSPrint(cmd_rf_id, sizeof(cmd_rf_id), " androidboot.id.rf=%d", RF_ID);
	AsciiSPrint(cmd_fp_id, sizeof(cmd_fp_id), " androidboot.id.fp=%d", FP_ID);
	AsciiSPrint(cmd_feature_id, sizeof(cmd_feature_id), " androidboot.id.feature=%d", FEATURE_ID);
	AsciiSPrint(cmd_jtag_id, sizeof(cmd_jtag_id), " androidboot.id.jtag=0x%x", JTAG_ID);
	
	AsciiSPrint(cmd_lgf_con_id, sizeof(cmd_lgf_con_id), " androidboot.id.bc=%d", LGF_CON_ID);
	AsciiSPrint(cmd_fc_id, sizeof(cmd_fc_id), " androidboot.id.fc=%d", FC_ID);
	AsciiSPrint(cmd_upper_id, sizeof(cmd_upper_id), " androidboot.id.upper=%d", UPPER_ID);
	AsciiSPrint(cmd_sub_id, sizeof(cmd_sub_id), " androidboot.id.sub=%d", SUB_ID);

	AsciiSPrint(cmd_pcb_id, sizeof(cmd_pcb_id), " androidboot.id.pcb=%x", PCBID);
	AsciiSPrint(cmd_factory_crc, sizeof(cmd_factory_crc), " androidboot.factory.crc=0x%x", FACTORY_CRC);
}

char* asus_strtok ( char * str, const char * delimiters ){
	if( delimiters == NULL) return NULL;
	static char * s_mem = NULL;
	if( str == NULL && s_mem == NULL) return NULL;
	
	char *s;
	if(str != NULL) s=str;
	else s=s_mem;
	char const * delim;
	
	int stat=1;
	while(stat){
		delim = delimiters;
		while( *delim && *s != *delim){
			delim++;
		}
		
		if(*delim){
			s++;
		}
		else stat=0; 
	}
	s_mem = s;
	
	while( *s){
		delim = delimiters;
		while( *delim && *s != *delim){
			delim++;
		}

		if(*delim){
			*s = '\0';
			char *t = s_mem;
			s_mem = s+1;
			return t;
		}
		s++;
	}

	char *t = s_mem;
	s_mem = NULL; 
	return t;
}

EFI_STATUS Get_XBL_INFO(void)
{
	EFI_STATUS Status = EFI_SUCCESS;
	char xbl_ver_info[128] = {0};
	char xbl_codebase[64] = {0};
	char xbl_info[64] = {0};
	char* xbl_info_temp;
	UINT8 xbl_ver = 0;
	UINTN xbl_codebase_size = sizeof(xbl_codebase);
	UINTN xbl_info_size = sizeof(xbl_info);
	UINTN xbl_ver_size = sizeof(xbl_ver);

	Status = gRT->GetVariable(
			L"XBL_CODEBASE",
			&gQcomTokenSpaceGuid,
			NULL,
			&xbl_codebase_size,
			&xbl_codebase);

	if (Status != EFI_SUCCESS)
		DEBUG((EFI_D_ERROR, "[ABL] (ERROR) Get XBL_CODEBASE failed : %r \n", Status));

	Status = gRT->GetVariable(
			L"XBL_INFO",
			&gQcomTokenSpaceGuid,
			NULL,
			&xbl_info_size,
			&xbl_info);

	if (Status != EFI_SUCCESS)
		DEBUG((EFI_D_ERROR, "[ABL] (ERROR) Get XBL_INFO failed : %r \n", Status));

	Status = gRT->GetVariable(
			L"XBL_VER",
			&gQcomTokenSpaceGuid,
			NULL,
			&xbl_ver_size,
			&xbl_ver);

	if (Status != EFI_SUCCESS)
		DEBUG((EFI_D_ERROR, "[ABL] (ERROR) Get XBL_VER failed : %r \n", Status));

	xbl_info_temp = asus_strtok(xbl_codebase, " ");
	//xbl_info_temp = xbl_codebase;
	AsciiSPrint(xbl_ver_info, sizeof(xbl_ver_info), "%a-%d-%a", xbl_info_temp, xbl_ver, xbl_info);
	SetXBLVersion(xbl_ver_info, sizeof(xbl_ver_info));

	DEBUG((EFI_D_ERROR, "\n" ));
	DEBUG((EFI_D_ERROR, "======================================================= \n" ));
	DEBUG((EFI_D_ERROR, "  XBL CODEBASE : %a\n", xbl_codebase));
	DEBUG((EFI_D_ERROR, "  XBL INFO : %a\n", xbl_info));
	DEBUG((EFI_D_ERROR, "  XBL VERSION : %d\n", xbl_ver));
	DEBUG((EFI_D_ERROR, "======================================================= \n" ));
	DEBUG((EFI_D_ERROR, "\n" ));

	return Status;
}
////////////////////////////////////////////////////////////////////////
// --- ROG7 : PROJECT FUNCTION
////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////
// +++ ROG7 : PROJECT COMMON FUNCTION
////////////////////////////////////////////////////////////////////////
// +++ ASUS_BSP : add for asus unlock state
int ASUS_Get_UNLOCK_STATE()
{
	if (IsAuthorized())
	{
		AsciiSPrint(cmd_enable_adb_mode, sizeof(cmd_enable_adb_mode), " ADB=Y");
		AsciiSPrint(cmd_enable_adb_prop, sizeof(cmd_enable_adb_prop), " androidboot.adb.enable=1");
		AsciiSPrint(cmd_authorized_prop, sizeof(cmd_authorized_prop), " androidboot.asus.authorized=1");
		AsciiSPrint(cmd_unlock, sizeof(cmd_unlock), " UNLOCKED=Y");
	}
	else if(IsAuthorized_2())
	{
		AsciiSPrint(cmd_enable_adb_mode, sizeof(cmd_enable_adb_mode), " ADB=Y");
		AsciiSPrint(cmd_enable_adb_prop, sizeof(cmd_enable_adb_prop), " androidboot.adb.enable=1");
		AsciiSPrint(cmd_authorized_prop, sizeof(cmd_authorized_prop), " androidboot.asus.authorized=1");
		AsciiSPrint(cmd_unlock, sizeof(cmd_unlock), " UNLOCKED=Y");
	}
	else
	{
		AsciiSPrint(cmd_authorized_prop, sizeof(cmd_authorized_prop), " androidboot.asus.authorized=0");
		AsciiSPrint(cmd_unlock, sizeof(cmd_unlock), " ");
	}

	// +++ ASUS_BSP : add for WaterMask unlock
	if (IsAuthorized_3())
	{
		DEBUG((EFI_D_ERROR, "[ABL] WaterMask unlock : set VIP=Y\n" ));
		AsciiSPrint(cmd_watermask_unlock, sizeof(cmd_watermask_unlock), " VIP=Y");
		
		AsciiSPrint(cmd_watermask_unlock_prop, sizeof(cmd_watermask_unlock_prop), " androidboot.asus.warranty=0");
	}
	// --- ASUS_BSP : add for WaterMask unlock

	DEBUG((EFI_D_ERROR, "\n" ));
	DEBUG((EFI_D_ERROR, "============================ \n" ));
	DEBUG((EFI_D_ERROR, "ASUS_Get_Authorized_STATE = %a \n", cmd_authorized_prop));
	DEBUG((EFI_D_ERROR, "ASUS_Get_UNLOCK_STATE = %a \n", cmd_unlock));
	DEBUG((EFI_D_ERROR, "============================ \n" ));
	DEBUG((EFI_D_ERROR, "\n" ));

	return 0;
}
// --- ASUS_BSP : add for asus unlock state

// +++ ASUS_BSP : add for get cpuid hash
/* Convert character to lowercase */
static int tolower (int c)
{
	if (('A' <= (c)) && ((c) <= 'Z'))
	{
		return (c - ('A' - 'a'));
	}
	return (c);
}

void Get_CPU_ID_HASH()
{
	EFI_STATUS Status = EFI_SUCCESS;
	UINT64 UUID = 0;
	char cpuid_hash[34]={};
	char hash_buf[64];
	MD5_CTX Md5Ctx;
	UINT8 md5sum[16];
	int i=0;
	char output[33];

	DEBUG((EFI_D_ERROR, "[ABL] +++ Get_CPU_ID_HASH()\n"));

	UUID = Get_CPU_ID();

	if (Status != EFI_SUCCESS)
	{
		DEBUG((EFI_D_ERROR, "[ABL] Get_CPU_ID_HASH : Failed to GetSerialNum() \n"));
		return;
	}
	else
	{
		DEBUG((EFI_D_ERROR, "[ABL] Get_CPU_ID_HASH : UUID : 0x%lx\n", UUID));
		AsciiSPrint(hash_buf, sizeof(hash_buf), "%lx", UUID);

		DEBUG((EFI_D_ERROR, "[ABL] Get_CPU_ID_HASH : hash_buf : %a\n", hash_buf));

		Status = MD5Init (&Md5Ctx);
		if (Status != EFI_SUCCESS)
		{
			DEBUG((EFI_D_ERROR, "[ABL] Get_CPU_ID_HASH : Failed to MD5Init() \n"));
			return;
		}

		MD5Update(&Md5Ctx, (char *) hash_buf, (UINTN)strlen(hash_buf));
		MD5Final(&Md5Ctx, md5sum);

		memset(output, 0, sizeof(output));
		AsciiSPrint(output, sizeof(output), "%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x",
					md5sum[0],md5sum[1],md5sum[2],md5sum[3],md5sum[4],md5sum[5],md5sum[6],md5sum[7],md5sum[8],
					md5sum[9],md5sum[10],md5sum[11],md5sum[12],md5sum[13],md5sum[14],md5sum[15]);

		DEBUG((EFI_D_ERROR, "[ABL] Get_CPU_ID_HASH : output=%a\n", output));

		memset(cpuid_hash, 0, 32);
		memcpy(cpuid_hash, output, 32);
		cpuid_hash[32]='\0';

		for( i = 0; i < 33; i++ )
		{
			cpuid_hash[i] = (char)tolower((int)cpuid_hash[i]);
		}

		DEBUG((EFI_D_ERROR, "[ABL] Get_CPU_ID_HASH : rand_num=%a\n", cpuid_hash));

	}

	Status = SetCpuidHash(cpuid_hash, sizeof(cpuid_hash));
	AsciiSPrint(cmd_cpuid_hash, sizeof(cmd_cpuid_hash), " androidboot.cpuid.hash=%a", cpuid_hash);

	DEBUG((EFI_D_ERROR, "[ABL] --- Get_CPU_ID_HASH()\n"));
	return;
}
// --- ASUS_BSP : add for get cpuid hash

/***** ASUS_GET_DDR_INFO *****/
/*
UINT8 Get_DDR_Manufacturer_ID(void)
{
	EFI_STATUS Status = EFI_SUCCESS;
	struct ddr_details_entry_info *DdrInfo;
	//UINT64 Revision;
	UINT32 DdrManufacturerId = 0;

	DdrInfo = AllocateZeroPool (sizeof (struct ddr_details_entry_info));
	if (DdrInfo == NULL) {
		DEBUG ((EFI_D_ERROR, "[ABL] DDR Info Buffer: Out of resources\n"));
	}

	Status = GetDDRInfo (DdrInfo);
	if (Status == EFI_SUCCESS) {
		DdrManufacturerId = DdrInfo->manufacturer_id;
	}

	AsciiSPrint(cmd_ddr_manufacturer, sizeof(cmd_ddr_manufacturer), " androidboot.ddr.manufacturer_id=%x", DdrManufacturerId);

	DEBUG((EFI_D_ERROR, "\n" ));
	DEBUG((EFI_D_ERROR, "======================================================= \n" ));
	DEBUG((EFI_D_ERROR, "  DDR Manufacturer ID: %x\n", DdrManufacturerId));

	return DdrManufacturerId;
}

UINT8 Get_DDR_Device_Type(void)
{
	EFI_STATUS Status = EFI_SUCCESS;
	struct ddr_details_entry_info *DdrInfo;
	//UINT64 Revision;
	UINT32 DdrDeviceType = 0;

	DdrInfo = AllocateZeroPool (sizeof (struct ddr_details_entry_info));
	if (DdrInfo == NULL) {
		DEBUG ((EFI_D_ERROR, "[ABL] DDR Info Buffer: Out of resources\n"));
	}

	Status = GetDDRInfo (DdrInfo);
	if (Status == EFI_SUCCESS) {
		DdrDeviceType = DdrInfo->device_type;
	}
	AsciiSPrint(cmd_ddr_device_type, sizeof(cmd_ddr_device_type), " androidboot.ddr.device_type=%x", DdrDeviceType);

	DEBUG((EFI_D_ERROR, "  DDR Device Type : %x\n", DdrDeviceType));
	DEBUG((EFI_D_ERROR, "======================================================= \n" ));
	DEBUG((EFI_D_ERROR, "\n" ));

	return DdrDeviceType;
}*/

/***** ASUS_GET_PROJECT_ID *****/
UINT32 Get_PJ_ID()
{
	EFI_STATUS Status;
	UINT32 ID_VALUE = 0;
	UINTN t_ID_SIZE = sizeof(ID_VALUE);

	Status = gRT->GetVariable(
			L"PJ_ID",
			&gQcomTokenSpaceGuid,
			NULL,
			&t_ID_SIZE,
			&ID_VALUE);

	if (Status != EFI_SUCCESS)
		DEBUG((EFI_D_ERROR, "[ABL] (ERROR) Get_PJ_ID failed : %r \n", Status));

	return ID_VALUE;
}

UINT32 Get_DT_ID()
{
	UINT32 DT_ID = 0;

	switch (Get_PJ_ID())
	{
		case PJ_ID_ENTRY:
		case PJ_ID_PRO:
		case PJ_ID_ULTIMATE:
			switch (Get_HW_ID())
			{
				case HW_ID_EVB:
					DT_ID = DT_ID_EVB;
				break;

				case HW_ID_SR1:
					DT_ID = DT_ID_SR1;
				break;
				
				case HW_ID_SR2:
					DT_ID = DT_ID_SR2;
				break;
				
				case HW_ID_ER1:
					DT_ID = DT_ID_ER1;
				break;
				
				case HW_ID_ER2:
					DT_ID = DT_ID_ER2;
				break;
				
				case HW_ID_PR:
					DT_ID = DT_ID_PR;
				break;
				
				case HW_ID_MP:
					DT_ID = DT_ID_MP;
				break;
				
				default:
					DT_ID = DT_ID_EVB;
					DEBUG((EFI_D_ERROR, "[ABL] (ERROR) Get_DT_ID: Unknown HW_ID(%d), set DT_ID=%d\n", Get_HW_ID(), DT_ID));
				break;
			}
		break;
		default:
			DT_ID = DT_ID_EVB;
			DEBUG((EFI_D_ERROR, "[ABL] (ERROR) Get_DT_ID: Unknown PJ_ID(%d), set DT_ID=%d\n", Get_PJ_ID(), DT_ID));
		break;
	}

	SetDeviceTreeID(DT_ID);

	return DT_ID;
}

EFI_STATUS Set_HWStage_Info(void)
{
	EFI_STATUS Status = EFI_SUCCESS;

	char project_str[64]= {0};
	char stage_str[64]= {0};

	switch (Get_PJ_ID())
	{
		case PJ_ID_ENTRY:
		case PJ_ID_PRO:
		case PJ_ID_ULTIMATE:
			AsciiSPrint(project_str, sizeof(project_str), "AI2205");
			switch (Get_HW_ID())
			{
				case HW_ID_EVB:
					AsciiSPrint(stage_str, sizeof(stage_str), "EVB");
				break;
				
				case HW_ID_SR1:
					AsciiSPrint(stage_str, sizeof(stage_str), "SR1");
				break;
				
				case HW_ID_SR2:
					AsciiSPrint(stage_str, sizeof(stage_str), "SR2");
				break;
				
				case HW_ID_ER1:
					AsciiSPrint(stage_str, sizeof(stage_str), "ER1");
				break;
				
				case HW_ID_ER2:
					AsciiSPrint(stage_str, sizeof(stage_str), "ER2");
				break;
				
				case HW_ID_PR:
					AsciiSPrint(stage_str, sizeof(stage_str), "PR");
				break;
				
				case HW_ID_MP:
					AsciiSPrint(stage_str, sizeof(stage_str), "MP");
				break;

				default:
					DEBUG((EFI_D_ERROR, "[ABL] (ERROR) Set_HWStage_Info : Unknown HW ID(%d), set to default HW ID\n", Get_HW_ID()));
					AsciiSPrint(stage_str, sizeof(stage_str), "EVB");
					break;
			}
			

			switch(Get_SKU_ID())
			{
				case SKU_ID_0:
					AsciiStrnCatS(stage_str, 64 ,"-SKU1", AsciiStrLen("-SKU1"));
				break;

				case SKU_ID_1:
					AsciiStrnCatS(stage_str, 64 ,"-SKU2", AsciiStrLen("-SKU2"));
				break;

				case SKU_ID_2:
					AsciiStrnCatS(stage_str, 64 ,"-SKU3", AsciiStrLen("-SKU3"));
				break;

				case SKU_ID_3:
					AsciiStrnCatS(stage_str, 64 ,"-SKU4", AsciiStrLen("-SKU4"));
				break;

				case SKU_ID_4:
					AsciiStrnCatS(stage_str, 64 ,"-SKU5", AsciiStrLen("-SKU5"));
				break;

				case SKU_ID_5:
					AsciiStrnCatS(stage_str, 64 ,"-SKU6", AsciiStrLen("-SKU6"));
				break;

				case SKU_ID_6:
					AsciiStrnCatS(stage_str, 64 ,"-SKU7", AsciiStrLen("-SKU7"));
				break;

				case SKU_ID_7:
					AsciiStrnCatS(stage_str, 64 ,"-SKU8", AsciiStrLen("-SKU8"));
				break;

				default:
					DEBUG((EFI_D_ERROR, "[ABL] (ERROR) Set_HWStage_Info : Unknown SKU ID(%d)\n", Get_SKU_ID()));
					AsciiStrnCatS(stage_str, 64 ,"-UNKNOWN", AsciiStrLen("-UNKNOWN"));
				break;
			}

			switch(Get_RF_ID())
			{
				case RF_ID_0:
					AsciiStrnCatS(stage_str, 64 ,"-RFID:0", AsciiStrLen("-RFID:0"));
				break;

				case RF_ID_1:
					AsciiStrnCatS(stage_str, 64 ,"-RFID:1", AsciiStrLen("-RFID:1"));
				break;

				case RF_ID_2:
					AsciiStrnCatS(stage_str, 64 ,"-RFID:2", AsciiStrLen("-RFID:2"));
				break;

				case RF_ID_3:
					AsciiStrnCatS(stage_str, 64 ,"-RFID:3", AsciiStrLen("-RFID:3"));
				break;

				default:
					DEBUG((EFI_D_ERROR, "[ABL] (ERROR) Set_HWStage_Info : Unknown RF ID(%d)\n", Get_RF_ID()));
					AsciiStrnCatS(stage_str, 64 ,"-UNKNOWN", AsciiStrLen("-UNKNOWN"));
				break;
			}
		break;
		default:
			DEBUG((EFI_D_ERROR, "[ABL] (ERROR) Set_HWStage_Info : Unknown PJ ID(%d), set to default Project ID\n", Get_PJ_ID()));
			AsciiSPrint(project_str, sizeof(project_str), "AI2201");
			DEBUG((EFI_D_ERROR, "[ABL] (ERROR) Set_HWStage_Info : Unknown HW ID(%d), set to default Stage ID\n", Get_HW_ID()));
			AsciiSPrint(stage_str, sizeof(stage_str), "EVB-SKU1-RF:0");
		break;
	}

	AsciiSPrint(asus_project_info, sizeof(asus_project_info), project_str);
	DEBUG((EFI_D_ERROR, "[ABL] asus_project_info = %a\n", asus_project_info));

	AsciiSPrint(asus_stage_info, sizeof(asus_stage_info), stage_str);
	DEBUG((EFI_D_ERROR, "[ABL] asus_stage_info = %a\n", asus_stage_info));

	memcpy(DevInfo.project_name, asus_project_info, sizeof(asus_project_info));
	//DEBUG((EFI_D_ERROR, "[ABL] DevInfo.project_name = %a\n", DevInfo.project_name));

	memcpy(DevInfo.hw_stage, asus_stage_info, sizeof(asus_stage_info));
	//DEBUG((EFI_D_ERROR, "[ABL] DevInfo.hw_stage = %a\n", DevInfo.hw_stage));

	Status = ReadWriteDeviceInfo(WRITE_CONFIG, (UINT8 *)&DevInfo, sizeof(DevInfo));
	if (Status != EFI_SUCCESS)
	{
		DEBUG((EFI_D_ERROR, "[ABL] (ERROR) Set_HWStage_Info : Unable to Write Device Info : project_name & hw_stage (Status=%r)\n", Status));
		return Status;
	}
	return Status;
}

// +++ ASUS_BSP : add for check sysconf partition
BOOLEAN check_sysconf(VOID)
{
	EFI_STATUS                   Status = EFI_SUCCESS;
	EFI_BLOCK_IO_PROTOCOL       *BlkIo;
	PartiSelectFilter            HandleFilter;
	HandleInfo                   HandleInfoList[1];
	STATIC UINT32                MaxHandles;
	STATIC UINT32                BlkIOAttrib = 0;
	VOID                        *Buffer;
	UINT32                       BufferSize = ALIGNMENT_MASK_4KB;
	BOOLEAN                      ResetDevInfo = FALSE;

	//sysconf
	EFI_GUID PartitionType = { 0xA03D4EB3, 0x7079, 0x4E02, { 0xAA, 0x98, 0x9F, 0x14, 0x6C, 0x9A, 0x43, 0x4D } };

	DEBUG((EFI_D_ERROR, "[ABL] +++ check sysconf partition() \n"));

	BlkIOAttrib = BLK_IO_SEL_PARTITIONED_MBR;
	BlkIOAttrib |= BLK_IO_SEL_PARTITIONED_GPT;
	BlkIOAttrib |= BLK_IO_SEL_MEDIA_TYPE_NON_REMOVABLE;
	BlkIOAttrib |= BLK_IO_SEL_MATCH_PARTITION_TYPE_GUID;

	HandleFilter.RootDeviceType = NULL;
	HandleFilter.PartitionType = &PartitionType;
	HandleFilter.VolumeName = NULL;

	MaxHandles = sizeof(HandleInfoList)/sizeof(*HandleInfoList);

	Status = GetBlkIOHandles (BlkIOAttrib, &HandleFilter, HandleInfoList, &MaxHandles);

	if(Status == EFI_SUCCESS) {
		if (MaxHandles == 0) {
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): no partitions found.\r\n"));
			//return EFI_NO_MEDIA;
		}else if (MaxHandles != 1) {
			// Unable to deterministically load from single partition
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): multiple partitions found.\r\n"));
			//return EFI_LOAD_ERROR;
		}
	}else {
		DEBUG ((EFI_D_ERROR, "[ABL] %s: GetBlkIOHandles failed: %r\n", __func__, Status));
		//return Status;
	}

	BlkIo = HandleInfoList[0].BlkIo;
	Buffer = AllocatePages(ALIGN_PAGES(BufferSize, ALIGNMENT_MASK_4KB));

	if (Buffer == NULL) {
		DEBUG ((EFI_D_ERROR, "[ABL] (ERROR) check sysconf partition - allocate buffer fail \n\n"));
		//return EFI_DEVICE_ERROR;
	}

	if (Buffer != NULL) {
		BlkIo->ReadBlocks (BlkIo,
						BlkIo->Media->MediaId,
						0,
						BufferSize,//ROUND_TO_PAGE(ImageSize, BlkIo->Media->BlockSize - 1),
						Buffer);
	}
	DEBUG ((EFI_D_INFO, "[ABL] check sysconf partition - Buffer = %a\n", Buffer));

	if(!AsciiStriCmp (Buffer, "")){
		DEBUG((EFI_D_ERROR, "[ABL] check sysconf partition : ResetDevInfo = TRUE \n"));
		ResetDevInfo = TRUE;
	}else{
		DEBUG((EFI_D_ERROR, "[ABL] check sysconf partition : ResetDevInfo = FALSE \n"));
		ResetDevInfo = FALSE;
	}

	DEBUG((EFI_D_ERROR, "[ABL] --- check sysconf partition() \n\n"));

	return ResetDevInfo ;
}
// --- ASUS_BSP : add for check sysconf partition

// +++ ASUS_BSP : add for record already reset DevInfo
EFI_STATUS RecordResetDevInfo(VOID)
{
	EFI_STATUS                   Status = EFI_SUCCESS;
	EFI_BLOCK_IO_PROTOCOL       *BlkIo;
	PartiSelectFilter            HandleFilter;
	HandleInfo                   HandleInfoList[1];
	STATIC UINT32                MaxHandles;
	STATIC UINT32                BlkIOAttrib = 0;
	VOID                        *Buffer;
	UINT32                       BufferSize = ALIGNMENT_MASK_4KB;

	//sysconf
	EFI_GUID PartitionType = { 0xA03D4EB3, 0x7079, 0x4E02, { 0xAA, 0x98, 0x9F, 0x14, 0x6C, 0x9A, 0x43, 0x4D } };

	DEBUG((EFI_D_ERROR, "[ABL] +++ RecordResetDevInfo() \n"));

	BlkIOAttrib = BLK_IO_SEL_PARTITIONED_MBR;
	BlkIOAttrib |= BLK_IO_SEL_PARTITIONED_GPT;
	BlkIOAttrib |= BLK_IO_SEL_MEDIA_TYPE_NON_REMOVABLE;
	BlkIOAttrib |= BLK_IO_SEL_MATCH_PARTITION_TYPE_GUID;

	HandleFilter.RootDeviceType = NULL;
	HandleFilter.PartitionType = &PartitionType;
	HandleFilter.VolumeName = NULL;

	MaxHandles = sizeof(HandleInfoList)/sizeof(*HandleInfoList);

	Status = GetBlkIOHandles (BlkIOAttrib, &HandleFilter, HandleInfoList, &MaxHandles);
	//DEBUG ((EFI_D_ERROR, "[ABL] read_partition - MaxHandles = %d\n", MaxHandles));

	if(Status == EFI_SUCCESS) {
		if (MaxHandles == 0) {
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): no partitions found.\r\n"));
			return EFI_NO_MEDIA;
		}else if (MaxHandles != 1) {
			// Unable to deterministically load from single partition
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): multiple partitions found.\r\n"));
			return EFI_LOAD_ERROR;
		}
	}else {
		DEBUG ((EFI_D_ERROR, "[ABL] %s: GetBlkIOHandles failed: %r\n", __func__, Status));
		return Status;
	}

	BlkIo = HandleInfoList[0].BlkIo;
	Buffer = AllocatePages(ALIGN_PAGES(BufferSize, ALIGNMENT_MASK_4KB));
	//DEBUG ((EFI_D_ERROR, "ALIGN_PAGES Image size: %u Bytes\n", BufferSize));

	if (Buffer == NULL) {
		DEBUG ((EFI_D_ERROR, "[ABL] (ERROR)  RecordResetDevInfo - allocate buffer fail \n\n"));
		return EFI_DEVICE_ERROR;
	}

	if (Buffer != NULL) {
		BlkIo->ReadBlocks (BlkIo,
						BlkIo->Media->MediaId,
						0,
						BufferSize,//ROUND_TO_PAGE(ImageSize, BlkIo->Media->BlockSize - 1),
						Buffer);
	}


	char *temp = Buffer;
	AsciiStrnCpyS (temp, ASUS_MAGIC_SIZE, ASUS_MAGIC, ASUS_MAGIC_SIZE);

	Status = BlkIo->WriteBlocks(BlkIo,
							BlkIo->Media->MediaId,
							0,
							BufferSize,
							Buffer);

	if(Status != EFI_SUCCESS) {
		DEBUG ((EFI_D_ERROR, "[ABL] (ERROR) RecordResetDevInfo - RecordResetDevInfo flags fail \n\n"));
		DEBUG ((EFI_D_ERROR, "[ABL] (ERROR) Status= %d \n\n", Status));
		return EFI_DEVICE_ERROR;
	}

	DEBUG((EFI_D_ERROR, "[ABL] --- RecordResetDevInfo() \n\n"));

	return Status;
}
// --- ASUS_BSP : add for record already reset DevInfo

// +++ ASUS_BSP : add for system info
sys_info sysinfo;
EFI_STATUS read_sysinfo(struct sys_info *in)
{
	EFI_STATUS                   Status = EFI_SUCCESS;
	EFI_BLOCK_IO_PROTOCOL       *BlkIo;
	PartiSelectFilter            HandleFilter;
	HandleInfo                   HandleInfoList[1];
	STATIC UINT32                MaxHandles;
	STATIC UINT32                BlkIOAttrib = 0;
	VOID                        *Buffer;
	UINT32                       BufferSize = ALIGNMENT_MASK_4KB;
	char                         temp[2*ALIGNMENT_MASK_4KB] = "";

	//sysinfo
	EFI_GUID PartitionType = { 0xc0b6b507, 0xe7d0, 0x4ee0, { 0x81, 0xe0, 0x56, 0x97, 0x38, 0xf7, 0x6b, 0x4c } };

	DEBUG((EFI_D_ERROR, "[ABL] +++ read_sysinfo() \n"));

	BlkIOAttrib = BLK_IO_SEL_PARTITIONED_MBR;
	BlkIOAttrib |= BLK_IO_SEL_PARTITIONED_GPT;
	BlkIOAttrib |= BLK_IO_SEL_MEDIA_TYPE_NON_REMOVABLE;
	BlkIOAttrib |= BLK_IO_SEL_MATCH_PARTITION_TYPE_GUID;

	HandleFilter.RootDeviceType = NULL;
	HandleFilter.PartitionType = &PartitionType;
	HandleFilter.VolumeName = NULL;

	MaxHandles = sizeof(HandleInfoList)/sizeof(*HandleInfoList);

	Status = GetBlkIOHandles (BlkIOAttrib, &HandleFilter, HandleInfoList, &MaxHandles);
	//DEBUG ((EFI_D_ERROR, "[ABL] read_partition - MaxHandles = %d\n", MaxHandles));

	if(Status == EFI_SUCCESS) {
		if (MaxHandles == 0) {
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): no partitions found.\r\n"));
			return EFI_NO_MEDIA;
		}else if (MaxHandles != 1) {
			// Unable to deterministically load from single partition
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): multiple partitions found.\r\n"));
			return EFI_LOAD_ERROR;
		}
	}else {
		DEBUG ((EFI_D_ERROR, "[ABL] %s: GetBlkIOHandles failed: %r\n", __func__, Status));
		return Status;
	}

	BlkIo = HandleInfoList[0].BlkIo;
	Buffer = AllocatePages(ALIGN_PAGES(BufferSize, ALIGNMENT_MASK_4KB));

	//DEBUG ((EFI_D_ERROR, "ALIGN_PAGES Image size: %u Bytes\n", BufferSize));

	if (Buffer != NULL)
	{
		BlkIo->ReadBlocks (BlkIo,
						BlkIo->Media->MediaId,
						0,
						BufferSize,//ROUND_TO_PAGE(ImageSize, BlkIo->Media->BlockSize - 1),
						Buffer);
	}
    
    memcpy(temp, Buffer, ALIGNMENT_MASK_4KB);

	DEBUG ((EFI_D_INFO, "[ABL] read_partition - Buffer = %a\n", temp));

	if (Buffer != NULL)
	{
		BlkIo->ReadBlocks (BlkIo,
						BlkIo->Media->MediaId,
						1,
						BufferSize,//ROUND_TO_PAGE(ImageSize, BlkIo->Media->BlockSize - 1),
						Buffer);
	}

	memcpy(temp + ALIGNMENT_MASK_4KB, Buffer, sizeof(sysinfo) - ALIGNMENT_MASK_4KB);

	if(!AsciiStrnCmp (temp, ASUS_MAGIC, ASUS_MAGIC_SIZE))
	{
		DEBUG ((EFI_D_ERROR, "[ABL] read_partition - copy sysinfo str \n\n"));
		memcpy(in, temp, sizeof(sysinfo));
	}
	else
	{
		DEBUG ((EFI_D_ERROR, "[ABL] read_partition - non-sysinfo struct \n"));
		Status = EFI_DEVICE_ERROR;
	}

	DEBUG((EFI_D_ERROR, "[ABL] --- read_sysinfo() \n"));

	return Status ;
}
// --- ASUS_BSP : add for system info

/***** ASUS_AVB_VERITY *****/
// +++ ASUS_BSP : add for read vbmeta magic
const char* read_vbmeta_magic(VOID)
{
	EFI_STATUS                   Status = EFI_SUCCESS;
	EFI_BLOCK_IO_PROTOCOL       *BlkIo;
	PartiSelectFilter            HandleFilter;
	HandleInfo                   HandleInfoList[1];
	STATIC UINT32                MaxHandles;
	STATIC UINT32                BlkIOAttrib = 0;
	VOID                        *Buffer;
	UINT32                       BufferSize = ALIGNMENT_MASK_4KB;

	//vbmeta_a
	EFI_GUID PartitionType = { 0x4b7a15d6, 0x322c, 0x42ac, { 0x81, 0x10, 0x88, 0xb7, 0xda, 0x0c, 0x5d, 0x77 } };

	DEBUG((EFI_D_ERROR, "[ABL] +++ read vbmeta magic() \n"));

	BlkIOAttrib = BLK_IO_SEL_PARTITIONED_MBR;
	BlkIOAttrib |= BLK_IO_SEL_PARTITIONED_GPT;
	BlkIOAttrib |= BLK_IO_SEL_MEDIA_TYPE_NON_REMOVABLE;
	BlkIOAttrib |= BLK_IO_SEL_MATCH_PARTITION_TYPE_GUID;

	HandleFilter.RootDeviceType = NULL;
	HandleFilter.PartitionType = &PartitionType;
	HandleFilter.VolumeName = NULL;

	MaxHandles = sizeof(HandleInfoList)/sizeof(*HandleInfoList);

	Status = GetBlkIOHandles (BlkIOAttrib, &HandleFilter, HandleInfoList, &MaxHandles);

	if(Status == EFI_SUCCESS) {
		if (MaxHandles == 0) {
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): no partitions found.\r\n"));
			//return EFI_NO_MEDIA;
		}else if (MaxHandles != 1) {
			// Unable to deterministically load from single partition
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): multiple partitions found.\r\n"));
			//return EFI_LOAD_ERROR;
		}
	}else {
		DEBUG ((EFI_D_ERROR, "[ABL] %s: GetBlkIOHandles failed: %r\n", __func__, Status));
		//return Status;
	}

	BlkIo = HandleInfoList[0].BlkIo;
	Buffer = AllocatePages(ALIGN_PAGES(BufferSize, ALIGNMENT_MASK_4KB));

	if (Buffer == NULL) {
		DEBUG ((EFI_D_ERROR, "[ABL] (ERROR)  read vbmeta magic - allocate buffer fail \n\n"));
		//return EFI_DEVICE_ERROR;
	}

	if (Buffer != NULL) {
		BlkIo->ReadBlocks (BlkIo,
						BlkIo->Media->MediaId,
						0,
						BufferSize,//ROUND_TO_PAGE(ImageSize, BlkIo->Media->BlockSize - 1),
						Buffer);
	}

	char *temp = Buffer;

	DEBUG ((EFI_D_ERROR, "[ABL] read vbmeta magic - temp = %a\n", temp));
	DEBUG ((EFI_D_ERROR, "[ABL] --- read vbmeta magic() \n"));

	return temp;
}
// --- ASUS_BSP : add for read vbmeta magic

// +++ ASUS_BSP : add for enable vbmeta magic
EFI_STATUS enable_vbmeta_magic(VOID)
{
	EFI_STATUS                   Status = EFI_SUCCESS;
	EFI_BLOCK_IO_PROTOCOL       *BlkIo;
	PartiSelectFilter            HandleFilter;
	HandleInfo                   HandleInfoList[1];
	STATIC UINT32                MaxHandles;
	STATIC UINT32                BlkIOAttrib = 0;
	VOID                        *Buffer;
	UINT32                       BufferSize = ALIGNMENT_MASK_4KB;

	//vbmeta_a
	EFI_GUID PartitionType = { 0x4b7a15d6, 0x322c, 0x42ac, { 0x81, 0x10, 0x88, 0xb7, 0xda, 0x0c, 0x5d, 0x77 } };

	DEBUG((EFI_D_ERROR, "[ABL] +++ enable vbmeta magic() \n"));

	BlkIOAttrib = BLK_IO_SEL_PARTITIONED_MBR;
	BlkIOAttrib |= BLK_IO_SEL_PARTITIONED_GPT;
	BlkIOAttrib |= BLK_IO_SEL_MEDIA_TYPE_NON_REMOVABLE;
	BlkIOAttrib |= BLK_IO_SEL_MATCH_PARTITION_TYPE_GUID;

	HandleFilter.RootDeviceType = NULL;
	HandleFilter.PartitionType = &PartitionType;
	HandleFilter.VolumeName = NULL;

	MaxHandles = sizeof(HandleInfoList)/sizeof(*HandleInfoList);

	Status = GetBlkIOHandles (BlkIOAttrib, &HandleFilter, HandleInfoList, &MaxHandles);
	//DEBUG ((EFI_D_ERROR, "[ABL] read_partition - MaxHandles = %d\n", MaxHandles));

	if(Status == EFI_SUCCESS) {
		if (MaxHandles == 0) {
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): no partitions found.\r\n"));
			return EFI_NO_MEDIA;
		}else if (MaxHandles != 1) {
			// Unable to deterministically load from single partition
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): multiple partitions found.\r\n"));
			return EFI_LOAD_ERROR;
		}
	}else {
		DEBUG ((EFI_D_ERROR, "[ABL] %s: GetBlkIOHandles failed: %r\n", __func__, Status));
		return Status;
	}

	BlkIo = HandleInfoList[0].BlkIo;
	Buffer = AllocatePages(ALIGN_PAGES(BufferSize, ALIGNMENT_MASK_4KB));
	//DEBUG ((EFI_D_ERROR, "ALIGN_PAGES Image size: %u Bytes\n", BufferSize));

	if (Buffer == NULL) {
		DEBUG ((EFI_D_ERROR, "[ABL] (ERROR)  enable vbmeta magic - allocate buffer fail \n\n"));
		return EFI_DEVICE_ERROR;
	}

	if (Buffer != NULL) {
		BlkIo->ReadBlocks (BlkIo,
						BlkIo->Media->MediaId,
						0,
						BufferSize,//ROUND_TO_PAGE(ImageSize, BlkIo->Media->BlockSize - 1),
						Buffer);
	}

	//DEBUG ((EFI_D_INFO, "[ABL] enable vbmeta magic - Buffer = %a\n", Buffer));

	char *temp = Buffer;
	AsciiStrnCpyS (temp, sizeof("AVB0"), "AVB0", sizeof("AVB0"));

	Status = BlkIo->WriteBlocks(BlkIo,
							BlkIo->Media->MediaId,
							0,
							BufferSize,
							Buffer);

	if(Status != EFI_SUCCESS) {
		DEBUG ((EFI_D_ERROR, "[ABL] (ERROR) enable vbmeta magic - enable vbmeta magic fail \n\n"));
		DEBUG ((EFI_D_ERROR, "[ABL] (ERROR) Status= %d \n\n", Status));
		return EFI_DEVICE_ERROR;
	}

	DEBUG ((EFI_D_ERROR, "[ABL] --- enable vbmeta magic() \n"));

	return Status;
}
// --- ASUS_BSP : add for enable vbmeta magic

// +++ ASUS_BSP : add for disable verity
EFI_STATUS disable_verity(VOID)
{
	EFI_STATUS                   Status = EFI_SUCCESS;
	EFI_BLOCK_IO_PROTOCOL       *BlkIo;
	PartiSelectFilter            HandleFilter;
	HandleInfo                   HandleInfoList[1];
	STATIC UINT32                MaxHandles;
	STATIC UINT32                BlkIOAttrib = 0;
	VOID                        *Buffer;
	UINT32                       BufferSize = ALIGNMENT_MASK_4KB;

	//vbmeta_a
	EFI_GUID PartitionType = { 0x4b7a15d6, 0x322c, 0x42ac, { 0x81, 0x10, 0x88, 0xb7, 0xda, 0x0c, 0x5d, 0x77 } };

	DEBUG((EFI_D_ERROR, "[ABL] +++ disable verity() \n"));

	BlkIOAttrib = BLK_IO_SEL_PARTITIONED_MBR;
	BlkIOAttrib |= BLK_IO_SEL_PARTITIONED_GPT;
	BlkIOAttrib |= BLK_IO_SEL_MEDIA_TYPE_NON_REMOVABLE;
	BlkIOAttrib |= BLK_IO_SEL_MATCH_PARTITION_TYPE_GUID;

	HandleFilter.RootDeviceType = NULL;
	HandleFilter.PartitionType = &PartitionType;
	HandleFilter.VolumeName = NULL;

	MaxHandles = sizeof(HandleInfoList)/sizeof(*HandleInfoList);

	Status = GetBlkIOHandles (BlkIOAttrib, &HandleFilter, HandleInfoList, &MaxHandles);
	//DEBUG ((EFI_D_ERROR, "[ABL] read_partition - MaxHandles = %d\n", MaxHandles));

	if(Status == EFI_SUCCESS) {
		if (MaxHandles == 0) {
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): no partitions found.\r\n"));
			return EFI_NO_MEDIA;
		}else if (MaxHandles != 1) {
			// Unable to deterministically load from single partition
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): multiple partitions found.\r\n"));
			return EFI_LOAD_ERROR;
		}
	}else {
		DEBUG ((EFI_D_ERROR, "[ABL] %s: GetBlkIOHandles failed: %r\n", __func__, Status));
		return Status;
	}

	BlkIo = HandleInfoList[0].BlkIo;
	Buffer = AllocatePages(ALIGN_PAGES(BufferSize, ALIGNMENT_MASK_4KB));
	//DEBUG ((EFI_D_ERROR, "ALIGN_PAGES Image size: %u Bytes\n", BufferSize));

	if (Buffer == NULL) {
		DEBUG ((EFI_D_ERROR, "[ABL] (ERROR)  disable verity - allocate buffer fail \n\n"));
		return EFI_DEVICE_ERROR;
	}

	if (Buffer != NULL) {
		BlkIo->ReadBlocks (BlkIo,
						BlkIo->Media->MediaId,
						0,
						BufferSize,//ROUND_TO_PAGE(ImageSize, BlkIo->Media->BlockSize - 1),
						Buffer);
	}

	//DEBUG ((EFI_D_INFO, "[ABL] disable verity - Buffer = %a\n", Buffer));

	char *temp = Buffer;
	temp[123] = 3;

	Status = BlkIo->WriteBlocks(BlkIo,
							BlkIo->Media->MediaId,
							0,
							BufferSize,
							Buffer);

	if(Status != EFI_SUCCESS) {
		DEBUG ((EFI_D_ERROR, "[ABL] (ERROR) disable verity - disable verity flags fail \n\n"));
		DEBUG ((EFI_D_ERROR, "[ABL] (ERROR) Status= %d \n\n", Status));
		return EFI_DEVICE_ERROR;
	}

	SetAvbVerity(TRUE);

	DEBUG((EFI_D_ERROR, "[ABL] --- disable verity() \n\n"));

	return Status;
}
// --- ASUS_BSP : add for disable verity

// +++ ASUS_BSP : add for enable verity
EFI_STATUS enable_verity(VOID)
{
	EFI_STATUS                   Status = EFI_SUCCESS;
	EFI_BLOCK_IO_PROTOCOL       *BlkIo;
	PartiSelectFilter            HandleFilter;
	HandleInfo                   HandleInfoList[1];
	STATIC UINT32                MaxHandles;
	STATIC UINT32                BlkIOAttrib = 0;
	VOID                        *Buffer;
	UINT32                       BufferSize = ALIGNMENT_MASK_4KB;

	//vbmeta_a
	EFI_GUID PartitionType = { 0x4b7a15d6, 0x322c, 0x42ac, { 0x81, 0x10, 0x88, 0xb7, 0xda, 0x0c, 0x5d, 0x77 } };

	DEBUG((EFI_D_ERROR, "[ABL] +++ enable verity() \n"));

	BlkIOAttrib = BLK_IO_SEL_PARTITIONED_MBR;
	BlkIOAttrib |= BLK_IO_SEL_PARTITIONED_GPT;
	BlkIOAttrib |= BLK_IO_SEL_MEDIA_TYPE_NON_REMOVABLE;
	BlkIOAttrib |= BLK_IO_SEL_MATCH_PARTITION_TYPE_GUID;

	HandleFilter.RootDeviceType = NULL;
	HandleFilter.PartitionType = &PartitionType;
	HandleFilter.VolumeName = NULL;

	MaxHandles = sizeof(HandleInfoList)/sizeof(*HandleInfoList);

	Status = GetBlkIOHandles (BlkIOAttrib, &HandleFilter, HandleInfoList, &MaxHandles);
	//DEBUG ((EFI_D_ERROR, "[ABL] read_partition - MaxHandles = %d\n", MaxHandles));

	if(Status == EFI_SUCCESS) {
		if (MaxHandles == 0) {
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): no partitions found.\r\n"));
			return EFI_NO_MEDIA;
		}else if (MaxHandles != 1) {
			// Unable to deterministically load from single partition
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): multiple partitions found.\r\n"));
			return EFI_LOAD_ERROR;
		}
	}else {
		DEBUG ((EFI_D_ERROR, "[ABL] %s: GetBlkIOHandles failed: %r\n", __func__, Status));
		return Status;
	}

	BlkIo = HandleInfoList[0].BlkIo;
	Buffer = AllocatePages(ALIGN_PAGES(BufferSize, ALIGNMENT_MASK_4KB));
	//DEBUG ((EFI_D_ERROR, "ALIGN_PAGES Image size: %u Bytes\n", BufferSize));

	if (Buffer == NULL) {
		DEBUG ((EFI_D_ERROR, "[ABL] (ERROR)  enable verity - allocate buffer fail \n\n"));
		return EFI_DEVICE_ERROR;
	}

	if (Buffer != NULL) {
		BlkIo->ReadBlocks (BlkIo,
						BlkIo->Media->MediaId,
						0,
						BufferSize,//ROUND_TO_PAGE(ImageSize, BlkIo->Media->BlockSize - 1),
						Buffer);
	}

	//DEBUG ((EFI_D_INFO, "[ABL] disable verity - Buffer = %a\n", Buffer));

	char *temp = Buffer;
	temp[123] = 0;

	Status = BlkIo->WriteBlocks(BlkIo,
							BlkIo->Media->MediaId,
							0,
							BufferSize,
							Buffer);

	if(Status != EFI_SUCCESS) {
		DEBUG ((EFI_D_ERROR, "[ABL] (ERROR) enable verity - enable verity flags fail \n\n"));
		DEBUG ((EFI_D_ERROR, "[ABL] (ERROR) Status= %d \n\n", Status));
		return EFI_DEVICE_ERROR;
	}

	SetAvbVerity(FALSE);

	DEBUG((EFI_D_ERROR, "[ABL] --- enable verity() \n\n"));

	return Status;
}
// --- ASUS_BSP : add for enable verity

// +++ ASUS_BSP : add for check AVB Verity
EFI_STATUS check_verity(VOID)
{
	EFI_STATUS                   Status = EFI_SUCCESS;
	EFI_BLOCK_IO_PROTOCOL       *BlkIo;
	PartiSelectFilter            HandleFilter;
	HandleInfo                   HandleInfoList[1];
	STATIC UINT32                MaxHandles;
	STATIC UINT32                BlkIOAttrib = 0;
	VOID                        *Buffer;
	UINT32                       BufferSize = ALIGNMENT_MASK_4KB;

	//vbmeta_a
	EFI_GUID PartitionType = { 0x4b7a15d6, 0x322c, 0x42ac, { 0x81, 0x10, 0x88, 0xb7, 0xda, 0x0c, 0x5d, 0x77 } };

	DEBUG((EFI_D_ERROR, "[ABL] +++ check AVB verity() \n"));

	BlkIOAttrib = BLK_IO_SEL_PARTITIONED_MBR;
	BlkIOAttrib |= BLK_IO_SEL_PARTITIONED_GPT;
	BlkIOAttrib |= BLK_IO_SEL_MEDIA_TYPE_NON_REMOVABLE;
	BlkIOAttrib |= BLK_IO_SEL_MATCH_PARTITION_TYPE_GUID;

	HandleFilter.RootDeviceType = NULL;
	HandleFilter.PartitionType = &PartitionType;
	HandleFilter.VolumeName = NULL;

	MaxHandles = sizeof(HandleInfoList)/sizeof(*HandleInfoList);

	Status = GetBlkIOHandles (BlkIOAttrib, &HandleFilter, HandleInfoList, &MaxHandles);

	if(Status == EFI_SUCCESS) {
		if (MaxHandles == 0) {
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): no partitions found.\r\n"));
			return EFI_NO_MEDIA;
		}else if (MaxHandles != 1) {
			// Unable to deterministically load from single partition
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): multiple partitions found.\r\n"));
			return EFI_LOAD_ERROR;
		}
	}else {
		DEBUG ((EFI_D_ERROR, "[ABL] %s: GetBlkIOHandles failed: %r\n", __func__, Status));
		return Status;
	}

	BlkIo = HandleInfoList[0].BlkIo;
	Buffer = AllocatePages(ALIGN_PAGES(BufferSize, ALIGNMENT_MASK_4KB));

	if (Buffer == NULL) {
		DEBUG ((EFI_D_ERROR, "[ABL] (ERROR)  AVB verity - allocate buffer fail \n\n"));
		return EFI_DEVICE_ERROR;
	}

	if (Buffer != NULL) {
		BlkIo->ReadBlocks (BlkIo,
						BlkIo->Media->MediaId,
						0,
						BufferSize,//ROUND_TO_PAGE(ImageSize, BlkIo->Media->BlockSize - 1),
						Buffer);
	}

	char *temp = Buffer;

	if (temp[123] != 0){
		SetAvbVerity(TRUE);
	}
	else{
		SetAvbVerity(FALSE);
	}

	DEBUG((EFI_D_ERROR, "[ABL] --- check AVB verity() \n\n"));
	return Status ;
}
// --- ASUS_BSP : add for check AVB Verity

// +++ ASUS_BSP : add for check apdp partition
EFI_STATUS check_apdp(VOID)
{
	EFI_STATUS                   Status = EFI_SUCCESS;
	EFI_BLOCK_IO_PROTOCOL       *BlkIo;
	PartiSelectFilter            HandleFilter;
	HandleInfo                   HandleInfoList[1];
	STATIC UINT32                MaxHandles;
	STATIC UINT32                BlkIOAttrib = 0;
	VOID                        *Buffer;
	UINT32                       BufferSize = ALIGNMENT_MASK_4KB;

	//apdp
	EFI_GUID PartitionType = { 0xE6E98DA2, 0xE22A, 0x4D12, { 0xAB, 0x33, 0x16, 0x9E, 0x7D, 0xEA, 0xA5, 0x07 } };

	DEBUG((EFI_D_ERROR, "[ABL] +++ check apdp partition() \n"));

	BlkIOAttrib = BLK_IO_SEL_PARTITIONED_MBR;
	BlkIOAttrib |= BLK_IO_SEL_PARTITIONED_GPT;
	BlkIOAttrib |= BLK_IO_SEL_MEDIA_TYPE_NON_REMOVABLE;
	BlkIOAttrib |= BLK_IO_SEL_MATCH_PARTITION_TYPE_GUID;

	HandleFilter.RootDeviceType = NULL;
	HandleFilter.PartitionType = &PartitionType;
	HandleFilter.VolumeName = NULL;

	MaxHandles = sizeof(HandleInfoList)/sizeof(*HandleInfoList);

	Status = GetBlkIOHandles (BlkIOAttrib, &HandleFilter, HandleInfoList, &MaxHandles);

	if(Status == EFI_SUCCESS) {
		if (MaxHandles == 0) {
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): no partitions found.\r\n"));
			return EFI_NO_MEDIA;
		}else if (MaxHandles != 1) {
			// Unable to deterministically load from single partition
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): multiple partitions found.\r\n"));
			return EFI_LOAD_ERROR;
		}
	}else {
		DEBUG ((EFI_D_ERROR, "[ABL] %s: GetBlkIOHandles failed: %r\n", __func__, Status));
		return Status;
	}

	BlkIo = HandleInfoList[0].BlkIo;
	Buffer = AllocatePages(ALIGN_PAGES(BufferSize, ALIGNMENT_MASK_4KB));

	if (Buffer == NULL) {
		DEBUG ((EFI_D_ERROR, "[ABL] (ERROR) check apdp partition - allocate buffer fail \n\n"));
		return EFI_DEVICE_ERROR;
	}

	if (Buffer != NULL) {
		BlkIo->ReadBlocks (BlkIo,
						BlkIo->Media->MediaId,
						0,
						BufferSize,//ROUND_TO_PAGE(ImageSize, BlkIo->Media->BlockSize - 1),
						Buffer);
	}
	DEBUG ((EFI_D_INFO, "[ABL] check apdp partition - Buffer = %a\n", Buffer));

	if(!AsciiStriCmp (Buffer, "")){
		DEBUG((EFI_D_ERROR, "[ABL] check apdp partition = FALSE \n"));
		SetAPDP(FALSE);
	}else{
		DEBUG((EFI_D_ERROR, "[ABL] check apdp partition = TRUE \n"));
		SetAPDP(TRUE);
	}

	DEBUG((EFI_D_ERROR, "[ABL] --- check apdp partition() \n\n"));
	return Status ;

}
// --- ASUS_BSP : add for check apdp partition

// +++ ASUS_BSP : add for erase_apdp_partition
EFI_STATUS erase_apdp_partition(VOID)
{
	EFI_STATUS                   Status = EFI_SUCCESS;
	EFI_BLOCK_IO_PROTOCOL       *BlkIo;
	PartiSelectFilter            HandleFilter;
	HandleInfo                   HandleInfoList[1];
	STATIC UINT32                MaxHandles;
	STATIC UINT32                BlkIOAttrib = 0;
	VOID                        *Buffer;
	UINT32                       BufferSize = ALIGNMENT_MASK_4KB;

	//apdp
	EFI_GUID PartitionType = { 0xE6E98DA2, 0xE22A, 0x4D12, { 0xAB, 0x33, 0x16, 0x9E, 0x7D, 0xEA, 0xA5, 0x07 } };

	DEBUG((EFI_D_ERROR, "[ABL] +++ erase_apdp_partition() \n"));

	BlkIOAttrib = BLK_IO_SEL_PARTITIONED_MBR;
	BlkIOAttrib |= BLK_IO_SEL_PARTITIONED_GPT;
	BlkIOAttrib |= BLK_IO_SEL_MEDIA_TYPE_NON_REMOVABLE;
	BlkIOAttrib |= BLK_IO_SEL_MATCH_PARTITION_TYPE_GUID;

	HandleFilter.RootDeviceType = NULL;
	HandleFilter.PartitionType = &PartitionType;
	HandleFilter.VolumeName = NULL;

	MaxHandles = sizeof(HandleInfoList)/sizeof(*HandleInfoList);

	Status = GetBlkIOHandles (BlkIOAttrib, &HandleFilter, HandleInfoList, &MaxHandles);

	if(Status == EFI_SUCCESS) {
		if (MaxHandles == 0) {
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): no partitions found.\r\n"));
			return EFI_NO_MEDIA;
		}else if (MaxHandles != 1) {
			// Unable to deterministically load from single partition
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): multiple partitions found.\r\n"));
			return EFI_LOAD_ERROR;
		}
	}else {
		DEBUG ((EFI_D_ERROR, "[ABL] %s: GetBlkIOHandles failed: %r\n", __func__, Status));
		return Status;
	}

	BlkIo = HandleInfoList[0].BlkIo;

	Buffer = AllocatePages(ALIGN_PAGES(BufferSize, ALIGNMENT_MASK_4KB));

	if (Buffer == NULL) {
		DEBUG ((EFI_D_ERROR, "[ABL] (ERROR) erase_apdp_partition - allocate buffer fail \n\n"));
		return EFI_DEVICE_ERROR ;
	}

	if (Buffer != NULL) {
		BlkIo->ReadBlocks (BlkIo,
						BlkIo->Media->MediaId,
						0,
						BufferSize,//ROUND_TO_PAGE(ImageSize, BlkIo->Media->BlockSize - 1),
						Buffer);
	}

	memset(Buffer,0,BufferSize);

	Status = BlkIo->WriteBlocks(BlkIo,
							BlkIo->Media->MediaId,
							0,
							BufferSize,
							Buffer);

	if (Status != EFI_SUCCESS) {
		DEBUG ((EFI_D_ERROR, "[ABL] (ERROR) erase_apdp_partition - erase_apdp_partition fail \n\n"));
		return EFI_DEVICE_ERROR ;
	}

	DEBUG((EFI_D_ERROR, "[ABL] --- erase_apdp_partition() \n\n"));
	return Status ;
}

// --- ASUS_BSP : add for erase_apdp_partition

// +++ ASUS_BSP : check if have rawdump partition or not
BOOLEAN check_rawdump_partition(void)
{
	EFI_STATUS                   Status = EFI_SUCCESS;
	PartiSelectFilter            HandleFilter;
	HandleInfo                   HandleInfoList[1];
	STATIC UINT32                MaxHandles;
	STATIC UINT32                BlkIOAttrib = 0;
	BOOLEAN                      rawdump_en = FALSE;

	//rawdump
	EFI_GUID PartitionType = { 0x66C9B323, 0xF7FC, 0x48B6, { 0xBF, 0x96, 0x6F, 0x32, 0xE3, 0x35, 0xA4, 0x28 } };

	DEBUG((EFI_D_ERROR, "[ABL] +++ check_rawdump_partition() \n"));

	BlkIOAttrib = BLK_IO_SEL_PARTITIONED_MBR;
	BlkIOAttrib |= BLK_IO_SEL_PARTITIONED_GPT;
	BlkIOAttrib |= BLK_IO_SEL_MEDIA_TYPE_NON_REMOVABLE;
	BlkIOAttrib |= BLK_IO_SEL_MATCH_PARTITION_TYPE_GUID;

	HandleFilter.RootDeviceType = NULL;
	HandleFilter.PartitionType = &PartitionType;
	HandleFilter.VolumeName = NULL;

	MaxHandles = sizeof(HandleInfoList)/sizeof(*HandleInfoList);

	Status = GetBlkIOHandles (BlkIOAttrib, &HandleFilter, HandleInfoList, &MaxHandles);

	if(Status == EFI_SUCCESS) {
		if(MaxHandles != 1) {
			//Unable to deterministically load from single partition
			DEBUG(( EFI_D_ERROR, "[ABL] ExecImgFromVolume(): MaxHandles = %d .\r\n", MaxHandles));
			DEBUG ((EFI_D_ERROR, "[ABL] Not found rawdump partition, set androidboot.rawdump.en=0\n"));
			AsciiSPrint(cmd_rawdump_en, sizeof(cmd_rawdump_en), " androidboot.rawdump_en=0");
			rawdump_en = FALSE;
		}else {
			DEBUG ((EFI_D_ERROR, "[ABL] Found rawdump partition, set androidboot.rawdump.en=1\n"));
			AsciiSPrint(cmd_rawdump_en, sizeof(cmd_rawdump_en), " androidboot.rawdump_en=1");
			rawdump_en = TRUE;
		}
	}

	DEBUG((EFI_D_ERROR, "[ABL] +++ check_rawdump_partition() \n\n"));
	return rawdump_en ;
}
// --- ASUS_BSP : check if have rawdump partition or not

// +++ ASUS_BSP : re-partition from gpt to partition:0 for add rawdump partition
EFI_STATUS RePartition(void)
{
	EFI_STATUS                   Status = EFI_SUCCESS;
	EFI_BLOCK_IO_PROTOCOL       *BlkIo;
	PartiSelectFilter            HandleFilter;
	HandleInfo                   HandleInfoList[1];
	STATIC UINT32                MaxHandles;
	STATIC UINT32                BlkIOAttrib = 0;
	VOID                        *Buffer;
	UINT32                       Partition_0_SizeImageSize = 45056;

	//gpt
	EFI_GUID PartitionType_gpt = { 0xe7411056, 0xc9cc, 0x4e25, { 0xaf, 0x32, 0x62, 0x94, 0xd7, 0x93, 0x69, 0xfa } };

	DEBUG((EFI_D_ERROR, "[ABL] +++ read gpt partition() \n"));

	BlkIOAttrib = BLK_IO_SEL_PARTITIONED_MBR;
	BlkIOAttrib |= BLK_IO_SEL_PARTITIONED_GPT;
	BlkIOAttrib |= BLK_IO_SEL_MEDIA_TYPE_NON_REMOVABLE;
	BlkIOAttrib |= BLK_IO_SEL_MATCH_PARTITION_TYPE_GUID;

	HandleFilter.RootDeviceType = NULL;
	HandleFilter.PartitionType = &PartitionType_gpt;
	HandleFilter.VolumeName = NULL;

	MaxHandles = sizeof(HandleInfoList)/sizeof(*HandleInfoList);

	Status = GetBlkIOHandles (BlkIOAttrib, &HandleFilter, HandleInfoList, &MaxHandles);

	if(Status == EFI_SUCCESS) {
		if (MaxHandles == 0) {
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): no partitions found.\r\n"));
			return EFI_NO_MEDIA;
		}else if (MaxHandles != 1) {
			// Unable to deterministically load from single partition
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): multiple partitions found.\r\n"));
			return EFI_LOAD_ERROR;
		}
	}else {
		DEBUG ((EFI_D_ERROR, "[ABL] %s: GetBlkIOHandles failed: %r\n", __func__, Status));
		return Status;
	}

	BlkIo = HandleInfoList[0].BlkIo;

	Buffer = AllocatePages(ALIGN_PAGES(Partition_0_SizeImageSize, ALIGNMENT_MASK_4KB));

	if (Buffer == NULL) {
		DEBUG ((EFI_D_ERROR, "[ABL] (ERROR) read gpt partition - allocate buffer fail \n\n"));
		return EFI_DEVICE_ERROR ;
	}

	if (Buffer != NULL) {
		BlkIo->ReadBlocks (BlkIo,
						BlkIo->Media->MediaId,
						0,
						Partition_0_SizeImageSize,//ROUND_TO_PAGE(ImageSize, BlkIo->Media->BlockSize - 1),
						Buffer);
	}
	DEBUG ((EFI_D_INFO, "[ABL] read gpt partition - Buffer = %a\n", Buffer));

	Status = FactoryResetFromRecovery();
	if (Status != EFI_SUCCESS) {
		DEBUG((EFI_D_ERROR, "[ABL] Write misc partition fail, Error= %r\n", Status));
	}

	Status = UpdatePartitionTable (Buffer, Partition_0_SizeImageSize, 0, Ptable);
	if (Status != EFI_SUCCESS) {
		DEBUG((EFI_D_ERROR, "[ABL] Unable to UpdatePartitionTable from get to partition:0, Error= %r\n", Status));
	}

	FreePages(Buffer, ALIGN_PAGES(Partition_0_SizeImageSize, ALIGNMENT_MASK_4KB));

	DEBUG((EFI_D_ERROR, "[ABL] --- read gpt partition() \n\n"));
	return Status;
}
// --- ASUS_BSP : re-partition from gpt to partition:0 for add rawdump partition

//+++ ASUS_BSP : erase_frp_partition
EFI_STATUS erase_frp_partition(VOID)
{
	EFI_STATUS                   Status = EFI_SUCCESS;
	EFI_BLOCK_IO_PROTOCOL       *BlkIo;
	PartiSelectFilter            HandleFilter;
	HandleInfo                   HandleInfoList[1];
	STATIC UINT32                MaxHandles;
	STATIC UINT32                BlkIOAttrib = 0;
	VOID                        *Buffer;
	UINT32                       BufferSize = ALIGNMENT_MASK_4KB;

	//frp
	EFI_GUID PartitionType = { 0x91B72D4D, 0x71E0, 0x4CBF, { 0x9B, 0x8E, 0x23, 0x63, 0x81, 0xCF, 0xF1, 0x7A } };

	DEBUG((EFI_D_ERROR, "[ABL] +++ erase_frp_partition() \n"));

	BlkIOAttrib = BLK_IO_SEL_PARTITIONED_MBR;
	BlkIOAttrib |= BLK_IO_SEL_PARTITIONED_GPT;
	BlkIOAttrib |= BLK_IO_SEL_MEDIA_TYPE_NON_REMOVABLE;
	BlkIOAttrib |= BLK_IO_SEL_MATCH_PARTITION_TYPE_GUID;

	HandleFilter.RootDeviceType = NULL;
	HandleFilter.PartitionType = &PartitionType;
	HandleFilter.VolumeName = NULL;

	MaxHandles = sizeof(HandleInfoList)/sizeof(*HandleInfoList);

	Status = GetBlkIOHandles (BlkIOAttrib, &HandleFilter, HandleInfoList, &MaxHandles);

	if(Status == EFI_SUCCESS) {
		if (MaxHandles == 0) {
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): no partitions found.\r\n"));
			return EFI_NO_MEDIA;
		}else if (MaxHandles != 1) {
			// Unable to deterministically load from single partition
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): multiple partitions found.\r\n"));
			return EFI_LOAD_ERROR;
		}
	}else {
		DEBUG ((EFI_D_ERROR, "[ABL] %s: GetBlkIOHandles failed: %r\n", __func__, Status));
		return Status;
	}

	BlkIo = HandleInfoList[0].BlkIo;

	Buffer = AllocatePages(ALIGN_PAGES(BufferSize, ALIGNMENT_MASK_4KB));

	if (Buffer == NULL) {
		DEBUG ((EFI_D_ERROR, "[ABL] (ERROR) erase_frp_partition - allocate buffer fail \n\n"));
		return EFI_DEVICE_ERROR ;
	}

	if (Buffer != NULL) {
		BlkIo->ReadBlocks (BlkIo,
						BlkIo->Media->MediaId,
						0,
						BufferSize,//ROUND_TO_PAGE(ImageSize, BlkIo->Media->BlockSize - 1),
						Buffer);
	}

	memset(Buffer,0,BufferSize);

	Status = BlkIo->WriteBlocks(BlkIo,
							BlkIo->Media->MediaId,
							0,
							BufferSize,
							Buffer);

	if (Status != EFI_SUCCESS) {
		DEBUG ((EFI_D_ERROR, "[ABL] (ERROR) erase_frp_partition - erase_frp_partition fail \n\n"));
		return EFI_DEVICE_ERROR ;
	}

	DEBUG((EFI_D_ERROR, "[ABL] --- erase_frp_partition() \n"));
	return Status ;
}
// --- ASUS_BSP : erase_frp_partition

/*
 * asuskey4:
 * 128 bytes for each block
 * index:
 * 0	SSN
 * 1	IMEI
 * 2	IMEI2
 * 3	ISN
 * 4	CID
 * 5	COUNTRY
 * 6	TOOLID
 */

#define asuskey4_parse_size     128
#define asuskey4_SSN_index        0
#define asuskey4_IMEI_index       1
#define asuskey4_IMEI2_index      2
#define asuskey4_ISN_index        3
#define asuskey4_CID_index        4
#define asuskey4_COUNTRY_index    5
#define asuskey4_TOOLID_index     6

// +++ ASUS_BSP : add for ssn info
EFI_STATUS read_ssn(void)
{
	EFI_STATUS                   Status = EFI_SUCCESS;
	EFI_BLOCK_IO_PROTOCOL       *BlkIo;
	PartiSelectFilter            HandleFilter;
	HandleInfo                   HandleInfoList[1];
	STATIC UINT32                MaxHandles;
	STATIC UINT32                BlkIOAttrib = 0;
	VOID                        *Buffer;
	UINT32                       BufferSize = ALIGNMENT_MASK_4KB;
	CHAR8                        ssn_tmp[SSN_LEN+1];
	int                          i=0;

	//asuskey4
	EFI_GUID PartitionType = { 0xb2fd0ae8, 0xbf27, 0x4646, { 0x88, 0xfb, 0x9f, 0x2a, 0x8b, 0xc7, 0x72, 0xac } };

	DEBUG((EFI_D_ERROR, "[ABL] +++ read_ssn() \n"));

	BlkIOAttrib = BLK_IO_SEL_PARTITIONED_MBR;
	BlkIOAttrib |= BLK_IO_SEL_PARTITIONED_GPT;
	BlkIOAttrib |= BLK_IO_SEL_MEDIA_TYPE_NON_REMOVABLE;
	BlkIOAttrib |= BLK_IO_SEL_MATCH_PARTITION_TYPE_GUID;

	HandleFilter.RootDeviceType = NULL;
	HandleFilter.PartitionType = &PartitionType;
	HandleFilter.VolumeName = NULL;

	MaxHandles = sizeof(HandleInfoList)/sizeof(*HandleInfoList);

	Status = GetBlkIOHandles (BlkIOAttrib, &HandleFilter, HandleInfoList, &MaxHandles);
	//DEBUG ((EFI_D_ERROR, "[ABL] read_partition - MaxHandles = %d\n", MaxHandles));

	if(Status == EFI_SUCCESS) {
		if (MaxHandles == 0) {
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): no partitions found.\r\n"));
			//set default serial number
			AsciiStrnCpyS (ssn_tmp, sizeof(ssn_tmp), "111111111111111", sizeof(ssn_tmp));
			ssn_tmp[SSN_LEN] = '\0';
			DEBUG ((EFI_D_ERROR, "[ABL][SSN] Default - ssn_tmp = %a\n", ssn_tmp));
			Status = SetSSNNum(ssn_tmp, sizeof(ssn_tmp));
			if (Status != EFI_SUCCESS)
			{
				DEBUG((EFI_D_ERROR, "[ABL] Unable to Write Device Info : ssn %r\n", Status));
				return Status ;
			}
			return EFI_NO_MEDIA;
		}else if (MaxHandles != 1) {
			// Unable to deterministically load from single partition
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): multiple partitions found.\r\n"));
			return EFI_LOAD_ERROR;
		}
	}else {
		DEBUG ((EFI_D_ERROR, "[ABL] %s: GetBlkIOHandles failed: %r\n", __func__, Status));
		return Status;
	}

	BlkIo = HandleInfoList[0].BlkIo;
	Buffer = AllocatePages(ALIGN_PAGES(BufferSize, ALIGNMENT_MASK_4KB));

	//DEBUG ((EFI_D_ERROR, "ALIGN_PAGES Image size: %u Bytes\n", BufferSize));

	if (Buffer != NULL)
	{
		BlkIo->ReadBlocks (BlkIo,
						BlkIo->Media->MediaId,
						0,
						BufferSize,//ROUND_TO_PAGE(ImageSize, BlkIo->Media->BlockSize - 1),
						Buffer);
	}

	//DEBUG ((EFI_D_INFO, "[ABL][SSN] read_partition - Buffer = %a\n", Buffer));

	CHAR8 *temp = Buffer;

	if(!AsciiStriCmp (Buffer, ""))
	{
		DEBUG ((EFI_D_ERROR, "[ABL][SSN] read_partition - asuskey4 no data \n"));
		//set default serial number
		AsciiStrnCpyS (ssn_tmp, sizeof(ssn_tmp), "111111111111111", sizeof(ssn_tmp));
		ssn_tmp[SSN_LEN] = '\0';
		DEBUG ((EFI_D_ERROR, "[ABL][SSN] Default - ssn_tmp = %a\n", ssn_tmp));
	}
	else
	{
		//AsciiStrnCpy (ssn_tmp, (CHAR8*)Buffer, sizeof(ssn_tmp));

		for (i=0 ; i<= SSN_LEN ; i++)
		{
			ssn_tmp[i] = temp[i + asuskey4_parse_size * asuskey4_SSN_index];
		}
		ssn_tmp[SSN_LEN] = '\0';

		DEBUG ((EFI_D_ERROR, "[ABL][SSN] read_partition - Buffer = %a, ssn_tmp = %a\n", Buffer, ssn_tmp));
	}

	Status = SetSSNNum(ssn_tmp, sizeof(ssn_tmp));
	if (Status != EFI_SUCCESS)
	{
		DEBUG((EFI_D_ERROR, "[ABL] Unable to Write Device Info : ssn %r\n", Status));
		return Status ;
	}

	DEBUG((EFI_D_ERROR, "[ABL] --- read_ssn() \n\n"));
	return Status ;
}
// --- ASUS_BSP : add for ssn info

// +++ ASUS_BSP : add for imei info
EFI_STATUS read_imei(void)
{
	EFI_STATUS                   Status = EFI_SUCCESS;
	EFI_BLOCK_IO_PROTOCOL       *BlkIo;
	PartiSelectFilter            HandleFilter;
	HandleInfo                   HandleInfoList[1];
	STATIC UINT32                MaxHandles;
	STATIC UINT32                BlkIOAttrib = 0;
	VOID                        *Buffer;
	UINT32                       BufferSize = ALIGNMENT_MASK_4KB;
	CHAR8                        imei_temp[IMEI_LEN+1];
	int                          i=0;

	//asuskey4
	EFI_GUID PartitionType = { 0xb2fd0ae8, 0xbf27, 0x4646, { 0x88, 0xfb, 0x9f, 0x2a, 0x8b, 0xc7, 0x72, 0xac } };

	DEBUG((EFI_D_ERROR, "[ABL] +++ read_imei() \n"));

	BlkIOAttrib = BLK_IO_SEL_PARTITIONED_MBR;
	BlkIOAttrib |= BLK_IO_SEL_PARTITIONED_GPT;
	BlkIOAttrib |= BLK_IO_SEL_MEDIA_TYPE_NON_REMOVABLE;
	BlkIOAttrib |= BLK_IO_SEL_MATCH_PARTITION_TYPE_GUID;

	HandleFilter.RootDeviceType = NULL;
	HandleFilter.PartitionType = &PartitionType;
	HandleFilter.VolumeName = NULL;

	MaxHandles = sizeof(HandleInfoList)/sizeof(*HandleInfoList);

	Status = GetBlkIOHandles (BlkIOAttrib, &HandleFilter, HandleInfoList, &MaxHandles);
	//DEBUG ((EFI_D_ERROR, "[ABL] read_partition - MaxHandles = %d\n", MaxHandles));

	if(Status == EFI_SUCCESS) {
		if (MaxHandles == 0) {
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): no partitions found.\r\n"));
			return EFI_NO_MEDIA;
		}else if (MaxHandles != 1) {
			// Unable to deterministically load from single partition
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): multiple partitions found.\r\n"));
			return EFI_LOAD_ERROR;
		}
	}else {
		DEBUG ((EFI_D_ERROR, "[ABL] %s: GetBlkIOHandles failed: %r\n", __func__, Status));
		return Status;
	}

	BlkIo = HandleInfoList[0].BlkIo;
	Buffer = AllocatePages(ALIGN_PAGES(BufferSize, ALIGNMENT_MASK_4KB));

	//DEBUG ((EFI_D_ERROR, "ALIGN_PAGES Image size: %u Bytes\n", BufferSize));

	if (Buffer != NULL)
	{
		BlkIo->ReadBlocks (BlkIo,
						BlkIo->Media->MediaId,
						0,
						BufferSize,//ROUND_TO_PAGE(ImageSize, BlkIo->Media->BlockSize - 1),
						Buffer);
	}

	//DEBUG ((EFI_D_INFO, "[ABL][IMEI] read_partition - Buffer = %a\n", Buffer));

	CHAR8 *temp = Buffer;

	if(!AsciiStriCmp (Buffer, ""))
	{
		DEBUG ((EFI_D_ERROR, "[ABL][IMEI] read_partition - asuskey4 no data \n"));
		//set default imei number
		AsciiStrnCpyS (imei_temp, sizeof(imei_temp), "111111111111111", sizeof(imei_temp));
		imei_temp[IMEI_LEN] = '\0';
		DEBUG ((EFI_D_ERROR, "[ABL][IMEI] Default - imei_temp = %a\n", imei_temp));
	}
	else
	{
		for (i=0 ; i<= IMEI_LEN ; i++)
		{
			imei_temp[i] = temp[i + asuskey4_parse_size * asuskey4_IMEI_index];
		}
		imei_temp[IMEI_LEN] = '\0';

		DEBUG ((EFI_D_ERROR, "[ABL][IMEI] read_partition - temp = %a, imei_temp = %a\n", temp, imei_temp));
	}

	Status = SetIMEINum(imei_temp, sizeof(imei_temp));

	if (Status != EFI_SUCCESS)
	{
		DEBUG((EFI_D_ERROR, "[ABL] Unable to Write Device Info : iemi %r\n", Status));
		return Status ;
	}

	DEBUG((EFI_D_ERROR, "[ABL] --- read_imei() \n\n"));
	return Status ;
}
// --- ASUS_BSP : add for imei info

// +++ ASUS_BSP : add for imei2 info
EFI_STATUS read_imei2(void)
{
	EFI_STATUS                   Status = EFI_SUCCESS;
	EFI_BLOCK_IO_PROTOCOL       *BlkIo;
	PartiSelectFilter            HandleFilter;
	HandleInfo                   HandleInfoList[1];
	STATIC UINT32                MaxHandles;
	STATIC UINT32                BlkIOAttrib = 0;
	VOID                        *Buffer;
	UINT32                       BufferSize = ALIGNMENT_MASK_4KB;
	CHAR8                        imei2_temp[IMEI_LEN+1];
	int                          i=0;

	//asuskey4
	EFI_GUID PartitionType = { 0xb2fd0ae8, 0xbf27, 0x4646, { 0x88, 0xfb, 0x9f, 0x2a, 0x8b, 0xc7, 0x72, 0xac } };

	DEBUG((EFI_D_ERROR, "[ABL] +++ read_imei2() \n"));

	BlkIOAttrib = BLK_IO_SEL_PARTITIONED_MBR;
	BlkIOAttrib |= BLK_IO_SEL_PARTITIONED_GPT;
	BlkIOAttrib |= BLK_IO_SEL_MEDIA_TYPE_NON_REMOVABLE;
	BlkIOAttrib |= BLK_IO_SEL_MATCH_PARTITION_TYPE_GUID;

	HandleFilter.RootDeviceType = NULL;
	HandleFilter.PartitionType = &PartitionType;
	HandleFilter.VolumeName = NULL;

	MaxHandles = sizeof(HandleInfoList)/sizeof(*HandleInfoList);

	Status = GetBlkIOHandles (BlkIOAttrib, &HandleFilter, HandleInfoList, &MaxHandles);
	//DEBUG ((EFI_D_ERROR, "[ABL] read_partition - MaxHandles = %d\n", MaxHandles));

	if(Status == EFI_SUCCESS) {
		if (MaxHandles == 0) {
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): no partitions found.\r\n"));
			return EFI_NO_MEDIA;
		}else if (MaxHandles != 1) {
			// Unable to deterministically load from single partition
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): multiple partitions found.\r\n"));
			return EFI_LOAD_ERROR;
		}
	}else {
		DEBUG ((EFI_D_ERROR, "[ABL] %s: GetBlkIOHandles failed: %r\n", __func__, Status));
		return Status;
	}

	BlkIo = HandleInfoList[0].BlkIo;
	Buffer = AllocatePages(ALIGN_PAGES(BufferSize, ALIGNMENT_MASK_4KB));

	//DEBUG ((EFI_D_ERROR, "ALIGN_PAGES Image size: %u Bytes\n", BufferSize));

	if (Buffer != NULL)
	{
		BlkIo->ReadBlocks (BlkIo,
						BlkIo->Media->MediaId,
						0,
						BufferSize,//ROUND_TO_PAGE(ImageSize, BlkIo->Media->BlockSize - 1),
						Buffer);
	}

	//DEBUG ((EFI_D_INFO, "[ABL][IMEI] read_partition - Buffer = %a\n", Buffer));

	CHAR8 *temp = Buffer;

	if(!AsciiStriCmp (Buffer, ""))
	{
		DEBUG ((EFI_D_ERROR, "[ABL][IMEI2] read_partition - asuskey4 no data \n"));
		//set default imei2 number
		AsciiStrnCpyS (imei2_temp, sizeof(imei2_temp), "111111111111111", sizeof(imei2_temp));
		imei2_temp[IMEI_LEN] = '\0';
		DEBUG ((EFI_D_ERROR, "[ABL][IMEI2] Default - imei2_temp = %a\n", imei2_temp));
	}
	else
	{
		for (i=0 ; i<= IMEI_LEN ; i++)
		{
			imei2_temp[i] = temp[i + asuskey4_parse_size * asuskey4_IMEI2_index];
		}
		imei2_temp[IMEI_LEN] = '\0';

		DEBUG ((EFI_D_ERROR, "[ABL][IMEI2] read_partition - temp = %a, imei_temp = %a\n", temp, imei2_temp));
	}

	Status = SetIMEI2Num(imei2_temp, sizeof(imei2_temp));

	if (Status != EFI_SUCCESS)
	{
		DEBUG((EFI_D_ERROR, "[ABL] Unable to Write Device Info : iemi %r\n", Status));
		return Status ;
	}

	DEBUG((EFI_D_ERROR, "[ABL] --- read_imei2() \n\n"));
	return Status ;
}
// --- ASUS_BSP : add for imei2 info

// +++ ASUS_BSP : add for isn info
EFI_STATUS read_isn(void)
{
	EFI_STATUS                   Status = EFI_SUCCESS;
	EFI_BLOCK_IO_PROTOCOL       *BlkIo;
	PartiSelectFilter            HandleFilter;
	HandleInfo                   HandleInfoList[1];
	STATIC UINT32                MaxHandles;
	STATIC UINT32                BlkIOAttrib = 0;
	VOID                        *Buffer;
	UINT32                       BufferSize = ALIGNMENT_MASK_4KB;
	CHAR8                        isn_tmp[ISN_LEN+1];
	int                          i=0;

	//asuskey4
	EFI_GUID PartitionType = { 0xb2fd0ae8, 0xbf27, 0x4646, { 0x88, 0xfb, 0x9f, 0x2a, 0x8b, 0xc7, 0x72, 0xac } };

	DEBUG((EFI_D_ERROR, "[ABL] +++ read_isn() \n"));

	BlkIOAttrib = BLK_IO_SEL_PARTITIONED_MBR;
	BlkIOAttrib |= BLK_IO_SEL_PARTITIONED_GPT;
	BlkIOAttrib |= BLK_IO_SEL_MEDIA_TYPE_NON_REMOVABLE;
	BlkIOAttrib |= BLK_IO_SEL_MATCH_PARTITION_TYPE_GUID;

	HandleFilter.RootDeviceType = NULL;
	HandleFilter.PartitionType = &PartitionType;
	HandleFilter.VolumeName = NULL;

	MaxHandles = sizeof(HandleInfoList)/sizeof(*HandleInfoList);

	Status = GetBlkIOHandles (BlkIOAttrib, &HandleFilter, HandleInfoList, &MaxHandles);
	//DEBUG ((EFI_D_ERROR, "[ABL] read_partition - MaxHandles = %d\n", MaxHandles));

	if(Status == EFI_SUCCESS) {
		if (MaxHandles == 0) {
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): no partitions found.\r\n"));
			return EFI_NO_MEDIA;
		}else if (MaxHandles != 1) {
			// Unable to deterministically load from single partition
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): multiple partitions found.\r\n"));
			return EFI_LOAD_ERROR;
		}
	}else {
		DEBUG ((EFI_D_ERROR, "[ABL] %s: GetBlkIOHandles failed: %r\n", __func__, Status));
		return Status;
	}

	BlkIo = HandleInfoList[0].BlkIo;
	Buffer = AllocatePages(ALIGN_PAGES(BufferSize, ALIGNMENT_MASK_4KB));

	//DEBUG ((EFI_D_ERROR, "ALIGN_PAGES Image size: %u Bytes\n", BufferSize));

	if (Buffer != NULL)
	{
		BlkIo->ReadBlocks (BlkIo,
						BlkIo->Media->MediaId,
						0,
						BufferSize,//ROUND_TO_PAGE(ImageSize, BlkIo->Media->BlockSize - 1),
						Buffer);
	}

	//DEBUG ((EFI_D_INFO, "[ABL][ISN] read_partition - Buffer = %a\n", Buffer));

	CHAR8 *temp = Buffer;

	if(!AsciiStriCmp (Buffer, ""))
	{
		DEBUG ((EFI_D_ERROR, "[ABL][ISN] read_partition - asuskey4 no data \n"));
		//set default serial number
		AsciiStrnCpyS (isn_tmp, sizeof(DevInfo.isn_num), "FFAABBCC123456789", sizeof(DevInfo.isn_num));
		isn_tmp[ISN_LEN] = '\0';
		DEBUG ((EFI_D_ERROR, "[ABL][ISN] Default - isn_tmp = %a\n", isn_tmp));
	}
	else
	{
		//AsciiStrnCpy (isn_tmp, (CHAR8*)Buffer, sizeof(DevInfo.isn_num));

		for (i=0 ; i<= ISN_LEN ; i++)
		{
			isn_tmp[i] = temp[i + asuskey4_parse_size * asuskey4_ISN_index];
		}
		isn_tmp[ISN_LEN] = '\0';

		DEBUG ((EFI_D_ERROR, "[ABL][ISN] read_partition - Buffer = %a, isn_tmp = %a\n", Buffer, isn_tmp));
	}

	Status = SetISNNum(isn_tmp, sizeof(isn_tmp));

	if (Status != EFI_SUCCESS)
	{
		DEBUG((EFI_D_ERROR, "[ABL] Unable to Write Device Info : isn %r\n", Status));
		return Status ;
	}

	DEBUG((EFI_D_ERROR, "[ABL] --- read_isn() \n\n"));
	return Status ;
}
// --- ASUS_BSP : add for isn info

// +++ ASUS_BSP : add for cid info
EFI_STATUS read_cid(void)
{
	EFI_STATUS                   Status = EFI_SUCCESS;
	EFI_BLOCK_IO_PROTOCOL       *BlkIo;
	PartiSelectFilter            HandleFilter;
	HandleInfo                   HandleInfoList[1];
	STATIC UINT32                MaxHandles;
	STATIC UINT32                BlkIOAttrib = 0;
	VOID                        *Buffer;
	UINT32                       BufferSize = ALIGNMENT_MASK_4KB;
	char                         cid_temp[CID_LEN+1] = {0};
	int                          i=0;

	//asuskey4
	EFI_GUID PartitionType = { 0xb2fd0ae8, 0xbf27, 0x4646, { 0x88, 0xfb, 0x9f, 0x2a, 0x8b, 0xc7, 0x72, 0xac } };

	DEBUG((EFI_D_ERROR, "[ABL] +++ read_cid() \n"));

	BlkIOAttrib = BLK_IO_SEL_PARTITIONED_MBR;
	BlkIOAttrib |= BLK_IO_SEL_PARTITIONED_GPT;
	BlkIOAttrib |= BLK_IO_SEL_MEDIA_TYPE_NON_REMOVABLE;
	BlkIOAttrib |= BLK_IO_SEL_MATCH_PARTITION_TYPE_GUID;

	HandleFilter.RootDeviceType = NULL;
	HandleFilter.PartitionType = &PartitionType;
	HandleFilter.VolumeName = NULL;

	MaxHandles = sizeof(HandleInfoList)/sizeof(*HandleInfoList);

	Status = GetBlkIOHandles (BlkIOAttrib, &HandleFilter, HandleInfoList, &MaxHandles);
	//DEBUG ((EFI_D_ERROR, "[ABL] read_partition - MaxHandles = %d\n", MaxHandles));

	if(Status == EFI_SUCCESS) {
		if (MaxHandles == 0) {
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): no partitions found.\r\n"));
			return EFI_NO_MEDIA;
		}else if (MaxHandles != 1) {
			// Unable to deterministically load from single partition
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): multiple partitions found.\r\n"));
			return EFI_LOAD_ERROR;
		}
	}else {
		DEBUG ((EFI_D_ERROR, "[ABL] %s: GetBlkIOHandles failed: %r\n", __func__, Status));
		return Status;
	}

	BlkIo = HandleInfoList[0].BlkIo;
	Buffer = AllocatePages(ALIGN_PAGES(BufferSize, ALIGNMENT_MASK_4KB));

	//DEBUG ((EFI_D_ERROR, "ALIGN_PAGES Image size: %u Bytes\n", BufferSize));

	if (Buffer != NULL)
	{
		BlkIo->ReadBlocks (BlkIo,
						BlkIo->Media->MediaId,
						0,
						BufferSize,//ROUND_TO_PAGE(ImageSize, BlkIo->Media->BlockSize - 1),
						Buffer);
	}

	//DEBUG ((EFI_D_INFO, "[ABL][CID] read_partition - Buffer = %a\n", Buffer));

	CHAR8 *temp = Buffer;

	if(!AsciiStriCmp (Buffer, ""))
	{
		DEBUG ((EFI_D_ERROR, "[ABL][CID] read_partition - asuskey4 no data \n"));
		//set default cid number
		AsciiStrnCpyS (cid_temp, sizeof(DevInfo.cid_name), "ASUS", sizeof(DevInfo.cid_name));
		cid_temp[CID_LEN] = '\0';
		DEBUG ((EFI_D_ERROR, "[ABL][CID] Default - cid_temp = %a\n", cid_temp));
	}
	else
	{
		for (i=0 ; i<= CID_LEN ; i++)
		{
			cid_temp[i] = temp[i + asuskey4_parse_size * asuskey4_CID_index];
		}
		cid_temp[CID_LEN] = '\0';

		DEBUG ((EFI_D_ERROR, "[ABL][CID] read_partition - temp = %a, cid_temp = %a\n", temp, cid_temp));
	}

	Status = SetCIDName(cid_temp, sizeof(cid_temp));
	AsciiSPrint(cid_name, sizeof(cid_name), cid_temp);

	if (Status != EFI_SUCCESS)
	{
		DEBUG((EFI_D_ERROR, "[ABL] Unable to Write Device Info : cid %r\n", Status));
		return Status ;
	}

	DEBUG((EFI_D_ERROR, "[ABL] --- read_cid() \n\n"));
	return Status ;
}
// --- ASUS_BSP : add for cid info

// +++ ASUS_BSP : add for read country code
EFI_STATUS read_countrycode(void)
{
	EFI_STATUS                   Status = EFI_SUCCESS;
	EFI_BLOCK_IO_PROTOCOL       *BlkIo;
	PartiSelectFilter            HandleFilter;
	HandleInfo                   HandleInfoList[1];
	STATIC UINT32                MaxHandles;
	STATIC UINT32                BlkIOAttrib = 0;
	VOID                        *Buffer;
	UINT32                       BufferSize = ALIGNMENT_MASK_4KB;
	char                         country_temp[COUNTRY_LEN+1] = {0};
	int                          i=0;

	//asuskey4
	EFI_GUID PartitionType = { 0xb2fd0ae8, 0xbf27, 0x4646, { 0x88, 0xfb, 0x9f, 0x2a, 0x8b, 0xc7, 0x72, 0xac } };

	DEBUG((EFI_D_ERROR, "[ABL] +++ read_countrycode() \n"));

	BlkIOAttrib = BLK_IO_SEL_PARTITIONED_MBR;
	BlkIOAttrib |= BLK_IO_SEL_PARTITIONED_GPT;
	BlkIOAttrib |= BLK_IO_SEL_MEDIA_TYPE_NON_REMOVABLE;
	BlkIOAttrib |= BLK_IO_SEL_MATCH_PARTITION_TYPE_GUID;

	HandleFilter.RootDeviceType = NULL;
	HandleFilter.PartitionType = &PartitionType;
	HandleFilter.VolumeName = NULL;

	MaxHandles = sizeof(HandleInfoList)/sizeof(*HandleInfoList);

	Status = GetBlkIOHandles (BlkIOAttrib, &HandleFilter, HandleInfoList, &MaxHandles);
	//DEBUG ((EFI_D_ERROR, "[ABL] read_partition - MaxHandles = %d\n", MaxHandles));

	if(Status == EFI_SUCCESS) {
		if (MaxHandles == 0) {
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): no partitions found.\r\n"));
			return EFI_NO_MEDIA;
		}else if (MaxHandles != 1) {
			// Unable to deterministically load from single partition
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): multiple partitions found.\r\n"));
			return EFI_LOAD_ERROR;
		}
	}else {
		DEBUG ((EFI_D_ERROR, "[ABL] %s: GetBlkIOHandles failed: %r\n", __func__, Status));
		return Status;
	}

	BlkIo = HandleInfoList[0].BlkIo;
	Buffer = AllocatePages(ALIGN_PAGES(BufferSize, ALIGNMENT_MASK_4KB));

	//DEBUG ((EFI_D_ERROR, "ALIGN_PAGES Image size: %u Bytes\n", BufferSize));

	if (Buffer != NULL)
	{
		BlkIo->ReadBlocks (BlkIo,
						BlkIo->Media->MediaId,
						0,
						BufferSize,//ROUND_TO_PAGE(ImageSize, BlkIo->Media->BlockSize - 1),
						Buffer);
	}

	//DEBUG ((EFI_D_INFO, "[ABL][COUNTRY] read_partition - Buffer = %a\n", Buffer));

	CHAR8 *temp = Buffer;

	if(!AsciiStriCmp (Buffer, ""))
	{
		DEBUG ((EFI_D_ERROR, "[ABL][COUNTRY] read_partition - asuskey4 no data \n"));

		//set default country code
		AsciiStrnCpyS (country_temp, sizeof(country_temp), "WW", sizeof(country_temp));
		country_temp[COUNTRY_LEN] = '\0';
		DEBUG ((EFI_D_ERROR, "[ABL][COUNTRY] Default - country_temp = %a\n", country_temp));
	}
	else
	{
		for (i=0 ; i<= COUNTRY_LEN ; i++)
		{
			country_temp[i] = temp[i + asuskey4_parse_size * asuskey4_COUNTRY_index];
		}
		country_temp[COUNTRY_LEN] = '\0';

		DEBUG ((EFI_D_ERROR, "[ABL][COUNTRY] read_partition - temp = %a, country_temp = %a\n", temp, country_temp));
	}

    AsciiSPrint(cmd_country_code, sizeof(cmd_country_code), " androidboot.country_code=%a", country_temp);

	Status = SetCountryCode(country_temp, sizeof(country_temp));
	if (Status != EFI_SUCCESS)
	{
		DEBUG((EFI_D_ERROR, "[ABL] Unable to Write Device Info : country_temp %r\n", Status));
		return Status ;
	}

    AsciiSPrint(cmd_product_name, sizeof(cmd_product_name), " androidboot.product.override=%a_%a", country_temp, DevInfo.project_name);
    DEBUG ((EFI_D_ERROR, "[ABL][PRODUCT] cmd_product_name = %a\n", cmd_product_name));

	DEBUG((EFI_D_ERROR, "[ABL] --- read_countrycode() \n\n"));
	return Status ;

}
// --- ASUS_BSP : add for read country code

// +++ ASUS_BSP : add for read TOOLID
EFI_STATUS read_TOOLID(void)
{
	EFI_STATUS                   Status = EFI_SUCCESS;
	EFI_BLOCK_IO_PROTOCOL       *BlkIo;
	PartiSelectFilter            HandleFilter;
	HandleInfo                   HandleInfoList[1];
	STATIC UINT32                MaxHandles;
	STATIC UINT32                BlkIOAttrib = 0;
	VOID                        *Buffer;
	UINT32                       BufferSize = ALIGNMENT_MASK_4KB;
	char                         toolid_temp[TOOLID_LEN+1] = {0};
	int                          i=0;

	//asuskey4
	EFI_GUID PartitionType = { 0xb2fd0ae8, 0xbf27, 0x4646, { 0x88, 0xfb, 0x9f, 0x2a, 0x8b, 0xc7, 0x72, 0xac } };

	DEBUG((EFI_D_ERROR, "[ABL] +++ read_TOOLID() \n"));

	BlkIOAttrib = BLK_IO_SEL_PARTITIONED_MBR;
	BlkIOAttrib |= BLK_IO_SEL_PARTITIONED_GPT;
	BlkIOAttrib |= BLK_IO_SEL_MEDIA_TYPE_NON_REMOVABLE;
	BlkIOAttrib |= BLK_IO_SEL_MATCH_PARTITION_TYPE_GUID;

	HandleFilter.RootDeviceType = NULL;
	HandleFilter.PartitionType = &PartitionType;
	HandleFilter.VolumeName = NULL;

	MaxHandles = sizeof(HandleInfoList)/sizeof(*HandleInfoList);

	Status = GetBlkIOHandles (BlkIOAttrib, &HandleFilter, HandleInfoList, &MaxHandles);
	//DEBUG ((EFI_D_ERROR, "[ABL] read_partition - MaxHandles = %d\n", MaxHandles));

	if(Status == EFI_SUCCESS) {
		if (MaxHandles == 0) {
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): no partitions found.\r\n"));
			return EFI_NO_MEDIA;
		}else if (MaxHandles != 1) {
			// Unable to deterministically load from single partition
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): multiple partitions found.\r\n"));
			return EFI_LOAD_ERROR;
		}
	}else {
		DEBUG ((EFI_D_ERROR, "[ABL] %s: GetBlkIOHandles failed: %r\n", __func__, Status));
		return Status;
	}

	BlkIo = HandleInfoList[0].BlkIo;
	Buffer = AllocatePages(ALIGN_PAGES(BufferSize, ALIGNMENT_MASK_4KB));

	//DEBUG ((EFI_D_ERROR, "ALIGN_PAGES Image size: %u Bytes\n", BufferSize));

	if (Buffer != NULL)
	{
		BlkIo->ReadBlocks (BlkIo,
						BlkIo->Media->MediaId,
						0,
						BufferSize,//ROUND_TO_PAGE(ImageSize, BlkIo->Media->BlockSize - 1),
						Buffer);
	}

	//DEBUG ((EFI_D_INFO, "[ABL][TOOLID] read_partition - Buffer = %a\n", Buffer));

	CHAR8 *temp = Buffer;
	sys_info sysinfo;
	char sysinfo_toolid_check[6];

	if(!AsciiStriCmp (Buffer, ""))
	{
		DEBUG ((EFI_D_ERROR, "[ABL][TOOLID] read_partition - asuskey4 no data \n"));
		//set default toolid
		if(read_sysinfo(&sysinfo) == EFI_SUCCESS)
		{
			AsciiSPrint(sysinfo_toolid_check, sizeof(sysinfo_toolid_check), "%a", sysinfo.toolid_check);
			if(!AsciiStrnCmp(sysinfo_toolid_check, "false", StrLen (L"false"))){
				AsciiStrnCpyS (toolid_temp, sizeof(toolid_temp), "ASUS_DEFAULT_ID_1", sizeof(toolid_temp));
			} else if (AsciiStrnCmp(sysinfo_toolid_check, "true", StrLen (L"true"))){
				AsciiStrnCpyS (toolid_temp, sizeof(toolid_temp), "ASUS_DEFAULT_ID_2", sizeof(toolid_temp));
			} else {
				AsciiStrnCpyS (toolid_temp, sizeof(toolid_temp), "ASUS_DEFAULT_ID_1", sizeof(toolid_temp));
			}
			toolid_temp[TOOLID_LEN] = '\0';
			DEBUG ((EFI_D_ERROR, "[ABL][TOOLID] Default - toolid_temp = %a\n", toolid_temp));
		}
	}
	else
	{
		for (i=0 ; i<= TOOLID_LEN ; i++)
		{
			toolid_temp[i] = temp[i + asuskey4_parse_size * asuskey4_TOOLID_index];
		}

		if(!AsciiStriCmp (toolid_temp, ""))
		{
			//set default toolid
			if(read_sysinfo(&sysinfo) == EFI_SUCCESS)
			{
				AsciiSPrint(sysinfo_toolid_check, sizeof(sysinfo_toolid_check), "%a", sysinfo.toolid_check);
				if(!AsciiStrCmp(sysinfo_toolid_check, "")){
					AsciiStrnCpyS (toolid_temp, sizeof(toolid_temp), "ASUS_DEFAULT_ID_1", sizeof(toolid_temp));
				}else if (!AsciiStrnCmp(sysinfo_toolid_check, "true", StrLen (L"true"))){
					AsciiStrnCpyS (toolid_temp, sizeof(toolid_temp), "ASUS_DEFAULT_ID_2", sizeof(toolid_temp));
				}
			}
		}

		toolid_temp[TOOLID_LEN] = '\0';
		DEBUG ((EFI_D_ERROR, "[ABL][TOOLID] read_partition - temp = %a, toolid_temp = %a\n", temp, toolid_temp));
	}

	Status = SetTOOLID(toolid_temp, sizeof(toolid_temp));
	AsciiSPrint(cmd_toolid, sizeof(cmd_toolid), " androidboot.toolid=%a", toolid_temp);

	if (Status != EFI_SUCCESS)
	{
		DEBUG((EFI_D_ERROR, "[ABL] Unable to Write Device Info : toolid_temp %r\n", Status));
		return Status ;
	}

	DEBUG((EFI_D_ERROR, "[ABL] --- read_TOOLID() \n\n"));
	return Status ;
}
// --- ASUS_BSP : add for read TOOLID

// +++ ASUS_BSP : add for user unlock
EFI_STATUS read_asuskey(UINT8 *signature_ptr, UINT32 signature_len)
{
	EFI_STATUS                   Status = EFI_SUCCESS;
	EFI_BLOCK_IO_PROTOCOL       *BlkIo;
	PartiSelectFilter            HandleFilter;
	HandleInfo                   HandleInfoList[1];
	STATIC UINT32                MaxHandles;
	STATIC UINT32                BlkIOAttrib = 0;
	VOID                        *Buffer;
	UINT32                       BufferSize = ALIGNMENT_MASK_4KB;

	//asuskey
	EFI_GUID PartitionType = { 0x30f09946, 0x3dea, 0x4b24, { 0xbe, 0xa4, 0xca, 0x6a, 0xb6, 0x4e, 0xbf, 0xde } };

	DEBUG((EFI_D_ERROR, "[ABL] +++ read_sysinfo() \n"));

	BlkIOAttrib = BLK_IO_SEL_PARTITIONED_MBR;
	BlkIOAttrib |= BLK_IO_SEL_PARTITIONED_GPT;
	BlkIOAttrib |= BLK_IO_SEL_MEDIA_TYPE_NON_REMOVABLE;
	BlkIOAttrib |= BLK_IO_SEL_MATCH_PARTITION_TYPE_GUID;

	HandleFilter.RootDeviceType = NULL;
	HandleFilter.PartitionType = &PartitionType;
	HandleFilter.VolumeName = NULL;

	DEBUG((EFI_D_ERROR, "[ABL] +++ read_asuskey(signature_len=%d) \n",signature_len));
	MaxHandles = sizeof(HandleInfoList)/sizeof(*HandleInfoList);

	Status = GetBlkIOHandles (BlkIOAttrib, &HandleFilter, HandleInfoList, &MaxHandles);
	//DEBUG ((EFI_D_ERROR, "[ABL] read_partition - MaxHandles = %d\n", MaxHandles));

	if(Status == EFI_SUCCESS) {
		if (MaxHandles == 0) {
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): no partitions found.\r\n"));
			return EFI_NO_MEDIA;
		}else if (MaxHandles != 1) {
			// Unable to deterministically load from single partition
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): multiple partitions found.\r\n"));
			return EFI_LOAD_ERROR;
		}
	}else {
		DEBUG ((EFI_D_ERROR, "[ABL] %s: GetBlkIOHandles failed: %r\n", __func__, Status));
		return Status;
	}

	BlkIo = HandleInfoList[0].BlkIo;
	Buffer = AllocatePages(ALIGN_PAGES(BufferSize, ALIGNMENT_MASK_4KB));

	//DEBUG ((EFI_D_ERROR, "ALIGN_PAGES Image size: %u Bytes\n", BufferSize));

	if (Buffer != NULL)
	{
		BlkIo->ReadBlocks (BlkIo,
						BlkIo->Media->MediaId,
						0,
						BufferSize,//ROUND_TO_PAGE(ImageSize, BlkIo->Media->BlockSize - 1),
						Buffer);
	}

	DEBUG ((EFI_D_INFO, "[ABL] read_asuskey - Buffer = %a\n", Buffer));

	if(!memcmp(signature_ptr, (UINT8*)Buffer, SIGNATURE_LEN))
	{
		DEBUG ((EFI_D_ERROR, "[ABL]  (ERROR) read_asuskey - asuskey no data \n\n"));
		return EFI_DEVICE_ERROR;
	}
	else
	{
		DEBUG ((EFI_D_ERROR, "[ABL] read_partition - Buffer = %a\n", Buffer));
		memcpy(signature_ptr,(UINT8*)Buffer,signature_len);
	}

	//erase asusdata partition : 4096
	memset(Buffer,0,BufferSize);

	Status = BlkIo->WriteBlocks(BlkIo, BlkIo->Media->MediaId, 0, BufferSize, Buffer);
	if(Status != EFI_SUCCESS)
	{
		DEBUG ((EFI_D_ERROR, "[ABL] (ERROR) read_asuskey - erase asuskey fail \n\n"));
		return EFI_DEVICE_ERROR ;
	}

	#if 0
	UINT32 i=0;
	UINT32 sz=SIGNATURE_LEN;

	DEBUG((EFI_D_ERROR, "[ABL] SIG[%d] = \n",sz));
	for(i=0;i<sz;i++)
	{
		DEBUG((EFI_D_ERROR, " %02x",signature_ptr[i] ));
	}
	DEBUG((EFI_D_ERROR, "\n"));
	#endif

	#if 0
	BlkIo->ReadBlocks (BlkIo,
						BlkIo->Media->MediaId,
						0,
						BufferSize,//ROUND_TO_PAGE(ImageSize, BlkIo->Media->BlockSize - 1),
						Buffer);
	i=0;
	sz=256;

	memset(tmp,0,256);
	memcpy(tmp,(UINT8*)Buffer,sizeof(tmp));

	DEBUG((EFI_D_ERROR, "[ABL] after erase asusdata : tmp[%d] = \n",sz));
	for(i=0;i<sz;i++)
	{
		DEBUG((EFI_D_ERROR, " %02x",tmp[i] ));
	}
	DEBUG((EFI_D_ERROR, "\n"));
	#endif

	DEBUG((EFI_D_ERROR, "[ABL] --- read_asuskey - return \n",Status));
	return Status ;
}

//CmdRsaTest()
unsigned is_unlock(UINT32 decrypt_type)
{
	EFI_STATUS Status = EFI_SUCCESS;

	unsigned val = 0;
	CHAR8 ISN[21]={0};	// Get ISN
	UINT8 SIG[SIGNATURE_LEN];
	size_t ISN_size = 0;

	// 1 Get ISN
	GetISNNum(ISN, sizeof(ISN));
	ISN_size = strlen(ISN);
	DEBUG((EFI_D_ERROR, "[ABL] +++ is_unlock(ISN=%a),size=%d\n",ISN,ISN_size));

	// 2. Get signature from asuskey partition
	memset(SIG,0,sizeof(SIG));
	read_asuskey(SIG,sizeof(SIG));
	if (Status != EFI_SUCCESS)
	{
		DEBUG((EFI_D_ERROR, "[ABL] is_unlock - read_asuskey fail \n"));
	}
	//AsciiSPrint(asus_key_info, sizeof(asus_key_info), "%a", "G6AZCY03S376EYA");

	// 3. RSA Verify
	Status = SecRSATestAppMain(SIG,sizeof(SIG),(UINT8*)ISN,ISN_size,decrypt_type);
	if (Status != EFI_SUCCESS)
	{
		DEBUG((EFI_D_ERROR, "[ABL]  is_unlock - Verify FAIL\n"));
		val =  0;
	}
	else
	{
		DEBUG((EFI_D_ERROR, "[ABL]  is_unlock - Verify PASS\n"));
		val =  1;
	}

	DEBUG((EFI_D_ERROR, "[ABL]  --- CmdRsaTest()\n"));
	return val;
}
// --- ASUS_BSP : add for user unlock

// +++ ASUS_BSP : add for check CN devices with IMEI range
int atoi(const char *nptr)
{
	int       Retval;
	BOOLEAN   Negative = FALSE;

	while(isspace((const unsigned char)*nptr)) ++nptr; // Skip leading spaces

	if(*nptr == '+') {
		Negative = FALSE;
		++nptr;
	}
	else if(*nptr == '-') {
		Negative = TRUE;
		++nptr;
	}
	Retval = (int)AsciiStrDecimalToUintn(nptr);
	if(Negative) {
		Retval = -Retval;
	}
	return Retval;
}

BOOLEAN valid_image = FALSE;
char cmd_valid_image[64] = {0};
char imei_range[3] = {"WW"}; //WW, CN
const UINT8 PREFIX_NUM=8;
const UINT8 SURFIX_NUM=7;
const char imei1_prefix_cn_entry[9]= {"35188118"};
const char imei1_prefix_cn_elite[9]= {"35042003"};
STATIC VOID CheckValidImage(void)
{
	CHAR8 imei1[16];
	GetIMEINum(imei1, sizeof(imei1));

	// Only compare IMEI range with TARGET_SKU of CN device
	// if pass, mark as valid and boot into kernel.
	if(!strncmp(imei1,imei1_prefix_cn_entry,PREFIX_NUM)) {
		memcpy(imei_range,"CN",sizeof("CN"));

		const char *mp_surfix = &imei1[PREFIX_NUM];
		int  imei_surfix = atoi(mp_surfix);

		DEBUG((EFI_D_INFO, "[ABL] CheckValidImage : CN ENTRY IMEI, surfix: %d\n", imei_surfix));
		if(imei_surfix >= 130010 && imei_surfix <= 9999999) {
			DEBUG((EFI_D_INFO, "[ABL] CheckValidImage : MP IMEI, CHECK\n"));
			// MP IMEI, check sku
			if(!strcmp("CN",ABL_BUILD_SKU)) {
				valid_image = TRUE;
			}
		} else {
			DEBUG((EFI_D_INFO, "[ABL] CheckValidImage : non MP IMEI, PASS\n"));
			// non MP IMEI , force valid.
			valid_image = TRUE;
		}
	} else if(!strncmp(imei1,imei1_prefix_cn_elite,PREFIX_NUM)) {
		memcpy(imei_range,"CN",sizeof("CN"));

		const char *mp_surfix = &imei1[PREFIX_NUM];
		int  imei_surfix = atoi(mp_surfix);

		DEBUG((EFI_D_INFO, "[ABL] CheckValidImage : CN ELITE IMEI, surfix: %d\n",imei_surfix));
		if(imei_surfix >= 130010 && imei_surfix <= 9999999) {
			DEBUG((EFI_D_INFO, "[ABL] CheckValidImage : MP IMEI, CHECK\n"));
			// MP IMEI, check sku
			if(!strcmp("CN",ABL_BUILD_SKU)) {
				valid_image = TRUE;
			}
		} else {
			// non MP IMEI , force valid.
			DEBUG((EFI_D_INFO, "[ABL] CheckValidImage : non MP IMEI, PASS\n"));
			valid_image = TRUE;
		}
	} else {
		// non-CN devices, mark it as WW. just skip valid image check.
		memcpy(imei_range,"WW",sizeof("WW"));
		valid_image = TRUE;
		DEBUG((EFI_D_INFO, "[ABL] CheckValidImage : WW or other IMEI, force image valid!\n"));
	}

	DEBUG((EFI_D_INFO, "[ABL] CheckValidImage : Result: %a\n",valid_image ? "PASS":"FAIL"));

	// force image valid on Factory build.
	if(!strcmp(ABL_BUILD_SKU, "OPEN")) {
		DEBUG((EFI_D_INFO, "[ABL] CheckValidImage : Image SKU: %a, force image valid!\n", ABL_BUILD_SKU));
		valid_image = TRUE;
	}

	DEBUG((EFI_D_INFO, "[ABL] CheckValidImage : Result: %a\n", valid_image ? "PASS":"FAIL"));
	DEBUG((EFI_D_INFO, "[ABL] CheckValidImage : Image SKU: %a\n", ABL_BUILD_SKU));
	DEBUG((EFI_D_INFO, "[ABL] CheckValidImage : IMEI  SKU: %a\n", imei_range));
	DEBUG((EFI_D_INFO, "[ABL] CheckValidImage : IMEI: %a\n", imei1));

	AsciiSPrint(cmd_valid_image, sizeof(cmd_valid_image), " androidboot.image.valid=%a", valid_image ? "Y":"N");
}
// --- ASUS_BSP : add for check CN devices with IMEI range

void Get_Boot_Count(){
	EFI_STATUS Status;
	UINTN boot_count = 0;
	UINTN boot_count_s = sizeof(boot_count);
	UINT32 boot_count_value = 0;

	Status = gRT->GetVariable(
			L"ASUSBootCount",
			&gEfiGlobalVariableGuid,
			NULL,
			&boot_count_s,
			&boot_count);

	boot_count_value = (UINT32)(boot_count & 0xFFFF);
	DEBUG((EFI_D_ERROR, "[ABL] Get_Boot_Count = %d\n", boot_count_value));

	if (Status != EFI_SUCCESS)
		DEBUG((EFI_D_ERROR, "[ABL] (ERROR) Unable to get boot count : %r \n", Status));

	SetBootCounter(boot_count_value);
	AsciiSPrint(cmd_boot_count, sizeof(cmd_boot_count), " androidboot.bootcount=%d", boot_count_value);
}

// +++ ASUS_BSP : check if powerkey pressed at xbl
UINT32 Get_XBL_PowerKey_Press(){
	EFI_STATUS Status;
	UINT32  xbl_powerkey_press = 0;
	UINTN xbl_powerkey_press_size = sizeof(xbl_powerkey_press);

	Status = gRT->GetVariable(
			L"ASUSPowerKeyPress",
			&gQcomTokenSpaceGuid,
			NULL,
			&xbl_powerkey_press_size,
			&xbl_powerkey_press);

	DEBUG((EFI_D_ERROR, "[ABL] check_xbl_powerkey_press : %d\n", xbl_powerkey_press));

	if (Status != EFI_SUCCESS) {
		DEBUG((EFI_D_ERROR, "[ABL] (ERROR) check_xbl_powerkey_press failed : %r \n", Status));
	}

	return xbl_powerkey_press;
}
// --- ASUS_BSP : check if powerkey pressed at xbl

// +++ ASUS_BSP : add for ftm mode
unsigned is_ftm_mode(void)
{
	unsigned ftm_val = ASUS_NORMAL_MODE; //default = 0 : normal mode

	#ifndef ABL_FTM
		ftm_val = ASUS_NORMAL_MODE;
		DEBUG((EFI_D_ERROR, "\n"));
		DEBUG((EFI_D_ERROR, "======================================================= \n"));
		DEBUG((EFI_D_ERROR, "  NORMAL MODE\n"));
		DEBUG((EFI_D_ERROR, "======================================================= \n"));
		DEBUG((EFI_D_ERROR, "\n"));
		AsciiSPrint(cmd_ftm_mode_prop, sizeof(cmd_ftm_mode_prop), " androidboot.pre-ftm=%d ", ftm_val);
	#else
		ftm_val = ASUS_FTM_MODE;
		DEBUG((EFI_D_ERROR, "\n"));
		DEBUG((EFI_D_ERROR, "======================================================= \n"));
		DEBUG((EFI_D_ERROR, "  FTM MODE\n"));
		DEBUG((EFI_D_ERROR, "======================================================= \n"));
		DEBUG((EFI_D_ERROR, "\n"));
	#endif

	return ftm_val;
}

void set_cmdline(int enter_mode)
{
	DEBUG((EFI_D_ERROR, "======================================================= \n"));

	if (ASUS_NORMAL_MODE != enter_mode)
	{
		// adb on
		AsciiSPrint(cmd_enable_adb_mode, sizeof(cmd_enable_adb_mode), " ADB=Y");
		AsciiSPrint(cmd_enable_adb_prop, sizeof(cmd_enable_adb_prop), " androidboot.adb.enable=1");
		DEBUG((EFI_D_ERROR," %a\n", cmd_enable_adb_mode));

		if(ASUS_FTM_MODE == enter_mode)
		{
			// permissive
			AsciiSPrint(cmd_selinux, sizeof(cmd_selinux), " selinux=0");
			AsciiSPrint(cmd_selinux_prop, sizeof(cmd_selinux_prop), " androidboot.selinux=permissive");
			DEBUG((EFI_D_ERROR," %a\n", cmd_selinux));

			// ftm property: default = 0 : normal mode ; 1 : FTM mode
			AsciiSPrint(cmd_ftm_mode, sizeof(cmd_ftm_mode), " UNLOCKED=F", ASUS_FTM_MODE);
			AsciiSPrint(cmd_ftm_mode_prop, sizeof(cmd_ftm_mode_prop), " androidboot.pre-ftm=%d", ASUS_FTM_MODE);
			DEBUG((EFI_D_ERROR," %a\n", cmd_ftm_mode));
		}
		else if(ASUS_USERDEBUG_MODE == enter_mode)
		{
			// ftm property: default = 0 : normal mode ; 1 : FTM mode 2 : ASUS_USERDEBUG_MODE
			AsciiSPrint(cmd_ftm_mode_prop, sizeof(cmd_ftm_mode_prop), " androidboot.mode=%a","userdebug");
			DEBUG((EFI_D_ERROR," %a\n", cmd_ftm_mode));
		}
	}
	else
	{
		DEBUG((EFI_D_ERROR, "[ABL] set NORMAL mode: do nothing\n"));
	}

	DEBUG((EFI_D_ERROR, "======================================================= \n"));
}
// --- ASUS_BSP : add for ftm mode

// +++ ASUS_BSP : add for logcat-asdf sevices
void IsLogcatAsdfOn(void)
{
    if(DevInfo.is_logcat_asdf_on)
    {
        DEBUG((EFI_D_ERROR, "Enable logcat asdf\n"));
        AsciiSPrint(cmd_enable_logcat_asdf, sizeof(cmd_enable_logcat_asdf), " androidboot.force_logcat_asdf=1");
    }
    else
    {
        AsciiSPrint(cmd_enable_logcat_asdf, sizeof(cmd_enable_logcat_asdf), " androidboot.force_logcat_asdf=0");
    }
}
// --- ASUS_BSP : add for logcat-asdf sevices

// ASUS_BSP : add for update devcfg for app unlock device +++
EFI_STATUS copy_asuskey3_to_devcfg(VOID)
{
    EFI_STATUS                   Status = EFI_SUCCESS;
    EFI_BLOCK_IO_PROTOCOL       *BlkIo;
    PartiSelectFilter            HandleFilter;
    HandleInfo                   HandleInfoList[1];
    STATIC UINT32                MaxHandles;
    STATIC UINT32                BlkIOAttrib = 0;
    VOID                        *Buffer;
    UINT32                       PartitionSize;
    UINT32                       asuskey3_ImageSize = 131072;//128*1024

    //CRC check first
    unsigned int cal_crc_devcfg_a = 0xffffffff;
    unsigned int cal_crc_devcfg_b = 0xffffffff;
    unsigned int cal_asuskey3_128k = 0xffffffff;
    cal_crc_devcfg_a = AsusCalculatePtCrc32(L"devcfg_a");
    cal_crc_devcfg_b = AsusCalculatePtCrc32(L"devcfg_b");
    cal_asuskey3_128k = AsusCalculatePtCrc32Size(L"asuskey3", asuskey3_ImageSize);
    
    //DEBUG((EFI_D_INFO, "cal_crc_devcfg_a = %x\n", cal_crc_devcfg_a));
    //DEBUG((EFI_D_INFO, "cal_crc_devcfg_b = %x\n", cal_crc_devcfg_b));
    //DEBUG((EFI_D_INFO, "cal_asuskey3_128k = %x\n", cal_asuskey3_128k));
    //return Status;
	
    if(cal_crc_devcfg_a == cal_asuskey3_128k && cal_crc_devcfg_b == cal_asuskey3_128k){
        DEBUG((EFI_D_ERROR, "[ABL] cm.mbn is already copied\n"));
        return Status;
    }

    DEBUG((EFI_D_ERROR, "[ABL] read Partition asuskey3\n"));

    BlkIOAttrib = BLK_IO_SEL_PARTITIONED_MBR;
    BlkIOAttrib |= BLK_IO_SEL_PARTITIONED_GPT;
    BlkIOAttrib |= BLK_IO_SEL_MEDIA_TYPE_NON_REMOVABLE;
    BlkIOAttrib |= BLK_IO_SEL_MATCH_PARTITION_LABEL;

    HandleFilter.RootDeviceType = NULL;
    HandleFilter.PartitionLabel = L"asuskey3";
    HandleFilter.VolumeName = NULL;

    MaxHandles = sizeof(HandleInfoList)/sizeof(*HandleInfoList);

    Status = GetBlkIOHandles (BlkIOAttrib, &HandleFilter, HandleInfoList, &MaxHandles);

    if(Status == EFI_SUCCESS) {
        if (MaxHandles == 0) {
            DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): no partitions found.\r\n"));
            return EFI_NO_MEDIA;
        }else if (MaxHandles != 1) {
            // Unable to deterministically load from single partition
            DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): multiple partitions found.\r\n"));
            return EFI_LOAD_ERROR;
        }
    }else {
        DEBUG ((EFI_D_ERROR, "[ABL] %s: GetBlkIOHandles failed: %r\n", __func__, Status));
        return Status;
    }

    BlkIo = HandleInfoList[0].BlkIo;

    PartitionSize = (BlkIo->Media->LastBlock + 1) * BlkIo->Media->BlockSize;

    Buffer = AllocatePages(ALIGN_PAGES(PartitionSize, ALIGNMENT_MASK_4KB));

    if (Buffer == NULL) {
        DEBUG ((EFI_D_ERROR, "[ABL] (ERROR) read asuskey3 partition - allocate buffer fail \n\n"));
        return EFI_DEVICE_ERROR;
    }

    if (Buffer != NULL) {
        BlkIo->ReadBlocks (BlkIo,
                        BlkIo->Media->MediaId,
                        0,
                        PartitionSize,//ROUND_TO_PAGE(ImageSize, BlkIo->Media->BlockSize - 1),
                        Buffer);
    }
    
    //check asuskey3 header is .elf or not
    //DEBUG ((EFI_D_INFO, "[ABL] Buffer[0] = %02x\n", ((uint8_t*) Buffer)[0]));
    //DEBUG ((EFI_D_INFO, "[ABL] Buffer[1] = %02x\n", ((uint8_t*) Buffer)[1]));
    //DEBUG ((EFI_D_INFO, "[ABL] Buffer[2] = %02x\n", ((uint8_t*) Buffer)[2]));
    //DEBUG ((EFI_D_INFO, "[ABL] Buffer[3] = %02x\n", ((uint8_t*) Buffer)[3]));
    if( (((uint8_t*) Buffer)[0] != 0x7F)  || (((uint8_t*) Buffer)[1] != 0x45) || 
           (((uint8_t*) Buffer)[2] != 0x4C)  || (((uint8_t*) Buffer)[3] != 0x46)){
        DEBUG ((EFI_D_ERROR, "[ABL] asuskey3 header error, %02x%02x%02x%02x\n", ((uint8_t*) Buffer)[0], ((uint8_t*) Buffer)[1], ((uint8_t*) Buffer)[2], ((uint8_t*) Buffer)[3]));
        return Status;
    }
	
    if(cal_crc_devcfg_a != cal_asuskey3_128k){
        DEBUG ((EFI_D_INFO, "[ABL] read devcfg_a partition - Buffer = %a\n", Buffer));

        HandleFilter.RootDeviceType = NULL;
        HandleFilter.PartitionLabel = L"devcfg_a";
        HandleFilter.VolumeName = NULL;

        DEBUG((EFI_D_ERROR, "[ABL] +++ load devcfg_a partition() \n"));

        MaxHandles = sizeof(HandleInfoList)/sizeof(*HandleInfoList);

        Status = GetBlkIOHandles (BlkIOAttrib, &HandleFilter, HandleInfoList, &MaxHandles);

        if(Status == EFI_SUCCESS) {
            if (MaxHandles == 0) {
                DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): no partitions found.\r\n"));
                return EFI_NO_MEDIA;
            }else if (MaxHandles != 1) {
                // Unable to deterministically load from single partition
                DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): multiple partitions found.\r\n"));
                return EFI_LOAD_ERROR;
            }
        }else {
            DEBUG ((EFI_D_ERROR, "[ABL] %s: GetBlkIOHandles failed: %r\n", __func__, Status));
            return Status;
        }

        BlkIo = HandleInfoList[0].BlkIo;

        PartitionSize = (BlkIo->Media->LastBlock + 1) * BlkIo->Media->BlockSize;
        DEBUG ((EFI_D_ERROR, "[ABL] PartitionSize = %d \n",PartitionSize));
		
        DEBUG((EFI_D_ERROR, "[ABL] +++ write devcfg_a partition() \n"));
        Status = BlkIo->WriteBlocks(BlkIo,
                                BlkIo->Media->MediaId,
                                0,
                                PartitionSize,
                                Buffer);

        if(Status != EFI_SUCCESS) {
            DEBUG ((EFI_D_ERROR, "[ABL] (ERROR) write devcfg_a partition fail \n\n"));
            DEBUG ((EFI_D_ERROR, "[ABL] (ERROR) Status= %d \n\n",Status));
            return EFI_DEVICE_ERROR;
        }

        DEBUG((EFI_D_ERROR, "[ABL] --- write devcfg_a partition() \n\n"));
    }
	
    if(cal_crc_devcfg_b != cal_asuskey3_128k){
        DEBUG ((EFI_D_INFO, "[ABL] read devcfg_b partition - Buffer = %a\n", Buffer));

        HandleFilter.RootDeviceType = NULL;
        HandleFilter.VolumeName = NULL;
        HandleFilter.PartitionLabel = L"devcfg_b";
		
        DEBUG((EFI_D_ERROR, "[ABL] +++ load devcfg_b partition() \n"));

        MaxHandles = sizeof(HandleInfoList)/sizeof(*HandleInfoList);

        Status = GetBlkIOHandles (BlkIOAttrib, &HandleFilter, HandleInfoList, &MaxHandles);

        if(Status == EFI_SUCCESS) {
            if (MaxHandles == 0) {
                DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): no partitions found.\r\n"));
                return EFI_NO_MEDIA;
            }else if (MaxHandles != 1) {
                // Unable to deterministically load from single partition
                DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): multiple partitions found.\r\n"));
                return EFI_LOAD_ERROR;
            }
        }else {
            DEBUG ((EFI_D_ERROR, "[ABL] %s: GetBlkIOHandles failed: %r\n", __func__, Status));
            return Status;
        }

        BlkIo = HandleInfoList[0].BlkIo;

        PartitionSize = (BlkIo->Media->LastBlock + 1) * BlkIo->Media->BlockSize;
        DEBUG ((EFI_D_ERROR, "[ABL] PartitionSize = %d \n",PartitionSize));
		
        DEBUG((EFI_D_ERROR, "[ABL] +++ write devcfg_b partition() \n"));
        Status = BlkIo->WriteBlocks(BlkIo,
                                BlkIo->Media->MediaId,
                                0,
                                PartitionSize,
                                Buffer);

        if(Status != EFI_SUCCESS) {
            DEBUG ((EFI_D_ERROR, "[ABL] (ERROR) write devcfg_b partition fail \n\n"));
            DEBUG ((EFI_D_ERROR, "[ABL] (ERROR) Status= %d \n\n",Status));
            return EFI_DEVICE_ERROR;
        }

        DEBUG((EFI_D_ERROR, "[ABL] --- write devcfg_b partition() \n\n"));
    }

    return Status;
}
// ASUS_BSP :add for update devcfg for app unlock device ---

// +++ ASUS_BSP : add for check TAR for main sku and CN sku
EFI_STATUS copy_fsg3_to_fsg(VOID)
{
	EFI_STATUS                   Status = EFI_SUCCESS;
	EFI_BLOCK_IO_PROTOCOL       *BlkIo;
	PartiSelectFilter            HandleFilter;
	HandleInfo                   HandleInfoList[1];
	STATIC UINT32                MaxHandles;
	STATIC UINT32                BlkIOAttrib = 0;
	VOID                        *Buffer;
	UINT32                       PartitionSize;

	//fsgCA
	EFI_GUID PartitionType_fsg3 = { 0x638FF8E2, 0x22C9, 0xE33B, { 0x8F, 0x5D, 0x0E, 0x81, 0x68, 0x6A, 0x68, 0xCF } };
	//fsg
	EFI_GUID PartitionType_fsg = { 0x638FF8E2, 0x22C9, 0xE33B, { 0x8F, 0x5D, 0x0E, 0x81, 0x68, 0x6A, 0x68, 0xCB } };

	DEBUG((EFI_D_ERROR, "[ABL] +++ read fsgCA partition() \n"));

	BlkIOAttrib = BLK_IO_SEL_PARTITIONED_MBR;
	BlkIOAttrib |= BLK_IO_SEL_PARTITIONED_GPT;
	BlkIOAttrib |= BLK_IO_SEL_MEDIA_TYPE_NON_REMOVABLE;
	BlkIOAttrib |= BLK_IO_SEL_MATCH_PARTITION_TYPE_GUID;

	HandleFilter.RootDeviceType = NULL;
	HandleFilter.PartitionType = &PartitionType_fsg3;
	HandleFilter.VolumeName = NULL;

	MaxHandles = sizeof(HandleInfoList)/sizeof(*HandleInfoList);

	Status = GetBlkIOHandles (BlkIOAttrib, &HandleFilter, HandleInfoList, &MaxHandles);

	if(Status == EFI_SUCCESS) {
		if (MaxHandles == 0) {
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): no partitions found.\r\n"));
			return EFI_NO_MEDIA;
		}else if (MaxHandles != 1) {
			// Unable to deterministically load from single partition
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): multiple partitions found.\r\n"));
			return EFI_LOAD_ERROR;
		}
	}else {
		DEBUG ((EFI_D_ERROR, "[ABL] %s: GetBlkIOHandles failed: %r\n", __func__, Status));
		return Status;
	}

	BlkIo = HandleInfoList[0].BlkIo;

	PartitionSize = (BlkIo->Media->LastBlock + 1) * BlkIo->Media->BlockSize;

	Buffer = AllocatePages(ALIGN_PAGES(PartitionSize, ALIGNMENT_MASK_4KB));

	if (Buffer == NULL) {
		DEBUG ((EFI_D_ERROR, "[ABL] (ERROR) read fsgCA partition - allocate buffer fail \n\n"));
		return EFI_DEVICE_ERROR;
	}

	if (Buffer != NULL) {
		BlkIo->ReadBlocks (BlkIo,
						BlkIo->Media->MediaId,
						0,
						PartitionSize,//ROUND_TO_PAGE(ImageSize, BlkIo->Media->BlockSize - 1),
						Buffer);
	}
	DEBUG ((EFI_D_INFO, "[ABL] read fsgCA partition - Buffer = %a\n", Buffer));

	HandleFilter.RootDeviceType = NULL;
	HandleFilter.PartitionType = &PartitionType_fsg;
	HandleFilter.VolumeName = NULL;

	DEBUG((EFI_D_ERROR, "[ABL] +++ write fsg partition() \n"));

	MaxHandles = sizeof(HandleInfoList)/sizeof(*HandleInfoList);

	Status = GetBlkIOHandles (BlkIOAttrib, &HandleFilter, HandleInfoList, &MaxHandles);

	if(Status == EFI_SUCCESS) {
		if (MaxHandles == 0) {
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): no partitions found.\r\n"));
			return EFI_NO_MEDIA;
		}else if (MaxHandles != 1) {
			// Unable to deterministically load from single partition
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): multiple partitions found.\r\n"));
			return EFI_LOAD_ERROR;
		}
	}else {
		DEBUG ((EFI_D_ERROR, "[ABL] %s: GetBlkIOHandles failed: %r\n", __func__, Status));
		return Status;
	}

	BlkIo = HandleInfoList[0].BlkIo;

	PartitionSize = (BlkIo->Media->LastBlock + 1) * BlkIo->Media->BlockSize;
	DEBUG ((EFI_D_ERROR, "[ABL] PartitionSize = %d \n",PartitionSize));

	Status = BlkIo->WriteBlocks(BlkIo,
							BlkIo->Media->MediaId,
							0,
							PartitionSize,
							Buffer);

	if(Status != EFI_SUCCESS) {
		DEBUG ((EFI_D_ERROR, "[ABL] (ERROR) write fsg partition fail \n\n"));
		DEBUG ((EFI_D_ERROR, "[ABL] (ERROR) Status= %d \n\n",Status));
		return EFI_DEVICE_ERROR;
	}

	DEBUG((EFI_D_ERROR, "[ABL] --- write fsg partition() \n\n"));
	return Status ;

}

EFI_STATUS copy_fsg2_to_fsg(VOID)
{
	EFI_STATUS                   Status = EFI_SUCCESS;
	EFI_BLOCK_IO_PROTOCOL       *BlkIo;
	PartiSelectFilter            HandleFilter;
	HandleInfo                   HandleInfoList[1];
	STATIC UINT32                MaxHandles;
	STATIC UINT32                BlkIOAttrib = 0;
	VOID                        *Buffer;
	UINT32                       PartitionSize;

	//fsgCA
	EFI_GUID PartitionType_fsg2 = { 0x638FF8E2, 0x22C9, 0xE33B, { 0x8F, 0x5D, 0x0E, 0x81, 0x68, 0x6A, 0x68, 0xCD } };
	//fsg
	EFI_GUID PartitionType_fsg = { 0x638FF8E2, 0x22C9, 0xE33B, { 0x8F, 0x5D, 0x0E, 0x81, 0x68, 0x6A, 0x68, 0xCB } };

	DEBUG((EFI_D_ERROR, "[ABL] +++ read fsgCA partition() \n"));

	BlkIOAttrib = BLK_IO_SEL_PARTITIONED_MBR;
	BlkIOAttrib |= BLK_IO_SEL_PARTITIONED_GPT;
	BlkIOAttrib |= BLK_IO_SEL_MEDIA_TYPE_NON_REMOVABLE;
	BlkIOAttrib |= BLK_IO_SEL_MATCH_PARTITION_TYPE_GUID;

	HandleFilter.RootDeviceType = NULL;
	HandleFilter.PartitionType = &PartitionType_fsg2;
	HandleFilter.VolumeName = NULL;

	MaxHandles = sizeof(HandleInfoList)/sizeof(*HandleInfoList);

	Status = GetBlkIOHandles (BlkIOAttrib, &HandleFilter, HandleInfoList, &MaxHandles);

	if(Status == EFI_SUCCESS) {
		if (MaxHandles == 0) {
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): no partitions found.\r\n"));
			return EFI_NO_MEDIA;
		}else if (MaxHandles != 1) {
			// Unable to deterministically load from single partition
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): multiple partitions found.\r\n"));
			return EFI_LOAD_ERROR;
		}
	}else {
		DEBUG ((EFI_D_ERROR, "[ABL] %s: GetBlkIOHandles failed: %r\n", __func__, Status));
		return Status;
	}

	BlkIo = HandleInfoList[0].BlkIo;

	PartitionSize = (BlkIo->Media->LastBlock + 1) * BlkIo->Media->BlockSize;

	Buffer = AllocatePages(ALIGN_PAGES(PartitionSize, ALIGNMENT_MASK_4KB));

	if (Buffer == NULL) {
		DEBUG ((EFI_D_ERROR, "[ABL] (ERROR) read fsgCA partition - allocate buffer fail \n\n"));
		return EFI_DEVICE_ERROR;
	}

	if (Buffer != NULL) {
		BlkIo->ReadBlocks (BlkIo,
						BlkIo->Media->MediaId,
						0,
						PartitionSize,//ROUND_TO_PAGE(ImageSize, BlkIo->Media->BlockSize - 1),
						Buffer);
	}
	DEBUG ((EFI_D_INFO, "[ABL] read fsgCA partition - Buffer = %a\n", Buffer));

	HandleFilter.RootDeviceType = NULL;
	HandleFilter.PartitionType = &PartitionType_fsg;
	HandleFilter.VolumeName = NULL;

	DEBUG((EFI_D_ERROR, "[ABL] +++ write fsg partition() \n"));

	MaxHandles = sizeof(HandleInfoList)/sizeof(*HandleInfoList);

	Status = GetBlkIOHandles (BlkIOAttrib, &HandleFilter, HandleInfoList, &MaxHandles);

	if(Status == EFI_SUCCESS) {
		if (MaxHandles == 0) {
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): no partitions found.\r\n"));
			return EFI_NO_MEDIA;
		}else if (MaxHandles != 1) {
			// Unable to deterministically load from single partition
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): multiple partitions found.\r\n"));
			return EFI_LOAD_ERROR;
		}
	}else {
		DEBUG ((EFI_D_ERROR, "[ABL] %s: GetBlkIOHandles failed: %r\n", __func__, Status));
		return Status;
	}

	BlkIo = HandleInfoList[0].BlkIo;

	PartitionSize = (BlkIo->Media->LastBlock + 1) * BlkIo->Media->BlockSize;
	DEBUG ((EFI_D_ERROR, "[ABL] PartitionSize = %d \n",PartitionSize));

	Status = BlkIo->WriteBlocks(BlkIo,
							BlkIo->Media->MediaId,
							0,
							PartitionSize,
							Buffer);

	if(Status != EFI_SUCCESS) {
		DEBUG ((EFI_D_ERROR, "[ABL] (ERROR) write fsg partition fail \n\n"));
		DEBUG ((EFI_D_ERROR, "[ABL] (ERROR) Status= %d \n\n",Status));
		return EFI_DEVICE_ERROR;
	}

	DEBUG((EFI_D_ERROR, "[ABL] --- write fsg partition() \n\n"));
	return Status ;

}

EFI_STATUS erase_fsg(VOID)
{
	EFI_STATUS                   Status = EFI_SUCCESS;
	EFI_BLOCK_IO_PROTOCOL       *BlkIo;
	PartiSelectFilter            HandleFilter;
	HandleInfo                   HandleInfoList[1];
	STATIC UINT32                MaxHandles;
	STATIC UINT32                BlkIOAttrib = 0;
	VOID                        *Buffer;
	UINT32                       BufferSize = ALIGNMENT_MASK_4KB;

	//fsg
	EFI_GUID PartitionType = { 0x638FF8E2, 0x22C9, 0xE33B, { 0x8F, 0x5D, 0x0E, 0x81, 0x68, 0x6A, 0x68, 0xCB } };

	DEBUG((EFI_D_ERROR, "[ABL] +++ erase_fsg() \n"));

	BlkIOAttrib = BLK_IO_SEL_PARTITIONED_MBR;
	BlkIOAttrib |= BLK_IO_SEL_PARTITIONED_GPT;
	BlkIOAttrib |= BLK_IO_SEL_MEDIA_TYPE_NON_REMOVABLE;
	BlkIOAttrib |= BLK_IO_SEL_MATCH_PARTITION_TYPE_GUID;

	HandleFilter.RootDeviceType = NULL;
	HandleFilter.PartitionType = &PartitionType;
	HandleFilter.VolumeName = NULL;

	MaxHandles = sizeof(HandleInfoList)/sizeof(*HandleInfoList);

	Status = GetBlkIOHandles (BlkIOAttrib, &HandleFilter, HandleInfoList, &MaxHandles);

	if(Status == EFI_SUCCESS) {
		if (MaxHandles == 0) {
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): no partitions found.\r\n"));
			return EFI_NO_MEDIA;
		}else if (MaxHandles != 1) {
			// Unable to deterministically load from single partition
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): multiple partitions found.\r\n"));
			return EFI_LOAD_ERROR;
		}
	}else {
		DEBUG ((EFI_D_ERROR, "[ABL] %s: GetBlkIOHandles failed: %r\n", __func__, Status));
		return Status;
	}

	BlkIo = HandleInfoList[0].BlkIo;

	Buffer = AllocatePages(ALIGN_PAGES(BufferSize, ALIGNMENT_MASK_4KB));

	if (Buffer == NULL) {
		DEBUG ((EFI_D_ERROR, "[ABL] (ERROR) erase_fsg_partition - allocate buffer fail \n\n"));
		return EFI_DEVICE_ERROR ;
	}

	if (Buffer != NULL) {
		BlkIo->ReadBlocks (BlkIo,
						BlkIo->Media->MediaId,
						0,
						BufferSize,//ROUND_TO_PAGE(ImageSize, BlkIo->Media->BlockSize - 1),
						Buffer);
	}

	memset(Buffer,0,BufferSize);

	Status = BlkIo->WriteBlocks(BlkIo,
							BlkIo->Media->MediaId,
							0,
							BufferSize,
							Buffer);

	if (Status != EFI_SUCCESS) {
		DEBUG ((EFI_D_ERROR, "[ABL] (ERROR) erase_fsg_partition - erase_fsg_partition fail \n\n"));
		return EFI_DEVICE_ERROR ;
	}

	DEBUG((EFI_D_ERROR, "[ABL] --- erase_fsg() \n\n"));
	return Status ;
}

BOOLEAN need2cpFfg=FALSE;
EFI_STATUS check_tar_step(VOID)
{
	EFI_STATUS                   Status = EFI_SUCCESS;

	DEBUG((EFI_D_ERROR, "[ABL] +++ Check_TAR_Step \n"));
	
	if (need2cpFfg == FALSE){
           DEBUG ((EFI_D_INFO, "[ABL] do nothing.\n"));
           return EFI_SUCCESS;
	}

	/*
	fsg   => RF_ID_0: CN
	fsg2  => RF_ID_1: WW
	fsg3  => RF_ID_2: WW(US)
	*/
	if (Get_RF_ID() == RF_ID_2) {
		DEBUG ((EFI_D_INFO, "[ABL] Check_TAR_Step - RF_ID_0 \n"));
		/*
		DEBUG ((EFI_D_INFO, "[ABL] Check_TAR_Step - start to erase fsg \n"));
		erase_fsg();
		DEBUG ((EFI_D_INFO, "[ABL] Check_TAR_Step - end to erase fsg \n"));
		*/
		DEBUG ((EFI_D_INFO, "[ABL] Check_TAR_Step - start to copy fsg3 to fsg \n"));
		copy_fsg3_to_fsg();
		DEBUG ((EFI_D_INFO, "[ABL] Check_TAR_Step - end to copy fsg3 to fsg \n"));
	} else if (Get_RF_ID() == RF_ID_1) {
		DEBUG ((EFI_D_INFO, "[ABL] Check_TAR_Step - RF_ID_1 \n"));
		DEBUG ((EFI_D_INFO, "[ABL] Check_TAR_Step - start to copy fsg2 to fsg \n"));
		copy_fsg2_to_fsg();
		DEBUG ((EFI_D_INFO, "[ABL] Check_TAR_Step - end to copy fsg2 to fsg \n"));
	} else {
		DEBUG ((EFI_D_INFO, "[ABL] Check_TAR_Step : no need to change fsg. \n"));
	}

	DEBUG((EFI_D_ERROR, "[ABL] --- Check_TAR_Step \n\n"));
	return Status ;
}
// --- ASUS_BSP : add for check TAR for main sku and CN sku

// +++ ASUS_BSP : add for re-unTAR process
EFI_STATUS erase_modemst1(VOID)
{
	EFI_STATUS                   Status = EFI_SUCCESS;
	EFI_BLOCK_IO_PROTOCOL       *BlkIo;
	PartiSelectFilter            HandleFilter;
	HandleInfo                   HandleInfoList[1];
	STATIC UINT32                MaxHandles;
	STATIC UINT32                BlkIOAttrib = 0;
	VOID                        *Buffer;
	UINT32                       BufferSize = ALIGNMENT_MASK_4KB;

	//modemst1
	EFI_GUID PartitionType = { 0xEBBEADAF, 0x22C9, 0xE33B, { 0x8F, 0x5D, 0x0E, 0x81, 0x68, 0x6A, 0x68, 0xCB } };

	DEBUG((EFI_D_ERROR, "[ABL] +++ erase_modemst1() \n"));

	BlkIOAttrib = BLK_IO_SEL_PARTITIONED_MBR;
	BlkIOAttrib |= BLK_IO_SEL_PARTITIONED_GPT;
	BlkIOAttrib |= BLK_IO_SEL_MEDIA_TYPE_NON_REMOVABLE;
	BlkIOAttrib |= BLK_IO_SEL_MATCH_PARTITION_TYPE_GUID;

	HandleFilter.RootDeviceType = NULL;
	HandleFilter.PartitionType = &PartitionType;
	HandleFilter.VolumeName = NULL;

	MaxHandles = sizeof(HandleInfoList)/sizeof(*HandleInfoList);

	Status = GetBlkIOHandles (BlkIOAttrib, &HandleFilter, HandleInfoList, &MaxHandles);

	if(Status == EFI_SUCCESS) {
		if (MaxHandles == 0) {
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): no partitions found.\r\n"));
			return EFI_NO_MEDIA;
		}else if (MaxHandles != 1) {
			// Unable to deterministically load from single partition
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): multiple partitions found.\r\n"));
			return EFI_LOAD_ERROR;
		}
	}else {
		DEBUG ((EFI_D_ERROR, "[ABL] %s: GetBlkIOHandles failed: %r\n", __func__, Status));
		return Status;
	}

	BlkIo = HandleInfoList[0].BlkIo;

	Buffer = AllocatePages(ALIGN_PAGES(BufferSize, ALIGNMENT_MASK_4KB));

	if (Buffer == NULL) {
		DEBUG ((EFI_D_ERROR, "[ABL] (ERROR) erase_modemst1_partition - allocate buffer fail \n\n"));
		return EFI_DEVICE_ERROR ;
	}

	if (Buffer != NULL) {
		BlkIo->ReadBlocks (BlkIo,
						BlkIo->Media->MediaId,
						0,
						BufferSize,//ROUND_TO_PAGE(ImageSize, BlkIo->Media->BlockSize - 1),
						Buffer);
	}

	memset(Buffer,0,BufferSize);

	Status = BlkIo->WriteBlocks(BlkIo,
							BlkIo->Media->MediaId,
							0,
							BufferSize,
							Buffer);

	if (Status != EFI_SUCCESS) {
		DEBUG ((EFI_D_ERROR, "[ABL] (ERROR) erase_modemst1_partition - erase_modemst1_partition fail \n\n"));
		return EFI_DEVICE_ERROR ;
	}

	DEBUG((EFI_D_ERROR, "[ABL] --- erase_modemst1() \n\n"));
	return Status ;
}

EFI_STATUS erase_modemst2(VOID)
{
	EFI_STATUS                   Status = EFI_SUCCESS;
	EFI_BLOCK_IO_PROTOCOL       *BlkIo;
	PartiSelectFilter            HandleFilter;
	HandleInfo                   HandleInfoList[1];
	STATIC UINT32                MaxHandles;
	STATIC UINT32                BlkIOAttrib = 0;
	VOID                        *Buffer;
	UINT32                       BufferSize = ALIGNMENT_MASK_4KB;

	//modemst2
	EFI_GUID PartitionType = { 0x0A288B1F, 0x22C9, 0xE33B, { 0x8F, 0x5D, 0x0E, 0x81, 0x68, 0x6A, 0x68, 0xCB } };

	DEBUG((EFI_D_ERROR, "[ABL] +++ erase_modemst2() \n"));

	BlkIOAttrib = BLK_IO_SEL_PARTITIONED_MBR;
	BlkIOAttrib |= BLK_IO_SEL_PARTITIONED_GPT;
	BlkIOAttrib |= BLK_IO_SEL_MEDIA_TYPE_NON_REMOVABLE;
	BlkIOAttrib |= BLK_IO_SEL_MATCH_PARTITION_TYPE_GUID;

	HandleFilter.RootDeviceType = NULL;
	HandleFilter.PartitionType = &PartitionType;
	HandleFilter.VolumeName = NULL;

	MaxHandles = sizeof(HandleInfoList)/sizeof(*HandleInfoList);

	Status = GetBlkIOHandles (BlkIOAttrib, &HandleFilter, HandleInfoList, &MaxHandles);

	if(Status == EFI_SUCCESS) {
		if (MaxHandles == 0) {
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): no partitions found.\r\n"));
			return EFI_NO_MEDIA;
		}else if (MaxHandles != 1) {
			// Unable to deterministically load from single partition
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): multiple partitions found.\r\n"));
			return EFI_LOAD_ERROR;
		}
	}else {
		DEBUG ((EFI_D_ERROR, "[ABL] %s: GetBlkIOHandles failed: %r\n", __func__, Status));
		return Status;
	}

	BlkIo = HandleInfoList[0].BlkIo;

	Buffer = AllocatePages(ALIGN_PAGES(BufferSize, ALIGNMENT_MASK_4KB));

	if (Buffer == NULL) {
		DEBUG ((EFI_D_ERROR, "[ABL] (ERROR) erase_modemst2_partition - allocate buffer fail \n\n"));
		return EFI_DEVICE_ERROR ;
	}

	if (Buffer != NULL) {
		BlkIo->ReadBlocks (BlkIo,
						BlkIo->Media->MediaId,
						0,
						BufferSize,//ROUND_TO_PAGE(ImageSize, BlkIo->Media->BlockSize - 1),
						Buffer);
	}

	memset(Buffer,0,BufferSize);

	Status = BlkIo->WriteBlocks(BlkIo,
							BlkIo->Media->MediaId,
							0,
							BufferSize,
							Buffer);

	if (Status != EFI_SUCCESS) {
		DEBUG ((EFI_D_ERROR, "[ABL] (ERROR) erase_modemst2_partition - erase_modemst2_partition fail \n\n"));
		return EFI_DEVICE_ERROR ;
	}

	DEBUG((EFI_D_ERROR, "[ABL] --- erase_modemst2() \n\n"));
	return Status ;
}

EFI_STATUS prepare_untar_step(VOID)
{
	EFI_STATUS                   Status = EFI_SUCCESS;
	EFI_BLOCK_IO_PROTOCOL       *BlkIo;
	PartiSelectFilter            HandleFilter;
	HandleInfo                   HandleInfoList[1];
	STATIC UINT32                MaxHandles;
	STATIC UINT32                BlkIOAttrib = 0;
	VOID                        *Buffer;
	UINT32                       BufferSize = ALIGNMENT_MASK_4KB;

	//fsc
	EFI_GUID PartitionType = { 0x57B90A16, 0x22C9, 0xE33B, { 0x8F, 0x5D, 0x0E, 0x81, 0x68, 0x6A, 0x68, 0xCB } };
	const char magic_num[8] = {0x55,0xaa,0x55,0xaa,0x55,0xaa,0x55,0xaa};

	DEBUG((EFI_D_ERROR, "[ABL] +++ Prepare_unTAR_Step \n"));

	BlkIOAttrib = BLK_IO_SEL_PARTITIONED_MBR;
	BlkIOAttrib |= BLK_IO_SEL_PARTITIONED_GPT;
	BlkIOAttrib |= BLK_IO_SEL_MEDIA_TYPE_NON_REMOVABLE;
	BlkIOAttrib |= BLK_IO_SEL_MATCH_PARTITION_TYPE_GUID;

	HandleFilter.RootDeviceType = NULL;
	HandleFilter.PartitionType = &PartitionType;
	HandleFilter.VolumeName = NULL;

	MaxHandles = sizeof(HandleInfoList)/sizeof(*HandleInfoList);

	Status = GetBlkIOHandles (BlkIOAttrib, &HandleFilter, HandleInfoList, &MaxHandles);

	if(Status == EFI_SUCCESS) {
		if (MaxHandles == 0) {
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): no partitions found.\r\n"));
			return EFI_NO_MEDIA;
		}else if (MaxHandles != 1) {
			// Unable to deterministically load from single partition
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): multiple partitions found.\r\n"));
			return EFI_LOAD_ERROR;
		}
	}else {
		DEBUG ((EFI_D_ERROR, "[ABL] %s: GetBlkIOHandles failed: %r\n", __func__, Status));
		return Status;
	}

	BlkIo = HandleInfoList[0].BlkIo;
	Buffer = AllocatePages(ALIGN_PAGES(BufferSize, ALIGNMENT_MASK_4KB));
	//DEBUG ((EFI_D_ERROR, "ALIGN_PAGES Image size: %u Bytes\n", BufferSize));

	if (Buffer == NULL) {
		DEBUG ((EFI_D_ERROR, "[ABL] (ERROR) Prepare_unTAR_Step - allocate buffer fail \n\n"));
		return EFI_DEVICE_ERROR;
	}

	if (Buffer != NULL) {
		BlkIo->ReadBlocks (BlkIo,
						BlkIo->Media->MediaId,
						0,
						BufferSize,//ROUND_TO_PAGE(ImageSize, BlkIo->Media->BlockSize - 1),
						Buffer);
	}

	DEBUG ((EFI_D_INFO, "[ABL] Prepare_unTAR_Step - Buffer = %a\n", Buffer));

	if(!strncmp(magic_num, Buffer, 8)){
		need2cpFfg=TRUE;
		DEBUG ((EFI_D_INFO, "[ABL] check_unTAR_Step - start to erase\n"));
		erase_modemst1();
		erase_modemst2();
		DEBUG ((EFI_D_INFO, "[ABL] check_unTAR_Step - end to erase\n"));
	}else
	{
		DEBUG ((EFI_D_INFO, "[ABL] check_unTAR_Step : do nothing \n"));
	}

	DEBUG((EFI_D_ERROR, "[ABL] --- Prepare_unTAR_Step \n\n"));
	return Status ;
}
// --- ASUS_BSP : add for re-unTAR process

// +++ ASUS_BSP : add for get pmic reg value
EFI_STATUS GetPmicReg
(
  IN UINT32 PmicDeviceIndex,
  IN UINT32 RegAddress,
  OUT UINT8 *RegValue
)
{
	EFI_QCOM_PMIC_GPIO_PROTOCOL *PmicGpioProtocol = NULL;
	EFI_STATUS Status = EFI_SUCCESS;
	UINT8 value = 0;

	DEBUG((EFI_D_INFO, "[ABL] +++ GetPmicReg()\n"));

	DEBUG((EFI_D_ERROR, "[ABL] GetPmicReg: Status : %r, PmicDeviceIndex=%x, RegAddress=%x\n", Status, PmicDeviceIndex, RegAddress));

	if (EFI_SUCCESS != gBS->LocateProtocol(&gQcomPmicGpioProtocolGuid, NULL, (void **)&PmicGpioProtocol))
	{
		DEBUG((EFI_D_ERROR, "[ABL] (ERROR) GetPmicReg: Locate PmicGpio protocol failed: %r\n", Status));
	}

	if ((EFI_SUCCESS == Status) && (NULL != PmicGpioProtocol))
	{
		//call XBL : EFI_PmGpioGetPmicReg()
		Status = PmicGpioProtocol->GetPmicReg(PmicDeviceIndex, RegAddress, &value);
		if (EFI_SUCCESS != Status)
		{
			DEBUG((EFI_D_ERROR, "[ABL] (ERROR) GetPmicReg fail : %r\n", Status));
			return Status ;
		}

		DEBUG((EFI_D_INFO, "[ABL] GetPmicReg :  value : %x\n", value));
		*RegValue = value;
	}

	DEBUG((EFI_D_INFO, "[ABL] --- GetPmicReg()\n\n"));
	return Status;
}
// --- ASUS_BSP : add for get pmic reg value

// +++ ASUS_BSP : add for write pmic reg value
EFI_STATUS WritePmicReg
(
  IN UINT32 PmicDeviceIndex,
  IN UINT32 RegAddress,
  IN UINT32 RegValue
)
{
	EFI_QCOM_PMIC_GPIO_PROTOCOL *PmicGpioProtocol = NULL;
	EFI_STATUS Status = EFI_SUCCESS;

	DEBUG((EFI_D_INFO, "[ABL] +++ WritePmicReg()\n"));

	DEBUG((EFI_D_ERROR, "[ABL] WritePmicReg : Status : %r, PmicDeviceIndex=%x, RegAddress=%x, RegValue=%x\n", Status, PmicDeviceIndex, RegAddress, RegValue));

	if (EFI_SUCCESS != gBS->LocateProtocol(&gQcomPmicGpioProtocolGuid, NULL, (void **)&PmicGpioProtocol))
	{
		DEBUG((EFI_D_ERROR, "[ABL] (ERROR) WritePmicReg: Locate PmicGpio protocol failed: %r\n", Status));
	}

	if ((EFI_SUCCESS == Status) && (NULL != PmicGpioProtocol))
	{
		//call XBL : EFI_PmGpioWritePmicReg
		Status = PmicGpioProtocol->WritePmicReg(PmicDeviceIndex, RegAddress, RegValue);
		if (EFI_SUCCESS != Status)
		{
			DEBUG((EFI_D_ERROR, "[ABL] (ERROR) WritePmicReg fail : %r\n", Status));
			return Status ;
		}

	}
	DEBUG((EFI_D_INFO, "[ABL] --- WritePmicReg()\n\n"));
	return Status;
}
// --- ASUS_BSP : add for write pmic reg value

// +++ ASUS_BSP : add for check CRC
STATIC INT32 Lun = NO_LUN;
STATIC BOOLEAN LunSet;

STATIC EFI_STATUS
PartitionGetInfo ( IN CHAR16 *PartitionName,
						OUT EFI_BLOCK_IO_PROTOCOL **BlockIo,
						OUT EFI_HANDLE **Handle)
{
	EFI_STATUS Status;
	EFI_PARTITION_ENTRY *PartEntry;
	UINT16 i;
	UINT32 j;
	/* By default the LunStart and LunEnd would point to '0' and max value */
	UINT32 LunStart = 0;
	UINT32 LunEnd = GetMaxLuns ();

	/* If Lun is set in the Handle flash command then find the block io for that
	* lun */
	if (LunSet) {
	LunStart = Lun;
	LunEnd = Lun + 1;
	}
	for (i = LunStart; i < LunEnd; i++) {
		for (j = 0; j < Ptable[i].MaxHandles; j++) {
			Status = gBS->HandleProtocol (Ptable[i].HandleInfoList[j].Handle,
											&gEfiPartitionRecordGuid, (VOID **)&PartEntry);
			if (EFI_ERROR (Status)) {
			continue;
			}
			if (!(StrCmp (PartitionName, PartEntry->PartitionName))) {
				*BlockIo = Ptable[i].HandleInfoList[j].BlkIo;
				*Handle = Ptable[i].HandleInfoList[j].Handle;
				return Status;
			}
		}
	}
	DEBUG ((EFI_D_ERROR, "Partition not found : %s\n", PartitionName));
	return EFI_NOT_FOUND;
}

/*+++
Routine Description:

  The CalculateCrc32 routine.

Arguments:

  Data        - The buffer contaning the data to be processed
  DataSize    - The size of data to be processed
  CrcOut      - A pointer to the caller allocated UINT32 that on
                contains the CRC32 checksum of Data

Returns:

  EFI_SUCCESS               - Calculation is successful.
  EFI_INVALID_PARAMETER     - Data / CrcOut = NULL, or DataSize = 0
---*/
EFI_STATUS
CalculateCrc32_super (
  IN  UINT8                             *Data,
  IN  UINTN                             DataSize,
  IN OUT UINT32                         *CrcOut
)
{
	UINT32  Crc;
	UINTN   Index;
	UINT8   *Ptr;

	if (DataSize == 0) {
		DEBUG ((EFI_D_INFO, "DataSize = 0 \n"));
	}
	if (Data == NULL) {
		DEBUG ((EFI_D_INFO, "Data = NULL \n"));
	}
	if (CrcOut == NULL) {
		DEBUG ((EFI_D_INFO, "CrcOut = NULL \n"));
	}

	DEBUG ((EFI_D_INFO, "DataSize =%u \n",DataSize));

	if ((DataSize == 0) || (Data == NULL) || (CrcOut == NULL)) {
		DEBUG ((EFI_D_INFO, "EFI_INVALID_PARAMETER\n"));
		return EFI_INVALID_PARAMETER;
	}

	Crc = *CrcOut;
	//DEBUG ((EFI_D_INFO, "before Crc =%x \n",Crc));

	for (Index = 0, Ptr = Data; Index < DataSize; Index++, Ptr++) {
		Crc = (Crc >> 8) ^ mCrcTable[(UINT8) Crc ^ *Ptr];
		//DEBUG ((EFI_D_INFO, "Index = %d ,  Crc = %x, Ptr = %x\n", Index,Crc,*Ptr));
	}

	*CrcOut = Crc;
	//DEBUG ((EFI_D_INFO, "after Crc =%x \n",Crc));

	return EFI_SUCCESS;
}

/*+++
Routine Description:

  The CalculateCrc32 routine.

Arguments:

  Data        - The buffer contaning the data to be processed
  DataSize    - The size of data to be processed
  CrcOut      - A pointer to the caller allocated UINT32 that on
                contains the CRC32 checksum of Data

Returns:

  EFI_SUCCESS               - Calculation is successful.
  EFI_INVALID_PARAMETER     - Data / CrcOut = NULL, or DataSize = 0
---*/

EFI_STATUS
CalculateCrc32 (
  IN  UINT8                             *Data,
  IN  UINTN                             DataSize,
  IN OUT UINT32                         *CrcOut
)
{
	UINT32  Crc;
	UINTN   Index;
	UINT8   *Ptr;

	if (DataSize == 0) {
		DEBUG ((EFI_D_INFO, "DataSize = 0 \n"));
	}
	if (Data == NULL) {
		DEBUG ((EFI_D_INFO, "Data = NULL \n"));
	}
	if (CrcOut == NULL) {
		DEBUG ((EFI_D_INFO, "CrcOut = NULL \n"));
	}

	DEBUG ((EFI_D_INFO, "DataSize =%u \n",DataSize));

	if ((DataSize == 0) || (Data == NULL) || (CrcOut == NULL)) {
		DEBUG ((EFI_D_INFO, "EFI_INVALID_PARAMETER\n"));
		return EFI_INVALID_PARAMETER;
	}

	Crc = 0xffffffff;
	for (Index = 0, Ptr = Data; Index < DataSize; Index++, Ptr++) {
		Crc = (Crc >> 8) ^ mCrcTable[(UINT8) Crc ^ *Ptr];
		//DEBUG ((EFI_D_INFO, "Index = %d ,  Crc = %x, Ptr = %x\n", Index,Crc,*Ptr));
	}

	*CrcOut = Crc ^ 0xffffffff;

	return EFI_SUCCESS;
}

unsigned long AsusCalculatePtCrc32(CHAR16 *Pname)
{
	EFI_STATUS                   Status;
	EFI_BLOCK_IO_PROTOCOL       *BlkIo;
	EFI_HANDLE                  *Handle = NULL;
	UINT32                       ImageSize = 0;
	UINT64                       ImageSize_64 =0;
	UINT32                       super_part = 0;
	UINT32                       ipart = 0;
	UINT32                       SIZE_256M = 268435456;//256*1024*1024
	//UINT32                       xbl_config_ImageSize = 237568;//232*1024
	//UINT32                       rtice_ImageSize = 524288;//512*1024
	UINT32                       apdp_ImageSize = 262144;//256*1024
	UINT32                       offset_part = 0;
	UINT32                       temp_crc = 0xFFFFFFFF;
	VOID                         *Buffer;

	DEBUG(( EFI_D_INFO, "[ABL] Pname = %s\r\n",Pname));

	Status = PartitionGetInfo (Pname, &BlkIo, &Handle);
	if (Status != EFI_SUCCESS){
		DEBUG((EFI_D_ERROR, "[ABL] (ERROR) Get PartitionGetInfo failed : %r \n", Status));
		return Status;
	}

	/* Check image will fit on device */
	ImageSize = (BlkIo->Media->LastBlock + 1) * BlkIo->Media->BlockSize;

	DEBUG ((EFI_D_INFO, "[ABL] Image partition size: %u Bytes\n", ImageSize));
	if (!StrnCmp(Pname, L"super", StrLen(L"super"))) {
		ImageSize_64 = (UINT64)(BlkIo->Media->LastBlock + 1) * BlkIo->Media->BlockSize;
		DEBUG ((EFI_D_INFO, "[ABL] super Image partition size: %u Bytes\n", ImageSize_64));
		super_part = ImageSize_64/SIZE_256M;

		//DEBUG ((EFI_D_INFO, "[ABL] super_part = %u Bytes\n", super_part));

		for(ipart = 0; ipart<super_part; ipart++){
			Buffer = AllocatePages(ALIGN_PAGES(SIZE_256M, ALIGNMENT_MASK_4KB));
			//DEBUG ((EFI_D_INFO, "[ABL] ipart = %u \n", ipart));
			offset_part = SIZE_256M/ALIGNMENT_MASK_4KB*ipart;
			//DEBUG ((EFI_D_INFO, "[ABL] offset_part = %u \n", offset_part));
			if (Buffer != NULL) {
				BlkIo->ReadBlocks (
					BlkIo,
					BlkIo->Media->MediaId,
					offset_part,
					ROUND_TO_PAGE(SIZE_256M, BlkIo->Media->BlockSize - 1),
					Buffer
				);
			}
			//DEBUG ((EFI_D_INFO, "[ABL] before temp_crc = %x \n", temp_crc));
			Status = CalculateCrc32_super(Buffer, SIZE_256M, &temp_crc);

			if (Status != EFI_SUCCESS)
				DEBUG((EFI_D_ERROR, "[ABL] (ERROR) Get CalculateCrc32_super failed : %r \n", Status));

			//DEBUG ((EFI_D_INFO, "[ABL] after temp_crc = %x \n", temp_crc));
			//DEBUG ((EFI_D_INFO, "[ABL] after temp_crc with ff = %x \n", (temp_crc ^ 0xffffffff)));
			FreePages(Buffer, ALIGN_PAGES(SIZE_256M, ALIGNMENT_MASK_4KB));
		}
		temp_crc = temp_crc ^ 0xffffffff;
	}
	else if (!StrnCmp(Pname, L"apdp", StrLen(L"apdp"))){
		Buffer = AllocatePages(ALIGN_PAGES(apdp_ImageSize, ALIGNMENT_MASK_4KB));

		if (Buffer != NULL) {
			BlkIo->ReadBlocks (
				BlkIo,
				BlkIo->Media->MediaId,
				0,
				ROUND_TO_PAGE(apdp_ImageSize, BlkIo->Media->BlockSize - 1),
				Buffer
			);
		}
		Status = CalculateCrc32(Buffer,apdp_ImageSize,&temp_crc);
		if (Status != EFI_SUCCESS)
			DEBUG((EFI_D_ERROR, "[ABL] (ERROR) Get CalculateCrc32_apdp failed : %r \n", Status));
	}
	else if (!StrnCmp(Pname, L"apdpb", StrLen(L"apdpb"))){
		Buffer = AllocatePages(ALIGN_PAGES(apdp_ImageSize, ALIGNMENT_MASK_4KB));

		if (Buffer != NULL) {
			BlkIo->ReadBlocks (
				BlkIo,
				BlkIo->Media->MediaId,
				0,
				ROUND_TO_PAGE(apdp_ImageSize, BlkIo->Media->BlockSize - 1),
				Buffer
			);
		}
		Status = CalculateCrc32(Buffer,apdp_ImageSize,&temp_crc);
		if (Status != EFI_SUCCESS)
			DEBUG((EFI_D_ERROR, "[ABL] (ERROR) Get CalculateCrc32_apdpb failed : %r \n", Status));
	}
/*
 * To comment out this code because abl could get right image size from partiton table.
 * 
	else if (!StrnCmp(Pname, L"xbl_config", StrLen(L"xbl_config"))){
		Buffer = AllocatePages(ALIGN_PAGES(xbl_config_ImageSize, ALIGNMENT_MASK_4KB));

		if (Buffer != NULL) {
			BlkIo->ReadBlocks (
				BlkIo,
				BlkIo->Media->MediaId,
				0,
				ROUND_TO_PAGE(xbl_config_ImageSize, BlkIo->Media->BlockSize - 1),
				Buffer
			);
		}
		Status = CalculateCrc32(Buffer,xbl_config_ImageSize,&temp_crc);
		if (Status != EFI_SUCCESS)
			DEBUG((EFI_D_ERROR, "[ABL] (ERROR) Get CalculateCrc32_xbl_config failed : %r \n", Status));
	}
	else if (!StrnCmp(Pname, L"rtice", StrLen(L"rtice"))){
		Buffer = AllocatePages(ALIGN_PAGES(rtice_ImageSize, ALIGNMENT_MASK_4KB));

		if (Buffer != NULL) {
			BlkIo->ReadBlocks (
				BlkIo,
				BlkIo->Media->MediaId,
				0,
				ROUND_TO_PAGE(rtice_ImageSize, BlkIo->Media->BlockSize - 1),
				Buffer
			);
		}
		Status = CalculateCrc32(Buffer,rtice_ImageSize,&temp_crc);
		if (Status != EFI_SUCCESS)
			DEBUG((EFI_D_ERROR, "[ABL] (ERROR) Get CalculateCrc32_rtice failed : %r \n", Status));

	}
*/
	else{
		Buffer = AllocatePages(ALIGN_PAGES(ImageSize, ALIGNMENT_MASK_4KB));

		if (Buffer != NULL) {
			BlkIo->ReadBlocks (
				BlkIo,
				BlkIo->Media->MediaId,
				0,
				ROUND_TO_PAGE(ImageSize, BlkIo->Media->BlockSize - 1),
				Buffer
			);
		}
		Status = CalculateCrc32(Buffer,ImageSize,&temp_crc);
		if (Status != EFI_SUCCESS)
			DEBUG((EFI_D_ERROR, "[ABL] (ERROR) Get CalculateCrc32 failed : %r \n", Status));
	}
	DEBUG ((EFI_D_INFO, "[ABL] temp_crc = %x\n", temp_crc));
	return temp_crc;
}

unsigned long AsusCalculatePtCrc32Size(CHAR16 *Pname, UINT32 ImageSize)
{
	EFI_STATUS                   Status;
	EFI_BLOCK_IO_PROTOCOL       *BlkIo;
	EFI_HANDLE                  *Handle = NULL;
	UINT32                       temp_crc = 0xFFFFFFFF;
	VOID                         *Buffer;

	DEBUG(( EFI_D_INFO, "[ABL] Pname = %s\r\n",Pname));

	Status = PartitionGetInfo (Pname, &BlkIo, &Handle);
	if (Status != EFI_SUCCESS){
		DEBUG((EFI_D_ERROR, "[ABL] (ERROR) Get PartitionGetInfo failed : %r \n", Status));
		return Status;
	}

	Buffer = AllocatePages(ALIGN_PAGES(ImageSize, ALIGNMENT_MASK_4KB));

	if (Buffer != NULL) {
		BlkIo->ReadBlocks (
			BlkIo,
			BlkIo->Media->MediaId,
			0,
			ROUND_TO_PAGE(ImageSize, BlkIo->Media->BlockSize - 1),
			Buffer
		);
	}
	
	Status = CalculateCrc32(Buffer,ImageSize,&temp_crc);
	if (Status != EFI_SUCCESS)
		DEBUG((EFI_D_ERROR, "[ABL] (ERROR) Get CalculateCrc32 failed : %r \n", Status));

	DEBUG ((EFI_D_INFO, "[ABL] temp_crc = %x\n", temp_crc));
	return temp_crc;
}
// --- ASUS_BSP : add for check CRC

// +++ ASUS_BSP : add for enter shipping mode
EFI_STATUS EnterShippingMode(void)
{
	EFI_STATUS Status = EFI_SUCCESS;

	DEBUG((EFI_D_INFO, "[ABL] +++ EnterShippingMode()\n"));

	/*Status = WritePmicReg(PM8550B, 0x2852, 0x1);
	if (Status != EFI_SUCCESS)
	{
		DEBUG((EFI_D_ERROR, "[ABL] (ERROR) EnterShippingMode : set address %x to %x fail : %r\n", 0x2852, 0x1, Status));
	}*/
	
	adb_enter_shipping_mode = TRUE;
	DEBUG((EFI_D_INFO, "[ABL] --- EnterShippingMode()\n"));

	return Status;
}
// --- ASUS_BSP : add for enter shipping mode

// +++ ASUS_BSP : add for fuse blow
void get_fuse_status(void)
{
	DEBUG((EFI_D_ERROR, "\n" ));
	DEBUG((EFI_D_ERROR, "======================================================= \n" ));
	DEBUG((EFI_D_ERROR, "  asus_security_flag = 0x%x \n", IsSecureBootEnabled()));
	DEBUG((EFI_D_ERROR, "======================================================= \n" ));
	DEBUG((EFI_D_ERROR, "\n" ));

	if(IsSecureBootEnabled())
	{
		AsciiSPrint(cmd_fuse_Info, sizeof(cmd_fuse_Info), " SB=Y");
		DEBUG((EFI_D_ERROR, "[ABL] cmd_fuse_Info = %a\n", cmd_fuse_Info));
		AsciiSPrint(cmd_fuse_prop, sizeof(cmd_fuse_prop), " androidboot.fused=1");
		DEBUG((EFI_D_ERROR, "[ABL] cmd_fuse_prop = %a\n", cmd_fuse_prop));
	}
	else
	{
		AsciiSPrint(cmd_fuse_Info, sizeof(cmd_fuse_Info), " SB=N");
		DEBUG((EFI_D_ERROR, "[ABL] cmd_fuse_Info = %a\n", cmd_fuse_Info));
		AsciiSPrint(cmd_fuse_prop, sizeof(cmd_fuse_prop), " androidboot.fused=0");
		DEBUG((EFI_D_ERROR, "[ABL] cmd_fuse_prop = %a\n", cmd_fuse_prop));
	}

	// +++ ASUS_BSP : add for check fuse with no rpmb
	EFI_STATUS Status = EFI_SUCCESS;
	BOOLEAN SecureDeviceNoRpmb = FALSE;

	Status = IsSecureDeviceNoCheckRpmb(&SecureDeviceNoRpmb);
	if (Status != EFI_SUCCESS) {
		DEBUG ((EFI_D_ERROR, "[ABL] Failed read device state: %r\n", Status));
	}

	if(SecureDeviceNoRpmb)
	{
		AsciiSPrint(cmd_fuse_no_rpmb_Info, sizeof(cmd_fuse_no_rpmb_Info), " SBNR=Y");
		DEBUG((EFI_D_ERROR, "[ABL] cmd_fuse_no_rpmb_Info = %a\n", cmd_fuse_no_rpmb_Info));
		AsciiSPrint(cmd_fuse_no_rpmb_prop, sizeof(cmd_fuse_no_rpmb_prop), " androidboot.fused.norpmb=1");
		DEBUG((EFI_D_ERROR, "[ABL] cmd_fuse_no_rpmb_prop = %a\n", cmd_fuse_no_rpmb_prop));
	}
	else
	{
		AsciiSPrint(cmd_fuse_no_rpmb_Info, sizeof(cmd_fuse_no_rpmb_Info), " SBNR=N");
		DEBUG((EFI_D_ERROR, "[ABL] cmd_fuse_no_rpmb_Info = %a\n", cmd_fuse_no_rpmb_Info));
		AsciiSPrint(cmd_fuse_no_rpmb_prop, sizeof(cmd_fuse_no_rpmb_prop), " androidboot.fused.norpmb=0");
		DEBUG((EFI_D_ERROR, "[ABL] cmd_fuse_no_rpmb_prop = %a\n", cmd_fuse_no_rpmb_prop));
	}
	// --- ASUS_BSP : add for check fuse with no rpmb
}
// --- ASUS_BSP : add for fuse blow

// +++ ASUS_BSP : Display read panel UID
void ASUSGetPanelUniqueID(void)
{
	EFI_STATUS Status;

	UINT8 lcdUID[1];
	UINTN IDsize = sizeof(lcdUID);

	Status = gRT->GetVariable(
			L"LCD",
			&gQcomTokenSpaceGuid,
			NULL,
			&IDsize,
			&lcdUID);
	if (Status != EFI_SUCCESS) {
		DEBUG((EFI_D_ERROR, "[ABL] (ERROR) Can't GetVariable LCD\n", Status));
		AsciiSPrint(cmd_unique_id, sizeof(cmd_unique_id), " LCD=bdeeeafd");
	} else {
		AsciiSPrint(cmd_unique_id, sizeof(cmd_unique_id),
		            " msm_drm.LCDUID=%02x",
		            lcdUID[0]);
	}
	DEBUG((EFI_D_ERROR, "[DISPLAY] cmd_panel_uid : %a\n", cmd_unique_id));
}
// --- ASUS_BSP : Display read panel UID

// +++ ASUS_BSP : add for backup factory data
EFI_STATUS backup_persist_to_factory(VOID)
{
	EFI_STATUS                   Status = EFI_SUCCESS;
	EFI_BLOCK_IO_PROTOCOL       *BlkIo;
	PartiSelectFilter            HandleFilter;
	HandleInfo                   HandleInfoList[1];
	STATIC UINT32                MaxHandles;
	STATIC UINT32                BlkIOAttrib = 0;
	VOID                        *Buffer;
	UINT32                       PartitionSize;

	//persist
	EFI_GUID PartitionType_persist = { 0x6C95E238, 0xE343, 0x4BA8, { 0xB4, 0x89, 0x86, 0x81, 0xED, 0x22, 0xAD, 0x0B } };
	//factory
	EFI_GUID PartitionType_factory = { 0x7fd7c081, 0x6d59, 0x4ca3, { 0xab, 0x15, 0xb8, 0x5d, 0x38, 0x09, 0x95, 0x70 } };

	DEBUG((EFI_D_ERROR, "[ABL] +++ read persist partition() \n"));

	BlkIOAttrib = BLK_IO_SEL_PARTITIONED_MBR;
	BlkIOAttrib |= BLK_IO_SEL_PARTITIONED_GPT;
	BlkIOAttrib |= BLK_IO_SEL_MEDIA_TYPE_NON_REMOVABLE;
	BlkIOAttrib |= BLK_IO_SEL_MATCH_PARTITION_TYPE_GUID;

	HandleFilter.RootDeviceType = NULL;
	HandleFilter.PartitionType = &PartitionType_persist;
	HandleFilter.VolumeName = NULL;

	MaxHandles = sizeof(HandleInfoList)/sizeof(*HandleInfoList);

	Status = GetBlkIOHandles (BlkIOAttrib, &HandleFilter, HandleInfoList, &MaxHandles);

	if(Status == EFI_SUCCESS) {
		if (MaxHandles == 0) {
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): no partitions found.\r\n"));
			return EFI_NO_MEDIA;
		}else if (MaxHandles != 1) {
			// Unable to deterministically load from single partition
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): multiple partitions found.\r\n"));
			return EFI_LOAD_ERROR;
		}
	}else {
		DEBUG ((EFI_D_ERROR, "[ABL] %s: GetBlkIOHandles failed: %r\n", __func__, Status));
		return Status;
	}

	BlkIo = HandleInfoList[0].BlkIo;

	PartitionSize = (BlkIo->Media->LastBlock + 1) * BlkIo->Media->BlockSize;

	Buffer = AllocatePages(ALIGN_PAGES(PartitionSize, ALIGNMENT_MASK_4KB));

	if (Buffer == NULL) {
		DEBUG ((EFI_D_ERROR, "[ABL] (ERROR) read persist partition - allocate buffer fail \n\n"));
		return EFI_DEVICE_ERROR;
	}

	if (Buffer != NULL) {
		BlkIo->ReadBlocks (BlkIo,
						BlkIo->Media->MediaId,
						0,
						PartitionSize,//ROUND_TO_PAGE(ImageSize, BlkIo->Media->BlockSize - 1),
						Buffer);
	}
	DEBUG ((EFI_D_INFO, "[ABL] read persist partition - Buffer = %a\n", Buffer));

	HandleFilter.RootDeviceType = NULL;
	HandleFilter.PartitionType = &PartitionType_factory;
	HandleFilter.VolumeName = NULL;

	DEBUG((EFI_D_ERROR, "[ABL] +++ write factory partition() \n"));

	MaxHandles = sizeof(HandleInfoList)/sizeof(*HandleInfoList);

	Status = GetBlkIOHandles (BlkIOAttrib, &HandleFilter, HandleInfoList, &MaxHandles);

	if(Status == EFI_SUCCESS) {
		if (MaxHandles == 0) {
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): no partitions found.\r\n"));
			return EFI_NO_MEDIA;
		}else if (MaxHandles != 1) {
			// Unable to deterministically load from single partition
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): multiple partitions found.\r\n"));
			return EFI_LOAD_ERROR;
		}
	}else {
		DEBUG ((EFI_D_ERROR, "[ABL] %s: GetBlkIOHandles failed: %r\n", __func__, Status));
		return Status;
	}

	BlkIo = HandleInfoList[0].BlkIo;

	PartitionSize = (BlkIo->Media->LastBlock + 1) * BlkIo->Media->BlockSize;
	DEBUG ((EFI_D_ERROR, "[ABL] PartitionSize = %d \n",PartitionSize));

	Status = BlkIo->WriteBlocks(BlkIo,
							BlkIo->Media->MediaId,
							0,
							PartitionSize,
							Buffer);

	if(Status != EFI_SUCCESS) {
		DEBUG ((EFI_D_ERROR, "[ABL] (ERROR) write factory partition fail \n\n"));
		DEBUG ((EFI_D_ERROR, "[ABL] (ERROR) Status= %d \n\n",Status));
		return EFI_DEVICE_ERROR;
	}

	DEBUG((EFI_D_ERROR, "[ABL] --- write factory partition() \n\n"));
	return Status ;

}
// --- ASUS_BSP : add for backup factory data

// +++ ASUS_BSP : add for restore factory data
EFI_STATUS restore_factory_to_persist(VOID)
{
	EFI_STATUS                   Status = EFI_SUCCESS;
	EFI_BLOCK_IO_PROTOCOL       *BlkIo;
	PartiSelectFilter            HandleFilter;
	HandleInfo                   HandleInfoList[1];
	STATIC UINT32                MaxHandles;
	STATIC UINT32                BlkIOAttrib = 0;
	VOID                        *Buffer;
	UINT32                       PartitionSize;

	//factory
	EFI_GUID PartitionType_factory = { 0x7fd7c081, 0x6d59, 0x4ca3, { 0xab, 0x15, 0xb8, 0x5d, 0x38, 0x09, 0x95, 0x70 } };
	//persist
	EFI_GUID PartitionType_persist = { 0x6C95E238, 0xE343, 0x4BA8, { 0xB4, 0x89, 0x86, 0x81, 0xED, 0x22, 0xAD, 0x0B } };

	DEBUG((EFI_D_ERROR, "[ABL] +++ read factory partition() \n"));

	BlkIOAttrib = BLK_IO_SEL_PARTITIONED_MBR;
	BlkIOAttrib |= BLK_IO_SEL_PARTITIONED_GPT;
	BlkIOAttrib |= BLK_IO_SEL_MEDIA_TYPE_NON_REMOVABLE;
	BlkIOAttrib |= BLK_IO_SEL_MATCH_PARTITION_TYPE_GUID;

	HandleFilter.RootDeviceType = NULL;
	HandleFilter.PartitionType = &PartitionType_factory;
	HandleFilter.VolumeName = NULL;

	MaxHandles = sizeof(HandleInfoList)/sizeof(*HandleInfoList);

	Status = GetBlkIOHandles (BlkIOAttrib, &HandleFilter, HandleInfoList, &MaxHandles);

	if(Status == EFI_SUCCESS) {
		if (MaxHandles == 0) {
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): no partitions found.\r\n"));
			return EFI_NO_MEDIA;
		}else if (MaxHandles != 1) {
			// Unable to deterministically load from single partition
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): multiple partitions found.\r\n"));
			return EFI_LOAD_ERROR;
		}
	}else {
		DEBUG ((EFI_D_ERROR, "[ABL] %s: GetBlkIOHandles failed: %r\n", __func__, Status));
		return Status;
	}

	BlkIo = HandleInfoList[0].BlkIo;

	PartitionSize = (BlkIo->Media->LastBlock + 1) * BlkIo->Media->BlockSize;

	Buffer = AllocatePages(ALIGN_PAGES(PartitionSize, ALIGNMENT_MASK_4KB));

	if (Buffer == NULL) {
		DEBUG ((EFI_D_ERROR, "[ABL] (ERROR) read factory partition - allocate buffer fail \n\n"));
		return EFI_DEVICE_ERROR;
	}

	if (Buffer != NULL) {
		BlkIo->ReadBlocks (BlkIo,
						BlkIo->Media->MediaId,
						0,
						PartitionSize,//ROUND_TO_PAGE(ImageSize, BlkIo->Media->BlockSize - 1),
						Buffer);
	}
	DEBUG ((EFI_D_INFO, "[ABL] read factory partition - Buffer = %a\n", Buffer));

	HandleFilter.RootDeviceType = NULL;
	HandleFilter.PartitionType = &PartitionType_persist;
	HandleFilter.VolumeName = NULL;

	DEBUG((EFI_D_ERROR, "[ABL] +++ write persist partition() \n"));

	MaxHandles = sizeof(HandleInfoList)/sizeof(*HandleInfoList);

	Status = GetBlkIOHandles (BlkIOAttrib, &HandleFilter, HandleInfoList, &MaxHandles);

	if(Status == EFI_SUCCESS) {
		if (MaxHandles == 0) {
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): no partitions found.\r\n"));
			return EFI_NO_MEDIA;
		}else if (MaxHandles != 1) {
			// Unable to deterministically load from single partition
			DEBUG ((EFI_D_INFO, "[ABL] ExecImgFromVolume(): multiple partitions found.\r\n"));
			return EFI_LOAD_ERROR;
		}
	}else {
		DEBUG ((EFI_D_ERROR, "[ABL] %s: GetBlkIOHandles failed: %r\n", __func__, Status));
		return Status;
	}

	BlkIo = HandleInfoList[0].BlkIo;

	PartitionSize = (BlkIo->Media->LastBlock + 1) * BlkIo->Media->BlockSize;
	DEBUG ((EFI_D_ERROR, "[ABL] PartitionSize = %d \n",PartitionSize));

	Status = BlkIo->WriteBlocks(BlkIo,
							BlkIo->Media->MediaId,
							0,
							PartitionSize,
							Buffer);

	if(Status != EFI_SUCCESS) {
		DEBUG ((EFI_D_ERROR, "[ABL] (ERROR) write persist partition fail \n\n"));
		DEBUG ((EFI_D_ERROR, "[ABL] (ERROR) Status= %d \n\n",Status));
		return EFI_DEVICE_ERROR;
	}

	DEBUG((EFI_D_ERROR, "[ABL] --- write persist partition() \n\n"));
	return Status ;

}
// --- ASUS_BSP : add for restore factory data

/***** ASUS_ABL_INFO *****/
void set_abl_info(void)
{
	char abl_info[64]={0};
	char code_base[16]="0.0.024.0";
#ifdef ASUS_AI2205_BUILD
	char AI2205_abl_ver[8]="13";
#endif
	char build_type[32]={0};
	char build_sku[32]={0};

	/***** ASUS_BUILD_TYPE *****/
	#if ASUS_USERDEBUG_BUILD
		AsciiSPrint(build_type, sizeof(build_type), "userdebug");
	#endif

	#if ASUS_USER_BUILD
		AsciiSPrint(build_type, sizeof(build_type), "user");
	#endif

	#if ABL_FTM
		AsciiSPrint(build_type, sizeof(build_type), "FTM");
	#endif

	/***** ASUS_BUILD_SKU *****/
	#if WW_BUILD
		AsciiSPrint(build_sku, sizeof(build_sku), "WW");
	#endif

	#if OPEN_BUILD
		AsciiSPrint(build_sku, sizeof(build_sku), "OPEN");
	#endif

	#if CN_BUILD
		AsciiSPrint(build_sku, sizeof(build_sku), "CN");
	#endif

	/***** ASUS_BUILD_INFO *****/
	#ifdef ASUS_AI2205_BUILD
		AsciiSPrint(abl_info, sizeof(abl_info), "%a-%a-%a-%a", code_base, AI2205_abl_ver, build_sku, build_type);
	#endif

	SetBootloaderVersion(abl_info, sizeof(abl_info));
	DEBUG((EFI_D_ERROR, "\n" ));
	DEBUG((EFI_D_ERROR, "======================================================= \n" ));
	DEBUG((EFI_D_ERROR, "  ABL VERSION : %a\n",abl_info));
	DEBUG((EFI_D_ERROR, "======================================================= \n" ));
	DEBUG((EFI_D_ERROR, "\n" ));
}

void IsUartOn(void)
{
    if(DevInfo.is_uart_on)
    {
        DEBUG((EFI_D_ERROR, "[ABL] UART: ON\n"));
        AsciiSPrint(cmd_uart_status, sizeof(cmd_uart_status), " earlycon qcom_geni_serial.con_enabled=1");		
    }
}

#define COUNTRY_MAX_LEN 16
void set_hw_sku_info(void)
{
    UINT32 NFC_ID = 0;
    CHAR8 StrCountry[COUNTRY_MAX_LEN]="";
    NFC_ID = Get_NFC_ID();
    GetCountryCode(StrCountry, sizeof(StrCountry));
    if(NFC_ID == 2){
#ifdef ABL_FTM
		AsciiSPrint(cmd_hardware_sku_prop, sizeof(cmd_hardware_sku_prop), " androidboot.product.hardware.sku=CN");
		DEBUG((EFI_D_ERROR, "[ABL] set hw sku to CN for NFC check"));
#else
		if(!AsciiStrCmp ("CN", StrCountry)){
			AsciiSPrint(cmd_hardware_sku_prop, sizeof(cmd_hardware_sku_prop), " androidboot.product.hardware.sku=CN");
			DEBUG((EFI_D_ERROR, "[ABL] set hw sku to CN for NFC check"));
		}
#endif        
    }else{
        AsciiSPrint(cmd_hardware_sku_prop, sizeof(cmd_hardware_sku_prop), "");
        DEBUG((EFI_D_ERROR, "[ABL] Skip set hw sku\n"));
    }
}

/***** ASUS_ABL_INIT *****/
void ASUS_Init(void)
{

	Get_XBL_INFO();
	Get_DT_ID();
	Set_HWStage_Info();

	Get_CPU_ID_HASH();
	//Get_DDR_Manufacturer_ID();
	//Get_DDR_Device_Type();

	Get_Boot_Count();
	Get_XBL_PowerKey_Press();

	DEBUG((EFI_D_ERROR, "\n"));
	DEBUG((EFI_D_ERROR, "======================================================= \n"));
	DEBUG((EFI_D_ERROR, "  PROJECT ID INFO \n"));
	DEBUG((EFI_D_ERROR, "======================================================= \n"));

	Get_PCBID();
	DEBUG((EFI_D_ERROR, "[ABL] ASUS_PJ_ID = %d\n", Get_PJ_ID()));
	DEBUG((EFI_D_ERROR, "[ABL] ASUS_HW_ID = %d\n", Get_HW_ID()));
	DEBUG((EFI_D_ERROR, "[ABL] ASUS_LGF_ID = %d\n", Get_LGF_ID()));
	DEBUG((EFI_D_ERROR, "[ABL] ASUS_SKU_ID = %d\n", Get_SKU_ID()));
	DEBUG((EFI_D_ERROR, "[ABL] ASUS_DDR_ID = %d\n", Get_DDR_ID()));
	DEBUG((EFI_D_ERROR, "[ABL] ASUS_NFC_ID = %d\n", Get_NFC_ID()));
	DEBUG((EFI_D_ERROR, "[ABL] ASUS_RF_ID = %d\n", Get_RF_ID()));
	DEBUG((EFI_D_ERROR, "[ABL] ASUS_FP_ID = %d\n", Get_FP_ID()));
	DEBUG((EFI_D_ERROR, "[ABL] ASUS_FEATURE_ID = %x\n", Get_FEATURE_ID()));
	DEBUG((EFI_D_ERROR, "[ABL] ASUS_JTAG_ID = 0x%x\n", Get_JTAG_ID()));
	DEBUG((EFI_D_ERROR, "[ABL] ASUS_CPU_ID = 0x%x\n", Get_CPU_ID()));
	DEBUG((EFI_D_ERROR, "[ABL] ASUS_DT_ID = %d\n", Get_DT_ID()));
	DEBUG((EFI_D_ERROR, "[ABL] ASUS_LGF_CON_ID = %d\n", Get_LGF_CON_ID()));
	DEBUG((EFI_D_ERROR, "[ABL] ASUS_FC_ID = %d\n", Get_FC_ID()));
	DEBUG((EFI_D_ERROR, "[ABL] ASUS_UPPER_ID = %d\n", Get_UPPER_ID()));
	DEBUG((EFI_D_ERROR, "[ABL] ASUS_SUB_ID = %d\n", Get_SUB_ID()));

	//ASUS BSP Display +++
	ASUSGetPanelUniqueID();

	DEBUG((EFI_D_ERROR, "\n"));
	DEBUG((EFI_D_ERROR, "ASUS_PROJECT = %a\n", asus_project_info));
	DEBUG((EFI_D_ERROR, "ASUS_HW_STAGE = %a\n", asus_stage_info));
	DEBUG((EFI_D_ERROR, "======================================================= \n"));
	DEBUG((EFI_D_ERROR, "\n"));

}

////////////////////////////////////////////////////////////////////////
// --- ROG7 : PROJECT COMMON FUNCTION
////////////////////////////////////////////////////////////////////////


EFI_STATUS ASUS_ABL_INIT(void)
{
	EFI_STATUS status = EFI_SUCCESS;

	DEBUG((EFI_D_ERROR, "\n"));
	DEBUG((EFI_D_ERROR, "##### ASUS ABL INIT START #####\n"));

	ASUS_Init();

	// +++ ASUS_BSP : add for ftm mode & charger mode
	if(is_ftm_mode())
	{
		set_cmdline(ASUS_FTM_MODE);
		EnableChargingScreen(FALSE);
	}
	else
	{
		EnableChargingScreen(TRUE);
	}
	// --- ASUS_BSP : add for ftm mode & charger mode

	set_abl_info(); // +++ ASUS_BSP : add for abl info

	check_verity();// +++ ASUS_BSP : add for check AVB Verity
	check_apdp();// +++ ASUS_BSP : add for check apdp partition
	check_rawdump_partition();// +++ ASUS_BSP : check if have rawdump partition or not

	read_ssn(); // +++ ASUS_BSP : add for ssn info (from asuskey4)
	read_imei(); // +++ ASUS_BSP : add for imei info (from asuskey4)
	read_imei2(); // +++ ASUS_BSP : add for imei info (from asuskey4)
	read_isn(); // +++ ASUS_BSP : add for isn info (from asuskey4)
	read_cid(); // +++ ASUS_BSP : add for cid info (from asuskey4)
	read_countrycode(); // +++ ASUS_BSP : add for country info (from asuskey4)
	read_TOOLID(); // +++ ASUS_BSP : add for toolid info (from asuskey4)
	set_hw_sku_info(); // +++ ASUS_BSP : add for NFC check hardware sku
#ifdef ASUS_AI2205_BUILD
    CheckValidImage(); // +++ ASUS_BSP : add for check CN devices with IMEI range
#endif
	prepare_untar_step(); // +++ ASUS_BSP : add for re-unTAR process
#if ABL_FTM
	check_tar_step(); // +++ ASUS_BSP : add for check TAR for main sku and CN sku
#endif
	IsLogcatAsdfOn(); // +++ ASUS_BSP : add for logcat-asdf sevices

	IsUartOn();
	
	DEBUG((EFI_D_ERROR, "###### ASUS ABL INIT END ######\n"));
	DEBUG((EFI_D_ERROR, "\n"));

	return status;
}
////////////////////////////////////////////////////////////////////////
#endif

