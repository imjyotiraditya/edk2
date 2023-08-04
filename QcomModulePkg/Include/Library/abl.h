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

#ifndef __ABL_H__
#define __ABL_H__

#include "DeviceInfo.h"

#ifdef ASUS_BUILD
////////////////////////////////////////////////////////////////////////
// ROG7 : PROJECT COMMON DEFINITION & STRUCTURE
////////////////////////////////////////////////////////////////////////
/***** PMIC_INDEX *****/
#define PMK8050    0
#define PM8550     1
#define PM8550VS_C 2
#define PM8550VS_D 3
#define PM8550VS_E 4
#define PM8550VE_F 5
#define PM8550VS_G 6
#define PM8550B    7
#define PMR735D    0xA
#define PM8010_1   0xC
#define PM8010_2   0xD

/***** PJ_ID *****/
#define PJ_ID_ENTRY             0
#define PJ_ID_PRO               1
#define PJ_ID_ULTIMATE          2
#define PJ_ID_TBD               3

#define PJ_ID_UNKNOWN           (-1)

/***** ASUS_KEY *****/
#define ASUS_POWER_KEY               0xFF    // (non-define)
#define ASUS_VOLUME_UP_KEY              6    // PM8350_GPIO06
#define ASUS_VOLUME_DOWN_KEY         0xFF    // (non-define)

/***** ASUS_KEY_EVENT *****/
#define KEY_PRESSED                0x55aa
#define KEY_RELEASE                0xaa55
#define GET_KEY_ERROR              0xffff

/***** ASUS_ENTER_MODE *****/
#define ASUS_NORMAL_MODE                0
#define ASUS_FTM_MODE                   1
#define ASUS_USERDEBUG_MODE             2
#define ASUS_CTS_MODE                   3

/***** ASUS_PHONE_INFO *****/
#define SSN_LEN                        20
#define IMEI_LEN                       15
#define IMEI2_LEN                      15
#define ISN_LEN                        20
#define CID_LEN                        17
#define COUNTRY_LEN                    17
#define TOOLID_LEN                     64

/***** ASUS_USER_UNLOCK *****/
#define SIGNATURE_LEN 256 // +++ ASUS_BSP : add for user unlock

/***** ABL_BUILD_SKU *****/
#if WW_BUILD
#define ABL_BUILD_SKU "WW"
#endif
#if CN_BUILD
#define ABL_BUILD_SKU "CN"
#endif
#if OPEN_BUILD
#define ABL_BUILD_SKU "OPEN"
#endif
#ifndef ABL_BUILD_SKU
#define ABL_BUILD_SKU "OPEN"
#endif

/***** ASUS_AB_RETRY_UPDATE *****/
#define ASUS_AB_RETRY_UPDATE 0
////////////////////////////////////////////////////////////////////////
#endif


#ifdef ASUS_AI2205_BUILD
////////////////////////////////////////////////////////////////////////
// ROG7 : PROJECT DEFINITION & STRUCTURE
////////////////////////////////////////////////////////////////////////
/***** AI2205_HW_ID *****/
#define HW_ID_EVB            0xD
#define HW_ID_SR1            0x1 
#define HW_ID_SR2            0x2
#define HW_ID_ER1            0x4
#define HW_ID_ER2            0x8
#define HW_ID_PR             0x9
#define HW_ID_MP             0xB

/***** AI2205_DT_ID *****/
#define DT_ID_EVB        30
#define DT_ID_SR1        40
#define DT_ID_SR2        41
#define DT_ID_ER1        50
#define DT_ID_ER2        51
#define DT_ID_PR         60
#define DT_ID_MP         70
#define DT_ID_UNKNOWN    (-1)

/***** LGF_ID *****/
#define LGF_ID_ENTRY          0
#define LGF_ID_PRO            1
#define LGF_ID_ULTIMATE       2
#define LGF_ID_UNKNOWN        (-1)

/***** LGF_CON_ID *****/
#define LGF_CON_ID_LGF             0
#define LGF_CON_ID_PMOLED          1
#define LGF_CON_ID_UNKNOWN         (-1)

/***** SKU_ID *****/
#define SKU_ID_0         0    // UFS3.1/128GB
#define SKU_ID_1         1    // UFS3.1/256GB
#define SKU_ID_2         2    // UFS3.1/512GB
#define SKU_ID_3         3    // UFS3.1/1TB
#define SKU_ID_4         4    // reserved
#define SKU_ID_5         5    // reserved
#define SKU_ID_6         6    // reserved
#define SKU_ID_7         7    // reserved
#define SKU_ID_UNKNOWN         (-1)

/***** DDR_ID *****/
#define DDR_ID_0         0    // reserved
#define DDR_ID_1         1    // reserved
#define DDR_ID_UNKNOWN         (-1)

/***** NFC_ID *****/
#define NFC_ID_0         0
#define NFC_ID_1         1    // NC (un-mount)
#define NFC_ID_UNKNOWN         (-1)

/***** RF_ID *****/
#define RF_ID_0          0   //CN
#define RF_ID_1          1   //WW
#define RF_ID_2          2   //WW(US) 
#define RF_ID_3          3   //no RF board
#define RF_ID_UNKNOWN    (-1)

/***** FP_ID *****/
#define FP_ID_0          0
#define FP_ID_1          1
#define FP_ID_UNKNOWN    (-1)

/***** FC_ID *****/
#define FC_ID_0          0    // grip sensor
#define FC_ID_1          1 
#define FC_ID_UNKNOWN    (-1)

/***** UPPER_ID *****/
#define UPPER_ID_0       0  // WW-H
#define UPPER_ID_1       1  // WW-L/CN
#define UPPER_ID_UNKNOWN       (-1)

/***** SUB_ID *****/
#define SUB_ID_0          0    // WW-H/WW-L
#define SUB_ID_1          1    // CN
#define SUB_ID_UNKNOWN          (-1)

/***** USER_UNLOCK *****/
#define SIGNATURE_LEN          256    // +++ ASUS_BSP : add for user unlock

/***** ASUS_SYS_INFO *****/
#define ASUS_MAGIC "ASUS-MAGIC-NUM!!"
#define ASUS_MAGIC_SIZE 16

struct sys_info {
	char magic[ASUS_MAGIC_SIZE];
	char board_info[16];
	char mem_info[16];
	char cpu_freq[16];
	char product_name[16];
//	char product_locale[16];
	char product_carrier[64];
	char csc_build_version[64];
	char bt_mac[16];
	char wifi_mac[16];
	char wifi_mac_2[16];
	char imei[16];
	char imei2[16];
	char ssn[16];
	char isn[18];
	char color[16];
	char country[16];
	char customer[16];
//	char versatility[16];
	char ufs_info[64];
	char setupwizard[16];
	char toolid_check[16];
	char falling_magic[16];
	int  falling_count;
	char falling_time[50][32];
	char hit_magic[16];
	int  hit_count;
	char hit_time[50][32];
	char thump_magic[16];
	int  thump_count;
	char thump_time[50][32];
};
typedef struct sys_info sys_info;
////////////////////////////////////////////////////////////////////////
#endif

#ifdef ASUS_BUILD
////////////////////////////////////////////////////////////////////////
// ROG7 : PROJECT COMMON FUNCTION
////////////////////////////////////////////////////////////////////////
EFI_STATUS ASUS_ABL_INIT(void);

/***** ASUS_GET_PROJECT_ID *****/
UINT32 Get_SOC_ID();

/***** ASUS_GET_PROJECT_ID *****/
UINT32 Get_PJ_ID();

/***** ASUS_GET_DT_ID *****/
UINT32 Get_DT_ID();

/***** ASUS_Get_Boot_Count *****/
void Get_Boot_Count();

/***** ASUS_Get_PowerKey_Press *****/
UINT32 Get_XBL_PowerKey_Press();

/***** ASUS_MODEM_FUNCTION *****/
EFI_STATUS erase_fsg(VOID);        // +++ ASUS_BSP : add for re-unTAR process
EFI_STATUS erase_modemst1(VOID);        // +++ ASUS_BSP : add for re-unTAR process
EFI_STATUS erase_modemst2(VOID);        // +++ ASUS_BSP : add for re-unTAR process
EFI_STATUS prepare_untar_step(VOID);         // +++ ASUS_BSP : add for re-unTAR process
EFI_STATUS erase_frp_partition(VOID);          //+++ ASUS_BSP : erase_frp_partition

/***** ASUS_AVB_VERITY *****/
const char* read_vbmeta_magic(VOID);// +++ ASUS_BSP : add for read vbmeta magic
EFI_STATUS enable_vbmeta_magic(VOID);// +++ ASUS_BSP : add for enable vbmeta magic
EFI_STATUS disable_verity(VOID);        // +++ ASUS_BSP : add for disable verity
EFI_STATUS enable_verity(VOID);        // +++ ASUS_BSP : add for enable verity
EFI_STATUS check_verity(VOID);        // +++ ASUS_BSP : add for check Verity

/***** ASUS_DEBUG_POLICY *****/
EFI_STATUS check_apdp(VOID);       // +++ ASUS_BSP : add for check apdp partition
EFI_STATUS erase_apdp_partition(VOID);          //+++ ASUS_BSP : add for erase_apdp_partition

/***** ASUS_PMIC_INFO *****/
EFI_STATUS GetPmicReg(IN UINT32 PmicDeviceIndex,IN UINT32 RegAddress,OUT UINT8 *RegValue);       // +++ ASUS_BSP : add for get_pmic_reg value
EFI_STATUS WritePmicReg(IN UINT32 PmicDeviceIndex,IN UINT32 RegAddress,IN UINT32 RegValue);       // +++ ASUS_BSP : add for write_pmic_reg value

/***** ASUS_SYS_INFO *****/
EFI_STATUS read_sysinfo(struct sys_info *in);

/***** ASUS_RAWDUMP_CONFIG *****/
EFI_STATUS RePartition(void);       // +++ ASUS_BSP : add for re-partition from gpt to partition:0 for add rawdump partition
BOOLEAN check_rawdump_partition(void);           // +++ ASUS_BSP : add for check if have rawdump partition or not

/***** ASUS_CRC_32 *****/
EFI_STATUS CalculateCrc32 (IN UINT8 *Data, IN UINTN DataSize, IN OUT UINT32 *CrcOut);
unsigned long AsusCalculatePtCrc32(CHAR16 *Pname);
unsigned long AsusCalculatePtCrc32Size(CHAR16 *Pname, UINT32 ImageSize);

/***** USER_UNLOCK *****/
EFI_STATUS read_asuskey(UINT8 *signature_ptr, UINT32 signature_len);        // +++ ASUS_BSP : add for user unlock
unsigned is_unlock(UINT32 decrypt_type);       // +++ASUS_BSP : add for user unlock

/***** ASUS_UNLOCK *****/
int ASUS_Get_UNLOCK_STATE(); // +++ ASUS_BSP : add for asus unlock state

/***** ASUS_EnterShippingMode *****/
EFI_STATUS EnterShippingMode(void); // +++ ASUS_BSP : add for enter shipping mode

/***** Check_FTM_MODE *****/
unsigned is_ftm_mode(void);

/***** Check_FUSE_STATUS *****/
void get_fuse_status(void);

/***** ASUS_GET_DDR_INFO *****/
//UINT8 Get_DDR_Manufacturer_ID(void);
//UINT8 Get_DDR_Device_Type(void);

/***** ASUS_FACTORY_FUNCTION *****/
EFI_STATUS backup_persist_to_factory(VOID); // +++ ASUS_BSP : add for backup factory data
EFI_STATUS restore_factory_to_persist(VOID); // +++ ASUS_BSP : add for restore factory data
////////////////////////////////////////////////////////////////////////
#endif

#ifdef ASUS_AI2205_BUILD
////////////////////////////////////////////////////////////////////////
// ROG7 : PROJECT FUNCTION
////////////////////////////////////////////////////////////////////////
/***** ASUS_GET_ID_VALUE *****/
UINT32 Get_HW_ID();
UINT32 Get_SKU_ID();
UINT32 Get_LGF_ID();
UINT32 Get_DDR_ID();
UINT32 Get_NFC_ID();
UINT32 Get_RF_ID();
UINT32 Get_FP_ID();
UINT32 Get_FEATURE_ID();
UINT32 Get_JTAG_ID();
UINT64 Get_CPU_ID();
UINT32 Get_LGF_CON_ID();
UINT32 Get_FC_ID();
UINT32 Get_SUB_ID();
UINT32 Get_UPPER_ID();
void Get_PCBID(void);
BOOLEAN check_sysconf(VOID);
EFI_STATUS RecordResetDevInfo(VOID);
EFI_STATUS copy_asuskey3_to_devcfg(VOID);
////////////////////////////////////////////////////////////////////////
#endif

#endif
