/*===============================================================================
  Copyright (c) 2017-2019, 2021, 2022 Qualcomm Technologies, Inc.  All Rights Reserved.
  
  FILE:         EFIPIL.h
  DESCRIPTION:    
  
  REVISION HISTORY
  when       who         what, where, why
  --------   ---         --------------------------------------------------------
  08/08/22   cm          Added MediaType support in UEFI pil
                         Add Support for loading images from backup partition
  12/08/21   cm          Added Protocol Version 3
  09/10/20   yw          added two APIS, ShutdownPilImage and GetSubsysCfgCpyById
  03/21/19   md          Added Protocol to Authenticate ELF-FV image
  06/14/17   yw          initial version

================================================================================*/

#ifndef __EFIPIL_H__
#define __EFIPIL_H__

#define EFI_PIL_PROTOCOL_VERSION    EFI_PIL_PROTOCOL_VERSION_V4

#define EFI_PIL_PROTOCOL_VERSION_V1 0x0000000000010001
#define EFI_PIL_PROTOCOL_VERSION_V2 0x0000000000010002
#define EFI_PIL_PROTOCOL_VERSION_V3 0x0000000000010003

/*Added new cfg version PIL_SUBSYS_CFG_VERSION_V3 */
#define EFI_PIL_PROTOCOL_VERSION_V4 0x0000000000010004

#define EFI_PIL_PROTOCOL_GUID \
  { 0x3bec87db, 0x4e76, 0x491c, { 0x96, 0x6c, 0x7c, 0xa5, 0x81, 0x2a, 0x64, 0xc9 } }
  
#define NAME_MAX_LENGTH        32
  
typedef enum _PIL_ELF_TYPE_
{
    ELF_FV           = 1,            /*FV image pushed to raw parition*/
    ELF_SPLIT        = 2,            /*ELF image are splited to xxx.mdt xxx.b00 xxx,b01... */
    ELF_SINGLE       = 3,            /*original ELF image */
    ELF_MERGED       = 4,            /*merged splited image */
    MAX_PIL_ELF_TYPE = 0x7FFFFFFF
} PIL_ELF_TYPE;


#define    PIL_SUBSYS_CFG_VERSION_V1        1
#define    PIL_SUBSYS_CFG_VERSION_V2        2
#define    PIL_SUBSYS_CFG_VERSION_V3        3
#define    PIL_SUBSYS_CFG_LATEST_VERSION    PIL_SUBSYS_CFG_VERSION_V3

/**
  struct to describe a subsystem which will be loaded by PIL
**/
typedef struct _PIL_SUBSYS_CFG_TYPE_
{
    //
    // version of the struct, caller must set the appropriate version
    //
    UINT64  Version;
    
    //
    // Elf type of the image to be loaded
    //
    PIL_ELF_TYPE  Type;
    
    //
    // firmware name of the subsys
    // optional for FAT partition
    // required for RAW partition
    //
    UINT16   FwName[NAME_MAX_LENGTH] OPTIONAL;
    
    // 
    // Partition label where to load the image, for RAW/FAT partition
    //
    UINT16   PartiLabel[NAME_MAX_LENGTH] OPTIONAL;
    
    //
    // RootGuid of the partition where to load the iamge, used for RAW partition
    // optional for FAT partition
    //
    EFI_GUID    PartiRootGuid OPTIONAL;
    
    //
    // GUID of the partition where to load the image, used for RAW partition
    // optional if PartiLabel is given
    // 
    EFI_GUID    PartiGuid OPTIONAL;
    
    //
    // Image path where to load the image, used for FAT patition
    // example: \\image\\adsp
    // optional if it's RAW partition
    //
    UINT16   ImagePath[NAME_MAX_LENGTH] OPTIONAL;
    
    //
    // subsystem ID of the proc
    //
    UINT32  SubsysID;
    
    //
    // start address of the reserved memory region where to load the image
    // if the ResvRegionStart is not specified memory will be allocated dynamically
    //
    UINT64    ResvRegionStart OPTIONAL;
    
    //
    // size of the reserved loading memory region
    //
    UINT64    ResvRegionSize OPTIONAL;
    
    //
    // if image load info is required to fill in IMEM
    // default is FLASE unless it's specified
    //
    BOOLEAN ImageLoadInfo;
    
    // 
    // if unlock is required immdiatly  after bring subsys out of reset
    //
    BOOLEAN Unlock;
    
    // 
    // if yes ingore checking if loading address read from ELF matches ipcat
    //
    BOOLEAN OverrideElfAddr;

    //
    // Guid corresponding to the EFI_PIL_PROXY_PROTOCOL implementation
    //
    EFI_GUID    ProxyGuid;
	
	// 
    // Backup Partition label where to load the image, for RAW/FAT partition
    //
    UINT16  BackupPartiLabel[NAME_MAX_LENGTH] OPTIONAL;  /*Added in PIL_SUBSYS_CFG_VERSION_V2 */
    
    //
    // Backup RootGuid of the partition where to load the iamge, used for RAW partition
    // optional for FAT partition
    //
    EFI_GUID  BackupPartiRootGuid OPTIONAL;              /*Added in PIL_SUBSYS_CFG_VERSION_V2 */
    
    //
    // Backup GUID of the partition where to load the image, used for RAW partition
    // optional if PartiLabel is given
    // 
    EFI_GUID  BackupPartiGuid OPTIONAL;                 /*Added in PIL_SUBSYS_CFG_VERSION_V2 */

    //
    //Name of Image to be loaded before the main Image
    //
    UINT16 PreImage[NAME_MAX_LENGTH] OPTIONAL;          /*Added in PIL_SUBSYS_CFG_VERSION_V3 */

    //
    //Name of Image to be loaded before the main Image
    //
    UINT16 PostImage[NAME_MAX_LENGTH] OPTIONAL;        /*Added in PIL_SUBSYS_CFG_VERSION_V3 */

    //
    //Image to be loaded from Storage Partition 
	//or Network
    //
    CHAR16 MediaType[NAME_MAX_LENGTH] OPTIONAL;        /*Added in PIL_SUBSYS_CFG_VERSION_V3 */
} PIL_SUBSYS_CFG_TYPE;


extern EFI_GUID gEfiPilProtocolGuid;

/**
  Boot PIL images, taking the firmware name input, find out the PIL image 
  configuration from SecParti.cfg and load the image

  @param  Subsys               firmware name of the image

  @retval EFI_SUCCESS          The image is loaded successfully.
  
**/
typedef 
EFI_STATUS 
(EFIAPI *EFI_PROCESS_PIL_IMAGE) (
    IN CHAR16*  Subsys
    );
    
/**
  Boot PIL images, taking the PIL_SUBSYS_CFG_TYPE struct as input,
  load the image, verify authentication and boot the image

  @param  SubsysCfg            Configure structure of the image

  @retval EFI_SUCCESS          The image is loaded successfully.
  
**/
typedef 
EFI_STATUS 
(EFIAPI *EFI_PROCESS_PIL_IMAGE_EXT) (
    IN PIL_SUBSYS_CFG_TYPE*  SubsysCfg
    );


typedef
EFI_STATUS 
(EFIAPI *EFI_PROCESS_PIL_AuthELFFVImageFromBuffer) (
   VOID *Buffer,
   VOID** FvImageBase
   );
   
typedef
EFI_STATUS
(EFIAPI *EFI_PROCESS_PIL_SHUTDOWN) (
    IN UINT32  SubsysId
    );
    
typedef
EFI_STATUS
(EFIAPI *EFI_PROCESS_PIL_GET_LOADED_IMAGE_CFG_BY_ID) (
    IN UINT32  SubsysId,
    OUT PIL_SUBSYS_CFG_TYPE* CfgPtr
    );

/**
  find the cfg in cfg file and return pil cfg struct.

  @param  cfg     [in]          unique FwName of the subsys
  @param  cfg     [out]         return pil cfg struct
  
  @return EFI_SUCCESS           The function completed successfully.

**/
typedef
EFI_STATUS
(EFIAPI *EFI_PROCESS_PIL_INIT_CFG_BY_UNI_CODE_NAME) (
    IN CHAR16 *subsys_name,
    OUT PIL_SUBSYS_CFG_TYPE* CfgOut
    );

typedef struct _EFI_PIL_PROTOCOL {
   UINT64                                       Revision;
   EFI_PROCESS_PIL_IMAGE                        ProcessPilImage;
   EFI_PROCESS_PIL_IMAGE_EXT                    ProcessPilImageExt;
   EFI_PROCESS_PIL_AuthELFFVImageFromBuffer     AuthELFFVImageFromBuffer;
   EFI_PROCESS_PIL_SHUTDOWN                     ProcessPilShutdown;       /*Added in EFI_PIL_PROTOCOL_VERSION_V2 */
   EFI_PROCESS_PIL_GET_LOADED_IMAGE_CFG_BY_ID   GetLoadedImageCfgCpyById; /*Added in EFI_PIL_PROTOCOL_VERSION_V2 */
   EFI_PROCESS_PIL_INIT_CFG_BY_UNI_CODE_NAME    PilInitCfgByUnicodeName;  /*Added in EFI_PIL_PROTOCOL_VERSION_V3*/

}EFI_PIL_PROTOCOL;

#endif
