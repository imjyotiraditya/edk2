#/*
# * Copyright (c) 2015-2021, The Linux Foundation. All rights reserved.
# *
# * Redistribution and use in source and binary forms, with or without
# * modification, are permitted provided that the following conditions are
# * met:
# * * Redistributions of source code must retain the above copyright
# *  notice, this list of conditions and the following disclaimer.
# *  * Redistributions in binary form must reproduce the above
# * copyright notice, this list of conditions and the following
# * disclaimer in the documentation and/or other materials provided
# *  with the distribution.
# *   * Neither the name of The Linux Foundation nor the names of its
# * contributors may be used to endorse or promote products derived
# * from this software without specific prior written permission.
# *
# * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
# * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
# * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
# * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
# * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
# * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
# * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#*/

#/*
# *  Changes from Qualcomm Innovation Center are provided under the following license:
# *
# *  Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
# *
# *  Redistribution and use in source and binary forms, with or without
# *  modification, are permitted (subject to the limitations in the
# *  disclaimer below) provided that the following conditions are met:
# *
# *      * Redistributions of source code must retain the above copyright
# *        notice, this list of conditions and the following disclaimer.
# *
# *      * Redistributions in binary form must reproduce the above
# *        copyright notice, this list of conditions and the following
# *        disclaimer in the documentation and/or other materials provided
# *        with the distribution.
# *
# *      * Neither the name of Qualcomm Innovation Center, Inc. nor the names of its
# *        contributors may be used to endorse or promote products derived
# *        from this software without specific prior written permission.
# *
# *  NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
# *  GRANTED BY THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
# *  HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
# *   WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
# *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
# *  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
# *  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
# *  GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
# *  IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
# *  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
# *  IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#*/

#/** @file QcomModulePkg.dsc
# QcomModule package.

################################################################################
#
# Defines Section - statements that will be processed to create a Makefile.
#
################################################################################
[Defines]
  PLATFORM_NAME                  = QcomModulePkg
  PLATFORM_GUID                  = 4476742F-4C2D-BA9D-992A-CB82914F5E58
  PLATFORM_VERSION               = 0.1
  DSC_SPECIFICATION              = 0x00010005
  OUTPUT_DIRECTORY               = $(ABL_OUT_DIR)
  SUPPORTED_ARCHITECTURES        = ARM|AARCH64
  BUILD_TARGETS                  = DEBUG|RELEASE
  SKUID_IDENTIFIER               = DEFAULT
  FLASH_DEFINITION               = QcomModulePkg/QcomModulePkg.fdf

[LibraryClasses.common]
  BaseStackCheckLib|MdePkg/Library/BaseStackCheckLib/BaseStackCheckLib.inf
  DebugPrintErrorLevelLib|MdePkg/Library/BaseDebugPrintErrorLevelLib/BaseDebugPrintErrorLevelLib.inf
  MemoryAllocationLib|MdePkg/Library/UefiMemoryAllocationLib/UefiMemoryAllocationLib.inf
  BaseLib|MdePkg/Library/BaseLib/BaseLib.inf
  BaseMemoryLibOptDxe|MdePkg/Library/BaseMemoryLibOptDxe/BaseMemoryLibOptDxe.inf
  PrintLib|MdePkg/Library/BasePrintLib/BasePrintLib.inf
  CacheMaintenanceLib|ArmPkg/Library/ArmCacheMaintenanceLib/ArmCacheMaintenanceLib.inf
  IoLib|MdePkg/Library/BaseIoLibIntrinsic/BaseIoLibIntrinsic.inf
  UefiLib|MdePkg/Library/UefiLib/UefiLib.inf
  HobLib|MdePkg/Library/DxeHobLib/DxeHobLib.inf
  UefiRuntimeServicesTableLib|MdePkg/Library/UefiRuntimeServicesTableLib/UefiRuntimeServicesTableLib.inf
  DevicePathLib|MdePkg/Library/UefiDevicePathLib/UefiDevicePathLib.inf
  UefiBootServicesTableLib|MdePkg/Library/UefiBootServicesTableLib/UefiBootServicesTableLib.inf
  DxeServicesTableLib|MdePkg/Library/DxeServicesTableLib/DxeServicesTableLib.inf
  UefiApplicationEntryPoint|MdePkg/Library/UefiApplicationEntryPoint/UefiApplicationEntryPoint.inf
  PcdLib|MdePkg/Library/BasePcdLibNull/BasePcdLibNull.inf
  FdtLib|EmbeddedPkg/Library/FdtLib/FdtLib.inf
  LibUfdt|EmbeddedPkg/Library/LibUfdt/LibUfdt.inf
  TimerLib|ArmPkg/Library/ArmArchTimerLib/ArmArchTimerLib.inf
  ArmGenericTimerCounterLib|ArmPkg/Library/ArmGenericTimerPhyCounterLib/ArmGenericTimerPhyCounterLib.inf
  Zlib|QcomModulePkg/Library/zlib/zlib.inf
  DebugLib|MdeModulePkg/Library/PeiDxeDebugLibReportStatusCode/PeiDxeDebugLibReportStatusCode.inf
  ReportStatusCodeLib|MdeModulePkg/Library/DxeReportStatusCodeLib/DxeReportStatusCodeLib.inf
  DebugPrintErrorLevelLib|MdeModulePkg/Library/DxeDebugPrintErrorLevelLib/DxeDebugPrintErrorLevelLib.inf
  UefiDriverEntryPoint|MdePkg/Library/UefiDriverEntryPoint/UefiDriverEntryPoint.inf
  PerformanceLib|MdeModulePkg/Library/DxePerformanceLib/DxePerformanceLib.inf
  AvbLib|QcomModulePkg/Library/avb/AvbLib.inf
  AesLib|QcomModulePkg/Library/aes/AesLib.inf

[LibraryClasses.ARM]
  ArmLib|ArmPkg/Library/ArmLib/ArmBaseLib.inf
  NULL|ArmPkg/Library/CompilerIntrinsicsLib/CompilerIntrinsicsLib.inf

[LibraryClasses.AARCH64]
  ArmLib|ArmPkg/Library/ArmLib/ArmBaseLib.inf
  NULL|ArmPkg/Library/CompilerIntrinsicsLib/CompilerIntrinsicsLib.inf

[LibraryClasses.common.UEFI_APPLICATION]
  ReportStatusCodeLib|MdeModulePkg/Library/DxeReportStatusCodeLib/DxeReportStatusCodeLib.inf
  ExtractGuidedSectionLib|MdePkg/Library/DxeExtractGuidedSectionLib/DxeExtractGuidedSectionLib.inf

[BuildOptions.common]
  GCC:*_*_*_ARCHCC_FLAGS  = -Wno-shift-negative-value -fstack-protector-all -Wno-varargs -fno-common -Wno-misleading-indentation -Wno-unknown-warning-option
  GCC:*_*_*_DLINK_FLAGS = -Wl,-Ttext=0x0
  GCC:*_*_*_CC_FLAGS = -DZ_SOLO
  GCC:*_*_*_CC_FLAGS = -DPRODUCT_NAME=\"$(BOARD_BOOTLOADER_PRODUCT_NAME)\"

  !if $(VERIFIED_BOOT_LE)
      GCC:*_*_*_CC_FLAGS = -DVERIFIED_BOOT_LE
  !endif
  !if $(EARLY_ETH_ENABLED)
      GCC:*_*_*_CC_FLAGS = -DEARLY_ETH_ENABLED
  !endif
  !if $(HIBERNATION_SUPPORT_NO_AES)
      GCC:*_*_*_CC_FLAGS = -DHIBERNATION_SUPPORT_NO_AES
      GCC:*_*_*_PP_FLAGS = -DHIBERNATION_SUPPORT_NO_AES
  !endif
  !if $(AB_RETRYCOUNT_DISABLE)
      GCC:*_*_*_CC_FLAGS = -DAB_RETRYCOUNT_DISABLE
  !endif
  !if $(TARGET_BOARD_TYPE_AUTO)
      GCC:*_*_*_CC_FLAGS = -DTARGET_BOARD_TYPE_AUTO
  !endif
  !if $(VERITY_LE)
      GCC:*_*_*_CC_FLAGS = -DVERITY_LE
  !endif
  !if $(USER_BUILD_VARIANT) == 0
      GCC:*_*_*_CC_FLAGS = -DENABLE_UPDATE_PARTITIONS_CMDS -DENABLE_BOOT_CMD -DENABLE_DEVICE_CRITICAL_LOCK_UNLOCK_CMDS -DUSERDEBUG_BUILD_VARIANT -DASUS_USERDEBUG_BUILD
  !else
      GCC:*_*_*_CC_FLAGS = -DENABLE_UPDATE_PARTITIONS_CMDS -DENABLE_BOOT_CMD -DENABLE_DEVICE_CRITICAL_LOCK_UNLOCK_CMDS -DUSER_BUILD_VARIANT -DASUS_USER_BUILD
  !endif
  !if $(ENABLE_LE_VARIANT) == 1
      GCC:*_*_*_CC_FLAGS = -DENABLE_LE_VARIANT
  !endif

  !if $(ABL_FTM) == 1
      GCC:*_*_*_CC_FLAGS = -DABL_FTM
  !endif

  !if $(F2FS_BUILD) == y
      GCC:*_*_*_CC_FLAGS = -DF2FS_BUILD
  !endif

  !if $(WW_BUILD) == WW
      GCC:*_*_*_CC_FLAGS = -DWW_BUILD
  !endif

  !if $(OPEN_BUILD) == OPEN
      GCC:*_*_*_CC_FLAGS = -DOPEN_BUILD
  !endif

  !if $(CN_BUILD) == CN
      GCC:*_*_*_CC_FLAGS = -DCN_BUILD
  !endif

  !if $(ASUS_AI2205_BUILD) == 1
      GCC:*_*_*_CC_FLAGS = -DASUS_AI2205_BUILD -DASUS_BUILD
  !endif

  !if $(BUILD_SYSTEM_ROOT_IMAGE)
      GCC:*_*_*_CC_FLAGS = -DBUILD_SYSTEM_ROOT_IMAGE
  !endif
  !if $(DISABLE_PARALLEL_DOWNLOAD_FLASH) == 1
      GCC:*_*_*_CC_FLAGS = -DDISABLE_PARALLEL_DOWNLOAD_FLASH
  !endif
  !if $(BUILD_USES_RECOVERY_AS_BOOT)
      GCC:*_*_*_CC_FLAGS = -DBUILD_USES_RECOVERY_AS_BOOT
  !endif
  !ifdef $(INIT_BIN)
      GCC:*_*_*_CC_FLAGS = -DINIT_BIN='"$(INIT_BIN)"'
  !endif
  !if $(NAND_SQUASHFS_SUPPORT)
      GCC:*_*_*_CC_FLAGS = -DNAND_SQUASHFS_SUPPORT
  !endif
  !if $(VERIFIED_BOOT_ENABLED)
      GCC:*_*_*_CC_FLAGS = -DVERIFIED_BOOT_ENABLED
  !endif
  !if $(BASE_ADDRESS)
      GCC:*_*_*_CC_FLAGS = -DBASE_ADDRESS=$(BASE_ADDRESS)
  !endif
  !if $(LINUX_BOOT_CPU_SELECTION_ENABLED)
      GCC:*_*_*_CC_FLAGS = -DLINUX_BOOT_CPU_SELECTION_ENABLED
      GCC:*_*_*_CC_FLAGS = -DTARGET_LINUX_BOOT_CPU_ID=$(TARGET_LINUX_BOOT_CPU_ID)
  !endif

[PcdsFixedAtBuild.common]

# DEBUG_ASSERT_ENABLED       0x01
# DEBUG_PRINT_ENABLED        0x02
# DEBUG_CODE_ENABLED         0x04
# CLEAR_MEMORY_ENABLED       0x08
# ASSERT_BREAKPOINT_ENABLED  0x10
# ASSERT_DEADLOOP_ENABLED    0x20

  gEfiMdePkgTokenSpaceGuid.PcdDebugPropertyMask|0x2f
  gEfiMdePkgTokenSpaceGuid.PcdDebugPrintErrorLevel|0x80000042
  gEfiMdePkgTokenSpaceGuid.PcdReportStatusCodePropertyMask|0x06

################################################################################
#
# Components Section - list of all EDK II Modules needed by this Platform
#
################################################################################
[Components.common]

	QcomModulePkg/Application/LinuxLoader/LinuxLoader.inf {
		<LibraryClasses>
			DxeServicesTableLib|MdePkg/Library/DxeServicesTableLib/DxeServicesTableLib.inf
			UefiLib|MdePkg/Library/UefiLib/UefiLib.inf
			UefiApplicationEntryPoint|MdePkg/Library/UefiApplicationEntryPoint/UefiApplicationEntryPoint.inf
			CacheMaintenanceLib|ArmPkg/Library/ArmCacheMaintenanceLib/ArmCacheMaintenanceLib.inf
			Zlib|QcomModulePkg/Library/zlib/zlib.inf
			ArmLib|ArmPkg/Library/ArmLib/ArmBaseLib.inf
			BaseLib|MdePkg/Library/BaseLib/BaseLib.inf
			DevicePathLib|MdePkg/Library/UefiDevicePathLib/UefiDevicePathLib.inf
			HobLib|MdePkg/Library/DxeHobLib/DxeHobLib.inf
			PerformanceLib|MdeModulePkg/Library/DxePerformanceLib/DxePerformanceLib.inf
			DebugPrintErrorLevelLib|MdePkg/Library/BaseDebugPrintErrorLevelLib/BaseDebugPrintErrorLevelLib.inf

			FdtLib|EmbeddedPkg/Library/FdtLib/FdtLib.inf
			LibUfdt|EmbeddedPkg/Library/LibUfdt/LibUfdt.inf
			ArmSmcLib|ArmPkg/Library/ArmSmcLib/ArmSmcLib.inf
			BootLib|QcomModulePkg/Library/BootLib/BootLib.inf
			StackCanary|QcomModulePkg/Library/StackCanary/StackCanary.inf
			FastbootLib|QcomModulePkg/Library/FastbootLib/FastbootLib.inf
			AvbLib|QcomModulePkg/Library/avb/AvbLib.inf
			AesLib|QcomModulePkg/Library/aes/AesLib.inf
			UbsanLib|QcomModulePkg/Library/UbsanLib/UbsanLib.inf
	}
