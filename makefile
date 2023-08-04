UEFI_TOP_DIR := .

ifndef $(BOOTLOADER_OUT)
	BOOTLOADER_OUT := $(shell pwd)
endif
export $(BOOTLOADER_OUT)

BUILDDIR=$(shell pwd)
export WRAPPER := $(PREBUILT_PYTHON_PATH) $(BUILDDIR)/clang-wrapper.py
export MAKEPATH := $(MAKEPATH)

export CLANG35_BIN := $(CLANG_BIN)
export CLANG35_GCC_TOOLCHAIN := $(CLANG35_GCC_TOOLCHAIN)
export $(BOARD_BOOTLOADER_PRODUCT_NAME)

ifeq ($(TARGET_ARCHITECTURE),arm)
export ARCHITECTURE := ARM
export CLANG35_ARM_PREFIX := $(CLANG_PREFIX)
else
export ARCHITECTURE := AARCH64
export CLANG35_AARCH64_PREFIX := $(CLANG_PREFIX)
endif

export BUILD_REPORT_DIR := $(BOOTLOADER_OUT)/build_report
ABL_OUT := $(BOOTLOADER_OUT)/Build

WORKSPACE=$(BUILDDIR)
TARGET_TOOLS := CLANG35
TARGET := DEBUG
BUILD_ROOT := $(ABL_OUT)/$(TARGET)_$(TARGET_TOOLS)
EDK_TOOLS := $(BUILDDIR)/BaseTools
EDK_TOOLS_BIN := $(EDK_TOOLS)/Source/C/bin
ABL_FV_IMG := $(BUILD_ROOT)/FV/abl.fv
ABL_FV_ELF := $(BOOTLOADER_OUT)/../../unsigned_abl.elf
SHELL:=/bin/bash

EDK_TOOLS_SRC_FILE := $(shell find $(EDK_TOOLS) -name "*" -type f)
EDK_TOOLS_PATH_MARK_FILE := $(ABL_OUT)/BaseTools_Mark
EDK_TOOLS_GENERATE_CLEAN := $(ABL_OUT)/BaseTools_Clean
export TARGET_EDK_TOOLS_BIN := $(ABL_OUT)/Source/C/bin

define edk_tools_generate
  mkdir -p $(ABL_OUT)/Scripts
  cp -rf $(EDK_TOOLS)/Scripts/GccBase.lds $(ABL_OUT)/Scripts

  (. ./edksetup.sh BaseTools && \
  $(MAKEPATH)make -C $(EDK_TOOLS) $(PREBUILT_HOST_TOOLS) -j1)

  mkdir -p $(TARGET_EDK_TOOLS_BIN)
  cp -rf $(EDK_TOOLS_BIN)/* $(TARGET_EDK_TOOLS_BIN)
  touch $(EDK_TOOLS_PATH_MARK_FILE)
endef

# This function is to check version compatibility, used to control features based on the compiler version. \
Arguments should be return value, current version and supported version in order. \
It sets return value to true if the current version is equal or greater than the supported version.
define check_version_compatibility
	$(eval CURR_VERSION := $(shell $(2)/clang --version |& grep -i "clang version" |& sed 's/[^0-9.]//g'))
	$(eval CURR_VERSION_MAJOR := $(shell echo $(CURR_VERSION) |& cut -d. -f1))
	$(eval CURR_VERSION_MINOR := $(shell echo $(CURR_VERSION) |& cut -d. -f2))
	$(eval SUPPORTED_VERSION := $(3))
	$(eval SUPPORTED_VERSION_MAJOR := $(shell echo $(SUPPORTED_VERSION) |& cut -d. -f1))
	$(eval SUPPORTED_VERSION_MINOR := $(shell echo $(SUPPORTED_VERSION) |& cut -d. -f2))

	ifeq ($(shell expr $(CURR_VERSION_MAJOR) \> $(SUPPORTED_VERSION_MAJOR)), 1)
		$(1) := true
	endif
	ifeq ($(shell expr $(CURR_VERSION_MAJOR) \= $(SUPPORTED_VERSION_MAJOR)), 1)
		ifeq ($(shell expr $(CURR_VERSION_MINOR) \>= $(SUPPORTED_VERSION_MINOR)), 1)
			$(1) := true
		endif
	endif
endef

# UEFI UBSAN Configuration
# ENABLE_UEFI_UBSAN := true

ifeq "$(ENABLE_UEFI_UBSAN)" "true"
	UBSAN_GCC_FLAG_UNDEFINED := -fsanitize=undefined
	UBSAN_GCC_FLAG_ALIGNMENT := -fno-sanitize=alignment
else
	UBSAN_GCC_FLAG_UNDEFINED :=
	UBSAN_GCC_FLAG_ALIGNMENT :=
endif

ifeq ($(TARGET_ARCHITECTURE), arm)
	LOAD_ADDRESS := 0X8FB00000
else
	LOAD_ADDRESS := 0X9FA00000
endif

ifeq ($(ENABLE_LE_VARIANT), true)
	ENABLE_LE_VARIANT := 1
else
	ENABLE_LE_VARIANT := 0
endif

ifeq ($(EARLY_ETH_ENABLED), 1)
	EARLY_ETH_ENABLED := 1
else
	EARLY_ETH_ENABLED := 0
endif


ifeq "$(ABL_USE_SDLLVM)" "true"
	SDLLVM_COMPILE_ANALYZE := --compile-and-analyze
	SDLLVM_ANALYZE_REPORT := $(BUILD_REPORT_DIR)
else
	SDLLVM_COMPILE_ANALYZE :=
	SDLLVM_ANALYZE_REPORT :=
endif

ifneq "$(INIT_BIN_LE)" ""
	INIT_BIN := $(INIT_BIN_LE)
else
	INIT_BIN := "/init"
endif

ifeq "$(BASE_ADDRESS)" ""
	BASE_ADDRESS := 0x80000000
endif

#===================================================================
# ASUS build options begin
#===================================================================
ifeq "$(ASUS_BUILD_PROJECT)" "AI2205"
	ASUS_AI2205_BUILD := 1
	ASUS_COMMON_FUCTION := 1
else
	ASUS_AI2205_BUILD := 0
endif

ifeq "$(ASUS_FTM)" "y"
	ABL_FTM := 1
endif

ifeq "$(F2FS_BUILD)" "y"
	F2FS_BUILD := 1
endif

$(info makefile ASUS_BUILD_PROJECT=$(ASUS_BUILD_PROJECT) \
                ASUS_AI2205_BUILD=$(ASUS_AI2205_BUILD) \
                ASUS_FTM=$(ASUS_FTM) \
                ABL_FTM=$(ABL_FTM))
#===================================================================
# ASUS build options end
#===================================================================


ifeq "$(TARGET_LINUX_BOOT_CPU_SELECTION)" "true"
	LINUX_BOOT_CPU_SELECTION_ENABLED := 1
else
	LINUX_BOOT_CPU_SELECTION_ENABLED := 0
endif

ifeq "$(TARGET_LINUX_BOOT_CPU_ID)" ""
	TARGET_LINUX_BOOT_CPU_ID := 0
endif

export SDLLVM_COMPILE_ANALYZE := $(SDLLVM_COMPILE_ANALYZE)
export SDLLVM_ANALYZE_REPORT := $(SDLLVM_ANALYZE_REPORT)

CLANG_SUPPORTS_SAFESTACK := false
$(eval $(call check_version_compatibility, CLANG_SUPPORTS_SAFESTACK, $(CLANG_BIN), $(SAFESTACK_SUPPORTED_CLANG_VERSION)))

ifeq "$(ABL_SAFESTACK)" "true"
	ifeq "$(CLANG_SUPPORTS_SAFESTACK)" "true"
		LLVM_ENABLE_SAFESTACK := -fsanitize=safe-stack
		LLVM_SAFESTACK_USE_PTR := -mllvm -safestack-use-pointer-address
		LLVM_SAFESTACK_COLORING := -mllvm -safe-stack-coloring=true
	endif
else
	LLVM_ENABLE_SAFESTACK :=
	LLVM_SAFESTACK_USE_PTR :=
	LLVM_SAFESTACK_COLORING :=
endif
export LLVM_ENABLE_SAFESTACK := $(LLVM_ENABLE_SAFESTACK)
export LLVM_SAFESTACK_USE_PTR := $(LLVM_SAFESTACK_USE_PTR)
export LLVM_SAFESTACK_COLORING := $(LLVM_SAFESTACK_COLORING)

.PHONY: all cleanall

all: ABL_FV_ELF

cleanall:
	@. ./edksetup.sh BaseTools && \
	build -p $(WORKSPACE)/QcomModulePkg/QcomModulePkg.dsc -a $(ARCHITECTURE) -t $(TARGET_TOOLS) -b $(TARGET) -j build_modulepkg.log cleanall
	rm -rf $(WORKSPACE)/QcomModulePkg/Bin64
	rm -rf $(TARGET_EDK_TOOLS_BIN)

$(EDK_TOOLS_PATH_MARK_FILE): $(EDK_TOOLS_SRC_FILE)
	@$(call edk_tools_generate)

ABL_FV_IMG: $(EDK_TOOLS_PATH_MARK_FILE)
	@. ./edksetup.sh BaseTools && \
	build -p $(WORKSPACE)/QcomModulePkg/QcomModulePkg.dsc \
	-a $(ARCHITECTURE) \
	-t $(TARGET_TOOLS) \
	-b $(TARGET) \
	-D ABL_OUT_DIR=$(ABL_OUT) \
	-D VERIFIED_BOOT_LE=$(VERIFIED_BOOT_LE) \
	-D VERIFIED_BOOT_ENABLED=$(VERIFIED_BOOT_ENABLED) \
	-D EARLY_ETH_ENABLED=$(EARLY_ETH_ENABLED) \
	-D HIBERNATION_SUPPORT_NO_AES=$(HIBERNATION_SUPPORT_NO_AES) \
	-D AB_RETRYCOUNT_DISABLE=$(AB_RETRYCOUNT_DISABLE) \
	-D TARGET_BOARD_TYPE_AUTO=$(TARGET_BOARD_TYPE_AUTO) \
	-D VERITY_LE=$(VERITY_LE) \
	-D USER_BUILD_VARIANT=$(USER_BUILD_VARIANT) \
	-D DISABLE_PARALLEL_DOWNLOAD_FLASH=$(DISABLE_PARALLEL_DOWNLOAD_FLASH) \
	-D ENABLE_LE_VARIANT=$(ENABLE_LE_VARIANT) \
	-D BUILD_USES_RECOVERY_AS_BOOT=$(BUILD_USES_RECOVERY_AS_BOOT) \
	-D INIT_BIN=$(INIT_BIN) \
	-D UBSAN_UEFI_GCC_FLAG_UNDEFINED=$(UBSAN_GCC_FLAG_UNDEFINED) \
	-D UBSAN_UEFI_GCC_FLAG_ALIGNMENT=$(UBSAN_GCC_FLAG_ALIGNMENT) \
	-D NAND_SQUASHFS_SUPPORT=$(NAND_SQUASHFS_SUPPORT) \
	-D BASE_ADDRESS=$(BASE_ADDRESS) \
	-D F2FS_BUILD=$(F2FS_BUILD) \
	-D ABL_FTM=$(ABL_FTM) \
	-D WW_BUILD=$(TARGET_SKU) \
	-D OPEN_BUILD=$(TARGET_SKU) \
	-D CN_BUILD=$(TARGET_SKU) \
	-D ASUS_USER_BUILD=$(ASUS_USER_BUILD) \
	-D ASUS_USERDEBUG_BUILD=$(ASUS_USERDEBUG_BUILD) \
	-D ASUS_AI2205_BUILD=$(ASUS_AI2205_BUILD) \
	-D LINUX_BOOT_CPU_SELECTION_ENABLED=$(LINUX_BOOT_CPU_SELECTION_ENABLED) \
	-D TARGET_LINUX_BOOT_CPU_ID=$(TARGET_LINUX_BOOT_CPU_ID) \
	-j build_modulepkg.log $*

	cp $(BUILD_ROOT)/FV/FVMAIN_COMPACT.Fv $(ABL_FV_IMG)

$(EDK_TOOLS_GENERATE_CLEAN): $(EDK_TOOLS_PATH_MARK_FILE)
	@$(MAKEPATH)make -C $(BUILDDIR)/BaseTools/Source/C clean > /dev/null
	touch $(EDK_TOOLS_GENERATE_CLEAN)

BASETOOLS_CLEAN: ABL_FV_IMG
	@rm -rf $(BUILDDIR)/Conf/BuildEnv.sh

ABL_FV_ELF: BASETOOLS_CLEAN $(EDK_TOOLS_GENERATE_CLEAN)
	python3 $(WORKSPACE)/QcomModulePkg/Tools/image_header.py $(ABL_FV_IMG) $(ABL_FV_ELF) $(LOAD_ADDRESS) elf 32 nohash
