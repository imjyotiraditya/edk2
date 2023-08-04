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

#include "TLMM.h"

EFI_STATUS gpio_tlmm_config(UINT32 gpio,UINT8 func,UINT8 dir,UINT8 pull,UINT8 drive)
{
	EFI_STATUS Status = EFI_SUCCESS;
	//Handle to the TLMM DXE protocol API.
	EFI_TLMM_PROTOCOL * TLMMProtocol = NULL;
	UINT32 config;

	DEBUG((EFI_D_INFO, "[XBL] +++ gpio_tlmm_config(%d)\n", gpio));

	/* Access the TLMM protocol. */
	Status = gBS->LocateProtocol( &gEfiTLMMProtocolGuid, NULL, (void**)&TLMMProtocol);

	if (Status != EFI_SUCCESS)
	{
		DEBUG((EFI_D_ERROR,"[XBL] Input Open TLMMProtocol failed Status:%r \n", Status));
		return Status;
	}

	if(TLMMProtocol)
	{
		config = EFI_GPIO_CFG(gpio, func, dir, pull, drive);

		if (EFI_SUCCESS !=TLMMProtocol->ConfigGpio((UINT32)config,TLMM_GPIO_ENABLE))
		{
			DEBUG((EFI_D_ERROR, "[XBL] Error configuring TLMM GPIO%d\n", gpio));
			return Status;
		}
	}

	DEBUG((EFI_D_INFO, "[XBL] --- gpio_tlmm_config(%d)\n", gpio));

	return Status;
}

int gpio_status(UINT32 gpio,UINT8 func,UINT8 dir,UINT8 pull,UINT8 drive)
{
	int Status = EFI_SUCCESS;
	UINT32 config = 0;
	UINT32 value = GPIO_HIGH_VALUE;
	//Handle to the TLMM DXE protocol API.
	EFI_TLMM_PROTOCOL * TLMMProtocol = NULL;

	/* Access the TLMM protocol. */
	Status = gBS->LocateProtocol( &gEfiTLMMProtocolGuid, NULL, (void**)&TLMMProtocol);

	if (Status != EFI_SUCCESS)
	{
		DEBUG((EFI_D_ERROR,"[XBL] gpio_status :  LocateProtocol failed Status:%r \n", Status));
		return Status;
	}

	if(TLMMProtocol)
	{
		config = EFI_GPIO_CFG(gpio, func, dir, pull, drive);

		if (EFI_SUCCESS !=TLMMProtocol->ConfigGpio((UINT32)config,TLMM_GPIO_ENABLE))
		{
			DEBUG((EFI_D_ERROR, "[XBL]: gpio_status : Error configuring TLMM GPIO%d\n", gpio));
			return Status;
		}
	}

	Status = TLMMProtocol->GpioIn(config, &value);

	if (Status != EFI_SUCCESS)
	{
		DEBUG((EFI_D_ERROR,"[XBL] gpio_status :  GpioIn failed Status:%r \n", Status));
		return Status;
	}

	DEBUG((EFI_D_INFO, "[XBL] --- gpio_status(%d)\n", gpio));

	return value;
}
