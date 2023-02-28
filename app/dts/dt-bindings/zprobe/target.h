/*
 * Copyright (c) 2023 XiNGRZ
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#define CREATE_FAMILY_ID(vendor, family) ((vendor) << 8 | (family))

#define VENDOR_ID_STUB	  0
#define VENDOR_ID_NXP	  11
#define VENDOR_ID_TI	  16
#define VENDOR_ID_MAXIM	  23
#define VENDOR_ID_NORDIC  54
#define VENDOR_ID_TOSHIBA 92
#define VENDOR_ID_RENESAS 117
#define VENDOR_ID_AMBIQ	  120
#define VENDOR_ID_REALTEK 124

#define FAMILY_ID_STUB_HWRESET	    CREATE_FAMILY_ID(VENDOR_ID_STUB, 1)
#define FAMILY_ID_STUB_SWVECTRESET  CREATE_FAMILY_ID(VENDOR_ID_STUB, 2)
#define FAMILY_ID_STUB_SWSYSRESET   CREATE_FAMILY_ID(VENDOR_ID_STUB, 3)
#define FAMILY_ID_NXP_KINETISK	    CREATE_FAMILY_ID(VENDOR_ID_NXP, 1)
#define FAMILY_ID_NXP_KINETISL	    CREATE_FAMILY_ID(VENDOR_ID_NXP, 2)
#define FAMILY_ID_NXP_MIMXRT	    CREATE_FAMILY_ID(VENDOR_ID_NXP, 3)
#define FAMILY_ID_NXP_RAPIDIOT	    CREATE_FAMILY_ID(VENDOR_ID_NXP, 4)
#define FAMILY_ID_NXP_KINETISK32W   CREATE_FAMILY_ID(VENDOR_ID_NXP, 5)
#define FAMILY_ID_NXP_LPC55XX	    CREATE_FAMILY_ID(VENDOR_ID_NXP, 6)
#define FAMILY_ID_NORDIC_NRF51	    CREATE_FAMILY_ID(VENDOR_ID_NORDIC, 1)
#define FAMILY_ID_NORDIC_NRF51	    CREATE_FAMILY_ID(VENDOR_ID_NORDIC, 2)
#define FAMILY_ID_REALTEK_RTL8195AM CREATE_FAMILY_ID(VENDOR_ID_REALTEK, 1)
#define FAMILY_ID_TI_CC3220SF	    CREATE_FAMILY_ID(VENDOR_ID_TI, 1)
#define FAMILY_ID_TOSHIBA_TZ	    CREATE_FAMILY_ID(VENDOR_ID_TOSHIBA, 1)
#define FAMILY_ID_RENESAS	    CREATE_FAMILY_ID(VENDOR_ID_RENESAS, 1)
#define FAMILY_ID_AMBIQ_AMA3B1KK    CREATE_FAMILY_ID(VENDOR_ID_AMBIQ, 1)
#define FAMILY_ID_MAXIM_MAX3262X    CREATE_FAMILY_ID(VENDOR_ID_MAXIM, 1)
#define FAMILY_ID_MAXIM_MAX3266X    CREATE_FAMILY_ID(VENDOR_ID_MAXIM, 2)
