/*
 * Copyright (c) 2024, Microchip
 *
 * SPDX-License-Identifier: Apache-2.0
 */

GROUP_START(FLASH_BCFG)

SECTION_PROLOGUE(.mchp_bcfg_altcfg,0x45e88,)
{
    . = ALIGN(4);
    KEEP(*(.mchp_bcfg_altcfg))
}
SECTION_PROLOGUE(.mchp_bcfg_altcpn,0x45ebc,)
{
    . = ALIGN(4);
    KEEP(*(.mchp_bcfg_altcpn))
}
SECTION_PROLOGUE(.mchp_bcfg_mark1,0x45edc,)
{
    . = ALIGN(4);
    KEEP(*(.mchp_bcfg_mark1))
}
SECTION_PROLOGUE(.mchp_bcfg_devcfg,0x45f88,)
{
    . = ALIGN(4);
    KEEP(*(.mchp_bcfg_devcfg))
}
SECTION_PROLOGUE(.mchp_bcfg_cpn,0x45fbc,)
{
    . = ALIGN(4);
    KEEP(*(.mchp_bcfg_cpn))
}
SECTION_PROLOGUE(.mchp_bcfg_mark2,0x45fdc,)
{
    . = ALIGN(4);
    KEEP(*(.mchp_bcfg_mark2))
}

GROUP_END(FLASH_BCFG)

#ifdef CONFIG_BT
SECTION_PROLOGUE(.mchp_bt,0x20018C00(NOLOAD),)
{
    . = ALIGN(4);
    KEEP(*(.mchp_bt))
}
#endif
SECTION_PROLOGUE (.pbss,0x44014000, )
{
    KEEP(*(SORT_BY_NAME(".pbss*")))
}

