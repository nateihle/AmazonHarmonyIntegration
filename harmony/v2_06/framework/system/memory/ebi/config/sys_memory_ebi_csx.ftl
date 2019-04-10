config SYS_MEMORY_EBI_CS_NUMBER_GT_${INSTANCE+1}
    bool
    depends on USE_SYS_MEMORY_EBI
<#if INSTANCE != 0>
	default n if SYS_MEMORY_EBI_CS_NUMBER_GT_${INSTANCE} = n
</#if>
	default n if SYS_MEMORY_EBI_CS_NUMBER = ${INSTANCE+1}
	default y

config SYS_MEMORY_EBI_CSX${INSTANCE}
    depends on USE_SYS_MEMORY_EBI
<#if INSTANCE != 0>
	             && SYS_MEMORY_EBI_CS_NUMBER_GT_${INSTANCE}
</#if>
    bool "EBI Chip Select Instance${INSTANCE}"
    default y
    ---help---
    IDH_HTML_PLIB_EBI_ChipSelectEnableSet_EBI_MODULE_ID_bool_bool_bool_bool
    ---endhelp---

ifblock SYS_MEMORY_EBI_CSX${INSTANCE}

config SYS_MEMORY_EBI_CS_NUM_CSX${INSTANCE}
    string "Select CS Pin"
    depends on USE_SYS_MEMORY_EBI
    depends on SYS_MEMORY_EBI_CSX${INSTANCE}
    range SYS_MEMORY_EBI_CS_NUM
    default "/EBICS0"
	---help---
	EBI Chip Select Pin
	---endhelp---

config SYS_MEMORY_EBI_CS_NUMBER_CSX${INSTANCE}
    int
    depends on USE_SYS_MEMORY_EBI
    depends on SYS_MEMORY_EBI_CSX${INSTANCE}
    default 0 if SYS_MEMORY_EBI_CS_NUM_CSX${INSTANCE} = "/EBICS0"
    default 1 if SYS_MEMORY_EBI_CS_NUM_CSX${INSTANCE} = "/EBICS1"
    default 2 if SYS_MEMORY_EBI_CS_NUM_CSX${INSTANCE} = "/EBICS2"
    default 3 if SYS_MEMORY_EBI_CS_NUM_CSX${INSTANCE} = "/EBICS3"

config SYS_MEMORY_EBI_BASE_ADDR_CSX${INSTANCE}
    string "External Memory Base Address"
    depends on USE_SYS_MEMORY_EBI
    depends on SYS_MEMORY_EBI_CSX${INSTANCE}
    default "0x20000000"
    ---help---
    IDH_HTML_PLIB_EBI_BaseAddressSet_EBI_MODULE_ID_int_uint32_t
    ---endhelp---

config SYS_MEMORY_EBI_MEMORY_SIZE_CSX${INSTANCE}
    string "External Memory Size"
    depends on USE_SYS_MEMORY_EBI
    depends on SYS_MEMORY_EBI_CSX${INSTANCE}
    range EBI_MEMORY_SIZE
    default "MEMORY_SIZE_2MB"
    ---help---
    IDH_HTML_EBI_MEMORY_SIZE
    ---endhelp---

config SYS_MEMORY_EBI_MEMORY_TYPE_CSX${INSTANCE}
    string "External Memory Type"
    depends on USE_SYS_MEMORY_EBI
    depends on SYS_MEMORY_EBI_CSX${INSTANCE}
    range EBI_MEMORY_TYPE
    default "SRAM"
    ---help---
    IDH_HTML_EBI_MEMORY_TYPE
    ---endhelp---

config SYS_MEMORY_EBI_CS_TIMING_CSX${INSTANCE}
    string "EBI Timing Register"
    depends on USE_SYS_MEMORY_EBI
    depends on SYS_MEMORY_EBI_CSX${INSTANCE}
    range EBI_CS_TIMING
    default "CS_TIMING_0"
    ---help---
    IDH_HTML_EBI_CS_TIMING
    ---endhelp---

endif
