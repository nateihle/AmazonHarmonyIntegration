config DRV_SRAM_INSTANCES_NUMBER_GT_${INSTANCE+1}
    depends on USE_DRV_SRAM
    bool
<#if INSTANCE != 0>
	default n if DRV_SRAM_INSTANCES_NUMBER_GT_${INSTANCE} = n     
</#if>	
	default n if DRV_SRAM_INSTANCES_NUMBER = ${INSTANCE+1}
	default y
	
config DRV_SRAM_INST_IDX${INSTANCE}
    depends on USE_DRV_SRAM
<#if INSTANCE != 0>
	             && DRV_SRAM_INSTANCES_NUMBER_GT_${INSTANCE}
</#if>
    bool "SRAM Driver Instance ${INSTANCE}"
    default y
    ---help---
    IDH_HTML_DRV_SRAM_INSTANCES_NUMBER
    ---endhelp---

ifblock DRV_SRAM_INST_IDX${INSTANCE}

config DRV_SRAM_MEDIA_START_ADDRESS_IDX${INSTANCE}
    string "Media Start address"
    depends on USE_DRV_SRAM
    default "SRAM_MEDIA_0_DATA" if ${INSTANCE} = 0
    default "SRAM_MEDIA_1_DATA" if ${INSTANCE} = 1
    ---help---
      IDH_HTML_DRV_SRAM_MEDIA_SIZE
    ---endhelp---
    
config DRV_SRAM_MEDIA_SIZE_IDX${INSTANCE}
    int "Media Size in KiloBytes"
    depends on USE_DRV_SRAM
    default 32
    ---help---
      IDH_HTML_DRV_SRAM_MEDIA_SIZE
    ---endhelp---
    
config DRV_SRAM_MEDIA_BLOCK_SIZE_IDX${INSTANCE}
    string "Media block Size in bytes"
    depends on USE_DRV_SRAM
    range DRV_SRAM_MEDIA_BLOCK_SIZE
    default 1
    ---help---
      IDH_HTML_DRV_SRAM_MEDIA_BLOCK_SIZE
    ---endhelp---

config USE_DRV_SRAM_SYS_FS_REGISTER_IDX${INSTANCE}
    bool "Register with File System?"
    depends on USE_DRV_SRAM
    select USE_SYS_FS_NEEDED
    select DRIVER
    default n
    ---help---
      IDH_HTML_DRV_SRAM_SYS_FS_REGISTER
    ---endhelp---

endif

