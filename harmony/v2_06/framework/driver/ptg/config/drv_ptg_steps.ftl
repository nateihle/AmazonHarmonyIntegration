config DRV_PTG_STEP_INSTANCES_NUMBER_GT_${INSTANCE+1}
    bool
<#if INSTANCE != 0>
	default n if DRV_PTG_STEP_INSTANCES_NUMBER_GT_${INSTANCE} = n     
</#if>	
	default n if DRV_PTG_STEP_INSTANCES_NUMBER = ${INSTANCE+1}
	default y
	
config DRV_STEP_INST_${INSTANCE}
    depends on USE_DRV_PTG 
<#if INSTANCE != 0>
	             && DRV_PTG_STEP_INSTANCES_NUMBER_GT_${INSTANCE}
</#if>
    bool "STEP ${INSTANCE}"
    default y

ifblock DRV_STEP_INST_${INSTANCE}
	
config DRV_PTG_STEP_CMD_${INSTANCE}
	string "STEP ${INSTANCE} Command"
	depends on USE_DRV_PTG
    range DRV_PTG_CMD
    default "PTGCTRL"

ifblock DRV_PTG_STEP_CMD_${INSTANCE} = "PTGCTRL"
config DRV_PTG_STEP_PTGCTRL_PRM_${INSTANCE}
	string "STEP ${INSTANCE} Parameter" 
	depends on USE_DRV_PTG
	range PTGCTRL_PARAM
	default "NOP" 
endif

ifblock DRV_PTG_STEP_CMD_${INSTANCE} = "PTGADD_COPY"
config DRV_PTG_STEP_PTGADD_PRM_${INSTANCE}
	string "STEP ${INSTANCE} Parameter" 
	depends on USE_DRV_PTG
	range PTGADD_PARAM
	default "ADD_ADJ_C1" 
endif

ifblock (DRV_PTG_STEP_CMD_${INSTANCE} = "PTGSTRB")
config DRV_PTG_STEP_STRB_PRM_${INSTANCE}
	int "STEP ${INSTANCE} Parameter" 
	depends on USE_DRV_PTG
	range 0 31
	default 0
endif

ifblock (DRV_PTG_STEP_CMD_${INSTANCE} = "PTGJMP")
config DRV_PTG_STEP_JMP_PRM_${INSTANCE}
	int "STEP ${INSTANCE} Parameter" 
	depends on USE_DRV_PTG
	range 0 31
	default 0
endif

ifblock (DRV_PTG_STEP_CMD_${INSTANCE} = "PTGJMPC0")
config DRV_PTG_STEP_JMPC0_PRM_${INSTANCE}
	int "STEP ${INSTANCE} Parameter" 
	depends on USE_DRV_PTG
	range 0 31
	default 0
endif

ifblock (DRV_PTG_STEP_CMD_${INSTANCE} = "PTGJMPC1")
config DRV_PTG_STEP_JMPC1_PRM_${INSTANCE}
	int "STEP ${INSTANCE} Parameter" 
	depends on USE_DRV_PTG
	range 0 31
	default 0
endif

ifblock ((DRV_PTG_STEP_CMD_${INSTANCE} = "PTGWHI") || (DRV_PTG_STEP_CMD_${INSTANCE} = "PTGWLO"))
config DRV_PTG_STEP_PTGHILO_PRM_${INSTANCE}
	string "STEP ${INSTANCE} Parameter" 
	depends on USE_DRV_PTG
	range PTGHILO_PARAM
	default "WAIT_OCMP1" 
endif

ifblock (DRV_PTG_STEP_CMD_${INSTANCE} = "PTGIRQ")
config DRV_PTG_STEP_PTGIRQ_PRM_${INSTANCE}
	string "STEP ${INSTANCE} Parameter" 
	depends on USE_DRV_PTG
	range PTGIRQ_PARAM
	default "PTG_IRQ0" 
endif

ifblock (DRV_PTG_STEP_CMD_${INSTANCE} = "PTGTRIG")
config DRV_PTG_STEP_PTGTRIG_PRM_${INSTANCE}
	string "STEP ${INSTANCE} Parameter" 
	depends on USE_DRV_PTG
	range PTGTRIG_PARAM
	default "PTG_ADC_TRIG12" 
endif
endif	