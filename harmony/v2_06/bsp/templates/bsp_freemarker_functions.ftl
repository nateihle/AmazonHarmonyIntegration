<#macro str2hex str reg>
	<#assign REG_A_HEX = []>
		<#if str == "0000">
			<#assign REG_A_HEX = ["0"]>
		</#if>
		<#if str == "0001">
			<#assign REG_A_HEX = ["1"]>
		</#if>
		<#if str == "0010">
			<#assign REG_A_HEX = ["2"]>
		</#if>
		<#if str == "0011">
			<#assign REG_A_HEX = ["3"]>
		</#if>
		<#if str == "0100">
			<#assign REG_A_HEX = ["4"]>
		</#if>
		<#if str == "0101">
			<#assign REG_A_HEX = ["5"]>
		</#if>
		<#if str == "0110">
			<#assign REG_A_HEX = ["6"]>
		</#if>
		<#if str == "0111">
			<#assign REG_A_HEX = ["7"]>
		</#if>
		<#if str == "1000">
			<#assign REG_A_HEX = ["8"]>
		</#if>
		<#if str == "1001">
			<#assign REG_A_HEX = ["9"]>
		</#if>
		<#if str == "1010">
			<#assign REG_A_HEX = ["A"]>
		</#if>
		<#if str == "1011">
			<#assign REG_A_HEX = ["B"]>
		</#if>
		<#if str == "1100">
			<#assign REG_A_HEX = ["C"]>
		</#if>
		<#if str == "1101">
			<#assign REG_A_HEX = ["D"]>
		</#if>
		<#if str == "1110">
			<#assign REG_A_HEX = ["E"]>
		</#if>
		<#if str == "1111">
			<#assign REG_A_HEX = ["F"]>
		</#if>
		<#if reg == "MODE">
			<#assign MODE_REG_A_HEX_STR = (REG_A_HEX?join(""))>
		</#if>
		<#if reg == "TRIS">
			<#assign TRIS_REG_A_HEX_STR = (REG_A_HEX?join(""))>
		</#if>
		<#if reg == "LAT">
			<#assign LAT_REG_A_HEX_STR = (REG_A_HEX?join(""))>
		</#if>
		<#if reg == "OD">
			<#assign OD_REG_A_HEX_STR = (REG_A_HEX?join(""))>
		</#if>
		<#if reg == "CNEN">
			<#assign CNEN_REG_A_HEX_STR = (REG_A_HEX?join(""))>
		</#if>
		<#if reg == "CNPU">
			<#assign CNPU_REG_A_HEX_STR = (REG_A_HEX?join(""))>
		</#if>
		<#if reg == "CNPD">
			<#assign CNPD_REG_A_HEX_STR = (REG_A_HEX?join(""))>
		</#if>
</#macro>
<#--  =====================
      MACRO mhc_process_leds
      ===================== -->
<#macro mhc_process_leds>
<#assign LED_Name_List = []>
<#assign LED_PortPin_List = []>
<#assign LED_PortChannel_List = []>
<#assign LED_ActiveLevel_List = []>
<#list 1..350 as i>
<#assign functype = "CONFIG_BSP_PIN_" + i + "_FUNCTION_TYPE">
<#if .vars[functype]?has_content>
<#if (.vars[functype] == "LED_AH") || (.vars[functype] == "LED_AL")>
<#assign funcname = "CONFIG_BSP_PIN_" + i + "_FUNCTION_NAME">
<#if .vars[funcname]?has_content>
<#assign pinport = "CONFIG_BSP_PIN_" + i + "_PORT_PIN">
<#if .vars[pinport]?has_content>
<#assign pinchannel = "CONFIG_BSP_PIN_" + i + "_PORT_CHANNEL">
<#if .vars[pinchannel]?has_content>
<#assign LED_Name_List = LED_Name_List + [.vars[funcname]]>
<#assign LED_PortPin_List = LED_PortPin_List + [.vars[pinport]]>
<#assign LED_PortChannel_List = LED_PortChannel_List + [.vars[pinchannel]]>
<#if (.vars[functype] == "LED_AH")>
<#assign LED_ActiveLevel_List = LED_ActiveLevel_List + ["High"]>
<#else>
<#assign LED_ActiveLevel_List = LED_ActiveLevel_List + ["Low"]>
</#if>
</#if>
</#if>
</#if>
</#if>
</#if>
</#list>
</#macro>
<#--  =====================
      MACRO mhc_process_switches
      ===================== -->
<#macro mhc_process_switches>
<#assign Switch_Name_List = []>
<#assign Switch_PortPin_List = []>
<#assign Switch_PortChannel_List = []>
<#list 1..350 as i>
<#assign functype = "CONFIG_BSP_PIN_" + i + "_FUNCTION_TYPE">
<#if .vars[functype]?has_content>
<#if .vars[functype] == "SWITCH">
<#assign funcname = "CONFIG_BSP_PIN_" + i + "_FUNCTION_NAME">
<#if .vars[funcname]?has_content>
<#assign pinport = "CONFIG_BSP_PIN_" + i + "_PORT_PIN">
<#if .vars[pinport]?has_content>
<#assign pinchannel = "CONFIG_BSP_PIN_" + i + "_PORT_CHANNEL">
<#if .vars[pinchannel]?has_content>
<#assign Switch_Name_List = Switch_Name_List + [.vars[funcname]]>
<#assign Switch_PortPin_List = Switch_PortPin_List + [.vars[pinport]]>
<#assign Switch_PortChannel_List = Switch_PortChannel_List + [.vars[pinchannel]]>
</#if>
</#if>
</#if>
</#if>
</#if>
</#list>
</#macro>
<#--  =====================
      MACRO mhc_process_vbus
      ===================== -->
<#macro mhc_process_vbus>
<#assign VBUS_PortPin_List = []>
<#assign VBUS_PortChannel_List = []>
<#assign VBUS_ActiveLevel_List = []>
<#list 1..350 as i>
<#assign functype = "CONFIG_BSP_PIN_" + i + "_FUNCTION_TYPE">
<#if .vars[functype]?has_content>
<#if (.vars[functype] == "VBUS_AH") || (.vars[functype] == "VBUS_AL") || (.vars[functype] == "VBUS")>
<#assign pinport = "CONFIG_BSP_PIN_" + i + "_PORT_PIN">
<#if .vars[pinport]?has_content>
<#assign pinchannel = "CONFIG_BSP_PIN_" + i + "_PORT_CHANNEL">
<#if .vars[pinchannel]?has_content>
<#assign VBUS_PortPin_List = VBUS_PortPin_List + [.vars[pinport]]>
<#assign VBUS_PortChannel_List = VBUS_PortChannel_List + [.vars[pinchannel]]>
<#if (.vars[functype] == "VBUS_AL")>
<#assign VBUS_ActiveLevel_List = VBUS_ActiveLevel_List + ["Low"]>
<#else>
<#assign VBUS_ActiveLevel_List = VBUS_ActiveLevel_List + ["High"]>
</#if>
</#if>
</#if>
</#if>
</#if>
</#list>
</#macro>
<#--  =====================
      MACRO mhc_process_gpio_out
      ===================== -->
<#macro mhc_process_gpio_out>
<#assign GPIO_OUT_Name_List = []>
<#assign GPIO_OUT_PortPin_List = []>
<#assign GPIO_OUT_PortChannel_List = []>
<#list 1..350 as i>
<#assign functype = "CONFIG_BSP_PIN_" + i + "_FUNCTION_TYPE">
<#if .vars[functype]?has_content>
<#if .vars[functype] == "GPIO_OUT">
<#assign funcname = "CONFIG_BSP_PIN_" + i + "_FUNCTION_NAME">
<#if .vars[funcname]?has_content>
<#assign pinport = "CONFIG_BSP_PIN_" + i + "_PORT_PIN">
<#if .vars[pinport]?has_content>
<#assign pinchannel = "CONFIG_BSP_PIN_" + i + "_PORT_CHANNEL">
<#if .vars[pinchannel]?has_content>
<#assign GPIO_OUT_Name_List = GPIO_OUT_Name_List + [.vars[funcname]]>
<#assign GPIO_OUT_PortPin_List = GPIO_OUT_PortPin_List + [.vars[pinport]]>
<#assign GPIO_OUT_PortChannel_List = GPIO_OUT_PortChannel_List + [.vars[pinchannel]]>
</#if>
</#if>
</#if>
</#if>
</#if>
</#list>
</#macro>
<#--  =====================
      MACRO mhc_process_gpio_in
      ===================== -->
<#macro mhc_process_gpio_in>
<#assign GPIO_IN_Name_List = []>
<#assign GPIO_IN_PortPin_List = []>
<#assign GPIO_IN_PortChannel_List = []>
<#list 1..350 as i>
<#assign functype = "CONFIG_BSP_PIN_" + i + "_FUNCTION_TYPE">
<#if .vars[functype]?has_content>
<#if .vars[functype] == "GPIO_IN">
<#assign funcname = "CONFIG_BSP_PIN_" + i + "_FUNCTION_NAME">
<#if .vars[funcname]?has_content>
<#assign pinport = "CONFIG_BSP_PIN_" + i + "_PORT_PIN">
<#if .vars[pinport]?has_content>
<#assign pinchannel = "CONFIG_BSP_PIN_" + i + "_PORT_CHANNEL">
<#if .vars[pinchannel]?has_content>
<#assign GPIO_IN_Name_List = GPIO_IN_Name_List + [.vars[funcname]]>
<#assign GPIO_IN_PortPin_List = GPIO_IN_PortPin_List + [.vars[pinport]]>
<#assign GPIO_IN_PortChannel_List = GPIO_IN_PortChannel_List + [.vars[pinchannel]]>
</#if>
</#if>
</#if>
</#if>
</#if>
</#list>
</#macro>
<#--  =====================
      MACRO mhc_process_gpio
      ===================== -->
<#macro mhc_process_gpio>
<#assign GPIO_Name_List = []>
<#assign GPIO_PortPin_List = []>
<#assign GPIO_PortChannel_List = []>
<#list 1..350 as i>
<#assign functype = "CONFIG_BSP_PIN_" + i + "_FUNCTION_TYPE">
<#if .vars[functype]?has_content>
<#if .vars[functype] == "GPIO">
<#assign funcname = "CONFIG_BSP_PIN_" + i + "_FUNCTION_NAME">
<#if .vars[funcname]?has_content>
<#assign pinport = "CONFIG_BSP_PIN_" + i + "_PORT_PIN">
<#if .vars[pinport]?has_content>
<#assign pinchannel = "CONFIG_BSP_PIN_" + i + "_PORT_CHANNEL">
<#if .vars[pinchannel]?has_content>
<#assign GPIO_Name_List = GPIO_Name_List + [.vars[funcname]]>
<#assign GPIO_PortPin_List = GPIO_PortPin_List + [.vars[pinport]]>
<#assign GPIO_PortChannel_List = GPIO_PortChannel_List + [.vars[pinchannel]]>
</#if>
</#if>
</#if>
</#if>
</#if>
</#list>
</#macro>
<#--  =====================
      MACRO mhc_process_port
      ===================== -->
<#macro mhc_process_port port>
<#assign MODEA_List = []>
<#assign MODEA_List_Sorted = []>
<#assign TRISA_List = []>
<#assign TRISA_List_Sorted = []>
<#assign LATA_List = []>
<#assign LATA_List_Sorted = []>
<#assign ODA_List = []>
<#assign ODA_List_Sorted = []>
<#assign CNENA_List = []>
<#assign CNENA_List_Sorted = []>
<#assign CNPUA_List = []>
<#assign CNPUA_List_Sorted = []>
<#assign CNPDA_List = []>
<#assign CNPDA_List_Sorted = []>
<#assign BitPositionA_List = []>
<#list 1..350 as i>
    <#assign portchannel = "CONFIG_BSP_PIN_" + i + "_PORT_CHANNEL">
    <#if .vars[portchannel]?has_content>
        <#if .vars[portchannel] == port>
            <#assign bitposition  = "CONFIG_BSP_PIN_" + i + "_PORT_PIN">
            <#assign BitPositionA_List = BitPositionA_List + [.vars[bitposition]]>
            <#assign mode = "CONFIG_BSP_PIN_" + i + "_MODE">
            <#if .vars[mode]?has_content>
                <#assign MODEA_List = MODEA_List + [.vars[mode]]>
		    <#else>
                <#assign MODEA_List = MODEA_List + ["ERROR"]>
            </#if>
            <#assign tris = "CONFIG_BSP_PIN_" + i + "_DIR">
            <#if .vars[tris]?has_content>
                <#assign TRISA_List = TRISA_List + [.vars[tris]]>
		    <#else>
                <#assign TRISA_List = TRISA_List + ["IN"]>
            </#if>
            <#assign lat = "CONFIG_BSP_PIN_" + i + "_LAT">
            <#if .vars[lat]?has_content>
                <#assign LATA_List = LATA_List + [.vars[lat]]>
		    <#else>
                <#assign LATA_List = LATA_List + ["LOW"]>
            </#if>
            <#assign od = "CONFIG_BSP_PIN_" + i + "_OD">
            <#if .vars[od]?has_content>
                <#assign ODA_List = ODA_List + [.vars[od]]>
		    <#else>
                <#assign ODA_List = ODA_List + ["FALSE"]>
            </#if>
            <#assign cnen = "CONFIG_BSP_PIN_" + i + "_CN">
            <#if .vars[cnen]?has_content>
                <#assign CNENA_List = CNENA_List + [.vars[cnen]]>
		    <#else>
                <#assign CNENA_List = CNENA_List + ["FALSE"]>
            </#if>
            <#assign cnpu = "CONFIG_BSP_PIN_" + i + "_PU">
            <#if .vars[cnpu]?has_content>
                <#assign CNPUA_List = CNPUA_List + [.vars[cnpu]]>
		    <#else>
                <#assign CNPUA_List = CNPUA_List + ["FALSE"]>
            </#if>
            <#assign cnpd = "CONFIG_BSP_PIN_" + i + "_PD">
            <#if .vars[cnpd]?has_content>
                <#assign CNPDA_List = CNPDA_List + [.vars[cnpd]]>
		    <#else>
                <#assign CNPDA_List = CNPDA_List + ["FALSE"]>
            </#if>
        </#if>
    </#if>
</#list>
<#if (BitPositionA_List?size == 0)>
    <#return>
</#if>
<#-- MODE -->
<#list 0..15 as i>
	<#assign InsertFlag = 0>
	<#list BitPositionA_List as bitpositiona>
		<#if bitpositiona?has_content>
			<#if bitpositiona?number == i>
				<#list MODEA_List as modea>
					<#if bitpositiona?index == modea?index>
            			<#assign MODEA_List_Sorted = MODEA_List_Sorted + [modea]>
						<#assign InsertFlag = 1>
    				</#if>
				</#list>
    		</#if>
    	</#if>
	</#list>
	<#if InsertFlag == 0>
        <#assign MODEA_List_Sorted = MODEA_List_Sorted + ["ERROR"]>
	</#if>
</#list>
<#-- TRIS -->
<#list 0..15 as i>
	<#assign InsertFlag = 0>
	<#list BitPositionA_List as bitpositiona>
		<#if bitpositiona?has_content>
			<#if bitpositiona?number == i>
				<#list TRISA_List as trisa>
					<#if bitpositiona?index == trisa?index>
            			<#assign TRISA_List_Sorted = TRISA_List_Sorted + [trisa]>
						<#assign InsertFlag = 1>
    				</#if>
				</#list>
    		</#if>
    	</#if>
	</#list>
	<#if InsertFlag == 0>
        <#assign TRISA_List_Sorted = TRISA_List_Sorted + ["IN"]>
	</#if>
</#list>
<#-- LAT -->
<#list 0..15 as i>
	<#assign InsertFlag = 0>
	<#list BitPositionA_List as bitpositiona>
		<#if bitpositiona?has_content>
			<#if bitpositiona?number == i>
				<#list LATA_List as lata>
					<#if bitpositiona?index == lata?index>
            			<#assign LATA_List_Sorted = LATA_List_Sorted + [lata]>
						<#assign InsertFlag = 1>
    				</#if>
				</#list>
    		</#if>
    	</#if>
	</#list>
	<#if InsertFlag == 0>
        <#assign LATA_List_Sorted = LATA_List_Sorted + ["LOW"]>
	</#if>
</#list>
<#-- OD -->
<#list 0..15 as i>
	<#assign InsertFlag = 0>
	<#list BitPositionA_List as bitpositiona>
		<#if bitpositiona?has_content>
			<#if bitpositiona?number == i>
				<#list ODA_List as oda>
					<#if bitpositiona?index == oda?index>
            			<#assign ODA_List_Sorted = ODA_List_Sorted + [oda]>
						<#assign InsertFlag = 1>
    				</#if>
				</#list>
    		</#if>
    	</#if>
	</#list>
	<#if InsertFlag == 0>
        <#assign ODA_List_Sorted = ODA_List_Sorted + ["FALSE"]>
	</#if>
</#list>
<#-- CNEN -->
<#list 0..15 as i>
	<#assign InsertFlag = 0>
	<#list BitPositionA_List as bitpositiona>
		<#if bitpositiona?has_content>
			<#if bitpositiona?number == i>
				<#list CNENA_List as cnena>
					<#if bitpositiona?index == cnena?index>
            			<#assign CNENA_List_Sorted = CNENA_List_Sorted + [cnena]>
						<#assign InsertFlag = 1>
    				</#if>
				</#list>
    		</#if>
    	</#if>
	</#list>
	<#if InsertFlag == 0>
        <#assign CNENA_List_Sorted = CNENA_List_Sorted + ["FALSE"]>
	</#if>
</#list>
<#-- CNPU -->
<#list 0..15 as i>
	<#assign InsertFlag = 0>
	<#list BitPositionA_List as bitpositiona>
		<#if bitpositiona?has_content>
			<#if bitpositiona?number == i>
				<#list CNPUA_List as cnpua>
					<#if bitpositiona?index == cnpua?index>
            			<#assign CNPUA_List_Sorted = CNPUA_List_Sorted + [cnpua]>
						<#assign InsertFlag = 1>
    				</#if>
				</#list>
    		</#if>
    	</#if>
	</#list>
	<#if InsertFlag == 0>
        <#assign CNPUA_List_Sorted = CNPUA_List_Sorted + ["FALSE"]>
	</#if>
</#list>
<#-- CNPD -->
<#list 0..15 as i>
	<#assign InsertFlag = 0>
	<#list BitPositionA_List as bitpositiona>
		<#if bitpositiona?has_content>
			<#if bitpositiona?number == i>
				<#list CNPDA_List as cnpda>
					<#if bitpositiona?index == cnpda?index>
            			<#assign CNPDA_List_Sorted = CNPDA_List_Sorted + [cnpda]>
						<#assign InsertFlag = 1>
    				</#if>
				</#list>
    		</#if>
    	</#if>
	</#list>
	<#if InsertFlag == 0>
        <#assign CNPDA_List_Sorted = CNPDA_List_Sorted + ["FALSE"]>
	</#if>
</#list>
<#assign MODE_REG_A_VAL =["0x"]>
<#assign TRIS_REG_A_VAL =["0x"]>
<#assign LAT_REG_A_VAL =["0x"]>
<#assign OD_REG_A_VAL =["0x"]>
<#assign CNEN_REG_A_VAL =["0x"]>
<#assign CNPU_REG_A_VAL =["0x"]>
<#assign CNPD_REG_A_VAL =["0x"]>
<#-- MODE -->
<#list 0..3 as i>
	<#assign MODE_REG_A = []>
	<#assign MODE_REG_A_HEX = []>
	<#list MODEA_List_Sorted?reverse[(i*4)..(i*4+3)] as val>
		<#if (val?upper_case == "ANALOG") || (val?upper_case == "ERROR")>
			<#assign MODE_REG_A = MODE_REG_A + [1]>
		<#else>
			<#assign MODE_REG_A = MODE_REG_A + [0]>
		</#if>
		<#assign MODE_REG_A_STR = (MODE_REG_A?join(""))>
		<@str2hex str=MODE_REG_A_STR reg="MODE"/>
	</#list>
	<#assign MODE_REG_A_VAL = MODE_REG_A_VAL + [MODE_REG_A_HEX_STR]>
	<#assign MODE_REG_A_VAL_STR = (MODE_REG_A_VAL?join(""))>
<#-- TRIS -->
	<#assign TRIS_REG_A = []>
	<#assign TRIS_REG_A_HEX = []>
	<#list TRISA_List_Sorted?reverse[(i*4)..(i*4+3)] as val>
		<#if (val?upper_case == "IN")>
			<#assign TRIS_REG_A = TRIS_REG_A + [1]>
		<#else>
			<#assign TRIS_REG_A = TRIS_REG_A + [0]>
		</#if>
		<#assign TRIS_REG_A_STR = (TRIS_REG_A?join(""))>
		<@str2hex str=TRIS_REG_A_STR reg="TRIS"/>
	</#list>
	<#assign TRIS_REG_A_VAL = TRIS_REG_A_VAL + [TRIS_REG_A_HEX_STR]>
	<#assign TRIS_REG_A_VAL_STR = (TRIS_REG_A_VAL?join(""))>
<#-- LAT -->
	<#assign LAT_REG_A = []>
	<#assign LAT_REG_A_HEX = []>
	<#list LATA_List_Sorted?reverse[(i*4)..(i*4+3)] as val>
		<#if (val?upper_case == "HIGH")>
			<#assign LAT_REG_A = LAT_REG_A + [1]>
		<#else>
			<#assign LAT_REG_A = LAT_REG_A + [0]>
		</#if>
		<#assign LAT_REG_A_STR = (LAT_REG_A?join(""))>
		<@str2hex str=LAT_REG_A_STR reg="LAT"/>
	</#list>
	<#assign LAT_REG_A_VAL = LAT_REG_A_VAL + [LAT_REG_A_HEX_STR]>
	<#assign LAT_REG_A_VAL_STR = (LAT_REG_A_VAL?join(""))>
<#-- OD -->
	<#assign OD_REG_A = []>
	<#assign OD_REG_A_HEX = []>
	<#list ODA_List_Sorted?reverse[(i*4)..(i*4+3)] as val>
		<#if (val?upper_case == "TRUE")>
			<#assign OD_REG_A = OD_REG_A + [1]>
			<#else>
			<#assign OD_REG_A = OD_REG_A + [0]>
		</#if>
		<#assign OD_REG_A_STR = (OD_REG_A?join(""))>
		<@str2hex str=OD_REG_A_STR reg="OD"/>
	</#list>
	<#assign OD_REG_A_VAL = OD_REG_A_VAL + [OD_REG_A_HEX_STR]>
	<#assign OD_REG_A_VAL_STR = (OD_REG_A_VAL?join(""))>
<#-- CNEN -->
	<#assign CNEN_REG_A = []>
	<#assign CNEN_REG_A_HEX = []>
	<#list CNENA_List_Sorted?reverse[(i*4)..(i*4+3)] as val>
		<#if (val?upper_case == "TRUE")>
			<#assign CNEN_REG_A = CNEN_REG_A + [1]>
			<#else>
			<#assign CNEN_REG_A = CNEN_REG_A + [0]>
		</#if>
		<#assign CNEN_REG_A_STR = (CNEN_REG_A?join(""))>
		<@str2hex str=CNEN_REG_A_STR reg="CNEN"/>
	</#list>
	<#assign CNEN_REG_A_VAL = CNEN_REG_A_VAL + [CNEN_REG_A_HEX_STR]>
	<#assign CNEN_REG_A_VAL_STR = (CNEN_REG_A_VAL?join(""))>
<#-- CNPU -->
	<#assign CNPU_REG_A = []>
	<#assign CNPU_REG_A_HEX = []>
	<#list CNPUA_List_Sorted?reverse[(i*4)..(i*4+3)] as val>
		<#if (val?upper_case == "TRUE")>
			<#assign CNPU_REG_A = CNPU_REG_A + [1]>
			<#else>
			<#assign CNPU_REG_A = CNPU_REG_A + [0]>
		</#if>
		<#assign CNPU_REG_A_STR = (CNPU_REG_A?join(""))>
		<@str2hex str=CNPU_REG_A_STR reg="CNPU"/>
	</#list>
	<#assign CNPU_REG_A_VAL = CNPU_REG_A_VAL + [CNPU_REG_A_HEX_STR]>
	<#assign CNPU_REG_A_VAL_STR = (CNPU_REG_A_VAL?join(""))>
<#-- CNPD -->
	<#assign CNPD_REG_A = []>
	<#assign CNPD_REG_A_HEX = []>
	<#list CNPDA_List_Sorted?reverse[(i*4)..(i*4+3)] as val>
		<#if (val?upper_case == "TRUE")>
			<#assign CNPD_REG_A = CNPD_REG_A + [1]>
			<#else>
			<#assign CNPD_REG_A = CNPD_REG_A + [0]>
		</#if>
		<#assign CNPD_REG_A_STR = (CNPD_REG_A?join(""))>
		<@str2hex str=CNPD_REG_A_STR reg="CNPD"/>
	</#list>
	<#assign CNPD_REG_A_VAL = CNPD_REG_A_VAL + [CNPD_REG_A_HEX_STR]>
	<#assign CNPD_REG_A_VAL_STR = (CNPD_REG_A_VAL?join(""))>
</#list>
<#if CONFIG_HAVE_PPS == true>
#define SYS_PORT_${port}_ANSEL        ${MODE_REG_A_VAL_STR}
</#if>
#define SYS_PORT_${port}_TRIS         ${TRIS_REG_A_VAL_STR}
#define SYS_PORT_${port}_LAT          ${LAT_REG_A_VAL_STR}
#define SYS_PORT_${port}_ODC          ${OD_REG_A_VAL_STR}
<#if CONFIG_HAVE_PPS == true>
#define SYS_PORT_${port}_CNPU         ${CNPU_REG_A_VAL_STR}
#define SYS_PORT_${port}_CNPD         ${CNPD_REG_A_VAL_STR}
#define SYS_PORT_${port}_CNEN         ${CNEN_REG_A_VAL_STR}
</#if>

</#macro>
<#--  =====================
      MACRO execution
      ===================== -->
<@mhc_process_leds/>
<@mhc_process_switches/>
<@mhc_process_vbus/>
<@mhc_process_gpio_out/>
<@mhc_process_gpio_in/>
<@mhc_process_gpio/>
