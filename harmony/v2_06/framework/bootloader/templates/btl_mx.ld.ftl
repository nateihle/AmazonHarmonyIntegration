/* Default linker script, for normal executables */
OUTPUT_FORMAT("elf32-tradlittlemips")
OUTPUT_ARCH(pic32mx)
ENTRY(_reset)
/*
 * Provide for a minimum stack and heap size
 * - _min_stack_size - represents the minimum space that must be made
 *                     available for the stack.  Can be overridden from
 *                     the command line using the linker's --defsym option.
 * - _min_heap_size  - represents the minimum space that must be made
 *                     available for the heap.  Can be overridden from
 *                     the command line using the linker's --defsym option.
 */
EXTERN (_min_stack_size _min_heap_size)

/*************************************************************************
 * Processor-specific object file.  Contains SFR definitions.
 *************************************************************************/
INPUT("processor.o")


/*************************************************************************
 * Processor-specific peripheral libraries are optional
 *************************************************************************/
OPTIONAL("libmchp_peripheral.a")

/*************************************************************************
 * For interrupt vector handling
 *************************************************************************/
<#if CONFIG_BOOT_ADDR_SIZE == "0xbf0">
    <#if CONFIG_BOOTLOADER_TYPE == "USB_HOST">
        <#assign bootloader_length = "0x13C00">
        <#assign bootloader_elength = "0x0000">
    <#elseif CONFIG_BOOTLOADER_TYPE == "USB_DEVICE">
        <#assign bootloader_length = "0x7400">
        <#assign bootloader_elength = "0x0000">
    <#elseif CONFIG_BOOTLOADER_TYPE == "USART" || CONFIG_BOOTLOADER_TYPE == "I2C">
        <#assign bootloader_length = "0x2000">
        <#assign bootloader_elength = "0x0000">
    <#else>
        <#assign bootloader_length = "0xDC00">
        <#assign bootloader_elength = "0x0000">
    </#if>
<#else>
    <#if CONFIG_BOOTLOADER_TYPE == "USB_HOST">
        <#assign bootloader_length = "0x14000">
        <#assign bootloader_elength = "0x0000">
    <#elseif CONFIG_BOOTLOADER_TYPE == "USB_DEVICE">
        <#assign bootloader_length = "0x7000">
        <#assign bootloader_elength = "0x0000">
    <#elseif CONFIG_BOOTLOADER_TYPE == "ETHERNET_UDP_PULL">
        <#assign bootloader_length = "0x13000">
        <#assign bootloader_elength = "0x0000">
    <#elseif CONFIG_BOOTLOADER_TYPE == "USART" || CONFIG_BOOTLOADER_TYPE == "I2C">
        <#assign bootloader_length = "0x1900">
        <#assign bootloader_elength = "0x0000">
    <#else>
        <#assign bootloader_length = "0x3000">
        <#assign bootloader_elength = "0x1000">
    </#if>
</#if>
PROVIDE(_vector_spacing = 0x00000001);
<#if CONFIG_BOOT_ADDR_SIZE == "0xbf0">
    <#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == false>
        <#lt>_ebase_address = 0x9D000000;
        <#assign eaddress = "0x9D000000">
        <#assign elength = "0x0000">
    <#elseif CONFIG_BOOTLOADER_TYPE == "USB_HOST">
        <#lt>_ebase_address = 0x9D014000;
        <#assign eaddress = "0x9D014000">
        <#assign elength = "0x0000">
    <#elseif CONFIG_BOOTLOADER_TYPE == "USB_DEVICE">
        <#lt>_ebase_address = 0x9D008000;
        <#assign eaddress = "0x9D008000">
        <#assign elength = "0x0000">
    <#elseif CONFIG_BOOTLOADER_TYPE != "USART" && CONFIG_BOOTLOADER_TYPE != "I2C">
        <#lt>_ebase_address = 0x9D00E000;
        <#assign eaddress = "0x9D00E000">
        <#assign elength = "0x0000">
    <#else>
        <#lt>_ebase_address = 0x9D003000;
        <#assign eaddress = "0x9D003000">
        <#assign elength = "0x0000">
    </#if>
<#else>
    <#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == false>
        <#if CONFIG_BOOTLOADER_TYPE == "USART" || CONFIG_BOOTLOADER_TYPE == "I2C">
            <#lt>_ebase_address =  0x9FC00300;
            <#assign eaddress = "0x9FC00300">
            <#assign elength = "0x0200">
        <#else>
            <#lt>_ebase_address =  0x9FC01000;
            <#assign eaddress = "0x9FC01000">
            <#assign elength = "0x1000">
        </#if>
    <#elseif CONFIG_BOOTLOADER_TYPE == "USB_HOST">
        <#lt>_ebase_address = 0x9D015000;
        <#assign eaddress = "0x9D015000">
        <#assign elength = "0x0000">
    <#elseif CONFIG_BOOTLOADER_TYPE == "USB_DEVICE">
        <#lt>_ebase_address = 0x9D008000;
        <#assign eaddress = "0x9D008000">
        <#assign elength = "0x0000">
    <#elseif CONFIG_BOOTLOADER_TYPE == "ETHERNET_UDP_PULL">
        <#lt>_ebase_address = 0x9D014000;
        <#assign eaddress = "0x9D014000">
        <#assign elength = "0x0000">
    <#elseif CONFIG_BOOTLOADER_TYPE != "USART" && CONFIG_BOOTLOADER_TYPE != "I2C">
        <#lt>_ebase_address =  0x9D00E000;
        <#assign eaddress = "0x9D00E000">
        <#assign elength = "0x1000">
    <#else>
        <#lt>_ebase_address =  0x9D001000;
        <#assign eaddress = "0x9D000000">
        <#assign elength = "0x0000">
    </#if>
</#if>

/*************************************************************************
 * Memory Address Equates
 * _RESET_ADDR      -- Reset Vector
 * _BEV_EXCPT_ADDR  -- Boot exception Vector
 * _DBG_EXCPT_ADDR  -- In-circuit Debugging Exception Vector
 * _DBG_CODE_ADDR   -- In-circuit Debug Executive address
 * _DBG_CODE_SIZE   -- In-circuit Debug Executive size
 * _GEN_EXCPT_ADDR  -- General Exception Vector
 *************************************************************************/
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == false>
    <#assign resetaddress = "0xBFC00000">
<#else>
    <#if CONFIG_BOOT_ADDR_SIZE == "0xbf0">
        <#if CONFIG_BOOTLOADER_TYPE == "USB_HOST">
            <#assign resetaddress = "(0x9D013C00)">
        <#elseif CONFIG_BOOTLOADER_TYPE == "USB_DEVICE">
            <#assign resetaddress = "(0x9D007400)">
        <#else>
            <#assign resetaddress = "(0x9D002000)">
        </#if>
    <#elseif CONFIG_BOOTLOADER_TYPE == "USB_HOST">
        <#assign resetaddress = "(0x9D014000)">
    <#elseif CONFIG_BOOTLOADER_TYPE == "ETHERNET_UDP_PULL">
        <#assign resetaddress = "(0x9D013000)">
    <#elseif CONFIG_BOOTLOADER_TYPE == "USB_DEVICE">
        <#assign resetaddress = "(0x9D007000)">
    <#else>
        <#assign resetaddress = "(0x9D000000)">
    </#if>
</#if>
_RESET_ADDR              = ${resetaddress};
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == false>
    <#lt>_BEV_EXCPT_ADDR          = (${resetaddress} + 0x380);
    <#lt>_DBG_EXCPT_ADDR          = (${resetaddress} + 0x480);
    <#if CONFIG_BOOT_ADDR_SIZE != "0xbf0">
        <#lt>_DBG_CODE_ADDR          = 0xBFC02000;
        <#assign debugexecaddr = "0xBFC02000">
        <#lt>_DBG_CODE_SIZE           = 0xFF0;
    <#else>
        <#lt>_DBG_CODE_ADDR          = 0x9FC00490;
        <#assign debugexecaddr = "0x9FC00490">
        <#lt>_DBG_CODE_SIZE           = 0x760;
    </#if>
</#if>
_GEN_EXCPT_ADDR          = _ebase_address + 0x180;

/*************************************************************************
 * Memory Regions
 *
 * Memory regions without attributes cannot be used for orphaned sections.
 * Only sections specifically assigned to these regions can be allocated
 * into these regions.
 *************************************************************************/
MEMORY
{
<#if CONFIG_BOOT_ADDR_SIZE == "0xbf0">
    <#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true>
        <#lt>  kseg0_program_mem    (rx)  : ORIGIN = 0x9D000000 + ${bootloader_length}, LENGTH = ${CONFIG_PFM_ADDR_SIZE} - ${bootloader_length}   /* All C Files will be located here */ 
    <#else>
        <#lt>  kseg0_program_mem    (rx)  : ORIGIN = 0x9D000000, LENGTH = ${bootloader_length}   /* All C Files will be located here */ 
    </#if>
    <#lt>  kseg0_boot_mem             : ORIGIN = 0x9FC00000, LENGTH = 0x0                                    /* This memory region is dummy */
<#elseif CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true>
    <#if CONFIG_BOOTLOADER_TYPE != "USART">
        <#lt>  kseg0_program_mem    (rx)  : ORIGIN = (0x9D000000 + ${bootloader_length}), LENGTH = (${CONFIG_PFM_ADDR_SIZE} - ${bootloader_length}) /* All C Files will be located here */
        <#lt>  kseg0_boot_mem             : ORIGIN = 0x9D000000 + ${bootloader_length}, LENGTH = 0x0 /* This memory region is dummy */
    <#else>
        <#lt>  kseg0_program_mem    (rx)  : ORIGIN = (0x9D000000), LENGTH = ${CONFIG_PFM_ADDR_SIZE} /* All C Files will be located here */
        <#lt>  kseg0_boot_mem             : ORIGIN = 0x9D000000, LENGTH = 0x0 /* This memory region is dummy */
    </#if>
<#else>
    <#if CONFIG_BOOTLOADER_TYPE != "USART">
        <#lt>  kseg0_program_mem    (rx)  : ORIGIN = 0x9D000000, LENGTH = ${bootloader_length} /* All C Files will be located here */ 
        <#lt>  kseg0_boot_mem             : ORIGIN = 0x9FC00490, LENGTH = 0x0 /* This memory region is dummy */ 
    <#else>
        <#lt>  kseg0_program_mem    (rx)  : ORIGIN = 0x9FC00500, LENGTH = 0x2AF0 /* All C Files will be located here */ 
        <#lt>  kseg0_boot_mem             : ORIGIN = 0x9FC00020, LENGTH = 0x0 /* This memory region is dummy */ 
    </#if>
</#if>
<#if (CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == false) && (CONFIG_BOOT_ADDR_SIZE != "0xbf0")>
  exception_mem              : ORIGIN = 0x9FC01000, LENGTH = 0x1000  /* Interrupt vector table */
</#if>
<#if CONFIG_BOOT_ADDR_SIZE != "0xbf0">
    <#lt>  config3                    : ORIGIN = 0xBFC02FF0, LENGTH = 0x4
    <#lt>  config2                    : ORIGIN = 0xBFC02FF4, LENGTH = 0x4
    <#lt>  config1                    : ORIGIN = 0xBFC02FF8, LENGTH = 0x4
    <#lt>  config0                    : ORIGIN = 0xBFC02FFC, LENGTH = 0x4
<#else>
    <#lt>  config3                    : ORIGIN = 0xBFC00BF0, LENGTH = 0x4
    <#lt>  config2                    : ORIGIN = 0xBFC00BF4, LENGTH = 0x4
    <#lt>  config1                    : ORIGIN = 0xBFC00BF8, LENGTH = 0x4
    <#lt>  config0                    : ORIGIN = 0xBFC00BFC, LENGTH = 0x4
</#if>
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == false>
  kseg1_boot_mem             : ORIGIN = ${resetaddress}, LENGTH = 0x300 /* C Startup code */
</#if>
  kseg1_data_mem       (w!x) : ORIGIN = 0xA0000000, LENGTH = ${CONFIG_SRAM_ADDR_SIZE}
  sfrs                       : ORIGIN = 0xBF800000, LENGTH = 0x100000
<#if CONFIG_BOOT_ADDR_SIZE != "0xbf0">
  debug_exec_mem             : ORIGIN = 0xBFC02000, LENGTH = 0xFF0
  configsfrs                 : ORIGIN = 0xBFC02FF0, LENGTH = 0x10
<#else>
  debug_exec_mem             : ORIGIN = 0x9FC00490, LENGTH = 0x760
  configsfrs                 : ORIGIN = 0xBFC00BF0, LENGTH = 0x10
</#if>
}

/*************************************************************************
 * Configuration-word sections
 *************************************************************************/
<#if CONFIG_BOOT_ADDR_SIZE != "0xbf0">
<#assign hex = "2F">
<#else>
<#assign hex = "0B">
</#if>
SECTIONS
{ 
  .config_BFC02FF0 : {
    KEEP(*(.config_BFC0${hex}F0))
  } > config3
  .config_BFC02FF4 : {
    KEEP(*(.config_BFC0${hex}F4))
  } > config2
  .config_BFC02FF8 : {
    KEEP(*(.config_BFC0${hex}F8))
  } > config1
  .config_BFC02FFC : {
    KEEP(*(.config_BFC0${hex}FC))
  } > config0
}
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == false>
    <#lt>PROVIDE(_DBG_CODE_ADDR = 0xBFC02000) ;
    <#lt>PROVIDE(_DBG_CODE_SIZE = 0xFF0) ;
</#if>
SECTIONS
{
  /* Boot Sections */
  .reset _RESET_ADDR :
  {
    KEEP(*(.reset))
    KEEP(*(.reset.startup))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == false>
  } > kseg1_boot_mem
<#else>
  } > kseg0_program_mem
</#if>
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == false>
  .bev_excpt _BEV_EXCPT_ADDR :
  {
    KEEP(*(.bev_handler))
  } > kseg1_boot_mem
  .dbg_excpt _DBG_EXCPT_ADDR (NOLOAD) :
  {
    . += (DEFINED (_DEBUGGER) ? 0x8 : 0x0);
  } > kseg1_boot_mem
  .dbg_code _DBG_CODE_ADDR (NOLOAD) :
  {
    . += (DEFINED (_DEBUGGER) ? _DBG_CODE_SIZE : 0x0);
  } > debug_exec_mem
</#if>
  .app_excpt _GEN_EXCPT_ADDR :
  {
    KEEP(*(.gen_handler))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>

<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || 
    (CONFIG_BOOTLOADER_TYPE != "USART")>
   .vector_0 _ebase_address + 0x200 :
  {
    KEEP(*(.vector_0))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_0) <= (_vector_spacing << 5), "function at exception vector 0 too large")
  .vector_1 _ebase_address + 0x200 + (_vector_spacing << 5) * 1 :
  {
    KEEP(*(.vector_1))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_1) <= (_vector_spacing << 5), "function at exception vector 1 too large")
  .vector_2 _ebase_address + 0x200 + (_vector_spacing << 5) * 2 :
  {
    KEEP(*(.vector_2))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_2) <= (_vector_spacing << 5), "function at exception vector 2 too large")
  .vector_3 _ebase_address + 0x200 + (_vector_spacing << 5) * 3 :
  {
    KEEP(*(.vector_3))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_3) <= (_vector_spacing << 5), "function at exception vector 3 too large")
  .vector_4 _ebase_address + 0x200 + (_vector_spacing << 5) * 4 :
  {
    KEEP(*(.vector_4))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_4) <= (_vector_spacing << 5), "function at exception vector 4 too large")
  .vector_5 _ebase_address + 0x200 + (_vector_spacing << 5) * 5 :
  {
    KEEP(*(.vector_5))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_5) <= (_vector_spacing << 5), "function at exception vector 5 too large")
  .vector_6 _ebase_address + 0x200 + (_vector_spacing << 5) * 6 :
  {
    KEEP(*(.vector_6))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_6) <= (_vector_spacing << 5), "function at exception vector 6 too large")
  .vector_7 _ebase_address + 0x200 + (_vector_spacing << 5) * 7 :
  {
    KEEP(*(.vector_7))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_7) <= (_vector_spacing << 5), "function at exception vector 7 too large")
  .vector_8 _ebase_address + 0x200 + (_vector_spacing << 5) * 8 :
  {
    KEEP(*(.vector_8))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_8) <= (_vector_spacing << 5), "function at exception vector 8 too large")
  .vector_9 _ebase_address + 0x200 + (_vector_spacing << 5) * 9 :
  {
    KEEP(*(.vector_9))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_9) <= (_vector_spacing << 5), "function at exception vector 9 too large")
  .vector_10 _ebase_address + 0x200 + (_vector_spacing << 5) * 10 :
  {
    KEEP(*(.vector_10))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_10) <= (_vector_spacing << 5), "function at exception vector 10 too large")
  .vector_11 _ebase_address + 0x200 + (_vector_spacing << 5) * 11 :
  {
    KEEP(*(.vector_11))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_11) <= (_vector_spacing << 5), "function at exception vector 11 too large")
  .vector_12 _ebase_address + 0x200 + (_vector_spacing << 5) * 12 :
  {
    KEEP(*(.vector_12))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_12) <= (_vector_spacing << 5), "function at exception vector 12 too large")
  .vector_13 _ebase_address + 0x200 + (_vector_spacing << 5) * 13 :
  {
    KEEP(*(.vector_13))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_13) <= (_vector_spacing << 5), "function at exception vector 13 too large")
  .vector_14 _ebase_address + 0x200 + (_vector_spacing << 5) * 14 :
  {
    KEEP(*(.vector_14))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_14) <= (_vector_spacing << 5), "function at exception vector 14 too large")
  .vector_15 _ebase_address + 0x200 + (_vector_spacing << 5) * 15 :
  {
    KEEP(*(.vector_15))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_15) <= (_vector_spacing << 5), "function at exception vector 15 too large")
  .vector_16 _ebase_address + 0x200 + (_vector_spacing << 5) * 16 :
  {
    KEEP(*(.vector_16))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_16) <= (_vector_spacing << 5), "function at exception vector 16 too large")
  .vector_17 _ebase_address + 0x200 + (_vector_spacing << 5) * 17 :
  {
    KEEP(*(.vector_17))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_17) <= (_vector_spacing << 5), "function at exception vector 17 too large")
  .vector_18 _ebase_address + 0x200 + (_vector_spacing << 5) * 18 :
  {
    KEEP(*(.vector_18))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_18) <= (_vector_spacing << 5), "function at exception vector 18 too large")
  .vector_19 _ebase_address + 0x200 + (_vector_spacing << 5) * 19 :
  {
    KEEP(*(.vector_19))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_19) <= (_vector_spacing << 5), "function at exception vector 19 too large")
  .vector_20 _ebase_address + 0x200 + (_vector_spacing << 5) * 20 :
  {
    KEEP(*(.vector_20))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_20) <= (_vector_spacing << 5), "function at exception vector 20 too large")
  .vector_21 _ebase_address + 0x200 + (_vector_spacing << 5) * 21 :
  {
    KEEP(*(.vector_21))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_21) <= (_vector_spacing << 5), "function at exception vector 21 too large")
  .vector_22 _ebase_address + 0x200 + (_vector_spacing << 5) * 22 :
  {
    KEEP(*(.vector_22))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_22) <= (_vector_spacing << 5), "function at exception vector 22 too large")
  .vector_23 _ebase_address + 0x200 + (_vector_spacing << 5) * 23 :
  {
    KEEP(*(.vector_23))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_23) <= (_vector_spacing << 5), "function at exception vector 23 too large")
  .vector_24 _ebase_address + 0x200 + (_vector_spacing << 5) * 24 :
  {
    KEEP(*(.vector_24))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_24) <= (_vector_spacing << 5), "function at exception vector 24 too large")
  .vector_25 _ebase_address + 0x200 + (_vector_spacing << 5) * 25 :
  {
    KEEP(*(.vector_25))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_25) <= (_vector_spacing << 5), "function at exception vector 25 too large")
  .vector_26 _ebase_address + 0x200 + (_vector_spacing << 5) * 26 :
  {
    KEEP(*(.vector_26))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_26) <= (_vector_spacing << 5), "function at exception vector 26 too large")
  .vector_27 _ebase_address + 0x200 + (_vector_spacing << 5) * 27 :
  {
    KEEP(*(.vector_27))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_27) <= (_vector_spacing << 5), "function at exception vector 27 too large")
  .vector_28 _ebase_address + 0x200 + (_vector_spacing << 5) * 28 :
  {
    KEEP(*(.vector_28))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_28) <= (_vector_spacing << 5), "function at exception vector 28 too large")
  .vector_29 _ebase_address + 0x200 + (_vector_spacing << 5) * 29 :
  {
    KEEP(*(.vector_29))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_29) <= (_vector_spacing << 5), "function at exception vector 29 too large")
  .vector_30 _ebase_address + 0x200 + (_vector_spacing << 5) * 30 :
  {
    KEEP(*(.vector_30))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_30) <= (_vector_spacing << 5), "function at exception vector 30 too large")
  .vector_31 _ebase_address + 0x200 + (_vector_spacing << 5) * 31 :
  {
    KEEP(*(.vector_31))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_31) <= (_vector_spacing << 5), "function at exception vector 31 too large")
  .vector_32 _ebase_address + 0x200 + (_vector_spacing << 5) * 32 :
  {
    KEEP(*(.vector_32))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_32) <= (_vector_spacing << 5), "function at exception vector 32 too large")
  .vector_33 _ebase_address + 0x200 + (_vector_spacing << 5) * 33 :
  {
    KEEP(*(.vector_33))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_33) <= (_vector_spacing << 5), "function at exception vector 33 too large")
  .vector_34 _ebase_address + 0x200 + (_vector_spacing << 5) * 34 :
  {
    KEEP(*(.vector_34))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_34) <= (_vector_spacing << 5), "function at exception vector 34 too large")
  .vector_35 _ebase_address + 0x200 + (_vector_spacing << 5) * 35 :
  {
    KEEP(*(.vector_35))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_35) <= (_vector_spacing << 5), "function at exception vector 35 too large")
  .vector_36 _ebase_address + 0x200 + (_vector_spacing << 5) * 36 :
  {
    KEEP(*(.vector_36))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_36) <= (_vector_spacing << 5), "function at exception vector 36 too large")
  .vector_37 _ebase_address + 0x200 + (_vector_spacing << 5) * 37 :
  {
    KEEP(*(.vector_37))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_37) <= (_vector_spacing << 5), "function at exception vector 37 too large")
  .vector_38 _ebase_address + 0x200 + (_vector_spacing << 5) * 38 :
  {
    KEEP(*(.vector_38))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_38) <= (_vector_spacing << 5), "function at exception vector 38 too large")
  .vector_39 _ebase_address + 0x200 + (_vector_spacing << 5) * 39 :
  {
    KEEP(*(.vector_39))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_39) <= (_vector_spacing << 5), "function at exception vector 39 too large")
  .vector_40 _ebase_address + 0x200 + (_vector_spacing << 5) * 40 :
  {
    KEEP(*(.vector_40))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_40) <= (_vector_spacing << 5), "function at exception vector 40 too large")
  .vector_41 _ebase_address + 0x200 + (_vector_spacing << 5) * 41 :
  {
    KEEP(*(.vector_41))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_41) <= (_vector_spacing << 5), "function at exception vector 41 too large")
  .vector_42 _ebase_address + 0x200 + (_vector_spacing << 5) * 42 :
  {
    KEEP(*(.vector_42))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_42) <= (_vector_spacing << 5), "function at exception vector 42 too large")
  .vector_43 _ebase_address + 0x200 + (_vector_spacing << 5) * 43 :
  {
    KEEP(*(.vector_43))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_43) <= (_vector_spacing << 5), "function at exception vector 43 too large")
  .vector_44 _ebase_address + 0x200 + (_vector_spacing << 5) * 44 :
  {
    KEEP(*(.vector_44))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_44) <= (_vector_spacing << 5), "function at exception vector 44 too large")
  .vector_45 _ebase_address + 0x200 + (_vector_spacing << 5) * 45 :
  {
    KEEP(*(.vector_45))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_45) <= (_vector_spacing << 5), "function at exception vector 45 too large")
  .vector_46 _ebase_address + 0x200 + (_vector_spacing << 5) * 46 :
  {
    KEEP(*(.vector_46))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_46) <= (_vector_spacing << 5), "function at exception vector 46 too large")
  .vector_47 _ebase_address + 0x200 + (_vector_spacing << 5) * 47 :
  {
    KEEP(*(.vector_47))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_47) <= (_vector_spacing << 5), "function at exception vector 47 too large")
  .vector_48 _ebase_address + 0x200 + (_vector_spacing << 5) * 48 :
  {
    KEEP(*(.vector_48))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_48) <= (_vector_spacing << 5), "function at exception vector 48 too large")
  .vector_49 _ebase_address + 0x200 + (_vector_spacing << 5) * 49 :
  {
    KEEP(*(.vector_49))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_49) <= (_vector_spacing << 5), "function at exception vector 49 too large")
  .vector_50 _ebase_address + 0x200 + (_vector_spacing << 5) * 50 :
  {
    KEEP(*(.vector_50))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_50) <= (_vector_spacing << 5), "function at exception vector 50 too large")
  .vector_51 _ebase_address + 0x200 + (_vector_spacing << 5) * 51 :
  {
    KEEP(*(.vector_51))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_51) <= (_vector_spacing << 5), "function at exception vector 51 too large")
  .vector_52 _ebase_address + 0x200 + (_vector_spacing << 5) * 52 :
  {
    KEEP(*(.vector_52))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_52) <= (_vector_spacing << 5), "function at exception vector 52 too large")
  .vector_53 _ebase_address + 0x200 + (_vector_spacing << 5) * 53 :
  {
    KEEP(*(.vector_53))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_53) <= (_vector_spacing << 5), "function at exception vector 53 too large")
  .vector_54 _ebase_address + 0x200 + (_vector_spacing << 5) * 54 :
  {
    KEEP(*(.vector_54))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_54) <= (_vector_spacing << 5), "function at exception vector 54 too large")
  .vector_55 _ebase_address + 0x200 + (_vector_spacing << 5) * 55 :
  {
    KEEP(*(.vector_55))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_55) <= (_vector_spacing << 5), "function at exception vector 55 too large")
  .vector_56 _ebase_address + 0x200 + (_vector_spacing << 5) * 56 :
  {
    KEEP(*(.vector_56))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_56) <= (_vector_spacing << 5), "function at exception vector 56 too large")
  .vector_57 _ebase_address + 0x200 + (_vector_spacing << 5) * 57 :
  {
    KEEP(*(.vector_57))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_57) <= (_vector_spacing << 5), "function at exception vector 57 too large")
  .vector_58 _ebase_address + 0x200 + (_vector_spacing << 5) * 58 :
  {
    KEEP(*(.vector_58))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_58) <= (_vector_spacing << 5), "function at exception vector 58 too large")
  .vector_59 _ebase_address + 0x200 + (_vector_spacing << 5) * 59 :
  {
    KEEP(*(.vector_59))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_59) <= (_vector_spacing << 5), "function at exception vector 59 too large")
  .vector_60 _ebase_address + 0x200 + (_vector_spacing << 5) * 60 :
  {
    KEEP(*(.vector_60))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_60) <= (_vector_spacing << 5), "function at exception vector 60 too large")
  .vector_61 _ebase_address + 0x200 + (_vector_spacing << 5) * 61 :
  {
    KEEP(*(.vector_61))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_61) <= (_vector_spacing << 5), "function at exception vector 61 too large")
  .vector_62 _ebase_address + 0x200 + (_vector_spacing << 5) * 62 :
  {
    KEEP(*(.vector_62))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_62) <= (_vector_spacing << 5), "function at exception vector 62 too large")
  .vector_63 _ebase_address + 0x200 + (_vector_spacing << 5) * 63 :
  {
    KEEP(*(.vector_63))
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true || CONFIG_BOOT_ADDR_SIZE == "0xbf0">
  } > kseg0_program_mem
<#else>
  } > exception_mem
</#if>
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_63) <= (_vector_spacing << 5), "function at exception vector 63 too large")
  </#if>
  /*  Starting with C32 v2.00, the startup code is in the .reset.startup section.
   *  Keep this here for backwards compatibility.
   */

  .startup ORIGIN(kseg0_boot_mem) :
  {
    KEEP(*(.startup))
  } > kseg0_boot_mem
  /* Code Sections - Note that input sections *(.text) and *(.text.*)
  ** are not mapped here. Starting in C32 v2.00, the best-fit allocator
  ** locates them, so that .text may flow around absolute sections
  ** as needed.
  */
  .text :
  {
    *(.stub .gnu.linkonce.t.*)
    KEEP (*(.text.*personality*))
    *(.mips16.fn.*)
    *(.mips16.call.*)
    *(.gnu.warning)
    . = ALIGN(4) ;
  } >kseg0_program_mem
  /* Global-namespace object initialization */
  .init   :
  {
    KEEP (*crti.o(.init))
    KEEP (*crtbegin.o(.init))
    KEEP (*(EXCLUDE_FILE (*crtend.o *crtend?.o *crtn.o ).init))
    KEEP (*crtend.o(.init))
    KEEP (*crtn.o(.init))
    . = ALIGN(4) ;
  } >kseg0_program_mem
  .fini   :
  {
    KEEP (*(.fini))
    . = ALIGN(4) ;
  } >kseg0_program_mem
  .preinit_array   :
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array))
    PROVIDE_HIDDEN (__preinit_array_end = .);
    . = ALIGN(4) ;
  } >kseg0_program_mem
  .init_array   :
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array))
    PROVIDE_HIDDEN (__init_array_end = .);
    . = ALIGN(4) ;
  } >kseg0_program_mem
  .fini_array   :
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array))
    PROVIDE_HIDDEN (__fini_array_end = .);
    . = ALIGN(4) ;
  } >kseg0_program_mem
  .ctors   :
  {
    /* XC32 uses crtbegin.o to find the start of
       the constructors, so we make sure it is
       first.  Because this is a wildcard, it
       doesn't matter if the user does not
       actually link against crtbegin.o; the
       linker won't look for a file to match a
       wildcard.  The wildcard also means that it
       doesn't matter which directory crtbegin.o
       is in.  */
    KEEP (*crtbegin.o(.ctors))
    KEEP (*crtbegin?.o(.ctors))
    /* We don't want to include the .ctor section from
       the crtend.o file until after the sorted ctors.
       The .ctor section from the crtend file contains the
       end of ctors marker and it must be last */
    KEEP (*(EXCLUDE_FILE (*crtend.o *crtend?.o ) .ctors))
    KEEP (*(SORT(.ctors.*)))
    KEEP (*(.ctors))
    . = ALIGN(4) ;
  } >kseg0_program_mem
  .dtors   :
  {
    KEEP (*crtbegin.o(.dtors))
    KEEP (*crtbegin?.o(.dtors))
    KEEP (*(EXCLUDE_FILE (*crtend.o *crtend?.o ) .dtors))
    KEEP (*(SORT(.dtors.*)))
    KEEP (*(.dtors))
    . = ALIGN(4) ;
  } >kseg0_program_mem
  /* Read-only sections */
  .rodata   :
  {
    *( .gnu.linkonce.r.*)
    *(.rodata1)
    . = ALIGN(4) ;
  } >kseg0_program_mem
  /*
   * Small initialized constant global and static data can be placed in the
   * .sdata2 section.  This is different from .sdata, which contains small
   * initialized non-constant global and static data.
   */
  .sdata2 ALIGN(4) :
  {
    *(.sdata2 .sdata2.* .gnu.linkonce.s2.*)
    . = ALIGN(4) ;
  } >kseg0_program_mem
  /*
   * Uninitialized constant global and static data (i.e., variables which will
   * always be zero).  Again, this is different from .sbss, which contains
   * small non-initialized, non-constant global and static data.
   */
  .sbss2 ALIGN(4) :
  {
    *(.sbss2 .sbss2.* .gnu.linkonce.sb2.*)
    . = ALIGN(4) ;
  } >kseg0_program_mem
  .eh_frame_hdr   :
  {
    *(.eh_frame_hdr)
  } >kseg0_program_mem
    . = ALIGN(4) ;
  .eh_frame   : ONLY_IF_RO
  {
    KEEP (*(.eh_frame))
  } >kseg0_program_mem
    . = ALIGN(4) ;
  .gcc_except_table   : ONLY_IF_RO
  {
    *(.gcc_except_table .gcc_except_table.*)
  } >kseg0_program_mem
    . = ALIGN(4) ;
  .dbg_data (NOLOAD) :
  {
    . += (DEFINED (_DEBUGGER) ? 0x200 : 0x0);
  } >kseg1_data_mem
  .jcr   :
  {
    KEEP (*(.jcr))
    . = ALIGN(4) ;
  } >kseg1_data_mem
  .eh_frame    : ONLY_IF_RW
  {
    KEEP (*(.eh_frame))
  } >kseg1_data_mem
    . = ALIGN(4) ;
  .gcc_except_table    : ONLY_IF_RW
  {
    *(.gcc_except_table .gcc_except_table.*)
  } >kseg1_data_mem
    . = ALIGN(4) ;
  /* Persistent data - Use the new C 'persistent' attribute instead. */
  .persist   :
  {
    _persist_begin = .;
    *(.persist .persist.*)
    *(.pbss .pbss.*)
    . = ALIGN(4) ;
    _persist_end = .;
  } >kseg1_data_mem
  /*
   * Note that input sections named .data* are no longer mapped here.
   * Starting in C32 v2.00, the best-fit allocator locates them, so
   * that they may flow around absolute sections as needed.
   */
  .data   :
  {
    *( .gnu.linkonce.d.*)
    SORT(CONSTRUCTORS)
    *(.data1)
    . = ALIGN(4) ;
  } >kseg1_data_mem
  . = .;
  _gp = ALIGN(16) + 0x7ff0;
  .got ALIGN(4) :
  {
    *(.got.plt) *(.got)
    . = ALIGN(4) ;
  } >kseg1_data_mem /* AT>kseg0_program_mem */
  /*
   * Note that "small" data sections are still mapped in the linker
   * script. This ensures that they are grouped together for
   * gp-relative addressing. Absolute sections are allocated after
   * the "small" data sections so small data cannot flow around them.
   */
  /*
   * We want the small data sections together, so single-instruction offsets
   * can access them all, and initialized data all before uninitialized, so
   * we can shorten the on-disk segment size.
   */
  .sdata ALIGN(4) :
  {
    _sdata_begin = . ;
    *(.sdata .sdata.* .gnu.linkonce.s.*)
    . = ALIGN(4) ;
    _sdata_end = . ;
  } >kseg1_data_mem
  .lit8           :
  {
    *(.lit8)
  } >kseg1_data_mem
  .lit4           :
  {
    *(.lit4)
  } >kseg1_data_mem
  . = ALIGN (4) ;
  _data_end = . ;
  _bss_begin = . ;
  .sbss ALIGN(4) :
  {
    _sbss_begin = . ;
    *(.dynsbss)
    *(.sbss .sbss.* .gnu.linkonce.sb.*)
    *(.scommon)
    _sbss_end = . ;
    . = ALIGN(4) ;
  } >kseg1_data_mem
  /*
   * Align here to ensure that the .bss section occupies space up to
   * _end.  Align after .bss to ensure correct alignment even if the
   * .bss section disappears because there are no input sections.
   *
   * Note that input sections named .bss* are no longer mapped here.
   * Starting in C32 v2.00, the best-fit allocator locates them, so
   * that they may flow around absolute sections as needed.
   *
   */
  .bss     :
  {
    *(.dynbss)
    *(COMMON)
   /* Align here to ensure that the .bss section occupies space up to
      _end.  Align after .bss to ensure correct alignment even if the
      .bss section disappears because there are no input sections. */
   . = ALIGN(. != 0 ? 4 : 1);
  } >kseg1_data_mem
  . = ALIGN(4) ;
  _end = . ;
  _bss_end = . ;
  /* Starting with C32 v2.00, the heap and stack are dynamically
   * allocated by the linker.
   */
  /*
   * RAM functions go at the end of our stack and heap allocation.
   * Alignment of 2K required by the boundary register (BMXDKPBA).
   *
   * RAM functions are now allocated by the linker. The linker generates
   * _ramfunc_begin and _bmxdkpba_address symbols depending on the
   * location of RAM functions.
   */
  _bmxdudba_address = LENGTH(kseg1_data_mem) ;
  _bmxdupba_address = LENGTH(kseg1_data_mem) ;
    /* The .pdr section belongs in the absolute section */
    /DISCARD/ : { *(.pdr) }
  .gptab.sdata : { *(.gptab.data) *(.gptab.sdata) }
  .gptab.sbss : { *(.gptab.bss) *(.gptab.sbss) }
  .mdebug.abi32 : { KEEP(*(.mdebug.abi32)) }
  .mdebug.abiN32 : { KEEP(*(.mdebug.abiN32)) }
  .mdebug.abi64 : { KEEP(*(.mdebug.abi64)) }
  .mdebug.abiO64 : { KEEP(*(.mdebug.abiO64)) }
  .mdebug.eabi32 : { KEEP(*(.mdebug.eabi32)) }
  .mdebug.eabi64 : { KEEP(*(.mdebug.eabi64)) }
  .gcc_compiled_long32 : { KEEP(*(.gcc_compiled_long32)) }
  .gcc_compiled_long64 : { KEEP(*(.gcc_compiled_long64)) }
  /* Stabs debugging sections.  */
  .stab          0 : { *(.stab) }
  .stabstr       0 : { *(.stabstr) }
  .stab.excl     0 : { *(.stab.excl) }
  .stab.exclstr  0 : { *(.stab.exclstr) }
  .stab.index    0 : { *(.stab.index) }
  .stab.indexstr 0 : { *(.stab.indexstr) }
  .comment       0 : { *(.comment) }
  /* DWARF debug sections.
     Symbols in the DWARF debugging sections are relative to the beginning
     of the section so we begin them at 0.  */
  /* DWARF 1 */
  .debug          0 : { *(.debug) }
  .line           0 : { *(.line) }
  /* GNU DWARF 1 extensions */
  .debug_srcinfo  0 : { *(.debug_srcinfo) }
  .debug_sfnames  0 : { *(.debug_sfnames) }
  /* DWARF 1.1 and DWARF 2 */
  .debug_aranges  0 : { *(.debug_aranges) }
  .debug_pubnames 0 : { *(.debug_pubnames) }
  /* DWARF 2 */
  .debug_info     0 : { *(.debug_info .gnu.linkonce.wi.*) }
  .debug_abbrev   0 : { *(.debug_abbrev) }
  .debug_line     0 : { *(.debug_line) }
  .debug_frame    0 : { *(.debug_frame) }
  .debug_str      0 : { *(.debug_str) }
  .debug_loc      0 : { *(.debug_loc) }
  .debug_macinfo  0 : { *(.debug_macinfo) }
  /* SGI/MIPS DWARF 2 extensions */
  .debug_weaknames 0 : { *(.debug_weaknames) }
  .debug_funcnames 0 : { *(.debug_funcnames) }
  .debug_typenames 0 : { *(.debug_typenames) }
  .debug_varnames  0 : { *(.debug_varnames) }
  .debug_pubtypes 0 : { *(.debug_pubtypes) }
  .debug_ranges   0 : { *(.debug_ranges) }
  /DISCARD/ : { *(.rel.dyn) }
  .gnu.attributes 0 : { KEEP (*(.gnu.attributes)) }
  /DISCARD/ : { *(.note.GNU-stack) }
  /DISCARD/ : { *(.note.GNU-stack) *(.gnu_debuglink) *(.gnu.lto_*) *(.discard) }
  /DISCARD/ : { *(._debug_exception) }
<#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == true>
  /DISCARD/ : { *(.config_*) }
</#if>
}
