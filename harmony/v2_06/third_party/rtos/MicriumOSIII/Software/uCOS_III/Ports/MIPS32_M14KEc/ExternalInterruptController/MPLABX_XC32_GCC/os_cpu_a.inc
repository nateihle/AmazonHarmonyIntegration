/*
*********************************************************************************************************
*                           CONSTANTS USED TO ACCESS TASK CONTEXT STACK
*********************************************************************************************************
*/

.equ    STK_OFFSET_SR,      4
.equ    STK_OFFSET_EPC,     STK_OFFSET_SR      + 4
.equ    STK_OFFSET_DSPCTRL, STK_OFFSET_EPC     + 4
.equ    STK_OFFSET_AC0LO,   STK_OFFSET_DSPCTRL + 4
.equ    STK_OFFSET_AC0HI,   STK_OFFSET_AC0LO   + 4
.equ    STK_OFFSET_AC1LO,   STK_OFFSET_AC0HI   + 4
.equ    STK_OFFSET_AC1HI,   STK_OFFSET_AC1LO   + 4
.equ    STK_OFFSET_AC2LO,   STK_OFFSET_AC1HI   + 4
.equ    STK_OFFSET_AC2HI,   STK_OFFSET_AC2LO   + 4
.equ    STK_OFFSET_AC3LO,   STK_OFFSET_AC2HI   + 4
.equ    STK_OFFSET_AC3HI,   STK_OFFSET_AC3LO   + 4
 #if defined (__mips_hard_float)      
.equ    STK_OFFSET_FPR0,    STK_OFFSET_AC3HI   + 4
.equ    STK_OFFSET_FPR1,    STK_OFFSET_FPR0    + 8
.equ    STK_OFFSET_FPR2,    STK_OFFSET_FPR1    + 8 
.equ    STK_OFFSET_FPR3,    STK_OFFSET_FPR2    + 8 
.equ    STK_OFFSET_FPR4,    STK_OFFSET_FPR3    + 8 
.equ    STK_OFFSET_FPR5,    STK_OFFSET_FPR4    + 8 
.equ    STK_OFFSET_FPR6,    STK_OFFSET_FPR5    + 8 
.equ    STK_OFFSET_FPR7,    STK_OFFSET_FPR6    + 8 
.equ    STK_OFFSET_FPR8,    STK_OFFSET_FPR7    + 8 
.equ    STK_OFFSET_FPR9,    STK_OFFSET_FPR8    + 8 
.equ    STK_OFFSET_FPR10,   STK_OFFSET_FPR9    + 8 
.equ    STK_OFFSET_FPR11,   STK_OFFSET_FPR10   + 8 
.equ    STK_OFFSET_FPR12,   STK_OFFSET_FPR11   + 8 
.equ    STK_OFFSET_FPR13,   STK_OFFSET_FPR12   + 8 
.equ    STK_OFFSET_FPR14,   STK_OFFSET_FPR13   + 8
.equ    STK_OFFSET_FPR15,   STK_OFFSET_FPR14   + 8 
.equ    STK_OFFSET_FPR16,   STK_OFFSET_FPR15   + 8 
.equ    STK_OFFSET_FPR17,   STK_OFFSET_FPR16   + 8 
.equ    STK_OFFSET_FPR18,   STK_OFFSET_FPR17   + 8  
.equ    STK_OFFSET_FPR19,   STK_OFFSET_FPR18   + 8 
.equ    STK_OFFSET_FPR20,   STK_OFFSET_FPR19   + 8 
.equ    STK_OFFSET_FPR21,   STK_OFFSET_FPR20   + 8 
.equ    STK_OFFSET_FPR22,   STK_OFFSET_FPR21   + 8  
.equ    STK_OFFSET_FPR23,   STK_OFFSET_FPR22   + 8 
.equ    STK_OFFSET_FPR24,   STK_OFFSET_FPR23   + 8 
.equ    STK_OFFSET_FPR25,   STK_OFFSET_FPR24   + 8 
.equ    STK_OFFSET_FPR26,   STK_OFFSET_FPR25   + 8  
.equ    STK_OFFSET_FPR27,   STK_OFFSET_FPR26   + 8 
.equ    STK_OFFSET_FPR28,   STK_OFFSET_FPR27   + 8 
.equ    STK_OFFSET_FPR29,   STK_OFFSET_FPR28   + 8 
.equ    STK_OFFSET_FPR30,   STK_OFFSET_FPR29   + 8      
.equ    STK_OFFSET_FPR31,   STK_OFFSET_FPR30   + 8
.equ    STK_OFFSET_FCSR,    STK_OFFSET_FPR31   + 8
.equ    STK_OFFSET_GPR1,    STK_OFFSET_FCSR    + 4 
#else
.equ    STK_OFFSET_GPR1,    STK_OFFSET_AC3HI   + 4
#endif
.equ    STK_OFFSET_GPR2,    STK_OFFSET_GPR1    + 4
.equ    STK_OFFSET_GPR3,    STK_OFFSET_GPR2    + 4
.equ    STK_OFFSET_GPR4,    STK_OFFSET_GPR3    + 4
.equ    STK_OFFSET_GPR5,    STK_OFFSET_GPR4    + 4
.equ    STK_OFFSET_GPR6,    STK_OFFSET_GPR5    + 4
.equ    STK_OFFSET_GPR7,    STK_OFFSET_GPR6    + 4
.equ    STK_OFFSET_GPR8,    STK_OFFSET_GPR7    + 4
.equ    STK_OFFSET_GPR9,    STK_OFFSET_GPR8    + 4
.equ    STK_OFFSET_GPR10,   STK_OFFSET_GPR9    + 4
.equ    STK_OFFSET_GPR11,   STK_OFFSET_GPR10   + 4
.equ    STK_OFFSET_GPR12,   STK_OFFSET_GPR11   + 4
.equ    STK_OFFSET_GPR13,   STK_OFFSET_GPR12   + 4
.equ    STK_OFFSET_GPR14,   STK_OFFSET_GPR13   + 4
.equ    STK_OFFSET_GPR15,   STK_OFFSET_GPR14   + 4
.equ    STK_OFFSET_GPR16,   STK_OFFSET_GPR15   + 4
.equ    STK_OFFSET_GPR17,   STK_OFFSET_GPR16   + 4
.equ    STK_OFFSET_GPR18,   STK_OFFSET_GPR17   + 4
.equ    STK_OFFSET_GPR19,   STK_OFFSET_GPR18   + 4
.equ    STK_OFFSET_GPR20,   STK_OFFSET_GPR19   + 4
.equ    STK_OFFSET_GPR21,   STK_OFFSET_GPR20   + 4
.equ    STK_OFFSET_GPR22,   STK_OFFSET_GPR21   + 4
.equ    STK_OFFSET_GPR23,   STK_OFFSET_GPR22   + 4
.equ    STK_OFFSET_GPR24,   STK_OFFSET_GPR23   + 4
.equ    STK_OFFSET_GPR25,   STK_OFFSET_GPR24   + 4
.equ    STK_OFFSET_GPR26,   STK_OFFSET_GPR25   + 4
.equ    STK_OFFSET_GPR27,   STK_OFFSET_GPR26   + 4
.equ    STK_OFFSET_GPR28,   STK_OFFSET_GPR27   + 4
.equ    STK_OFFSET_GPR30,   STK_OFFSET_GPR28   + 4
.equ    STK_OFFSET_GPR31,   STK_OFFSET_GPR30   + 4
.equ    STK_CTX_SIZE,       STK_OFFSET_GPR31   + 4        

/*
*********************************************************************************************************
*                                MACRO TO SAVE and RESTORE CPU CONTEXT
*
*
* Note(s): 1) The restoreCPU_CONTEXT MACRO use GP registers $8 and $9 to do work.  Therefore, these 
*             registers must be free when the MACRO is invoked.  
*          2) Any instruction, can safely follow both macros, as all execution and instruction hazards 
*             are taken care of withing the context of both MACRO's.  Instruction interlocks are handled
*             by the hardware through a pipeline slip mechanism as per the MIPS documentation.
*********************************************************************************************************
*/
.macro   saveCPU_CONTEXT

    addiu  $29, $29, -(STK_CTX_SIZE)           /* Adjust the stack pointer				*/
#if ( __mips_micromips == 1 )    
    swp    $1,  STK_OFFSET_GPR1($29)            /* Save the General Pupose Registers                    */
    swp    $3,  STK_OFFSET_GPR3($29)
    swp    $5,  STK_OFFSET_GPR5($29)
    swp    $7,  STK_OFFSET_GPR7($29)
    swp    $9,  STK_OFFSET_GPR9($29)
    swp    $11, STK_OFFSET_GPR11($29)
    swp    $13, STK_OFFSET_GPR13($29)
    sw    $15, STK_OFFSET_GPR15($29)
 SWM32    $16-$23,$16,STK_OFFSET_GPR16($29)
    swp    $24, STK_OFFSET_GPR24($29) 
    swp    $26, STK_OFFSET_GPR26($29)
    sw    $28, STK_OFFSET_GPR28($29)
    swp    $30, STK_OFFSET_GPR30($29)
#else
    sw    $1,  STK_OFFSET_GPR1($29)            /* Save the General Pupose Registers                    */
    sw    $2,  STK_OFFSET_GPR2($29)
    sw    $3,  STK_OFFSET_GPR3($29)
    sw    $4,  STK_OFFSET_GPR4($29)
    sw    $5,  STK_OFFSET_GPR5($29)
    sw    $6,  STK_OFFSET_GPR6($29)
    sw    $7,  STK_OFFSET_GPR7($29)
    sw    $8,  STK_OFFSET_GPR8($29)
    sw    $9,  STK_OFFSET_GPR9($29)
    sw    $10, STK_OFFSET_GPR10($29)
    sw    $11, STK_OFFSET_GPR11($29)
    sw    $12, STK_OFFSET_GPR12($29)
    sw    $13, STK_OFFSET_GPR13($29)
    sw    $14, STK_OFFSET_GPR14($29)
    sw    $15, STK_OFFSET_GPR15($29)
    sw    $16, STK_OFFSET_GPR16($29)
    sw    $17, STK_OFFSET_GPR17($29)
    sw    $18, STK_OFFSET_GPR18($29)
    sw    $19, STK_OFFSET_GPR19($29)
    sw    $20, STK_OFFSET_GPR20($29)
    sw    $21, STK_OFFSET_GPR21($29)
    sw    $22, STK_OFFSET_GPR22($29)
    sw    $23, STK_OFFSET_GPR23($29)
    sw    $24, STK_OFFSET_GPR24($29)
    sw    $25, STK_OFFSET_GPR25($29)
    sw    $26, STK_OFFSET_GPR26($29)
    sw    $27, STK_OFFSET_GPR27($29)
    sw    $28, STK_OFFSET_GPR28($29)
    sw    $30, STK_OFFSET_GPR30($29)
    sw    $31, STK_OFFSET_GPR31($29)
#endif    /* ( __mips_micromips == 1 )    */
    /* Ensure effective address is double word aligned */
#if defined (__mips_hard_float)  
  
    SDC1  $f0, STK_OFFSET_FPR0($29)
    SDC1  $f1, STK_OFFSET_FPR1($29)
    SDC1  $f2, STK_OFFSET_FPR2($29)
    SDC1  $f3, STK_OFFSET_FPR3($29)
    SDC1  $f4, STK_OFFSET_FPR4($29)
    SDC1  $f5, STK_OFFSET_FPR5($29)
    SDC1  $f6, STK_OFFSET_FPR6($29)
    SDC1  $f7, STK_OFFSET_FPR7($29)
    SDC1  $f8, STK_OFFSET_FPR8($29)
    SDC1  $f9, STK_OFFSET_FPR9($29)
    SDC1  $f10, STK_OFFSET_FPR10($29)
    SDC1  $f11, STK_OFFSET_FPR11($29)
    SDC1  $f12, STK_OFFSET_FPR12($29)
    SDC1  $f13, STK_OFFSET_FPR13($29)
    SDC1  $f14, STK_OFFSET_FPR14($29)
    SDC1  $f15, STK_OFFSET_FPR15($29)
    SDC1  $f16, STK_OFFSET_FPR16($29)
    SDC1  $f17, STK_OFFSET_FPR17($29)
    SDC1  $f18, STK_OFFSET_FPR18($29)
    SDC1  $f19, STK_OFFSET_FPR19($29)
    SDC1  $f20, STK_OFFSET_FPR20($29)
    SDC1  $f21, STK_OFFSET_FPR21($29)
    SDC1  $f22, STK_OFFSET_FPR22($29)
    SDC1  $f23, STK_OFFSET_FPR23($29)
    SDC1  $f24, STK_OFFSET_FPR24($29)
    SDC1  $f25, STK_OFFSET_FPR25($29)
    SDC1  $f26, STK_OFFSET_FPR26($29)
    SDC1  $f27, STK_OFFSET_FPR27($29)  
    SDC1  $f28, STK_OFFSET_FPR28($29)
    SDC1  $f29, STK_OFFSET_FPR29($29)
    SDC1  $f30, STK_OFFSET_FPR30($29)
    SDC1  $f31, STK_OFFSET_FPR31($29)
    CFC1  $8,$f31
    sw    $8,STK_OFFSET_FCSR($29)
#endif

#if ( __mips_micromips == 1 )      
    mflo  $8,  $ac0                            /* Save the contents of the ac 0-3 registers            */
    mfhi  $9,  $ac0
    swp   $8,  STK_OFFSET_AC0LO($29)

    mflo  $8,  $ac1
    mfhi  $9,  $ac1
    swp   $8,  STK_OFFSET_AC1LO($29)
    
    mflo  $8,  $ac2
    mfhi  $9,  $ac2
    swp   $8,  STK_OFFSET_AC2LO($29)

    mflo  $8,  $ac3
    mfhi  $9,  $ac3
    swp   $8,  STK_OFFSET_AC3LO($29)    
#else

    mflo  $8,  $ac0                            /* Save the contents of the ac 0-3 registers            */
    mfhi  $9,  $ac0
    sw    $8,  STK_OFFSET_AC0LO($29)
    sw    $9,  STK_OFFSET_AC0HI($29)

    mflo  $8,  $ac1
    mfhi  $9,  $ac1
    sw    $8,  STK_OFFSET_AC1LO($29)
    sw    $9,  STK_OFFSET_AC1HI($29)
    
    mflo  $8,  $ac2
    mfhi  $9,  $ac2
    sw    $8,  STK_OFFSET_AC2LO($29)
    sw    $9,  STK_OFFSET_AC2HI($29)

    mflo  $8,  $ac3
    mfhi  $9,  $ac3
    sw    $8,  STK_OFFSET_AC3LO($29)
    sw    $9,  STK_OFFSET_AC3HI($29)  
    
#endif    /* ( __mips_micromips == 1 )    */
    rddsp $8
    sw    $8,  STK_OFFSET_DSPCTRL($29)         /* Save the DSP Control Register                        */

    mfc0  $8,  $14, 0                          /* Save the EPC                                         */
    sw    $8,  STK_OFFSET_EPC($29)

    mfc0  $8,  $12, 0                          /* Save the Status register                             */
    sw    $8,  STK_OFFSET_SR($29)
.endm

/*
*********************************************************************************************************
* see Note(s), in above comment header.
*********************************************************************************************************
*/
.macro   restoreCPU_CONTEXT
    lw    $8,  STK_OFFSET_SR($29)              /* Restore the Status register                          */
    mtc0  $8,  $12, 0
    ehb
    
    lw    $8,  STK_OFFSET_EPC($29)             /* Restore the EPC                                      */
    mtc0  $8,  $14, 0
    ehb 
    
    lw    $8, STK_OFFSET_DSPCTRL($29)         /* Restore the contents of the DSP Control Register      */
    wrdsp $8
#if ( __mips_micromips == 1 )       
    lwp   $8, STK_OFFSET_AC0LO($29)           /* Restore the contents of the ac0 LO and HI registers   */
    mtlo  $8, $ac0
    mthi  $9, $ac0

    lwp   $8, STK_OFFSET_AC1LO($29)           /* Restore the contents of the ac1 LO and HI registers   */
    mtlo  $8, $ac1
    mthi  $9, $ac1
    
    lwp   $8, STK_OFFSET_AC2LO($29)           /* Restore the contents of the ac2 LO and HI registers   */
    mtlo  $8, $ac2
    mthi  $9, $ac2
    
    lwp   $8, STK_OFFSET_AC3LO($29)           /* Restore the contents of the ac3 LO and HI registers   */
    mtlo  $8, $ac3
    mthi  $9, $ac3
#else
    lw    $8, STK_OFFSET_AC0LO($29)           /* Restore the contents of the ac0 LO and HI registers   */
    lw    $9, STK_OFFSET_AC0HI($29)
    mtlo  $8, $ac0
    mthi  $9, $ac0

    lw    $8, STK_OFFSET_AC1LO($29)           /* Restore the contents of the ac1 LO and HI registers   */
    lw    $9, STK_OFFSET_AC1HI($29)
    mtlo  $8, $ac1
    mthi  $9, $ac1
    
    lw    $8, STK_OFFSET_AC2LO($29)           /* Restore the contents of the ac2 LO and HI registers   */
    lw    $9, STK_OFFSET_AC2HI($29)
    mtlo  $8, $ac2
    mthi  $9, $ac2
    
    lw    $8, STK_OFFSET_AC3LO($29)           /* Restore the contents of the ac3 LO and HI registers   */
    lw    $9, STK_OFFSET_AC3HI($29)
    mtlo  $8, $ac3
    mthi  $9, $ac3    
#endif    /* ( __mips_micromips == 1 )    */
					       /* Ensure effective address is double word aligned */
#if defined (__mips_hard_float) 
					       
    LDC1  $f0, STK_OFFSET_FPR0($29)
    LDC1  $f1, STK_OFFSET_FPR1($29)
    LDC1  $f2, STK_OFFSET_FPR2($29)
    LDC1  $f3, STK_OFFSET_FPR3($29)
    LDC1  $f4, STK_OFFSET_FPR4($29)
    LDC1  $f5, STK_OFFSET_FPR5($29)
    LDC1  $f6, STK_OFFSET_FPR6($29)
    LDC1  $f7, STK_OFFSET_FPR7($29)
    LDC1  $f8, STK_OFFSET_FPR8($29)
    LDC1  $f9, STK_OFFSET_FPR9($29)
    LDC1  $f10, STK_OFFSET_FPR10($29)
    LDC1  $f11, STK_OFFSET_FPR11($29)
    LDC1  $f12, STK_OFFSET_FPR12($29)
    LDC1  $f13, STK_OFFSET_FPR13($29)
    LDC1  $f14, STK_OFFSET_FPR14($29)
    LDC1  $f15, STK_OFFSET_FPR15($29)
    LDC1  $f16, STK_OFFSET_FPR16($29)
    LDC1  $f17, STK_OFFSET_FPR17($29)
    LDC1  $f18, STK_OFFSET_FPR18($29)
    LDC1  $f19, STK_OFFSET_FPR19($29)
    LDC1  $f20, STK_OFFSET_FPR20($29)
    LDC1  $f21, STK_OFFSET_FPR21($29)
    LDC1  $f22, STK_OFFSET_FPR22($29)
    LDC1  $f23, STK_OFFSET_FPR23($29)
    LDC1  $f24, STK_OFFSET_FPR24($29)
    LDC1  $f25, STK_OFFSET_FPR25($29)
    LDC1  $f26, STK_OFFSET_FPR26($29)
    LDC1  $f27, STK_OFFSET_FPR27($29)  
    LDC1  $f28, STK_OFFSET_FPR28($29)
    LDC1  $f29, STK_OFFSET_FPR29($29)
    LDC1  $f30, STK_OFFSET_FPR30($29)
    LDC1  $f31, STK_OFFSET_FPR31($29)
    lw    $8,STK_OFFSET_FCSR($29)    
    CTC1  $8,$f31
    
#endif
#if ( __mips_micromips == 1 )                 /* Restore the General Purpose Registers                */
    lwp    $30, STK_OFFSET_GPR30($29) 
    lw    $28, STK_OFFSET_GPR28($29)
    lwp    $26, STK_OFFSET_GPR26($29) 
    lwp    $24, STK_OFFSET_GPR24($29) 
 lwm32    $16-$23,$16,STK_OFFSET_GPR16($29) 
    lw    $15, STK_OFFSET_GPR15($29) 
    lwp    $13, STK_OFFSET_GPR13($29) 
    lwp    $11, STK_OFFSET_GPR11($29) 
    lwp    $9,  STK_OFFSET_GPR9($29)  
    lwp    $7,  STK_OFFSET_GPR7($29)  
    lwp    $5,  STK_OFFSET_GPR5($29)  
    lwp    $3,  STK_OFFSET_GPR3($29) 
    lwp    $1,  STK_OFFSET_GPR1($29) 
#else
    lw    $31, STK_OFFSET_GPR31($29)           /* Restore the General Purpose Registers                */
    lw    $30, STK_OFFSET_GPR30($29) 
    lw    $28, STK_OFFSET_GPR28($29)
    lw    $27, STK_OFFSET_GPR27($29) 
    lw    $26, STK_OFFSET_GPR26($29) 
    lw    $25, STK_OFFSET_GPR25($29) 
    lw    $24, STK_OFFSET_GPR24($29) 
    lw    $23, STK_OFFSET_GPR23($29) 
    lw    $22, STK_OFFSET_GPR22($29) 
    lw    $21, STK_OFFSET_GPR21($29) 
    lw    $20, STK_OFFSET_GPR20($29) 
    lw    $19, STK_OFFSET_GPR19($29) 
    lw    $18, STK_OFFSET_GPR18($29) 
    lw    $17, STK_OFFSET_GPR17($29) 
    lw    $16, STK_OFFSET_GPR16($29) 
    lw    $15, STK_OFFSET_GPR15($29) 
    lw    $14, STK_OFFSET_GPR14($29) 
    lw    $13, STK_OFFSET_GPR13($29) 
    lw    $12, STK_OFFSET_GPR12($29) 
    lw    $11, STK_OFFSET_GPR11($29) 
    lw    $10, STK_OFFSET_GPR10($29) 
    lw    $9,  STK_OFFSET_GPR9($29)  
    lw    $8,  STK_OFFSET_GPR8($29)  
    lw    $7,  STK_OFFSET_GPR7($29)  
    lw    $6,  STK_OFFSET_GPR6($29)  
    lw    $5,  STK_OFFSET_GPR5($29)  
    lw    $4,  STK_OFFSET_GPR4($29)  
    lw    $3,  STK_OFFSET_GPR3($29) 
    lw    $2,  STK_OFFSET_GPR2($29)  
    lw    $1,  STK_OFFSET_GPR1($29) 
#endif    /* ( __mips_micromips == 1 )    */
                               
    addiu  $29, $29, STK_CTX_SIZE              /* Adjust the stack pointer                             */
    eret
.endm