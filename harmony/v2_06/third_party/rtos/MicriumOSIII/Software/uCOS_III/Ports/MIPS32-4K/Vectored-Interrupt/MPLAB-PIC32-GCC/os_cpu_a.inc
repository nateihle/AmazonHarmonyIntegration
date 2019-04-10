/*
*********************************************************************************************************
*                           CONSTANTS USED TO ACCESS TASK CONTEXT STACK
*********************************************************************************************************
*/
.equ    STK_OFFSET_SR,      4
.equ    STK_OFFSET_EPC,     STK_OFFSET_SR    + 4
.equ    STK_OFFSET_LO,      STK_OFFSET_EPC   + 4
.equ    STK_OFFSET_HI,      STK_OFFSET_LO    + 4
.equ    STK_OFFSET_GPR1,    STK_OFFSET_HI    + 4
.equ    STK_OFFSET_GPR2,    STK_OFFSET_GPR1  + 4
.equ    STK_OFFSET_GPR3,    STK_OFFSET_GPR2  + 4
.equ    STK_OFFSET_GPR4,    STK_OFFSET_GPR3  + 4
.equ    STK_OFFSET_GPR5,    STK_OFFSET_GPR4  + 4
.equ    STK_OFFSET_GPR6,    STK_OFFSET_GPR5  + 4
.equ    STK_OFFSET_GPR7,    STK_OFFSET_GPR6  + 4
.equ    STK_OFFSET_GPR8,    STK_OFFSET_GPR7  + 4
.equ    STK_OFFSET_GPR9,    STK_OFFSET_GPR8  + 4
.equ    STK_OFFSET_GPR10,   STK_OFFSET_GPR9  + 4
.equ    STK_OFFSET_GPR11,   STK_OFFSET_GPR10 + 4
.equ    STK_OFFSET_GPR12,   STK_OFFSET_GPR11 + 4
.equ    STK_OFFSET_GPR13,   STK_OFFSET_GPR12 + 4
.equ    STK_OFFSET_GPR14,   STK_OFFSET_GPR13 + 4
.equ    STK_OFFSET_GPR15,   STK_OFFSET_GPR14 + 4
.equ    STK_OFFSET_GPR16,   STK_OFFSET_GPR15 + 4
.equ    STK_OFFSET_GPR17,   STK_OFFSET_GPR16 + 4
.equ    STK_OFFSET_GPR18,   STK_OFFSET_GPR17 + 4
.equ    STK_OFFSET_GPR19,   STK_OFFSET_GPR18 + 4
.equ    STK_OFFSET_GPR20,   STK_OFFSET_GPR19 + 4
.equ    STK_OFFSET_GPR21,   STK_OFFSET_GPR20 + 4
.equ    STK_OFFSET_GPR22,   STK_OFFSET_GPR21 + 4
.equ    STK_OFFSET_GPR23,   STK_OFFSET_GPR22 + 4
.equ    STK_OFFSET_GPR24,   STK_OFFSET_GPR23 + 4
.equ    STK_OFFSET_GPR25,   STK_OFFSET_GPR24 + 4
.equ    STK_OFFSET_GPR26,   STK_OFFSET_GPR25 + 4
.equ    STK_OFFSET_GPR27,   STK_OFFSET_GPR26 + 4
.equ    STK_OFFSET_GPR28,   STK_OFFSET_GPR27 + 4
.equ    STK_OFFSET_GPR30,   STK_OFFSET_GPR28 + 4
.equ    STK_OFFSET_GPR31,   STK_OFFSET_GPR30 + 4
.equ    STK_CTX_SIZE,       STK_OFFSET_GPR31 + 4



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
   addiu $29, $29, -STK_CTX_SIZE              /* Adjust the stack pointer                             */

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
                                               /* Save the contents of the LO and HI registers         */
   mflo  $8
   mfhi  $9
   sw    $8,  STK_OFFSET_LO($29)
   sw    $9,  STK_OFFSET_HI($29)

   mfc0  $8,  $14, 0                          /* Save the EPC                                         */
   sw    $8,  STK_OFFSET_EPC($29)

   mfc0  $8,  $12, 0
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

   lw    $8,  STK_OFFSET_EPC($29)             /* Restore the EPC                                      */
   mtc0  $8,  $14, 0

   lw    $8,  STK_OFFSET_LO($29)              /* Restore the contents of the LO and HI registers      */
   lw    $9,  STK_OFFSET_HI($29)
   mtlo  $8
   mthi  $9

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

   addiu $29, $29, STK_CTX_SIZE               /* Adjust the stack pointer                             */
   
   eret
.endm