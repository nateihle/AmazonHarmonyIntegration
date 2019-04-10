# int16 mips_errn_lms16(int16 in, int16 ref, int16 *coeffs, int16 *delayline,
#           int16 *error, int16 K, int mu)
#
# Description:
#     Updates the X(n-1) to X(n) and Calculates the y'(n), e(n) 
#
# $a0 - in
# $a1 - ref
# $a2 - coeffs
# $a3 - delayline
# 16($sp) - error from the last iteration
# 20($sp) - K = 2*k >= 4
# 24($sp) - mu (adaptation rate)

# $at - &e(n) - addr return value

# $t0 - e(n-1), e(n), X(n) value 
# $t1 - 
# $t2 - 
# $t3 - y'(n) k mult
# $t4 - X(n) value and update tap k+1
# $t5 - 
# $t6 - 
# $t7 - y'(n) k+1 mult
# $t8 - &coeffs[K-1]
# $t9 - adjust factor = *error * mu

# $s0 - e(n-1) 
# $s1 - K 
# $s2 - mu 
# $s3 - &X(n)
# $s4 - &H(n) 

# $v0 - accumulator in Q7.24 format
# $v1 - e(n) temporary

# accumulator guard bits are required to prevent overflow:
# GBIT    = Number of Guard Bits (loss of precision, bigger range)
# GBITRND = 1<<(GBITS-1),  position to round up to the next bit 

#####define GBITS    6   (ALWAYS IMPLEMENTED)
#####define GBITRND     0x0040   (See Below) 

# optional rounding of multiplication results:

#define RNDMUL

#

    .text
    .set        noreorder
    .set        nomacro
    #.set        noat

    .global     mips_errn_lms16
    .ent        mips_errn_lms16

mips_errn_lms16:

    # reserve stack space, needed if ANY storage is used
    addiu    $sp, $sp,-32         
    sw        $s0, 28($sp)    
    sw        $s1, 24($sp)
    sw        $s2, 20($sp)
    sw        $s3, 16($sp)
    sw        $s4, 12($sp)
    #sw        $s5, 8($sp)
    #sw        $s6, 4($sp)
    #sw        $s7, 0($sp)

    #lwm32    $s0-$s2, 16+32($sp)    # load &e(n-1), K, mu
    lw      $s0, 16+32($sp)   # &e(n)
    lw      $s1, 20+32($sp)   # K
    lw      $s2, 24+32($sp)   # mu

    addiu   $s4, $a2, 0       # &H(n)
    addiu   $s3, $a3, 0       # &X(n) 

    addiu  $t9, $s2,  0       # mu

    li     $v0, 0x0000        # RND FIR accumulator

    lh     $t0, 0($s0)        # e(n-1) Q17.15 SE (Sign Ext Q1.15) 
    addiu  $t8, $s1, 0        # K

    addiu  $t8, $t8, -2       # K-2
    sll    $t8, $t8, 1        # (K-2) * sizeof(int16)
    addu   $t8, $t8, $a2      # &X[K-2] 2nd to last address FIR Coef.(xloop)

# ########## FIR/XDELAY Loop (K/2 - 1 Loops, increment by 2)
xloop:
    # Perform the X(n-1) delay shift to update with x(n) at index k
    lh     $t0, 2($a3)         # X[k+1]  (Q1.15, Q17.15 SE)
    lh     $t4, 4($a3)         # X[k+2]  (Q1.15, Q17.15 SE)
    sh     $t0, 0($a3)         # X[k]   = X[k+1]
    sh     $t4, 2($a3)         # X[i+1] = X[k+2]
    addiu  $a3, $a3, 4         # X(n) Next k: 2 * sizeof(int16)
    addiu  $a2, $a2, 4         # H(n) Next k: 2 * sizeof(int16)

    # y'(n) = H(n)*X(n)
    lh     $t1, -4($a2)        # H[k]   (Q1.15 Q17.15 SE)
    lh     $t5, -2($a2)        # H[k+1]
    mul    $t3, $t0, $t1       # H[k]*X[k]  (Q1.15*Q1.15 -> Q2.30)
    mul    $t7, $t4, $t5       # H[k+1]*X[k+1]  (Q1.15*Q1.15 -> Q2.30)

    # GUARD BITS to guard agains accumulator overflow
#ifdef RNDMUL
    addiu  $t3, $t3, 0x0000    # RND GBITS(6) from Q2.30
#endif
    sra    $t3, $t3, 0         # GBITS(6) Q2+6.30-6 -> Q8.24
    addu   $v0, $v0, $t3       # acc += X[k]*H[k]

#ifdef RNDMUL
    addiu    $t7, $t7, 0x0000  # RND GBITS(6)
#endif
    sra      $t7, $t7, 0       # GBITS(6) Q2+6.30-6 -> Q8.24

    bne      $a2, $t8, xloop
    addu     $v0, $v0, $t7     # acc += x2 * c2 (Q2.15 or Q8.7 adds)
# ###### END FIR/XDELAY Loop
    #Branch Stall Cycles

    #Last two X(n) update with x(n)
    lh       $t0, 2($a3)       # X[K-1]

    lh       $t1, 0($a2)       # H[K-2]
    lh       $t5, 2($a2)       # H[K-1]

    addiu    $t8, $zero, 0x7FFF # load upper limit 0x7FFF
    addiu    $t9, $zero, 0x8000 # load lower limit 0x8000 (NOTsign extended)

    mul      $t7, $t0, $t1     # X[K-1] * H[K-2] 
    mul      $t3, $a0, $t5     # x(n) * H[K-1] 

#ifdef RNDMUL
    addiu    $t7, $t7, 0x0000  #RND round at the GBITS(0)
#endif
    sra      $t7, $t7, 0       # GBITS(0) Q2+0.30-0 -> Q2.30
    addu     $v0, $v0, $t7     # acc += X[K-1] * H[K-2] 

#ifdef RNDMUL
    addiu    $t3, $t3, 0x0000  #RND round off GBITS(0)
#endif
    sra      $t3, $t3, 0       #GBITS(0) Q2+0.30-0 -> Q2.30
    addu     $v0, $v0, $t3     # acc += x(n) * H[K-1]   

    sh       $t0, 0($a3)       # X[K-2] = X[K-1]
    sh       $a0, 2($a3)       # X[K-1] = x(n) 

    sra      $v0, $v0, (15-0)  # y'(n), unscaled by GBITS(0) -> Q1.15 result
    subu     $v1, $a1, $v0     # e(n) = y(n) - y'(n) (Used for update)

    # Saturate the y'(n) filter result
    slt      $t2, $t8, $v0     # set $t2 if result larger than 0x7FFF
    movn     $v0, $t8, $t2     # positive clipping to 0x7FFF if $t2 set
    slt      $t2, $v0, $t9     # set $t2 if result smaller than 0xFFFF8000
    movn     $v0, $t9, $t2     # negative clipping to 0xFFFF8000 if $t2 set


# #### End of X(n) update, e(n), y'(n) calculations

    sh       $v1, 0($s0)       # *e(n)

    #Restore Registers
    lw      $s0,28($sp) 
    lw      $s1,24($sp)
    lw      $s2,20($sp)
    lw      $s3,16($sp)
    lw      $s4,12($sp)
    #lw     $s5, 8($sp)
    #lw     $s6, 4($sp)
    #lw     $s7, 0($sp)

    addiu   $sp, $sp,32  # Deallocate the Stack Frame
    
    jr       $ra
    nop

    .end        mips_errn_lms16


