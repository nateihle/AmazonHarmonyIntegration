# int16 mips_hadj_lms16(int16 in, int16 ref, int16 *coeffs, int16 *delayline,
#           int16 *error, int16 K, int mu)
# $a0 - in(n)
# $a1 - ref(n)
# $a2 - coeffs(n)
# $a3 - delayline(n)
# 16(sp)  - &e(n)/|X(n)|^2, normalized error 
# 20($sp) - K = 2*k >= 4
# 24($sp) - mu (adaptation rate)

# $at - &e(n) - addr return value

# $t0 - e(n-1), e(n), X(n) value and update tap k
# $t1 - H(n+1) update, k tap
# $t2 - hAdj, k tap
# $t3 - y'(n) k mult
# $t4 - X(n) value and update tap k+1
# $t5 - H(n+1) update, k+1 tap
# $t6 - hAdj, k+2
# $t7 - y'(n) k+1 mult
# $t8 - &coeffs[K-1]
# $t9 - adjust factor = *error * mu

# $s0 - e(n)/|X(n)|^2, normalized error 
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

    .global     mips_hadj_lms16
    .ent        mips_hadj_lms16

mips_hadj_lms16:

    # reserve stack space, needed if ANY storage is used
    addiu     $sp, $sp,-32    # Allocate the Stack Frame         
    sw        $s0, 28($sp)    
    sw        $s1, 24($sp)
    sw        $s2, 20($sp)
    sw        $s3, 16($sp)
    sw        $s4, 12($sp)
    #sw        $s5, 8($sp)
    #sw        $s6, 4($sp)
    #sw        $s7, 0($sp)

    #lwm32    $s0-$s2, 16+32($sp)    # load &e(n-1), K, mu
    lw      $s0, 16+32($sp)   # &e(n)/|xX(n)|^2, norm err(n)
    lw      $s1, 20+32($sp)   # K
    lw      $s2, 24+32($sp)   # mu Q1.15

    addiu   $s4, $a2, 0       # &H(n)
    addiu   $s3, $a3, 0       # &X(n) 

    addiu  $t9, $s2,  0       # mu

    li     $v0, 0x0000        # RND FIR accumulator
    lh     $v1, 0($s0)        # normalized error, e(n)
    lh     $t0, 0($s0)        # e(n-1) Q17.15 SE (Sign Ext Q1.15) 

    addiu  $t8, $s1, 0        # K

    addiu  $t8, $t8, -2       # K-2
    sll    $t8, $t8, 1        # (K-2) * sizeof(int16)
    addu   $t8, $t8, $a2      # &X[K-2] 2nd to last address FIR Coef.(xloop)

# #### Start H update
    mul    $t9, $v1, $s2      # adj(n) = mu * e(n) (Q15*Q15 -> Q2.30 no overflow)
    addiu  $t9, $t9, 0x4000   # adj round to Q1.15 (Q17.15 SE)
    sra    $t9, $t9, 15       # (Q1.15 SE)

    addiu  $a2, $s4, 0        # &H[0]
    addiu  $a3, $s3, 0        # &X[0]

    addiu  $t8, $s1, 0        # (K)
    sll    $t8, $t8, 1        # (K) * sizeof(int16)
    addu   $t8, $t8, $a2      # &X[K-1]

# ######### H Update Loop (K/2 Loops, increment by 2)
    # H(n+1) = H(n) + mu*e(n)*X(n) Update loop   
    # h[k] += hAdj[k], hAdj[k] = mu*e(n)*X[k]
hloop:
    lh     $t0, 0($a3)         # X[k]  (Q1.15, Q17.15 SE)
    lh     $t4, 2($a3)         # X[k+1](Q1.15, Q17.15 SE)
    
    mul    $t2, $t0, $t9       #  X[k]   * adj(n)   (Q1.15*Q1.15 -> Q2.30)
    mul    $t6, $t4, $t9       #  X[k+1] * adj(n)
    
    lh     $t1, 0($a2)         # h[k]   (Q1.15 Q17.15 SE)
    lh     $t5, 2($a2)         # h[k+1]

    #increment hloop pointers to next 2 sample block
    addiu  $a3, $a3, 4         # 2 * sizeof(int16)
    addiu  $a2, $a2, 4         # 2 * sizeof(int16)

    #hAdj[k] and hAdj[k+1] Q1.15
    addiu  $t2, $t2, 0x4000    # Round hAdj at after bit 15  
    sra    $t2, $t2, 15        # Q2.30 -> Q2.15, Q17.15 SE 
    addu   $t1, $t1, $t2       # h[k] += hAdj[k]  (Q1.15+Q1.15) to saturate 

    addiu  $t6, $t6, 0x4000    # Round at bit 15 
    sra    $t6, $t6, 15        # Q2.30 -> Q2.15, Q17.15 SE
    addu   $t5, $t5, $t6       # h[k+1] += hAdj[k+1] (Q1.15+Q1.15) to saturate

    sh       $t1, -4($a2)      # Save H[k]

    bne      $a2, $t8, hloop
    sh       $t5, -2($a2)      # Save H[k+1]
# #### END H Update Loop @ H[K-1]

    sh       $v1, 0($s0)       # *e(n)

    #Restore Stack Frame Registers
    lw      $s0,28($sp) 
    lw      $s1,24($sp)
    lw      $s2,20($sp)
    lw      $s3,16($sp)
    lw      $s4,12($sp)
    #lw     $s5, 8($sp)
    #lw     $s6, 4($sp)
    #lw     $s7, 0($sp)

    addiu    $sp, $sp,32  # Deallocate the Stack Frame
    
    jr       $ra
    nop

    .end        mips_hadj_lms16


