addi sp, zero, 400
addi a0, zero, 6
addi s11, zero, 20 #f
jalr ra, 0(s11)
ecall

f:
    addi sp, sp, -8
    sw ra, 8(sp)
    sw s0, 4(sp)
    beq a0, zero, default
    addi t0, zero, 1
    beq a0, t0, default
    addi s0, a0, 0
    addi a0, a0, -1
    jalr ra, 0(s11)
    addi t0, s0, -2
    addi s0, a0, 0
    addi a0, t0, 0
    jalr ra, 0(s11)
    add a0, s0, a0
    beq zero, zero, f_ret
    default:
    addi a0, zero, 1
    f_ret:
    lw s0, 4(sp) 
    lw ra, 8(sp)
    addi sp, sp, 8
    jalr zero, 0(ra)
