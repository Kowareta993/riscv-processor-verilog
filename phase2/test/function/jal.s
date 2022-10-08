addi sp, zero, 400
addi a0, zero, 6
jal ra, f
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
    jal ra, f
    addi t0, s0, -2
    addi s0, a0, 0
    addi a0, t0, 0
    jal ra, f
    add a0, s0, a0
    beq zero, zero, f_ret
    default:
    addi a0, zero, 1
    f_ret:
    lw s0, 4(sp) 
    lw ra, 8(sp)
    addi sp, sp, 8
    jalr zero, 0(ra)
