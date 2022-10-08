li x1, 0xfafbfcfd # pseudoinstruction. equls next two commented instructions
# lui x1, 0xfafc0
# addi x1, x1, 0xcfd
li x2, 0x01020304
li x3, 0xa0b0c0d0
addi x7, x0, 12
sw x1, 8(x7)
sw x2, 8(x0)
sw x3, 0(x7)
lw x4, 8(x7)
lw x5, 8(x0)
lw x6, 0(x7)
ecall
