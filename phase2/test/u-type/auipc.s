auipc x1, %hi(-4096)
auipc x2, %hi(4096)
auipc x3, %hi(0x80000000)
auipc x4, %hi(0x7ffff000)

addi x5, x0, 10
auipc x5, %hi(8192)
ecall
