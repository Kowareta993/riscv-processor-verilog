li x1, 0xfafbfcfd
li x2, 0x01020304
sw x1, 0(x0)
sw x2, 4(x0)
lhu x3, 0(x0)
lhu x4, 2(x0)
lhu x5, 4(x0)
lhu x6, 6(x0)
ecall
