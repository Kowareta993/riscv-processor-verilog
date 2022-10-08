li x1, 0xfafbfcfd
li x2, 0x01020304
sw x1, 0(x0)
sw x2, 4(x0)
lh x3, 0(x0)
lh x4, 2(x0)
lh x5, 4(x0)
lh x6, 6(x0)
ecall
