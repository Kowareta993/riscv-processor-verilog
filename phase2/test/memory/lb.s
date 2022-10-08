li x1, 0xfafbfcfd
li x2, 0x01020304
sw x1, 0(x0)
sw x2, 4(x0)
lb x3, 0(x0)
lb x4, 1(x0)
lb x5, 2(x0)
lb x6, 3(x0)
lb x7, 4(x0)
lb x8, 5(x0)
lb x9, 6(x0)
lb x10, 7(x0)
ecall
