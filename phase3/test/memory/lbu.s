li x1, 0xfafbfcfd
li x2, 0x01020304
sw x1, 0(x0)
sw x2, 4(x0)
lbu x3, 0(x0)
lbu x4, 1(x0)
lbu x5, 2(x0)
lbu x6, 3(x0)
lbu x7, 4(x0)
lbu x8, 5(x0)
lbu x9, 6(x0)
lbu x10, 7(x0)
ecall
