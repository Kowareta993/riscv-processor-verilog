addi x1, x0, -10
addi x2, x0, 20
addi x3, x0, 0
bltu x2, x1, label1 # taken
addi x31, x0, 70
label1:
    add x3, x3, x1;
    addi x1, x1, 1;
bltu x2, x1, label1 # do while(x1<x2)
bltu x0, x0, label1 # not taken
addi x4, x0, 23
ecall