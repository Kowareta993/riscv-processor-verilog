OUTPUT=output

n=\e[0m
g=\e[1;32m
r=\e[1;31m

TEST_DIRS = $(wildcard test/**/)
ASM = $(wildcard test/**/*.s)
MEM = $(ASM:%.s=%.mem)

# binutils-riscv64-linux-gnu is needed for assembleing.
%.elf: %.s
	riscv64-linux-gnu-as -march=rv32i $< -o $@

%.bin: %.elf
	dd skip=52 bs=1 if=$< of=$@ count=$$(expr $$(stat --printf="%s" $<) - 444)

%.mem: %.bin
	xxd -p -c1 $< $@

assemble: $(MEM)

INPUT ?= test/i-type/addi

obj_dir/Vriscv_machine: src/*.sv 323src/*.sv 323src/sim_main.cpp
	docker run -ti -v ${PWD}:/work												\
			verilator/verilator:latest --exe --build --cc --top riscv_machine	\
					-Wno-BLKLOOPINIT											\
					`find src 323src -iname '*.v' -o -iname '*.sv'`				\
					323src/sim_main.cpp

compile: obj_dir/Vriscv_machine

sim: compile assemble
	cp ${INPUT}.mem output/instructions.mem
	./obj_dir/Vriscv_machine

verify: sim
	diff -u ${INPUT}.reg output/regdump.reg 1>&2

verify-all: compile assemble
	@fail=0;																				\
	for test in `find test -iname '*.mem'`; do												\
		if ! make verify INPUT=$${test%".mem"}; then fail=$$(expr $$fail + 1); fi;			\
	done; 																					\
	if [ $$fail -ne 0 ]; then 																\
		echo "$r$$fail test failed from $$(find test -iname '*.mem' | wc -l) tests :($n"; 	\
	else 																					\
		echo "$gAll tests passed! ($$(find test -iname '*.mem' | wc -l) tests)$n";			\
	fi

.PHONY: clean
.PHONY: clean-mem

clean:
	rm -rf obj_dir

clean-mem:
	rm -rf $(MEM)
