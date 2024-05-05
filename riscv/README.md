

### Getting Started

#### Setting Up Environment
The primary tools are iverilog, gtkwave and riscv32-unknown-elf-gcc cross compiler.

#### Directory Structure
The purpose of each directory is outlined below:
- build: Build directory for Verilog source code
- imuldiv: Mul/Div unit
- riscvstall: Pipelined RISC-V processor with stalling source code
- riscvlong: Pipelined RISC-V processor integrated with pipelined muldiv unit and bypass logic
- tests: Assembly test build system
- riscv: RISC-V assembly tests
- scripts: miscellaneous scripts for build system
- ubmark: benchmarks for evaluation
- vc: Additional Verilog components

### Running Assembly Tests:
```
cd $CORE_ROOT/tests
mkdir build && cd build
../configure --host=riscv32-unknown-elf
make
../convert
cd $CORE_ROOT/build
make
make check-asm-riscvstall
make check-asm-rand-riscvstall
make check-asm-riscvlong
make check-asm-rand-riscvlong
```

You can also run a single assembly separately by invoking the simulator
directly. For example, if we want to run the assembly test for riscv-addi on its
own:
```
cd $CORE_ROOT/build
./riscvstall-sim +exe=../tests/build/vmh/riscv-addi.vmh +stats=1
```

### Running C Microbenchmarks:
```
cd $CORE_ROOT/ubmark
mkdir build && cd build
../configure --host=riscv32-unknown-elf
make
../convert
cd $CORE_ROOT/build
make
make run-bmark-riscvstall
make run-bmark-riscvlong
```

### Disassembling Instructions for Debugging

The simulators for this lab are equipped with disassembly features that may
prove useful during debugging. There are 3 levels of disassembly, which is
specified as a command line option to the simulator, +disasm=#. For instance,
you can run the riscvstall simulator with a disassembly level of 2 like this:

```
cd $CORE_ROOT/build
./riscvstall-sim +disasm=2 +exe=../tests/build/vmh/riscv-addi.vmh
```
Of course, this option can be used in conjunction with other options like, +vcd
or +stats.

### Finally

Good luck on the project. Additionally, please avoid sharing the code in any
public forums, domains, or on GitHub, as such action will lead to an immediate
project failure.