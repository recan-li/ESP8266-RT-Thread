# T-HEAD SMART-EVB Introduction
SMART-EVB is a development board provided by T-HEAD, based on FPGA to provide implementation, integrating T-HEAD's RISC-V CPU (eg. E902/E906/C906) and C-SKY CPU (eg. E805/E804/E803/E802 ), integrates basic peripheral resources such as GPIO/TIMER/UART/RAM.

## The main resources on board are as follows:

1. SMART-EVB for T-Head CPU E9xx Series

| res | description |
| -- | -- |
|ISA | RISCV |
|CPU | E906  |
|FREQ| 20MHz |
|SRAM| 768KB |


2. SMART-EVB for E804/E804F/E804D

| res | description |
| -- | -- |
|ISA | C-SKY |
|CPU | E804  |
|FREQ| 20MHz |
|SRAM| 768KB |


# Compile T-HEAD BSP
SMART-EVB BSP supports GCC compiler, the version information is:
1. SMART-EVB for E906/7/F/D/P

| IDE/Compiler| version|
| - | - |
| GCC | gcc version 8.4.0 (C-SKY RISCV Tools V1.9.6 B20200616) |
2. select cpu in rtconfig.py
3. scons -c; scons

# Quick start with qemu
1. download qemu
wget https://occ-oss-prod.oss-cn-hangzhou.aliyuncs.com/resource/1356021/1612269502091/csky-qemu-x86_64-Ubuntu-16.04-20210202-1445.tar.gz
2. qemu run
qemu-system-riscv32 -cpu e906fdp -M smartl -kernel rtthread-e9xx.elf -nographic

# Quick start with smart-evb
1. Connect JTAG
2. Connect the serial port
3. riscv64-unknown-elf-gdb rtthread-e906f.elf

run log as follows:
```bash
 \ | /
- RT -     Thread Operating System
 / | \     4.0.3 build Sep  2 2020
 2006 - 2020 Copyright by rt-thread team
msh >
```
