
if [ "$1" = "" ]; then
    elf=./build/hello-world.elf
else
    elf=$1
fi

~/compiler/gcc-xtensa-lx106/bin/xtensa-lx106-elf-objdump -l -d -x -s -S $elf > xxx.log
