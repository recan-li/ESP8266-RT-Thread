
if [ "$1" = "" ]; then
	echo "Must input addr !!!"
	echo "Uasge: ./backtrace addr [elf_file]"
	exit
else
	addr="$1"
fi

if [ "$2" = "" ]; then
    elf=./build/hello-world.elf
else
    elf=$2
fi

echo " $addr @ $elf"

result=$(echo $addr | grep ":")
if [ "$result" != "" ]; then
    #echo "addr list" $addr
    for a in $addr; do
    	a=`echo $a | awk -F ':' '{print $1}'`
    	echo addr=$a
    	#echo "~/compiler/gcc-xtensa-lx106/bin/xtensa-lx106-elf-addr2line $a -e $elf -f -C -s -p"
		~/compiler/gcc-xtensa-lx106/bin/xtensa-lx106-elf-addr2line $a -e $elf -f -C -s -p
    done
else
    echo "single addr"
    #echo "~/compiler/gcc-xtensa-lx106/bin/xtensa-lx106-elf-addr2line $addr -e $elf -f -C -s -p"
	~/compiler/gcc-xtensa-lx106/bin/xtensa-lx106-elf-addr2line $addr -e $elf -f -C -s -p
fi
