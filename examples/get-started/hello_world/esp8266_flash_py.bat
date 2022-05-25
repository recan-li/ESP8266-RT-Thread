if xxx%1 == xxxdebug goto debug_only
python ..\..\..\components\esptool_py\esptool\esptool.py --chip esp8266 --port COM7 --baud 921600 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 40m --flash_size 4MB   0x0 ..\..\..\examples\get-started\hello_world\build\bootloader\bootloader.bin 0x10000 ..\..\..\examples\get-started\hello_world\build\hello-world.bin 0x8000 ..\..\..\examples\get-started\hello_world\build\partitions_singleapp.bin
:debug_only
python ..\..\..\components\esptool_py\esptool\espuart.py COM7 115200
