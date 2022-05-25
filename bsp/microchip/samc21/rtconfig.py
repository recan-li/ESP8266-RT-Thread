import os

# toolchains options
ARCH='arm'
CPU='cortex-m0'
CROSS_TOOL='gcc'

if os.getenv('RTT_CC'):
    CROSS_TOOL = os.getenv('RTT_CC')
if os.getenv('RTT_ROOT'):
    RTT_ROOT = os.getenv('RTT_ROOT')

DEVICE_SERIES = 'SAMC21'
#DEVICE_SERIES = 'SAMC20'
DEVICE_TYPE = '__SAMC21J18A__'
#DEVICE_TYPE = '__SAMC21G18A__'
DEVICE_PART = 'samc21'

# cross_tool provides the cross compiler
# EXEC_PATH is the compiler execute path, for example, CodeSourcery, Keil MDK, IAR
if  CROSS_TOOL == 'gcc':
    PLATFORM    = 'gcc'
    EXEC_PATH   = r'C:/workspace/tools/embedcpp/xpack-arm-none-eabi-gcc-10.2.1-1.1/bin'
elif CROSS_TOOL == 'keil':
    PLATFORM    = 'armcc'
    EXEC_PATH   = r'C:/Keil_v5'
elif CROSS_TOOL == 'iar':
    PLATFORM    = 'iar'
    EXEC_PATH   = r'C:/Program Files (x86)/IAR Systems/Embedded Workbench 8.0'

if os.getenv('RTT_EXEC_PATH'):
    EXEC_PATH = os.getenv('RTT_EXEC_PATH')

BUILD = 'debug'

if PLATFORM == 'gcc':
    # toolchains
    PREFIX = 'arm-none-eabi-'
    CC = PREFIX + 'gcc'
    AS = PREFIX + 'gcc'
    AR = PREFIX + 'ar'
    CXX = PREFIX + 'g++'
    LINK = PREFIX + 'gcc'
    TARGET_EXT = 'elf'
    SIZE = PREFIX + 'size'
    OBJDUMP = PREFIX + 'objdump'
    OBJCPY = PREFIX + 'objcopy'

    DEVICE = ' -mcpu=cortex-m0plus -mthumb -ffunction-sections -fdata-sections -mlong-calls'
    CFLAGS = DEVICE + ' -Dgcc'
    AFLAGS = ' -c' + DEVICE + ' -x assembler-with-cpp'
    LFLAGS = DEVICE + ' -Wl,--gc-sections,-Map=rt-thread-' + DEVICE_PART + '.map,-cref,-u,Reset_Handler -T bsp/samc21/gcc/gcc/samc21j18a_flash.ld'

    CPATH = ''
    LPATH = ''

    if BUILD == 'debug':
        CFLAGS += ' -O0 -gdwarf-2 -g'
        AFLAGS += ' -gdwarf-2'
    else:
        CFLAGS += ' -O2'

    CXXFLAGS = CFLAGS 
    CFLAGS += ' -std=c99'

    POST_ACTION = OBJCPY + ' -O binary $TARGET rt-thread-' + DEVICE_PART + '.bin\n' + SIZE + ' $TARGET \n'

