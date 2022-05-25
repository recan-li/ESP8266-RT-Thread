
# Component Makefile
#

$(warning CONFIG_ENABLE_ESP_OSAL_RTTHREAD=$(CONFIG_ENABLE_ESP_OSAL_RTTHREAD))

ifeq ($(CONFIG_ENABLE_ESP_OSAL_RTTHREAD),y)

COMPONENT_ADD_INCLUDEDIRS := include \
                             include/freertos \
                             include/freertos/private \
                             port/esp8266/include \
                             port/esp8266/include/freertos


COMPONENT_SRCDIRS := port/esp8266
COMPONENT_SRCDIRS += freertos

# add rt-thread

COMPONENT_ADD_INCLUDEDIRS	+= rtthread
COMPONENT_ADD_INCLUDEDIRS	+= rtthread/include
COMPONENT_ADD_INCLUDEDIRS	+= rtthread/include/libc
COMPONENT_ADD_INCLUDEDIRS	+= rtthread/finsh
COMPONENT_ADD_INCLUDEDIRS	+= rtthread/bsp
	
COMPONENT_SRCDIRS			+= rtthread/bsp
COMPONENT_SRCDIRS			+= rtthread/finsh
COMPONENT_SRCDIRS			+= rtthread/src

$(warning ------------------COMPONENT_ADD_INCLUDEDIRS=$(COMPONENT_ADD_INCLUDEDIRS))

COMPONENT_ADD_LDFRAGMENTS 	+= linker.lf

endif