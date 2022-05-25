
# Component Makefile
#
$(warning COMPONENT_ADD_INCLUDEDIRS=$(COMPONENT_ADD_INCLUDEDIRS))
$(warning .....................11111111111)
$(warning CONFIG_ENABLE_ESP_OSAL_RTTHREAD=$(CONFIG_ENABLE_ESP_OSAL_RTTHREAD))
$(warning CONFIG_DISABLE_FREERTOS=$(CONFIG_DISABLE_FREERTOS))

ifndef CONFIG_DISABLE_FREERTOS

COMPONENT_ADD_INCLUDEDIRS := include \
                             include/freertos \
                             include/freertos/private \
                             port/esp8266/include \
                             port/esp8266/include/freertos


COMPONENT_SRCDIRS := port/esp8266
COMPONENT_SRCDIRS += freertos

else

COMPONENT_ADD_INCLUDEDIRS 	:= 
COMPONENT_SRCDIRS 			:=

endif

$(warning COMPONENT_ADD_INCLUDEDIRS=$(COMPONENT_ADD_INCLUDEDIRS))

COMPONENT_ADD_LDFRAGMENTS += linker.lf
