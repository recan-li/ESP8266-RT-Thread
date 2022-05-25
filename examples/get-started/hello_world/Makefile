#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_NAME := hello-world

# list system components
SYS_COMPONENTS := 	app_update bootloader bootloader_support console \
					esp8266 esp_common esptool_py log partition_table \
					heap spi_flash esp_ringbuf esp_event tcpip_adapter lwip vfs nvs_flash \
					wpa_supplicant newlib mbedtls pthread

# list RTOS components
RTOS_COMPONENTS := freertos freertos_wrapper_rtthread

# add app components
COMPONENTS := $(sort $(SYS_COMPONENTS) $(RTOS_COMPONENTS) main)

include $(IDF_PATH)/make/project.mk

