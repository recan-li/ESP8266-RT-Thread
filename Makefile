
SUB_PATH := examples/get-started/hello_world/

all:
	make -C $(SUB_PATH) $@ -j32

clean:
	make -C $(SUB_PATH) $@

distclean:
	rm -rf $(SUB_PATH)/build

menuconfig:
	make -C $(SUB_PATH) $@

objdump:
	@cd $(SUB_PATH); ./objdump.sh; mv xxx.log ../../../

backtrace:
	@cd $(SUB_PATH); ./backtrace.sh "$(ADDR)"