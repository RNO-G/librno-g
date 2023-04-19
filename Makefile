BUILD_DIR=build
RNO_G_INSTALL_DIR?=/rno-g/
PREFIX?=$(RNO_G_INSTALL_DIR)

include config.mk

CFLAGS=-fPIC -Og -Wall -Wextra -g -std=gnu11 -I./src -DRADIANT_SPI_SPEED=$(RADIANT_SPI_SPEED_MHZ)
CFLAGS+=$(EXTRA_CFLAGS) 
CXXFLAGS+=-fPIC -Og -Wall -Wextra -g

#CFLAGS+=-DRADIANT_SET_DBG

ON_BBB=no
# are we on the BBB? 
ifeq ($(shell uname -m),armv7l) 
CFLAGS+=-mfpu=neon
CFLAGS+=-DON_BBB 
ON_BBB=yes
endif

LDFLAGS=-shared 
LIBS=-lz -pthread
INCLUDES=src/rno-g.h src/rno-g-wf-utils.h
DAQ_INCLUDES=src/radiant.h src/cobs.h src/adf4350.h src/flower.h src/rno-g-cal.h 
PYBIND_INCLUDES=$(shell python3 -m pybind11 --includes) 

.PHONY: client daq clean install install-daq client-py daq-py cppcheck test daq-test-progs rno-g-utils

client:  $(BUILD_DIR)/librno-g.so  

daq: client $(BUILD_DIR)/libradiant.so  $(BUILD_DIR)/libflower.so $(BUILD_DIR)/librno-g-cal.so 

daq-test-progs:  $(addprefix $(BUILD_DIR)/test/, flower-configure-trigger flower-dump flower-equalize flower-set-thresholds\
	                  flower-status flower-trigger-enables flower-trigout-enables flower-wave radiant-check-trigger radiant-dump\
										radiant-scan radiant-threshold-scan radiant-try-daqstatus radiant-try-event radiant-try-ped cal-cmd)

rno-g-utils:  $(addprefix $(BUILD_DIR)/test/, rno-g-dump-ds rno-g-dump-hdr rno-g-dump-ped rno-g-dump-wf rno-g-wf-sample-diff-hists rno-g-wf-stats)

client-py: client $(BUILD_DIR)/rno_g.so 
daq-py: daq client-py $(BUILD_DIR)/radiant.so 


BLUE=\e[1;34m
RED=\e[1;31m
BOLD=\e[1m
GREEN=\e[1;32m
NC=\e[0m

$(shell printf "/*This file is auto-generated by the Makefile!*/\n#include \"rno-g.h\"\n\nconst char *  rno_g_get_git_hash() { return \"$$(git describe --always --dirty --match 'NOT A TAG')\";}" > src/rno-g-version.c.tmp; if diff -q src/rno-g-version.c.tmp src/rno-g-version.c >/dev/null 2>&1; then rm src/rno-g-version.c.tmp; else mv src/rno-g-version.c.tmp src/rno-g-version.c; fi)

## Super hacky test-suite :) 
TESTS=test-cobs neon-test
test: $(addprefix $(BUILD_DIR)/test/, $(TESTS) )
	@mkdir -p $(BUILD_DIR)/test-outputs/ 
	@let total=0 ; \
	let passed=0 ; \
	for test in $(TESTS) ; do \
 	echo -e "$(BLUE)Running test$(NC): $$test " ; \
 	let total++ ; \
	mkdir -p $(BUILD_DIR)/test-tmp/$$test ;\
	TEST_TMPDIR=$(BUILD_DIR)/test-tmp/$$test LD_LIBRARY_PATH+=:$(BUILD_DIR) $(BUILD_DIR)/test/$$test &> $(BUILD_DIR)/test-outputs/$$test ; \
	result=$$?; \
	if [ $$result -ne 0 ]; then \
	echo -e "$(RED)  FAILED (returned $$result)  $(NC)" ; \
	cat $(BUILD_DIR)/test-outputs/$$test ; \
	else  \
	echo -e '$(GREEN) PASSED $(NC)' ; \
	let passed++; \
	fi ; \
	done ; \
	echo -e "\n$(BOLD) $$passed/$$total tests passed$(NC)\n " 
		

install: client
	mkdir -p $(PREFIX)/lib
	mkdir -p $(PREFIX)/include
	install $(BUILD_DIR)/librno-g.so $(PREFIX)/lib/
	install $(INCLUDES) $(PREFIX)/include/

install-daq: install $(BUILD_DIR)/libradiant.so $(BUILD_DIR)/libflower.so $(BUILD_DIR)/librno-g-cal.so  
	install $(BUILD_DIR)/libradiant.so $(PREFIX)/lib/
	install $(BUILD_DIR)/libflower.so $(PREFIX)/lib/
	install $(BUILD_DIR)/librno-g-cal.so $(PREFIX)/lib/
	install src/radiant.h src/flower.h src/rno-g-cal.h $(PREFIX)/include/

ifeq ($(ON_BBB),yes)
	mkdir -p /data/test
	chown rno-g:rno-g /data/test 
	ldconfig  # just put this here... doesn't seem to be needed on my laptop but mabye on BBB (Debian things?) 
endif

clean: 
	@echo Nuking $(BUILD_DIR) from orbit
	@ rm -rf $(BUILD_DIR) 

$(BUILD_DIR): 
	@ mkdir -p $(BUILD_DIR)
	@ mkdir -p $(BUILD_DIR)/test

CLIENT_OBJS=rno-g.o rno-g-version.o rno-g-wf-utils.o
$(BUILD_DIR)/librno-g.so: $(addprefix $(BUILD_DIR)/, $(CLIENT_OBJS))
	@echo Linking $@
	@cc -o $@ $(LDFLAGS) $^  $(LIBS) 

RAD_OBJS=radiant.o cobs.o adf4350.o 
$(BUILD_DIR)/libradiant.so: $(addprefix $(BUILD_DIR)/, $(RAD_OBJS))
	@echo Linking $@
	@cc -o $@ $(LDFLAGS) $^  $(LIBS) 

CAL_OBJS=rno-g-cal.o 
$(BUILD_DIR)/librno-g-cal.so: $(addprefix $(BUILD_DIR)/, $(CAL_OBJS))
	@echo Linking $@
	@cc -o $@ $(LDFLAGS) $^  $(LIBS) 

FLWR_OBJS=flower.o 
$(BUILD_DIR)/libflower.so: $(addprefix $(BUILD_DIR)/, $(FLWR_OBJS))
	@echo Linking $@
	@cc -o $@ $(LDFLAGS) $^  $(LIBS) 


# non-DAQ objects begin with rno-.... haas to be rno- instead of rno-g so that rno-g.c works :) 
$(BUILD_DIR)/rno-%.o: src/rno-%.c $(INCLUDES) | $(BUILD_DIR)
	@echo Compiling non-DAQ object $@
	@cc -c -o $@ $(CFLAGS) $< 


$(BUILD_DIR)/rno_g.so:  src/rno-g-pybind.cc  $(INCLUDES) $(BUILD_DIR)/librno-g.so | $(BUILD_DIR)
	@echo Generating Python bindings for rno-g. This might take a while if you are on the BBB
	@c++ $(CXXFLAGS) -shared -std=c++11  $(PYBIND_INCLUDES) $< -o $@ -L$(BUILD_DIR) -lrno-g

$(BUILD_DIR)/radiant.so:  src/radiant-pybind.cc  $(INCLUDES) $(BUILD_DIR)/librno-g.so $(BUILD_DIR)/libradiant.so | $(BUILD_DIR)
	@echo Generating Python bindings for radiant. This might take a while if you are on the BBB
	@c++ $(CXXFLAGS) -shared -std=c++11 $(PYBIND_INCLUDES) $< -o $@ -L$(BUILD_DIR) -lrno-g -lradiant -lm

$(BUILD_DIR)/%.o: src/%.c $(DAQ_INCLUDES) | $(BUILD_DIR)
	@echo Compiling $@
	@cc -c -o $@ $(CFLAGS) $< 


$(BUILD_DIR)/test/rno-g-%: test/rno-g-%.c $(INCLUDES) $(BUILD_DIR)/librno-g.so | $(BUILD_DIR)
	@echo Compiling $@
	@cc  -o $@ $(CFLAGS) -Isrc/ -L$(BUILD_DIR) -lrno-g -lz -lm $< 

$(BUILD_DIR)/test/%: test/%.c $(INCLUDES) $(DAQ_INCLUDES) $(BUILD_DIR)/librno-g.so $(BUILD_DIR)/libradiant.so $(BUILD_DIR)/libflower.so $(BUILD_DIR)/librno-g-cal.so | $(BUILD_DIR)
	@echo Compiling $@
	@cc  -o $@ $(CFLAGS) -Isrc/ -L$(BUILD_DIR) -lradiant -lrno-g -lflower -lrno-g-cal -lz -lm $< 

$(BUILD_DIR)/test/%: test/%.py $(INCLUDES) $(DAQ_INCLUDES) $(BUILD_DIR)/librno-g.so  $(BUILD_DIR)/_rno_g.so $(BUILD_DIR)/libradiant.so | $(BUILD_DIR)
	ln  $@ $<


cppcheck: 
	cppcheck --enable=portability --enable=performance --enable=information  src 

config.mk: 
	@echo "Creating a default config.mk"
	@cat config.mk.default > $@

