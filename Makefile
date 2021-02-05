BUILD_DIR=build
PREFIX=/rno-g
CFLAGS=-fPIC -Os -Wall 
CXXFLAGS=-fPIC -Os -Wall 

# are we on the BBB? 
ifeq ($(shell uname -m),armv7l) 
CFLAGS+=-mfpu=neon
CFLAGS+=-DON_BBB 
endif

LDFLAGS=-shared 
LIBS=-lz
INCLUDES=src/rno-g.h
DAQ_INCLUDES=src/radiant.h src/cobs.h 
PYBIND_INCLUDES=$(shell python3 -m pybind11 --includes) 

.PHONY: client daq clean install install-daq client-py daq-py install-py install-daq-py 

client:  $(BUILD_DIR)/librno-g.so  

daq: client $(BUILD_DIR)/libradiant.so 

client-py: client $(BUILD_DIR)/rno_g.so 
daq-py: daq client-py $(BUILD_DIR)/radiant.so 


BLUE=\e[1;34m
RED=\e[1;31m
BOLD=\e[1m
GREEN=\e[1;32m
NC=\e[0m


## Super hacky test-suite :) 
TESTS=test-cobs
test: $(addprefix $(BUILD_DIR)/test/, $(TESTS) )
	@mkdir -p $(BUILD_DIR)/test-outputs/ 
	@let total=0 ; \
	let passed=0 ; \
	for test in $(TESTS) ; do \
 	echo -e "$(BLUE)Running test$(NC): $$test " ; \
 	let total++ ; \
	mkdir -p $(BUILD_DIR)/test-tmp/$$test ;\
	TEST_TMPDIR=$(BUILD_DIR)/test-tmp/$$test LD_LIBRARY_PATH+=$(BUILD_DIR) $(BUILD_DIR)/test/$$test &> $(BUILD_DIR)/test-outputs/$$test ; \
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
	install $(BUILD_DIR)/librno-g.so $(PREFIX)/lib 
	install $(INCLUDES) $(PREFIX)/include 

install-py: install

install-daq: install 

install-daq-py: install-daq 

clean: 
	@echo Nuking $(BUILD_DIR) from orbit
	@ rm -rf $(BUILD_DIR) 

$(BUILD_DIR): 
	@ mkdir -p $(BUILD_DIR)
	@ mkdir -p $(BUILD_DIR)/test

CLIENT_OBJS=rno-g.o 
$(BUILD_DIR)/librno-g.so: $(addprefix $(BUILD_DIR)/, $(CLIENT_OBJS))
	@echo Linking $@
	@cc -o $@ $(LDFLAGS) $^  $(LIBS) 

DAQ_OBJS=radiant.o cobs.o 
$(BUILD_DIR)/libradiant.so: $(addprefix $(BUILD_DIR)/, $(DAQ_OBJS))
	@echo Linking $@
	@cc -o $@ $(LDFLAGS) $^  $(LIBS) 
 
$(BUILD_DIR)/rno-g.o: src/rno-g.c $(INCLUDES) | $(BUILD_DIR)
	@echo Compiling $@
	@cc -c -o $@ $(CFLAGS) $< 

$(BUILD_DIR)/rno_g.so:  src/rno-g-pybind.cc  $(INCLUDES) $(BUILD_DIR)/librno-g.so | $(BUILD_DIR)
	@echo Generating Python bindings for rno-g. This might take a while if you are on the BBB
	@c++ $(CXXFLAGS) -shared -std=c++11  $(PYBIND_INCLUDES) $< -o $@ -L$(BUILD_DIR) -lrno-g

$(BUILD_DIR)/radiant.so:  src/radiant-pybind.cc  $(INCLUDES) $(BUILD_DIR)/librno-g.so $(BUILD_DIR)/libradiant.so | $(BUILD_DIR)
	@echo Generating Python bindings for radiant. This might take a while if you are on the BBB
	@c++ $(CXXFLAGS) -shared -std=c++11 $(PYBIND_INCLUDES) $< -o $@ -L$(BUILD_DIR) -lrno-g -lradiant

$(BUILD_DIR)/%.o: src/%.c $(DAQ_INCLUDES) | $(BUILD_DIR)
	@echo Compiling $@
	@cc -c -o $@ $(CFLAGS) $< 

$(BUILD_DIR)/test/%: test/%.c $(INCLUDES) $(DAQ_INCLUDES) $(BUILD_DIR)/librno-g.so $(BUILD_DIR)/libradiant.so | $(BUILD_DIR)
	@echo Compiling $@
	@cc  -o $@ $(CFLAGS) -Isrc/ -L$(BUILD_DIR) -lradiant -lrno-g $< 

$(BUILD_DIR)/test/%: test/%.py $(INCLUDES) $(DAQ_INCLUDES) $(BUILD_DIR)/librno-g.so  $(BUILD_DIR)/_rno_g.so $(BUILD_DIR)/libradiant.so | $(BUILD_DIR)
	ln  $@ $<



