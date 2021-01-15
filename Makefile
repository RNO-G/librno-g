BUILD_DIR=build
PREFIX=/rno-g
CFLAGS=-fPIC -Os 

# are we on the BBB? 
ifeq ($(shell uname -m),armv7l) 
CFLAGS+=-mfpu=neon
CFLAGS+=-DON_BBB 
endif

LDFLAGS=-shared 
LIBS=-lz
INCLUDES=src/rno-g.h
DAQ_INCLUDES=src/radiant.h src/cobs.h 

.PHONY: client daq clean install install-daq

client:  $(BUILD_DIR)/librno-g.so  

daq: client $(BUILD_DIR)/libradiant.so 


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
	LD_LIBRARY_PATH+=$(BUILD_DIR) $(BUILD_DIR)/test/$$test &> $(BUILD_DIR)/test-outputs/$$test ; \
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
		


		


install: 

install-daq: install 

clean: 
	rm -rf build 

.DUMMY: install install-daq clean client daq test 


$(BUILD_DIR): 
	mkdir -p $(BUILD_DIR)
	mkdir -p $(BUILD_DIR)/test

$(BUILD_DIR)/librno-g.so: $(BUILD_DIR)/rno-g.o
	cc -o $@ $(LDFLAGS) $^  $(LIBS) 

$(BUILD_DIR)/libradiant.so: $(BUILD_DIR)/radiant.o $(BUILD_DIR)/cobs.o 
	cc -o $@ $(LDFLAGS) $^  $(LIBS) 
 
$(BUILD_DIR)/rno-g.o: src/rno-g.c $(INCLUDES) | $(BUILD_DIR)
	cc -c -o $@ $(CFLAGS) $< 

$(BUILD_DIR)/%.o: src/%.c $(DAQ_INCLUDES) | $(BUILD_DIR)
	cc -c -o $@ $(CFLAGS) $< 

$(BUILD_DIR)/test/%: test/%.c $(INCLUDES) $(DAQ_INCLUDES) $(BUILD_DIR)/librno-g.so $(BUILD_DIR)/libradiant.so | $(BUILD_DIR)
	cc  -o $@ $(CFLAGS) -Isrc/ -L$(BUILD_DIR) -lradiant -lrno-g $< 
  
