BUILD_DIR=build
PREFIX=/rno-g
CFLAGS=-fPIC -Os 

# are we on the BBB? 
ifeq ($(shell uname -m),armv7l) 
CFLAGS+=-mfpu=neon
endif

LDFLAGS=-shared 
LIBS=-lz
INCLUDES=src/rno-g.h
DAQ_INCLUDES=src/radiant.h 

.PHONY: client daq clean install install-daq

client:  $(BUILD_DIR)/librno-g.so  

daq: client $(BUILD_DIR)/libradiant.so 

install: 

install-daq: install 

clean: 
	rm -rf build 


$(BUILD_DIR): 
	mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/librno-g.so: $(BUILD_DIR)/rno-g.o
	cc -o $@ $(LDFLAGS) $^  $(LIBS) 

$(BUILD_DIR)/libradiant.so: $(BUILD_DIR)/radiant.o
	cc -o $@ $(LDFLAGS) $^  $(LIBS) 
 
$(BUILD_DIR)/%.o: src/%.c $(INCLUDES) | $(BUILD_DIR)
	cc -c -o $@ $(CFLAGS) $< 
  
