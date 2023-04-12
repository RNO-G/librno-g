# librno-g

i/o and hardware access libraries for RNO-G

`make` or `make client` will compile client libraries (i.e. librno-g). `make
install` will install that (to PREFIX if it exists, otherwise to
RNO_G_INSTALL_DIR which will default to /rno-g if not defined). This is what
you need if you are reading DAQ data or converting DAQ data. 

`make daq` will compile DAQ libraries and `make install-daq` will install the
DAQ libraries. This only make sense on the SBC and is very unlikely to work on
any non-Linux system. 

`make useful-test-progs` will compile a bunch of useful programs for testing
data taking, which will appear under build/test . You can `source env.sh` to add
the relevant directories to the `PATH`/`LD_LIBRARY_PATH`. For example,
`radiant-try-event` can be used to take RADIANT data without the full DAQ for
testing (or weird one-off configurations). 

There is a start to some python bindings, but it's incomplete and likely
doesn't work yet (the goal would be ventually to have some of the board bringup
scripts take data using librno-g, but it might be easier to convert the board
bringup scripts to C). 
