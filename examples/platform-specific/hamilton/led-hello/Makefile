CONTIKI_PROJECT = led-hello

DEBUG_SCRIPTS = ./debug_scripts/
DEBUGGER = $(DEBUG_SCRIPTS)/debug.sh 
DEBUGSERVER = $(DEBUG_SCRIPTS)/debugserver.sh
FLASHER = $(DEBUG_SCRIPTS)/flash.sh
RTTTERM = $(DEBUG_SCRIPTS)/term.sh


all: $(CONTIKI_PROJECT) 

CONTIKI = ../../../..
include $(CONTIKI)/Makefile.include

.PHONY: debug, debug-server, flash, term
debug: 
	$(DEBUGGER) . led-hello-flash.elf	
debug-server:
	$(DEBUGSERVER)

flash:
	$(FLASHER) . led-hello-flash.bin
term:
	$(RTTTERM)

