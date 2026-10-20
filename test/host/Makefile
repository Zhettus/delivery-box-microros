# Host unit tests: door logic (state machine, current guard, latch) and the
# audio DSP (components/audio: mic_dsp.c, audio_resample.c).
#
#   cd test/host && make        # build + run
#   make clean
#
# Only needs gcc and make; no ESP-IDF, no board.

COMP    := ../../components
CFLAGS  := -std=c11 -O1 -g -Wall -Wextra -Werror -Wno-unused-function \
           -fsanitize=address,undefined \
           -I$(COMP)/door/include -I$(COMP)/audio/include
LDLIBS  := -lm

TESTS := test_door_fsm test_mic_dsp

all: $(TESTS)
	@for t in $(TESTS); do ./$$t || exit 1; done
	@echo "ALL HOST TESTS PASSED"

test_door_fsm: test_door_fsm.c $(COMP)/door/door_fsm.c $(COMP)/door/motor_control.c \
               $(COMP)/door/current_guard.c $(COMP)/door/door_lock.c
	$(CC) $(CFLAGS) $^ -o $@ $(LDLIBS)

test_mic_dsp: test_mic_dsp.c $(COMP)/audio/mic_dsp.c $(COMP)/audio/audio_resample.c
	$(CC) $(CFLAGS) $^ -o $@ $(LDLIBS)

clean:
	rm -f $(TESTS)

.PHONY: all clean
