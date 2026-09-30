# Host unit tests for the hardware-independent logic.
#
#   cd test/host && make        # build + run all tests
#   make clean
#
# Only needs gcc and make; no ESP-IDF, no board.

COMP    := ../../components
CFLAGS  := -std=c11 -O1 -g -Wall -Wextra -Werror -Wno-unused-function \
           -fsanitize=address,undefined \
           -I$(COMP)/door/include -I$(COMP)/led_strips/include \
           -I$(COMP)/wm8960/include -I$(COMP)/ina228/include
LDLIBS  := -lm

TESTS := test_door_fsm test_led_effects test_codecs

all: $(TESTS)
	@for t in $(TESTS); do ./$$t || exit 1; done
	@echo "ALL HOST TESTS PASSED"

test_door_fsm: test_door_fsm.c $(COMP)/door/door_fsm.c $(COMP)/door/motor_control.c \
               $(COMP)/door/current_guard.c $(COMP)/door/door_lock.c
	$(CC) $(CFLAGS) $^ -o $@ $(LDLIBS)

test_led_effects: test_led_effects.c $(COMP)/led_strips/led_effects.c
	$(CC) $(CFLAGS) $^ -o $@ $(LDLIBS)

test_codecs: test_codecs.c $(COMP)/wm8960/wm8960_clock.c $(COMP)/ina228/ina228_conv.c
	$(CC) $(CFLAGS) $^ -o $@ $(LDLIBS)

clean:
	rm -f $(TESTS)

.PHONY: all clean
