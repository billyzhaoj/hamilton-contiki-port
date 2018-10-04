#include "contiki.h"
#include "dev/radio.h"
#include "sys/clock.h"
#include "sys/rtimer.h"
#include "at86rf233.h"

extern at86rf2xx_t *dev;

static int
init(void) {
  radio_init();
  return 1;
}

static int
prepare(const void *payload, unsigned short payload_len) {
  return 1;
}

static int
transmit(unsigned short transmit_len) {
  return 1;
}

/** Prepare & transmit a packet. */
static int
send(const void *payload, unsigned short payload_len) {
  return 1;
}

/** Read a received packet into a buffer. */
static int read(void *buf, unsigned short buf_len) {
  return 1;
}

/** Perform a Clear-Channel Assessment (CCA) to find out if there is
    a packet in the air or not. */
static int channel_clear(void) {
  return 1;
}

/** Check if the radio driver is currently receiving a packet */
static int receiving_packet(void) {
  return 1;
}

/** Check if the radio driver has just received a packet */
static int pending_packet(void) {
  return 1;
}

/** Turn the radio on. */
static int on(void) {
  return 1;
}

/** Turn the radio off. */
static int off(void) {
  return 1;
}

/** Get a radio parameter value. */
static radio_result_t
get_value(radio_param_t param, radio_value_t *value) {
  return RADIO_RESULT_ERROR;
}

/** Set a radio parameter value. */
static radio_result_t
set_value(radio_param_t param, radio_value_t value) {
  return RADIO_RESULT_ERROR;
}

/**
 * Get a radio parameter object. The argument 'dest' must point to a
 * memory area of at least 'size' bytes, and this memory area will
 * contain the parameter object if the function succeeds.
 */
static radio_result_t
get_object(radio_param_t param, void *dest, size_t size) {
  return RADIO_RESULT_ERROR;
}

/**
 * Set a radio parameter object. The memory area referred to by the
 * argument 'src' will not be accessed after the function returns.
 */
static radio_result_t
set_object(radio_param_t param, const void *src, size_t size) {
  return RADIO_RESULT_ERROR;
}

const struct radio_driver samd21_rf_driver = {
        init,
        prepare,
        transmit,
        send,
        read,
        channel_clear,
        receiving_packet,
        pending_packet,
        on,
        off,
        get_value,
        set_value,
        get_object,
        set_object
};