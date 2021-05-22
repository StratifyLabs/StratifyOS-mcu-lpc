#include <sos/config.h>

#include <mcu/tmr.h>

#include "lpc_arch.h"

const tmr_config_t m_clock_tmr_config = {
    .port = 3,
    .attr = {
        .o_flags
        = TMR_FLAG_SET_TIMER | TMR_FLAG_IS_SOURCE_CPU | TMR_FLAG_IS_AUTO_RELOAD,
        .period = SOS_USECOND_PERIOD,
        .freq = 1000000UL,
        .pin_assignment = {
            .channel[0] = {0xff, 0xff},
            .channel[1] = {0xff, 0xff},
            .channel[2] = {0xff, 0xff},
            .channel[3] = {0xff, 0xff}}}};

static const devfs_handle_t m_clock_tmr_handle
    = {.port = 3, .state = NULL, .config = &m_clock_tmr_config};

void lpc_clock_initialize(
    int (*handle_match_channel0)(void *context, const mcu_event_t *data),
    int (*handle_match_channel1)(void *context, const mcu_event_t *data),
    int (*handle_overflow)(void *context, const mcu_event_t *data)) {
  // use TIM2 -- 32-bit timer

  mcu_action_t action;
  mcu_channel_t chan_req;

  // Open the microsecond timer
  mcu_tmr_open(&m_clock_tmr_handle);
  mcu_tmr_setattr(&m_clock_tmr_handle, (void*)&m_clock_tmr_config.attr);

  // Initialize the value of the timer to zero
  mcu_tmr_set(&m_clock_tmr_handle, (void *)0);

  action.prio = 0;
  action.channel = 0; // doesn't matter
  action.o_events = MCU_EVENT_FLAG_OVERFLOW;
  action.handler.callback = handle_overflow;
  action.handler.context = 0;
  mcu_tmr_setaction(&m_clock_tmr_handle, &action);

  // This sets up the output compare unit used with the usleep() function
  chan_req.loc = 0;
  chan_req.value = SOS_USECOND_PERIOD + 1;
  lpc_clock_set_channel(&chan_req);

  action.channel = 0;
  action.o_events = MCU_EVENT_FLAG_MATCH;
  action.handler.callback = handle_match_channel0;
  action.handler.context = 0;
  mcu_tmr_setaction(&m_clock_tmr_handle, &action);

  chan_req.loc = 0;
  lpc_clock_set_channel(&chan_req);

  if (handle_match_channel1) {
    action.channel = 1;
    action.o_events = MCU_EVENT_FLAG_MATCH;
    action.handler.callback = handle_match_channel1;
    action.handler.context = 0;
    mcu_tmr_setaction(&m_clock_tmr_handle, &action);
  }

  lpc_clock_enable();
}

void lpc_clock_enable() {
  mcu_tmr_enable(&m_clock_tmr_handle, NULL);
}

u32 lpc_clock_disable() {
  mcu_tmr_disable(&m_clock_tmr_handle, NULL);
  u32 result;
  mcu_tmr_get(&m_clock_tmr_handle, &result);
  return result;
}

void lpc_clock_set_channel(const mcu_channel_t *channel) {
  mcu_tmr_setchannel(&m_clock_tmr_handle, (void *)channel);
}

void lpc_clock_get_channel(mcu_channel_t *channel) {
  mcu_tmr_getchannel(&m_clock_tmr_handle, (void *)channel);
}

u32 lpc_clock_microseconds() {
  u32 result;
  mcu_tmr_get(&m_clock_tmr_handle, &result);
  return result;
}

u32 lpc_clock_nanoseconds() { return 0; }
