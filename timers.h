#ifndef _TIMERS_H_
#define _TIMERS_H_

#include "types.h"

extern bool clock_external;

typedef void (*clock_pulse_t)(u8 phase);
extern volatile clock_pulse_t clock_pulse;

//------------------------------
//----- types

// callback function
typedef void (*timer_callback_t)(void* caller);

// timer class, element in linked list
typedef volatile struct _softTimer {
  // decreasing count of ticks remaining
  u32 ticksRemain;
  // interval in ticks
  u32 ticks;
  // callback function pointer
  timer_callback_t callback;
  // links
  volatile struct _softTimer* next;
  volatile struct _softTimer* prev;
  // arbitrary argument to differentiate different callers
  void* caller;
} softTimer_t;

//------------------------------
//---- functions

// initialize timers
void init_timers(void);
// add a timer to the processing list
// return 1 if set, 0 if not
u8 timer_add(softTimer_t* timer, u32 ticks, timer_callback_t callback, void* caller);
// find remove a timer from the processing list
// return 1 if removed, 0 if not found
u8 timer_remove(softTimer_t* timer);
// process the timer list; call this on each tick.
void process_timers(void);

void timer_set(softTimer_t* timer, u32 ticks);
void timer_reset(softTimer_t* timer);
void timer_reset_set(softTimer_t* timer, u32 ticks);
void timer_manual(softTimer_t* timer);

u32 time_now(void);
void time_clear(void);

// clear the list
extern void timers_clear(void);

#endif  // header guard
