#ifndef _INTERRUPTS_H_
#define _INTERRUPTS_H_

#include <Arduino.h>

#include "types.h"

inline u8 irqs_pause(void) {
  cli();
  return 0;
}

inline void irqs_resume(u8 irq_flags) {
  sei();
}

#endif
