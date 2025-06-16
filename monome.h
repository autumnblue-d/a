#ifndef _MONOME_H_
#define _MONOME_H_

#include <USBHost_t36.h>

#include "types.h"

// data sizes
#define MONOME_MAX_LED_BYTES 256
#define MONOME_QUAD_LEDS 8
#define MONOME_LED_ROW_BYTES 16
#define MONOME_LED_ROW_LS 4
#define MONOME_GRID_MAX_FRAMES 4
#define MONOME_GRID_MAP_BYTES 8
#define MONOME_RING_MAX_FRAMES 4
#define MONOME_RING_MAP_SIZE 32

// device enumeration
typedef enum {
  eDeviceGrid,       // any grid device
  eDeviceArc,        // any arc device
  eDeviceNumDevices  // dummy and count
} eMonomeDevice;

extern u8 monomeLedBuffer[256];
extern u8 monomeFrameDirty;

extern USBHost myusb;
extern USBSerial userial;

void serial_read();
void monome_grid_key_parse_event_data(s32 data, u8* x, u8* y, u8* z);

void monome_calc_quadrant_flag(u8 x, u8 y);
void monome_set_quadrant_flag(u8 q);

void monome_mext_refresh();

void monome_setup_mext();

u8 monome_size_x();
u8 monome_size_y();

eMonomeDevice monome_device(void);

#endif
