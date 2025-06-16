#pragma once

#ifndef __A_H__
#define __A_H__

#include "timers.h"

#include "ansible_grid.h"
// #include "ansible_arc.h"
// #include "ansible_midi.h"
// #include "ansible_tt.h"
#include "ansible_ii_leader.h"


#define AB00 1
#define AB01 2
#define AB02 3
#define AB03 4

#define AB06 6
#define AB07 7
#define AB10 10
#define NMI 8

#define TR1 1
#define TR2 2
#define TR3 3
#define TR4 4

#define KEY_HOLD_TIME 8

// WARNING: order must match array order of
// connected_t_options[], ansible_mode_options[]
// in ansible_preset_docdef.c
typedef enum {
  conNONE,
  conARC,
  conGRID,
  conMIDI,
  conFLASH
} connected_t;

typedef enum {
  mArcLevels,
  mArcCycles,
  mGridKria,
  mGridMP,
  mGridES,
  mMidiStandard,
  mMidiArp,
  mTT,
  mUsbDisk,
} ansible_mode_t;
// END WARNING

extern connected_t connected;

// extern bool leader_mode;
extern uint16_t aux_param[2][4];

typedef struct {
  connected_t connected;
  ansible_mode_t arc_mode;
  ansible_mode_t grid_mode;
  ansible_mode_t midi_mode;
  ansible_mode_t none_mode;
  uint8_t i2c_addr;
  uint8_t grid_varibrightness;
  i2c_follower_t followers[I2C_FOLLOWER_COUNT];
} ansible_state_t;


// NVRAM data structure located in the flash array.
// typedef const struct {
typedef struct {
  uint8_t fresh;
  ansible_state_t state;
  kria_state_t kria_state;
  mp_state_t mp_state;
  es_state_t es_state;
  // levels_state_t levels_state;
  // cycles_state_t cycles_state;
  // midi_standard_state_t midi_standard_state;
  // midi_arp_state_t midi_arp_state;
  // tt_state_t tt_state;
  uint8_t scale[16][8];
  uint16_t tuning_table[4][120];
} nvram_data_t;

typedef struct {
  bool tr;
  uint16_t semitones;
  int16_t bend;
  uint16_t slew;
  uint16_t dac_target;
} ansible_output_t;

extern nvram_data_t f;
extern ansible_mode_t ansible_mode;
extern i2c_follower_t followers[I2C_FOLLOWER_COUNT];

extern uint16_t tuning_table[4][120];
extern ansible_output_t outputs[4];

extern void (*clockQ)(u8 phase);

void init_tuning(void);
void default_tuning(void);
void fit_tuning(int mode);

extern void handler_None(s32 data);
extern void clock_null(u8 phase);
// extern void ii_null(uint8_t *d, uint8_t l);
// extern void ii_follower_pause(void);
// extern void ii_follower_resume(void);

void set_mode(ansible_mode_t m);
void update_leds(uint8_t m);
void set_tr(uint8_t n);
void clr_tr(uint8_t n);
void set_cv_note_noii(uint8_t n, uint16_t cv, int16_t bend);
void set_cv_note(uint8_t n, uint16_t cv, int16_t bend);
void set_cv_slew(uint8_t n, uint16_t s);
void reset_outputs(void);
void toggle_follower(uint8_t n);
uint8_t get_tr(uint8_t n);
void clock_set(uint32_t n);
void clock_set_tr(uint32_t n, uint8_t phase);

void ii_ansible(uint8_t* d, uint8_t len);
void load_flash_state(void);
void flash_unfresh(void);

void ii_follower_pause();
void ii_follower_resume();

typedef void (*process_ii_t)(uint8_t *d, uint8_t l);

#endif
