/*

  b00 led1
  b01 led2

  b02 tr1
  b03 tr2
  b04 tr3
  b05 tr4

  b06 k1
  b07 k2

  b08 in1
  b09 in2
  b10 in1-detect

  nmi


  usb flash

*/

#include <Arduino.h>
#include <Bounce.h>
#include <USBHost_t36.h>
#include <MIDI.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

// Use these with the Teensy 3.5 & 3.6 & 4.1 SD card
#define SDCARD_CS_PIN BUILTIN_SDCARD
#define SDCARD_MOSI_PIN 11  // not actually used
#define SDCARD_SCK_PIN 13   // not actually used

USBHost myusb;
USBHub hub1(myusb);
USBHub hub2(myusb);
USBSerial userial(myusb);
MIDIDevice midiDevice(myusb);
// MIDIDevice_BigBuffer midiDevice(myusb);

// MIDI TRS
MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, MIDI);

#define LED1 3
#define LED2 2

#include <stdio.h>
#include <string.h>  // memcpy

// asf
// #include "delay.h"
// #include "compiler.h"
#include "flashc.h"
// #include "preprocessor.h"
#include "print_funcs.h"
// #include "intc.h"
// #include "pm.h"
#include "gpio.h"
// #include "spi.h"
// #include "sysclk.h"

// libavr32
#include "types.h"
#include "events.h"
#include "fix16.h"
// #include "i2c.h"
// #include "init_ansible.h"
// #include "init_common.h"
#include "monome.h"
// #include "midi.h"
#include "music.h"
#include "notes.h"
#include "timers.h"
// #include "util.h"
// #include "ftdi.h"
#include "ii.h"
#include "dac.h"
// #include "cdc.h"

// #include "conf_board.h"

// ansible
#include "a.h"
#include "ansible_grid.h"
// #include "ansible_arc.h"
// #include "ansible_midi.h"
// #include "ansible_tt.h"
// #include "ansible_usb_disk.h"
// #include "ansible_ii_leader.h"


#define FIRSTRUN_KEY 0x22

uint8_t front_timer;

// uint8_t preset_mode;

// __attribute__((__section__(".flash_nvram")))
nvram_data_t f;

volatile process_ii_t process_ii;
ansible_mode_t ansible_mode;

bool leader_mode = false;
uint16_t aux_param[2][4] = { { 0 } };

////////////////////////////////////////////////////////////////////////////////
// prototypes

// start/stop monome polling/refresh timers
extern void timers_set_monome(void);
extern void timers_unset_monome(void);

// check the event queue
static void check_events(void);

// handler protos
static void handler_KeyTimer(s32 data);
static void handler_Front(s32 data);
static void handler_FrontShort(s32 data);
static void handler_FrontLong(s32 data);
static void handler_MidiConnect(s32 data);
static void handler_MidiDisconnect(s32 data);

u8 flash_is_fresh(void);
void flash_write(void);
void flash_read(void);
void state_write(void);
void state_read(void);

void ii_ansible(uint8_t* d, uint8_t len);
static ansible_mode_t ii_ansible_mode_for_cmd(uint8_t cmd);
static uint8_t ii_ansible_cmd_for_mode(ansible_mode_t mode);
////////////////////////////////////////////////////////////////////////////////
// timers

static softTimer_t clockTimer = { .next = NULL, .prev = NULL };
static softTimer_t midiTimer = { .next = NULL, .prev = NULL };
static softTimer_t keyTimer = { .next = NULL, .prev = NULL };
static softTimer_t cvTimer = { .next = NULL, .prev = NULL };
static softTimer_t monomePollTimer = { .next = NULL, .prev = NULL };
static softTimer_t monomeRefreshTimer = { .next = NULL, .prev = NULL };
static softTimer_t midiPollTimer = { .next = NULL, .prev = NULL };

extern softTimer_t auxTimer[4];

uint16_t tuning_table[4][120];

ansible_output_t outputs[4];

static uint8_t clock_phase;

void (*clockQ)(u8 phase) = 0;

void handler_None(s32 data) {
  ;
  ;
}

static void clockTimer_callback(void* o) {
  // Serial.printf("clockTimer: %x\n", clockQ);
  clock_phase++;
  if (clock_phase > 1)
    clock_phase = 0;
  clockQ(clock_phase);
}

static void keyTimer_callback(void* o) {
  // Serial.printf("keyTimer: %x\n", clockQ);
  static event_t e;
  e.type = kEventKeyTimer;
  e.data = 0;
  event_post(&e);
}

static void cvTimer_callback(void* o) {
  dac_timer_update();
}

static void monome_poll_timer_callback(void* obj) {
  serial_read();
}

static void monome_refresh_timer_callback(void* obj) {
  if (monomeFrameDirty > 0) {
    static event_t e;
    e.type = kEventMonomeRefresh;
    event_post(&e);
  }
}

void midi_read() { //DF
  ;
  ;
}

static void midi_poll_timer_callback(void* obj) {
  midi_read();
}

void midi_tx(u8 note, u8 velocity, u8 channel) {
  Serial.printf("midi: %d, %d, %d\n", note, velocity, channel);
  MIDI.sendNoteOn(note, velocity, channel);
};

void midi_tx_off(u8 channel) {
  MIDI.sendControlChange(123, 0, channel);
};

void timers_set_monome(void) {
  timer_add(&monomePollTimer, 20, &monome_poll_timer_callback, NULL);
  timer_add(&monomeRefreshTimer, 17, &monome_refresh_timer_callback, NULL);
}

void timers_unset_monome(void) {
  timer_remove(&monomePollTimer);
  timer_remove(&monomeRefreshTimer);
}

void set_mode(ansible_mode_t m) {
  ansible_mode = m;
  // flashc_memset32((void*)&(f.state.mode), m, 4, true);
  // print_dbg("\r\nset mode ");
  // print_dbg_ulong(f.state.mode);

	timer_remove(&auxTimer[0]);
	timer_remove(&auxTimer[1]);
	timer_remove(&auxTimer[2]);
	timer_remove(&auxTimer[3]);

  switch (m) {
    case mGridKria:
    case mGridMP:
    case mGridES:
      set_mode_grid();
      break;
      // case mArcLevels:
      // case mArcCycles:
      // 	set_mode_arc();
      // 	break;
      // case mMidiStandard:
      // case mMidiArp:
      // 	set_mode_midi();
      // 	break;
    case mTT:
      // set_mode_tt();
      break;
    case mUsbDisk:
      // set_mode_usb_disk();
      break;
    default:
      break;
  }
}

////////////////////////////////////////////////////////////////////////////////
// event handlers

static void handler_FtdiConnect(s32 data) {
  // ftdi_setup();
}

static void handler_SerialConnect(s32 data) {
  //DF monome_setup_mext();
}

connected_t connected = conNONE;

static void handler_FtdiDisconnect(s32 data) {
  timers_unset_monome();
  app_event_handlers[kEventFrontShort] = &handler_FrontShort;
  app_event_handlers[kEventFrontLong] = &handler_FrontLong;
  connected = conNONE;
  // set_mode(f.state.mode);
}

static void handler_MonomeConnect(s32 data) {
  print_dbg("\r\n> connect: monome ");

  switch (monome_device()) {
    case eDeviceGrid:
      print_dbg("GRID");
      connected = conGRID;
      if (ansible_mode != f.state.grid_mode)
        set_mode(f.state.grid_mode);
      monomeFrameDirty++;
      app_event_handlers[kEventFrontShort] = &handler_GridFrontShort;
      app_event_handlers[kEventFrontLong] = &handler_GridFrontLong;
      break;
      // case eDeviceArc:
      // 	print_dbg("ARC");
      // 	connected = conARC;
      // 	if(ansible_mode != f.state.arc_mode)
      // 		set_mode(f.state.arc_mode);
      // 	monomeFrameDirty++;
      // 	app_event_handlers[kEventFrontShort] = &handler_ArcFrontShort;
      // 	app_event_handlers[kEventFrontLong] = &handler_ArcFrontLong;
      // 	break;
    default:
      break;
  }
  timers_set_monome();
}

static void handler_MonomePoll(s32 data) { //DF
  // monome_read_serial();
}

static void handler_MidiConnect(s32 data) {
  print_dbg("\r\n> midi connect");
  timer_add(&midiPollTimer, 8, &midi_poll_timer_callback, NULL);
  connected = conMIDI;
  f.state.none_mode = mTT;
  // flashc_memset32((void*)&(f.state.none_mode), mTT, 4, true);
  set_mode(f.state.midi_mode);
}

static void handler_MidiDisconnect(s32 data) {
  print_dbg("\r\n> midi disconnect");
  timer_remove(&midiPollTimer);
  app_event_handlers[kEventFrontShort] = &handler_FrontShort;
  app_event_handlers[kEventFrontLong] = &handler_FrontLong;
  connected = conNONE;
  set_mode(mTT);
}

static volatile bool front_held = false;

static void handler_MscConnect(s32 data) {
  print_dbg("\r\n> usb disk connect");
  if (front_held) {
    // usb_disk_select_app(ansible_mode);
  }
  set_mode(mUsbDisk);
}

static void handler_MscDisconnect(s32 data) {
  print_dbg("\r\n> usb disk disconnect");
  // usb_disk_exit();
  update_leds(0);
  app_event_handlers[kEventFront] = &handler_Front;
  // usb_disk_skip_apps(false);
}

static void handler_Front(s32 data) {
  // print_dbg("\r\n+ front ");
  // print_dbg_ulong(data);

  if (data == 1) {
    front_timer = KEY_HOLD_TIME;
    front_held = true;
  } else {
    front_held = false;
    if (front_timer) {
      static event_t e;
      e.type = kEventFrontShort;
      e.data = 0;
      event_post(&e);
    }
    front_timer = 0;
  }
}

static void handler_FrontShort(s32 data) {
  if (ansible_mode != mTT) {
    f.state.none_mode = mTT;
    // flashc_memset32((void*)&(f.state.none_mode), mTT, 4, true);
    set_mode(mTT);
  }
}

static void handler_FrontLong(s32 data) {
  print_dbg("\r\n+ front long");
  uint8_t addr = 0xA0 + (!gpio_get_pin_value(AB07) * 2) + (!gpio_get_pin_value(AB06) * 4);
  f.state.i2c_addr = addr;
  // flashc_memset8((void*)&(f.state.i2c_addr), addr, 1, true);
  // print_dbg("\r\n+ i2c address: ");
  // print_dbg_hex(f.state.i2c_addr);
  // TEST
  // if (!leader_mode) init_i2c_follower(f.state.i2c_addr);
}

static void handler_SaveFlash(s32 data) {
  flash_write();
}

static void handler_KeyTimer(s32 data) {
  static uint8_t key0_state;
  static uint8_t key1_state;
  static uint8_t keyfront_state;
  static uint8_t tr0normal_state;

  if (key0_state != !gpio_get_pin_value(AB07)) {
    key0_state = !gpio_get_pin_value(AB07);
    static event_t e;
    e.type = kEventKey;
    e.data = key0_state;
    event_post(&e);
  }

  if (key1_state != !gpio_get_pin_value(AB06)) {
    key1_state = !gpio_get_pin_value(AB06);
    static event_t e;
    e.type = kEventKey;
    e.data = key1_state + 2;
    event_post(&e);
  }

  if (keyfront_state != !gpio_get_pin_value(NMI)) {
    keyfront_state = !gpio_get_pin_value(NMI);
    static event_t e;
    e.type = kEventFront;
    e.data = keyfront_state;
    event_post(&e);
  }

  if (tr0normal_state != !gpio_get_pin_value(B10)) {
    tr0normal_state = !gpio_get_pin_value(B10);
    static event_t e;
    e.type = kEventTrNormal;
    e.data = tr0normal_state;
    event_post(&e);
  }

  if (front_timer) {
    if (front_timer == 1) {
      static event_t e;
      e.type = kEventFrontLong;
      e.data = 0;
      event_post(&e);
    }
    front_timer--;
  }

  if (connected == conGRID)
    grid_keytimer();
  // else if(connected == conARC)
  // 	arc_keytimer();
}

// assign default event handlers
static inline void assign_main_event_handlers(void) {
  app_event_handlers[kEventFront] = &handler_Front;
  app_event_handlers[kEventFrontShort] = &handler_FrontShort;
  app_event_handlers[kEventFrontLong] = &handler_FrontLong;
  app_event_handlers[kEventKeyTimer] = &handler_KeyTimer;
  app_event_handlers[kEventSaveFlash] = &handler_SaveFlash;
  app_event_handlers[kEventFtdiConnect] = &handler_FtdiConnect;
  app_event_handlers[kEventFtdiDisconnect] = &handler_FtdiDisconnect;
  app_event_handlers[kEventMscConnect] = &handler_MscConnect;
  app_event_handlers[kEventMscDisconnect] = &handler_MscDisconnect;
  app_event_handlers[kEventMonomeConnect] = &handler_MonomeConnect;
  app_event_handlers[kEventMonomeDisconnect] = &handler_None;
  app_event_handlers[kEventMonomePoll] = &handler_MonomePoll;
  app_event_handlers[kEventMonomeRefresh] = &handler_None;
  app_event_handlers[kEventMonomeGridKey] = &handler_None;
  app_event_handlers[kEventMonomeRingEnc] = &handler_None;
  app_event_handlers[kEventTr] = &handler_None;
  app_event_handlers[kEventTrNormal] = &handler_None;
  app_event_handlers[kEventKey] = &handler_None;
  app_event_handlers[kEventMidiConnect] = &handler_MidiConnect;
  app_event_handlers[kEventMidiDisconnect] = &handler_MidiDisconnect;
  app_event_handlers[kEventMidiPacket] = &handler_None;
  app_event_handlers[kEventSerialConnect] = &handler_SerialConnect;
  app_event_handlers[kEventSerialDisconnect] = &handler_FtdiDisconnect;
}

// app event loop
void check_events(void) {
  static event_t e;
  if (event_next(&e)) {
    (app_event_handlers)[e.type](e.data);
  }
}



////////////////////////////////////////////////////////////////////////////////
// flash

u8 flash_is_fresh(void) {
  return (f.fresh != FIRSTRUN_KEY);
}

void flash_unfresh(void) {
  f.fresh = FIRSTRUN_KEY;
  // flashc_memset8((void*)&(f.fresh), FIRSTRUN_KEY, 1, true);
}

void flash_write(void) {
  print_dbg("\r\n> write preset ");
  if (SD.exists("ANSIBLE.BIN")) SD.remove("ANSIBLE.BIN");
  File preset_file = SD.open("ANSIBLE.BIN", FILE_WRITE);
  preset_file.write(&f, sizeof(f));
  preset_file.close();
  // print_dbg_ulong(preset_select);
  // flashc_memset8((void*)&(f.preset_select), preset_select, 4, true);
  // flashc_memcpy((void *)&(f.state), &ansible_state, sizeof(ansible_state), true);
}

void flash_read(void) {
  print_dbg("\r\n> read preset ");
  // print_dbg_ulong(preset_select);

  // preset_select = f.preset_select;

  // memcpy(&ansible_state, &f.state, sizeof(ansible_state));

  // ...
}

////////////////////////////////////////////////////////////////////////////////
// tuning

void default_tuning(void) {
  for (uint8_t i = 0; i < 4; i++) {
    for (uint8_t j = 0; j < 120; j++) {
      tuning_table[i][j] = ET[j];
    }
  }
  memcpy((void*)f.tuning_table, tuning_table, sizeof(tuning_table));
  // flashc_memcpy((void*)f.tuning_table, tuning_table, sizeof(tuning_table), true);
}

void init_tuning(void) {
  memcpy((void*)tuning_table, f.tuning_table, sizeof(tuning_table));  
  // flashc_memcpy((void*)f.tuning_table, tuning_table, sizeof(tuning_table), true);
}

void fit_tuning(int mode) {
  switch (mode) {
    case 0:
      {  // fixed offset per channel
        for (uint8_t i = 0; i < 4; i++) {
          uint16_t offset = tuning_table[i][0];
          for (uint8_t j = 0; j < 120; j++) {
            tuning_table[i][j] = ET[j] + offset;
          }
        }
        break;
      }
    case 1:
      {  // linear fit between octaves
        for (uint8_t i = 0; i < 4; i++) {
          fix16_t step = 0;
          for (uint8_t j = 0; j < 10; j++) {
            fix16_t acc = fix16_from_int(tuning_table[i][j * 12]);
            if (j < 9) {
              step = fix16_div(
                fix16_from_int(tuning_table[i][(j + 1) * 12] - tuning_table[i][j * 12]),
                fix16_from_int(12));
            }
            for (uint8_t k = j * 12; k < (j + 1) * 12; k++) {
              tuning_table[i][k] = fix16_to_int(acc);
              acc = fix16_add(acc, step);
            }
          }
        }
        break;
      }
    default: break;
  }
}

////////////////////////////////////////////////////////////////////////////////
// functions

void init_i2c_follower(u8 a) { //DF
  // Serial.printf("> init_i2c_follower: %04x\n", a);
}

void init_i2c_leader() { //DF
  // Serial.printf("> init_i2c_leader\n");
}

extern void ii_tx_queue(u8 data);

void clock_null(u8 phase) { ;; }

void update_leds(uint8_t m) {
  if (m & 1)
    // gpio_set_gpio_pin(AB00);
    digitalWrite(LED1, HIGH);
  else
    // gpio_clr_gpio_pin(AB00);
    digitalWrite(LED1, LOW);

  if (m & 2)
    // gpio_set_gpio_pin(AB01);
    digitalWrite(LED2, HIGH);
  else
    // gpio_clr_gpio_pin(AB01);
    digitalWrite(LED2, LOW);
}

void set_tr(uint8_t n) {
  // Serial.printf("set_tr: %d\n", n);
  // gpio_set_gpio_pin(n);
  uint8_t tr = n - TR1;
  outputs[tr].tr = true;
  for (uint8_t i = 0; i < I2C_FOLLOWER_COUNT; i++) {
    bool play_follower = followers[i].active
                         && followers[i].track_en & (1 << tr);
    if (play_follower) {
      followers[i].opsx.tr(&followers[i], tr, 1);
    }
  }
}

// u8 midi_note[4] = { {0} };

void clr_tr(uint8_t n) {
  // Serial.printf("clr_tr: %d\n", n);
  // gpio_clr_gpio_pin(n);
  // u8 note = midi_note[n-TR1];
  // MIDI.sendNoteOff(note, 100, n);
  // Serial.printf("midi off: %d, %d\n", note, n);
  uint8_t tr = n - TR1;
  outputs[tr].tr = false;
  for (uint8_t i = 0; i < I2C_FOLLOWER_COUNT; i++) {
    bool play_follower = followers[i].active
                         && followers[i].track_en & (1 << tr);
    if (play_follower) {
      followers[i].opsx.tr(&followers[i], tr, 0);
    }
  }
  // Serial.printf("exit clr_tr: %d\n", n);
}

uint8_t get_tr(uint8_t n) {
  return gpio_get_pin_value(n);
}

void set_cv_note_noii(uint8_t n, uint16_t note, int16_t bend) {
  outputs[n].semitones = note;
  outputs[n].bend = bend;
  outputs[n].dac_target = (int16_t)tuning_table[n][note] + bend;
  // dac_set_value(n, outputs[n].dac_target);
  // Serial.printf("midi on: %d, %d\n", 48+note, n+1);
}

void set_cv_note(uint8_t n, uint16_t note, int16_t bend) {
  set_cv_note_noii(n, note, bend);
  // MIDI.sendNoteOn(48+note, 100, n+1);
  // midi_note[n] = 48+note;
  for (uint8_t i = 0; i < I2C_FOLLOWER_COUNT; i++) {
    bool play_follower = followers[i].active
                         && followers[i].track_en & (1 << n);
    if (play_follower) {
      uint16_t cv_transposed = (int16_t)ET[note] + bend;
      followers[i].opsx.cv(&followers[i], n, cv_transposed);
    }
  }
}

void set_cv_slew(uint8_t n, uint16_t s) {
  outputs[n].slew = s;
  // dac_set_slew(n, outputs[n].slew);
  for (uint8_t i = 0; i < I2C_FOLLOWER_COUNT; i++) {
    bool play_follower = followers[i].active
                         && followers[i].track_en & (1 << n);
    if (play_follower) {
      followers[i].opsx.slew(&followers[i], n, s);
    }
  }
}

void reset_outputs(void) {
  for (uint8_t n = 0; n < 4; n++) {
    outputs[n].slew = 0;
    // dac_set_slew(n, 0);
    outputs[n].tr = false;
    // gpio_clr_gpio_pin(n + TR1);
    for (uint8_t i = 0; i < I2C_FOLLOWER_COUNT; i++) {
      bool play_follower = followers[i].active
                           && followers[i].track_en & (1 << n);
      if (play_follower) {
        followers[i].opsx.mute(&followers[n], 0, 0);
      }
    }
  }
}


// void init_i2c_followers()
// {
//   i2c_follower_t* f;
//   f = &followers[0]; 
//   f->ops = new i2c_ops_t {
//     .init = ii_init_jf,
//     .mode = ii_mode_jf,
//     .tr = ii_tr_jf,
//     .mute = ii_mute_jf,
//     .cv = ii_u16_nop,
//     .octave = ii_octave_jf,
//     .slew = ii_u16_nop,
//     .mode_ct = 3,
//   };
//   f = &followers[1]; 
//   f->ops = new i2c_ops_t {
//     .init = ii_init_txo,
//     .mode = ii_mode_txo,
//     .tr = ii_tr_txo,
//     .mute = ii_mute_txo,
//     .cv = ii_cv_txo,
//     .octave = ii_octave_txo,
//     .slew = ii_slew_txo,
//     .mode_ct = 2,
//   };
//   f = &followers[2]; 
//   f->ops = new i2c_ops_t {
//     .init = ii_u8_nop,
//     .mode = ii_u8_nop,
//     .tr = ii_tr_txo,
//     .mute = ii_mute_txo,
//     .cv = ii_cv_txo,
//     .octave = ii_octave_txo,
//     .slew = ii_slew_txo,
//     .mode_ct = 1,
//   };
//   f = &followers[3]; 
//   f->ops = new i2c_ops_t {
//     .init = ii_u8_nop,
//     .mode = ii_mode_disting_ex,
//     .tr = ii_tr_disting_ex,
//     .mute = ii_mute_disting_ex,
//     .cv = ii_cv_disting_ex,
//     .octave = ii_s8_nop,
//     .slew = ii_u16_nop,
//     .mode_ct = 4,
//   };
//   f = &followers[4]; 
//   f->ops = new i2c_ops_t {
//     .init = ii_init_wsyn,
//     .mode = ii_mode_wsyn,
//     .tr = ii_tr_wsyn,
//     .mute = ii_mute_wsyn,
//     .cv = ii_cv_wsyn,
//     .octave = ii_s8_nop,
//     .slew = ii_u16_nop,
//     .mode_ct = 2,
//   };
//   f = &followers[5]; 
//   f->ops = new i2c_ops_t {
//     .init = ii_u8_nop,
//     .mode = ii_mode_crow,
//     .tr = ii_tr_crow,
//     .mute = ii_u8_nop,
//     .cv = ii_u16_nop,
//     .octave = ii_s8_nop,
//     .slew = ii_u16_nop,
//     .mode_ct = 1,
//   };
// }

void follower_on(uint8_t n) {
  // Serial.printf("follower on: %d\n", n);
  for (uint8_t i = 0; i < 4; i++) {
    followers[n].opsx.init(&followers[n], i, 1);
    followers[n].opsx.mode(&followers[n], i, followers[n].active_mode);
    followers[n].opsx.octave(&followers[n], 0, followers[n].oct);
  }
  // Serial.printf("follower on: done\n");
}

void follower_off(uint8_t n) {
  // Serial.printf("follower off: %d\n", n);
  for (uint8_t i = 0; i < 4; i++) {
    followers[n].opsx.init(&followers[n], i, 0);
  }
  // Serial.printf("follower off: done\n");
}

void toggle_follower(uint8_t n) {
  // Serial.printf("toggle_follower: %d\n");
  followers[n].active = !followers[n].active;
  if (followers[n].active) {
    for (uint8_t i = 0; i < I2C_FOLLOWER_COUNT; i++) {
      if (i != n && followers[i].active) {
        follower_on(n);
        return;
      }
    }
    print_dbg("\r\n> enter i2c leader mode");
    leader_mode = true;
    init_i2c_leader();
    follower_on(n);
  } else {
    follower_off(n);
    for (uint8_t i = 0; i < I2C_FOLLOWER_COUNT; i++) {
      if (i != n && followers[i].active) {
        return;
      }
    }
    print_dbg("\r\n> exit i2c leader mode");
    leader_mode = false;
    ii_follower_resume();
  }
  // Serial.printf("toggle_follower: done\n");
}

void ii_follower_pause(void) {
  if (!leader_mode) {
    // 0x03 is a reserved address 'for future use' in the i2c spec
    // used to effectively stop listening for i2c
    init_i2c_follower(0x03);
  }
}

void ii_follower_resume(void) {
  if (!leader_mode) {
    switch (ansible_mode) {
      case mArcLevels:
        init_i2c_follower(II_LV_ADDR);
        break;
      case mArcCycles:
        init_i2c_follower(II_CY_ADDR);
        break;
      case mGridKria:
        init_i2c_follower(II_KR_ADDR);
        break;
      case mGridMP:
        init_i2c_follower(II_MP_ADDR);
        break;
      case mGridES:
        init_i2c_follower(ES);
        break;
      case mMidiStandard:
        init_i2c_follower(II_MID_ADDR);
        break;
      case mMidiArp:
        init_i2c_follower(II_ARP_ADDR);
        break;
      case mTT:
        init_i2c_follower(f.state.i2c_addr);
        break;
      default:
        break;
    }
  }
}

void clock_set(uint32_t n) {
  // printf("clock_period: %d\n", n);
  if (n<1) n = 60;
  timer_set(&clockTimer, 3*(n/3));
  timer_set(&midiTimer, n/3);  
}

void clock_set_tr(uint32_t n, uint8_t phase) {
  // printf("clock_period: %d\n", n);
  if (n<1) n = 60;
  timer_set(&clockTimer, 3*(n/3));
  timer_set(&midiTimer, n/3);  
  clock_phase = phase;
  timer_manual(&clockTimer);
  timer_manual(&midiTimer);
}

///////
// global ii handlers
void load_flash_state(void) {
  init_tuning();
  //init_levels();
  //init_cycles();
  init_kria();
  init_mp();
  init_es();
  // init_tt();

  print_dbg("\r\ni2c addr: ");
  Serial.printf("%04x\n",f.state.i2c_addr);

  leader_mode = false;
  memcpy((void*)followers, f.state.followers, sizeof(followers));
  for (uint8_t i = 0; i < I2C_FOLLOWER_COUNT; i++) {
    if (followers[i].active) {
      if (!leader_mode) {
        leader_mode = true;
        // wait to allow for any i2c devices to fully initalise
        delay(50);
        init_i2c_leader();
      }
      follower_on(i);
    }
  }
  if (!leader_mode) {
    init_i2c_follower(f.state.i2c_addr);
  }
}

void ii_ansible(uint8_t* d, uint8_t len) {
  // print_dbg("\r\nii/ansible (");
  // print_dbg_ulong(len);
  // print_dbg(") ");
  // for(int i=0;i<len;i++) {
  // 	print_dbg_ulong(d[i]);
  // 	print_dbg(" ");
  // }

  if (len < 1) {
    return;
  }

  switch (d[0]) {
    case II_ANSIBLE_APP:
      if (len >= 2) {
        ansible_mode_t next_mode = ii_ansible_mode_for_cmd(d[1]);
        if (next_mode < 0) {
          break;
        }
        set_mode(next_mode);
      }
      break;
    case II_ANSIBLE_APP + II_GET:
      {
        uint8_t cmd = ii_ansible_cmd_for_mode(ansible_mode);
        ii_tx_queue(cmd);
        break;
      }
    default:
      break;
  }
}

static ansible_mode_t ii_ansible_mode_for_cmd(uint8_t cmd) {
  switch (cmd) {
      // case 0:  return mArcLevels;
      // case 1:  return mArcCycles;
    case 2: return mGridKria;
    case 3: return mGridMP;
    case 4:
      return mGridES;
      // case 5:  return mMidiStandard;
      // case 6:  return mMidiArp;
    case 7: return mTT;
    default: return -1;
  }
}

static uint8_t ii_ansible_cmd_for_mode(ansible_mode_t mode) {
  switch (mode) {
      // case mArcLevels:    return 0;
      // case mArcCycles:    return 1;
    case mGridKria: return 2;
    case mGridMP: return 3;
    case mGridES:
      return 4;
      // case mMidiStandard: return 5;
      // case mMidiArp:      return 6;
    case mTT: return 7;
    default: return -1;
  }
}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// main

Bounce button1 = Bounce(37, 10);
Bounce button2 = Bounce(36, 10);
Bounce button3 = Bounce(35, 10);
Bounce button4 = Bounce(34, 10);
Bounce button5 = Bounce(33, 10);

IntervalTimer tcTicks;

static void midiTick(void* o) {
  MIDI.sendClock();
  // midiDevice.sendRealTime(midiDevice.Clock);
}

void setup(void) {

  Serial.begin(31250);
  Serial1.begin(31250);
  // Serial1.begin(115200);

  // sysclk_init();
  // init_dbg_rs232(FMCK_HZ);

  print_dbg("\r\n\n// ansible //////////////////////////////// ");
  print_dbg("\r\n== FLASH struct size: ");
  print_dbg_ulong(sizeof(f));

  Serial.println("Initializing SD card...");
  SPI.setMOSI(SDCARD_MOSI_PIN);
  SPI.setSCK(SDCARD_SCK_PIN);
  if (!(SD.begin(SDCARD_CS_PIN))) {
    // stop here if no SD card, but print a message
    while (1) {
      print_dbg("Unable to access the SD card");
      delay(500);
    }
  }

  Wire.begin();
  delay(1000);
  // do {
  //   Wire.beginTransmission(TELEXO);
  // } while (Wire.endTransmission()>0);

  //Wire.setTimeout(5, true);
  //Wire.setClock(100000);

  delay(10);

  // SD.remove("ANSIBLE.BIN");

  if (SD.exists("ANSIBLE.BIN")) {
    // load from flash at startup
    File preset_file = SD.open("ANSIBLE.BIN", FILE_READ);
    print_dbg("Reading preset file.");
    preset_file.read(&f, sizeof(f));
    preset_file.close();

    // memcpy((void*)f.state.followers, followers, sizeof(followers));

  }

  if (flash_is_fresh()) {
    // store flash defaults
    print_dbg("\r\nfirst run.");
    f.state.none_mode = mTT;
    f.state.grid_mode = mGridKria;
    f.state.arc_mode = mArcLevels;
    f.state.midi_mode = mMidiStandard;
    f.state.i2c_addr = 0xA0;
    f.state.grid_varibrightness = 16;

    // init_i2c_followers();
    memcpy((void*)f.state.followers, followers, sizeof(followers));

    // flashc_memset32((void*)&(f.state.none_mode), mTT, 4, true);
    // flashc_memset32((void*)&(f.state.grid_mode), mGridKria, 4, true);
    // flashc_memset32((void*)&(f.state.arc_mode), mArcLevels, 4, true);
    // flashc_memset32((void*)&(f.state.midi_mode), mMidiStandard, 4, true);
    // flashc_memset8((void*)&(f.state.i2c_addr), 0xA0, 1, true);
    // flashc_memset8((void*)&(f.state.grid_varibrightness), 16, 1, true);
    // flashc_memcpy((void*)f.state.followers, followers, sizeof(followers), true);
    default_tuning();
    default_kria();
    default_mp();
    default_es();
    // default_levels();
    // default_cycles();
    // default_midi_standard();
    // default_midi_arp();
    // default_tt();
    flash_unfresh();
  }

  // init_gpio();
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);

  pinMode(37, INPUT_PULLUP);
  pinMode(36, INPUT_PULLUP);
  pinMode(35, INPUT_PULLUP);
  pinMode(34, INPUT_PULLUP);
  pinMode(33, INPUT_PULLUP);

  assign_main_event_handlers();
  init_events();
  // init_tc();
  tcTicks.begin(process_timers, 1000);

  // init_spi();
  // init_adc();

  // irq_initialize_vectors();
  // register_interrupts();
  // cpu_irq_enable();

  load_flash_state();
  process_ii = &ii_ansible;

  clr_tr(TR1);
  clr_tr(TR2);
  clr_tr(TR3);
  clr_tr(TR4);

  clockQ = &clock_null;

  timer_add(&clockTimer, 1002, &clockTimer_callback, NULL);
	timer_add(&midiTimer,1002/3,&midiTick, NULL);
  timer_add(&keyTimer, 50, &keyTimer_callback, NULL);
  timer_add(&cvTimer,DAC_RATE_CV,&cvTimer_callback, NULL);

  init_dacs();

  connected = conNONE;
  set_mode(f.state.none_mode);

  // init_usb_host();
  myusb.begin();
  // init_monome();
  userial.begin(115200);
  delay(1000);

  myusb.Task();

  for (int i=0; i<30; i++) {
    if (userial && midiDevice) {
      // Serial.println("Grid available");
    } else {
      Serial.println("Grid or Midi not yet online");
      delay(100);
    }
  }
  if (userial) {
    const uint8_t *psz = userial.product();
    if (psz && *psz) Serial.printf("  product: % s\n", psz);
    const uint8_t *psm = userial.manufacturer();
    if (psm && *psm) Serial.printf("  manufacturer: % s\n", psm);
    const uint8_t *pss = userial.serialNumber();
    if (pss && *pss) Serial.printf("  serial number: % s\n", pss);
  }
  if (midiDevice) {
    const uint8_t *psz = midiDevice.product();
    if (psz && *psz) Serial.printf("  product: % s\n", psz);
    const uint8_t *psm = midiDevice.manufacturer();
    if (psm && *psm) Serial.printf("  manufacturer: % s\n", psm);
    const uint8_t *pss = midiDevice.serialNumber();
    if (pss && *pss) Serial.printf("  serial number: % s\n", pss);
  }

  memset(monomeLedBuffer, 12, sizeof(monomeLedBuffer));
  monome_mext_refresh();
  delay(500);
  memset(monomeLedBuffer, 0, sizeof(monomeLedBuffer));
  monome_mext_refresh();
  delay(20);

  connected = conGRID;
  set_mode(f.state.grid_mode);
  handler_MonomeConnect(1);

  MIDI.begin();

  // set_mode(mGridKria);
  // set_mode(mGridMP);
  // set_mode(mGridES);
  // init_kria();
  // init_mp();
  // init_es();

  // timers_set_monome();
  clock_external = 0;

  // while (true) {
  //   check_events();
  // }
}

void check_serial();

void loop() {
  myusb.Task();
  check_events();
  check_serial();

  if (button1.update()) {
    Serial.printf("Pin 37 is PUSHED.\n");
    static event_t e;
    e.type = kEventKey;
    e.data = (button1.fallingEdge() == true);
    event_post(&e);
  }

  if (button2.update()) {
    Serial.printf("Pin 36 is PUSHED.\n");
    static event_t e;
    e.type = kEventKey;
    e.data = (button2.fallingEdge() == true) + 2;
    event_post(&e);
  }

  if (button3.update()) {
    Serial.printf("Pin 35 is PUSHED.\n");
    static event_t e;
    e.type = kEventFrontShort;
    e.data = 1;
    if (button3.fallingEdge()) event_post(&e);
  }

  if (button4.update()) {
    Serial.printf("Pin 34 is PUSHED.\n");
    static event_t e;
    e.type = kEventFrontLong;
    e.data = 1;
    if (button4.fallingEdge()) event_post(&e);
  }

  if (button5.update()) {
    Serial.printf("Pin 33 is PUSHED.\n");
    if (button5.fallingEdge()) flash_write();
  }

  while (midiDevice.read()) {
    static event_t e;

    uint8_t type =       midiDevice.getType();
    uint8_t data1 =      midiDevice.getData1();
    uint8_t data2 =      midiDevice.getData2();
    uint8_t channel =    midiDevice.getChannel();
    const uint8_t *sys = midiDevice.getSysExArray();
    Serial.printf("type: %d, data: %0x, channel: %d\n", type, 
      (data1<<8|data2), channel);

    u16 data = (data1<<8 | data2);
    switch (data1) {
      case 0x16:
        e.type = kEventKey;
        e.data = (data2 == 0x00);
        event_post(&e);
        break;
      case 0x17:
        e.type = kEventKey;
        e.data = (data2 == 0x00) + 2;
        event_post(&e);
        break;
      case 0x18:
        if (data2 == 0x00) {
          e.type = kEventFrontShort;
          e.data = (data2 == 0x00);
          event_post(&e);
        }
        break;
      case 0x19:
        if (data2 == 0x00) {
          e.type = kEventFrontLong;
          e.data = (data2 == 0x00);
          event_post(&e);
        }
        break;
      case 0x1a:
        if (data2 == 0x00) {
          flash_write();
          memset(monomeLedBuffer, 12, sizeof(monomeLedBuffer));
          monomeFrameDirty++;
        }
      default: 
        break;
    }
  }
}

void check_serial() {
  u8 i1 = 0;
  u8 buffer[64];
  static event_t e;

  memset(buffer, 0, sizeof(buffer));
  String s = "";
  while (i1<sizeof(buffer) && Serial.available()) {
    buffer[i1++] = Serial.read();
  }
  if (i1) {
    // Serial.printf("buffer size: 0x%.4x\n", i1);
    for(u8 i2=0; i2<i1; i2++) {
      // Serial.printf("buffer[%d]: 0x%.4x\n", i2, buffer[i2]);
      if (i2>0 && isDigit(buffer[i2])) s += (char)buffer[i2];
    }
    u16 a = s.toInt();
    switch (buffer[0]) {
      case 0x61: // a
        e.type = kEventKey;
        e.data = a;
        event_post(&e);
        break;
      case 0x62: // b
        e.type = kEventKey;
        e.data = a + 2;
        event_post(&e);
        break;
      case 0x63: // c
        e.type = kEventFrontShort;
        e.data = 1;
        event_post(&e);
        break;
      case 0x64: // d
        e.type = kEventFrontLong;
        e.data = 1;
        event_post(&e);
        break;
      case 0x73: // s
        Serial.println("Saving Preset");
        Serial.printf("clock_period: %d\n", f.kria_state.clock_period);
        flash_write();
        break;
      case 0x7a: // z
        Serial.println("ZAP");
        Serial.printf("clock_period: %d\n", f.kria_state.clock_period);
        SD.remove("ANSIBLE.BIN");
        break;
    }
  }
}

