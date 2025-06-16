#pragma once

#include "types.h"

#define I2C_FOLLOWER_COUNT 7

struct i2c_follower_t;

typedef void (*ii_u8_cb)(struct i2c_follower_t* follower, uint8_t track, uint8_t param);
typedef void (*ii_s8_cb)(struct i2c_follower_t* follower, uint8_t track, int8_t param);
typedef void (*ii_u16_cb)(struct i2c_follower_t* follower, uint8_t track, uint16_t param);


typedef struct i2c_ops_t {
  ii_u8_cb init;
  ii_u8_cb mode;
  ii_u8_cb tr;
  ii_u8_cb mute;

  ii_u16_cb cv;
  ii_s8_cb octave;
  ii_u16_cb slew;

  uint8_t mode_ct;
} i2c_ops_t;

typedef struct i2c_follower_t {
  uint8_t addr;
  bool active;
  uint8_t track_en;
  int8_t oct;
  uint8_t active_mode;
  i2c_ops_t opsx;
  //i2c_ops_t* ops;
} i2c_follower_t;

extern struct i2c_ops_t ops[I2C_FOLLOWER_COUNT];
extern struct i2c_follower_t followers[I2C_FOLLOWER_COUNT];

void follower_change_mode(struct i2c_follower_t* follower, uint8_t param);
void follower_change_octave(struct i2c_follower_t* follower, int8_t param);
