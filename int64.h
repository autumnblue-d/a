#ifndef __libfixmath_int64_h__
#define __libfixmath_int64_h__

#ifndef FIXMATH_NO_64BIT
static inline int64_t int64_const(int32_t hi, uint32_t lo) {
  return (((int64_t)hi << 32) | lo);
}
static inline int64_t int64_from_int32(int32_t x) {
  return (int64_t)x;
}
static inline int32_t int64_hi(int64_t x) {
  return (x >> 32);
}
static inline uint32_t int64_lo(int64_t x) {
  return (x & ((1ULL << 32) - 1));
}

static inline int64_t int64_add(int64_t x, int64_t y) {
  return (x + y);
}
static inline int64_t int64_neg(int64_t x) {
  return (-x);
}
static inline int64_t int64_sub(int64_t x, int64_t y) {
  return (x - y);
}
static inline int64_t int64_shift(int64_t x, int8_t y) {
  return (y < 0 ? (x >> -y) : (x << y));
}

static inline int64_t int64_mul_i32_i32(int32_t x, int32_t y) {
  return (x * y);
}
static inline int64_t int64_mul_i64_i32(int64_t x, int32_t y) {
  return (x * y);
}

static inline int64_t int64_div_i64_i32(int64_t x, int32_t y) {
  return (x / y);
}

static inline int int64_cmp_eq(int64_t x, int64_t y) {
  return (x == y);
}
static inline int int64_cmp_ne(int64_t x, int64_t y) {
  return (x != y);
}
static inline int int64_cmp_gt(int64_t x, int64_t y) {
  return (x > y);
}
static inline int int64_cmp_ge(int64_t x, int64_t y) {
  return (x >= y);
}
static inline int int64_cmp_lt(int64_t x, int64_t y) {
  return (x < y);
}
static inline int int64_cmp_le(int64_t x, int64_t y) {
  return (x <= y);
}
#else

typedef struct {
  int32_t hi;
  uint32_t lo;
} __int64_t;

static inline __int64_t int64_const(int32_t hi, uint32_t lo) {
  return (__int64_t){ hi, lo };
}
static inline __int64_t int64_from_int32(int32_t x) {
  return (__int64_t){ (x < 0 ? -1 : 0), x };
}
static inline int32_t int64_hi(__int64_t x) {
  return x.hi;
}
static inline uint32_t int64_lo(__int64_t x) {
  return x.lo;
}

static inline int int64_cmp_eq(__int64_t x, __int64_t y) {
  return ((x.hi == y.hi) && (x.lo == y.lo));
}
static inline int int64_cmp_ne(__int64_t x, __int64_t y) {
  return ((x.hi != y.hi) || (x.lo != y.lo));
}
static inline int int64_cmp_gt(__int64_t x, __int64_t y) {
  return ((x.hi > y.hi) || ((x.hi == y.hi) && (x.lo > y.lo)));
}
static inline int int64_cmp_ge(__int64_t x, __int64_t y) {
  return ((x.hi > y.hi) || ((x.hi == y.hi) && (x.lo >= y.lo)));
}
static inline int int64_cmp_lt(__int64_t x, __int64_t y) {
  return ((x.hi < y.hi) || ((x.hi == y.hi) && (x.lo < y.lo)));
}
static inline int int64_cmp_le(__int64_t x, __int64_t y) {
  return ((x.hi < y.hi) || ((x.hi == y.hi) && (x.lo <= y.lo)));
}

static inline __int64_t int64_add(__int64_t x, __int64_t y) {
  __int64_t ret;
  ret.hi = x.hi + y.hi;
  ret.lo = x.lo + y.lo;
  if ((ret.lo < x.lo) || (ret.hi < y.hi))
    ret.hi++;
  return ret;
}

static inline __int64_t int64_neg(__int64_t x) {
  __int64_t ret;
  ret.hi = ~x.hi;
  ret.lo = ~x.lo + 1;
  if (ret.lo == 0)
    ret.hi++;
  return ret;
}

static inline __int64_t int64_sub(__int64_t x, __int64_t y) {
  return int64_add(x, int64_neg(y));
}

static inline __int64_t int64_shift(__int64_t x, int8_t y) {
  __int64_t ret;
  if (y > 0) {
    if (y >= 32)
      return (__int64_t){ 0, 0 };
    ret.hi = (x.hi << y) | (x.lo >> (32 - y));
    ret.lo = (x.lo << y);
  } else {
    y = -y;
    if (y >= 32)
      return (__int64_t){ 0, 0 };
    ret.lo = (x.lo >> y) | (x.hi << (32 - y));
    ret.hi = (x.hi >> y);
  }
  return ret;
}

static inline __int64_t int64_mul_i32_i32(int32_t x, int32_t y) {
  int16_t hi[2] = { (x >> 16), (y >> 16) };
  uint16_t lo[2] = { (x & 0xFFFF), (y & 0xFFFF) };

  int32_t r_hi = hi[0] * hi[1];
  int32_t r_md = (hi[0] * lo[1]) + (hi[1] * lo[0]);
  uint32_t r_lo = lo[0] * lo[1];

  r_hi += (r_md >> 16);
  r_lo += (r_md << 16);

  return (__int64_t){ r_hi, r_lo };
}

static inline __int64_t int64_mul_i64_i32(__int64_t x, int32_t y) {
  int neg = ((x.hi ^ y) < 0);
  if (x.hi < 0)
    x = int64_neg(x);
  if (y < 0)
    y = -y;

  uint32_t _x[4] = { (x.hi >> 16), (x.hi & 0xFFFF), (x.lo >> 16), (x.lo & 0xFFFF) };
  uint32_t _y[2] = { (y >> 16), (y & 0xFFFF) };

  uint32_t r[4];
  r[0] = (_x[0] * _y[0]);
  r[1] = (_x[1] * _y[0]) + (_x[0] * _y[1]);
  r[2] = (_x[1] * _y[1]) + (_x[2] * _y[0]);
  r[3] = (_x[2] * _y[0]) + (_x[1] * _y[1]);

  __int64_t ret;
  ret.lo = r[0] + (r[1] << 16);
  ret.hi = (r[3] << 16) + r[2] + (r[1] >> 16);
  return (neg ? int64_neg(ret) : ret);
}

static inline __int64_t int64_div_i64_i32(__int64_t x, int32_t y) {
  int neg = ((x.hi ^ y) < 0);
  if (x.hi < 0)
    x = int64_neg(x);
  if (y < 0)
    y = -y;

  __int64_t ret = { (x.hi / y), (x.lo / y) };
  x.hi = x.hi % y;
  x.lo = x.lo % y;

  __int64_t _y = int64_from_int32(y);

  __int64_t i;
  for (i = int64_from_int32(1); int64_cmp_lt(_y, x); _y = int64_shift(_y, 1), i = int64_shift(i, 1))
    ;

  while (x.hi) {
    _y = int64_shift(_y, -1);
    i = int64_shift(i, -1);
    if (int64_cmp_ge(x, _y)) {
      x = int64_sub(x, _y);
      ret = int64_add(ret, i);
    }
  }

  ret = int64_add(ret, int64_from_int32(x.lo / y));
  return (neg ? int64_neg(ret) : ret);
}

#define int64_t __int64_t

#endif

#endif
