// ADDRESSES
// each device must use a unique address (0x01-0x7F)
// 0 is RESERVED for global commands

#define CROW 0x01  // alias of CROW_ADDR_0 for legacy usage
#define CROW_ADDR_0 0x01
#define CROW_ADDR_1 0x02
#define CROW_ADDR_2 0x03
#define CROW_ADDR_3 0x04

#define WW 0x10
#define WW_KRIA 0x10  // ww kria uses ww address

#define II_ANSIBLE_ADDR 0x20
#define II_ANSIBLE_ADDR_2 0x21
#define II_ANSIBLE_ADDR_3 0x22
#define II_ANSIBLE_ADDR_4 0x23
#define II_MID_ADDR 0x24
#define II_ARP_ADDR 0x26
#define II_KR_ADDR 0x28
#define II_MP_ADDR 0x2A
#define II_LV_ADDR 0x2C
#define II_CY_ADDR 0x2E

#define MP 0x30
#define ER301_1 0x31
#define ER301_2 0x32
#define ER301_3 0x33
#define FADER 0x34
#define FADER_2 0x35
#define FADER_3 0x36
#define FADER_4 0x37
#define MATRIXARCHATE 0x38
#define MATRIXARCHATE_2 0x39
#define MATRIXARCHATE_3 0x3A
#define TETRAPAD 0x3B
#define TETRAPAD_2 0x3C
#define TETRAPAD_3 0x3D
#define TETRAPAD_4 0x3E

#define ORCA 0x40

#define DISTING_EX_1 0x41
#define DISTING_EX_2 0x42
#define DISTING_EX_3 0x43
#define DISTING_EX_4 0x44

#define ES 0x50

#define TELEXO 0x60
#define TELEXO_0 0x60
#define TELEXO_1 0x61
#define TELEXO_2 0x62
#define TELEXO_3 0x63
#define TELEXO_4 0x64
#define TELEXO_5 0x65
#define TELEXO_6 0x66
#define TELEXO_7 0x67

#define TELEXI 0x68
#define TELEXI_0 0x68
#define TELEXI_1 0x69
#define TELEXI_2 0x6A
#define TELEXI_3 0x6B
#define TELEXI_4 0x6C
#define TELEXI_5 0x6D
#define TELEXI_6 0x6E
#define TELEXI_7 0x6F

#define JF_ADDR 0x70
#define WS_T_ADDR 0x71
#define WS_T_ADDR_2 0x72
// 0x73 RESERVED
// 0x74 RESERVED
#define JF_ADDR_2 0x75
#define WS_S_ADDR 0x76
#define WS_S_ADDR_2 0x77
#define WS_D_ADDR 0x78
#define WS_D_ADDR_2 0x79

// COMMANDS
// new commands can overlap with existing ones
// 0b11xxxxxx (0xC0-0xFF) range is RESERVED for global commands

#define WW_PRESET 0x11
#define WW_POS 0x12
#define WW_SYNC 0x13
#define WW_START 0x14
#define WW_END 0x15
#define WW_PMODE 0x16
#define WW_PATTERN 0x17
#define WW_QPATTERN 0x18
#define WW_MUTE1 0x19
#define WW_MUTE2 0x1A
#define WW_MUTE3 0x1B
#define WW_MUTE4 0x1C
#define WW_MUTEA 0x1D
#define WW_MUTEB 0x1E

#define MP_PRESET 0x31
#define MP_RESET 0x32
#define MP_SYNC 0x33
#define MP_MUTE 0x34
#define MP_UNMUTE 0x35
#define MP_FREEZE 0x36
#define MP_UNFREEZE 0x37
#define MP_STOP 0x38

#define ORCA_TRACK 0x40
#define ORCA_CLOCK 0x41
#define ORCA_DIVISOR 0x42
#define ORCA_PHASE 0x43
#define ORCA_RESET 0x44
#define ORCA_WEIGHT 0x45
#define ORCA_MUTE 0x46
#define ORCA_SCALE 0x47
#define ORCA_BANK 0x48
#define ORCA_PRESET 0x49
#define ORCA_RELOAD 0x4A
#define ORCA_ROTATES 0x4B
#define ORCA_ROTATEW 0x4C
#define ORCA_GRESET 0x4D
#define ORCA_CVA 0x4E
#define ORCA_CVB 0x4F

#define ES_PRESET 0x51
#define ES_MODE 0x52
#define ES_CLOCK 0x53
#define ES_RESET 0x54
#define ES_PATTERN 0x55
#define ES_TRANS 0x56
#define ES_STOP 0x57
#define ES_TRIPLE 0x58
#define ES_MAGIC 0x59
#define ES_CV 0x5A

#define II_GET 128
#define II_ANSIBLE_TR 1
#define II_ANSIBLE_TR_TOG 2
#define II_ANSIBLE_TR_PULSE 3
#define II_ANSIBLE_TR_TIME 4
#define II_ANSIBLE_TR_POL 5
#define II_ANSIBLE_CV 6
#define II_ANSIBLE_CV_SLEW 7
#define II_ANSIBLE_CV_OFF 8
#define II_ANSIBLE_CV_SET 9
#define II_ANSIBLE_INPUT 10
#define II_ANSIBLE_APP 15

#define II_MID_SLEW 1
#define II_MID_SHIFT 2

#define II_ARP_STYLE 0
#define II_ARP_HOLD 1
#define II_ARP_RPT 2
#define II_ARP_GATE 3
#define II_ARP_DIV 4
#define II_ARP_ROT 5
#define II_ARP_SLEW 6
#define II_ARP_PULSE 7
#define II_ARP_RESET 8
#define II_ARP_SHIFT 9
#define II_ARP_FILL 10
#define II_ARP_ER 11

#define II_GRID_KEY 16
#define II_GRID_LED 17
#define II_ARC_ENC 24
#define II_ARC_LED 25

#define II_KR_PRESET 0
#define II_KR_PATTERN 1
#define II_KR_SCALE 2
#define II_KR_PERIOD 3
#define II_KR_POS 4
#define II_KR_LOOP_ST 5
#define II_KR_LOOP_LEN 6
#define II_KR_RESET 7
#define II_KR_CV 8
#define II_KR_MUTE 9
#define II_KR_TMUTE 10
#define II_KR_CLK 11
#define II_KR_PAGE 12
#define II_KR_CUE 13
#define II_KR_DIR 14
#define II_KR_DURATION 15

#define II_MP_PRESET 0
#define II_MP_RESET 1
#define II_MP_STOP 2
#define II_MP_SCALE 3
#define II_MP_PERIOD 4
#define II_MP_CV 5

#define II_LV_PRESET 0
#define II_LV_RESET 1
#define II_LV_POS 2
#define II_LV_L_ST 3
#define II_LV_L_LEN 4
#define II_LV_L_DIR 5
#define II_LV_CV 6

#define II_CY_PRESET 0
#define II_CY_RESET 1
#define II_CY_POS 2
#define II_CY_REV 3
#define II_CY_CV 4


#define JF_TR 1
#define JF_RMODE 2
#define JF_RUN 3
#define JF_SHIFT 4
#define JF_VTR 5
#define JF_MODE 6
#define JF_TICK 7
#define JF_VOX 8
#define JF_NOTE 9
#define JF_GOD 10
#define JF_TUNE 11
#define JF_QT 12
#define JF_PITCH 13
#define JF_ADDRESS 14
#define JF_SPEED 15
#define JF_TSC 16
#define JF_RAMP 17
#define JF_CURVE 18
#define JF_FM 19
#define JF_TIME 20
#define JF_INTONE 21

#define WS_REC 1
#define WS_PLAY 2
#define WS_LOOP 3
#define WS_CUE 4

#define WS_S_VEL 2
#define WS_S_PITCH 3
#define WS_S_VOX 4
#define WS_S_NOTE 5
#define WS_S_AR_MODE 7
#define WS_S_CURVE 8
#define WS_S_RAMP 9
#define WS_S_FM_INDEX 10
#define WS_S_FM_RATIO 11
#define WS_S_LPG_TIME 12
#define WS_S_LPG_SYMMETRY 13
#define WS_S_PATCH 14
#define WS_S_VOICES 15
#define WS_S_FM_ENV 16

#define WS_D_FEEDBACK 2
#define WS_D_MIX 3
#define WS_D_LOWPASS 4
#define WS_D_FREEZE 5
#define WS_D_TIME 6
#define WS_D_LENGTH 7
#define WS_D_POSITION 8
#define WS_D_CUT 9
#define WS_D_FREQ_RANGE 10
#define WS_D_RATE 11
#define WS_D_FREQ 12
#define WS_D_CLK 13
#define WS_D_CLK_RATIO 14
#define WS_D_PLUCK 15
#define WS_D_MOD_RATE 16
#define WS_D_MOD_AMOUNT 17

#define WS_T_RECORD 1
#define WS_T_PLAY 2
#define WS_T_REV 3
#define WS_T_SPEED 4
#define WS_T_FREQ 5
#define WS_T_PRE_LEVEL 6
#define WS_T_MONITOR_LEVEL 7
#define WS_T_REC_LEVEL 8
#define WS_T_HEAD_ORDER 9
#define WS_T_LOOP_START 10
#define WS_T_LOOP_END 11
#define WS_T_LOOP_ACTIVE 12
#define WS_T_LOOP_SCALE 13
#define WS_T_LOOP_NEXT 14
#define WS_T_TIMESTAMP 15
#define WS_T_SEEK 16
#define WS_T_CLEARTAPE 18

#define CROW_VOLTS 1
#define CROW_SLEW 2
#define CROW_CALL1 4
#define CROW_CALL2 5
#define CROW_CALL3 6
#define CROW_CALL4 7
#define CROW_RESET 8
#define CROW_PULSE 9
#define CROW_AR 10
#define CROW_LFO 11
#define CROW_IN (II_GET + 3)
#define CROW_OUT (II_GET + 4)
#define CROW_QUERY0 (II_GET + 5)
#define CROW_QUERY1 (II_GET + 6)
#define CROW_QUERY2 (II_GET + 7)
#define CROW_QUERY3 (II_GET + 8)
