#ifndef CAN_IDS_H
#define CAN_IDS_H

// ================================================================
// FRONT MODULE SENSORS
// ================================================================
#define CAN_ID_FRONT_ANALOG   0x500 // Data: [D1_H, D1_L, D2_H, D2_L, STR_H, STR_L]
#define CAN_ID_ACCEL          0x501 // Data: [AX_H, AX_L, AY_H, AY_L, AZ_H, AZ_L]
#define CAN_ID_GYRO           0x502 // Data: [GX_H, GX_L, GY_H, GY_L, GZ_H, GZ_L]

// ================================================================
// ENGINE (MEGASQUIRT)
// ================================================================
#define CAN_ID_RPM            0x600 
#define CAN_ID_VOLTAGE        0x601
#define CAN_ID_WATER_TEMP     0x602

// ================================================================
// REAR MODULE SENSORS
// ================================================================
#define CAN_ID_GEAR           0x700 // Data: [GearNum]
#define CAN_ID_REAR_ANALOG    0x701 // Data: [RL_H, RL_L, RR_H, RR_L, BRK_H, BRK_L]

// GPS (Split into 2 frames because floats are 4 bytes each)
#define CAN_ID_GPS_POS        0x800 // Data: [Lat_B0..B3, Lon_B0..B3]
#define CAN_ID_GPS_SPD        0x801 // Data: [Spd_B0..B3, 0, 0, 0, 0]

#define CAN_ID_LAPTIME        0x900 // laptime

#endif