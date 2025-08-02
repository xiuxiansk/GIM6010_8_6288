#pragma once
#include "main.h"
#include <stdint.h>

typedef enum {
    CMD_ID_GET_POSITION = 0x01,
    CMD_ID_GET_VELOCITY = 0x02,
    CMD_ID_GET_TORQUE   = 0x03,
    CMD_ID_CLEAR_ERRORS = 0x04,
} Cmd_Id_e;

#pragma pack(1)
// typedef struct
// {
//     int cmd_id;
//     float cmd_rx_data;
// } Cmd_Rx_s;
typedef struct
{
    int16_t torque[4];
} Cmd_Rx_s;
typedef struct
{
    int16_t position;
    int16_t velocity;
    int16_t torque;
    int16_t unused;
} Cmd_Tx_s;
#pragma pack()