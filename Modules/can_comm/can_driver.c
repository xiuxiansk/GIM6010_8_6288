#include "can_driver.h" // 包含 CAN 驱动的头文件
#include "string.h"     // 包含字符串处理函数的头文件
#include "stdlib.h"
#include "crc8.h"
#include "motor_def.h"
static Cmd_Rx_s can_rx;
/**
 * @brief 重置CAN comm的接收状态和buffer
 *
 * @param ins 需要重置的实例
 */
static void FDCANCommResetRx(FDCANCommInstance *ins)
{
    // 当前已经收到的buffer清零
    memset(ins->raw_recvbuf, 0, ins->cur_recv_len);
    ins->recv_state   = 0; // 接收状态重置
    ins->cur_recv_len = 0; // 当前已经收到的长度重置
}
Cmd_Rx_s *FDCANGetRxBuff(void)
{
    return &can_rx;
}
/**
 * @brief fdcancomm的接收回调函数
 *
 * @param _instance
 */
static void FDCANCommRxCallback(FDCANInstance *_instance)
{
    memcpy((uint8_t *)(&can_rx), _instance->rx_buff, _instance->rx_len);
}

static void FDCANCommLostCallback(void *cancomm)
{
    FDCANCommInstance *comm = (FDCANCommInstance *)cancomm;
    FDCANCommResetRx(comm);
}

FDCANCommInstance *FDCANCommInit(FDCANComm_Init_Config_s *comm_config)
{
    FDCANCommInstance *ins = (FDCANCommInstance *)malloc(sizeof(FDCANCommInstance));
    memset(ins, 0, sizeof(FDCANCommInstance));

    ins->recv_data_len                                                         = comm_config->recv_data_len;
    ins->recv_buf_len                                                          = comm_config->recv_data_len + FDCAN_COMM_OFFSET_BYTES; // head + datalen + crc8 + tail
    ins->send_data_len                                                         = comm_config->send_data_len;
    ins->send_buf_len                                                          = comm_config->send_data_len + FDCAN_COMM_OFFSET_BYTES;
    ins->raw_sendbuf[0]                                                        = FDCAN_COMM_HEADER;          // head,直接设置避免每次发送都要重新赋值,下面的tail同理
    ins->raw_sendbuf[1]                                                        = comm_config->send_data_len; // datalen
    ins->raw_sendbuf[comm_config->send_data_len + FDCAN_COMM_OFFSET_BYTES - 1] = FDCAN_COMM_TAIL;
    // can instance的设置
    comm_config->can_config.id                    = ins;                                     // FDCANComm的实例指针作为FDCANInstance的id,回调函数中会用到
    comm_config->can_config.fdcan_module_callback = FDCANCommRxCallback;                     // 接收回调函数
    ins->can_ins                                  = FDCANRegister(&comm_config->can_config); // 注册CAN实例

    Daemon_Init_Config_s daemon_config = {
        .callback     = FDCANCommLostCallback,
        .owner_id     = (void *)ins,
        .reload_count = comm_config->daemon_count,
    };
    ins->comm_daemon = DaemonRegister(&daemon_config);
    return ins;
}

void FDCANCommSend(FDCANCommInstance *instance, uint8_t *data)
{
    memcpy(instance->can_ins->tx_buff, data, 8);
    FDCANTransmit(instance->can_ins, 2);
}

void *FDCANCommGet(FDCANCommInstance *instance)
{
    instance->update_flag = 0; // 读取后将更新flag置为0
    return instance->unpacked_recv_data;
}

uint8_t FDCANCommIsOnline(FDCANCommInstance *instance)
{
    return DaemonIsOnline(instance->comm_daemon);
}
