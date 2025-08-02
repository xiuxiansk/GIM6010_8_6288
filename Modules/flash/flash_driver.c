#include "flash_driver.h"
#include "bsp_flash.h"

#define FLASH_STORAGE_PARAM_ADDR ADDR_FLASH_PAGE_62 // 电机结构参数存在Flash第62页
#define FLASH_STORAGE_ID_ADDR    ADDR_FLASH_PAGE_63 // 电机ID存在Flash第63页
#define FLASH_STORAGE_AVAIL_FLAG 0xA5A5A5A5         // Flash数据有效标志

/**
***********************************************************************
* @brief:      flash_erase_page(uint8_t page)
* @param:		   page：flash页面
* @retval:     i 0：失败 1：成功
* @details:    按页面擦除，擦单页
***********************************************************************
**/
uint8_t flash_erase_page(uint32_t page)
{
    uint8_t i;
    i = flash_bsp_erase_page(page);
    return i;
}
/**
***********************************************************************
* @brief:      flash_erase_pages(uint32_t start_addr, uint32_t end_addr)
* @param:		   start_addr：flash首地址
* @param:		   end_addr：flash尾地址
* @retval:     i 0：失败 1：成功
* @details:    按页面地址擦除，擦多页
***********************************************************************
**/
uint8_t flash_erase_pages(uint32_t start_add, uint32_t end_add)
{
    uint8_t i;
    i = flash_bsp_erase_pages(start_add, end_add);
    return i;
}
/**
***********************************************************************
* @brief:      flash_write_data(uint32_t addr, void *data, uint32_t size)
* @param:		   addr：flash 地址
* @param:		   data：写入的数据
* @param:		   size：写入的数据的字节数
* @retval:     void
* @details:    向flash中存入数据
***********************************************************************
**/
void flash_write_data(uint32_t addr, void *data, uint32_t size)
{
    flash_bsp_write_data(addr, data, size);
}
/**
***********************************************************************
* @brief:      flash_read_data(uint32_t addr, void *data, uint32_t size)
* @param:		   addr：flash 地址
* @param:		   data：读取的数据
* @param:		   size：读取的数据字节数
* @retval:     void
* @details:    从flash中读取数据
***********************************************************************
**/
void flash_read_data(uint32_t addr, uint32_t *data, uint32_t size)
{
    memcpy(data, (uint32_t *)addr, size);
}

// 将电机ID存入Flash
void Flash_SaveMotorID(uint32_t id)
{
    flash_erase_page(63);
    uint32_t buf[2];
    buf[0] = FLASH_STORAGE_AVAIL_FLAG;
    buf[1] = id;
    flash_write_data(FLASH_STORAGE_ID_ADDR, buf, 8);
}

// 将电机结构参数存入Flash
void Flash_SaveMotorParam(uint32_t poles, int32_t encoder_offset, int32_t dir)
{
    flash_erase_page(62);
    uint32_t buf[4];
    buf[0] = FLASH_STORAGE_AVAIL_FLAG;
    buf[1] = *((uint32_t *)&poles);
    buf[2] = *((uint32_t *)&encoder_offset);
    buf[3] = *((uint32_t *)&dir);
    flash_write_data(FLASH_STORAGE_PARAM_ADDR, buf, sizeof(buf));
}

// 从Flash中读取电机ID
uint32_t Flash_ReadMotorID(void)
{
    uint32_t *buf = (uint32_t *)FLASH_STORAGE_ID_ADDR;
    if (buf[0] != FLASH_STORAGE_AVAIL_FLAG)
        return 0;
    return buf[1];
}

// 从Flash中读取电机结构参数
HAL_StatusTypeDef Flash_ReadMotorParam(uint32_t *poles, int32_t *encoder_offset, int32_t *dir)
{

    uint32_t *buf = (uint32_t *)FLASH_STORAGE_PARAM_ADDR;
    if (buf[0] != FLASH_STORAGE_AVAIL_FLAG)
        return HAL_ERROR;
    *poles          = *((uint32_t *)&buf[1]);
    *encoder_offset = *((int32_t *)&buf[2]);
    *dir            = *((int32_t *)&buf[3]);

    return HAL_OK;
}