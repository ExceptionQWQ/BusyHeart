/**
 * @file ads1292.h
 * @author BusyBox (busybox177634@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-12-26
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

int ads1292_init();
uint8_t ads1292_read_register(uint8_t address);
int ads1292_write_register(uint8_t address, uint8_t data);
int ads1292_send_cmd(uint8_t cmd);
uint32_t ads1292_read_channel1();
uint32_t ads1292_read_channel2();
