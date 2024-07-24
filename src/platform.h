/**
 ******************************************************************************
 * @file    platform.h
 * @author  STMicroelectronics
 * @version V1.0.0
 * @date    17 July 2024
 * @brief   Header file of the platform dependent structures.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2024 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

#ifndef _PLATFORM_H_
#define _PLATFORM_H_

#ifdef __cplusplus
extern "C" {
#endif


#pragma once

#include <stdint.h>
#include <string.h>
#include "platform_config.h"

#define VL53L7CH_COMMS_CHUNK_SIZE 4096
#define SPI_WRITE_MASK(x) (uint16_t)(x | 0x8000)
#define SPI_READ_MASK(x)  (uint16_t)(x & ~0x8000)
#ifndef DEFAULT_I2C_BUFFER_LEN




typedef uint8_t (*VL53L7CH_wait_Func)(void *, uint32_t);
typedef uint8_t (*VL53L7CH_write_Func)(void *, uint16_t, uint8_t *, uint32_t);
typedef uint8_t (*VL53L7CH_read_Func)(void *, uint16_t, uint8_t *, uint32_t);


#ifdef ARDUINO_SAM_DUE
/* FIXME: It seems that an I2C buffer of BUFFER_LENGTH does not work on Arduino DUE. So, we need to decrease the size */
#define DEFAULT_I2C_BUFFER_LEN (BUFFER_LENGTH - 2)
#else
#ifdef BUFFER_LENGTH
#define DEFAULT_I2C_BUFFER_LEN BUFFER_LENGTH
#else
#define DEFAULT_I2C_BUFFER_LEN 32
#endif
#endif
#endif
typedef struct {
  uint16_t  address;
  VL53L7CH_write_Func Write;
  VL53L7CH_read_Func Read;
  VL53L7CH_wait_Func Wait;
  void *handle;
} VL53L7CH_Platform;

typedef VL53L7CH_Platform VL53LMZ_Platform;

uint8_t RdByte(
  VL53L7CH_Platform *p_platform,
  uint16_t RegisterAddress,
  uint8_t *p_value);

uint8_t WrByte(
  VL53L7CH_Platform *p_platform,
  uint16_t RegisterAddress,
  uint8_t value);

uint8_t WrMulti(
  VL53L7CH_Platform *p_platform,
  uint16_t RegisterAddress,
  uint8_t *p_values,
  uint32_t size);

uint8_t RdMulti(
  VL53L7CH_Platform *p_platform,
  uint16_t RegisterAddress,
  uint8_t *p_values,
  uint32_t size);

uint8_t WaitMs(
  VL53L7CH_Platform *p_platform,
  uint32_t TimeMs);
void SwapBuffer(
  uint8_t     *buffer,
  uint16_t     size);

#ifdef __cplusplus
}
#endif

#endif  // _PLATFORM_H_
