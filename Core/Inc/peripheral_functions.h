/**
  ******************************************************************************
  * @file           : peripheral_functions.h
  * @brief          : Header for peripheral_functions.c file.
  *                   This file contains the prototypes for the functions
  *                   that involve working with peripherals
  ******************************************************************************
*/

#ifndef __PERIPHERAL_FUNCTIONS_H
#define __PERIPHERAL_FUNCTIONS_H

/* Include files which are necessary for the project */
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "main.h"

/* Function Declarations */
void Clock_Init(void);
void GPIO_Init(void);
void TIMER3_Init(void);
void USART2_Init(void);
void I2C1_Init(void);
void I2C_Read(uint8_t Slave_Address, uint8_t Register_Address, uint8_t *pdata);
void I2C_Write(uint8_t Slave_Address, uint8_t Register_Address, uint8_t *pdata);
void SendToUART(uint8_t *pbuf);

#endif
