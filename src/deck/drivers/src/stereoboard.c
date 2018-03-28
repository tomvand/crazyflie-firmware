/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2016 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * crazycar.c - Deck driver for the Crazyflie 2.0 Crazycar deck
 */
#define DEBUG_MODULE "StereoboardDeck"

#include <stdint.h>
#include <stdlib.h>
#include "stm32fxxx.h"

#include "deck.h"

#include "FreeRTOS.h"
#include "timers.h"
#include "debug.h"
#include "log.h"
#include "uart1.h"
#include "system.h"
#include "pprz_datalink.h"
#include "pprzlink/intermcu_msg.h"

 static char ch = 'a';

void stereoboardTask(void* arg)
{
  systemWaitStart();

  while(1){
    uart1Getchar(&ch);
  }
}


/* Initialize the deck driver */
static void stereoboardDeckInit(DeckInfo *info)
{
  uart1Init(115200);
  DEBUG_PRINT("Test StereoboardDeck!\n");

  xTaskCreate(stereoboardTask, UART_RX_TASK_NAME, UART_RX_TASK_STACKSIZE, NULL, 2, NULL);


}



static const DeckDriver stereoboard_deck = {
    .vid = 0xBC,
    .pid = 0x00,
  .name = "bcStereoboard",
  .usedPeriph = DECK_USING_UART1,
  .usedGpio = DECK_USING_TX1 | DECK_USING_RX1,

  .init = stereoboardDeckInit
};

DECK_DRIVER(stereoboard_deck);

LOG_GROUP_START(stereoboard)
LOG_ADD(LOG_UINT8, character, &ch)
LOG_GROUP_STOP(stereoboard)
