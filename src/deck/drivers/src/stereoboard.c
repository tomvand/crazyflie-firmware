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


#include "stereoboard.h"


 static char ch = 'a';


struct stereocam_t stereocam = {
   .device = 0,
   .msg_available = false
 };
 static uint8_t stereocam_msg_buf[256]  __attribute__((aligned));   ///< The message buffer for the stereocamera
 uint8_t msg_buf[4*64];           // define local data

 struct pprz_transport pprz;
 struct link_device dev;
 struct uint8array {
   uint8_t len;
   uint8_t height;
   uint8_t *data;
   bool data_new;
 };
void stereoboardTask(void* arg)
{
  struct uint8array stereocam_data = {.len = 0, .data = msg_buf, .data_new = 0, .height = 0}; // buffer used to contain image without line endings

  pprz_check_and_parse(&dev, &pprz, stereocam_data.data, &stereocam_data.data_new);

}


/* Initialize the deck driver */
void stereoboardDeckInit(DeckInfo *info)
{
  uart1Init(115200);
  struct UartDataStruct USART1_Data ;

  datalink_init(&USART1_Data);

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
