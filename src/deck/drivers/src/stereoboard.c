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


//static char ch = 'a';
static float velx, vely, velz;


/*struct stereocam_t stereocam = {
   .device = 0,
   .msg_available = false
 };*/
uint8_t msg_buf[4*64];           // define local data

/* struct pprz_transport pprz;
 struct link_device dev;*/
struct uint8array {
  uint8_t len;
  uint8_t height;
  uint8_t *data;
  bool data_new;
};


void stereoboardTask(void* arg)
{

  systemWaitStart();
  struct uint8array stereocam_data = {.len = 0, .data = msg_buf, .data_new = 0, .height = 0}; // buffer used to contain image without line endings

  while(1){
    pprz_check_and_parse(&dev, &pprz, stereocam_data.data, &stereocam_data.data_new);


    uint8_t msg_id = stereocam_data.data[1];
    switch (msg_id) {

      case 81: {

        //float res = (float)DL_STEREOCAM_VELOCITY_resolution(stereocam_data.data);


        velx = (float)DL_STEREOCAM_VELOCITY_velx(stereocam_data.data);
        vely = (float)DL_STEREOCAM_VELOCITY_vely(stereocam_data.data);
        velz = (float)DL_STEREOCAM_VELOCITY_velz(stereocam_data.data);


       /*float noise = 1-(float)DL_STEREOCAM_VELOCITY_vRMS(stereocam_msg_buf)/res;

       // Rotate camera frame to body frame
       struct FloatVect3 body_vel;
       float_rmat_transp_vmult(&body_vel, &stereocam.body_to_cam, &camera_vel);
         */

        break;
      }

    }
    //DEBUG_PRINT("%d, check!\n",vel);

    vTaskDelay(1);

  }
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
LOG_ADD(LOG_FLOAT, velocity x, &velx)
LOG_ADD(LOG_FLOAT, velocity y, &vely)
LOG_ADD(LOG_FLOAT, velocity z, &velz)

LOG_GROUP_STOP(stereoboard)
