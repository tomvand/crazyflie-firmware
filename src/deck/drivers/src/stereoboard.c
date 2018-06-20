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
#include "zranger.h"
#include "arm_math.h"
#include "stabilizer_types.h"
#include "estimator_kalman.h"
#include "param.h"


uint16_t range_last;

//static char ch = 'a';
static float velx, vely, velz;
static float homingvector_x, homingvector_y;
static float motion_x, motion_y;
static bool make_snapshot;
uint8_t msg_id_check = 0;
uint8_t use_stereoboard = 1;
/*struct stereocam_t stereocam = {
   .device = 0,
   .msg_available = false
 };*/
struct UartDataStruct USART1_Data;
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
    msg_id_check = msg_id;

    motion_x = (float)msg_id;
    switch (msg_id) {


      case 90: {

       homingvector_x = DL_STEREOCAM_VISUALHOMING_X(stereocam_data.data);
       homingvector_y = DL_STEREOCAM_VISUALHOMING_Y(stereocam_data.data);

       uint8_t snapshot_on = 1;

        pprz_msg_send_STEREOCAM_VISUALHOMING_COMMAND(&(pprz.trans_tx), &dev, 0,
          &snapshot_on );
       if(make_snapshot == true)
       {
         // Stuff for Visual Homing
        // uint8_t dummy_uint8 = 0;
        // int8_t dummy_int8= 0;

          // make_snapshot = false;
       }


      }

      case 81: {

        //float res = (float)DL_STEREOCAM_VELOCITY_resolution(stereocam_data.data);


        velx = (float)DL_STEREOCAM_VELOCITY_velx(stereocam_data.data);
        vely = (float)DL_STEREOCAM_VELOCITY_vely(stereocam_data.data);
        velz = (float)DL_STEREOCAM_VELOCITY_velz(stereocam_data.data);

        float Npix = 30.0f;
        float thetapix = 0.07f;
        float rad_to_pixel = Npix / thetapix;


       //motion_x = rad_to_pixel*(float)atan((velz * 0.01f)*0.001f/(range_last * 0.001f));
       motion_y = rad_to_pixel*(float)atan((velx * 0.01f)*0.001f/(range_last * 0.001f));

        /*flowMeasurement_t flowData;
       flowData.stdDevX = 0.25;    // [pixels] should perhaps be made larger?
       flowData.stdDevY = 0.25;    // [pixels] should perhaps be made larger?
       flowData.dt = 0.01;
       flowData.dpixelx = motion_x;
       flowData.dpixely = motion_y;

      if (abs(motion_x) < 100 && abs(motion_y) < 100 && range_last > 100 && use_stereoboard == 1)
           estimatorKalmanEnqueueFlow(&flowData);*/

       /*float noise = 1-(float)DL_STEREOCAM_VELOCITY_vRMS(stereocam_msg_buf)/res;

       // Rotate camera frame to body frame
       struct FloatVect3 body_vel;
       float_rmat_transp_vmult(&body_vel, &stereocam.body_to_cam, &camera_vel);
         */

        break;
      }

    }
    //DEBUG_PRINT("%d, check!\n",vel);

    vTaskDelay(10);
  }
}


/* Initialize the deck driver */
void stereoboardDeckInit(DeckInfo *info)
{
  uart1Init(115200);

  datalink_init(&USART1_Data);

  DEBUG_PRINT("Test StereoboardDeck!\n");

  xTaskCreate(stereoboardTask, UART_RX_TASK_NAME, UART_RX_TASK_STACKSIZE, NULL, 3, NULL);


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
LOG_ADD(LOG_FLOAT, motion_x, &motion_x)
LOG_ADD(LOG_FLOAT, motion_y, &motion_y)

LOG_GROUP_STOP(stereoboard)

LOG_GROUP_START(monocam)
LOG_ADD(LOG_FLOAT, homingvector_x, &homingvector_x)
LOG_ADD(LOG_FLOAT, homingvector_y, &homingvector_y)
LOG_ADD(LOG_UINT8, make_snapshot, &make_snapshot)
LOG_ADD(LOG_UINT8, msg_id_check, &msg_id_check)

LOG_GROUP_STOP(monocam)


PARAM_GROUP_START(visualhoming)
PARAM_ADD(PARAM_UINT8, disable, &make_snapshot)
PARAM_GROUP_STOP(visualhoming)
