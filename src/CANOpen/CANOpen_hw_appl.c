#include "CANOpen_hw_appl.h"

/* USER CODE BEGIN Includes */
#include <unistd.h>
#include <string.h>

#include <linux/can.h>

#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>
/* USER CODE END Includes */

/* Private typedef */
/* USER CODE BEGIN PD */
extern int s_can;
/* USER CODE END PD */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */  


void CANOpen_sendFrame(uint16_t cobID, uint8_t* data, uint8_t len){

  /* USER CODE BEGIN 1 */

  static struct can_frame frame;

  static struct timeval timeout;
  static fd_set set;
  static int rv;

  frame.can_id = cobID;
  frame.can_dlc = len;
  memcpy(frame.data, data, len);

  FD_ZERO(&set);
  FD_SET(s_can, &set);

  timeout.tv_sec = 0;
  timeout.tv_usec = 100000;

  rv = select(s_can + 1, NULL, &set, NULL, &timeout);
  if(rv >= 0 && FD_ISSET(s_can, &set)){
	  write(s_can, &frame, sizeof(frame));
  }

  /* USER CODE END 1 */  

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */  
