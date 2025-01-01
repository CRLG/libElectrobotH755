#include <errno.h>
#include <stdio.h>
#include "stm32h7xx_hal.h"

#if !defined(OS_USE_SEMIHOSTING)
 
#define STDIN_FILENO  0
#define STDOUT_FILENO 1
#define STDERR_FILENO 2
 
UART_HandleTypeDef *gHuart;

// Fonction à appler pour pouvoir utiliser le printf et rediriger vers l'UART routé sur le port USB
void RetargetInit(UART_HandleTypeDef *huart) {
  gHuart = huart;
 
  /* Disable I/O buffering for STDOUT stream, so that
   * chars are sent out as soon as they are printed. */
  setvbuf(stdout, NULL, _IONBF, 0);
}

int _write(int fd, char* ptr, int len) {
  HAL_StatusTypeDef hstatus;
 
  if (fd == STDOUT_FILENO || fd == STDERR_FILENO) {
    hstatus = HAL_UART_Transmit(gHuart, (uint8_t *) ptr, len, HAL_MAX_DELAY);
    if (hstatus == HAL_OK)
      return len;
    else
      return EIO;
  }
  errno = EBADF;
  return -1;
}

int _read(int fd, char* ptr, int len) {
  HAL_StatusTypeDef hstatus;
 
  if (fd == STDIN_FILENO) {
    hstatus = HAL_UART_Receive(gHuart, (uint8_t *) ptr, 1, HAL_MAX_DELAY);
    if (hstatus == HAL_OK)
      return 1;
    else
      return EIO;
  }
  errno = EBADF;
  return -1;
}

#endif //#if !defined(OS_USE_SEMIHOSTING)
