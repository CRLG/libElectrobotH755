#ifndef _RETARGET_H__
#define _RETARGET_H__
 
#include <sys/stat.h>
#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

void RetargetInit(UART_HandleTypeDef *huart);
//int _isatty(int fd);
int _write(int fd, char* ptr, int len);
//int _close(int fd);
//int _lseek(int fd, int ptr, int dir);
int _read(int fd, char* ptr, int len);
//int _fstat(int fd, struct stat* st);

#ifdef __cplusplus
}
#endif

#endif //#ifndef _RETARGET_H__

