#ifndef _UPDATE_H_
#define _UPDATE_H_

#define FLASH_SIZE                        0x40000                   //256K
#define APP_ADDR_IN_FLASH                 0x8003000                 //12K
#define FLASH_APP_SIZE                    (FLASH_SIZE - (APP_ADDR_IN_FLASH - 0x08000000))  //计算app空间可用大小

void UpdateApp(void);

#endif
