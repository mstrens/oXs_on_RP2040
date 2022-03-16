/* 

WIKI:
        https://github.com/Wiz-IO/wizio-pico/wiki/ARDUINO#vfs--file-system

OTHER USER CONFIG KEYS

LFS:    https://github.com/littlefs-project/littlefs   

        https://github.com/Wiz-IO/framework-wizio-pico/blob/main/arduino/libraries/RP2040/VFS/VFS_LFS.h

FATFS:  http://elm-chan.org/fsw/ff/00index_e.html

        https://github.com/Wiz-IO/framework-wizio-pico/blob/main/arduino/libraries/RP2040/VFS/VFS_FATFS.h

*/

#define MAX_OPEN_FILES  4

#define USE_LFS         /* Enable littlefs                  default pats */
#define USE_LFS_RAM     /* Use Ram disk                     R:/file_path */
#define USE_LFS_ROM     /* Use Rom disk ( internal flash )  F:/file_path */
#define USE_FATFS       /* Enable FatFS                     0:/file_path */

