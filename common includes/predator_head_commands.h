#ifndef _HEAD_COMMAND_H
#define _HEAD_COMMAND_H

enum head_command : byte { C_STATUS = 0,
                      C_PLAY_FILE_ONCE,
                      C_PLAY_FILE_CONT,
                      C_PLAY_FILE_NEXT_CONT,
                      C_PLAY_RANDOM_FILE,

                      C_SET_VOLUME = 200,
                      C_FLASH_LEDS = 231,
                      C_LASERS_ON,
                      C_LASERS_OFF,

                      C_PAUSE = 255
                    };
					
#endif //_HEAD_COMMAND_H