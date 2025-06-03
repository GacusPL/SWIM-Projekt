/*
 * dfplayer_mini.h
 */

#ifndef INC_DFPLAYER_MINI_H_
#define INC_DFPLAYER_MINI_H_

#include "stm32f3xx_hal.h"

// Komendy DFPlayerMini
#define DFPLAYER_CMD_PLAY_TRACK         0x03 // Odtwórz konkretny utwór (folder/plik)
#define DFPLAYER_CMD_SET_VOLUME         0x06 // Ustaw głośność (0-30)
#define DFPLAYER_CMD_SET_EQ             0x07 // Ustaw EQ (0:Normal, 1:Pop, 2:Rock, 3:Jazz, 4:Classic, 5:Base)
#define DFPLAYER_CMD_NEXT_TRACK         0x01 // Następny utwór
#define DFPLAYER_CMD_PREV_TRACK         0x02 // Poprzedni utwór
#define DFPLAYER_CMD_PLAY               0x0D // Odtwarzaj
#define DFPLAYER_CMD_PAUSE              0x0E // Pauza
#define DFPLAYER_CMD_STOP               0x16 // Stop
#define DFPLAYER_CMD_SPECIFY_FOLDER_PLAY 0x0F // Odtwórz plik z konkretnego folderu np. 01/001.mp3
#define DFPLAYER_CMD_QUERY_STATUS       0x42 // Zapytaj o status
#define DFPLAYER_CMD_QUERY_VOLUME       0x43 // Zapytaj o głośność
#define DFPLAYER_CMD_QUERY_FILES_COUNT  0x48 // Zapytaj o liczbę plików
#define DFPLAYER_CMD_SELECT_DEVICE      0x09 // Wybierz urządzenie (0x02 dla karty SD)

void dfplayer_init(UART_HandleTypeDef *huart, GPIO_TypeDef* busy_port, uint16_t busy_pin);
void dfplayer_send_cmd(uint8_t cmd, uint16_t arg);
void dfplayer_play_track(uint16_t track_num); // Odtwarza utwór numer X (w folderze domyślnym lub głównym)
void dfplayer_play_track_in_folder(uint8_t folder_num, uint8_t track_num_in_folder); // Odtwarza folder/plik
void dfplayer_set_volume(uint8_t volume); // 0-30
void dfplayer_next_track(void);
void dfplayer_prev_track(void);
void dfplayer_pause(void);
void dfplayer_resume(void);
void dfplayer_stop(void);
uint8_t dfplayer_is_busy(void);

#endif /* INC_DFPLAYER_MINI_H_ */
