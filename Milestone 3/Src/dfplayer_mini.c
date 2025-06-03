/*
 * dfplayer_mini.c
 */


#include "dfplayer_mini.h"
#include <stdio.h>

static UART_HandleTypeDef *dfp_huart;
static GPIO_TypeDef* dfp_busy_port;
static uint16_t dfp_busy_pin;

// Bufor na komendę
static uint8_t cmd_buffer[10];

// Funkcja do obliczania sumy kontrolnej
static uint16_t calculate_checksum(uint8_t *buffer) {
    uint16_t sum = 0;
    for (int i = 1; i < 7; i++) { // Sumowanie od Version do Data_LSB
        sum += buffer[i];
    }
    return -sum;
}

void dfplayer_init(UART_HandleTypeDef *huart, GPIO_TypeDef* busy_port, uint16_t busy_pin) {
    dfp_huart = huart;
    dfp_busy_port = busy_port;
    dfp_busy_pin = busy_pin;

    HAL_Delay(1000); // Czas na inicjalizację

    // Wybranie karty SD jako źródło
    dfplayer_send_cmd(DFPLAYER_CMD_SELECT_DEVICE, 0x0002); // 0x02 dla SD
    HAL_Delay(200);

    // Domyślna głośność
    dfplayer_set_volume(10);
    HAL_Delay(200);
}

void dfplayer_send_cmd(uint8_t cmd_code, uint16_t arg) {
    cmd_buffer[0] = 0x7E; // Bajt startu
    cmd_buffer[1] = 0xFF; // Wersja
    cmd_buffer[2] = 0x06; // Długość (nie licząc Start, End, Checksum)
    cmd_buffer[3] = cmd_code; // Command
    cmd_buffer[4] = 0x00; // Feedback (0x01 - z feedbackiem, 0x00 - bez)
    cmd_buffer[5] = (uint8_t)(arg >> 8);   // Argument MSB
    cmd_buffer[6] = (uint8_t)(arg & 0xFF); // Argument LSB

    uint16_t checksum = calculate_checksum(cmd_buffer);
    cmd_buffer[7] = (uint8_t)(checksum >> 8);   // Suma kontrolna MSB
    cmd_buffer[8] = (uint8_t)(checksum & 0xFF); // Suma kontrolna LSB
    cmd_buffer[9] = 0xEF; // Bajt końcowy

    HAL_UART_Transmit(dfp_huart, cmd_buffer, 10, HAL_MAX_DELAY);
}

void dfplayer_play_track(uint16_t track_num) {
    dfplayer_send_cmd(DFPLAYER_CMD_PLAY_TRACK, track_num);
}

void dfplayer_play_track_in_folder(uint8_t folder_num, uint8_t track_num_in_folder) {
    uint16_t arg = ((uint16_t)folder_num << 8) | track_num_in_folder;
    dfplayer_send_cmd(DFPLAYER_CMD_SPECIFY_FOLDER_PLAY, arg);
}

void dfplayer_set_volume(uint8_t volume) {
    if (volume > 30) volume = 30;
    dfplayer_send_cmd(DFPLAYER_CMD_SET_VOLUME, volume);
}

void dfplayer_next_track(void) {
    dfplayer_send_cmd(DFPLAYER_CMD_NEXT_TRACK, 0);
}

void dfplayer_prev_track(void) {
    dfplayer_send_cmd(DFPLAYER_CMD_PREV_TRACK, 0);
}

void dfplayer_pause(void) {
    dfplayer_send_cmd(DFPLAYER_CMD_PAUSE, 0);
}

void dfplayer_resume(void) {
    dfplayer_send_cmd(DFPLAYER_CMD_PLAY, 0);
}

void dfplayer_stop(void) {
    dfplayer_send_cmd(DFPLAYER_CMD_STOP, 0);
}

// Funkcja pomocnicza sprawdzająca stan logiczny pinu BUSY
// BUSY pin jest LOW gdy odtwarza, HIGH gdy jest wolny/zatrzymany
uint8_t dfplayer_is_busy(void) {
    if (GPIOC == NULL) return 0;
    return (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET); // Zwraca 1 jeśli gra, 0 jeśli wolny
}
