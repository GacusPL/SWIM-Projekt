/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "line_follower.h"
#include "dfplayer_mini.h"
#include <string.h> // Potrzebne dla strlen
#include <stdio.h>
#include <ctype.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Typ enum dla trybów pracy robota
typedef enum {
    ROBOT_MODE_STOPPED,
    ROBOT_MODE_LINE_FOLLOWER,
    ROBOT_MODE_BLUETOOTH_MANUAL
} RobotOperatingMode;

#define ADC_CHANNELS 8 // Definicja liczby kanałów ADC używanych do odczytu czujników linii
#define BT_COMMAND_BUFFER_SIZE 32 // Maksymalna długość komendy BT

uint16_t adc_buffer[ADC_CHANNELS]; // Bufor przechowujący odczyty z czujników linii (ADC)

volatile uint8_t robot_running = 0; // Zmienna globalna ustawiana w przerwaniu po naciśnięciu przycisku użytkownika (USER button).

volatile uint32_t echo_start = 0;	// Czas (w tickach timera) rozpoczęcia odbieranego echa
volatile uint32_t echo_end = 0;		// Czas (w tickach timera) zakończenia odbieranego echa
volatile uint8_t echo_captured = 0; // Flaga statusu przechwytywania echa (0: nic, 1: start, 2: koniec).

static char bt_command_buffer[BT_COMMAND_BUFFER_SIZE]; // Bufor na komendy z Bluetooth
static volatile uint8_t bt_cmd_buffer_idx = 0;         // Indeks w buforze komend BT

// Początkowe wartości PWM dla sterowania Bluetooth
const uint16_t INITIAL_BT_PWM_SPEED_LEFT  = 999; // Początkowa prędkość dla lewego silnika przy sterowaniu BT
const uint16_t INITIAL_BT_PWM_SPEED_RIGHT = 900; // Początkowa prędkość dla prawego silnika przy sterowaniu BT
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc4;
DMA_HandleTypeDef hdma_adc4;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile RobotOperatingMode current_robot_mode = ROBOT_MODE_STOPPED; // Domyślny tryb robota
static uint8_t dfp_current_volume = 12; // Domyślna głośność (0-30)
static uint8_t dfp_is_commanded_to_play = 0; // Czy użytkownik zażądał odtwarzania
static uint8_t dfp_is_user_paused = 0;     // Czy użytkownik zapauzował

// Zmienne przechowujące aktualne docelowe prędkości PWM dla sterowania BT
static uint16_t current_bt_pwm_left = INITIAL_BT_PWM_SPEED_LEFT;
static uint16_t current_bt_pwm_right = INITIAL_BT_PWM_SPEED_RIGHT;

// Stała różnica prędkości między lewym a prawym silnikiem, obliczana na podstawie wartości początkowych
static const int16_t pwm_speed_diff = INITIAL_BT_PWM_SPEED_LEFT - INITIAL_BT_PWM_SPEED_RIGHT;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC4_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

// Prototypy funkcji, które mogą być używane przez inne moduły (np. line_follower)
void robot_drive(uint16_t pwm_right, uint8_t dir_right, uint16_t pwm_left, uint8_t dir_left);
void robot_stop(void);
void UART_SendString(char *str);
float HCSR04_ReadDistance(void);
void HCSR04_Trigger(void);
uint8_t HC05_State(void);

void ProcessBluetoothCommand(char* command_str);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//Funkcja do uruchamiania robota w żądanym kierunku i prędkoscią za pomocą parametrów

void robot_drive(uint16_t pwm_right, uint8_t dir_right,
                 uint16_t pwm_left, uint8_t dir_left)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_right); // Ustawia wypełnienie PWM na kanale 1 (prawy silnik) – kontrola prędkości
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm_left); // Ustawia wypełnienie PWM na kanale 2 (lewy silnik) – kontrola prędkości
    // ustawianie odpowiednich stanów logicznych na wyjściach GPIO
    // prawy silnik
    if (dir_right) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);   // IN1
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); // IN2
    } else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // IN1
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);   // IN2
    }

    // lewy silnik
    if (dir_left) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);   // IN3
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET); // IN4
    } else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET); // IN3
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);   // IN4
    }

}

// Funkcja zatrzymująca ruch robota. Ustawia wszystkie wyjścia GPIO NA 0 oraz wypełnienie PWM na 0 na obu silnikach
void robot_stop()
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
}

// Funkcja Generuje krótki (10µs) impuls na pinie TRIG czujnika HC-SR04, aby zainicjować pomiar odległości
void HCSR04_Trigger(void)
{
	// Ustawia pin PB10 (TRIG) w stan wysoki, czeka 10 mikrosekund, a następnie ustawia go w stan niski.
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
    HAL_Delay(0.01);  // 10 us za pomocą HAL_Delay. Do zmiany w przyszłości aby liczyło czas za pomocą timera.
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
}

// Funkcja mierząca odległość za pomocą czujnika HC-SR04
// Inicjuje pomiar, czeka na impuls echa, oblicza czas jego trwania i przelicza na odległość w cm
// Używa timera TIM2 w trybie Input Capture do pomiaru czasu trwania impulsu echa
float HCSR04_ReadDistance(void)
{
    echo_captured = 0; // Reset flagi statusu przechwytywania echa

    // Uruchamia timer TIM2 w trybie Input Capture na kanale 4 (oczekiwanie na zbocze narastające)
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);

    HCSR04_Trigger(); // Wywołanie funkcji wysyłającej impuls

    uint32_t timeout = HAL_GetTick() + 100; // timeout na 100ms
    while (echo_captured < 2) // Czeka, aż zostaną przechwycone oba zbocza impulsu echa (start i end)
    {
        if (HAL_GetTick() > timeout)
        {
            return -1.0f; // wystąpił timeout (brak echa w ciągu 100ms), funkcja zwraca błąd -1.0f
        }
    }
    // Zatrzymanie timera
    HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_4);

    uint32_t ticks;	// Zmienna na czas trwania impulsu echa w tickach timera
    if (echo_end >= echo_start) 		// Normalny przypadek
        ticks = echo_end - echo_start;
    else								// Przypadek przepełnienia licznika timera
        ticks = (0xFFFFFFFF - echo_start + echo_end);

    float distance = ticks * 0.000239463f;
    // Obliczony współczynnik dla zegara systemowego 72MHz i prędkosci dzwięku ok 343m/s
    // Daje on wystarczającą dokładność dla wykrywania przeszkód podczas jazdy oraz
    // mierzenia dystansu w zakresie pracy czujnika

    return distance; // Zwrócenie obliczonej odległości
}

// Funkcja pomocnicza do wysyłająca wiadomości poprzez UART
// Jako parametr przyjmuje wskaźnik do ciągu znaków do wysłania
void UART_SendString(char *str) {
    HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
}


// Funkcja sprawdzająca stan połączenia Bluetooth
uint8_t HC05_State(void) {
	// Zwraca 1 jeśli połączony (HC-05 wysyła HIGH na pinie State)
    return HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_4);
}


// Funkcja przetwarzająca komendy z Bluetooth sterując ruchem robota lub zmieniając jego tryb pracy.
// Wywoływana po odebraniu pełnej komendy zakończonej znakiem nowej linii.
void ProcessBluetoothCommand(char* command_str) {
    char msg[70]; // Bufor na wiadomości dla UART
    int value;    // Zmiennna do przechowywania wartości liczbowych z komend (np. głośność, numer utworu, procent prędkości)

    // Wysyłanie potwierdzenia odebrania komendy (debugowanie)
    // snprintf(msg, sizeof(msg), "BT Odebrana komenda: [%s]\r\n", command_str);
    // UART_SendString(msg);

    if (strlen(command_str) == 0) { // Ignoruje puste komendy
        return;
    }

    char command_char = tolower(command_str[0]); // Pobiera pierwszy znak komendy i zamienia na małą literę

    switch (command_char) {
        case 'f': // Przód
            if (strlen(command_str) == 1) {
                if (current_robot_mode == ROBOT_MODE_BLUETOOTH_MANUAL) {
                    robot_drive(current_bt_pwm_right, 1, current_bt_pwm_left, 1);
                    UART_SendString("BT CMD: FWD\r\n");
                } else {
                    UART_SendString("BT Info: Tryb manualny nie aktywny\r\n");
                }
            } else {
                 snprintf(msg, sizeof(msg), "BT CMD: Nieznana komenda '%s'\r\n", command_str);
                 UART_SendString(msg);
            }
            break;
        case 'b': // Tył
            if (strlen(command_str) == 1) {
                if (current_robot_mode == ROBOT_MODE_BLUETOOTH_MANUAL) {
                    robot_drive(current_bt_pwm_right, 0, current_bt_pwm_left, 0);
                    UART_SendString("BT CMD: BCK\r\n");
                } else {
                    UART_SendString("BT Info: Tryb manualny nie aktywny\r\n");
                }
            } else {
                 snprintf(msg, sizeof(msg), "BT CMD: Nieznana komenda '%s'\r\n", command_str);
                 UART_SendString(msg);
            }
            break;
        case 'l': // Lewo
            if (strlen(command_str) == 1) {
                if (current_robot_mode == ROBOT_MODE_BLUETOOTH_MANUAL) {
                    robot_drive(current_bt_pwm_right, 1, current_bt_pwm_left, 0);
                    UART_SendString("BT CMD: LEFT\r\n");
                } else {
                    UART_SendString("BT Info: Tryb manualny nie aktywny\r\n");
                }
            } else {
                 snprintf(msg, sizeof(msg), "BT CMD: Nieznana komenda '%s'\r\n", command_str);
                 UART_SendString(msg);
            }
            break;
        case 'r': // Prawo
            if (strlen(command_str) == 1) {
                if (current_robot_mode == ROBOT_MODE_BLUETOOTH_MANUAL) {
                    robot_drive(current_bt_pwm_right, 0, current_bt_pwm_left, 1);
                    UART_SendString("BT CMD: RIGHT\r\n");
                } else {
                    UART_SendString("BT Info: Tryb manualny nie aktywny\r\n");
                }
            } else {
                 snprintf(msg, sizeof(msg), "BT CMD: Nieznana komenda '%s'\r\n", command_str);
                 UART_SendString(msg);
            }
            break;
        case 's': // Stop
            if (strlen(command_str) == 1) {
                robot_running = 0; // flaga na 0
                robot_stop();
                current_robot_mode = ROBOT_MODE_BLUETOOTH_MANUAL; // Po stop pozostaje w trybie manualnym aby płynnej sterować aplikacją w telefonie
                line_follower_reset_state();
                UART_SendString("BT CMD: STOP\r\n");
            } else {
                 snprintf(msg, sizeof(msg), "BT CMD: Nieznana komenda '%s'\r\n", command_str);
                 UART_SendString(msg);
            }
            break;

        case 'a': // Tryb autonomiczny - śledzenie linii
             if (strlen(command_str) == 1) {
                if (current_robot_mode != ROBOT_MODE_LINE_FOLLOWER || !robot_running) { // Jeśli nie był już w LF i nie jechał
                     robot_stop(); // Zatrzymanie, jeśli np. jedzie w trybie BT
                }
                current_robot_mode = ROBOT_MODE_LINE_FOLLOWER;
                robot_running = 1; // Odrazu jedzie (zmienna z PA0 ustawiona na 1)
                line_follower_reset_state();
                UART_SendString("BT CMD: Tryb -> Sledzenie linii\r\n");
             } else {
                 snprintf(msg, sizeof(msg), "BT CMD: Nieznana komenda '%s'\r\n", command_str);
                 UART_SendString(msg);
             }
            break;

        case 'm': // Tryb manualny - sterowanie Bluetooth
             if (strlen(command_str) == 1) {
                if (robot_running && current_robot_mode == ROBOT_MODE_LINE_FOLLOWER) {
                    robot_stop(); // Zatrzymanie jeśli jedzie
                }
                current_robot_mode = ROBOT_MODE_BLUETOOTH_MANUAL;
                robot_running = 0; // W trybie manualnym, `robot_running` nie kontroluje robotem bezpośrednio
                line_follower_reset_state();
                UART_SendString("BT CMD: Tryb -> Sterowanie reczne\r\n");
             } else {
                 snprintf(msg, sizeof(msg), "BT CMD: Nieznana komenda '%s'\r\n", command_str);
                 UART_SendString(msg);
             }
            break;

            // --- Sterowanie DFPlayer Mini ---


        case '1': // Odtwórz utwór 1
                	   dfplayer_play_track(1);
                    dfp_is_commanded_to_play = 1;
                    dfp_is_user_paused = 0;
                    UART_SendString("BT CMD: Track 1\r\n");
                    break;


            // Odtwórz utwór T<numer>
            case 't': // Komenda 'T' z parametrem numeru utworu
                if (command_char == 't' && strlen(command_str) > 1) {
                    if (sscanf(command_str + 1, "%d", &value) == 1) {
                        if (value > 0 && value <= 255) { // DFPlayer obsługuje do 255 utworów w folderze głównym
                            dfplayer_play_track((uint16_t)value);
                            dfp_is_commanded_to_play = 1;
                            dfp_is_user_paused = 0;
                            snprintf(msg, sizeof(msg), "BT CMD: Playing Track %d\r\n", value);
                            UART_SendString(msg);
                        } else {
                            UART_SendString("BT ERR: Numer utworu poza zakresem (1-255)\r\n");
                        }
                    } else {
                        UART_SendString("BT ERR: Bledny format komendy utworu. Uzyj T<numer_1_255>\r\n");
                    }
                } else if (command_char == 't' && strlen(command_str) == 1) { // Samo 't' bez parametru
                     UART_SendString("BT ERR: Komenda T wymaga numeru utworu. Uzyj T<numer_1_255>\r\n");
                } else { // Coś innego zaczynającego się na 't'
                    snprintf(msg, sizeof(msg), "BT CMD: Nieznana komenda '%s'\r\n", command_str);
                    UART_SendString(msg);
                }
                break;

        case '+': // Głośniej
            if (strlen(command_str) == 1) {
                if (dfp_current_volume < 30) {
                    dfp_current_volume++;
                }
                dfplayer_set_volume(dfp_current_volume);
                snprintf(msg, sizeof(msg), "BT CMD: Glosnosc w gore (%d/30)\r\n", dfp_current_volume);
                UART_SendString(msg);
            } else {
                snprintf(msg, sizeof(msg), "BT CMD: Nieznana komenda '%s'\r\n", command_str);
                UART_SendString(msg);
            }
            break;
        case '-': // Ciszej
            if (strlen(command_str) == 1) {
                if (dfp_current_volume > 0) {
                    dfp_current_volume--;
                }
                dfplayer_set_volume(dfp_current_volume);
                snprintf(msg, sizeof(msg), "BT CMD: Glosnosc w dol (%d/30)\r\n", dfp_current_volume);
                UART_SendString(msg);
            } else {
                snprintf(msg, sizeof(msg), "BT CMD: Nieznana komenda '%s'\r\n", command_str);
                UART_SendString(msg);
            }
            break;
        case 'n': // Następny utwór
            if (strlen(command_str) == 1) {
                dfplayer_next_track();
                dfp_is_commanded_to_play = 1;
                dfp_is_user_paused = 0;
                UART_SendString("BT CMD: Następny utwór\r\n");
            } else {
                snprintf(msg, sizeof(msg), "BT CMD: Nieznana komenda '%s'\r\n", command_str);
                UART_SendString(msg);
            }
            break;
        case 'v': // Poprzedni utwór (v) LUB Ustaw głośność (V<poziom>)
            if (command_char == 'v' && strlen(command_str) == 1) {
                dfplayer_prev_track();
                dfp_is_commanded_to_play = 1;
                dfp_is_user_paused = 0;
                UART_SendString("BT CMD: Poprzedni utwór\r\n");
            }
            // Ustaw głośność V<poziom>
            else if (command_char == 'v' && strlen(command_str) > 1) { // Komenda 'V' z parametrem głośności
                if (sscanf(command_str + 1, "%d", &value) == 1) {
                    if (value >= 0 && value <= 30) {
                        dfp_current_volume = (uint8_t)value;
                        dfplayer_set_volume(dfp_current_volume);
                        snprintf(msg, sizeof(msg), "BT CMD: Glosnosc ustawiona na %d/30\r\n", dfp_current_volume);
                        UART_SendString(msg);
                    } else {
                        UART_SendString("BT ERR: Glosnosc poza zakresem (0-30)\r\n");
                    }
                } else {
                    UART_SendString("BT ERR: Bledny format komendy glosnosci. Uzyj V<0-30>\r\n");
                }
            }
            else {
                 snprintf(msg, sizeof(msg), "BT CMD: Nieznana komenda '%s'\r\n", command_str);
                 UART_SendString(msg);
            }
            break;
        case 'p': // Pauza/Wznów
            if (strlen(command_str) == 1) {
                if (dfp_is_commanded_to_play) { // Działa tylko, jeśli muzyka była odtwarzana
                    if (!dfp_is_user_paused) {
                        dfplayer_pause();
                        dfp_is_user_paused = 1;
                        UART_SendString("BT CMD: Pauza\r\n");
                    } else {
                        dfplayer_resume();
                        dfp_is_user_paused = 0;
                        UART_SendString("BT CMD: Wznowiono\r\n");
                    }
                } else {
                    UART_SendString("BT Info: Nie odtwarzam muzyki.\r\n");
                }
            } else {
                 snprintf(msg, sizeof(msg), "BT CMD: Nieznana komenda '%s'\r\n", command_str);
                 UART_SendString(msg);
            }
            break;
         case 'x': // Stop odtwarzania
            if (strlen(command_str) == 1) {
                dfplayer_stop();
                dfp_is_commanded_to_play = 0;
                dfp_is_user_paused = 0;
                UART_SendString("BT CMD: Muzyka stop\r\n");
            } else {
                 snprintf(msg, sizeof(msg), "BT CMD: Nieznana komenda '%s'\r\n", command_str);
                 UART_SendString(msg);
            }
            break;

            // Ustaw prędkość W<procent_0_100>
            case 'w': // Komenda 'W' z parametrem procentu prędkości
                if (command_char == 'w' && strlen(command_str) > 1) { // Sprawdzanie czy jest podany parametr
                    if (sscanf(command_str + 1, "%d", &value) == 1) {
                        if (value >= 0 && value <= 100) {
                            // Obliczanie docelowego PWM dla lewego silnika (0-999) na podstawie procentu
                            uint16_t target_pwm_left = (uint16_t)((value / 100.0f) * 999.0f);

                            // Ograniczenie górne PWM do 999
                            if (target_pwm_left > 999) target_pwm_left = 999;

                            current_bt_pwm_left = target_pwm_left; // Ustawia nowe PWM dla lewego silnika

                            // Obliczanie PWM dla prawego silnika używając stałej różnicy
                            int16_t calculated_right_pwm = target_pwm_left - pwm_speed_diff;

                            // Sprawdzanie czy PWM dla prawego silnika nie jest ujemne i nie przekracza 999
                            if (calculated_right_pwm < 0) {
                                current_bt_pwm_right = 0;
                            } else if (calculated_right_pwm > 999) {
                                current_bt_pwm_right = 999;
                            }
                            else {
                                current_bt_pwm_right = (uint16_t)calculated_right_pwm; // Ustawia nowe PWM dla prawego silnika
                            }

                            snprintf(msg, sizeof(msg), "BT CMD: Predkosc ustawiona na %d%% (L:%d, P:%d)\r\n", value, current_bt_pwm_left, current_bt_pwm_right);
                            UART_SendString(msg);
                        } else {
                            UART_SendString("BT ERR: Procent predkosci poza zakresem (0-100)\r\n");
                        }
                    } else {
                        UART_SendString("BT ERR: Bledny format komendy predkosci. Uzyj W<0-100>\r\n");
                    }
                } else if (command_char == 'w' && strlen(command_str) == 1) { // Samo 'w' bez parametru
                    UART_SendString("BT ERR: Komenda W wymaga procentu predkosci. Uzyj W<0-100>\r\n");
                } else {
                    snprintf(msg, sizeof(msg), "BT CMD: Nieznana komenda '%s'\r\n", command_str);
                    UART_SendString(msg);
                }
                break;

        default:
            snprintf(msg, sizeof(msg), "BT CMD: Nieznana komenda '%s'\r\n", command_str);
            UART_SendString(msg);
            break;
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_ADC4_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  // Inicjalizacja DFPlayer Mini
  dfplayer_init(&huart2, GPIOC, GPIO_PIN_13);
  HAL_Delay(500);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);// Start generowania sygnału PWM na kanale 1 (prawy silnik)
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); // Start generowania sygnału PWM na kanale 2 (lewy silnik)

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET); // IrED = 1 -> Diody IR w czujniku odbiciowym włączone

  line_follower_init(); // Inicjalizacja modułu śledzenia linii

  //const uint16_t pwm_r = 980; // Minimalnie niższe wypełnienie PWM dla prawego silnika, by skorygować tor jazdy
  //const uint16_t pwm_l = 999; // Maksymalne wypełnienie PWM (pełna prędkość) dla lewegp silnika

  // Uruchomienie nasłuchiwania na porcie UART1 dla komend Bluetooth.
  // Odbieranie pojedynczych bajtów do bufora bt_command_buffer.
  HAL_UART_Receive_IT(&huart1, (uint8_t*)&bt_command_buffer[bt_cmd_buffer_idx], 1);

  HAL_ADC_Start_DMA(&hadc4, (uint32_t*)adc_buffer, ADC_CHANNELS); // Uruchomienie konwersji ADC w trybie DMA
   	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  // Odczyty z `ADC_CHANNELS` kanałów będą
  																  // automatycznie zapisywane do `adc_buffer

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  // Ustawienie początkowego trybu i stanu
  current_robot_mode = ROBOT_MODE_STOPPED; //  Ustawienie początkowego trybu robota na zatrzymany
  robot_running = 0; // Domyślnie zatrzymany

  // Wysłanie komunikatów powitalnych przez UART
  UART_SendString("Robot Gotowy!!!\r\nWyślij 'A' aby włączyć Podążanie po linii, 'M' dla ręcznego sterowania.\r\n");
  UART_SendString("Aktualny Tryb: Zatrzymano. Wciśnij PA0 albo wyślij komendę BT.\r\n");

  while (1)
  { // sterowanie diodą state (PE8)
	  if (HC05_State()) {
	      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);  // Dioda ON (połączony)
	  }
	  else {
	      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET); // Dioda OFF (brak połączenia)
	  }

	  /* Kontrola diody LED dla statusu BUSY DFPlayera */
	     if (dfplayer_is_busy()) {
	         // DFPlayer jest zajęty (gra muzykę) - zaświeć diodę na PE15
	         HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
	     } else {
	         // DFPlayer jest wolny (nie gra) - zgaś diodę na PE15
	         HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
	     }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  // switch sterujący robotem w zależności od ustawionego trybu
	  switch (current_robot_mode) {
	          case ROBOT_MODE_LINE_FOLLOWER:
	              if (robot_running) {
	                  float distance = HCSR04_ReadDistance(); // Pomiar dystansu

	                  if (distance >= 16.0f || distance < 0.0f) { // Jeśli nie ma przeszkody lub błąd czujnika
	                      line_follower_process(); // Wywołanie funkcji śledzenia linii
	                      HAL_Delay(1); // krótkie opóźnienie
	                  } else { // Przeszkoda wykryta
	                      robot_stop();
	                      UART_SendString("Przeszkoda: Zatrzymano.\r\n");
	                      // Opcjonalnie: Zawracanie
	                      // UART_SendString("Przeszkoda: Zawracam.\r\n");
	                      // HAL_Delay(500);
	                      // robot_drive(BT_PWM_SPEED_RIGHT, 1, BT_PWM_SPEED_LEFT, 0);
	                      // HAL_Delay(680);
	                      // robot_stop();
	                      // HAL_Delay(50);
	                  }
	              } else {
	                  robot_stop(); // zatrzymanie po wcisnieciu przycisku
	                  line_follower_reset_state();
	                  HAL_Delay(200);
	              }
	              break;

	          case ROBOT_MODE_BLUETOOTH_MANUAL:
	              // Logika ruchu jest obsługiwana w ProcessBluetoothCommand poprzez przerwanie UART
	              HAL_Delay(50); // Krótkie opóźnienie
	              break;

	          case ROBOT_MODE_STOPPED:
	              // Robot jest zatrzymany, silniki wyłączone
	              HAL_Delay(200); // odciążanie CPU
	              break;
	  }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_TIM1|RCC_PERIPHCLK_ADC34;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_HSI;
  PeriphClkInit.Adc34ClockSelection = RCC_ADC34PLLCLK_DIV1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC4_Init(void)
{

  /* USER CODE BEGIN ADC4_Init 0 */

  /* USER CODE END ADC4_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC4_Init 1 */

  /* USER CODE END ADC4_Init 1 */

  /** Common config
  */
  hadc4.Instance = ADC4;
  hadc4.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc4.Init.Resolution = ADC_RESOLUTION_12B;
  hadc4.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc4.Init.ContinuousConvMode = ENABLE;
  hadc4.Init.DiscontinuousConvMode = DISABLE;
  hadc4.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc4.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc4.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc4.Init.NbrOfConversion = 8;
  hadc4.Init.DMAContinuousRequests = ENABLE;
  hadc4.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc4.Init.LowPowerAutoWait = DISABLE;
  hadc4.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_61CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC4_Init 2 */

  /* USER CODE END ADC4_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xffffffff;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 PA3 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PF4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PE8 PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Funkcja obsługująca przerwanai zewnętrzne wywoływane przez wciśniecie User Button (PA0)
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_0) // Sprawdza, czy przerwanie pochodzi od przycisku użytkownika (PA0)
    {
        // Prosty debounce
        static uint32_t last_press_time = 0; // Zmienna przechowuje czas ostatniego zarejestrowanego naciśnięcia
        if (HAL_GetTick() - last_press_time < 250) { // 250ms debounce
            return;	// Ignoruj to naciśnięcie
        }
        last_press_time = HAL_GetTick();

        if (current_robot_mode == ROBOT_MODE_LINE_FOLLOWER) {
            robot_running = !robot_running; // Przełącz stan start/stop dla line-followera
            if (robot_running) {
                line_follower_reset_state(); // Resetowanie stanu przy starcie
                UART_SendString("PA0: START podążania po linii\r\n");
            } else { // Zatrzymanie robota
                robot_stop();
                UART_SendString("PA0: STOP podążania po linii\r\n");
            }
        } else if (current_robot_mode == ROBOT_MODE_BLUETOOTH_MANUAL) {
            // W trybie manualnym BT, PA0 działa jako nagły stop
            robot_stop();
            UART_SendString("PA0: Zatrzymano ręczne sterowanie\r\n");
        } else if (current_robot_mode == ROBOT_MODE_STOPPED) {
            // Jeśli zatrzymany, PA0 może go przełączyć i uruchomić w trybie Line Follower
            current_robot_mode = ROBOT_MODE_LINE_FOLLOWER;
            robot_running = 1; // Uruchomianie od razu
            line_follower_reset_state();
            UART_SendString("PA0: Zmieniono tryb na podążanie po linii\r\n");
        }
    }
}

// funkcja obsługująca przerwania od timera w trybie Input Capture (przechwytywania wejścia)
// Wywoływana, gdy timer wykryje zbocze na skonfigurowanym kanale
// służy do pomiaru czasu trwania impulsu echo z HC-SR04
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	// Sprawdzanie, czy przerwanie pochodzi od TIM2 i jego aktywnego kanału 4
    if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
    {
        if (echo_captured == 0) // Jeśli to pierwsze (narastające) zbocze impulsu echo
        {
            echo_start = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4); // Odczytaj i zapisz wartość licznika timera (czas startu)

            // Zmień polaryzację przechwytywania na zbocze opadające, aby złapać koniec impulsu
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_FALLING);
            echo_captured = 1; // Ustawia flagę, że start echa został przechwycony
        }
        else if (echo_captured == 1) // Jeśli to drugie (opadające) zbocze impulsu echo
        {
            echo_end = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4); // Odczytaj i zapisz wartość licznika timera (czas końca)

            // Zmień polaryzację z powrotem na zbocze narastające dla następnego pomiaru
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_RISING);
            echo_captured = 2; // Ustaw flagę, że koniec echa został przechwycony (pomiar kompletny)
        }
    }
}

// Callback wywoływany po odebraniu danych przez UART (Bluetooth)
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1) // Sprawdza, czy przerwanie pochodzi od USART1
  {
    uint8_t received_char = bt_command_buffer[bt_cmd_buffer_idx]; // Pobierz właśnie odebrany znak

    // Sprawdzenie czy odebrano znak końca linii
    if (received_char == '\n') // Jeśli odebrano LF
    {
        bt_command_buffer[bt_cmd_buffer_idx] = '\0'; // Kończenie stringa w miejscu LF
        // Uniknięcie pustych komend poprzez zamianę CR na \0
        if (bt_cmd_buffer_idx > 0 && bt_command_buffer[bt_cmd_buffer_idx - 1] == '\r') {
            bt_command_buffer[bt_cmd_buffer_idx - 1] = '\0';
        }

        if (strlen(bt_command_buffer) > 0) { // Przetwarza tylko jeśli komenda nie jest pusta
            ProcessBluetoothCommand(bt_command_buffer);
        }

        bt_cmd_buffer_idx = 0; // Reset indeksu dla nowej komendy
        memset(bt_command_buffer, 0, BT_COMMAND_BUFFER_SIZE); // Czyszczenie buforu
    }
    else if (received_char == '\r') // Jeśli odebrano CR
    {
        bt_command_buffer[bt_cmd_buffer_idx] = '\0'; // Kończenie stringa w miejscu CR
        // Komenda jest gotowa do przetworzenia
        if (strlen(bt_command_buffer) > 0) {
             ProcessBluetoothCommand(bt_command_buffer);
        }
        bt_cmd_buffer_idx = 0; // Reset indeksu dla nowej komendy
        memset(bt_command_buffer, 0, BT_COMMAND_BUFFER_SIZE); // Czyszczenie bufora
    }
    else
    {
        // Jeśli to zwykły znak to dodaje do bufora i inkrementuje indeks
        // Sprawdzenie, czy nie przekroczono rozmiaru bufora
        if (bt_cmd_buffer_idx < BT_COMMAND_BUFFER_SIZE - 1) {
            bt_cmd_buffer_idx++;
        } else {
            // Jeśli bufor jest pełny a nie było znaku końca to błąd
            UART_SendString("BT ERR: Bufor komendy przepełniony\r\n");
            bt_cmd_buffer_idx = 0; // Reset buforu
            memset(bt_command_buffer, 0, BT_COMMAND_BUFFER_SIZE);
        }
    }
    // Ponowne włączanie nasłuchiwania i zapisanie go w aktualnej pozycji w buforze
    HAL_UART_Receive_IT(&huart1, (uint8_t*)&bt_command_buffer[bt_cmd_buffer_idx], 1);
  }
}

// Opcjonalnie: obsługa błędów UART
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        // UART_SendString("UART Error\r\n");
        // Spróbuj ponownie uruchomić odbiór
        // Zabezpieczenie przed zapętlaniem się błędów
        uint32_t error = HAL_UART_GetError(huart);
        if(error != HAL_UART_ERROR_NONE) {
     	   // Można zresetować flagi błędów
     	   __HAL_UART_CLEAR_PEFLAG(huart);
     	   __HAL_UART_CLEAR_FEFLAG(huart);
     	   __HAL_UART_CLEAR_NEFLAG(huart);
     	   __HAL_UART_CLEAR_OREFLAG(huart);

     	  bt_cmd_buffer_idx = 0;
     	  memset(bt_command_buffer, 0, BT_COMMAND_BUFFER_SIZE);
        }

        if (HAL_UART_Receive_IT(&huart1, (uint8_t*)&bt_command_buffer[bt_cmd_buffer_idx], 1) != HAL_OK)
               {
            // Poważny błąd, można zresetować UART lub podjąć inne działania
        }
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
