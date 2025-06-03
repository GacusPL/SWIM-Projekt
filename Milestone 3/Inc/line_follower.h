/*
 * line_follower.h
 */

#ifndef LINE_FOLLOWER_H_
#define LINE_FOLLOWER_H_

#include "main.h"
#include <stdint.h>

//Konfiguracja Śledzenia Linii
#define ADC_CHANNELS 8 // Liczba czujników linii (kanałów ADC)

#define LINE_FOLLOWER_KP 0.001f // Współczynnik proporcjonalny (P) regulatora PID do korekcji błędu pozycji
#define LINE_FOLLOWER_KD 0.001f // Współczynnik różniczkujący (D) regulatora PID do tłumienia oscylacji

#define LINE_FOLLOWER_BASE_SPEED_PWM 450 // Podstawowa prędkość PWM silników
#define LINE_FOLLOWER_MIN_SPEED_PWM 450 // Minimalna dopuszczalna prędkość PWM silników
#define LINE_FOLLOWER_MAX_SPEED_PWM 850// Maksymalna dopuszczalna prędkość PWM silników

// Definicja bazowych prędkości PWM dla obu silników uwzględniajac kalibrację
#define LEFT_MOTOR_BASE_PWM (LINE_FOLLOWER_BASE_SPEED_PWM + 20 > LINE_FOLLOWER_MAX_SPEED_PWM ? LINE_FOLLOWER_MAX_SPEED_PWM : LINE_FOLLOWER_BASE_SPEED_PWM + 20)
#define RIGHT_MOTOR_BASE_PWM LINE_FOLLOWER_BASE_SPEED_PWM

#define SENSOR_LINE_THRESHOLD 600 // Próg odczytu ADC, powyżej którego uznaje się,
								  //że czujnik jest nad linią (białe tło, czarna linia)
#define SENSOR_WEIGHT_MULTIPLIER 1000 // Mnożnik używany do obliczania ważonej pozycji linii

// Docelowa wartość pozycji, gdy robot jest idealnie na środku linii
#define LINE_CENTER_POSITION ( ( (ADC_CHANNELS - 1) * SENSOR_WEIGHT_MULTIPLIER ) / 2 )

//Konfiguracja ostrych zakrętów
#define SHARP_TURN_PWM 550 // Prędkość PWM silników podczas wykonywania ostrego zakrętu
#define SHARP_TURN_DURATION_MS 600 // Czas trwania manewru ostrego zakrętu w milisekundach
#define SHARP_TURN_OUTER_SENSORS_ACTIVE 2 // Liczba skrajnych zewnętrznych czujników,
										  //które muszą wykryć linię, aby zidentyfikować ostry zakręt

#define SHARP_TURN_INNER_SENSORS_INACTIVE 3 // Liczba wewnętrznych czujników (od strony zakrętu),
											// które muszą NIE wykrywać linii,
											// aby potwierdzić ostry zakręt

//Konfiguracja oszukiwania zgubionej linii
#define LOST_LINE_REVERSE_PWM 550           // Prędkość PWM podczas cofania
#define LOST_LINE_REVERSE_DURATION_MS 400   // Czas cofania po zgubieniu linii
#define LOST_LINE_SWEEP_TURN_PWM 550        // Prędkość PWM podczas obrotu w miejscu przy szukaniu
#define LOST_LINE_SWEEP_DURATION_MS 550     // Czas jednego obrotu przy szukaniu
#define LOST_LINE_MAX_SEARCH_CYCLES 5       // Liczba pełnych cykli poszukiwania (lewo-prawo) zanim robot się podda


// Stany robota
typedef enum {
    STATE_FOLLOWING_LINE,				   // Stan gdy robot jedzie po linii

    // Stany dla zgubionej linii
    STATE_LOST_LINE_INITIATE_SEARCH,       // Stan początkowy po zgubieniu linii
    STATE_LOST_LINE_REVERSING,             // Robot cofa
    STATE_LOST_LINE_SEARCH_LEFT,     	   // Robot szuka obracając się w lewo w miejscu
    STATE_LOST_LINE_SEARCH_RIGHT,    	   // Robot szuka obracając się w prawo w miejscu
    STATE_LOST_LINE_FAIL_STOP,             // Robot się poddaje i zatrzymuje

    // Stany dla ostrych zakrętów
    STATE_DETECTED_SHARP_LEFT_TURN,
    STATE_PERFORMING_SHARP_LEFT_TURN,
    STATE_DETECTED_SHARP_RIGHT_TURN,
    STATE_PERFORMING_SHARP_RIGHT_TURN,
} RobotLineState;

extern uint16_t adc_buffer[ADC_CHANNELS]; // Bufor przechowujący odczyty z czujników linii (ADC)

void line_follower_init(void);            // Funkcja inicjalizująca moduł śledzenia linii
void line_follower_process(void);         // Główna funkcja przetwarzająca logikę śledzenia linii, wywoływana cyklicznie
void line_follower_reset_state(void);     // Funkcja resetująca stan robota do początkowego (np. STATE_FOLLOWING_LINE)

#endif /* LINE_FOLLOWER_H_ */
