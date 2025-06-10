#include "line_follower.h" //import zmiennych i definicji z pliku nagłówkowego
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// Tablica wag przypisanych do każdego czujnika. Wagi rosną od lewej do prawej,
// co pozwala na obliczenie środka ciężkości wykrytej linii.
static const int32_t sensor_weights[ADC_CHANNELS] = {
    0 * SENSOR_WEIGHT_MULTIPLIER, 1 * SENSOR_WEIGHT_MULTIPLIER, 2 * SENSOR_WEIGHT_MULTIPLIER, 3 * SENSOR_WEIGHT_MULTIPLIER,
    4 * SENSOR_WEIGHT_MULTIPLIER, 5 * SENSOR_WEIGHT_MULTIPLIER, 6 * SENSOR_WEIGHT_MULTIPLIER, 7 * SENSOR_WEIGHT_MULTIPLIER
};

// Statyczne zmienne modułu przechowujące jego wewnętrzny stan

static float line_last_error_for_decision = 0.0f; // Ostatni zarejestrowany błąd pozycji linii,
//używany do podjęcia decyzji o kierunku poszukiwania linii po jej zgubieniu.
//Aktualizowany tylko, gdy linia jest wykrywana stabilnie

static float line_last_error_for_pid = 0.0f;      // Ostatni zarejestrowany błąd pozycji linii,
//używany do obliczenia członu różniczkującego regulatora PID

static int32_t last_known_position = LINE_CENTER_POSITION; // Ostatnia znana pozycja linii, nawet jeśli była chwilowa
static RobotLineState current_robot_state = STATE_FOLLOWING_LINE; // Aktualny stan robota
static uint32_t state_timer = 0; // Zmienna używana do odmierzania czasu trwania określonych stanów lub operacji
static uint8_t search_cycle_counter = 0; // Licznik cykli poszukiwania wykonanych podczas szukania zgubionej linii

// Prototypy funkcji prywatnych
static int32_t calculate_line_position(uint16_t *sensor_values);
static void handle_lost_line(void);
static void handle_sharp_turns(int32_t position, uint16_t *sensor_values);


// Funkcja pomocnicza inicjalizująca stan modułu szukania linii. Wytołana tylko raz

void line_follower_init(void) {
    line_last_error_for_decision = 0.0f;
    line_last_error_for_pid = 0.0f;
    last_known_position = LINE_CENTER_POSITION; // Zakładamy, że na starcie robot jest na środku linii
    current_robot_state = STATE_FOLLOWING_LINE; // Domyślny stan początkowy.
    state_timer = HAL_GetTick(); // Inicjalizacja prostego timera stanów za pomocą HAL
    search_cycle_counter = 0; // Zmienna zliczająca ilość cykli poszukiwania
}

// Resetuje stan modułu, pozwalając na uruchomienie od npwa podążania za linią
void line_follower_reset_state(void) {
    line_last_error_for_decision = 0.0f;
    line_last_error_for_pid = 0.0f;
    current_robot_state = STATE_FOLLOWING_LINE;
    search_cycle_counter = 0;
}

// Funkcja pomocnicza obliczająca pozycję linii na podstawie odczytów z czujników odbiciowych
// Zwraca wartość pozycji lub -1, jeśli linia nie została wykryta.

static int32_t calculate_line_position(uint16_t *sensor_values) {
    uint32_t weighted_sum = 0;          // Suma ważona (wartość czujnika * waga czujnika)
    uint16_t sum_of_active_sensors = 0; // Suma wartości aktywnych czujników
    uint8_t active_sensor_count = 0;    // Liczba czujników, które wykryły linie
    uint8_t first_active_sensor = ADC_CHANNELS; // Indeks pierwszego aktywnego czujnika od lewej

    // Czujnik jest uznawany za aktywny, jeśli jego odczyt przekracza próg SENSOR_LINE_THRESHOLD
    for (uint8_t i = 0; i < ADC_CHANNELS; i++) {
        if (sensor_values[i] > SENSOR_LINE_THRESHOLD) {
            weighted_sum += (uint32_t)sensor_values[i] * sensor_weights[i];
            sum_of_active_sensors += sensor_values[i];
            active_sensor_count++;
            if (i < first_active_sensor) { // Zapamiętanie indeksu pierwszego aktywnego czujnika.
                first_active_sensor = i;
            }
        }
    }

    if (active_sensor_count == 0) {
        return -1; // -1 oznacza że, linia nie została wykryta przez żaden czujnik
    }

    // Jeśli wszystkie czujniki widzą linię (np szeroka linia) to uznajemy, że robot jest na środku
    if (active_sensor_count == ADC_CHANNELS) {
        last_known_position = LINE_CENTER_POSITION;
        return LINE_CENTER_POSITION;
    }

    // Jeśli tylko jeden czujnik jest aktywny, jego pozycja jest traktowana jako pozycja linii.
    if (active_sensor_count == 1) {
         if (first_active_sensor == 0) { // Aktywny tylko skrajny lewy czujnik.
            last_known_position = sensor_weights[0];
            return sensor_weights[0];
         }
         if (first_active_sensor == ADC_CHANNELS - 1) { // Aktywny tylko skrajny prawy czujnik.
            last_known_position = sensor_weights[ADC_CHANNELS - 1];
            return sensor_weights[ADC_CHANNELS - 1];
         }
    }
    // Jeśli aktywny jest jeden ze środkowych czujników to obliczana jest średnia ważona
    // Normalizacja wyniku daje większą wagę czujnikom z silniejszym sygnałem
    last_known_position = weighted_sum / sum_of_active_sensors;
    return last_known_position; // Zwracamy pozycję
}

// Główna funkcja przetwarzająca logikę podążania za linią wywoływana cyklicznie

void line_follower_process(void) {
    // Obliczanie aktualnej pozycji linii na podstawie danych z bufora ADC
    int32_t current_position = calculate_line_position(adc_buffer);

    //Debug UART do sprawdzania wartości wskazywanych przez czujniki
    /*
    char uart_buf[120];
    sprintf(uart_buf, "ADC:[%4u %4u %4u %4u %4u %4u %4u %4u] Pos:%ld State:%d\r\n",
             adc_buffer[0], adc_buffer[1], adc_buffer[2], adc_buffer[3],
             adc_buffer[4], adc_buffer[5], adc_buffer[6], adc_buffer[7],
             current_position, current_robot_state);
     UART_SendString(uart_buf);
     */

    // W pierwszej kolejności sprawdzamy czy nie ma zakrętu
    handle_sharp_turns(current_position, adc_buffer);
    // Jeśli robot jest w trakcie zakrętu pomijamy resztę logiki
    if (current_robot_state == STATE_PERFORMING_SHARP_LEFT_TURN ||
        current_robot_state == STATE_PERFORMING_SHARP_RIGHT_TURN) {
        return;
    }

    // Linia jest znaleziona
    if (current_position != -1) {

        current_robot_state = STATE_FOLLOWING_LINE; // Ustawienie stanu robota na podążanie za linią
        search_cycle_counter = 0; // Zresetowanie licznika cykli poszukiwania bo linia została znaleziona

        // Obliczenie błędu jako odchylenia aktualnej pozycji robota od idealnej pozycji linii
        float error = (float)current_position - LINE_CENTER_POSITION;

        // Aktualizacja zmiennej line_last_error_for_decision tylko wtedy,
        //gdy błąd nie jest ekstremalny czyli np kiedy linia nie jest na skraju
        uint8_t active_sensors_count_temp = 0;
        for(int i=0; i<ADC_CHANNELS; ++i) if(adc_buffer[i] > SENSOR_LINE_THRESHOLD) active_sensors_count_temp++;

        if (abs((int)error) < (LINE_CENTER_POSITION * 0.85f) && active_sensors_count_temp > 0) {
             line_last_error_for_decision = error;
        }

        // Obliczenie członu różniczkującego (D) regulatora PID.
        float derivative = error - line_last_error_for_pid;
        // Obliczenie korekty prędkości silników za pomocą regulatora PD (Proporcjonalno-Różniczkującego)
        float motor_speed_correction = (LINE_FOLLOWER_KP * error) + (LINE_FOLLOWER_KD * derivative);
        // Zapisanie bieżącego błędu do użycia w następnej iteracji jako `line_last_error_for_pid`
        line_last_error_for_pid = error;

        // Obliczenie docelowych wartości PWM dla silników

        // Korekta jest odejmowana od lewego silnika i dodawana do prawego i
        // powoduje to skręt w kierunku przeciwnym do znaku błędu
        int16_t pwm_left_target = LEFT_MOTOR_BASE_PWM - (int16_t)motor_speed_correction;
        int16_t pwm_right_target = RIGHT_MOTOR_BASE_PWM + (int16_t)motor_speed_correction;

        // Ograniczenie prędkości silników do zdefiniowanych limitów w line_follower.h
        if (pwm_left_target > LINE_FOLLOWER_MAX_SPEED_PWM) pwm_left_target = LINE_FOLLOWER_MAX_SPEED_PWM;
        else if (pwm_left_target < LINE_FOLLOWER_MIN_SPEED_PWM) pwm_left_target = LINE_FOLLOWER_MIN_SPEED_PWM;

        if (pwm_right_target > LINE_FOLLOWER_MAX_SPEED_PWM) pwm_right_target = LINE_FOLLOWER_MAX_SPEED_PWM;
        else if (pwm_right_target < LINE_FOLLOWER_MIN_SPEED_PWM) pwm_right_target = LINE_FOLLOWER_MIN_SPEED_PWM;

        // Dodatkowa korekta dla bardzo dużych błędów (min sytuacje gdy linia jest na skrajnych czujnikach)
        // Ma to na celu wymuszenie ostrzejszego skrętu
        //zmienna `extreme_error_threshold` określa próg dla bardzo dużego błędu

        const int32_t extreme_error_threshold = (ADC_CHANNELS / 2 - 1) * SENSOR_WEIGHT_MULTIPLIER * 0.8f;
        if (abs((int)error) > extreme_error_threshold) {
            if (error > 0) { // Linia mocno po prawej, robot musi skręcić w prawo (lewe koło szybciej, prawe wolniej)
                pwm_left_target = LINE_FOLLOWER_MAX_SPEED_PWM;
                pwm_right_target = LINE_FOLLOWER_MIN_SPEED_PWM - 200 > 0 ? LINE_FOLLOWER_MIN_SPEED_PWM - 200 : 0; // Zmniejszenie prędkości prawego
            } else { // Linia mocno po lewej, robot musi skręcić w lewo
                pwm_right_target = LINE_FOLLOWER_MAX_SPEED_PWM;
                pwm_left_target = LINE_FOLLOWER_MIN_SPEED_PWM - 200 > 0 ? LINE_FOLLOWER_MIN_SPEED_PWM - 200 : 0;
            }
        }

        // Sterowanie silnikami z obliczonymi wartościami PWM i kierunkiem
        robot_drive(pwm_right_target, 1, pwm_left_target, 1);

        // Na końcu sprawdzamy czy linia nie została zgubiona (current_position == -1)
    } else {
        // Jeśli robot nie jest już w stanie poszukiwania lub wykonywania ostrego zakrętu,
        // a był wcześniej w stanie podążania za linią lub właśnie wykrył zakręt i zgubił linię,
        // to inicjujemy procedurę poszukiwania
        if (current_robot_state == STATE_FOLLOWING_LINE || current_robot_state == STATE_DETECTED_SHARP_LEFT_TURN ||
            current_robot_state == STATE_DETECTED_SHARP_RIGHT_TURN) {

            current_robot_state = STATE_LOST_LINE_INITIATE_SEARCH;

            // Zmienne `line_last_error_for_decision` i `last_known_position`
            // przechowują wartości sprzed zgubienia linii i zostaną użyte w funkcji handle_lost_line
        }
        handle_lost_line(); // Jeśli linia zgubiona to wywołujemy funkcję obsługującą ten stan
    }
}

// Funkcja obsługująca logikę zachowania robota po zgubieniu linii
// Linia jest poszukiwana sekwencyjnie do momentu aż robot znajdzie linię w ciągu ustalonej liczby cykli poszukiwania
static void handle_lost_line(void) {
    uint32_t current_time = HAL_GetTick(); // Pobranie aktualnego czasu systemowego.

    switch (current_robot_state) {
    // Inicjalizacja sekwencji
        case STATE_LOST_LINE_INITIATE_SEARCH:
            // Najpierw zaczyna on cofnięcia
            current_robot_state = STATE_LOST_LINE_REVERSING;
            state_timer = current_time; // Zapisz czas rozpoczęcia cofania.
            search_cycle_counter = 0;   // Zresetuj licznik cykli dla nowej sekwencji poszukiwania
            robot_drive(LOST_LINE_REVERSE_PWM, 0, LOST_LINE_REVERSE_PWM, 0); // Cofanie prosto z ustalonym PWM dla cofania
            break;

        case STATE_LOST_LINE_REVERSING:
            // Sprawdzanie czy nie upłynął czas przeznaczony na cofanie
            if (current_time - state_timer > LOST_LINE_REVERSE_DURATION_MS) {
                // Po cofnięciu robot decyduje o pierwszym kierunku obrotu w miejscu
                // Decyzja jest podjęta na podstawie wartości `line_last_error_for_decision` lub, jeśli jest on bliski zera, na `last_known_position`
                // Jeśli linia była po prawej (błąd > 0 lub LKP > centrum), to zaczyna szukać w prawo

            	// LKP - last_known_position

                if (line_last_error_for_decision > SENSOR_WEIGHT_MULTIPLIER / 4 || (abs((int)line_last_error_for_decision) <=
                		SENSOR_WEIGHT_MULTIPLIER / 4 && last_known_position > LINE_CENTER_POSITION)) {
                    current_robot_state = STATE_LOST_LINE_SEARCH_RIGHT;
                    robot_drive(LOST_LINE_SWEEP_TURN_PWM, 0, LOST_LINE_SWEEP_TURN_PWM, 1); // Obrót w prawo
                } else { // Linia była po lewej, na środku, lub błąd był nieznaczny i LKP <= centrum
                    current_robot_state = STATE_LOST_LINE_SEARCH_LEFT;
                    robot_drive(LOST_LINE_SWEEP_TURN_PWM, 1, LOST_LINE_SWEEP_TURN_PWM, 0); // Obrót w lewo
                }
                state_timer = current_time; // Zapisz czas rozpoczęcia obrotu
            } else {
                // Kontynuuj cofanie, jeśli czas nie upłynął
                robot_drive(LOST_LINE_REVERSE_PWM, 0, LOST_LINE_REVERSE_PWM, 0);
            }
            break;

        case STATE_LOST_LINE_SEARCH_LEFT:
            // Sprawdzanie czy upłynął czas przeznaczony na obrót w lewo
            if (current_time - state_timer > LOST_LINE_SWEEP_DURATION_MS) {
                // Zakończono obrót w lewo robot przełącza się na obrót w prawo
                current_robot_state = STATE_LOST_LINE_SEARCH_RIGHT;
                state_timer = current_time;
                robot_drive(LOST_LINE_SWEEP_TURN_PWM, 0, LOST_LINE_SWEEP_TURN_PWM, 1); // Obrót w prawo
                UART_SendString("Lost: Sweeping Right (from Left)\r\n");
                search_cycle_counter++; // Inkrementacja licznika cykli
            } else {
                // Kontynuacja obrotu w lewo
                robot_drive(LOST_LINE_SWEEP_TURN_PWM, 1, LOST_LINE_SWEEP_TURN_PWM, 0);
            }
            break;

        case STATE_LOST_LINE_SEARCH_RIGHT:
            // Sprawdzanie czy upłynął czas przeznaczony na obrót w prawo
            if (current_time - state_timer > LOST_LINE_SWEEP_DURATION_MS) {
                // Zakończono obrót w prawo
                // Sprawdzanie czy nie przekroczono maksymalnej liczby cykli poszukiwania
                // mnożnik *2 bo jeden cykl to obrót w lewo i w prawo, -1 bo licznik zaczyna od 0
                if (search_cycle_counter >= LOST_LINE_MAX_SEARCH_CYCLES * 2 -1) {
                    current_robot_state = STATE_LOST_LINE_FAIL_STOP;
                    robot_stop(); // Zatrzymaj robota.
                    UART_SendString("Linia zgubiona: Koniec cykli poszukiwania. Zatrzymano Robota.\r\n");
                } else {
                    // Robot przełącza się na obrót w lewo, kontynuując poszukiwanie.
                    current_robot_state = STATE_LOST_LINE_SEARCH_LEFT;
                    state_timer = current_time;
                    robot_drive(LOST_LINE_SWEEP_TURN_PWM, 1, LOST_LINE_SWEEP_TURN_PWM, 0); // Obrót w lewo
                    search_cycle_counter++; // Inkrementuj licznik cykli (pół cyklu)
                }
            } else {
                // Kontynuacja obrotu w prawo
                robot_drive(LOST_LINE_SWEEP_TURN_PWM, 0, LOST_LINE_SWEEP_TURN_PWM, 1);
            }
            break;

        case STATE_LOST_LINE_FAIL_STOP:
            robot_stop(); // Zatrzymanie robota
            break;

        default:
            // W przypadku nieoczekiwanego stanu inicjujemy poszukiwanie jeszcze raz
            current_robot_state = STATE_LOST_LINE_INITIATE_SEARCH;
            break;
    }
}

// Funkcja obsługująca wykrywanie i wykonywanie ostrych zakrętów.
static void handle_sharp_turns(int32_t position, uint16_t *sensor_values) {
    uint8_t active_left_outer = 0;  // Liczba aktywnych czujników na skrajnej lewej stronie
    uint8_t active_right_outer = 0; // Liczba aktywnych czujników na skrajnej prawej stronie
    uint8_t active_center = 0;      // Liczba aktywnych czujników w środkowej części
    uint8_t total_active = 0;       // Całkowita liczba aktywnych czujników

    // Zliczanie aktywnych czujników w poszczególnych strefach
    // Zmienna SHARP_TURN_OUTER_SENSORS_ACTIVE definiuje, ile skrajnych czujników jest branych pod uwagę
    for(int i = 0; i < ADC_CHANNELS; i++) {
        if (sensor_values[i] > SENSOR_LINE_THRESHOLD) {
            total_active++;
            if (i < SHARP_TURN_OUTER_SENSORS_ACTIVE) active_left_outer++;
            else if (i >= ADC_CHANNELS - SHARP_TURN_OUTER_SENSORS_ACTIVE) active_right_outer++;
            else active_center++;
        }
    }

    uint32_t current_time = HAL_GetTick(); // Pobranie czasu

    // Logika wykonywania ostrego zakrętu w lewo
    if (current_robot_state == STATE_PERFORMING_SHARP_LEFT_TURN) {
        // Zakończenie manewru skrętu, jeśli upłynął zdefiniowany czas lub jeśli środkowe czujniki ponownie wykryły linię
        if (current_time - state_timer > SHARP_TURN_DURATION_MS || (active_center > 0 && total_active < ADC_CHANNELS -1 && total_active > 0)) {
            int32_t pos_after_turn = calculate_line_position(sensor_values); // Sprawdzamy pozycję lini po zakręcie
            // Jeśli linia jest znaleziona i widoczna przez środkowe czujniki to ustawiamy odpowiedni stan
            if (pos_after_turn != -1 && active_center > 0) {
                current_robot_state = STATE_FOLLOWING_LINE;
                line_last_error_for_pid = 0; // Reset błędu PID w celu uniknięcia gwałtownej reakcji
                robot_drive(LINE_FOLLOWER_BASE_SPEED_PWM, 1, LINE_FOLLOWER_BASE_SPEED_PWM, 1);
            } else { // Linia nie została znaleziona lub nie jest widoczna centralnie po zakręcie
                current_robot_state = STATE_LOST_LINE_INITIATE_SEARCH; // Rozpocznij poszukiwanie linii
                // handle_lost_line zostanie wywołane w głównej pętli line_follower_process
            }
        } else {
             // Kontynuacja wykonywania ostrego skrętu w lewo
             robot_drive(SHARP_TURN_PWM, 1, (uint16_t)(SHARP_TURN_PWM * 0.3f), 0);
        }
        return; // Zakończenie jeśli wykonujemy zakręt
    }

    // Logika wykonywania ostrego zakrętu w prawo podobnie jak w lewo
    if (current_robot_state == STATE_PERFORMING_SHARP_RIGHT_TURN) {
        if (current_time - state_timer > SHARP_TURN_DURATION_MS || (active_center > 0 && total_active < ADC_CHANNELS-1 && total_active > 0 )) {
            int32_t pos_after_turn = calculate_line_position(sensor_values);
            if (pos_after_turn != -1 && active_center > 0) {
                current_robot_state = STATE_FOLLOWING_LINE;
                line_last_error_for_pid = 0;
                robot_drive(LINE_FOLLOWER_BASE_SPEED_PWM, 1, LINE_FOLLOWER_BASE_SPEED_PWM, 1);
            } else {
                current_robot_state = STATE_LOST_LINE_INITIATE_SEARCH;
            }
        } else {
            robot_drive((uint16_t)(SHARP_TURN_PWM * 0.3f), 0, SHARP_TURN_PWM, 1);
        }
        return;
    }

    // Logika wykrywania ostrych zakrętów (tylko jeśli robot jest w stanie STATE_FOLLOWING_LINE i widzi linię)
    if (current_robot_state == STATE_FOLLOWING_LINE && position != -1) {
        // Warunek dla ostrego zakrętu w lewo:
        // - Co najmniej SHARP_TURN_OUTER_SENSORS_ACTIVE skrajnych lewych czujników jest aktywnych
        // - Maksymalnie 1 aktywny czujnik w centrum (aby odróżnić od prostego odcinka)
        // - Całkowita liczba aktywnych czujników mieści się w pewnym zakresie
        // - Obliczona pozycja linii jest zdecydowanie po lewej stronie (mniejsza niż 1/3 LINE_CENTER_POSITION).
        if (active_left_outer >= SHARP_TURN_OUTER_SENSORS_ACTIVE &&
            active_center <= 1 &&
            total_active >= SHARP_TURN_OUTER_SENSORS_ACTIVE && total_active <= SHARP_TURN_OUTER_SENSORS_ACTIVE + 2 &&
            position < (LINE_CENTER_POSITION / 3) )
        {
            current_robot_state = STATE_DETECTED_SHARP_LEFT_TURN;
        }
        // Warunek dla ostrego zakrętu w prawo (analogiczny):
        // - Obliczona pozycja linii jest zdecydowanie po prawej stronie (większa niż centrum + 2/3 odległości od centrum do prawej krawędzi)
        else if (active_right_outer >= SHARP_TURN_OUTER_SENSORS_ACTIVE &&
                 active_center <= 1 &&
                 total_active >= SHARP_TURN_OUTER_SENSORS_ACTIVE && total_active <= SHARP_TURN_OUTER_SENSORS_ACTIVE + 2 &&
                 position > (LINE_CENTER_POSITION + (LINE_CENTER_POSITION / 3) * 2) )
        {
             current_robot_state = STATE_DETECTED_SHARP_RIGHT_TURN;
        }
    }

    // Rozpoczęcie wykonywania ostrego zakrętu po jego wykryciu
    if (current_robot_state == STATE_DETECTED_SHARP_LEFT_TURN) {
        current_robot_state = STATE_PERFORMING_SHARP_LEFT_TURN; // Zmiana stanu na wykonywanie.
        state_timer = HAL_GetTick(); // Zapisanie czasu rozpoczęcia manewru
        robot_drive(SHARP_TURN_PWM, 1, (uint16_t)(SHARP_TURN_PWM * 0.3f), 0);
        return;
    }

    if (current_robot_state == STATE_DETECTED_SHARP_RIGHT_TURN) {
        current_robot_state = STATE_PERFORMING_SHARP_RIGHT_TURN;
        state_timer = HAL_GetTick();
        robot_drive((uint16_t)(SHARP_TURN_PWM * 0.3f), 0, SHARP_TURN_PWM, 1);
        return;
    }
}
