# SWIM-Projekt 

# 🚗 Autonomiczny Pojazd z STM32

Projekt semestralny z przedmiotu **Systemy Wbudowane i Mikrokontrolery**  
Autorzy: _Kacper Szponar_, _Oliwier Bogdański_  
Numery indeksu: _21306_, _21181_  
Data rozpoczęcia: _23.04.2025_  
Repozytorium zawiera kod, dokumentację oraz materiały projektowe.

---

## 📌 Opis projektu

Celem projektu jest opracowanie modelu autonomicznego pojazdu sterowanego za pomocą mikrokontrolera STM32. Pojazd porusza się w trybie półautomatycznym lub automatycznym, omija przeszkody i/lub śledzi linię. Komunikacja z użytkownikiem odbywa się przez UART (Bluetooth lub przewodowo).

---

## 🛠️ Zastosowane technologie i komponenty

### Mikrokontroler
- STM32F303VCT6 Discovery
- Programowanie w języku C z użyciem bibliotek HAL/LL
- Sterownik silnika krokowego 2 DC L293D mini mostek H
- IDE: STM32CubeIDE

### Zasilanie
- 3x ogniwo Li-Ion INR18650-F1HR 3350mAh
- Łączne napięcie dostosowane do wymagań silników i mikrokontrolera
- Przetwornica napięcia LM2596HVS

### Napęd i sterowanie
- Chassis Rectangle 2WD – podwozie robota z napędem na dwa koła
- Silniki DC sterowane przy użyciu mostka H L293D

### Sensory
- HC-SR04 – ultradźwiękowy czujnik odległości (2–200 cm) z uchwytem montażowym
- KAmodQTR8A – moduł z 8 czujnikami odbiciowymi KTIR0711S (detekcja linii)

### Komunikacja
- Moduł Bluetooth HC-05
- Interfejs UART (komunikacja bezprzewodowa)


---

## ⚙️ Funkcjonalności

- ✅ Napęd sterowany przez PWM z użyciem Timerów
- ✅ Obsługa sensorów ultradźwiękowych (pomiar odległości)
- ✅ Odczyt wartości z sensorów IR (linia / przeszkody) przy użyciu ADC
- ✅ Detekcja kolizji i unikanie przeszkód
- ✅ Sterowanie ruchem przez UART (komendy tekstowe)
- ✅ Zasilanie bateryjne – pełna autonomia
- ✅ Regularne wersjonowanie kodu (min. 1 commit/tydzień)

---

## 📁 Struktura repozytorium


---

## 🔌 Komendy UART

| Komenda | Opis                           |
|--------:|------------------------------- |
| `A` | Tryb śledzenia linii               |
| `S`  | Zatrzymuje pojazd                 |
| `M`  | Tryb manualnej jazdy (bluetooth)  |
| `R` | Skręt w prawo                      |
| `L` | Skręt w lewo                       |  
| `B` | Jazda do tyłu                      | 
| `F` | Jazda prosto                       | 

---

## 🧪 Scenariusze testowe

- [x] Detekcja przeszkody z przodu (sensor HC-SR04)
- [x] Reakcja na białą/czarną linię (IR)
- [x] Komunikacja przez Bluetooth
- [x] Test zasilania bateryjnego
- [x] Sterowanie ruchem w czasie rzeczywistym

---

## 📸 Demo i zdjęcia

- Zdjęcia pojazdu: [`/Media/photos/`](./Media/photos/)
- Nagranie testów: [Demo Video](#) *(https://www.youtube.com/shorts/g8iPMdD_pI8)*

---

## 📄 Dokumentacja

Pełna dokumentacja projektu znajduje się w folderze [`Docs/`](./Docs/), w tym:
- Raport końcowy (PDF)
- Schematy układów
- Lista komponentów

---

## 🔌 Schemat układu

Poniżej znajduje się schemat układu elektronicznego pojazdu:

![Schemat układu](./Milestone%202/img/Schemat.png)

---

## 📅 Harmonogram pracy

- Tydzień 1–2: Koncepcja i lista komponentów  
- Tydzień 3–5: Budowa pojazdu i montaż elektroniki  
- Tydzień 6–9: Programowanie sensorów i napędu  
- Tydzień 10–12: Komunikacja UART + testy  
- Tydzień 13–14: Finalizacja, dokumentacja, raport  

---

## 🧠 Wnioski

_(Tutaj uzupełnij po zakończeniu projektu)_

---

## 📬 Kontakt

W razie pytań:
- Email: _21306@student.ans-elblag.pl_ lub _21181@student.ans-elblag.pl_
- GitHub: Kacper Szponar (https://github.com/GacusPL), Oliwier Bogdański (https://github.com/Thiago1717)

---

**Licencja:** MIT  
