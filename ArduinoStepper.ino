/*
 * =====================================================================
 * Projekt: Arduino ZC-600 Pulzný Generátor pre Pohon Ďalekohľadu
 * Platforma: Arduino Uno
 * =====================================================================
 *
 * Popis a architektúra programu:
 * ---------------------------------------------------------------------
 *
 * 1. Úloha:
 *    Arduino funguje ako hardvérový generátor pulzov a prevodník
 *    riadiacich signálov pre MAX485.
 *
 *    Astrometrické výpočty ani výpočet polohy ďalekohľadu Arduino
 *    nevykonáva. Riadenie zabezpečuje nadradený MASTER systém.
 *
 * 2. Sériová komunikácia:
 *    Komunikácia s MASTER systémom prebieha cez Serial:
 *
 *        9600 baud
 *
 *    MASTER posiela celé číslo ukončené znakom CR alebo LF.
 *
 *    Príklady:
 *
 *        175      -> oneskorenie 175 µs, smer HIGH
 *       -200      -> oneskorenie 200 µs, smer LOW
 *          0      -> STOP
 *
 *    Absolútna hodnota prijatého čísla určuje hodnotu `mydelay`.
 *    Znamienko určuje smer pohonu cez `pindirtx`.
 *
 * 3. Generovanie pulzov:
 *    Pulzy sú generované na výstupe `pinpulsetx` pomocou Timer1 v CTC
 *    režime s presnosťou 0.5 µs na pozadí (prerušenie).
 *
 *    Hodnota `mydelay` určuje trvanie HIGH aj LOW časti pulzu.
 *
 *    Pre `mydelay = 175 µs`:
 *
 *        HIGH = 175 µs
 *        LOW  = 175 µs
 *
 *    Celková perióda je:
 *
 *        175 + 175 = 350 µs
 *
 *    Frekvencia je presne:
 *
 *        1 / 350 µs = 2.857 kHz
 *
 * 4. Riadenie chodu:
 *    Arduino sleduje stav vstupu `pinACT` (pin 12).
 *
 *        LOW  = BEH
 *        HIGH = STOP
 *
 *    Stav vstupu je ošetrený debounce mechanizmom s časom 5 ms.
 *
 *    Pri HIGH sa pulzové vysielanie deaktivuje.
 *    Pri LOW sa pulzové vysielanie povolí, ak nebol sériovým
 *    príkazom vydaný STOP.
 *
 * 5. Smer:
 *    Znamienko prijatého čísla určuje stav výstupu `pindirtx`.
 *
 *        kladné číslo -> pindirtx HIGH
 *        záporné číslo -> pindirtx LOW
 *
 *    Napríklad:
 *
 *        175  -> mydelay = 175, smer HIGH
 *       -200  -> mydelay = 200, smer LOW
 *
 * 6. Sériový príkaz STOP:
 *    Príkaz:
 *
 *        0
 *
 *    deaktivuje generovanie pulzov, vypne príslušné MAX485 vysielanie
 *    a nastaví predvolenú hodnotu `mydelay` na 175 µs.
 *
 *    Ďalší nenulový platný príkaz opäť povolí sériové generovanie.
 *
 * 7. Sériové čítanie:
 *    Sériový vstup je spracovávaný neblokujúcim spôsobom po znakoch.
 *    Program čaká na ukončenie celého číselného príkazu znakom CR alebo LF.
 *
 *    Tým sa zabraňuje používaniu blokujúceho Serial.parseInt()
 *    a nechcenému spracovaniu neúplných údajov ako hodnoty 0.
 *
 * 8. MAX485:
 *    Arduino generuje TTL riadiace signály, ktoré sú pomocou MAX485
 *    prevedené na diferenciálny RS-485 signál.
 *
 *    Riadenie vysielačov:
 *
 *        pindiren   -> povolenie smerového TX
 *        pinpulseen -> povolenie pulzového TX
 *
 * =====================================================================
 *
 * Pulzný signál:
 *
 * 5V |      ┌─────────┐         ┌─────────┐
 *    |      │         │         │         │
 * 0V +──────┘         └─────────┘         └─────────► Čas
 *          <---175---> <---175--->
 *             HIGH        LOW
 *
 *    Pri mydelay = 175 µs:
 *
 *        HIGH = 175 µs
 *        LOW  = 175 µs
 *        perióda = 350 µs
 *        frekvencia = 2.857 kHz
 *
 * =====================================================================
 */

const int pindiren    = 2;   // MAX485 smer TX povolenie
const int pinpulseen  = 3;   // MAX485 pulz TX povolenie
const int pindirrx    = 4;   // MAX485 smer RX
const int pindirtx    = 5;   // MAX485 smer TX
const int pinpulserx  = 6;   // MAX485 pulz RX
const int pinpulsetx  = 7;   // MAX485 pulz TX
const int pinACT      = 12;  // Aktivita: LOW = BEH, HIGH = STOP
const int pinLED      = 13;  // LED: svieti = BEH
const int BAUNDS      = 9600;

// ---------------------------------------------------------------------
// Nastavenie rýchlosti a časovania pulzov
// ---------------------------------------------------------------------
volatile unsigned long mydelay = 175;  // Oneskorenie v µs
volatile bool pulseState = LOW;        // Aktuálny stav výstupného pulzu

// ---------------------------------------------------------------------
// Stav systému
// ---------------------------------------------------------------------
volatile int val     = HIGH;
int prevval          = HIGH;
// true  = sériový príkaz povoľuje chod
// false = sériovým príkazom bol vydaný STOP (0)
volatile bool serialEnabled = true;

// ---------------------------------------------------------------------
// Debounce pre pinACT
// ---------------------------------------------------------------------
int lastReading = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 5;

// ---------------------------------------------------------------------
// Buffer pre neblokujúce čítanie zo sériovej linky
// ---------------------------------------------------------------------
char inputBuffer[16];
byte bufferPos = 0;

// ---------------------------------------------------------------------
// Hardvérové prerušenie Timer1 pre presné generovanie pulzov
// ---------------------------------------------------------------------
ISR(TIMER1_COMPA_vect) {
  if ((val == LOW) && serialEnabled) {
    pulseState = !pulseState;
    if (pulseState) {
      PORTD |= (1 << PORTD7);  // HIGH na pin 7
    } else {
      PORTD &= ~(1 << PORTD7); // LOW na pin 7
    }
  } else {
    pulseState = LOW;
    PORTD &= ~(1 << PORTD7);   // Vynútené LOW na pin 7 pri STOPe
  }
}

// ---------------------------------------------------------------------
// Spracovanie sériového príkazu
// ---------------------------------------------------------------------
void processCommand(long inputVal) {
  Serial.print("READ INT ");
  Serial.println(inputVal);
  // -------------------------------------------------------------------
  // STOP
  // -------------------------------------------------------------------
  if (inputVal == 0) {
    serialEnabled = false;
    mydelay = 175;
    
    // Aktualizácia Timer1 pre predvolenú hodnotu 175 µs
    noInterrupts();
    TCNT1 = 0;
    OCR1A = (175 * 2) - 1; // Prescaler 8 (0.5 µs na tik)
    interrupts();

    // Okamžité vypnutie pulzového vysielania
    pulseState = LOW;
    PORTD &= ~(1 << PORTD7);
    digitalWrite(pinpulseen, LOW);
    // Vypnutie smerového vysielania
    digitalWrite(pindiren, LOW);
    Serial.println("Cycle stopped");
    return;
  }
  // -------------------------------------------------------------------
  // Pred zmenou smeru vždy vypneme pulzy
  // -------------------------------------------------------------------
  digitalWrite(pinpulseen, LOW);
  pulseState = LOW;
  PORTD &= ~(1 << PORTD7);
  // -------------------------------------------------------------------
  // Nastavenie smeru a rýchlosti
  // -------------------------------------------------------------------
  if (inputVal < 0) {
    mydelay = (unsigned long)(-inputVal);
    digitalWrite(pindirtx, LOW);
  } else {
    mydelay = (unsigned long)inputVal;
    digitalWrite(pindirtx, HIGH);
  }

  // Prepočet registra OCR1A pre Timer1 (prescaler 8 -> 1 tik = 0.5 µs)
  noInterrupts();
  TCNT1 = 0;
  OCR1A = (mydelay * 2) - 1;
  interrupts();

  // Krátka pauza po zmene smeru
  delayMicroseconds(10);
  // Nový platný príkaz opäť povoľuje sériové generovanie
  serialEnabled = true;
  // Smerový vysielač
  digitalWrite(pindiren, HIGH);
  // Pulzový vysielač povolíme iba pri BEHU
  if (val == LOW) {
    digitalWrite(pinpulseen, HIGH);
  }
  Serial.print("NEW DELAY ");
  Serial.print(mydelay);
  Serial.print(" us DIR ");
  Serial.println(digitalRead(pindirtx));
}

// ---------------------------------------------------------------------
// Neblokujúce čítanie sériovej linky
// ---------------------------------------------------------------------
void handleSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    // ---------------------------------------------------------------
    // Koniec príkazu
    // ---------------------------------------------------------------
    if (c == '\n' || c == '\r') {
      if (bufferPos > 0) {
        inputBuffer[bufferPos] = '\0';
        // -----------------------------------------------------------
        // Overenie formátu
        // -----------------------------------------------------------
        bool valid = true;
        bool hasDigit = false;
        for (byte i = 0; i < bufferPos; i++) {
          char ch = inputBuffer[i];
          if (i == 0 && ch == '-') {
            continue;
          }
          if (ch >= '0' && ch <= '9') {
            hasDigit = true;
          } else {
            valid = false;
            break;
          }
        }

        // -----------------------------------------------------------
        // Spracovanie iba platného čísla
        // -----------------------------------------------------------
        if (valid && hasDigit) {
          long inputVal = atol(inputBuffer);
          processCommand(inputVal);
        } else {
          Serial.print("INVALID COMMAND: ");
          Serial.println(inputBuffer);
        }
        bufferPos = 0;
      }
      continue;
    }

    // -----------------------------------------------------------------
    // Prijímame iba číslice a znamienko mínus
    // -----------------------------------------------------------------
    if ((c >= '0' && c <= '9') || c == '-') {
      if (bufferPos < sizeof(inputBuffer) - 1) {
        inputBuffer[bufferPos++] = c;
      } else {
        // Pretečenie bufferu
        bufferPos = 0;
        Serial.println("INPUT BUFFER OVERFLOW");
      }
    }
  }
}

// ---------------------------------------------------------------------
// Inicializácia
// ---------------------------------------------------------------------
void setup() {
  // -------------------------------------------------------------------
  // Master používa 9600 baud
  // -------------------------------------------------------------------
  Serial.begin(BAUNDS);
  // -------------------------------------------------------------------
  // Pin konfigurácia
  // -------------------------------------------------------------------
  pinMode(pinLED, OUTPUT);
  // LOW = BEH
  // HIGH = STOP
  pinMode(pinACT, INPUT_PULLUP);
  pinMode(pindiren, OUTPUT);
  pinMode(pinpulseen, OUTPUT);
  pinMode(pindirrx, INPUT);
  pinMode(pindirtx, OUTPUT);
  pinMode(pinpulserx, INPUT);
  pinMode(pinpulsetx, OUTPUT);
  // -------------------------------------------------------------------
  // Bezpečný počiatočný stav
  //
  // Najprv všetko vypneme.
  // Stav pinACT vyhodnotíme až potom.
  // -------------------------------------------------------------------
  digitalWrite(pindiren, LOW);
  digitalWrite(pinpulseen, LOW);
  digitalWrite(pinpulsetx, LOW);
  digitalWrite(pindirtx, HIGH);
  // -------------------------------------------------------------------
  // Zistenie počiatočného stavu pinACT
  // -------------------------------------------------------------------
  val = digitalRead(pinACT);
  lastReading = val;
  prevval = val;
  // -------------------------------------------------------------------
  // LED
  //
  // LOW  -> BEH -> LED ON
  // HIGH -> STOP -> LED OFF
  // -------------------------------------------------------------------
  digitalWrite(pinLED, !val);

  // -------------------------------------------------------------------
  // Konfigurácia Timer1 (CTC režim, prescaler 8)
  // -------------------------------------------------------------------
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  OCR1A  = (mydelay * 2) - 1; // Prescaler 8: 1 tik = 0.5 µs (16 MHz / 8 = 2 MHz)
  TCCR1B |= (1 << WGM12);     // CTC režim (Clear Timer on Compare Match)
  TCCR1B |= (1 << CS11);      // Prescaler 8
  TIMSK1 |= (1 << OCIE1A);    // Povolenie prerušenia Timer1 Compare Match A
  interrupts();

  // -------------------------------------------------------------------
  // Ak master už pri štarte povoľuje BEH,
  // aktivujeme vysielače.
  // -------------------------------------------------------------------
  if (val == LOW && serialEnabled) {
    digitalWrite(pindiren, HIGH);
    digitalWrite(pinpulseen, HIGH);
  }

  // -------------------------------------------------------------------
  // Diagnostické informácie
  // -------------------------------------------------------------------
  Serial.println("-------------------------------------");
  Serial.println("Arduino ZC-600 System Initialized");
  Serial.print("Serial: ");
  Serial.print(BAUNDS);
  Serial.println(" baud");
  Serial.print("SPEED IS ");
  Serial.print(mydelay);
  Serial.println(" us");
  Serial.print("Initial State: ");
  if (val == LOW) {
    Serial.println("RUNS (LOW)");
  } else {
    Serial.println("IDLE (HIGH)");
  }
  Serial.println("-------------------------------------");
}

// ---------------------------------------------------------------------
// Hlavná slučka
// ---------------------------------------------------------------------
void loop() {
  // ===================================================================
  // 1. SÉRIOVÁ KOMUNIKÁCIA
  // ===================================================================
  handleSerial();

  // ===================================================================
  // 2. PIN ACT + DEBOUNCE
  // ===================================================================
  int reading = digitalRead(pinACT);
  if (reading != lastReading) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != val) {
      prevval = val;
      val = reading;
      // ---------------------------------------------------------------
      // LED
      // ---------------------------------------------------------------
      digitalWrite(pinLED, !val);
      // ---------------------------------------------------------------
      // PIN ACT = HIGH
      // ---------------------------------------------------------------
      // Nadradený systém požaduje STOP.
      // Pulz okamžite vypneme.
      // ---------------------------------------------------------------
      if (val == HIGH) {
        pulseState = LOW;
        PORTD &= ~(1 << PORTD7);
        digitalWrite(pinpulseen, LOW);
        Serial.print("Tracking ");
        Serial.print(prevval);
        Serial.print(" > ");
        Serial.print(val);
        Serial.print(" DELAY ");
        Serial.print(mydelay);
        Serial.println(" IDLE");
      }
      // ---------------------------------------------------------------
      // PIN ACT = LOW
      // ---------------------------------------------------------------
      // Nadradený systém požaduje BEH.
      // Pulzy povolíme iba ak nebol sériovým príkazom
      // vydaný STOP (0).
      // ---------------------------------------------------------------
      else {
        if (serialEnabled) {
          digitalWrite(pindiren, HIGH);
          digitalWrite(pinpulseen, HIGH);
        }
        Serial.print("Tracking ");
        Serial.print(prevval);
        Serial.print(" > ");
        Serial.print(val);
        Serial.print(" DELAY ");
        Serial.print(mydelay);
        Serial.print(" RUNS DIR ");
        Serial.println(digitalRead(pindirtx));
      }
    }
  }

  lastReading = reading;

  // ===================================================================
  // 3. GENEROVANIE PULZOV
  // ===================================================================
  // Pulzy sú generované na piny 7 automaticky na pozadí v prerušení
  // Timer1 (ISR TIMER1_COMPA_vect) s presnosťou 0.5 µs a nulovým driftom.
}
