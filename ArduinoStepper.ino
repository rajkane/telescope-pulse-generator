/*
 * =====================================================================
 * Projekt: Arduino ZC-600 Pulzný Generátor pre Pohon Ďalekohľadu
 * Platforma: Arduino Uno
 * =====================================================================
 * 
 * Popis a architektúra programu:
 * ---------------------------------------------------------------------
 * 1. Úloha: Program nespočítava astrometrickú pozíciu (to robí RTS2), 
 *    funguje ako hardvérový generátor pulzov a prevodník pre MAX485.
 * 2. Pulzy: Generuje presné pulzy na `pinpulsetx` podľa oneskorenia (`mydelay`).
 * 3. Riadenie: Sleduje stavový pin `pinACT` (12) s ošetrením zákmitych. 
 *    LOW = beh, HIGH = stop.
 * 4. Smer: Znamienko prijatého čísla cez Serial určuje smer (pindirtx).
 * 5. Optimalizácia: Spoľahlivé čítanie celého riadku (bez zbytočných dobehov 0).
 * =====================================================================
 * Napätie (Amplitude) 
5V|      ┌─────────┐         ┌─────────┐         ┌─────────┐
  |      │         │         │         │         │         │
0V+──────┘         └─────────┘         └─────────┘         └─────────► Čas (Time)
  0       175       350       525       700       875      1050 (µs)
         <--------> <-------->
          HIGH=175   LOW=175
           (Pulz)     (Pauza)
 * Amplitúda (Napätie): TTL logické úrovne Arduina (0 V v stave LOW a 5 V v stave HIGH). 
 * Poznámka: Modul MAX485 toto následne prevedie na diferenciálny RS-485 signál.
 * Trvanie pulzu (HIGH): nominálne 175 µs (delayMicroseconds(mydelay))
 * Trvanie medzery (LOW): nominálne 175 µs (delayMicroseconds(mydelay))
 * Celková perióda jedného cyklu: 175 + 175 = 350us. Frekvencia pulzov 2.86 kHz
 */

const int pindiren   = 2;                 // MAX485 smer TX povolenie
const int pinpulseen = 3;                 // MAX485 pulz TX povolenie
const int pindirrx   = 4;                 // MAX485 smer RX
const int pindirtx   = 5;                 // MAX485 smer TX
const int pinpulserx = 6;                 // MAX485 pulz RX
const int pinpulsetx = 7;                 // MAX485 pulz TX
const int pinACT     = 12;                // Vstupný signál aktivity/povolenia z TGA
const int pinLED     = 13;                // LED dióda na plosnom spoji s oznacenim L 

unsigned long mydelay = 175;            // Východiskové oneskorenie v µs
int val = 0;                
int prevval = 0;            
bool serialEnabled = true;

// Premenné pre ošetrenie zákmitych (debounce)
int lastReading = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 5;  // 5ms stabilizačný čas

void setup() {
  Serial.begin(9600);        
  Serial.setTimeout(50);
  delay(50); 
  
  // Vyčistenie sériového buffera pri štarte
  while (Serial.available() > 0) {
    Serial.read();
  }

  serialEnabled = true;
  mydelay = 175;

  pinMode(pinLED, OUTPUT);
  pinMode(pinACT, INPUT_PULLUP);       // Ochrana proti skratu pri rozpojení pinu
  pinMode(pindiren, OUTPUT);
  pinMode(pindirtx, OUTPUT);
  pinMode(pinpulseen, OUTPUT);
  pinMode(pinpulsetx, OUTPUT);
  
  digitalWrite(pindiren, HIGH);
  digitalWrite(pinpulseen, HIGH);
  digitalWrite(pinACT, HIGH);
  digitalWrite(pindirtx, HIGH);
  digitalWrite(pinpulsetx, LOW);

  // Načítanie reálneho počiatočného stavu pinu hneď na začiatku
  val = digitalRead(pinACT);
  lastReading = val;
  prevval = val; 
  digitalWrite(pinLED, val);

  // Úvodná správa pre Serial Monitor
  Serial.println("-------------------------------------");
  Serial.println("Arduino ZC-600 System Initialized");
  Serial.print("SPEED IS ");
  Serial.print(mydelay);
  Serial.println("us Running");
  Serial.print("Initial State: ");
  Serial.println(val == LOW ? "RUNS (LOW)" : "IDLE (HIGH)");
  Serial.println("-------------------------------------");
}

void loop() {
  // 1. Neblokujúce spracovanie príkazov zo sériovej linky (bezpečná riadková metóda)
  if (Serial.available() > 0) {
    // Prečítame celý reťazec až po znak nového riadku (tým sa vyprázdni buffer)
    String inputStr = Serial.readStringUntil('\n');
    inputStr.trim(); // Odstráni neviditeľné znaky (\r, medzery)

    if (inputStr.length() > 0) {
      long inputVal = inputStr.toInt();
      Serial.print("READ INT ");
      Serial.println(inputVal);
      
      if (inputVal == 0) {
        serialEnabled = false;
        mydelay = 175;

        // Najprv bezpečne vypnúť generovanie pulzov
        digitalWrite(pinpulseen, LOW);
        digitalWrite(pinpulsetx, LOW);
        digitalWrite(pindiren, LOW);

        Serial.println("Cycle stopped");
      } else {
        // Pri novom príkaze najprv vypnúť pulzy,
        // aby sa smer mohol zmeniť bez aktívneho pulzu.
        digitalWrite(pinpulseen, LOW);
        digitalWrite(pinpulsetx, LOW);

        if (inputVal < 0) {
          mydelay = abs(inputVal);
          // Nastavenie smeru ešte pred povolením pulzov
          digitalWrite(pindirtx, LOW);
        } else if (inputVal > 0) {
          mydelay = inputVal;
          // Nastavenie smeru ešte pred povolením pulzov
          digitalWrite(pindirtx, HIGH);
        }

        // Krátka pauza, aby bol DIR stabilný pred povolením pulzov
        delayMicroseconds(10);

        serialEnabled = true;

        // Pulzy sa povolia až po nastavení smeru
        digitalWrite(pindiren, HIGH);
        digitalWrite(pinpulseen, HIGH);
      }
    }
  }

  // 2. Sledovanie stavu hardvérového povolenia s ošetrením zákmitov v spojeni kontaktov
  int reading = digitalRead(pinACT);

  if (reading != lastReading) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != val) {
      prevval = val;
      val = reading;
      digitalWrite(pinLED, val);
      
      // 3. Výpis zmien stavu do Serial Monitoru (iba pri reálnej a stabilnej zmene)
      Serial.print("Tracking ");
      Serial.print(prevval);
      Serial.print(" > ");
      Serial.print(val);
      Serial.print(" DELAY ");
      Serial.print(mydelay);
      if (val == LOW) Serial.print(" RUNS");
      else Serial.print(" IDLE");
      Serial.print(" DIR ");
      Serial.println(digitalRead(pindirtx));
    }
  }
  lastReading = reading;

  // 4. Generovanie pulzov pre motor (plynulý chod)
  if ((val == LOW) && serialEnabled) {
    digitalWrite(pinpulsetx, HIGH);
    delayMicroseconds(mydelay);
    digitalWrite(pinpulsetx, LOW);
    delayMicroseconds(mydelay);
  }
}