/*
 * PWM Detector for ARDUINO NANO
 * Description: Reads PWM from pin 2 and controls an LED on pin 12, or pin 6 based on error evaluation.
 * Includes serial communication handling with commands -h -t -i -p -s -e -te -b -l including range validation.
 * Date: 10.12.2025
 * Author: Jaroslav Macháček
 *
 * OPTIMALIZACE: Textové konstanty přesunuty do PROGMEM (F() a const char PROGMEM)
 * Nahrazena třída String za char[] pro sériový vstup.
 */
#include <Arduino.h>
#include <EEPROM.h>

// Pro PROGMEM
#include <avr/pgmspace.h> 

#define VERSION_SIZE 5
#define VERSION "V1.2"

// If you uncomment the following line, you will activate debug mode
//#define DEBUG_MODE 

#define INPUT_PIN 2
#define OUTPUT_PIN 12
#define CHYBA_PIN 6
#define BUFFER_SIZE 10

// --- TEXTY PŘESUNUTY DO PROGMEM (ÚSPORA SRAM) ---
const char TEXT_T[] PROGMEM = "Command: -t [number1] [number2] to set duty cycle limits in % for flipping (1-99) pin6.";
const char TEXT_I[] PROGMEM = "Command: -i [number] 0/1 - no/yes invert output.";
const char TEXT_P[] PROGMEM = "Command: -p [number1] [number2] to set limits for correct period in us (100-65000).";
const char TEXT_S[] PROGMEM = "Command: -s [number1] [number2] to set limits for correct duty cycle in permille (1-499).";
const char TEXT_E[] PROGMEM = "Command: -e [number1] to set the number of consecutive errors before the error pin (0-255) pin12 flips.";
const char TEXT_TE[] PROGMEM = "Command: -te [number] minimum error signaling duration (10-65000) ms.";
const char TEXT_BPS[] PROGMEM = "Command: -b [number] Serial buat rate 96 -> 9600, 144 -> 14400, 192 -> 19200, 288 -> 28800, 384 -> 38400, 576 -> 57600, 1152 -> 115200";
const char TEXT_L[] PROGMEM = "Command: -l [number] 0/1 - no/yes lists current values of frequency and duty cycle";
const char TEXT_H[] PROGMEM = "\nCommands: -h for help\n";
const char TEXT_IDN[] PROGMEM = "Arduino NANO for measuring PWM signal duty cycle.";
const char TEXT_HIDN[] PROGMEM = "Command: *IDN?";
const char TEXT_FETC[] PROGMEM = "Command: :FETCh? returns the duty cycle values ​​of the PWM signal in per mille";
const char TEXT_PWID[] PROGMEM = "Command: :MEASure:PWIDth? returns the length value of the HIGH signal";


#define ERROR_OFF 255
#define MAX_COMMAND_LENGTH 60 // Max. délka příkazu pro char buffer

struct PwmMeasurement {
  unsigned long period;
  int dutyCyclePromile;
  unsigned long impuls;
};

volatile unsigned long zacatekChyby=0;

volatile PwmMeasurement buffer[BUFFER_SIZE];
volatile byte head = 0;
volatile byte tail = 0;

// Variables used in ISR
volatile unsigned long lastEdgeTime = 0; 
volatile unsigned long highDuration = 0;
volatile unsigned long lowDuration = 0;
volatile bool pinState = false; // Current state of the pin

// --- GLOBALNI DEKLARACE PROMENNYCH používaných ve smičce loop
unsigned long stablePeriod;  // perioda v us
int stableDutyCyclePromile;  // střída v promile skutečná od 0 do 1000
int stridaPromile;           // střida v promile překlopena tak, aby byla od 0 do 500
unsigned long impuls;        // delka impulsu v us
float pwid = 0.0;            // delka impulsu v sekundách
char inChar;                 // po čtení seriové linky



void pushMeasurement(unsigned long period, int dutyCycle, unsigned long impuls) {
  byte nextHead = (head + 1) % BUFFER_SIZE;
  if (nextHead != tail) {
    buffer[head].period = period;
    buffer[head].dutyCyclePromile = dutyCycle;
    buffer[head].impuls = impuls;
    head = nextHead;
  }
}

bool popMeasurement(unsigned long &period, int &dutyCycle, unsigned long &impuls) {
  if (head == tail) return false;
  period = buffer[tail].period;
  dutyCycle = buffer[tail].dutyCyclePromile;
  impuls = buffer[tail].impuls;
  tail = (tail + 1) % BUFFER_SIZE;
  return true;
}

//--------------------------------------------------------------
// Interrupt Service Routine (ISR) function
//--------------------------------------------------------------
void handlePinChange() {
  unsigned long currentTime = micros();
  unsigned long duration = currentTime - lastEdgeTime;
  lastEdgeTime = currentTime; // Update the edge time immediately
  bool log1=0;                // definice jaká je hodnota na stupu když optočlenem protéká proud

  if (digitalRead(INPUT_PIN) == log1) {
    // Transition from LOW to HIGH arrived. The previous segment was LOW.
    lowDuration = duration;
    pinState = true;
    
    // If we have both components, we can calculate the period and duty cycle
    if (highDuration > 0 && lowDuration > 0) {
        unsigned long currentPeriod = highDuration + lowDuration;
        if (currentPeriod > 0) {
             // Calculate duty cycle: highDuration / period * 1000
             int dutyCycle = (highDuration * 1000UL) / currentPeriod;
             pushMeasurement(currentPeriod, dutyCycle, highDuration);
        }
    }

  } else {
    // Transition from HIGH to LOW arrived. The previous segment was HIGH.
    highDuration = duration;
    pinState = false;
  }
}

// Structure definition that holds all our data together
struct MojeNastaveni {
  char verze[VERSION_SIZE]; // verze např V01.0
  byte spodni_hranice;      // in % when it should switch to LOW
  byte horni_hranice;      // in % when it should switch to HIGH
  byte input;              // typ LOW/HIGH
  unsigned int min_perioda; // time in us
  unsigned int max_perioda; // time in us
  unsigned int min_strida;  // in permille (tenths of a percent)
  unsigned int max_strida;  // in permille (tenths of a percent)
  byte max_error;          // number of consecutive errors needed to flip the error pin
  unsigned int t_error;    // minimum error signaling duration ms
  unsigned int bps;        // Serial buat rate
  byte listing;            // 0/1 - no/yes lists current values of frequency and duty cycle 
};

// Global instance of our structure, which we will work with in the program
MojeNastaveni aktualniNastaveni;

const int EEPROM_ADRESA = 0;
// --- NAHRAZENO MÍSTO String incomingString ---
char incomingBuffer[MAX_COMMAND_LENGTH];
byte bufferIndex = 0; 
bool newData = false;
int pocetChyb=0;
int pocetChybPer=0;

// Pomocná funkce pro tisk řetězců z PROGMEM
void tiskniProgmem(const char *str) {
  Serial.println(reinterpret_cast<const __FlashStringHelper *>(str));
}

void zobrazNastaveni() {
  Serial.print(F("Version: ")); Serial.println(aktualniNastaveni.verze); 
  Serial.print(F("Threshold LOW: ")); Serial.print(aktualniNastaveni.spodni_hranice); Serial.println(F("%"));
  Serial.print(F("Threshold HIGH: ")); Serial.print(aktualniNastaveni.horni_hranice); Serial.println(F("%"));
  Serial.print(F("Invert output: ")); Serial.println(aktualniNastaveni.input); 
  Serial.print(F("Min period: ")); Serial.print(aktualniNastaveni.min_perioda); Serial.println(F(" us"));
  Serial.print(F("Max period: ")); Serial.print(aktualniNastaveni.max_perioda); Serial.println(F(" us"));
  Serial.print(F("Min duty cycle: ")); Serial.print(aktualniNastaveni.min_strida); Serial.println(F(" permille"));
  Serial.print(F("Max duty cycle: ")); Serial.print(aktualniNastaveni.max_strida); Serial.println(F(" permille"));
  Serial.print(F("Max number of errors: ")); Serial.println(aktualniNastaveni.max_error); 
  Serial.print(F("Min error signaling duration: ")); Serial.print(aktualniNastaveni.t_error); Serial.println(F(" ms"));
  Serial.print(F("Serial line speed: ")); Serial.print(aktualniNastaveni.bps); Serial.println(F("00 baud"));
  Serial.print(F("Listing of measured frequency and duty cycle values: ")); Serial.print(aktualniNastaveni.listing); Serial.println(F("[0-No; 1-Yes]"));
}

void setup() {
  pinMode(INPUT_PIN, INPUT);
  pinMode(OUTPUT_PIN, OUTPUT);
  pinMode(CHYBA_PIN, OUTPUT);
  // We use CHANGE, so we react to both edges
  attachInterrupt(digitalPinToInterrupt(INPUT_PIN), handlePinChange, CHANGE);

  // Reading values from EEPROM
  EEPROM.get(EEPROM_ADRESA, aktualniNastaveni);

  // Test if there is unreasonable data in EEPROM, then set default values and write them to EEPROM
  if (strcmp(aktualniNastaveni.verze, VERSION) != 0){
    Serial.begin(9600); // Dočasné
    Serial.println(F("First run !!!!"));
    strncpy(aktualniNastaveni.verze, VERSION, VERSION_SIZE);
    aktualniNastaveni.spodni_hranice=40;
    aktualniNastaveni.horni_hranice=60;
    aktualniNastaveni.input=0;
    aktualniNastaveni.min_perioda=18000;
    aktualniNastaveni.max_perioda=22000;
    aktualniNastaveni.min_strida=280;
    aktualniNastaveni.max_strida=330;
    aktualniNastaveni.max_error=1;
    aktualniNastaveni.t_error=800;
    aktualniNastaveni.bps=96;
    aktualniNastaveni.listing=0;
  
    EEPROM.put(EEPROM_ADRESA, aktualniNastaveni);
  }
  // Použití nového a elegantního zápisu Serial.begin(aktualniNastaveni.bps * 100UL);
  Serial.begin(aktualniNastaveni.bps * 100UL);
  Serial.println(F("\n\nThe Arduino is ready to evaluate the rectangular signal on PIN 2. It will evaluate its period and duty cycle."));
  Serial.println(F("Flipping PIN12 based on duty cycle and PIN6 based on error."));
  zobrazNastaveni();
  tiskniProgmem(TEXT_H); // TEXT_H je definován výše v PROGMEM

}

// Funkce pro parsování, validaci a ukládání (až) dvou hodnot nastavení
// Přepracováno pro použití const char* (C-řetězce)
bool zpracujUniverzalniPrikaz(const char* command, unsigned int minValue, unsigned int maxValue, byte* lowPtr, byte* highPtr, unsigned int* uLowPtr = nullptr, unsigned int* uHighPtr = nullptr) {
    long nove_spodni = -1;
    long nove_horni = -1;
    int count;

    // Zkusíme parsovat dvě čísla
    if (uHighPtr != nullptr || highPtr != nullptr) {
        // Formát: -x číslo1 číslo2
        // Použití sscanf pro parsování příkazu a dvou čísel. Ignorujeme návratovou hodnotu.
        count = sscanf(command, "%*s %ld %ld", &nove_spodni, &nove_horni);
        if (count < 2) return false;
    } else {
        // Formát: -x číslo1
        // Pro příkazy s jedním číslem (-e, -i, -l, -te, -b)
        count = sscanf(command, "%*s %ld", &nove_spodni);
        if (count < 1) return false;
        nove_horni = nove_spodni; // Pro validaci nastavíme horní na spodní
    }

    // --- SPECIÁLNÍ VALIDACE PRO BPS ---
    if (strcmp(command, "-b") == 0) {
      if (nove_spodni == 96 || nove_spodni == 144 || nove_spodni == 192 || nove_spodni == 288 || nove_spodni == 384 || nove_spodni == 576 || nove_spodni == 1152 ){
        if (uLowPtr) *uLowPtr = (unsigned int)nove_spodni;
        EEPROM.put(EEPROM_ADRESA, aktualniNastaveni);
        Serial.println(F("\nLimits successfully updated:"));
        zobrazNastaveni();
        return true;
      }
      // Pokud je bps mimo povolené hodnoty
      goto error_range; // Použití goto pro skok na tisk chyby (úspora opakování kódu)
    }

    // --- UNIVERZÁLNÍ ROZSAHOVÁ VALIDACE ---
    bool rangeValid = (nove_spodni >= minValue && nove_spodni <= maxValue && 
                       nove_horni >= minValue && nove_horni <= maxValue);

    if (uHighPtr != nullptr || highPtr != nullptr) { // Jen pro příkazy se dvěma hodnotami
        rangeValid = rangeValid && (nove_spodni <= nove_horni);
    }

    if (rangeValid) {
        // Uložení hodnot do příslušných proměnných pomocí pointerů
        if (uLowPtr) *uLowPtr = (unsigned int)nove_spodni;
        if (uHighPtr) *uHighPtr = (unsigned int)nove_horni;
        if (lowPtr) *lowPtr = (byte)nove_spodni;
        if (highPtr) *highPtr = (byte)nove_horni;
        
        EEPROM.put(EEPROM_ADRESA, aktualniNastaveni);
        Serial.println(F("\nLimits successfully updated:"));
        zobrazNastaveni();
        return true;

    } else {
      error_range: // Cíl skoku pro chybovou zprávu
      // Hodnota je mimo rozsah nebo nesprávný formát
      Serial.print(F("\nError: Values must be in the range ("));
      Serial.print(minValue);
      Serial.print(F("-"));
      Serial.print(maxValue);
      Serial.println(F(") and the first value must be less than or equal to the second."));
      Serial.println(F("State was not changed."));
      zobrazNastaveni();
      return false;
    }
}


void loop() {
  // === PART 1: PWM Signal Detection ===
 
  // test whether there are new values in the buffer
  if (popMeasurement(stablePeriod, stableDutyCyclePromile, impuls)) {
    // Data was successfully found and copied
    // test whether to flip the output
    if (stableDutyCyclePromile > 10 * aktualniNastaveni.horni_hranice) {
      digitalWrite(OUTPUT_PIN, !aktualniNastaveni.input);
    } else if (stableDutyCyclePromile < 10 * aktualniNastaveni.spodni_hranice) {
      digitalWrite(OUTPUT_PIN, aktualniNastaveni.input);
    }
    
    // here is added evaluation whether everything is within the correct limits
    stridaPromile = stableDutyCyclePromile;
    if (stableDutyCyclePromile > 500){
      stridaPromile = 1000 - stableDutyCyclePromile;
    }
    
    // Test duty cycle tolerance
    if (stridaPromile < aktualniNastaveni.min_strida || stridaPromile > aktualniNastaveni.max_strida ){
      if (pocetChyb==0){
        zacatekChyby=millis();
      }
      pocetChyb+=1;
      if (pocetChyb > aktualniNastaveni.max_error && aktualniNastaveni.max_error != ERROR_OFF){
        Serial.print(F("Signal duty cycle is outside tolerance.  ")); Serial.print(stableDutyCyclePromile/10.0); Serial.println(F(" %"));
        digitalWrite(CHYBA_PIN, HIGH);
      }
    } else{
      pocetChyb=0;
    }

    // Test period tolerance
    if (stablePeriod < aktualniNastaveni.min_perioda || stablePeriod > aktualniNastaveni.max_perioda ){
      if (pocetChybPer == 0){
        zacatekChyby=millis();
      }
      pocetChybPer+=1;
      if (pocetChybPer > aktualniNastaveni.max_error && aktualniNastaveni.max_error != ERROR_OFF){
        Serial.print(F("Signal period is outside tolerance.  ")); Serial.print(stablePeriod); Serial.println(F(" us"));
        digitalWrite(CHYBA_PIN, HIGH);
      }
    } else{
      pocetChybPer=0;
    }

    // Test whether an error occurred
    if (pocetChyb == 0 && pocetChybPer == 0 && digitalRead(CHYBA_PIN)==HIGH){
      if (millis()-zacatekChyby > aktualniNastaveni.t_error){
        digitalWrite(CHYBA_PIN, LOW);
      }
    }

    if (aktualniNastaveni.listing){
      float frequencyHz = 1000000.0 / stablePeriod;
      Serial.print(F("Frequency: ")); Serial.print(frequencyHz); Serial.print(F(" Hz | Duty Cycle: ")); Serial.print(stableDutyCyclePromile/10.0); Serial.println(F(" %"));
      delay(200);
    }
  }

  // === PART 2: Serial Communication Detection and Command Processing (C-string) ===
  while (Serial.available() > 0) {
    inChar = Serial.read();

    if (inChar == '\n' || inChar == '\r') {
      if (bufferIndex > 0) { // Zpracovat příkaz, pokud něco přišlo
        incomingBuffer[bufferIndex] = '\0'; // Ukončovací znak
        newData = true;
        break; 
      }
    } else if (bufferIndex < MAX_COMMAND_LENGTH - 1) {
      // Přidáme jen znaky a ignorujeme whitespace na začátku
      if (bufferIndex > 0 || inChar != ' ') {
        incomingBuffer[bufferIndex++] = inChar;
      }
    }
  }

  if (newData) {
    // --- Používáme C-řetězce a strcmp/strncmp místo String.equals/startsWith ---
    const char *cmd = incomingBuffer;
    if (strcmp(cmd, "-h") == 0) {
      tiskniProgmem(TEXT_T);
      tiskniProgmem(TEXT_I);
      tiskniProgmem(TEXT_P);
      tiskniProgmem(TEXT_S);
      tiskniProgmem(TEXT_E);
      tiskniProgmem(TEXT_TE);
      tiskniProgmem(TEXT_BPS);
      tiskniProgmem(TEXT_L);
      tiskniProgmem(TEXT_HIDN);
      tiskniProgmem(TEXT_FETC);
      tiskniProgmem(TEXT_PWID);

    } 
    else if (strcmp(cmd, "*IDN?") == 0){
      //*IDN?
      Serial.print(reinterpret_cast<const __FlashStringHelper *>(TEXT_IDN)); Serial.print(F(" Version: ")); Serial.println(aktualniNastaveni.verze);
    }
    else if (strcmp(cmd, ":FETCh?") == 0 || strcmp(cmd, ":FETC?") == 0 || strcmp(cmd, ":FETCH?") == 0 ){
      //:FETCh?
      Serial.println(stableDutyCyclePromile);
    }
    else if (strcmp(cmd, ":MEASure:PWIDth?") == 0 || strcmp(cmd, ":MEAS:PWID?") == 0 || strcmp(cmd, ":MEASURE:PWIDTH?") == 0 ){
      //:MEASure:PWIDth?
      pwid = (float)impuls / 1000000.0;
      Serial.println(pwid,6);
    }

    else if (strncmp(cmd, "-te", 3) == 0) {
      if (! zpracujUniverzalniPrikaz(cmd, 100, 65000, nullptr, nullptr, &aktualniNastaveni.t_error,  nullptr)){
        VypisChybu(TEXT_TE);
      };
    } 
    else if (strncmp(cmd, "-t", 2) == 0) {
      if (! zpracujUniverzalniPrikaz(cmd, 1, 99, &aktualniNastaveni.spodni_hranice, &aktualniNastaveni.horni_hranice)){
        VypisChybu(TEXT_T);
      };
    } 
    else if (strncmp(cmd, "-i", 2) == 0) {
      if (! zpracujUniverzalniPrikaz(cmd, 0, 1, &aktualniNastaveni.input, nullptr, nullptr, nullptr)){
        VypisChybu(TEXT_I);
      };
    } 
    else if (strncmp(cmd, "-l", 2) == 0) {
      if (! zpracujUniverzalniPrikaz(cmd, 0, 1, &aktualniNastaveni.listing, nullptr, nullptr, nullptr)){
        VypisChybu(TEXT_L);
      };
    }
    else if (strncmp(cmd, "-p", 2) == 0) {
      if (! zpracujUniverzalniPrikaz(cmd, 100, 65000, nullptr, nullptr, &aktualniNastaveni.min_perioda, &aktualniNastaveni.max_perioda)){
        VypisChybu(TEXT_P);
      };
    } 
    else if (strncmp(cmd, "-s", 2) == 0) {
      if (! zpracujUniverzalniPrikaz(cmd, 1, 499, nullptr, nullptr, &aktualniNastaveni.min_strida, &aktualniNastaveni.max_strida)){
        VypisChybu(TEXT_S);
      };
    } 
    else if (strncmp(cmd, "-e", 2) == 0) {
      if (! zpracujUniverzalniPrikaz(cmd, 0, 255, &aktualniNastaveni.max_error, nullptr, nullptr, nullptr)){
        VypisChybu(TEXT_E);
      };
    } 
    else if (strncmp(cmd, "-b", 2) == 0) {
      if (! zpracujUniverzalniPrikaz(cmd, 96, 1152, nullptr, nullptr, &aktualniNastaveni.bps,  nullptr)){
        VypisChybu(TEXT_BPS);
      };
    } 
    else {
      Serial.print(F("Unknown command or wrong format: "));
      Serial.println(cmd);
      tiskniProgmem(TEXT_H);
    }

    bufferIndex = 0; // Reset index pro další příkaz
    newData = false;
  }

  #ifdef DEBUG_MODE
    delay(100);
  #endif
}

// Změna volání z VypisChybu(String Text) na VypisChybu(const char* Text)
void VypisChybu(const char* Text){
  Serial.println(F("Invalid command"));
  tiskniProgmem(Text);
}