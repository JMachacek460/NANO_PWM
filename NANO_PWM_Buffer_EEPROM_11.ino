/*
 * PWM Detector for ARDUINO NANO
 * Description: Reads PWM from pin 2 and controls an LED on pin 12, or pin 6 based on error evaluation.
 * Includes serial communication handling with commands -h -t -i -p -s -e -te -b -l -ds -cs including range validation.
 * Date: 14.12.2025
 * Author: Jaroslav Machacek
 *
 * OPTIMALIZACE: Textove konstanty presunuty do PROGMEM (F() a const char PROGMEM)
 * Nahrazena trida String za char[] pro seriovou linku.
 * pridany prikazy pro mereni *IDN?  :MEASure:PWIDth?  :FETCh? :MEASure:PERiod?
 */

#include <Arduino.h>
#include <EEPROM.h>
#include <avr/pgmspace.h> // Pro PROGMEM
#include <ctype.h> // Potreba pro funkci tolower()

//-------------------------------------
// definice verze
//-------------------------------------
#define VERSION "V1.3"
const size_t VERSION_SIZE = sizeof(VERSION);

// If you uncomment the following line, you will activate debug mode
//#define DEBUG_MODE 

//------------------------------------------------------------------------------
// --- DEFINICE PINU
//------------------------------------------------------------------------------
const byte INPUT_PIN = 2;
const byte OUTPUT_PIN = 12;
const byte ERROR_PIN = 6;

//------------------------------------------------------------------------------
// --- TEXTY PRESUNUTY DO PROGMEM (USPORA SRAM) ---
//------------------------------------------------------------------------------
const char TEXT_T[] PROGMEM = "-t [number1] [number2] to set duty cycle limits in % for flipping (1-99) pin12.";
const char TEXT_I[] PROGMEM = "-i [number] 0/1 - no/yes invert output.";
const char TEXT_P[] PROGMEM = "-p [number1] [number2] to set limits for correct period in us (100-65000).";
const char TEXT_S[] PROGMEM = "-s [number1] [number2] to set limits for correct duty cycle in permille (1-499).";
const char TEXT_E[] PROGMEM = "-e [number] to set the number of consecutive errors before the error pin (0-255) pin6 flips.";
const char TEXT_TE[] PROGMEM = "-te [number] minimum error signaling duration (10-65000) ms.";
const char TEXT_BPS[] PROGMEM = "-b [number] serial buat rate 96 -> 9600, 144 -> 14400, 192 -> 19200, 288 -> 28800, 384 -> 38400, 576 -> 57600, 1152 -> 115200.";
const char TEXT_L[] PROGMEM = "-l [number] 0/1 - no/yes lists current values of frequency and duty cycle.";
const char TEXT_DS[] PROGMEM = "-ds [char] decimal separator.";
const char TEXT_CS[] PROGMEM = "-cs [char] columns separator.";
const char TEXT_H[] PROGMEM = "-h for help.\r\n";

const char TEXT_HIDN[] PROGMEM = "*IDN? returns IDN";
const char TEXT_RST[] PROGMEM = "*RST sets all parameters to factory settings.";
const char TEXT_FETC[] PROGMEM = ":FETCh? returns the duty cycle values of the PWM signal in per mille.";
const char TEXT_PWID[] PROGMEM = ":MEASure:PWIDth? returns the length value of the HIGH signal.";
const char TEXT_PER[] PROGMEM = ":MEASure:PERiod? returns the signal period value.";
const char TEXT_EXAMPLE[] PROGMEM = "Example of a measurement command: :MEAS:PWID? :MEAS:PER?   answer e.g.: 0,006312; 0,020076";

const char TEXT_IDN[] PROGMEM = "Arduino NANO for measuring PWM signal duty cycle.";

const size_t MAX_COMMAND_LENGTH = 60; // Max. delka prikazu pro char buffer
const size_t EEPROM_ADRESA = 0;       // 

//------------------------------------------------------------------------------
// ---DEFINICE STRUKTUR
//------------------------------------------------------------------------------
struct MojeSerialCommand {
  char inChar;                                // po cteni seriove linky
  const char *cmdPtr;                         // ukazatel na incomingBuffer pro zpracování
  byte state = 0;                             // 0: nic, 2: mereni, 3: vytisknuto, 255: ukoncit
  char buffer[MAX_COMMAND_LENGTH];            // buffer pro nacitani seriove linky
  size_t index = 0;                           // ukazatel na buffer seriove linky
  bool isNewData = false;                     // true, kdyz jsou data v bufferu ukončena a připravena ke zpracování
};

struct MeasurementData {
    unsigned long period_us;                // stabilizovaná perioda v us
    unsigned long pulseLength_us;           // namerena delka impulsu v us
    unsigned int dutyCyclePromile;          // strida v promile skutecna (0-1000)
    unsigned int normalizedDutyCycle;       // normalizovana strida (0-500)     
};

struct ControlState {
    bool evaluateMeasurement = false;           // true - ma se vyhodnocovat novy odmer nebo vypadek signalu
    unsigned long lastMeasurementTime;          // cas kdy bylo naposledy uspesne precteno mereni z bufferu
    unsigned long startErrorTime=0;             // cas kdy zacla chyba
    const unsigned int signalTimeout_ms = 200;  // casVypadku: doba v ms po ktere se vyhodnoti neexistence signalu
    byte dutyCycleErrorCount = 0;               // pocitadlo kolikrat za sebou byla detekovana chyba stridy
    byte periodErrorCount = 0;                  // pocitadlo kolikrat za sebou byla detekovana chyba periody
    const byte ERROR_OFF = 255;                 // pokud je nastaveno pocet povolenych erroru na 255 tak se error nikdy nehlasi
};
// MojeNastaveni se uklada do EEPROM
struct MojeNastaveni {
  char verze[VERSION_SIZE]; // verze napr V01.0
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
  char decimalSeparator;
  char columnsSeparator;   // 
};

//------------------------------------------------------------------------------
// --- GLOBALNI DEKLARACE PROMENNYCH pouzivanych ve smicce loop
//------------------------------------------------------------------------------
MeasurementData mereni;
MojeSerialCommand mSerial;
ControlState control;
MojeNastaveni aktualniNastaveni;

//------------------------------------------------------------------------------
// --- CAST NA DETEKCI PWM
//------------------------------------------------------------------------------
const size_t BUFFER_SIZE = 10;
struct PwmMeasurement {
  unsigned long period; // perioda v us
  int duty;             // strida v promile
  unsigned long impuls; // delka HIGH impulsu v us
};

volatile PwmMeasurement buffer[BUFFER_SIZE];  // kruhovy buffer pro data z preruseni
volatile byte head = 0;                       // hlava bufferu PwmMeasurement
volatile byte tail = 0;                       // ocas bufferu PwmMeasurement

// Variables used in ISR
volatile unsigned long lastEdgeTime = 0; 
volatile unsigned long highDuration = 0;
volatile unsigned long lowDuration = 0;
volatile bool pinState = false; // Current state of the pin

void pushMeasurement(unsigned long period, int dutyCycle, unsigned long impuls) {
  byte nextHead = (head + 1) % BUFFER_SIZE;
  if (nextHead != tail) {
    buffer[head].period = period;
    buffer[head].duty = dutyCycle;
    buffer[head].impuls = impuls;
    head = nextHead;
  }
}

bool popMeasurement(unsigned long &period, unsigned int &dutyCycle, unsigned long &impuls) {
  if (head == tail) return false;
  period = buffer[tail].period;
  dutyCycle = buffer[tail].duty;
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
  bool log1=0;                // definice jaka je hodnota na stupu kdyz optoclenem proteka proud

  if (digitalRead(INPUT_PIN) == log1) {
    // Transition from LOW to HIGH arrived. The previous segment was LOW.
    lowDuration = duration;
    pinState = true;
    
    // If we have both components, we can calculate the period and duty cycle
    if (highDuration > 0 && lowDuration > 0) {
      unsigned long currentPeriod = highDuration + lowDuration;
      if (currentPeriod > 0) {
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

//------------------------------------------------------------------------------
// Pomocna funkce pro tisk retezců z PROGMEM
void tiskniProgmem(const char *str) {
  Serial.print(F("Command: "));
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
  Serial.print(F("Listing of measured frequency and duty cycle values: ")); Serial.print(aktualniNastaveni.listing); Serial.println(F(" [0-No; 1-Yes]"));
  Serial.print(F("Decimal separator: '")); Serial.print(aktualniNastaveni.decimalSeparator); Serial.println(F("'"));
  Serial.print(F("Columns separator: '")); Serial.print(aktualniNastaveni.columnsSeparator); Serial.println(F("'"));
} //void zobrazNastaveni() 

void tovarniNastaveni(){
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
    aktualniNastaveni.decimalSeparator=',';
    aktualniNastaveni.columnsSeparator=';';
  
    EEPROM.put(EEPROM_ADRESA, aktualniNastaveni);
} //void tovarniNastaveni

// Funkce pro parsovani, validaci a ukladani (az) dvou hodnot nastaveni
bool zpracujUniverzalniPrikaz(const char* command, unsigned int minValue, unsigned int maxValue, byte* lowPtr, byte* highPtr, unsigned int* uLowPtr = nullptr, unsigned int* uHighPtr = nullptr) {
    long nove_spodni = -1;
    long nove_horni = -1;
    int count;

    // Zkusime parsovat dve cisla
    if (uHighPtr != nullptr || highPtr != nullptr) {
        // Format: -x cislo1 cislo2
        // Pouziti sscanf pro parsovani prikazu a dvou cisel. Ignorujeme navratovou hodnotu.
        count = sscanf(command, "%*s %ld %ld", &nove_spodni, &nove_horni);
        if (count < 2) return false;
    } else {
        // Format: -x cislo1
        // Pro prikazy s jednim cislem (-e, -i, -l, -te, -b)
        count = sscanf(command, "%*s %ld", &nove_spodni);
        if (count < 1) return false;
        nove_horni = nove_spodni; // Pro validaci nastavime horni na spodni
    }

    // --- SPECIaLNi VALIDACE PRO BPS ---
    if (strcmp(command, "-b") == 0) {
      if (nove_spodni == 96 || nove_spodni == 144 || nove_spodni == 192 || nove_spodni == 288 || nove_spodni == 384 || nove_spodni == 576 || nove_spodni == 1152 ){
        if (uLowPtr) *uLowPtr = (unsigned int)nove_spodni;
        EEPROM.put(EEPROM_ADRESA, aktualniNastaveni);
        delay(10);
        Serial.println(F("\r\nLimits successfully updated:"));
        zobrazNastaveni();
        return true;
      }
      // Pokud je bps mimo povolene hodnoty
      goto error_range; // Pouziti goto pro skok na tisk chyby (úspora opakovani kódu)
    }

    // --- UNIVERZaLNi ROZSAHOVa VALIDACE ---
    bool rangeValid = (nove_spodni >= minValue && nove_spodni <= maxValue && 
                       nove_horni >= minValue && nove_horni <= maxValue);

    if (uHighPtr != nullptr || highPtr != nullptr) { // Jen pro prikazy se dvema hodnotami
        rangeValid = rangeValid && (nove_spodni <= nove_horni);
    }

    if (rangeValid) {
        // Ulozeni hodnot do prislusnych promennych pomoci pointerů
        if (uLowPtr) *uLowPtr = (unsigned int)nove_spodni;
        if (uHighPtr) *uHighPtr = (unsigned int)nove_horni;
        if (lowPtr) *lowPtr = (byte)nove_spodni;
        if (highPtr) *highPtr = (byte)nove_horni;
        
        EEPROM.put(EEPROM_ADRESA, aktualniNastaveni);
        Serial.println(F("\r\nLimits successfully updated:"));
        zobrazNastaveni();
        return true;

    } else {
      error_range: // Cil skoku pro chybovou zpravu
      // Hodnota je mimo rozsah nebo nespravny format
      Serial.print(F("\r\nError: Values must be in the range ("));
      Serial.print(minValue);
      Serial.print(F("-"));
      Serial.print(maxValue);
      if (uHighPtr == nullptr && highPtr == nullptr){
        Serial.println(F(")."));
      }
      else {
        Serial.println(F(") and the first value must be less than or equal to the second."));
        
      }
      Serial.println(F("State was not changed."));
      //zobrazNastaveni();
      return false;
    }
} //bool zpracujUniverzalniPrikaz

void tiskniFloat(float cislo, int pocetDesetinychMist,  char separator) {
    char buffer[13]; 
    dtostrf(cislo, 0, pocetDesetinychMist, buffer);
    char *tecka = strchr(buffer, '.');
    if (tecka != NULL) {
        *tecka = separator;
    }
    Serial.print(buffer);
} // void tiskniFloat

void setup() {
  pinMode(INPUT_PIN, INPUT);
  pinMode(OUTPUT_PIN, OUTPUT);
  pinMode(ERROR_PIN, OUTPUT);
  // We use CHANGE, so we react to both edges
  attachInterrupt(digitalPinToInterrupt(INPUT_PIN), handlePinChange, CHANGE);

  // Reading values from EEPROM
  EEPROM.get(EEPROM_ADRESA, aktualniNastaveni);

  // Test if there is unreasonable data in EEPROM, then set default values and write them to EEPROM
  if (strcmp(aktualniNastaveni.verze, VERSION) != 0){
    Serial.begin(9600); // Docasne
    Serial.println(F("First run !!!!"));
    tovarniNastaveni();
  }
  Serial.begin(aktualniNastaveni.bps * 100UL);
  Serial.println(F("\r\n\r\nThe Arduino is ready to evaluate the rectangular signal on PIN 2. It will evaluate its period and duty cycle."));
  Serial.println(F("Flipping PIN12 based on duty cycle and PIN6 based on error."));
  zobrazNastaveni();
  Serial.print(F("\r\n"));
  tiskniProgmem(TEXT_H); 
  control.lastMeasurementTime=millis();
}

void loop() {
  // === PART 1: PWM Signal Detection ===
 
  // test whether there are new values in the buffer
  if (popMeasurement(mereni.period_us, mereni.dutyCyclePromile, mereni.pulseLength_us)) {
    // Data was successfully found and copied
    control.lastMeasurementTime =millis();
    control.evaluateMeasurement=true;
  }

  if(control.evaluateMeasurement){
    // test whether to flip the output
    control.evaluateMeasurement=false;
    if (mereni.dutyCyclePromile > 10 * aktualniNastaveni.horni_hranice) {
      digitalWrite(OUTPUT_PIN, !aktualniNastaveni.input);
    } else if (mereni.dutyCyclePromile < 10 * aktualniNastaveni.spodni_hranice) {
      digitalWrite(OUTPUT_PIN, aktualniNastaveni.input);
    }
    
    // here is added evaluation whether everything is within the correct limits
    mereni.normalizedDutyCycle = mereni.dutyCyclePromile;
    if (mereni.dutyCyclePromile > 500){
      mereni.normalizedDutyCycle = 1000 - mereni.dutyCyclePromile;
    }
    
    // Test duty cycle tolerance
    if (mereni.normalizedDutyCycle < aktualniNastaveni.min_strida || mereni.normalizedDutyCycle > aktualniNastaveni.max_strida ){
      if (control.dutyCycleErrorCount==0){
        control.startErrorTime=millis();
      }
      control.dutyCycleErrorCount+=1;
      if (control.dutyCycleErrorCount > aktualniNastaveni.max_error && aktualniNastaveni.max_error != control.ERROR_OFF){
        Serial.print(F("Signal duty cycle is outside tolerance.  ")); Serial.print(mereni.dutyCyclePromile/10.0); Serial.println(F(" %"));
        digitalWrite(ERROR_PIN, HIGH);
      }
    } else{
      control.dutyCycleErrorCount=0;
    }
    
    // Test period tolerance
    if (mereni.period_us < aktualniNastaveni.min_perioda || mereni.period_us > aktualniNastaveni.max_perioda ){
      if (control.periodErrorCount == 0){
        control.startErrorTime=millis();
      }
      control.periodErrorCount+=1;
      if (control.periodErrorCount > aktualniNastaveni.max_error && aktualniNastaveni.max_error != control.ERROR_OFF){
        Serial.print(F("Signal period is outside tolerance.  ")); Serial.print(mereni.period_us); Serial.println(F(" us"));
        digitalWrite(ERROR_PIN, HIGH);
      }
    } else{
      control.periodErrorCount=0;
    }

    // Test whether an error occurred
    if (control.dutyCycleErrorCount == 0 && control.periodErrorCount == 0 && digitalRead(ERROR_PIN)==HIGH){
      if (millis()-control.startErrorTime > aktualniNastaveni.t_error){
        digitalWrite(ERROR_PIN, LOW);
      }
    }

    if (aktualniNastaveni.listing){
      float frequencyHz = 1000000.0 / mereni.period_us;
      Serial.print(F("Frequency: ")); Serial.print(frequencyHz); Serial.print(F(" Hz | Duty Cycle: ")); Serial.print(mereni.dutyCyclePromile/10.0); Serial.println(F(" %"));
      delay(200);
    }
  } //konec control.evaluateMeasurement

  if (millis()-control.lastMeasurementTime>control.signalTimeout_ms){
    // watchdog
    mereni.period_us=0;
    mereni.dutyCyclePromile=0;
    mereni.pulseLength_us=0;
    control.evaluateMeasurement=true;
    control.lastMeasurementTime=millis();
  } // if (millis()-control.lastMeasurementTime>control.signalTimeout_ms)

  // === PART 2: Serial Communication Detection and Command Processing (C-string) ===
  while (Serial.available() > 0) {
    mSerial.inChar = Serial.read();

    if (mSerial.inChar == '\n' || mSerial.inChar == '\r') {
      if (mSerial.index > 0) { // Zpracovat prikaz, pokud neco prislo
        if (mSerial.buffer[mSerial.index-1] ==' ') mSerial.index--; // odstrani mezeru na konci pokud by tam byla
        mSerial.buffer[mSerial.index] = '\0'; // Ukoncovaci znak
        mSerial.isNewData = true;
        break; 
      }
    } else if (mSerial.index < MAX_COMMAND_LENGTH - 1) {
      // Prida jen znaky a ignorujeme whitespace
      if (mSerial.index > 0 || mSerial.inChar != ' ') {
        if (mSerial.buffer[mSerial.index-1] !=' ' || mSerial.inChar != ' '){
          
          //mSerial.inChar = tolower(mSerial.inChar);
          mSerial.buffer[mSerial.index++] = tolower(mSerial.inChar);
        } 
      }
    }
  } //while (Serial.available() > 0) 

  if (mSerial.isNewData) {
    mSerial.cmdPtr = mSerial.buffer;
    mSerial.state = 0; 
    while (mSerial.state!=255){
      if (strcmp(mSerial.cmdPtr, "-h") == 0) {
        tiskniProgmem(TEXT_T);
        tiskniProgmem(TEXT_I);
        tiskniProgmem(TEXT_P);
        tiskniProgmem(TEXT_S);
        tiskniProgmem(TEXT_E);
        tiskniProgmem(TEXT_TE);
        tiskniProgmem(TEXT_BPS);
        tiskniProgmem(TEXT_L);
        tiskniProgmem(TEXT_DS);
        tiskniProgmem(TEXT_CS);
        tiskniProgmem(TEXT_HIDN);
        tiskniProgmem(TEXT_FETC);
        tiskniProgmem(TEXT_PWID);
        tiskniProgmem(TEXT_PER);
        Serial.print(reinterpret_cast<const __FlashStringHelper *>(TEXT_EXAMPLE));
        mSerial.state = 255; //ukonci dekodovani mSerial.bufferu
      } 
      if (strncmp(mSerial.cmdPtr, "*idn?", 5) == 0){
        //*IDN?
        Serial.print(reinterpret_cast<const __FlashStringHelper *>(TEXT_IDN)); Serial.print(F(" Version: ")); Serial.println(aktualniNastaveni.verze);
        mSerial.state = 255; //ukonci dekodovani mSerial.bufferu
      }
      if (strncmp(mSerial.cmdPtr, "*rst", 4) == 0){
        //*RST
        Serial.print(reinterpret_cast<const __FlashStringHelper *>(TEXT_RST)); Serial.print(F(" Version: ")); Serial.println(aktualniNastaveni.verze);
        tovarniNastaveni();
        zobrazNastaveni();
        mSerial.state = 255; //ukonci dekodovani mSerial.bufferu
      }
      if (strncmp(mSerial.cmdPtr, ":fetch?", 7) == 0 ){
        //:FETCh?
        if(mSerial.state==2 || mSerial.state==3){ Serial.print(aktualniNastaveni.columnsSeparator);Serial.print(' ');}
        Serial.print(mereni.dutyCyclePromile);
        if (strlen(mSerial.cmdPtr) > 8) { mSerial.cmdPtr += 8; mSerial.state = 2;}
        else mSerial.state = 4;
      }
      if (strncmp(mSerial.cmdPtr, ":fetc?", 6) == 0 ){
        //:FETCh?
        if(mSerial.state==2 || mSerial.state==3){ Serial.print(aktualniNastaveni.columnsSeparator);Serial.print(' ');}
        Serial.print(mereni.dutyCyclePromile);
        if (strlen(mSerial.cmdPtr) > 7) { mSerial.cmdPtr += 7; mSerial.state = 2;}
        else mSerial.state = 4;
      }
      if (strncmp(mSerial.cmdPtr, ":measure:pwidth?",16) == 0 ){
        //:MEASure:PWIDth?
        if(mSerial.state==2 || mSerial.state==3){Serial.print(aktualniNastaveni.columnsSeparator);Serial.print(' ');}
        tiskniFloat(mereni.pulseLength_us / 1000000.0,6,aktualniNastaveni.decimalSeparator);
        if (strlen(mSerial.cmdPtr) > 17) {mSerial.cmdPtr += 17; mSerial.state = 2;}
        else mSerial.state = 4;
      }
      if (strncmp(mSerial.cmdPtr, ":meas:pwid?", 11) == 0 ){
        //:MEASure:PWIDth?
        if(mSerial.state==2 || mSerial.state==3){Serial.print(aktualniNastaveni.columnsSeparator);Serial.print(' ');}
        tiskniFloat(mereni.pulseLength_us / 1000000.0,6,aktualniNastaveni.decimalSeparator);
        if (strlen(mSerial.cmdPtr) > 12) {mSerial.cmdPtr += 12; mSerial.state = 2;}
        else mSerial.state = 4;
      }
      if (strncmp(mSerial.cmdPtr, ":measure:period?", 16) == 0 ){
        //MEASure:PERiod?
        if(mSerial.state==2 || mSerial.state==3){Serial.print(aktualniNastaveni.columnsSeparator);Serial.print(' ');}
        tiskniFloat(mereni.period_us / 1000000.0,6,aktualniNastaveni.decimalSeparator);
        if (strlen(mSerial.cmdPtr) > 17) {mSerial.cmdPtr += 17; mSerial.state = 2;}
        else mSerial.state = 4;
      }
      if (strncmp(mSerial.cmdPtr, ":meas:per?", 10) == 0 ){
        //MEASure:PERiod?
        if(mSerial.state==2 || mSerial.state==3){Serial.print(aktualniNastaveni.columnsSeparator);Serial.print(' ');}
        tiskniFloat(mereni.period_us / 1000000.0,6,aktualniNastaveni.decimalSeparator);
        if (strlen(mSerial.cmdPtr) > 11) {mSerial.cmdPtr += 11; mSerial.state = 2;}
        else mSerial.state = 4;
      }
      if (strncmp(mSerial.cmdPtr, "-te", 3) == 0) {
        if (! zpracujUniverzalniPrikaz(mSerial.cmdPtr, 100, 65000, nullptr, nullptr, &aktualniNastaveni.t_error,  nullptr)){
          tiskniProgmem(TEXT_TE);
        }; mSerial.state = 255; //ukonci dekodovani mSerial.bufferu
      } 
      if (strncmp(mSerial.cmdPtr, "-t ", 3) == 0) {
        if (! zpracujUniverzalniPrikaz(mSerial.cmdPtr, 1, 99, &aktualniNastaveni.spodni_hranice, &aktualniNastaveni.horni_hranice)){
          tiskniProgmem(TEXT_T);
        }; mSerial.state = 255; //ukonci dekodovani mSerial.bufferu
      } 
      if (strncmp(mSerial.cmdPtr, "-i", 2) == 0) {
        if (! zpracujUniverzalniPrikaz(mSerial.cmdPtr, 0, 1, &aktualniNastaveni.input, nullptr, nullptr, nullptr)){
          tiskniProgmem(TEXT_I);
        }; mSerial.state = 255; //ukonci dekodovani mSerial.bufferu
      } 
      if (strncmp(mSerial.cmdPtr, "-l", 2) == 0) {
        if (! zpracujUniverzalniPrikaz(mSerial.cmdPtr, 0, 1, &aktualniNastaveni.listing, nullptr, nullptr, nullptr)){
          tiskniProgmem(TEXT_L);
        }; mSerial.state = 255; //ukonci dekodovani mSerial.bufferu
      }
      if (strncmp(mSerial.cmdPtr, "-p", 2) == 0) {
        if (! zpracujUniverzalniPrikaz(mSerial.cmdPtr, 100, 65000, nullptr, nullptr, &aktualniNastaveni.min_perioda, &aktualniNastaveni.max_perioda)){
          tiskniProgmem(TEXT_P);
        }; mSerial.state = 255; //ukonci dekodovani mSerial.bufferu
      } 
      if (strncmp(mSerial.cmdPtr, "-s", 2) == 0) {
        if (! zpracujUniverzalniPrikaz(mSerial.cmdPtr, 1, 499, nullptr, nullptr, &aktualniNastaveni.min_strida, &aktualniNastaveni.max_strida)){
          tiskniProgmem(TEXT_S);
        }; mSerial.state = 255; //ukonci dekodovani mSerial.bufferu
      } 
      if (strncmp(mSerial.cmdPtr, "-e", 2) == 0) {
        if (! zpracujUniverzalniPrikaz(mSerial.cmdPtr, 0, 255, &aktualniNastaveni.max_error, nullptr, nullptr, nullptr)){
          tiskniProgmem(TEXT_E);
        }; mSerial.state = 255; //ukonci dekodovani mSerial.bufferu
      } 
      if (strncmp(mSerial.cmdPtr, "-b", 2) == 0) {
        if (! zpracujUniverzalniPrikaz(mSerial.cmdPtr, 96, 1152, nullptr, nullptr, &aktualniNastaveni.bps,  nullptr)){
          tiskniProgmem(TEXT_BPS);
        }; mSerial.state = 255; //ukonci dekodovani mSerial.bufferu
      } 
      if (strncmp(mSerial.cmdPtr, "-ds", 3) == 0) {
        if(strlen(mSerial.cmdPtr) == 5) {
          char znak = mSerial.cmdPtr[4];
          aktualniNastaveni.decimalSeparator=znak;
          EEPROM.put(EEPROM_ADRESA, aktualniNastaveni);
          Serial.println(F("\r\nLimits successfully updated:"));
          zobrazNastaveni();
          mSerial.state = 255; //ukonci dekodovani mSerial.bufferu
        }
      } 

      if (strncmp(mSerial.cmdPtr, "-cs", 3) == 0) {
        if(strlen(mSerial.cmdPtr) == 5) {
          char znak = mSerial.cmdPtr[4];
          aktualniNastaveni.columnsSeparator=znak;
          EEPROM.put(EEPROM_ADRESA, aktualniNastaveni);
          Serial.println(F("\r\nLimits successfully updated:"));
          zobrazNastaveni();
          mSerial.state = 255; //ukonci dekodovani mSerial.bufferu
        }
      }
      
      if (mSerial.state==4){
        // prida konec radku za posledni odmer
        Serial.print(F("\r\n"));
        mSerial.state=255; //ukonci dekodovani mSerial.bufferu
      }
      if (mSerial.state==0 || mSerial.state==3 ){
        Serial.print(F("\r\nUnknown command or wrong format: "));
        Serial.println(mSerial.cmdPtr);
        tiskniProgmem(TEXT_H);
        mSerial.state=255; //ukonci dekodovani mSerial.bufferu
      }
      if (mSerial.state == 2) mSerial.state=3;

    } //while (mSerial.state!=255)
   
    mSerial.index = 0; // Reset index pro dalsi prikaz
    mSerial.isNewData = false;
  } //if (mSerial.isNewData)

  #ifdef DEBUG_MODE
    delay(100);
  #endif
}
