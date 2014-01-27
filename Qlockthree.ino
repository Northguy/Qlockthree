#pragma once
/**
 * Clockthree.pde (.ino)
 * Die Firmware der Selbstbau-QLOCKTWO.
 *
 * @mc       Arduino/RBBB (ATMEGA328)
 * @autor    Christian Aschoff / caschoff _AT_ mac _DOT_ com
 * @version  3.2.2
 * @created  1.11.2011
 * @updated  6.1.2014
 *
 * Versionshistorie:
 * V 1.1:   - DCF77 auf reine Zeit ohne Strings umgestellt.
 *          - Ganzes Datum wird in die DS1307 geschrieben (wieder verworfen).
 * V 1.2:   - Schnelles Schreiben der Shift-Register eingefuehrt.
 * V 1.3:   - Auf eigene DCF77-Bibliothek umgestellt (MyDCF77) und DCF77Helper verbessert.
 * V 1.4:   - Glimmen/Nachleuchten dank Tipp von Volker beseitigt (http://www.mikrocontroller.net/topic/209988).
 *          - Erneute Fehlerbereinigung in der DCF77-Bibliothek.
 * V 1.5:   - Die Helligkeit laesst sich ueber die Variable 'brightness' einstellen (Bereich 1-9) bzw.
 *            ueber den Compiler-Schalter 'ENABLE_LDR' die Regelung einschalten.
 * V 1.6:   - Fehler in der DCF77Helper-Bibliothek behoben.
 * V 1.7:   - SKIP_BLANK_LINES eingefuehrt, damit hellere LEDs (Danke an Johannes).
 * V 1.8:   - UPSIDE_DOWN eingefuehrt, damit man die Kabel beim Anoden-Multiplexer nicht kreuzen muss.
 * V 1.9:   - Diverse Verbesserungen und Fehlerbehebungen, die das Flackern drastisch reduzieren.
 * V 1.9.1: - Fehler behoben, der die Nutzung eines analogen Pins fuer den DCF77-Empfaenger verhindert hat.
 * V 1.9.2: - Timing-Parameter fuer die PWM ueber define anpassbar.
 *          - LDR-Grenzwerte automatisch anpassbar.
 *          - Schreibfehler in setHoures()->setHours() behoben.
 *          - Kompatibilitaet zu Arduino-IDE 1.0 hergestellt.
 * V 1.9.3: - Glimmen wieder behoben.
 * V 1.9.4: - Modus zum Einstellen der Helligkeit eingefuehrt.
 * V 2.0:   - Sprachrelevante Dinge ausgelagert, so kann man mehr Sprachen haben und einfach durch einkommentieren aktivieren.
 *          - setWords in setMinutes und setMinutes in setCorners umbenannt, weil es mehr Sinn ergibt.
 *          - setCorners in eigene Datei ausgelagert, weil viele Bastler sich vertun und in der Routine Aenderungen vornehmen muessen.
 *          - LDR in eigene Klasse ausgelagert und Werte geglaettet. Am Anfang werden 1000 Werte gelesen, damit er sich einpegelt.
 *          - Signal vom DCF77-Empfaenger geglaettet, damit nicht ein einzelner falscher Peak das Telegramm zerstoert. 
 *          - Status-LEDs koennen ueber DEFINEs ausgeschaltet werden.
 *          - DCF77- und SQW-LED blinken am Anfang drei Mal als 'Hello', damit man ohne Serial-Monitor sehen kann, ob der ATMEGA anlaeuft.
 *          - Serial-Output ist immer an, kleine Statusmeldung waehrend der Initialisierung, damit man beim Bespielen des ATMEGA ohne weitere 
 *            Elektronik sehen kann, ob das Programm drauf ist und laeuft.
 *          - Aenderung, die ein Stellen der Zeit nach dem DCF77-Signal auch im Modus 'BLANK' ermoeglicht - in diesem Modus ist der Empfang
 *            weniger gestoert, da die LED-Matrix abgeschaltet ist.
 *          - Zeitgesteuertes Abschalten des Displays eingefuehrt (Stromsparen in der Nacht/Schlafzimmer/besserer Empfang des DCF77-Empfaengers).
 *          - Speaker auf D13 eingefuehrt.
 *          - Alarm-Mode eingefuehrt.
 *          - Bayrische Sprachvariante (DE_BA): 'viertel nach Zehn', aber 'dreiviertel Elf'.
 * V 2.0.1: - DCF77-Tresholds angepasst.
 * V 2.0.2: - Strings in PROGMEM (flash) ausgelagert.
 * V 2.0.3: - SPEAKER_IS_BUZZER eingefuehrt, damit kann man sagen, ob man einen Lautsprecher oder Buzzer als Alarm verwendet.
 * V 2.0.4: - falsches BREAK dank Lars behoben.
 * V 2.0.5: - Franzoesisch hinzugefuegt, Woerter_??.h's anschaulicher gemacht. Weitere Sprachen angelegt, aber noch nicht die Logik implementiert (Hilfe?!).
 * V 2.0.6: - cleanWordsForAlarmSettingMode() eingefuehrt, damit das Stellen des Alarms sprachenunabhaengig ist.
 *          - Das DCF77-Signal kann per Compiler-Schalter invertiert werden.
 * V 2.0.7: - Neuer Modus: Umschaltung LDR auto/manuell dank Alexander.
 * V 2.0.8: - Genauigkeit verbessert, dank Peter (Interrupt auf FALLING). @Peter: Das Zurueckscheiben in die DS1307 passiert im Normalbetrieb ja nur beim Update
 *            ueber den DCF77-Empfaenger, daher habe ich es weggelassen.
 * V 2.0.9: - SplitSideDown rausgenommen, diesen seltenen Fehler braucht kein Mensch.
 * V 2.1:   - Neue Sprachen hinzugefuegt: Englisch und Italiensch. Spanisch ist angelegt, aber nicht implementiert, weil ich kein Wort Spanisch spreche.
 *            Man kann jetzt einfach relativ weit oben im Code ueber Compiler-Schalter festlegen, welche Sprache man haben moechte.
 * V 2.1.1: - Bitmaps in den PROGMEM ausgelagert. So kann man alle Buchstaben halten.
 * V 2.1.2: - Bug im Alaram-Modus behoben.
 * V 2.1.3: - Zusaetzlich zu den Ziffern auch Staben eingefuehrt, A/M-Modus darauf umgestellt (Vorbereitung auf Konfigurationsmoeglichkeit fuer alle Sprachen in einer Firmware)
 * V 2.1.4: - Optimierung hinsichtlich Speicherbedarf.
 * V 3.0:   - LedDriver-Klasse erstellt, welche die Shift-Register und UDN2981A als Led-Treiber kapseln. Dadurch kann man auch andere LED-Treiber nehmen (MAX7219, AS1130 etc.) und einfach
 *            eine passende Klasse von der LedDriver-Klasse vererben und in den Code einklinken.
 *          - Alle Sprachen im 'Renderer' zusammengefasst, so dass sie ohne die Firmware neu zu flashen umschaltbar werden.
 *          - Niederlaendisch hinzugefuegt (Danke an Wekog24 und Rudolf K.).
 *          - Eine kleine Debug-Klasse eingefuehrt, damit die #debug-Statements nicht so stark den Code zerhacken und er besser lesbar wird.
 *          - Beim Starten wird der freie Speicher und die Version auf der Console ausgegeben.
 *          - Eine Alarm-Klasse eingefuehrt, das macht den 'Hauptcode' nochmal besser lesbar.
 *          - Configuration.h eingefueht, in der alle wichtigen Compiler-Schalter zusammengefuehrt sind, dadurch gibt es eine zentrale Stelle fuer die Anpassungen (Danke fuer das Konzept an Markus K.). 
 *          - Settings.h eingefuehrt, in der die Spracheinstellung gehalten wird und im EEPROM gespeichert wird (Danke fuer das Konzept an Markus K.). 
 *          - Modes zum Einstellen der Sprache und des Eck-Modus eingefuehrt.
 *          - Extended Modes eingefuehrt.
 *          - Saechsisch als Sprache aufgenommen, danke an Sven.
 *          - Bug im setzten der Eck-LEDs fuer den Ueberpixel behoben.
 *          - Brightness ist normaler Mode, aber nur wenn ldr_auto == FALSE.
 *          - Automatische Alarm-Abschaltung nach MAX_BUZZ_TIME_IN_MINUTES.
 *          - Stellen der Uhr nur im erweiterten Modus moeglich.
 *          - LDR-Modus wird im EEPROM abgelegt.
 *          - kleinere Koorekturen, die nur in erweiterten Compiler-Ausgaben zu Warnings gefuehrt haben.
 *          - Kleinere Optimierungen, Danke an Michael K.
 *          - Bug aufgrund eines moeglichen Ueberlaufs von millis() in Button.cpp behoben.
 *          - Verbesserungen in der Helligkeitssteuerung.
 *          - ENABLE_LDR aus Configuration.h entfernt, da sich der LDR ueber das erweiterte Setup ausschalten laesst.
 * V 3.0.1: - Schweizer Layout angepasst.
 * V 3.1:   - Spanisch implementiert.
 *          - Der LDR skaliert jetzt selbst, dann flackert es weniger bei unguenstigen Lichtverhaeltnissen.
 * V 3.1.1. - Die Displayhelligkeit wird beim starten gesetzt.
 *          - Kleiner Fehler im Spanischen behoben.
 *          - Die alte Shiftregistermothode ist wieder aktiv, die schnelle kann in Configuration.h gesetzt werden.
 * V 3.1.2. - Led-Driver-Klassen sagen, wer sie sind (fuer die Ausgabe der Konfiguration beim Start).
 *          - Klassen-Namen verbessert.
 * V 3.2.   - Led-Driver fuer das Licht-Monster implementiert.
 *          - LDR-Klasse auf 0-100% umgebaut.
 *          - Brightness wird im EEPROM gesichert.
 *          - EnableAlarm und DCF77SignalIsInverted ins EEPROM ausgelagert und ueber das erweiterte Setup einstellbar.
 *          - Die Modi SCRAMBLE und ALL entfernt, dafuer den Modus TEST eingefuehrt. ALL ist fuer manche DisplayDriver gefaehrlich wegen des Stromverbrauchs. TEST schalte nacheinander alle LEDs einmal durch.
 *          - Moeglichkeit zur Zeitverschiebung eingefuehrt.
 * V 3.2.1. - Alle Deutsch-Varianten zusammengefasst, um Platz zu sparen.
 *          - Fehler im Italienischen behoben.
 *          - Fehler mit Doppelbelegung einer Variable im Qlockthree.ino behoben.
 *          - Der LDR wird nur alle 250ms gelesen, dann flackert er weniger bei unguenstigen Verhaeltnissen.
 * V 3.2.2. - DS1307 Multi-MCU-Faehig gemacht.
 *          - Initialisierung der DS1307 verbessert.
 * V 3.2.A  - Replaced DCF77 module with generic library
			- Replaced DS1307 module with generic library
			- Added Low Pass Filter routine for error filtering on DCF77 signal
			- Added running average routine on LDR in stead of doing a loop calculation once in a while
			  TODO:	* Code only usable for standard LED driver
					* Signal Inverted does not work yet
					* Alarm routine needs to be fixed
 */

#include <Wire.h> // Wire library fuer I2C
#include <avr/pgmspace.h>
#include <EEPROM.h>
#include "Configuration.h"
#include "LedDriver.h"
#include "LedDriverDefault.h"
#include "LedDriverUeberPixel.h"
#include "LedDriverPowerShiftRegister.h"
#include "Button.h"
#include "LDR.h"
#include "Renderer.h"
#include "Staben.h"
#include "Alarm.h"
#include "Settings.h"
#include "Zahlen.h"

/************************************************************************/
/* The following three #Includes use libraries as found in the public domain
/* The libraries are used for the DCF77 implementation that is used in this file.
/* The DCF77 implementation below is based on the work of Udo Klein
/* See http://blog.blinkenlight.net/experiments/dcf77/binary-clock/                                                                      
/************************************************************************/

#include <MsTimer2.h>	// Library downloaded from: http://www.pjrc.com/teensy/td_libs_MsTimer2.html. See also http://playground.arduino.cc/Main/MsTimer2
#include <DCF77.h>		// Library downloaded from: http://thijs.elenbaas.net/downloads/?did=1. See also http://playground.arduino.cc/Code/DCF77
#include <Time.h>		// Library downloaded from: http://www.pjrc.com/teensy/td_libs_Time.html. See also http://playground.arduino.cc/Code/time
#include <DS1307RTC.h>

#define FIRMWARE_VERSION "V 3.2.A vom 27.1.2014 Code Adapted by Northguy"  

/*
 * Den DEBUG-Schalter gibt es in allen Bibiliotheken. Wird er eingeschaltet, werden ueber den
 * 'Serial-Monitor' der Arduino-Umgebung Informationen ausgegeben. Die Geschwindigkeit im
 * Serial-Monitor muss mit der hier angegeben uebereinstimmen.
 * Default: ausgeschaltet
 */
   #define DEBUG
   #include "Debug.h"
// Die Geschwindigkeit der seriellen Schnittstelle. Default: 57600. Die Geschwindigkeit brauchen wir immer,
// da auch ohne DEBUG Meldungen ausgegeben werden!
   #define SERIAL_SPEED 57600
  
/*
 * Die persistenten (im EEPROM gespeicherten) Einstellungen.
 */
Settings settings;

/**
 * Hier definiert man die Ab- und Anschaltzeiten fuer das Display. Die Abschaltung des
 * Displays verbessert den Empfang des DCF77-Empfaengers. Und hilft, falls die Uhr im 
 * Schlafzimmer haengt.
 * Sind alle Werte auf 0 wird das Display nie abgeschaltet. Nach einer Minute kann man das 
 * Display manuell wieder ein- / ausschalten.
 * Achtung! Wenn sich die Uhr nachmittags abschaltet ist sie in der falschen Tageshaelfte!
 */

/*
typedef struct  {
	uint8_t Second;
	uint8_t Minute;
	uint8_t Hour;
	uint8_t Wday;   // day of week, sunday is day 1
	uint8_t Day;
	uint8_t Month;
	uint8_t Year;   // offset from 1970;
} 	tmElements_t, TimeElements, *tmElementsPtr_t;

*/
// um 3 Uhr Display abschalten (sec, min, hr, -, -, -)
TimeElements offTime = {0, 0, 3, 0, 0, 0};

// um 4:30 Uhr Display wieder anschalten (sec, min, hr, -, -, -)
TimeElements onTime = {0, 30, 4, 0, 0, 0};

/**
 * Der Renderer, der die Woerter auf die Matrix ausgibt.
 */
Renderer renderer;

/**
 * Der LED-Treiber fuer 74HC595-Shift-Register. Verwendet
 * von der Drei-Lochraster-Platinen-Version und dem
 * NachBau_V02-Board sowie dem WortWecker nach Christian.
 *
 * Data: 10; Clock: 12; Latch: 11; OutputEnable: 3
 * LinesToWrite: 10
 */
#ifdef LED_DRIVER_DEFAULT
 #define PIN_FILTERED_DCF 2           // 1        Filtered DCF signal  
 #define PIN_SQW_SIGNAL 2             // 2        SQW Signal  
 #define PIN_MATRIX_OUTPUT_ENABLE 3   // 3        Matrix OutputEnable  
 #define PIN_SQW_LED 4                // 4        Clock Frequency LED (Green)  
 #define PIN_M_PLUS 5                 // 5        M plus Switch  
 #define PIN_H_PLUS 6                 // 6        H plus Switch  
 #define PIN_MODE 7                   // 7        Mode Switch  
 #define PIN_DCF77_LED 8              // 8        DCF77 signal LED (yellow)  
 #define PIN_DCF77_SIGNAL 9           // 9        DCF77 digital signal  
 #define PIN_MATRIX_DATA 10           // 10        Data pin for LED matrix  
 #define PIN_MATRIX_Latch 11          // 11        Latch for Shift registers  
 #define PIN_MATRIX_Clock 12          // 12        Clock signal for LED matrix  
 #define PIN_SPEAKER 13               // 13        Speaker pin    
 #define PIN_LDR A3                   // A3        Light intensity sensor (LDR)
 
 /** * The LED Matrix driver */     
 
 LedDriverDefault ledDriver(PIN_MATRIX_DATA, PIN_MATRIX_Clock, PIN_MATRIX_Latch, PIN_MATRIX_OUTPUT_ENABLE, PIN_MATRIX_DATA);  
 
 
#endif  
  
/**
 * Der LED-Treiber fuer 4 MAX7219-Treiber wie im Ueberpixel.
 * Data: 10; Clock: 11; Load: 12
 */



#ifdef LED_DRIVER_UEBERPIXEL
  LedDriverUeberPixel ledDriver(5, 6, 7);

  #define PIN_MODE 8
  #define PIN_M_PLUS 3
  #define PIN_H_PLUS 4
  
  #define PIN_LDR A3
  
  #define PIN_SQW_SIGNAL 2
  #define PIN_DCF77_SIGNAL 9
  
  #define PIN_SQW_LED 10
  #define PIN_DCF77_LED 11

  #define PIN_SPEAKER 13
#endif

/**
 * Der LED-Treiber fuer Power-Shift-Register.
 * Data: 10; Clock: 11; Load: 12

*/
 
#ifdef LED_DRIVER_POWER_SHIFT_REGISTER
  LedDriverPowerShiftRegister ledDriver(10, 12, 11, 3);

  #define PIN_MODE 7
  #define PIN_M_PLUS 5
  #define PIN_H_PLUS 6
  
  #define PIN_LDR A3
  
  #define PIN_SQW_SIGNAL 2
  #define PIN_DCF77_SIGNAL 9
  
  #define PIN_SQW_LED -1
  #define PIN_DCF77_LED -1

  #define PIN_SPEAKER -1
#endif


/**
 * Die Real-Time-Clock
 */
DS1307RTC ds1307;
byte helperSeconds;

/**
 * Der Funkempfaenger (DCF77-Signal der PTB Braunschweig).
 */

DCF77 DCF = DCF77(PIN_FILTERED_DCF, 0);

volatile boolean sync = false;
volatile time_t time = 0;


/**
 * Variablen fuer den Alarm.
 */
Alarm alarm(PIN_SPEAKER);

/**
 * Der Helligkeitssensor
 */
LDR ldr(PIN_LDR);
unsigned long lastBrightnessCheck;

/**
 * Die Helligkeit zum Anzeigen mit den Balken.
 */
byte brightnessToDisplay;

/**
 * Die Tasten.
 */
Button minutesPlusButton(PIN_M_PLUS);
Button hoursPlusButton(PIN_H_PLUS);
Button extModeDoubleButton(PIN_M_PLUS, PIN_H_PLUS);
Button modeChangeButton(PIN_MODE);

/**
 * Die Standard-Modi.
 */
#define STD_MODE_NORMAL     0
#define STD_MODE_ALARM      1
#define STD_MODE_SECONDS    2
#define STD_MODE_BRIGHTNESS 3
#define STD_MODE_BLANK      4
#define STD_MODE_COUNT      4

/**
 * Die erweiterten Modi.
 */
#define EXT_MODE_START           10
#define EXT_MODE_LDR_MODE        10
#define EXT_MODE_CORNERS         11
#define EXT_MODE_ENABLE_ALARM    12
#define EXT_MODE_DCF_IS_INVERTED 13
#define EXT_MODE_LANGUAGE        14
#define EXT_MODE_TIMESET         15
#define EXT_MODE_TIME_SHIFT      16
#define EXT_MODE_TEST            17
#define EXT_MODE_COUNT           17

// Startmode...
byte mode = STD_MODE_NORMAL;
// Merker fuer den Modus vor der Abschaltung...
byte lastMode = mode;

// Ueber die Wire-Library festgelegt:
// Arduino analog input 4 = I2C SDA
// Arduino analog input 5 = I2C SCL

// Die Matrix, eine Art Bildschirmspeicher
// word = 16 bit unsigned integer
// therefore matrix = 16 x 16 bits

word matrix[16];

// Structure for storing RTCtime
  TimeElements RTCtime;

// Hilfsvariable, da I2C und Interrupts nicht zusammenspielen
volatile boolean needsUpdateFromRtc = true;

// Fuer den Bildschirm-Test
byte x, y;

/**
 * Aenderung der Anzeige als Funktion fuer den Interrupt, der ueber das SQW-Signal 
 * der Real-Time-Clock gesetzt wird. Da die Wire-Bibliothek benutzt wird, kann man
 * den Interrupt nicht direkt benutzen, sondern muss eine Hilfsvariable setzen, die
 * dann in loop() ausgewertet wird.
 */
void updateFromRtc() {
  needsUpdateFromRtc = true;
}

/**
 * Den freien Specher abschaetzen. 
 * Kopiert von: http://playground.arduino.cc/Code/AvailableMemory
 */
int freeRam() {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

/** 
* Low pass filter for DCF77 signal filtering. Code copied from routine of Udo Klein
**/

void low_pass_filter() {
	// http://en.wikipedia.org/wiki/Low-pass_filter#Continuous-time_low-pass_filters
	
	// I will use fixed point arithmetics with 5 decimals
	const uint16_t decimal_offset = 10000;
	static uint32_t smoothed = 0*decimal_offset;
	
	const uint32_t input = digitalRead(PIN_DCF77_SIGNAL) * decimal_offset;
	//const uint32_t input = analogRead(dcf77_analog_sample_pin)>200? decimal_offset: 0;
	
	// compute N such that the smoothed signal will always reach 50% of
	// the input after at most 50 samples (=50ms).
	// N = 1 / (1- 2^-(1/50)) = 72.635907286
	const uint16_t N = 72;
	smoothed = ((N-1) * smoothed + input) / N;
	
	// introduce some hysteresis
	static uint8_t square_wave_output = 0;
	
	if ((square_wave_output == 0) == (smoothed >= decimal_offset/2)) {
		// smoothed value more >= 50% away from output
		// ==> switch output
		square_wave_output = 1-square_wave_output;
		// ==> max the smoothed value in order to introduce some
		//     hysteresis, this also ensures that there is no
		//     "infinite memory"
		smoothed = square_wave_output? decimal_offset: 0;
	}
	
	digitalWrite(PIN_FILTERED_DCF, square_wave_output);
}

/** 
* Function to derive DCF Time. From code Udo Klein 
**/

unsigned long getDCFTime() {
	const time_t DCFTime = DCF.getTime();
	
	sync = (DCFTime != 0);
	return DCFTime;
}

/**
* Diagnosis Function to evaluate DCF signal.
* The loop program will not start until the DCF signal has been obtained                                                                     
**/

void diagnosis() {
	Serial.println("Waiting for DCF77 time ... ");
	
	int rising_edge = 0;
	int falling_edge = 0;
	int previous_rising_edge;
	bool was_high = false;
	
	while (timeStatus() == timeNotSet) {
		const uint8_t sensor_value = digitalRead(PIN_FILTERED_DCF);
		if (sensor_value) {
			if (!was_high) {
				rising_edge = millis();
				was_high = true;
			}
			} else {
			if (was_high) {
				falling_edge = millis();
				DEBUG_PRINT("Cycle, Pulse: ");
				
				const int cycle = rising_edge - previous_rising_edge;
				if (cycle < 1000) {
					DEBUG_PRINT(' ');
				}
				DEBUG_PRINT(cycle);
				DEBUG_PRINT(',');
				DEBUG_PRINT(' ');
				
				const int pulse = falling_edge - rising_edge;
				if (pulse < 100) {
					DEBUG_PRINT(' ');
				}
				DEBUG_PRINT(pulse);
				previous_rising_edge = rising_edge;
				was_high = false;
				
				DEBUG_PRINT(' ');
				DEBUG_PRINT(pulse < 180? '.': 'X');
				
				DEBUG_PRINT(' ');
				DEBUG_PRINT(cycle <1800? ' ': 'm');
				
				DEBUG_PRINTLN();
			}
		}
	}
	
	Serial.println();
	Serial.println("Exiting DCF77 diagnosis");
}


/**
 * Initialisierung. setup() wird einmal zu Beginn aufgerufen, wenn der
 * Arduino Strom bekommt.
 */
void setup() {
  Serial.begin(SERIAL_SPEED);
  Serial.println(F("Qlockthree is initializing..."));
  DEBUG_PRINTLN(F("... and starting in debug-mode..."));
  Serial.flush();

  if(settings.getEnableAlarm()) {
    // als Wecker Display nicht abschalten...
    TimeElements offTime = {0, 0, 0, 0, 0, 0};
    TimeElements onTime  = {0, 0, 0, 0, 0, 0};
  }
  
  // LED-Treiber initialisieren
  ledDriver.init();
  // Inhalt des Led-Treibers loeschen...
  ledDriver.clearData();
  // und Inhalt des Bildspeichers loeschen
  renderer.clearScreenBuffer(matrix);
  
  // DS1307-Interrupt-Pin konfigurieren
  pinMode(PIN_SQW_SIGNAL, INPUT);
  digitalWrite(PIN_SQW_SIGNAL, HIGH);
  
  // DCF77-LED drei Mal als 'Hello' blinken lassen
  // und Speaker piepsen lassen, falls ENABLE_ALARM eingeschaltet ist.
  for(byte i=0; i<3; i++) {
    statusLed(PIN_DCF77_LED,HIGH);
    if(settings.getEnableAlarm()) {
      alarm.buzz(true);
    }
    delay(100);
    statusLed(PIN_DCF77_LED,LOW);
    if(settings.getEnableAlarm()) {
      alarm.buzz(false);
    }
    delay(100);
  }

  // 1 Hz-SQW-Signal auf der DS1307 einschalten
  
  ds1307.read(RTCtime); 
   
  if((RTCtime.Second >= 60)||(RTCtime.Minute >= 60)||(RTCtime.Hour >= 24)) {
    // Echtzeituhr enthaelt Schrott, neu beschreiben...
    RTCtime.Hour	= 12;
	RTCtime.Minute  = 0;
	RTCtime.Second	= 0;
	RTCtime.Year	= 2014;
	RTCtime.Month	= 1;
	RTCtime.Day		= 1;
	RTCtime.Wday	= 1;
  }
  ds1307.write(RTCtime);
 
  DEBUG_PRINT(F("Time: "));
  DEBUG_PRINT(RTCtime.Hour);
  DEBUG_PRINT(F(":"));
  DEBUG_PRINT(RTCtime.Minute);
  DEBUG_PRINT(F(":"));
  DEBUG_PRINTLN(RTCtime.Second);
  DEBUG_FLUSH();

/**
   Den Interrupt konfigurieren,
   nicht mehr CHANGE, das sind 2 pro Sekunde,
   RISING ist einer pro Sekunde, das reicht.
   Auf FALLING geandert, das signalisiert
   den Sekundenwechsel, Danke an Peter.
   
   ATMega 328 can handle 2 external interrupts.
   Interrupt 0 on pin 2
   Interrupt 1 on pin 3
   http://arduino.cc/en/Reference/attachInterrupt
*/

  attachInterrupt(0, updateFromRtc, FALLING);

  // Werte vom LDR einlesen und vermuellen, da die ersten nichts taugen...
  for(int i=0; i<1000; i++) {
    ldr.value();
  }

  // rtcSQWLed-LED drei Mal als 'Hello' blinken lassen
  // und Speaker piepsen kassen, falls ENABLE_ALARM eingeschaltet ist.
  for(byte i=0; i<3; i++) {
    statusLed(PIN_SQW_LED,true);
    if(settings.getEnableAlarm()) {
      alarm.buzz(true);
    }
    delay(100);
    statusLed(PIN_SQW_LED,false);
    if(settings.getEnableAlarm()) {
      alarm.buzz(false);
    }
    delay(100);
  }

  // ein paar Infos ausgeben
  Serial.println(F("... done and ready to rock!"));
  
  Serial.print(F("Version: "));
  Serial.println(F(FIRMWARE_VERSION));
  Serial.print(F("Driver: "));
  ledDriver.printSignature();
  if(settings.getEnableAlarm()) {
    Serial.println(F("Alarm is enabled"));
  }
  if(settings.getDcfSignalIsInverted()) {
    Serial.println(F("DCF77-Signal is inverted."));
  }

  Serial.print(F("Free ram: "));
  Serial.print(freeRam());
  Serial.println(F(" bytes."));
  
  Serial.flush();

  // Display einschalten...
  ledDriver.wakeUp();
  ledDriver.setBrightness(settings.getBrightness());
  
//Enable DCF77 data acquisition
  pinMode(PIN_DCF77_SIGNAL, INPUT);
  digitalWrite(PIN_DCF77_SIGNAL, HIGH);
      
  pinMode(PIN_FILTERED_DCF, OUTPUT);
     
  // Define timer with resolution of 1ms for low_pass_filter 
  MsTimer2::set(1, low_pass_filter);
  MsTimer2::start();
  
  /**
  * Start the DCF Clock.... this will start the DCF polling through the 
  * Diagnosis loop. This means that the loop() routine of the clock will
  * NOT been entered until a valid DCF fix is found. Lets find out if this 
  * is desirable. Otherwise disable the Diagnosis loop.
  *
  * Please note that in this situation the DCF directly updates the internal
  * clock and not the RTC....
  *
  * Sync interval of internal clock can be set with setSyncInterval(xx)
  *
  **/
  
  // Start the DCF device
  DCF.Start();
  // set number of seconds between re-sync of internal clock
  setSyncInterval(30); 
  // time sync is obtained from DCF. 
  setSyncProvider(getDCFTime); 
  diagnosis();  
}

/**
 * loop() wird endlos auf alle Ewigkeit vom Microcontroller durchlaufen
 */
void loop() {
  //
  // Dimmung.
  //
  if (settings.getUseLdr()) {
    if(millis() < lastBrightnessCheck) {
      // wir hatten einen Ueberlauf...
      lastBrightnessCheck = millis();
    }    
    // Brightness is adjusted after 250 miliseconds
    // in LDR class a running average of LDR_MEAN_COUNT samples is taken (default = 32)
    if(lastBrightnessCheck + 250 < millis()) { // nur einmal pro Sekunde nachsehen...
      if(settings.getBrightness() != ldr.value()) {
        settings.setBrightness(ldr.value());
        DEBUG_PRINT(F("brightness: "));
        DEBUG_PRINTLN(settings.getBrightness());
        DEBUG_FLUSH();
        ledDriver.setBrightness(settings.getBrightness());
      }
      lastBrightnessCheck = millis();
    }  
  }

  //
  // needsUpdateFromRtc wird via Interrupt gesetzt ueber fallende 
  // Flanke des SQW-Signals von der RTC
  //
  if (needsUpdateFromRtc) {
    needsUpdateFromRtc = false;
    // die Zeit verursacht ein kurzes Flackern. Wir muessen
    // sie aber nicht immer lesen, im Modus 'normal' alle 60 Sekunden,
    // im Modus 'seconds' alle Sekunde, sonst garnicht.
    helperSeconds++;
    if(helperSeconds >= 60) {
      helperSeconds = 0;
    }
    
	
    //
    // Zeit einlesen...
    //
    switch(mode) {
      case STD_MODE_NORMAL:
      case EXT_MODE_TIMESET:
      case STD_MODE_ALARM:
        if(alarm.isActive()) {
          ds1307.read(RTCtime);
        }
        if(helperSeconds == 0) {
          ds1307.read(RTCtime);
          helperSeconds = RTCtime.Second;
        }
        break;
      case STD_MODE_SECONDS:
      case STD_MODE_BLANK:
        ds1307.read(RTCtime);
        helperSeconds = RTCtime.Second;
        break;
      // andere Modi egal...
    }

    //
    // Bildschirmpuffer beschreiben...
    //
    switch (mode) {
      case STD_MODE_NORMAL:
      case EXT_MODE_TIMESET:
        renderer.clearScreenBuffer(matrix);
        renderer.setMinutes(RTCtime.Hour + settings.getTimeShift(), RTCtime.Minute, settings.getLanguage(), matrix);
        renderer.setCorners(RTCtime.Minute, settings.getRenderCornersCw(), matrix);
      break;
      case EXT_MODE_TIME_SHIFT:
        renderer.clearScreenBuffer(matrix);
        if(settings.getTimeShift() < 0) {
          for(byte x=0; x<3; x++) {
            ledDriver.setPixelInScreenBuffer(x, 1, matrix);
          }
        } else if(settings.getTimeShift() > 0) {
          for(byte x=0; x<3; x++) {
            ledDriver.setPixelInScreenBuffer(x, 1, matrix);
          }
          for(byte y=0; y<3; y++) {
            ledDriver.setPixelInScreenBuffer(1, y, matrix);
          }
        }
        for (byte i = 0; i < 7; i++) {
          matrix[3 + i] |= pgm_read_byte_near(&(ziffern[abs(settings.getTimeShift())%10][i])) << 5;
          if(abs(settings.getTimeShift()) > 9) {
            matrix[3 + i] |= pgm_read_byte_near(&(ziffern[1][i])) << 10;
          }
        }
      break;
      case STD_MODE_ALARM:
        renderer.clearScreenBuffer(matrix);
        if(alarm.getShowAlarmTimeTimer() == 0) {
          renderer.setMinutes(RTCtime.Hour + settings.getTimeShift(), RTCtime.Minute, settings.getLanguage(), matrix);
          renderer.setCorners(RTCtime.Minute, settings.getRenderCornersCw(), matrix);
          matrix[4] |= 0b0000000000011111; // Alarm-LED
        } else {        
          renderer.setMinutes(alarm.getAlarmTime().Hour + settings.getTimeShift(), alarm.getAlarmTime().Minute, settings.getLanguage(), matrix);
          renderer.setCorners(alarm.getAlarmTime().Minute, settings.getRenderCornersCw(), matrix);
          renderer.cleanWordsForAlarmSettingMode(settings.getLanguage(), matrix); // ES IST weg
          if(alarm.getShowAlarmTimeTimer() % 2 == 0) {
            matrix[4] |= 0b0000000000011111; // Alarm-LED
          }
          alarm.decShowAlarmTimeTimer();
        }
      break;
      case STD_MODE_SECONDS:
        renderer.clearScreenBuffer(matrix);
        for (byte i = 0; i < 7; i++) {
          matrix[1 + i] |= pgm_read_byte_near(&(ziffern[RTCtime.Second/10][i])) << 11;
          matrix[1 + i] |= pgm_read_byte_near(&(ziffern[RTCtime.Second%10][i])) << 5;
        }
        break;
      case EXT_MODE_LDR_MODE:
        renderer.clearScreenBuffer(matrix);
        if (settings.getUseLdr()) {
          for (byte i = 0; i < 5; i++) {
            matrix[2 + i] |= pgm_read_byte_near(&(staben['A'-'A'][i])) << 8;
          }
        } else {
          for (byte i = 0; i < 5; i++) {
            matrix[2 + i] |= pgm_read_byte_near(&(staben['M'-'A'][i])) << 8;
          }
        }
        break;
      case STD_MODE_BLANK:
        renderer.clearScreenBuffer(matrix);
        break;
      case STD_MODE_BRIGHTNESS:
        renderer.clearScreenBuffer(matrix);
        brightnessToDisplay = map(settings.getBrightness(), 1, 100, 0, 9);
        for(byte xb = 0; xb < brightnessToDisplay; xb++) {
          for(byte yb = 0; yb <= xb; yb++) {
            matrix[9 - yb] |= 1 << (14 - xb);
          }
        }
      break;
      case EXT_MODE_CORNERS:
        renderer.clearScreenBuffer(matrix);
        if (settings.getRenderCornersCw()) {
          for (byte i = 0; i < 5; i++) {
            matrix[2 + i] |= pgm_read_byte_near(&(staben['C'-'A'][i])) << 11;
            matrix[2 + i] |= pgm_read_byte_near(&(staben['W'-'A'][i])) << 5;
          }
        } else {
          for (byte i = 0; i < 5; i++) {
            matrix[0 + i] |= pgm_read_byte_near(&(staben['C'-'A'][i])) << 8;
            matrix[5 + i] |= pgm_read_byte_near(&(staben['C'-'A'][i])) << 11;
            matrix[5 + i] |= pgm_read_byte_near(&(staben['W'-'A'][i])) << 5;
          }
        }
      break;
      case EXT_MODE_ENABLE_ALARM:
       renderer.clearScreenBuffer(matrix);
       if(settings.getEnableAlarm()) {
          for (byte i = 0; i < 5; i++) {
            matrix[0 + i] |= pgm_read_byte_near(&(staben['A'-'A'][i])) << 11;
            matrix[0 + i] |= pgm_read_byte_near(&(staben['L'-'A'][i])) << 5;
            matrix[5 + i] |= pgm_read_byte_near(&(staben['E'-'A'][i])) << 11;
            matrix[5 + i] |= pgm_read_byte_near(&(staben['N'-'A'][i])) << 5;
          }
        } else {
          for (byte i = 0; i < 5; i++) {
            matrix[0 + i] |= pgm_read_byte_near(&(staben['A'-'A'][i])) << 11;
            matrix[0 + i] |= pgm_read_byte_near(&(staben['L'-'A'][i])) << 5;
            matrix[5 + i] |= pgm_read_byte_near(&(staben['D'-'A'][i])) << 11;
            matrix[5 + i] |= pgm_read_byte_near(&(staben['A'-'A'][i])) << 5;
          }
        }
      break;
      case EXT_MODE_DCF_IS_INVERTED:
        renderer.clearScreenBuffer(matrix);
        if(settings.getDcfSignalIsInverted()) {
          for (byte i = 0; i < 5; i++) {
            matrix[0 + i] |= pgm_read_byte_near(&(staben['R'-'A'][i])) << 11;
            matrix[0 + i] |= pgm_read_byte_near(&(staben['S'-'A'][i])) << 5;
            matrix[5 + i] |= pgm_read_byte_near(&(staben['I'-'A'][i])) << 11;
            matrix[5 + i] |= pgm_read_byte_near(&(staben['N'-'A'][i])) << 5;
          }
        } else {
          for (byte i = 0; i < 5; i++) {
            matrix[0 + i] |= pgm_read_byte_near(&(staben['R'-'A'][i])) << 11;
            matrix[0 + i] |= pgm_read_byte_near(&(staben['S'-'A'][i])) << 5;
            matrix[5 + i] |= pgm_read_byte_near(&(staben['N'-'A'][i])) << 11;
            matrix[5 + i] |= pgm_read_byte_near(&(staben['O'-'A'][i])) << 5;
          }
        }
      break;
      case EXT_MODE_LANGUAGE:
      renderer.clearScreenBuffer(matrix);
      for (byte i = 0; i < 5; i++) {
        switch(settings.getLanguage()) {
          case LANGUAGE_DE_DE:
            matrix[2 + i] |= pgm_read_byte_near(&(staben['D'-'A'][i])) << 11;
            matrix[2 + i] |= pgm_read_byte_near(&(staben['E'-'A'][i])) << 5;
          break;
          case LANGUAGE_DE_SW:
            matrix[0 + i] |= pgm_read_byte_near(&(staben['D'-'A'][i])) << 11;
            matrix[0 + i] |= pgm_read_byte_near(&(staben['E'-'A'][i])) << 5;
            matrix[5 + i] |= pgm_read_byte_near(&(staben['S'-'A'][i])) << 11;
            matrix[5 + i] |= pgm_read_byte_near(&(staben['W'-'A'][i])) << 5;
          break;
          case LANGUAGE_DE_BA:
            matrix[0 + i] |= pgm_read_byte_near(&(staben['D'-'A'][i])) << 11;
            matrix[0 + i] |= pgm_read_byte_near(&(staben['E'-'A'][i])) << 5;
            matrix[5 + i] |= pgm_read_byte_near(&(staben['B'-'A'][i])) << 11;
            matrix[5 + i] |= pgm_read_byte_near(&(staben['A'-'A'][i])) << 5;
          break;
          case LANGUAGE_DE_SA:
            matrix[0 + i] |= pgm_read_byte_near(&(staben['D'-'A'][i])) << 11;
            matrix[0 + i] |= pgm_read_byte_near(&(staben['E'-'A'][i])) << 5;
            matrix[5 + i] |= pgm_read_byte_near(&(staben['S'-'A'][i])) << 11;
            matrix[5 + i] |= pgm_read_byte_near(&(staben['A'-'A'][i])) << 5;
          break;
          case LANGUAGE_CH:
            matrix[2 + i] |= pgm_read_byte_near(&(staben['C'-'A'][i])) << 11;
            matrix[2 + i] |= pgm_read_byte_near(&(staben['H'-'A'][i])) << 5;
          break;
          case LANGUAGE_EN:
            matrix[2 + i] |= pgm_read_byte_near(&(staben['E'-'A'][i])) << 11;
            matrix[2 + i] |= pgm_read_byte_near(&(staben['N'-'A'][i])) << 5;
          break;
          case LANGUAGE_FR:
            matrix[2 + i] |= pgm_read_byte_near(&(staben['F'-'A'][i])) << 11;
            matrix[2 + i] |= pgm_read_byte_near(&(staben['R'-'A'][i])) << 5;
          break;
          case LANGUAGE_IT:
            matrix[2 + i] |= pgm_read_byte_near(&(staben['I'-'A'][i])) << 11;
            matrix[2 + i] |= pgm_read_byte_near(&(staben['T'-'A'][i])) << 5;
          break;
          case LANGUAGE_NL:
            matrix[2 + i] |= pgm_read_byte_near(&(staben['N'-'A'][i])) << 11;
            matrix[2 + i] |= pgm_read_byte_near(&(staben['L'-'A'][i])) << 5;
          break; 
          case LANGUAGE_ES:
            matrix[2 + i] |= pgm_read_byte_near(&(staben['E'-'A'][i])) << 11;
            matrix[2 + i] |= pgm_read_byte_near(&(staben['S'-'A'][i])) << 5;
          break; 
        }
      }
      break;
      case EXT_MODE_TEST:
        renderer.clearScreenBuffer(matrix);
        renderer.setCorners(4, settings.getRenderCornersCw(), matrix);
        if(settings.getEnableAlarm()) {
          matrix[4] |= 0b0000000000011111; // Alarm-LED
        }
        ledDriver.setPixelInScreenBuffer(x, y, matrix);
        x++;
        if(x > 10) {
          x = 0;
          y++;
          if(y > 9) {
            y = 0;
          }
        }
        needsUpdateFromRtc = true;
        break;
    }

    // Update mit onChange = true, weil sich hier (aufgrund needsUpdateFromRtc) immer was geaendert hat.
    // Entweder weil wir eine Sekunde weiter sind, oder weil eine Taste gedrueckt wurde.
    ledDriver.writeScreenBufferToMatrix(matrix, true);
  }

  /*
   *
   * Tasten abfragen
   *
   */
  // M+ und H+ im STD_MODE_BLANK gedrueckt?
  if((mode == STD_MODE_BLANK) && extModeDoubleButton.pressed()) {
    DEBUG_PRINTLN(F("Minutes plus AND hours plus pressed in STD_MODE_BLANK..."));
    DEBUG_FLUSH();
    while(minutesPlusButton.pressed());
    while(hoursPlusButton.pressed());
    mode = EXT_MODE_START;
    needsUpdateFromRtc = true;
    ledDriver.wakeUp();
    DEBUG_PRINT(F("Entering EXT_MODEs, mode is now "));
    DEBUG_PRINT(mode);
    DEBUG_PRINTLN(F("..."));
    DEBUG_FLUSH();
  }
  
  // Taste Minuten++ (brighness++) gedrueckt?
  if (minutesPlusButton.pressed()) {
    DEBUG_PRINTLN(F("Minutes plus pressed..."));
    DEBUG_FLUSH();    
    needsUpdateFromRtc = true;
    switch(mode) {
      case EXT_MODE_TIMESET:
        RTCtime.Minute++;
        RTCtime.Second = 0;
        ds1307.write(RTCtime);
        ds1307.read(RTCtime);  // This seems an unneccessary line to me
        helperSeconds = 0;  
        DEBUG_PRINT(F("M is now "));
        DEBUG_PRINTLN(RTCtime.Minute);
        DEBUG_FLUSH();
        break;
      case EXT_MODE_TIME_SHIFT:
        if(settings.getTimeShift() < 13) {
          settings.setTimeShift(settings.getTimeShift() + 1);
        }
      break;
      case STD_MODE_ALARM:
	//    alarm._alarmTime.Minute++;
        alarm.setShowAlarmTimeTimer(10);
        DEBUG_PRINT(F("A is now "));
   //     DEBUG_PRINTLN(alarm.getAlarmTime()->asString());
        DEBUG_FLUSH();
        break;
      case STD_MODE_BRIGHTNESS:
        if(settings.getBrightness() < 100) {
          byte b = settings.getBrightness() + 10;
          if(b > 100) {
            b = 100;
          }
          settings.setBrightness(b);
          settings.saveToEEPROM();
          ledDriver.setBrightness(b);
        }
        break;
      case EXT_MODE_LDR_MODE:
        settings.setUseLdr(!settings.getUseLdr());
        DEBUG_PRINT(F("LDR is now "));
        DEBUG_PRINTLN(settings.getUseLdr());
        DEBUG_FLUSH();
        break;
      case EXT_MODE_CORNERS:
        settings.setRenderCornersCw(!settings.getRenderCornersCw());
      break;
      case EXT_MODE_ENABLE_ALARM:
        settings.setEnableAlarm(!settings.getEnableAlarm());
      break;
      case EXT_MODE_DCF_IS_INVERTED:
        settings.setDcfSignalIsInverted(!settings.getDcfSignalIsInverted());
      break;
      case EXT_MODE_LANGUAGE:
        settings.setLanguage(settings.getLanguage() + 1);
        if(settings.getLanguage() > LANGUAGE_COUNT) {
          settings.setLanguage(0);
        }
      break;
    }
  }
  
  // Taste Stunden++ (brightness--) gedrueckt?
  if (hoursPlusButton.pressed()) {
    DEBUG_PRINTLN(F("Hours plus pressed..."));
    DEBUG_FLUSH();
    needsUpdateFromRtc = true;
    switch(mode) {
      case EXT_MODE_TIMESET:
   //     ds1307.incHours();
   //    ds1307.setSeconds(0);
   //     ds1307.writeTime();
   //     ds1307.readTime();
        helperSeconds = 0;
        DEBUG_PRINT(F("H is now "));
//        DEBUG_PRINTLN(ds1307.getHours());
        DEBUG_FLUSH();
        break;
      case EXT_MODE_TIME_SHIFT:
        if(settings.getTimeShift() > -13) {
          settings.setTimeShift(settings.getTimeShift() - 1);
        }
      break;
      case STD_MODE_ALARM:
  //      alarm.getAlarmTime()->incHours();
  //      alarm.setShowAlarmTimeTimer(10);
        DEBUG_PRINT(F("A is now "));
//        DEBUG_PRINTLN(alarm.getAlarmTime()->asString());
        DEBUG_FLUSH();
        break;
      case STD_MODE_BRIGHTNESS:
        if(settings.getBrightness() > 1) {
          int i = settings.getBrightness() - 10;
          if(i < 2) {
            i = 1;
          }
          settings.setBrightness(i);
          settings.saveToEEPROM();
          ledDriver.setBrightness(i);
        }
        break;
      case EXT_MODE_LDR_MODE:
        settings.setUseLdr(!settings.getUseLdr());
        DEBUG_PRINT(F("LDR is now "));
        DEBUG_PRINTLN(settings.getUseLdr());
        DEBUG_FLUSH();
        break;
      case EXT_MODE_CORNERS:
        settings.setRenderCornersCw(!settings.getRenderCornersCw());
      break;
      case EXT_MODE_ENABLE_ALARM:
        settings.setEnableAlarm(!settings.getEnableAlarm());
      break;
      case EXT_MODE_DCF_IS_INVERTED:
        settings.setDcfSignalIsInverted(!settings.getDcfSignalIsInverted());
      break;
      case EXT_MODE_LANGUAGE:
        if(settings.getLanguage() == 0) {
          settings.setLanguage(LANGUAGE_COUNT);
        } else {
          settings.setLanguage(settings.getLanguage() - 1);
        }
      break;
    }
  }
  
  // Taste Moduswechsel gedrueckt?
  if (modeChangeButton.pressed()) {
    if(alarm.isActive()) 
    {
      alarm.deactivate();
      mode = STD_MODE_NORMAL;
    } 
    else {
      mode++;
    }
    // Brightness ueberspringen, wenn LDR verwendet wird.
    if(settings.getUseLdr() && (mode == STD_MODE_BRIGHTNESS)) {
      mode++;
    }
    // Alarm ueberspringen, wenn kein Alarm enabled ist.
    if(!settings.getEnableAlarm() && (mode == STD_MODE_ALARM)) {
      mode++;
    }
    if(mode == STD_MODE_COUNT + 1) {
      mode = STD_MODE_NORMAL;
    }
    if(mode == EXT_MODE_COUNT + 1) {
      mode = STD_MODE_NORMAL;
    }
    
    needsUpdateFromRtc = true;

    // brauchen wir alle Zeilen?
    if((mode == STD_MODE_NORMAL) || (mode == STD_MODE_SECONDS)) {
      ledDriver.setLinesToWrite(10);
    } else {
      ledDriver.setLinesToWrite(16);
    }

      if(mode == STD_MODE_ALARM) {
        // wenn auf Alarm gewechselt wurde, fuer 10 Sekunden die
        // Weckzeit anzeigen.
        alarm.setShowAlarmTimeTimer(10);
      } 

    DEBUG_PRINT(F("Change mode pressed, mode is now "));
    DEBUG_PRINT(mode);
    DEBUG_PRINTLN(F("..."));
    DEBUG_FLUSH();
    
    // Displaytreiber ausschalten, wenn BLANK
    if(mode == STD_MODE_BLANK) {
      DEBUG_PRINTLN(F("LED-Driver: ShutDown"));
      DEBUG_FLUSH();
      ledDriver.shutDown();
    } 
    // und einschalten, wenn BLANK verlassen wurde
    if(lastMode == STD_MODE_BLANK) {
      DEBUG_PRINTLN(F("LED-Driver: WakeUp"));
      DEBUG_FLUSH();
      ledDriver.wakeUp();
    }
    
    // Merker, damit wir nach einer automatischen Abschaltung
    // zum richtigen Mode zurueckkommen.
    lastMode = mode;

    // Werte speichern (die Funktion speichert nur geaenderten Werten)...
    settings.saveToEEPROM();
  }
  
  /*
   *
   * Display zeitgesteuert abschalten?
   * Das Verbessert den DCF77-Empfang bzw. ermoeglicht ein dunkles Schlafzimmer.
   *
   */
  if((offTime.Minute != 0) && (onTime.Minute != 0)) {
	ds1307.read(RTCtime);
    if((mode != STD_MODE_BLANK) && (offTime.Minute == RTCtime.Minute)) {
      mode = STD_MODE_BLANK;
      ledDriver.shutDown();
    }
    if((mode == STD_MODE_BLANK) && (onTime.Minute == RTCtime.Minute)) {
      mode = lastMode;
      ledDriver.wakeUp();
    }
  }

  /*
   *
   * Alarm?
   *
   */
  
  /***
  
    if((mode == STD_MODE_ALARM) && (alarm.getShowAlarmTimeTimer() == 0) && !alarm.isActive()) {
      if(alarm.getAlarmTime()->getMinutesOf12HoursDay(0) == ds1307.getMinutesOf12HoursDay()) {
        alarm.activate();
      }
    }
    if(alarm.isActive()) {
      // Nach 10 Minuten automatisch abschalten, falls der Wecker alleine rumsteht und die Nachbarn nervt...
      if(alarm.getAlarmTime()->getMinutesOf12HoursDay(MAX_BUZZ_TIME_IN_MINUTES) == ds1307.getMinutesOf12HoursDay()) {
        alarm.deactivate();
        alarm.buzz(false);
        mode = STD_MODE_NORMAL;
      }
    // Krach machen...    
      if(ds1307.getSeconds() % 2 == 0) {
        alarm.buzz(true);
      } else {
        alarm.buzz(false);
      }
  }

*/


  /*
   *
   * Die Matrix auf die LEDs multiplexen, hier 'Refresh-Zyklen'.
   *
   */
  if(mode != STD_MODE_BLANK) {
    ledDriver.writeScreenBufferToMatrix(matrix, false);
  }

  /*
   *
   * Status-LEDs ausgeben
   *
   */
#ifdef ENABLE_DCF_LED
  statusLed(PIN_DCF77_LED,HIGH);
#endif
#ifdef ENABLE_SQW_LED
  statusLed(PIN_SQW_LED, digitalRead(PIN_SQW_SIGNAL) == HIGH);
#endif

  /*
   *
   * DCF77-Empfaenger abfragen
   *
   */
    static time_t prevDisplay = 0;
	statusLed(PIN_DCF77_LED,LOW);
    
    if( now() != prevDisplay) {
	    prevDisplay = now();

      statusLed(PIN_DCF77_LED,HIGH);
	  ds1307.set(now());
	  
	  /*
	  ds1307.setSeconds(second());
      ds1307.setMinutes(minute());
      ds1307.setHours(hour());
      ds1307.setDate(day());
      ds1307.setDayOfWeek(weekday());
      ds1307.setMonth(month());
      ds1307.setYear(year());
      ds1307.writeTime();
	  
	  */
	  
      DEBUG_PRINTLN(F("DCF77-Time written to DS1307."));
      DEBUG_FLUSH();

      digitalClockDisplay();
    }	
	
	
  }

void digitalClockDisplay() {
	if (sync) {
		Serial.println("DCF sync good");
		sync = false;
	}
	
	DEBUG_PRINT(day());
	DEBUG_PRINT('.');
	DEBUG_PRINT(month());
	DEBUG_PRINT('.');
	DEBUG_PRINT(year());
	DEBUG_PRINT(' ');
	DEBUG_PRINT(' ');
	
	
	const uint8_t minutes = minute();
	const uint8_t hours = hour();
	
	print_2_digits(hours);
	DEBUG_PRINT(':');
	print_2_digits(minutes);
	DEBUG_PRINT(':');
	print_2_digits(second());
	
	DEBUG_PRINTLN();
}

void print_2_digits(const uint8_t digits) {
	if (digits < 10) {
		DEBUG_PRINT('0');
	}
	DEBUG_PRINTDEC(digits);
}

void statusLed(int _statusLedPin, boolean on) {
	if(on) {
		digitalWrite(_statusLedPin, HIGH);
		} else {
		digitalWrite(_statusLedPin, LOW);
	}
}
