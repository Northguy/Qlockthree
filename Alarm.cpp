/**
 * Alarm.cpp
 * Klasse fuer die Weckfunktion.
 *
 * @mc       Arduino/RBBB
 * @autor    Christian Aschoff / caschoff _AT_ mac _DOT_ com
 * @version  1.0
 * @created  22.1.2013
 *
 * Versionshistorie:
 * V 1.0:  - Erstellt.
 */
#include "Alarm.h"

// #define DEBUG
#include "Debug.h"

#include "Configuration.h"

/**
 * Konstruktor.
 * 
 * @param speakerPin Der Pin, an dem der Lautsprecher oder Buzzer haengt.
 */
Alarm::Alarm(byte speakerPin) {
  _speakerPin = speakerPin;
  pinMode(_speakerPin, OUTPUT);
  // um 7 Uhr geht das Alarm (sec, min, hr, -, -, -)
  TimeElements _alarmTime	= {0, 0, 7, 0, 0, 0};
  _isActive					= false;
  _showAlarmTimeTimer		= 0;
}

/**
 * Den Weckton einschalten.
 * 
 * @param: on - TRUE, der Ton wird eingeschaltet.
 *              FALSE, der Ton wird ausgeschaltet.
 */
void Alarm::buzz(boolean on) {
#ifdef SPEAKER_IS_BUZZER
  if(on) {
    digitalWrite(_speakerPin, HIGH);
  } else {
    digitalWrite(_speakerPin, LOW);
  }
#else
  if(on) {
    tone(_speakerPin, SPEAKER_FREQUENCY);
  } else {
    noTone(_speakerPin);
  }
#endif
}

/**
 * Die eingestellte Weckzeit bekommen.
 * 
 * @return Eine TimeStamp mit der eingstellten Weckzeit. 
 */
TimeElements Alarm::getAlarmTime() {
  return _alarmTime;
}
  
/**
 * Die verbleibende Zeit in Sekunden bekommen, fuer die 
 * die Weckzeit angezeigt werden soll.
 * 
 * @return Die Zeit in Sekunden. 
 */
byte Alarm::getShowAlarmTimeTimer() {
  return _showAlarmTimeTimer;
}

/**
 * Die Zeit in Sekunden setzten, fuer die die
 * Weckzeit angezeigt werden soll.
 * 
 * @param seconds Die Zeit in Sekunden.
 */
void Alarm::setShowAlarmTimeTimer(byte seconds) {
  _showAlarmTimeTimer = seconds;
}

/**
 * Die Zeit, fuer die die Weckzeit angezeigt werden soll,
 * um eine Sekunde verringern.
 */
void Alarm::decShowAlarmTimeTimer() {
  if(_showAlarmTimeTimer > 0) {
    _showAlarmTimeTimer--;
  }
}

/**
 * Ist der Wecker aktiv?
 * 
 * @return TRUE, wenn der Wecker aktiv ist.
 *         FALSE, wenn der Wekcer ausgeschaltet ist. 
 */
boolean Alarm::isActive() {
  return _isActive;
}

/**
 * Den Wecker einschalten.
 */
void Alarm::activate() {
  _isActive = true;
}

/**
 * Den Wecker ausschalten.
 */
void Alarm::deactivate() {
  _isActive = false;
  buzz(false);
}


