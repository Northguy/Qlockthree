/**
 * LDR.cpp
 * Klasse fuer den Zugriff auf einen lichtabhaengigen Widerstand.
 *
 * @mc       Arduino/RBBB
 * @autor    Christian Aschoff / caschoff _AT_ mac _DOT_ com
 * @version  1.5
 * @created  18.3.2012
 * @updated  18.8.2013
 *
 * Versionshistorie:
 * V 1.1:  - Optimierung hinsichtlich Speicherbedarf.
 * V 1.2:  - Verbessertes Debugging.
 * V 1.3:  - Beschraenkund der LDR-Werte bei autoscale == false.
 * V 1.4:  - Der LDR mapped die Werte jetzt selbst, dadurch wird flackern bei unguenstigen Lichtverhaeltnissen vermindert.
 * V 1.5:  - Der LDR gibt Werte zwischen 0 und 100% zurueck, das ist besser verstaendlich.
 */
#include "LDR.h"

// #define DEBUG
#include "Debug.h"

/**
 * Initialisierung mit dem Pin, an dem der LDR haengt.
 * Die Maximalwerte vom LDR koennen automatisch
 * eingemessen werden (LDR_AUTOSCALE).
 * Ansonsten muss man diese Werte im #define-DEBUG-Mode
 * ausmessen und eintragen.
 */
LDR::LDR(byte pin) {
  _pin  = pin;
  _meanpointer = 0;
  _lastValue   = 0;
  _outputValue = 0;
  _TotalMeanValues = 0;
  _BufferFull = false;
#ifdef LDR_AUTOSCALE
  _min = 1023;
  _max = 0;
#else
  _min = LDR_MANUAL_MIN;
  _max = LDR_MANUAL_MAX;
#endif
}

/**
 * Welchen Wert hat der LDR?
 */
unsigned int LDR::value() {
  unsigned int val = analogRead(_pin);
  if(val != _lastValue) {
    _lastValue = val;

//Autoscale defines the min and max value to determine the boundaries of the Brightness
#ifdef LDR_AUTOSCALE
    if(val < _min) {
      _min = val;
    }
    if(val > _max) {
      _max = val;
    }
#else
    val = constrain(val, _min, _max); 
#endif
    unsigned int mapVal = map(val, _min, _max, 100, 0);
    
    DEBUG_PRINT(F(" _min: "));
    DEBUG_PRINT(_min);
    DEBUG_PRINT(F(" _max: "));
    DEBUG_PRINT(_max);
    DEBUG_PRINT(F(" ldr: "));
    DEBUG_PRINT(val);
    DEBUG_PRINT(F(" mapValue: "));
    DEBUG_PRINTLN(mapVal);
    DEBUG_FLUSH();
    // glaetten
    
    if (_BufferFull != true) 
    {
      // buffer is not yet full
      if (_meanpointer == 0) 
      {
        _outputValue == mapVal;
        _TotalMeanValues == mapVal;
      }
       else
       {
         // store value in buffer position
         _meanvalues[_meanpointer] = mapVal;
         // calculate running average
         _TotalMeanValues == _TotalMeanValues + mapVal;  
         _outputValue = (unsigned int)(_TotalMeanValues)/(_meanpointer+1);
       }
     } //endif _meanpointer == 0
    else
    {
      // buffer is full
      // First remove the 'old' value from the buffer and substract this from the TotalMeanValues
      _TotalMeanValues == _TotalMeanValues - _meanvalues[_meanpointer];
      // now add MapVal
      _TotalMeanValues == (_TotalMeanValues + mapVal);
      // store new value in buffer position
      _meanvalues[_meanpointer] = mapVal;
      // Calculate average
      _outputValue = (unsigned int)(_TotalMeanValues)/LDR_MEAN_COUNT;
    }
    
    _meanpointer++; // increase meanpointer
    
    if(_meanpointer == LDR_MEAN_COUNT) 
    {
      _meanpointer = 0; // reset meanpointer if array index length is exceeded
      _BufferFull = true; // This only happens once because _Bufferfull is only false at initialisation. When the whole buffer is filled with data, the buffer becomes a circular buffer
    }
    
  return _outputValue;
}
}
