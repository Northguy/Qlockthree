/**
 * Renderer.cpp
 * Diese Klasse rendert die Woerter auf die Matrix.
 *
 * @mc       Arduino/RBBB
 * @autor    Christian Aschoff / caschoff _AT_ mac _DOT_ com
 * @version  1.3
 * @created  21.1.2013
 * @updated  9.9.2013
 *
 * Versionshistorie:
 * V 1.0:  - Erstellt.
 * V 1.1:  - Spanisch hinzugefuegt.
 * V 1.2:  - setMinutes - hours auf char umgestellt, damit Zeitverschiebung geht...
 * V 1.3:  - Alle Deutsch-Varianten zusammengefasst, um Platz zu sparen.
 *         - Fehler im Italienischen behoben.
 */
#include "Renderer.h"

#include "Woerter_DE.h"
// #include "Woerter_DE_MKF.h"
#include "Woerter_CH.h"
#include "Woerter_EN.h"
#include "Woerter_FR.h"
#include "Woerter_IT.h"
#include "Woerter_NL.h"
#include "Woerter_ES.h"

// #define DEBUG
#include "Debug.h"

Renderer::Renderer() {
}

/**
 * Ein Zufallsmuster erzeugen (zum Testen der LEDs)
 * A random fill of the matrix. Matrix = 16xWORD = 16 x 16 bits
 */
void Renderer::scrambleScreenBuffer(word matrix[16]) {
  for (byte i = 0; i < 16; i++) {
    matrix[i] = random(65536);
  }
}

/**
 * Die Matrix loeschen (zum Stromsparen, DCF77-Empfang
 * verbessern etc.)
 */
void Renderer::clearScreenBuffer(word matrix[16]) {
  for (byte i = 0; i < 16; i++) {
    matrix[i] = 0;
  }
}

/**
 * Die Matrix komplett einschalten (zum Testen der LEDs)
 */
void Renderer::setAllScreenBuffer(word matrix[16]) {
  for (byte i = 0; i < 16; i++) {
    matrix[i] = 65535;
  }
}

/**
 * Setzt die Wortminuten, je nach hours/minutes.
 */
void Renderer::setMinutes(char hours, byte minutes, byte language, word matrix[16]) {
  while (hours < 1) {
    hours += 12;
  }
  while (hours > 12) {
    hours -= 12;
  }
  
  switch (language) {
  //
  // Deutsch: Hochdeutsch
  //
  case LANGUAGE_DE_DE:
  case LANGUAGE_DE_SW:
  case LANGUAGE_DE_BA:
  case LANGUAGE_DE_SA:
     DE_ESIST;
  
    switch (minutes / 5) {
    case 0:
      // glatte Stunde
      setHours(hours, true, language, matrix);
      break;
    case 1:
      // 5 nach
      DE_FUENF;
      DE_NACH;
      setHours(hours, false, language, matrix);
      break;
    case 2:
      // 10 nach
      DE_ZEHN;
      DE_NACH;
      setHours(hours, false, language, matrix);
      break;
    case 3:
      // viertel nach
      if ((language == LANGUAGE_DE_SW) || (language == LANGUAGE_DE_SA)) {
        DE_VIERTEL;
        setHours(hours + 1, false, language, matrix);
      } else {
        DE_VIERTEL;
        DE_NACH;
        setHours(hours, false, language, matrix);
      }
      break;
    case 4:
      // 20 nach
      if (language == LANGUAGE_DE_SA) {
        DE_ZEHN;
        DE_VOR;
        DE_HALB;
        setHours(hours + 1, false, language, matrix);
      } else {
        DE_ZWANZIG;
        DE_NACH;
        setHours(hours, false, language, matrix);
        }
      break;
    case 5:
      // 5 vor halb
      DE_FUENF;
      DE_VOR;
      DE_HALB;
      setHours(hours + 1, false, language, matrix);
      break;
    case 6:
      // halb
      DE_HALB;
      setHours(hours + 1, false, language, matrix);
      break;
    case 7:
      // 5 nach halb
      DE_FUENF;
      DE_NACH;
      DE_HALB;
      setHours(hours + 1, false, language, matrix);
      break;
    case 8:
      // 20 vor
      if (language == LANGUAGE_DE_SA) {
        DE_ZEHN;
        DE_NACH;
        DE_HALB;
        setHours(hours + 1, false, language, matrix);
      } else {
        DE_ZWANZIG;
        DE_VOR;
        setHours(hours + 1, false, language, matrix);
      }
      break;
    case 9:
      // viertel vor
      if ((language == LANGUAGE_DE_SW) || (language == LANGUAGE_DE_BA) || (language == LANGUAGE_DE_SA)) {
        DE_DREIVIERTEL;
        setHours(hours + 1, false, language, matrix);
      } else {
        DE_VIERTEL;
        DE_VOR;
        setHours(hours + 1, false, language, matrix);
      }
      break;
    case 10:
      // 10 vor
      DE_ZEHN;
      DE_VOR;
      setHours(hours + 1, false, language, matrix);
      break;
    case 11:
      // 5 vor
      DE_FUENF;
      DE_VOR;
      setHours(hours + 1, false, language, matrix);
      break;
    }
    break;
  //
  // Schweiz: Berner-Deutsch
  //
  case LANGUAGE_CH:
    CH_ESISCH;
  
    switch (minutes / 5) {
    case 0:
      // glatte Stunde
      setHours(hours, true, language, matrix);
      break;
    case 1:
      // 5 ab
      CH_FUEF;
      CH_AB;
      setHours(hours, false, language, matrix);
      break;
    case 2:
      // 10 ab
      CH_ZAEAE;
      CH_AB;
      setHours(hours, false, language, matrix);
      break;
    case 3:
      // viertel ab
      CH_VIERTU;
      CH_AB;
      setHours(hours, false, language, matrix);
      break;
    case 4:
      // 20 ab
      CH_ZWAENZG;
      CH_AB;
      setHours(hours, false, language, matrix);
      break;
    case 5:
      // 5 vor halb
      CH_FUEF;
      CH_VOR;
      CH_HAUBI;
      setHours(hours + 1, false, language, matrix);
      break;
    case 6:
      // halb
      CH_HAUBI;
      setHours(hours + 1, false, language, matrix);
      break;
    case 7:
      // 5 ab halb
      CH_FUEF;
      CH_AB;
      CH_HAUBI;
      setHours(hours + 1, false, language, matrix);
      break;
    case 8:
      // 20 vor
      CH_ZWAENZG;
      CH_VOR;
      setHours(hours + 1, false, language, matrix);
      break;
    case 9:
      // viertel vor
      CH_VIERTU;
      CH_VOR;
      setHours(hours + 1, false, language, matrix);
      break;
    case 10:
      // 10 vor
      CH_ZAEAE;
      CH_VOR;
      setHours(hours + 1, false, language, matrix);
      break;
    case 11:
      // 5 vor
      CH_FUEF;
      CH_VOR;
      setHours(hours + 1, false, language, matrix);
      break;
    }
    break;
  //
  // Englisch
  //
  case LANGUAGE_EN:
    EN_ITIS;
  
    switch (minutes / 5) {
    case 0:
      // glatte Stunde
      setHours(hours, true, language, matrix);
      break;
    case 1:
      // 5 nach
      EN_FIVE;
      EN_PAST;
      setHours(hours, false, language, matrix);
      break;
    case 2:
      // 10 nach
      EN_TEN;
      EN_PAST;
      setHours(hours, false, language, matrix);
      break;
    case 3:
      // viertel nach
      EN_A;
      EN_QUATER;
      EN_PAST;
      setHours(hours, false, language, matrix);
      break;
    case 4:
      // 20 nach
      EN_TWENTY;
      EN_PAST;
      setHours(hours, false, language, matrix);
      break;
    case 5:
      // 5 vor halb
      EN_TWENTY;
      EN_FIVE;
      EN_PAST;
      setHours(hours, false, language, matrix);
      break;
    case 6:
      // halb
      EN_HALF;
      EN_PAST;
      setHours(hours, false, language, matrix);
      break;
    case 7:
      // 5 nach halb
      EN_TWENTY;
      EN_FIVE;
      EN_TO;
      setHours(hours + 1, false, language, matrix);
      break;
    case 8:
      // 20 vor
      EN_TWENTY;
      EN_TO;
      setHours(hours + 1, false, language, matrix);
      break;
    case 9:
      // viertel vor
      EN_A;
      EN_QUATER;
      EN_TO;
      setHours(hours + 1, false, language, matrix);
      break;
    case 10:
      // 10 vor
      EN_TEN;
      EN_TO;
      setHours(hours + 1, false, language, matrix);
      break;
    case 11:
      // 5 vor
      EN_FIVE;
      EN_TO;
      setHours(hours + 1, false, language, matrix);
      break;
    }
    break;
  //
  // Franzoesisch
  //
  case LANGUAGE_FR:
    FR_ILEST;
  
    switch (minutes / 5) {
    case 0:
      // glatte Stunde
      setHours(hours, true, language, matrix);
      FR_hours(hours, matrix);
      break;
    case 1:
      // 5 nach
      setHours(hours, false, language, matrix);
      FR_hours(hours, matrix);
      FR_CINQ;    
      break;
    case 2:
      // 10 nach
      setHours(hours, false, language, matrix);
      FR_hours(hours, matrix);
      FR_DIX;
      break;
    case 3:
      // viertel nach
      setHours(hours, false, language, matrix);
      FR_hours(hours, matrix);
      FR_ET;
      FR_QUART;
      break;
    case 4:
      // 20 nach
      setHours(hours, false, language, matrix);
      FR_hours(hours, matrix);
      FR_VINGT;
      break;
    case 5:
      // 5 vor halb
      setHours(hours, false, language, matrix);
      FR_hours(hours, matrix);
      FR_VINGT;
      FR_TRAIT;
      FR_CINQ;
      break;
    case 6:
      // halb
      setHours(hours, false, language, matrix);
      FR_hours(hours, matrix);
      FR_ET;
      FR_DEMI;
      break;
    case 7:
      // 5 nach halb
      setHours(hours + 1, false, language, matrix);
      FR_hours(hours + 1, matrix);
      FR_MOINS;
      FR_VINGT;
      FR_TRAIT;
      FR_CINQ;
      break;
    case 8:
      // 20 vor
      setHours(hours + 1, false, language, matrix);
      FR_hours(hours + 1, matrix);
      FR_MOINS;
      FR_VINGT;
      break;
    case 9:
      // viertel vor
      setHours(hours + 1, false, language, matrix);
      FR_hours(hours + 1, matrix);
      FR_MOINS;
      FR_LE;
      FR_QUART;
      break;
    case 10:
      // 10 vor
      setHours(hours + 1, false, language, matrix);
      FR_hours(hours + 1, matrix);
      FR_MOINS;
      FR_DIX;
      break;
    case 11:
      // 5 vor
      setHours(hours + 1, false, language, matrix);
      FR_hours(hours + 1, matrix);
      FR_MOINS;
      FR_CINQ;
      break;
    }
    break;
  //
  // Italienisch
  //  
  case LANGUAGE_IT:
    switch (minutes / 5) {
    case 0:
      // glatte Stunde
      setHours(hours, true, language, matrix);
      IT_hours(hours, matrix);
      break;
    case 1:
      // 5 nach
      IT_E2;
      IT_CINQUE;
      setHours(hours, false, language, matrix);
      IT_hours(hours, matrix);
      break;
    case 2:
      // 10 nach
      IT_E2;
      IT_DIECI;
      setHours(hours, false, language, matrix);
      IT_hours(hours, matrix);
      break;
    case 3:
      // viertel nach
      IT_E2;
      IT_UN;
      IT_QUARTO;
      setHours(hours, false, language, matrix);
      IT_hours(hours, matrix);
      break;
    case 4:
      // 20 nach
      IT_E2;
      IT_VENTI;
      setHours(hours, false, language, matrix);
      IT_hours(hours, matrix);
      break;
    case 5:
      // 5 vor halb
      IT_E2;
      IT_VENTI;
      IT_CINQUE;
      setHours(hours, false, language, matrix);
      IT_hours(hours, matrix);
      break;
    case 6:
      // halb
      IT_E2;
      IT_MEZZA;
      setHours(hours, false, language, matrix);
      IT_hours(hours, matrix);
      break;
    case 7:
      // 5 nach halb
      IT_MENO;
      IT_VENTI;
      IT_CINQUE;
      setHours(hours + 1, false, language, matrix);
      IT_hours(hours + 1, matrix);
      break;
    case 8:
      // 20 vor
      IT_MENO;
      IT_VENTI;
      setHours(hours + 1, false, language, matrix);
      IT_hours(hours + 1, matrix);
      break;
    case 9:
      // viertel vor
      IT_MENO;
      IT_UN;
      IT_QUARTO;
      setHours(hours + 1, false, language, matrix);
      IT_hours(hours + 1, matrix);
      break;
    case 10:
      // 10 vor
      IT_MENO;
      IT_DIECI;
      setHours(hours + 1, false, language, matrix);
      IT_hours(hours + 1, matrix);
      break;
    case 11:
      // 5 vor
      IT_MENO;
      IT_CINQUE;
      setHours(hours + 1, false, language, matrix);
      IT_hours(hours + 1, matrix);
      break;
    }
    break;
    //
    // Niederlaendisch
    //
    case LANGUAGE_NL:
    NL_HETIS;
  
    switch (minutes / 5) {
    case 0:
      // glatte Stunde
      setHours(hours, true, language, matrix);
      break;
    case 1:
      // 5 nach
      NL_VIJF;
      NL_OVER;
      setHours(hours, false, language, matrix);
      break;
    case 2:
      // 10 nach
      NL_TIEN;
      NL_OVER;
      setHours(hours, false, language, matrix);
      break;
    case 3:
      // viertel nach
      NL_KWART;
      NL_OVER2;
      setHours(hours, false, language, matrix);
      break;
    case 4:
      // 10 vor halb
      NL_TIEN;
      NL_VOOR;
      NL_HALF;
      setHours(hours +1, false, language, matrix);
      break;
    case 5:
      // 5 vor halb
      NL_VIJF;
      NL_VOOR;
      NL_HALF;
      setHours(hours + 1, false, language, matrix);
      break;
    case 6:
      // halb
      NL_HALF;
      setHours(hours + 1, false, language, matrix);
      break;
    case 7:
      // 5 nach halb
      NL_VIJF;
      NL_OVER;
      NL_HALF;
      setHours(hours + 1, false, language, matrix);
      break;
    case 8:
      // 20 vor
      NL_TIEN;
      NL_OVER;
      NL_HALF;
      setHours(hours + 1, false, language, matrix);
      break;
    case 9:
      // viertel vor
      NL_KWART;
      NL_VOOR2;
      setHours(hours + 1, false, language, matrix);
      break;
    case 10:
      // 10 vor
      NL_TIEN;
      NL_VOOR;
      setHours(hours + 1, false, language, matrix);
      break;
    case 11:
      // 5 vor
      NL_VIJF;
      NL_VOOR;
      setHours(hours + 1, false, language, matrix);
      break;
    }
    break;
  //
  // Spanisch
  //  
  case LANGUAGE_ES:
    switch (minutes / 5) {
    case 0:
      // glatte Stunde
      ES_hours(hours, matrix);
      setHours(hours, false, language, matrix);
      break;
    case 1:
      // 5 nach
      ES_Y;
      ES_CINCO;
      ES_hours(hours, matrix);
      setHours(hours, false, language, matrix);
      break;
    case 2:
      // 10 nach
      ES_Y;
      ES_DIEZ;
      ES_hours(hours, matrix);
      setHours(hours, false, language, matrix);
      break;
    case 3:
      // viertel nach
      ES_Y;
      ES_CUARTO;
      ES_hours(hours, matrix);
      setHours(hours, false, language, matrix);
      break;
    case 4:
      // 20 nach
      ES_Y;
      ES_VEINTE;
      ES_hours(hours, matrix);
      setHours(hours, false, language, matrix);
      break;
    case 5:
      // 5 vor halb
      ES_Y;
      ES_VEINTICINCO;
      ES_hours(hours, matrix);
      setHours(hours, false, language, matrix);
      break;
    case 6:
      // halb
      ES_Y;
      ES_MEDIA;
      ES_hours(hours, matrix);
      setHours(hours, false, language, matrix);
      break;
    case 7:
      // 5 nach halb
      ES_MENOS;
      ES_VEINTICINCO;
      ES_hours(hours + 1, matrix);
      setHours(hours + 1, false, language, matrix);
      break;
    case 8:
      // 20 vor
      ES_MENOS;
      ES_VEINTE;
      ES_hours(hours + 1, matrix);
      setHours(hours + 1, false, language, matrix);
      break;
    case 9:
      // viertel vor
      ES_MENOS;
      ES_CUARTO;
      ES_hours(hours + 1, matrix);
      setHours(hours + 1, false, language, matrix);
      break;
    case 10:
      // 10 vor
      ES_MENOS;
      ES_DIEZ;
      ES_hours(hours + 1, matrix);
      setHours(hours + 1, false, language, matrix);
      break;
    case 11:
      // 5 vor
      ES_MENOS;
      ES_CINCO;
      ES_hours(hours + 1, matrix);
      setHours(hours + 1, false, language, matrix);
      break;
    }
    break;
  }
}

/**
 * Setzt die Stunden, je nach hours. 'glatt' bedeutet,
 * es ist genau diese Stunde und wir muessen 'UHR'
 * dazuschreiben und EIN statt EINS, falls es 1 ist.
 * (Zumindest im Deutschen)
 */
void Renderer::setHours(byte hours, boolean glatt, byte language, word matrix[16]) {
  switch (language) {
  //
  // Deutsch (Hochdeutsch, Schwaebisch, Bayrisch)
  //
  case LANGUAGE_DE_DE:
  case LANGUAGE_DE_SW:
  case LANGUAGE_DE_BA:
  case LANGUAGE_DE_SA:
    if (glatt) {
      DE_UHR;
    }
  
    switch (hours) {
    case 0:
    case 12:
    case 24:
      DE_H_ZWOELF;
      break;
    case 1:
    case 13:
      if (glatt) {
        DE_H_EIN;
      } 
      else {
        DE_H_EINS;
      }
      break;
    case 2:
    case 14:
      DE_H_ZWEI;
      break;
    case 3:
    case 15:
      DE_H_DREI;
      break;
    case 4:
    case 16:
      DE_H_VIER;
      break;
    case 5:
    case 17:
      DE_H_FUENF;
      break;
    case 6:
    case 18:
      DE_H_SECHS;
      break;
    case 7:
    case 19:
      DE_H_SIEBEN;
      break;
    case 8:
    case 20:
      DE_H_ACHT;
      break;
    case 9:
    case 21:
      DE_H_NEUN;
      break;
    case 10:
    case 22:
      DE_H_ZEHN;
      break;
    case 11:
    case 23:
      DE_H_ELF;
      break;
    }
    break;
  //
  // Schweiz: Berner-Deutsch
  //
  case LANGUAGE_CH:
    switch (hours) {
    case 0:
    case 12:
    case 24:
      CH_H_ZWOEUFI;
      break;
    case 1:
    case 13:
      CH_H_EIS;
      break;
    case 2:
    case 14:
      CH_H_ZWOEI;
      break;
    case 3:
    case 15:
      CH_H_DRUE;
      break;
    case 4:
    case 16:
      CH_H_VIER;
      break;
    case 5:
    case 17:
      CH_H_FUEFI;
      break;
    case 6:
    case 18:
      CH_H_SAECHSI;
      break;
    case 7:
    case 19:
      CH_H_SIEBNI;
      break;
    case 8:
    case 20:
      CH_H_ACHTI;
      break;
    case 9:
    case 21:
      CH_H_NUENI;
      break;
    case 10:
    case 22:
      CH_H_ZAENI;
      break;
    case 11:
    case 23:
      CH_H_EUFI;
      break;
    }
    break;
  //
  // Englisch
  //  
  case LANGUAGE_EN:
    if (glatt) {
      EN_OCLOCK;
    }
  
    switch (hours) {
    case 0:
    case 12:
    case 24:
      EN_H_TWELVE;
      break;
    case 1:
    case 13:
      EN_H_ONE;
      break;
    case 2:
    case 14:
      EN_H_TWO;
      break;
    case 3:
    case 15:
      EN_H_THREE;
      break;
    case 4:
    case 16:
      EN_H_FOUR;
      break;
    case 5:
    case 17:
      EN_H_FIVE;
      break;
    case 6:
    case 18:
      EN_H_SIX;
      break;
    case 7:
    case 19:
      EN_H_SEVEN;
      break;
    case 8:
    case 20:
      EN_H_EIGHT;
      break;
    case 9:
    case 21:
      EN_H_NINE;
      break;
    case 10:
    case 22:
      EN_H_TEN;
      break;
    case 11:
    case 23:
      EN_H_ELEVEN;
      break;
    }
    break;
  //
  // Franzoesisch  
  case LANGUAGE_FR:
    switch (hours) {
    case 0:
    case 24:
      FR_H_MINUIT;
      break;
    case 12:
      FR_H_MIDI;
      break;
    case 1:
    case 13:
      FR_H_UNE;
      break;
    case 2:
    case 14:
      FR_H_DEUX;
      break;
    case 3:
    case 15:
      FR_H_TROIS;
      break;
    case 4:
    case 16:
      FR_H_QUATRE;
      break;
    case 5:
    case 17:
      FR_H_CINQ;
      break;
    case 6:
    case 18:
      FR_H_SIX;
      break;
    case 7:
    case 19:
      FR_H_SEPT;
      break;
    case 8:
    case 20:
      FR_H_HUIT;
      break;
    case 9:
    case 21:
      FR_H_NEUF;
      break;
    case 10:
    case 22:
      FR_H_DIX;
      break;
    case 11:
    case 23:
      FR_H_ONZE;
      break;
    }
    break;
  //
  // Italienisch
  //
  case LANGUAGE_IT:
    switch (hours) {
    case 0:
    case 12:
    case 24:
      IT_H_DODICI;
      break;
    case 1:
    case 13:
      IT_H_LUNA;
      break;
    case 2:
    case 14:
      IT_H_DUE;
      break;
    case 3:
    case 15:
      IT_H_TRE;
      break;
    case 4:
    case 16:
      IT_H_QUATTRO;
      break;
    case 5:
    case 17:
      IT_H_CINQUE;
      break;
    case 6:
    case 18:
      IT_H_SEI;
      break;
    case 7:
    case 19:
      IT_H_SETTE;
      break;
    case 8:
    case 20:
      IT_H_OTTO;
      break;
    case 9:
    case 21:
      IT_H_NOVE;
      break;
    case 10:
    case 22:
      IT_H_DIECI;
      break;
    case 11:
    case 23:
      IT_H_UNDICI;
      break;
    }
    break;
  //
  // Niederlaendisch
  //
  case LANGUAGE_NL:
    if (glatt) {
      NL_UUR;
    }
  
    switch (hours) {
    case 0:
    case 12:
    case 24:
      NL_H_TWAALF;
      break;
    case 1:
    case 13:
      NL_H_EEN;
      break;
    case 2:
    case 14:
      NL_H_TWEE;
      break;
    case 3:
    case 15:
      NL_H_DRIE;
      break;
    case 4:
    case 16:
      NL_H_VIER;
      break;
    case 5:
    case 17:
      NL_H_VIJF;
      break;
    case 6:
    case 18:
      NL_H_ZES;
      break;
    case 7:
    case 19:
      NL_H_ZEVEN;
      break;
    case 8:
    case 20:
      NL_H_ACHT;
      break;
    case 9:
    case 21:
      NL_H_NEGEN;
      break;
    case 10:
    case 22:
      NL_H_TIEN;
      break;
    case 11:
    case 23:
      NL_H_ELF;
      break;
    }
  break;
  //
  // Spanisch
  //
  case LANGUAGE_ES:  
    switch (hours) {
    case 0:
    case 12:
    case 24:
      ES_H_DOCE;
      break;
    case 1:
    case 13:
      ES_H_UNA;
      break;
    case 2:
    case 14:
      ES_H_DOS;
      break;
    case 3:
    case 15:
      ES_H_TRES;
      break;
    case 4:
    case 16:
      ES_H_CUATRO;
      break;
    case 5:
    case 17:
      ES_H_CINCO;
      break;
    case 6:
    case 18:
      ES_H_SEIS;
      break;
    case 7:
    case 19:
      ES_H_SIETE;
      break;
    case 8:
    case 20:
      ES_H_OCHO;
      break;
    case 9:
    case 21:
      ES_H_NUEVE;
      break;
    case 10:
    case 22:
      ES_H_DIEZ;
      break;
    case 11:
    case 23:
      ES_H_ONCE;
      break;
    }
    break;
  }
}

/**
 * Setzt die vier Punkte in den Ecken, je nach minutes % 5 (Rest).
 * 
 * @param ccw: TRUE -> clock wise -> im Uhrzeigersinn.
 *             FALSE -> counter clock wise -> gegen den Uhrzeigersinn.
 */
void Renderer::setCorners(byte minutes, boolean cw, word matrix[16]) {
  if(cw) {
    // im Uhrzeigersinn
    switch (minutes % 5) {
    case 0:
      break;
    case 1:
      matrix[1] |= 0b0000000000011111; // 1
      break;
    case 2:
      matrix[1] |= 0b0000000000011111; // 1
      matrix[0] |= 0b0000000000011111; // 2
      break;
    case 3:
      matrix[1] |= 0b0000000000011111; // 1
      matrix[0] |= 0b0000000000011111; // 2
      matrix[3] |= 0b0000000000011111; // 3
      break;
    case 4:
      matrix[1] |= 0b0000000000011111; // 1
      matrix[0] |= 0b0000000000011111; // 2
      matrix[3] |= 0b0000000000011111; // 3
      matrix[2] |= 0b0000000000011111; // 4
      break;
    }
  } else {
    // gegen den Uhrzeigersinn
    switch (minutes % 5) {
    case 0:
      break;
    case 1:
      matrix[0] |= 0b0000000000011111; // 1
      break;
    case 2:
      matrix[0] |= 0b0000000000011111; // 1
      matrix[1] |= 0b0000000000011111; // 2
      break;
    case 3:
      matrix[0] |= 0b0000000000011111; // 1
      matrix[1] |= 0b0000000000011111; // 2
      matrix[2] |= 0b0000000000011111; // 3
      break;
    case 4:
      matrix[0] |= 0b0000000000011111; // 1
      matrix[1] |= 0b0000000000011111; // 2
      matrix[2] |= 0b0000000000011111; // 3
      matrix[3] |= 0b0000000000011111; // 4
      break;
    }
  }
}

/**
 * Im Alarm-Einstell-Modus muessen bestimmte Woerter weg, wie z.B. "ES IST" im Deutschen.
 */
void Renderer::cleanWordsForAlarmSettingMode(byte language, word matrix[16]) {
  switch (language) {
  case LANGUAGE_DE_DE:
  case LANGUAGE_DE_SW:
  case LANGUAGE_DE_BA:
  case LANGUAGE_DE_SA:
    matrix[0] &= 0b0010001111111111; // ES IST weg
    break;
  case LANGUAGE_CH:
    matrix[0] &= 0b0010000111111111; // ES ISCH weg
    break;
  case LANGUAGE_EN:
    matrix[0] &= 0b0010011111111111; // IT IS weg
    break;
  case LANGUAGE_FR:
    matrix[0] &= 0b0010001111111111; // IL EST weg
    break;
  case LANGUAGE_IT:
    matrix[0] &= 0b0000100111111111; // SONO LE weg
    matrix[1] &= 0b0111111111111111; // E (L'UNA) weg
    break;
  case LANGUAGE_NL:
    matrix[0] &= 0b0001001111111111; // HET IS weg
    break;
  case LANGUAGE_ES:
    matrix[0] &= 0b1000100011111111; // SON LAS weg
    matrix[0] &= 0b0011100111111111; // ES LA weg
    break;  
  }
}

/**
 * Sprachlicher Spezialfall fuer Franzoesisch.
 */
void Renderer::FR_hours(byte hours, word matrix[16]) {
  if((hours == 1) || (hours == 13)) {
    FR_HEURE;
  } else if((hours == 0) || (hours == 12) || (hours == 24)) {
    // MIDI / MINUIT ohne HEURES
  } else {
    FR_HEURES;
  }
}

/**
 * Sprachlicher Spezialfall fuer Italienisch.
 */
void Renderer::IT_hours(byte hours, word matrix[16]) {
  if(hours != 1) {
    IT_SONOLE;
  } else {
    IT_E;
  }
}

/**
 * Sprachlicher Spezialfall fuer Spanisch.
 */
void Renderer::ES_hours(byte hours, word matrix[16]) {
  if((hours == 1) || (hours == 13)) {
    ES_ESLA;
  } else {
    ES_SONLAS;
  }
}

