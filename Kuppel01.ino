/*
Aufgabe:
Steuerung der Kuppel in der Sternwarte
Schnittstellen:
- Sensoren zur Erfassung der Umdrehungen einer Lagerrolle der Kuppel liefern die Relativposition der Kuppel
- Sensoren zur Erfassung einer Indexmarke (am beweglichen Teil der Kuppel) liefern eine absolute Position
- Aktoren zur Ansteuerung der Motoren zur Rechts- oder Linksdrehung der Kuppel
- Serielle Verbindung zu einem Kuppel-Client. 
  Der Kuppel-Client hat eine Bedienoberfläche mit Zustandsanzeigen und Bedienmöglichkeiten.
  Der Kuppel-Client hat Verbindung mit dem Teleskop-Server und erteilt der Kuppelsteuerung Aufträge.
  Die Kuppelsteuerung beantwortet jeden Auftrag.
Eigene Anzeigen:
- LCD-Display mit 4x20 Zeichen zeigt Zustandsdaten der Kuppelsteuerung
- Website mit Zustandsdaten, wird über die Wifi-Schnittstelle des Arduino-Nano angesteuert
*/

//  ---------   Includes   ------------------
  #include <NanoESP.h>
  #include <NanoESP_HTTP.h> //special library for http-connection
  #include <SoftwareSerial.h>
  #include <LiquidCrystal_I2C.h>
  #include <EEPROM.h>

  // Wifi - SSID und Passwort importieren. Mit diesen Daten versucht die Anwendung, sich ins Wifi-Netz einzuloggen.
  #include "d:\Arduino\WifiAccess.txt"
  
  // Statische Definiton der Website 
  #include "Website.h"  

//  ---------   Defines   ------------------
  #define APP_NAME "Kuppel-Stg "
  #define APP_VERSION "V1.0  "
  // IO-Ports der Sensoren und Aktoren
  #define SIGNAL1 2         // Port-Nr Sensor Signal 1 für die Relativposition
  #define SIGNAL2 3         // Port-Nr Sensor Signal 2 für die Relativposition
  #define INDEX01 5         // Port-Nr Sensor Indexmarke1 für die Absolutposition

  #define MOTOR_L 6         // Port-Nr Aktor Motor für die Linksdrehung der Kuppel
  #define MOTOR_R 7         // Port-Nr Aktor Motor für die Rechtsdrehung der Kuppel
  #define MOTOR_ON  HIGH    // Pegel zum Einschalten des Motors
  #define MOTOR_OFF LOW     // Pegel zum Ausschalten des Motors

  #define LED_INDEX01 8     // Port-Nr Kontroll-LED für Indexmarke1 auf dem externen Board
  #define LED_SIGNAL1 9     // Port-Nr Kontroll-LED für Signal 1 auf dem externen Board
  #define LED_SIGNAL2 10    // Port-Nr Kontroll-LED für Signal 2 auf dem externen Board

  #define LED_WLAN 13       // Port-Nr Kontroll-LED WLAN auf dem Arduino-Board

//  ---------   Daten   ------------------
  // Status der Flankenauswertung von SIGNAL1 und SIGNAL2:
  #define STATE_00 0                      // LOW  + LOW
  #define STATE_01 1                      // LOW  + HIGH
  #define STATE_10 2                      // HIGH + LOW
  #define STATE_11 3                      // HIGH + HIGH
  
  int oldStateSignals=STATE_00;           // aktueller Status der Flankenauswertung von SIGNAL1 und SIGNAL2
  int oldStateIndex=1;                    // aktuel+ler Status des Digitaleingangs für die Indexmarke (Activ-Low, im Ruhezustand = 1)
  int counterUpDown = 0;                  // Impulszähler zur Ermittlung der Relativposition der Kuppel
  int connectionId;                       // Id der Verbindung, von der der HTTP-Request kam
  #define NUMCONIDSSE 8                   // Anzahl der Elemente in connectionIdsSSE
  bool connectionIdsSSE[NUMCONIDSSE];     // Liste der Id's der Verbindungen, über die eine zyklische Datenaktualisierung mit Server Sent Events (SSE) erfolgen soll.
                                          // Liste kann nur die IDs von 0 bis length-1. Eine aktive Verbindung wird in der Liste mit True markiert.
  bool mMonitorOut = true;                // Ausgaben auf Monitor der Arduino-IDE ein/aus

  // Definitionen für recvWithStartEndMarkers = Empfangsroutine auf der Seriellen Schnittstelle
  const byte numChars = 32;               // Gräße des Empfangs-Zeichenpuffers
  char receivedChars[numChars];           // Empfangs-Zeichenpuffers
  boolean newSerialData = false;          // Flag: vollständige Nachricht auf serieller Schnittstelle empfangen
  char startMarker = ':';                 // Startzeichen für empfangene Kommandos und gesendete Antworten
  char endMarker = '#';                   // Endezeichen für empfangene Kommandos und gesendete Antworten
  const String cmdPseudo = "--";          // Pseudo-Kommandostring, wenn das Kommando nicht über die serielle Schnittstelle, sondern über die Website gekommen ist

  // Definitionen für den Status der Kuppel
  #define ACTION_NONE         0           // Keine Aktion
  #define ACTION_SEARCH_INDEX 1           // Suche Index
  #define ACTION_CALIBRATE    2           // Kalibrierung läuft
  #define ACTION_MANUAL_LEFT  3           // manuell nach links fahren
  #define ACTION_MANUAL_RIGHT 4           // manuell nach rechts fahren
  #define ACTION_GOTO_ANGLE   5           // auf einen bestimmten Winkel fahren
  #define ACTION_GOTO_ANGLE_F 6           // auf einen bestimmten Winkel fahren, Feineinstellung
  int actualAction = ACTION_NONE;

  // Definitionen für den Status der Motoren
  #define MOTOR_NONE          0           // beide Motoren aus
  #define MOTOR_LEFT_ON       1           // Motor Linksdrehung an
  #define MOTOR_RIGHT_ON      2           // Motor Rechtsdrehung an
  int motorState = MOTOR_NONE;
  #define DIR_LEFT            1           // Motor Drehrichtung links
  #define DIR_RIGHT           2           // Motor Drehrichtung rechts

  // Definitionen für den Status der Parkposition
  #define PARKPOS_NONE          0           // keine Parkposition
  #define PARKPOS_GOTO          1           // Kuppel fährt zur Parkposition
  #define PARKPOS_REACHED       2           // Kuppel hat Parkposition erreicht
  int parkPositionState = PARKPOS_NONE;

  // Definitionen für die Fehlerzustands-Verwaltung. Jedes Bit im errorState-Byte steht für einen von 8 möglichen Fehlerzuständen
  #define ERROR_INDEXTOCALIBR_DIFF     1           // beim Passieren der Indexmarke weicht der Zähler stark vom Kalibrierungswert ab
  #define ERROR_INDEXNOTLEAVED         2           // die Indexmarke bleibt belegt, obwohl der Kuppelmotor läuft
  #define ERROR_3                      4           // F3
  #define ERROR_4                      8           // F4
  #define ERROR_5                      16          // F5
  #define ERROR_6                      32          // F6
  #define ERROR_7                      64          // F7
  #define ERROR_8                      128         // F8
  uint8_t errorState = 0;

  // Variablen für die Steuerung der Drehbewegungen der Kuppel
  bool indexFound = false;                // Indexmarke nach Neustart gefunden? j/n
//  bool indexFindCancelled = false;        // Indexmarken-Suche wurde mit Stop-Kommando abgebrochen
  bool movingStopped = false;             // Mit dem Stopp-Kommando wurde die Kuppel angehalten. Wiederstart nur mit gegenteiligem Kommando!!!
  long indexTimestamp= -1;                // zum Entprellen eines Tasters im Testbetrieb (Millis-Zeitstempel)
  int targetCountsGoto;                   // Ziel-Impulszähler für ein laufendes Goto-Kommando
  int motorOffCountsGoto;                 // Ziel-Impulszähler für das Abschalten des Motors beim Goto-Kommando
  int calibrationPassingIndex;            // Bei Kalibrierung: Zähler für das Passieren der Indexmarke. 
                                          // Passage 1: Impulszähler löschen
                                          // Passage 2: Impulszähler als Kalibrierungswert übernehmen. Kalibrierung beenden

  // Timerdefinitionen, benutzt in Funktion "handleTimer"
  const unsigned long pollMillis=5;       // Poll-Zeit der Sensor-Eingaenge in Millisekunden 
  unsigned long lastMillis;               // letzter gespeicherter Millisekundenzähler
  
  const int timerUpdateGuiConst= 1000 / pollMillis; // Update der Zustandsanzeigen auf LCD + Website: Dauer in ms = Wert / pollMillis
  int       timerUpdateGui=timerUpdateGuiConst;     // lfd Counter für Aufruf des Updates der Zustandsanzeigen  

  const int timerMotorPauseConst= 1000 / pollMillis;// Pausezeit für den Motor nach dem Ausschalten: Dauer in ms = Wert / pollMillis
  int       timerMotorPause=0;                      // lfd Counter 

  const int timerIndexLeaveConst=500 / pollMillis;  // Überwachung, dass Indexmarke verlassen wird: Dauer in ms = Wert / pollMillis
  int       timerIndexLeave=0;                      // lfd Counter 

  // Beginn: Persistent zu haltende Konfigurationsdaten
    // Anzahl der Impulse für eine 360-Grad-Drehung. Wird bei der Kalibrierung ermittelt.
    #define EEADDR_COUNTER360DEGR 0        // EEPROM-Adresse für den Kalibrierungswert
    int counter360Degr = 1533;
  // Ende: Persistent zu haltende Konfigurationsdaten

  // Beginn: Konfigurationsdaten  
    // Ist Kalibrierung erfolgt j/n
    bool calibrated = false;
    // Offset zwischen der Indexmarke und der Nordrichtung in Grad
    int offsetIndexToNorthDegr = 190;
    // Die Parkposition soll knapp neben der Indexmarke liegen. Angabe in Grad
    int parkPositionDegr = 180;
    // Der Abschaltbefehl für den Kuppelmotor muss kurz vor Erreichen der Zielposition erteilt werden. Anzahl der Impulse.
    int motorOffPreCounts = 5;
    // Nach dem Einschalten des Kuppelmotors muss sich innerhalb dieser Überwachungszeit der Counter um mindestens 2 (??) Einheiten geändert haben. 
    // Bei Zeitablauf wird von einer Verklemmung der Kuppel ausgegangen und der Motor sicherheitshalber abgeschaltet. 
    int motorOnTimerMillis = 500; 
    // Toleranzwert für die Differenz des Zählerwertes zu Null, wenn die Indexmarke überfahren wird. Ist die Abweichung größer, erfolgt Fehlermeldung
    int tolCntToIndex = 5;
  // Ende: Konfigurationsdaten

//  ---------   Objekt-Instanzen erzeugen  ------------------
  NanoESP nanoesp = NanoESP();
  NanoESP_HTTP http = NanoESP_HTTP(nanoesp);

  // Über den I2C-Bus ist eine LCD-Anzeige mit 4x20 Zeichen angeschlossen
  LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 20 chars and 4 line display

//  ==========    Beginn: Setup   ==============================================================================================================
void setup() 
{
  // Serielle Schnittstelle initialisieren
  // Wird verwendet für Verbindung zum Kuppel-Client und für Monitor-Ausgaben
  // Baudrate 9600 gemäß Schnittstellen-Definition zum Kuppel-Client!!
  Serial.begin(9600);
  nanoesp.init();
  
  //Set Access Point AP
  nanoesp.println(F("AT+CIPSTA=\"192.168.10.12\""));      //AP
  nanoesp.configWifi(ACCESSPOINT, "Kuppel-10-12", "");    //AP
 
  //SET IP, SSID, PW
  nanoesp.println(F("AT+CIPSTA=\"192.168.178.62\""));     //Station
  nanoesp.configWifi(STATION, SSID, PASSWORD);            //

  //Dual Mode (Access Point + Station)
  nanoesp.configWifiMode(DUAL);

  // Wait till Connected to WiFi
  if (nanoesp.wifiConnected()) {
    digitalWrite(LED_WLAN, HIGH);
  }
  else {
    SerialDebug("Error Connect SSID");
    digitalWrite(LED_WLAN, LOW);
  }
  
  //Start TCP Server
  if (nanoesp.startTcpServer(80)) {
    SerialDebug("TCP Server Activ");
  }
  else {
    SerialDebug(F("TCP Server Error"));
    digitalWrite(LED_WLAN, LOW);
  }

  // Einstellung für TCP-Server: Multiple TCP Connections
  nanoesp.println("AT+CIPMUX=1");

  //Print IP in Terminal
  SerialDebug(nanoesp.getIp());

  // LCD-Display initialisieren
  InitLCD();
  
  // Pullup-Widerstaende an den Eingangs-Pins aktivieren
  pinMode(SIGNAL1, INPUT_PULLUP);
  pinMode(SIGNAL2, INPUT_PULLUP);
  pinMode(INDEX01, INPUT_PULLUP);

  // Ausgangs-Pins initialisieren
  pinMode(MOTOR_L, OUTPUT);
  pinMode(MOTOR_R, OUTPUT);
  pinMode(LED_SIGNAL1, OUTPUT);
  pinMode(LED_SIGNAL2, OUTPUT);
  pinMode(LED_INDEX01, OUTPUT);

  // Daten aus EEPROM zurücklesen:
  // Kalibrierungswert: Anzahl Impulse für 360 Grad
  counter360Degr = getCounter360Degr();  
  calibrated = true;

  // Beide Motoren stoppen
  stopMotor(1);

  // Initialisierung: Indexmarke noch nicht gefunden
  indexFound = false;
  // Indexmarke suchen, falls noch nicht gefunden. Aktion erfolgt nur 1x nach Neustart
  startSearchIndex();    
  // Liste der SSE-Verbindungen vorbesetzen
  initIdtoSSE();   
  
  SerialDebug("Start: " + String(APP_NAME) + String(APP_VERSION));
  SerialDebug(F("========================================")); 

}
//  ==========   Ende: Setup   ==============================================================================================================


//  ==========   Beginn: Loop  ============================================================================================

void loop() 
{
  // Serielle Schnittstelle auslesen, Daten in Empfangspuffer übertragen, 
  // Start- und Endekennung auswerten, Vorliegen einer vollständigen Nachricht signalisieren.
  recvWithStartEndMarkers();

  // Vollständig vorliegende Nachricht von der seriellen Schnittstelle analysieren, 
  // Kommando feststellen, Kommando direkt ausführen oder Ausführung des Kommandos starten
  doCommand();  
  
  // Aktionen, die nicht in jedem Loop-Durchlauf, sondern nur im Pollzyklus aufgerufen werden sollen
  if ((millis() - lastMillis) > pollMillis)
  {
    lastMillis = millis();
    
    // Impulse zur Ermittlung der Relativposition auswerten
    readCounter();

    // Impulse der Indexmarke Ermittlung der Absolutposition auswerten
    readIndex();

    // Pruefen, ob neue HTTP-Anforderungen vorliegen. Und Anforderungen verarbeiten
    recvHTTPRequests();

    // In Abhängigkeit von der derzeit laufenden Kuppelaktion und Impulszählerstand die notwendigen Reaktionen auslösen
    ctrlMoving();

    // diverse Timer dekrementieren und bei Nulldurchgang entsprechend reagieren
    handleTimer();
    
  }
}
//  ==========    Ende: Loop  ======================================================================================



//  ==========    Beginn: Interne Funktionen  ============================================================================================

int getCounter360Degr() 
{
  return int(EEPROM.get(EEADDR_COUNTER360DEGR, counter360Degr));
}
void putCounter360Degr() 
{
  EEPROM.put(EEADDR_COUNTER360DEGR, counter360Degr);
}


// Impulse zur Ermittlung der Relativposition auswerten
void readCounter() 
{     
 /*
 Aufgabe:
 Impulse an 2 Digitaleingängen erfassen, einen Zaehler hoch- bzw herunterzaehlen.
 Die 2 Eingänge kommen zeitlich versetzt. Ihre Reihenfolge bestimmt die Zählrichtung.
                                                                        
            Zaehler inkrementieren:             Zaehler dekrementieren:
                                                                       
               +------+                               +------+         
               |      |                               |      |         
Signal1     ---+      +---------                ------+      +-----    
                                                                       
                  +------+                          +------+           
                  |      |                          |      |           
Signal2     ------+      +------                ----+      +-------    

Einfacher Zustandsautomat, der bei jeder der 4 Flanken vorwärts bzw rückwärts zählt.
Siehe Diagramm "Quadratur.gif". Quelle: http://www.lothar-miller.de/s9y/archives/53-Drehgeber,-Encoder,-Quadraturdecoder.html
*/             
  int val, valueSignal1, valueSignal2, valueSignals;  
  int newState;
  int delta;

  // Zustand von Signal1 und Signal2 einlesen, alle Bits ausser Bit0 loeschen
  // Die Eingaenge sind activ-low. Im idle-Zustand also high
  valueSignal1 = digitalRead(SIGNAL1) & 1;
  valueSignal2 = digitalRead(SIGNAL2) & 1;
  // Die beiden Bits verodern: Bit0=Signal2, Bit1=Signal1
  val = (valueSignal1 << 1) | valueSignal2;
  // Bitzustand invertieren, damit sie activ-high sind
  valueSignals = val ^ 3;  

  // Zustand der beiden Eingangssignale auf 2 LEDs 'spiegeln' (LED soll leuchten, wenn Kontakt geschlossen)
  digitalWrite(LED_SIGNAL1, valueSignal1);
  digitalWrite(LED_SIGNAL2, valueSignal2);

  delta=0;
  newState = oldStateSignals;
  switch(oldStateSignals) 
  {
    case STATE_00:
      if      (valueSignals == STATE_01)  {newState = STATE_01; delta = +1;}
      else if (valueSignals == STATE_10)  {newState = STATE_10; delta = -1;}
      break;
    case STATE_01:
      if      (valueSignals == STATE_11)  {newState = STATE_11; delta = +1;}
      else if (valueSignals == STATE_00)  {newState = STATE_00; delta = -1;}
      break;
    case STATE_11:
      if      (valueSignals == STATE_10)  {newState = STATE_10; delta = +1;}
      else if (valueSignals == STATE_01)  {newState = STATE_01; delta = -1;}
      break;
    case STATE_10:
      if      (valueSignals == STATE_00)  {newState = STATE_00; delta = +1;}
      else if (valueSignals == STATE_11)  {newState = STATE_11; delta = -1;}
      break;  
  }
  // Zaehler inkrementieren oder dekrementieren  
  counterUpDown += delta;
 
  // während der Kalibrierung darf das Rücksetzen keinesfalls erfolgen!!
  if (actualAction != ACTION_CALIBRATE) 
  {
    // Bei Rechtsdrehung: Zähler beim Überschreiten des Kalibrierungswertes wieder auf Null setzen
    if (counterUpDown > counter360Degr-1) counterUpDown = 0;

    // Bei Linksdrehung: Zähler beim Unterschreiten von Null wieder auf den Kalibrierungswert setzen
    if (counterUpDown < 0 ) counterUpDown = counter360Degr-1;
  }

  // neuen Zustand uebernehmen
  oldStateSignals = newState;

  // if (delta != 0) {
  //   SerialDebug ("Bits=" + String(valueSignals) + " newState=" + String(newState) + " Delta=" + String(delta) + " Counter=" + String(counterUpDown));  
  // }
}
// Reset Counter
void resetCounter() 
{
  counterUpDown = 0;
}

// Impulse der Indexmarke zur Ermittlung der Absolutposition auswerten
// Fall 1:
// Der Eingang ist activ-low. Im idle-Zustand also high.
// Es kommt also beim Erreichen der Indexmarke die Abfallflanke, beim Verlassen die Anstiegsflanke
// Fall 2:
// Der Eingang ist activ-high. Im idle-Zustand also low.
// Es kommt also beim Erreichen der Indexmarke die Anstiegsflanke, beim Verlassen die Abfallflanke

void readIndex() 
{
  // Fall 1: Indexmarke ist activ-low
  // #define INDEXTRUE  0
  // #define INDEXFALSE 1

  // Fall 2: Indexmarke ist activ-high
  #define INDEXTRUE  1
  #define INDEXFALSE 0
  
  int newStateIndex;

  // aktuellen Zustand einlesen
  newStateIndex = digitalRead(INDEX01) & 1;
  // und auf LED spiegeln
  if (newStateIndex == INDEXTRUE)
  {
    digitalWrite(LED_INDEX01, 0); // LED einschalten
  } else 
  {
    digitalWrite(LED_INDEX01, 1); // LED ausschalten
  }

  if (abs(millis() - indexTimestamp) > 5)     // Prellen unterdrücken
  {

    // Indexmarke erreicht?
    if ( (oldStateIndex == INDEXFALSE) && (newStateIndex == INDEXTRUE) ) 
    {
      
      SerialDebug("Indexmarke erkannt");
      
      // Beim Überfahren der Indexmarke im laufenden Betrieb: 
      // Impulszähler gegen den Kalibrierungswert auf Plausibilität prüfen
      // Achtung: Aufruf _vor_ "handleIndexSearch" und "handleCalibration", sonst meldet checkCalibration bei IndexSuche und Kalibrierung immer einen Fehler!
      checkCalibration();
      // Bei Index-Search: reagieren auf Passieren der Indexmarke
      handleIndexSearch();
      // Bei Kalibrierung: reagieren auf Passieren der Indexmarke
      handleCalibration();
      // Überwachen, dass auch die Anstiegsflanke wieder kommt
      setTimerIndexLeave();    
      
      // Zum Schluss kann jetzt der Impulszähler neu gesetzt werden
      // neuer Wert ist abhängig von der Drehrichtung:
      // Bei Rechtsdrehung: auf Null setzen
      // Bei Linkssdrehung: auf Maximum = Kalibrierungswert setzen      
      if (motorState == MOTOR_RIGHT_ON) {counterUpDown = 0; } else { counterUpDown = counter360Degr; }         
    }
    indexTimestamp = millis();    
    oldStateIndex = newStateIndex;   // wird (und zwar entprellt) bei beiden Flanken aktualisiert
  }
}

// Prüfen, ob Indexmarke erreicht ist
bool isIndexSet() 
{
  if (oldStateIndex == 0) return true;  // Indexmarke ist gesetzt
  return false;       // Indexmarke ist nicht gesetzt              
}

// Bei Index-Search: reagieren auf Passieren der Indexmarke
void handleIndexSearch() 
{  
  // lief die Index-Suche?
  if (actualAction == ACTION_SEARCH_INDEX) 
  {
    SerialDebug("handleIndexSearch:");
    // Kennung setzen: Indexmarke gefunden
    indexFound = true;
    // Kuppel-Motor anhalten
    stopMotor(2);
    // Neuer Aktions-Status: Keine Aktion mehr aktiv
    actualAction = ACTION_NONE;      
  }
}

// Bei Kalibrierung: reagieren auf Passieren der Indexmarke
void handleCalibration() 
{  
  // lief die Kalibrierung?
  if (actualAction == ACTION_CALIBRATE) 
  {
    calibrationPassingIndex++;
    SerialDebug("handleCalibration1: Pass" + String(calibrationPassingIndex));
    // Erstes Überfahren der Indexmarke?
    if (calibrationPassingIndex == 1)
    {
      // Keine besondere Aktion. Nur Impulszähler löschen. Passiert weiter unten.
    }
    // Zweites Überfahren der Indexmarke?
    if (calibrationPassingIndex == 2)
    {
      // Impulszähler als Kalibrierungswert übernehmen. Kalibrierung beenden
      counter360Degr = abs(counterUpDown);
      putCounter360Degr();

      SerialDebug("handleCalibration2: Pass" + String(calibrationPassingIndex) + ": " + String(counter360Degr));

      //Kennung setzen: Kalibrierung ist erfolgt
      calibrated = true;
      // Kuppel-Motor anhalten
      stopMotor(3);
      // Neuer Aktions-Status: Keine Aktion mehr aktiv
      actualAction = ACTION_NONE;        
    }
  }
}

// Beim Überfahren der Indexmarke im laufenden Betrieb: 
//   Impulszähler gegen den Kalibrierungswert auf Plausibilität prüfen
void checkCalibration() 
{
  // keine Prüfung während Index-Suche oder Kalibrierung
  if ( (actualAction != ACTION_SEARCH_INDEX) && (actualAction != ACTION_CALIBRATE) ) 
  {
    int val, k, kh;
    k = counter360Degr;
    kh = k >> 1;
    val = counterUpDown;
    if (val > kh) val = k - val;

    // Ist die Abweichung > Toleranzwert?
    if (val > tolCntToIndex)
    {
      // Fehlerzustand melden
      setErrorState(ERROR_INDEXTOCALIBR_DIFF);
      SerialDebug("ERROR-1");  
    }
  }
}

// In Abhängigkeit von der derzeit laufenden Kuppelaktion und Impulszählerstand die notwendigen Reaktionen auslösen
void ctrlMoving()
{                  
  // Aktivitäten in Abhängigkeit von dem momentanen Status der Kuppel
  if (actualAction == ACTION_GOTO_ANGLE)
  {
    // Ziel-Position für das Abschalten erreicht?
    if (counterUpDown == motorOffCountsGoto) 
    {
      // auf Feinabstimmung der Position umschalten
      actualAction = ACTION_GOTO_ANGLE_F;
//      SerialDebug("ctrlMoving: stpM");
      // Motor ausschalten
      stopMotor(4);
    }
  } else if (actualAction == ACTION_GOTO_ANGLE_F) 
  {
    // Zielposition per Feinabstimmung erreicht?
//    SerialDebug("ctrlMoving:");      
    if (counterUpDown == targetCountsGoto)
    {
      // Aktion beenden
      actualAction = ACTION_NONE;
      // Motor ausschalten
      stopMotor(5);

      // Ziel-Position ist erreicht.
      // War es das Anfahren der Parkposition?
      if (parkPositionState == PARKPOS_GOTO) 
      {
        // Status setzen: Parkposition ist erreicht
        parkPositionState = PARKPOS_REACHED;
      }
    }    
  }
}
// Funktionsweise:
// Nach jedem Start der Kuppelsteuerung muss sie 1x die Indexmarke suchen, um die Absolutposition der Kuppel bestimmen zu können.
// Die Kuppel soll im Regelfall nahe neben der Indexmarke geparkt worden sein, bevor die Spannung ausgeschaltet wird.
// Mit der automatischen Rechtsdrehung soll dann die Indexmarke recht schnell gefunden werden.
// Wurde die Kuppel nicht nahe der Indexmarke geparkt, dann ist im schlechtesten Fall eine Drehung um 360 Grad erforderlich,
// bevor die Indexmarke erkannt wird.
void startSearchIndex() 
{
  if ((!indexFound) && (!movingStopped))
  {
    // wenn die Suche noch nicht läuft, dann jetzt starten
    if (!actualAction == ACTION_SEARCH_INDEX) 
    {
      // RECHTSDREHUNG
      startMotor(DIR_RIGHT, ACTION_SEARCH_INDEX);
    }

  }
}

// Funktionsweise:
// Kuppel um mehr als 360 Grad drehen und dabei die Anzahl der Zähl-Impulse für 360 Grad ermitteln
// Die Indexmarke muss dabei 2-mal passiert werden.
// Bei der 1. Passage wird nur der Impulszähler rückgesetzt.
// Bei der 2. Passage wird der aktuelle Impulszählerstand übernommen als Kalibrierungswert für 360Grad. Die Kalibrierung ist damit beendet.
void startCalibration() 
{
  // Zähler für das Passieren der Indexmarke bei der Kalibrierung löschen
  calibrationPassingIndex = 0;

  SerialDebug("startCalibration: Pass" + String(calibrationPassingIndex));

  // Drehrichtung festlegen
  // Prinzip: Nähe des aktuellen Impulszählers zum bisherigen Kalibrierungswert bestimmen

  // immer mit RECHTSDREHUNG, damit nur aufsteigende Zählerwerte erzeugt werden können
  startMotor(DIR_RIGHT, ACTION_CALIBRATE);

  // // Zählerwert > halber Kalibrierungswert? Dann ist rechtsrum der kürzeste Weg
  // if (counterUpDown > (counter360Degr >> 1)) 
  // {
  //   // RECHTSDREHUNG
  //   startMotor(DIR_RIGHT, ACTION_CALIBRATE);
  // } else 
  // {
  //   // LINKSDREHUNG
  //   startMotor(DIR_LEFT, ACTION_CALIBRATE);
  // }
}

void startGotoAzimut(int angle) 
{
  // Aus dem gegebenen Azimut-Winkel den dazugehörigen Counterwert berechnen
  int targetC = calcAzimutToCounter(angle);
  startGotoCounter(targetC);
}

void startGotoCounter(int target) 
{
  int action, delta, dir, motOff;

  SerialDebug("startGotoCounter: " + String(target));
  
  // Ziel-Counterwert für Goto global speichern
  targetCountsGoto = target;

  // alle Parameter für den Weg zum Ziel berechnen
  //  a) Differenz-Zählerwert
  //  b) kürzesten Weg der Kuppel zum Ziel bestimmen (Links- oder Rechtsdrehung)
  //  c) Zählerwert, bei dem der Motor abgeschaltet werden muss
  calcFromToCounts(counterUpDown, target, &delta, &dir, &motOff);

  // Zählerstand, bei dem der Motor abgeschaltet werden muss
  motorOffCountsGoto = motOff;

  // Kuppelaktion, mit der das Ziel angefahren wird
  action = ACTION_GOTO_ANGLE; 
  // Wenn das Ziel sehr nahe an der akt. Position liegt: das Ziel nur noch mit der Feineinstellung anfahren!
  if (delta <= motorOffPreCounts) action = ACTION_GOTO_ANGLE_F;

  if (dir == DIR_RIGHT) startMotor(DIR_RIGHT, action);
  if (dir == DIR_LEFT ) startMotor(DIR_LEFT , action);
}

// Motor rechts oder links wird gestartet. Läuft unendlich bis zu einem Stopp-Kommando
void startMovingManual(int direction) 
{
  // Linksdrehung gefordert?
  if (direction == DIR_LEFT ) startMotor(direction, ACTION_MANUAL_LEFT );
  if (direction == DIR_RIGHT) startMotor(direction, ACTION_MANUAL_RIGHT);
}

// Motor rechts oder links starten, Grund des Starts in Parameter action übergeben
void startMotor(int direction, int action) 
{
  SerialDebug("startMotor: dir=" + String(direction) + " action=" + String(action));

  if (direction != MOTOR_NONE) 
  {
    // Linksdrehung gefordert?
    if (direction == DIR_LEFT) 
    {
      // Motor links einschalten
      digitalWrite(MOTOR_L, MOTOR_ON); 
      // Motorstatus ändern
      motorState = MOTOR_LEFT_ON;
    } else 
    {
      // Motor rechts einschalten
      digitalWrite(MOTOR_R, MOTOR_ON);
      // Motorstatus ändern
      motorState = MOTOR_RIGHT_ON;
    }
    // Kuppelstatus ändern
    actualAction = action;

    // Wenn die Kuppel bis jetzt in der Parkposition stand, dann verläßt sie jetzt diese Position
    if (parkPositionState == PARKPOS_REACHED) parkPositionState = PARKPOS_NONE;

    // Überwachung der Indexmarke aktivieren
    setTimerIndexLeave();
  } else 
  {
    SerialDebug("startMotor: dir=0???");
  }  
}

void stopMotor(uint8_t src) 
{
  SerialDebug("stopMotor:" + String(src));
  // Beide Motoren ausschalten
  digitalWrite(MOTOR_L, MOTOR_OFF);
  digitalWrite(MOTOR_R, MOTOR_OFF);
  
  // Motorstatus bleibt bis zum Ende der Motorpause unverändert. 

  // Kuppelstatus bleibt bis zum Ende der Motorpause unverändert. 
  // Er bleibt noch "in Bewegung"
  // Damit weiß der Kuppel-Client, dass er noch keinen anderen Bewegungsauftrag absetzen darf.

  // Steht der Motor bereits (und Wartezeit ist abgelaufen) ODER läuft die Wartezeit schon?
  if ( (motorState == MOTOR_NONE) || (getTimerMotorPause() != 0) )
  {
    // keine weitere Aktion erforderlich. Motor steht bereits oder Wartezeit läuft bereits
  } else 
  {  
    // Dem Motor (und der Kuppel) nach dem Ausschalten eine kleine Pause gönnen
    setTimerMotorPause();
  }
}

void handleTimer() 
{
  // Zustandsanzeigen im LCD-Display und in Website zyklisch aktualisieren
  if (timerUpdateGui != 0) 
  {
    timerUpdateGui--;
    // 
    if (timerUpdateGui == 0)
    {
      // Zustandsanzeigen im LCD-Display und in Website aktualisieren
      updateDisplay();
      updateWebsite();

      // Timer sofort wieder neu aufziehen
      timerUpdateGui = timerUpdateGuiConst;      
    }    
  }
  
  // Motor Pausen-Timer
  handleTimerMotorPause();

  // Indexmarke überwachen
  handleTimerIndexLeave();
}

int handleTimerMotorPause() 
{
  // Dem Motor (und der Kuppel) nach dem Ausschalten eine kleine Pause gönnen
  if (timerMotorPause != 0) 
  {
    timerMotorPause--;
    // 
    if (timerMotorPause == 0)
    {
      if (actualAction == ACTION_GOTO_ANGLE_F) 
      {
        if ((counterUpDown != targetCountsGoto) && (!movingStopped))
        {
          // Ziel-Position ist noch nicht erreicht.
          // nochmal nachsteuern
          SerialDebug("handleTimerMotorPause: stGC");
          startGotoCounter(targetCountsGoto);
        } 
      } else 
      {
        // Motorpause ist beendet. Stopzustand ist erreicht. 

        // Motorstatus neu setzen
        motorState = MOTOR_NONE;        
        // Kuppelstatus neu setzen
        actualAction = ACTION_NONE;
        // SerialDebug("handleTimerMotorPause: End");
      }
    }    
  }
}

void setTimerMotorPause() 
{
  // Dem Motor (und der Kuppel) nach dem Ausschalten eine kleine Pause gönnen
  timerMotorPause = timerMotorPauseConst;
}

int getTimerMotorPause() 
{
  return timerMotorPause;
}

// Indexmarke überwachen
int handleTimerIndexLeave() 
{
  // läuft der Timer?
  if (timerIndexLeave != 0) 
  {
    timerIndexLeave--;
    // 
    if (timerIndexLeave == 0)
    {
      // Kuppel darf nicht mehr auf Indexmarke stehen
      // Indexmarke nicht verlassen?
      if (isIndexSet())
      {
        // Fehlerzustand setzen / melden
        setErrorState(ERROR_INDEXNOTLEAVED);
        SerialDebug("ERROR-2");  
      } else 
      {
        // SerialDebug("handleTimerIndexLeave: ok");        
      }
    }    
  }
}

// Indexmarke überwachen
void setTimerIndexLeave() 
{
//  SerialDebug("setTIL-1");  
  // läuft eine manuelle Bewegung oder eine Goto-Bewegung?
  if ( (actualAction == ACTION_MANUAL_LEFT) || (actualAction == ACTION_MANUAL_RIGHT) || (actualAction == ACTION_GOTO_ANGLE) ) 
  {
//    SerialDebug("setTIL-2");  
    // Indexmarke noch gesetzt?
    if (isIndexSet() == true) 
    {
//      SerialDebug("setTIL-3");  
      // Überwachung auf Verlassen der Indexmarke starten
      timerIndexLeave = timerIndexLeaveConst;
    }
  }
}

// Weg berechnen zwischen Start- und Ziel-Zählerwert
void calcFromToCounts(int startCount, int targetCount, int *delta, int *direction, int *motorOff) 
/*
Input-Parameter:
startcount  = Start-Zählerwert
Targetcount = Ziel-Zählerwert
Output-Parameter: 
delta     = Differenz zwischen Start und Ziel
direction = kürzeste Drehrichtung vom Start zum Ziel
motorOff  = Zählerwert, bei dem der Motor abgeschaltet werden soll
*/
{
  int diff, off;  
  int s = startCount;
  int t = targetCount;
  int k = counter360Degr;  // kalibrierungswert
  int kh = k >> 1;         // kalibrierungswert-Halbe = durch 2
  
  // Start im Halbkreis vor der Indexmarke UND Ziel im Halbkreis nach der Indexmarke?
  // --> dann den Wertebereich um 180 Grad (=halber Kailibrierungswert) nach links verschieben 
  if( (s > kh) && (t < kh) ) {s -= kh; t += kh;}
  // Differenz bilden: Ziel minus Start
  diff = t - s;
  // wenn Differenz > 180 Grad: Wert um 180 Grad reduzieren
  if (diff > kh) diff -= k;

  *direction = DIR_RIGHT;   // Vorbesetzung: Rechtsdrehung
  // Differnz negativ --> dann wird es eine Linksdrehung
  if (diff < 0) *direction = DIR_LEFT;
  // Betrag der Differenz ist der gesuchte Abstand zwischen Start und Ziel
  *delta = abs(diff);
  
  // Zählerwert, bei dem der Motor abgeschaltet werden soll, berechnen
  t = targetCount;
  if (*direction == DIR_RIGHT) 
  {
    // RECHTSDREHUNG: berechnen, bei welchem Zählerstand der Motor abgeschaltet werden muss
    off = t - motorOffPreCounts;
    if (off < 0) off += k;    
  } else 
  {
    // LINKSDREHUNG: berechnen, bei welchem Zählerstand der Motor abgeschaltet werden muss
    off = t + motorOffPreCounts;
    if (off >= k) off -= k;
  }
  *motorOff = off;
}

// Testfunktion: Weg berechnen zwischen Start- und Ziel-Zählerwert
// void testFromToCounts() 
// {
//   int diff, dir, off, x, y;
//   int rows = 10;
//   int columns = 2;
//   int testInts[rows][columns] = { {900,100}, {100,900}, {950,900}, {900,950}, {3,997}, {997,3}, {5,995}, {995,5}, {245,750}, {750,245} };
//   counter360Degr = 1000;      // Testdaten basieren auf der Voraussetzung: Kalibrierungswert = 1000 !!!

//    for ( int i = 0; i < rows; ++i ) {
//       // loop through columns of current row
//       x = testInts[i][0];
//       y = testInts[i][1];
//       calcFromToCounts( x, y, &diff, &dir, &off);
//       Serial.println("Start=" + String(x) + ", Ziel=" + String(y) + ", Diff=" + String(diff) + ", Dir=" + String(dir) + ", off=" + String(off));      
//    } 
// }

void setErrorState(uint8_t error) 
{
  errorState |= error;
}

void delErrorState(uint8_t error) 
{
  errorState = errorState & (~error);
}

void resetErrorState() 
{
  errorState = 0;
}

uint8_t getErrorStateB() 
{
  return errorState;
}

String getErrorStateS() 
{
  String strError;
  uint8_t err, mask;
  char letter;

  strError = "";
  err = getErrorStateB();
   for ( int i = 7; i >=0; i-- ) 
   {
    mask = 1 << i;
    letter = '0';
    if ( (mask & err) != 0) letter = '1'; 
    strError = strError + letter; 
   }
  return strError;  
}

// uint8_t testErrorStateB() 
// {
//   setErrorState(2);
//   setErrorState(64);
//   setErrorState(128);
//   setErrorState(1);
//   delErrorState(1);
//   Serial.println("Errorstate = " + getErrorStateS());
// }