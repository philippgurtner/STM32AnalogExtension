# Einführung Picoscope

Das Picoscope ist ein Logicanalyser. Mit ihm ist es Möglich mehrere Digitale Signale zu Analysieren und die Daten auszuwerten. Um Herauszufinden ob die Serielle Kommunikation richtig Funktioniert kann man mit diesem Hilfsmittel einfacher messen und die Daten analysieren. 

## Vorbereitung

Um Das Picoscope an einem PC benutzen zu können sind einige Schritte notwendig. 

### Installieren der Software

Die Software kann auf der Webseite von Picosope unter > Downloads heruntergeladen werden. 
Hier muss die Genaue Modelbezeichung angegeben werden. (Wir verwendeten das Picoscope der Schule [*PicoScope 2206B MSO*](https://www.picotech.com/downloads/_lightbox/picoscope-6))

### Anschliessen der Digitalen Kanäle

Die Digitalen Kanäle befinden sich beim 16-Pin Stecker in der Mitte. Das Mitgelieferte Kabel dient um mit dem UC Board zu verbinden. 

### Einrichten der Software

1. Digitale Kanäle Gruppen

   1. Kanal Auswählen

   2. Kanal intelligent benennen

      ![Digitale Kanäle](..\Dokumentation\Bilder\PicoScope_Channelsettings.PNG)

2. Hinzufügen eines Seriellen Encoders:

   1. Werkzeuge > Serielle Entschlüsselung -> Erstellen > SPI >

![SPI Analyser MISO and MOSI](..\Dokumentation\Bilder\PicoScope_SPI_Settings.PNG)

3. Setzen des Triggers (In der untersten Leiste)
   1. Trigger: Einzeln
   2. In den Triggereinstellungen (Rechts vom Trigger) kann auf Digital > CS Pos Flanke gesetzt werden
   3. Start

## SPI Analyse

Mit den Empfangen Daten lässt sich verstehen was in SPI gesendet wird

Folgend im Code und Als Analyse:

MISO, Die Daten werden vom Master an den Slave gesendet. Die Rückgabe des Slaves ist nicht wichtig. 

```c
HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin,0);

uint8_t au8_txdata = {0x42,		// Register INPMUX_REG_ADDR (0x02) | Write command 0x40
                      0,		// Write length -1 = 0
                      0x3C};	// Data = ADS_P_AIN3 (0x30)+ ADS_N_AINCOM (0x0C)		
HAL_SPI_Transmit(&hspi1,au8_txdata,3,10);
HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin,1);
```

![SPI Set Channel](..\Dokumentation\Bilder\SPI_Analyse_Setchannel.PNG)