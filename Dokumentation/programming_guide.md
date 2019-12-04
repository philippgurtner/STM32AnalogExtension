# Programmierguide
Als IDE wird die Opensoure-Freeware von STM verwendet. Sie kann unter ............... heruntergeladen werden. 
Die IDE beinhaltet folende module:

- STM32CubeMX	(Zum Konfigurieren der Hardware)
- Atollic Studio (Texteditor)
- ACR6 Debugger ???

## Konfiguration

Um das ADCBoard in ein Projekt einzubinden muss, bei einem STM32F0Discovery, folgende Konfigurationen gemacht werden. 

- Das Board kann beim erstellen des neuen STM32CubeMX Projekt ausgewählt werden

- Pinout & Configuration
	- Pinkonfiguration (Wichtig! Die User labels müssen genau gleich benannt werden --> HAL Namen wurden in Library übernommen)
		- PF5: ADC_CS
		- PC4: ADC_START
		- PC5: ADC_RESET
		- PB0: ADC_DRDY
	- Modul Connectivity>SPI1 hinzufügen
		- Data Size: 8Bit
		- First Bit: MSB First
		- Clock Polarity (CPOL): LOW
		- Clock Phase (CPHA): 2 Edge
- Project Manger
	- Code Generator
		- Generate Files: Generate peripheral initialization as a pair... (Erzeugt für jedes Modul eine eigene Datei. So Wird die main.c übersichtlicher)

Über das Zahnrad Symbol (Device Confiuration Tool Code Generation) kann der C-Code generiert werden. 

Beim weiteren Programmieren ist es wichtig sich an die vorgegebenen Schranken für Code zu halten. So kann noch immer eine neue Version generiert werden, ohne das handgeschriebener Code verlohren geht. 

## Programmablaf (nach [S.90](../Herstellerdaten/ads124s06.pdf#page=90) Datenblatt)

Als erstes muss der ADC Initialisiert werden. Dies geschieht über folgende komandos. Auf [Seite 90](../Herstellerdaten/ads124s06.pdf#page=90) des Datenblatts ist ein Pseudocode Beispiel zu finden, dieses wird hier Ebenfalls, wenn auch nicht gleich Detailliert behandelt. 















### Schleife

- Warten bis !DRDY 0 ist, also ADC fertig konvertiert hat. Dazu gibt es folgende Möglichkeiten
  - Pullen des Pins
  - Besser, über einen Interrupt der Ausgelöst wird wenn der Pin eine Negative Flanke hat. 
- ADC Adressieren, !CS = 0 
- Mindestens td(CSSC) Warten, dass ADC Bereit ist zu komunizieren. 
- Daten-lese-Komando senden, RDATA
- Empfangen von 24Bit, somit 24 Positive Flanken an SCLK ausgeben
- Mindestens td(SCCS) warten
- Serielles Interface zurücksetzen, !CS = 1



#### Code Beispiel



#### Prgrammieridee

Single read mode: daten werden nur dann gelesen wenn dies vom UC per start befehl geforderd wird, so kann auch jedes mal INPMUX neu gesetzt werden, dann gestartet werden. Sonst liest adc immer Kontinuierlich daten, auf dem gesetzten Kanal...






# Philipp Notizen zur Programmierstruktur
## Temperatur lesen // nicht ganz schön wegen langer ISR, dafür einfach anwendbar #KISS 
- Flag new temperature value = 0
- Setzen von Input auf AIN0
- ISR (Data Ready) 
	- Daten lesen Per command oder direkt (ehnter direkt da schon in der ISR)
	- Daten in variable (array) speichern

wenn noch nicht alle gelesen, dann:  
- Nächster input setzen (AIN1)

- if temperature value == 0 && alle daten gelesen 	
	auswerten der daten, flag setzen

3 Funktionen für die temperatur --> Buffern??

- if(newtemperature available)
	gettempperature();
- read temperature ---> muss in main immer ausfüren







