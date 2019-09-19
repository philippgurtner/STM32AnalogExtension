# Programmierguide
## Konfiguration
- Pinout & Configuration
	- Pinkonfiguration
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
		- Generate Files: Generate peripheral initialization as a pair...

## Programmablaf (nach [S.90](ads124s06.pdf#page=90) Datenblatt)

Als erstes muss der ADC Initialisiert werden. Dies geschieht über folgende komandos. Auf [Seite 90](ads124s06.pdf#page=90) des Datenblatts ist ein Pseudocode Beispiel zu finden, dieses wird hier Ebenfalls, wenn auch nicht gleich Detailliert behandelt. 

### Initialisieren

- Read Status Register um zu überprüfen ob der ADC Komunikationsbereit ist:
	
	
	```c
	HAL_GPIO_WritePin(ADC_CS_GPIO_Port,ADC_CS_Pin,0);
	
	Hal_SPI_Tramsmit()
	    
	    
	```
	
- Setzen aller Einstellungen

  - Setzen in Single-Shot Conversation Mode
  - Mode Bit, on 0x04

- Start Kommando (0x08) Senden, der ADC Konvertiert ab dann kontinuierlich.

- Serielles Interface zurücksetzen, !CS = 1

  - ```c
    HAL_GPIO_WritePin(ADC_CS_GPIO_Port,ADC_CS_Pin,0);
    ```

#### Code Beispiel





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





0010 0001  000n nnnn(3)



