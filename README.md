# STM32AnalogExtension
Für die Bereichsübergreifende Projekte unserer Schule solte eine Erweiterung für das STM32F051Discovery Kit erstellt werden. 

Analog Erweiterung 

ToDo:
-BNC Buchse



Anforderungen: 


Dokumentationsbeiträge von Philipp

Weitere Anschlüsse:
Als zusätzliche Anschlüsse wurden BNC Buchsen ausgewählt. Durch ihre Stndrdisierung und often gebrauch eignen sie sich um weitere Signale anzulegen und zu messen. 

 # Schaltungsaufbau
## ADC-Serial Interface
Der ADC besitzt ein Serial Peripherie Interface (SPI). Über diess kann mit dem Microntroller Komunizieren. 
Damit die Peripherie nicht per Bit-Banging angesteuert werden muss, müssen sie beim Microcontroller mit bei einem SPI fähigen Port verbunden werden.
# Dimensionierung
Als Temperatursensor wird ein TP100 verwendet. Nach spezifikation hat der Temperatursensor bei 0°C 100R Widerstand. Je nach Temperaturänderung verändert sich der Widerstand nach einer Spezifizierten Kurve. 
Als Messstrom wird bei 0°C 0.5mA verwendet. Da sich der Temperatursensor selbst erwärmt darf dieser Messstrom nicht zu hoch gewählt werden. 

Um den Geflossenen Strom durch den ADC zu messen wird über einem Refererenzwiderstand die Spannung gemessen welche vom Messstrom erzeugt wird. 
