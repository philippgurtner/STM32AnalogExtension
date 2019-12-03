# V0.2
- Pinänderung: 
  - CS (PA7) -> PF5 
  - MOSI (PA5) -> PA7
  - SCLK (PA4) -> PA5	
- AINCOM auf GND Legen !! --> Für single-ended measurements

If the DRDY output is not used, leave the DRDY pin unconnected or tie the DRDY pin to IOVDD using a weak pullup resistor

Hinzufügen von GPIO ausgang --> Auslesen Digitaler Signale (oder Ausgeben von Daten)

Full scale Range = +- Vref / Gain --> V ref muss angelegt werden sonst steht nur 2.5V intern zur verfügung (10.8.3, s.88 Datenblatt)

V ref anhängen (ref0 plus auf 3.3V, n auf gnd)

? BNC eingänge abschwächer einbauen mit 41.8kR --> 47kR oder so und 2.2kR (de womer scho hei) --> mit switch ischaltbar (denn sind igangsspannige bis 50V mglich (2.5V Interne referenz) --> Ua = 2.23V 
 
 Doku: 
 Berechnunt PT 100 noch zu vereinfachen Upt100= ADC0-2xADC1 + adc2 
 
 Nach Callendar-Van Dusen siet die berechnung folgend aus: 
 
 
 !!!!!!!!!!!!!!!!! Kalibirieren !!!! like in ?Nesselwangen
 
