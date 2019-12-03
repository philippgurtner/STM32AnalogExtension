# Variablen Viewer installieren (debuggen)
Das STMStudio ermöglicht es eine *debug Datei zu Importieren und diese Laufen zu lassen. So können mehrere globale Variablen angesehen werden. So kann eine Langzeitanalyse des Verhalten gemacht werden. 

- STM-STUDIO-STM32 (von https://www.st.com/en/development-tools/stm-studio-stm32.html ) installieren (benötigt Java https://www.java.com/de/download/win10.jsp)
- STM Studio öffnen
- "File" > "Import variables from executable" öffnen
	- "Executable file" .elf Datei suchen (im STM Projekt unter "Debugger")
	- (Warnung Ignorieren)
	- Variable zum Monitoren auswählen 
	- "Import" , "Close"
- Variable (Display Variables) in Viewer hineinziehen (Drag and Drop)
- Start Knopf drücken (Debugger aus IDE darf nicht gleichzeitig aktiv sein)
- Im Viewer: Rechtsklick, "Autorange" > "Both Axis"
- Eventuell als Balken darstellen, maximum wert beachten (über hex ganz einfach manuell einstellbar)

![Import variables](C:\Users\phili\Documents\GitHub\STM32AnalogExtension\Dokumentation\Bilder\Importvariablesfromexecutable.PNG)

![Variablenviewer als Balkenanzeige](C:\Users\phili\Documents\GitHub\STM32AnalogExtension\Dokumentation\Bilder\Balkendiagramm.PNG)