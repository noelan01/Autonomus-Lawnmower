# Autonomus-Lawnmower
## Kandidatarbete autonom gräsklippare 2024

Följ först Husqvarnas interna dokument för att göra HRP2 noden körbar. Beroendes på om det gamla SD-kortet sitter kvar eller ej i Raspberry PI:n kan det vara så att inget i Husqvarnas dokument behövs göras. Vi använde ej WI-FI dongles enligt deras instruktioner så det är inte nödvändigt.

För att kunna arbeta med Raspberry PI:n måste appen Automower laddas ner och ett konto måste göras. Klipparen som ska kopplas till är en Husqvarna 550 EPOS och koden är för tillfället 0000 eller 1111 i appen för att koppla till klipparen.

## Struktur:
Projektet är uppbyggt i 3 ROS2 noder varav en är husqvarnas egna HRP2 noder.
De egenskrivna noderna är **lawnmower_control** noden som sköter styrningen av gräsklipparen och **coordinate_init** noden som initierar koordinatsystemet.

Den mesta koden som är skriven ligger under src/, UserInputServer/, scripts/ och coordinate_init/. Koden skriven av Husqvarna ligger under hrp2-p2z-open-dist/ och ska ej ändras.

## Uppkoppling till gräsklipparen samt start av ROS noderna:
1. Sätt igång klipparen och koppla telefonen till klipparen med Automower appen (kod 1111 eller 0000).
2. använd ssh för att koppla till datorn:
    1. koppla till samma nätverk som klipparen.
    2. använd en terminal eller VS codes ssh extension. Skriv in "ssh mower@ubuntu.local" (alt. "ssh mower@192.168.1.110" för CASELAB wifi) (ssh hostname@ip) och Ros2mower som lösenord.
3. Byt till Autonomous-Lawnmower mappen med "cd Autonomous-Lawnmower/"
4. starta hrp2 noden genom att köra "source scripts/hrp2-autostart.sh" och skriv in lösenordet "Ros2mower" vid behov.
5. starta lawnmower-control noden genom att köra "source scripts/main_autostart.sh".
6. starta coord-noden genom att köra "source scripts/coord_main_autostart.sh".

Autostart scripten skrevs för att underlätta starten av ROS2 noderna men det går lika bra att skriva in innehållet av de filerna för hand i terminalen istället. 

Eftersom den skrivna regleringen av gräsklipparen baseras på mätningar från RTK-stationen så måste an aktiv koppling till den etableras för att det ska funka. Klipparen kopplas automatiskt till en RTK station då den är i närheten av den. Men det kan ta upp till 5 minuter då man kopplar till en ny RTK-station. Vi gjorde våra tester på mossens gräsplan som är IK virgos hemmaplan eftersom det fanns en RTK-station monterad där sen tidigare.

Vi gjorde en fin ställning till RTK stationen som vi inte använde eftersom vi höll oss till mossen. Men om man inte har tillgång till mossen är det ett bra alternativ (om det sparats).

En adapter till en husqvarna laddare ska finnas och kopplas in i en kabel som sticker ut fram undersida på klipparen, för att ladda.
För att kunna kunna köra klipparen utan kåpa är en magnet fasttejpad med silvertejp i fram, står det "krock" i appen kan man testa att flytta på denna.
