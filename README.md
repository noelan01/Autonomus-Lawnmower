# Autonomus-Lawnmower
Kandidatarbete autonom gräsklippare 2024

Följ först Husqvarnas interna dokument för att göra HRP2 noden körbar. Vi använde ej WI-FI dongles enligt deras instruktioner så det är inte nödvändigt.

Struktur:
    Projektet är uppbyggt i 3 ROS2 noder varav en är husqvarnas egna HRP2 noder.
    De egenskrivna noderna är lawnmower_contorl noden som sköter styrningen av gräsklipparen och coordinate_init noden som initierar koordinatsystemet.

    Den mesta koden som är skriven ligger under src/, UserInputServer/, scripts/ och coordinate_init/. Koden skriven av Husqvarna ligger under hrp2-p2z-open-dist/ och ska ej ändras.

Uppkoppling till gräsklipparen samt start av ROS noderna:
    1. Sätt igång klipparen och koppla telefonen till klipparen med Automower appen.
    2. använd ssh för att koppla till datorn:
        a. koppla till samma nätverk som klipparen.
        b. använd en terminal eller VS codes ssh extension. Skriv in "ssh mower@ubuntu.local" (ssh hostname@ip) och Ros2mower som lösenord.
    3. byt till Autonomous-Lawnmower mappen med "cd Autonomous-Lawnmower/"
    4. starta hrp2 noden genom att köra "source scripts/hrp2-autostart.sh" och skriv in lösenordet Ros2mower vid behov.
    5. starta lawnmower-control noden genom att köra "source scripts/main_autostart.sh".
    6. starta coord-noden genom att köra "source scripts/coord_main_autostart.sh".

Autostart scripten skrevs för att underlätta starten av ROS2 noderna men det går lika bra att skriva in innehållet av de filerna för hand i terminalen istället.

Eftersom den skrivna regleringen av gräsklipparen baseras på mätningar från RTK-stationen så måste an aktiv koppling till den etableras för att det ska funka. Klipparen kopplas automatiskt till en RTK station då den är i närheten av den. Men det kan ta upp till 5 minuter då man kopplar till en ny RTK-station. Vi gjorde våra tester på mossens gräsplan som är IK virgos hemmaplan eftersom det fanns en RTK-station monterad där sen tidigare.

