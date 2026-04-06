# IA Laborator 06 - Robotică (Pioneer P3-DX)
Proiectul conține implementarea cerințelor pentru controlul unui robot cu tracțiune diferențială în simulatorul CoppeliaSim, folosind ZMQ Remote API din Python.

## Structura Repository-ului
Cerințe de bază (3.1 - 3.6): Scripturi pentru conectare, control open-loop, citire senzori, oprire la obstacol, Braitenberg vehicle și wall-following.

## Tema A (Evitare cu recuperare): Implementarea unei mașini de stări pentru navigare autonomă și ieșire din blocaje.

Implementare Tema A
Pentru Tema A, am extins comportamentul de evitare folosind o logică de tip State Machine și un sistem de siguranță pentru cazurile în care senzorii nu detectează corect geometria obiectelor (ex: picioarele toaletelor sau scaunelor).

1. Mașina de Stări (State Machine)
Robotul trece prin 3 stări principale pentru a evita blocajele permanente:

FORWARD: Robotul înaintează folosind logica Braitenberg pentru evitare fluidă a obstacolelor.

BACKWARD: Dacă senzorii centrali detectează un obiect sub pragul de 0.15m, robotul intră automat în marșarier timp de 1 secundă.

TURNING: După marșarier, robotul execută o rotație random (stânga sau dreapta) pentru a schimba unghiul de atac și a relua mersul înainte.

2. Sistem Anti-Stuck (Detecție Blocaj Fizic)
Deoarece senzorii ultrasonici pot avea "blind spots" la geometrii complexe, am adăugat o verificare bazată pe coordonatele reale ale robotului (sim.getObjectPosition):

La fiecare 2 secunde, scriptul compară poziția curentă cu cea anterioară.

Dacă distanța parcursă este mai mică de 5cm (deși motoarele sunt active), se declanșează manevra de recuperare (BACKWARD + TURNING).

## Cum se rulează
Se deschide CoppeliaSim și se încarcă scena pioneer_lab06.ttt.

Se instalează dependențele:

```
pip install requirements.txt
```
Se pornește simularea (Play) și se rulează scriptul:

```
python tema/tema_a_recuperare.py
```
📊 Observații Experimentale
Am reglat coeficientul K_P la valoarea 1.0 și K_SENSOR la 4.0 pentru a obține un echilibru între viteza de deplasare și stabilitatea virajelor.

Logica de recuperare bazată pe coordonate (X, Y) s-a dovedit mult mai eficientă decât cea bazată exclusiv pe senzori în arenele aglomerate.
