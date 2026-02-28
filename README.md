# CEPS32
CEPS (Control Embedded Platform System): ESP32 Web gateway controlling mBot via I²C — inspired by the Ophiocordyceps “zombie fungus” concept.
# CEPS32 -- Control Embedded Platform System

## Ophiocordyceps32 Architecture

**CEPS (Control Embedded Platform System)** è l'architettura embedded
che implementa un sistema di controllo remoto Web per mBot utilizzando
un ESP32 come gateway I²C.

Il nome del progetto prende ispirazione dal fungo *Ophiocordyceps
unilateralis*, noto come "zombie fungus", capace di controllare il
comportamento dell'ospite.\
Allo stesso modo, CEPS32 introduce un livello di controllo remoto che
governa il sistema robotico fisico.

------------------------------------------------------------------------

# Struttura del Repository

Il progetto è composto da tre moduli principali:

## CEPS32_esp32.cpp

Firmware ESP32: - WebServer HTTP - API REST JSON - Master I²C - Gateway
rete → bus embedded - Gestione safety e configurazione

## CEPS32_Mbot_v1.cpp

Firmware mBot (mCore): - Slave I²C (indirizzo 0x12) - Controllo motori
DC - Lettura sensore ultrasuoni - Logica di sicurezza con isteresi

## CEPS32_pythonClient.py

Client Python: - Utilizzo libreria `requests` - Controllo remoto via API
HTTP - Test motori e lettura distanza

------------------------------------------------------------------------

# Architettura Logica

Python / Browser\
│\
│ HTTP (JSON)\
▼\
\[ ESP32 \]\
WebServer + API\
│\
│ I²C (100 kHz)\
▼\
\[ mBot - mCore \]\
Motori + Sensore Ultrasuoni

------------------------------------------------------------------------

# Comunicazione I²C

-   ESP32 opera come Master
-   mBot opera come Slave
-   Indirizzo slave: 0x12
-   Frequenza consigliata: 100 kHz

È raccomandato l'utilizzo di un level shifter I²C bidirezionale (BSS138)
per la conversione 3.3V ↔ 5V.

------------------------------------------------------------------------

# API HTTP Principali

Base URL:

http://`<IP_ESP32>`{=html}/

Endpoint:

-   /api/motors?left=VAL&right=VAL
-   /api/stop
-   /api/distance
-   /api/status
-   /api/config
-   /api/safety

------------------------------------------------------------------------

# Esempio Python

``` python
import requests

BASE = "http://IP_ESP32"

requests.get(f"{BASE}/api/motors", params={"left":120, "right":120})
print(requests.get(f"{BASE}/api/distance").json())
requests.get(f"{BASE}/api/stop")
```

------------------------------------------------------------------------

# Finalità Didattiche

CEPS32 permette di sperimentare:

-   architetture embedded distribuite
-   comunicazione master/slave I²C
-   integrazione hardware/software
-   sistemi di controllo remoto Web
-   robotica educativa

------------------------------------------------------------------------

# Evoluzioni Future

-   Endpoint aggregato /api/state
-   Autenticazione API
-   Dashboard Web avanzata
-   Integrazione MQTT
-   Controllo multi‑robot

------------------------------------------------------------------------

CEPS32 rappresenta un modello modulare e scalabile di controllo embedded
remoto, dove il livello cyber governa il livello fisico, in perfetto
stile Ophiocordyceps.
