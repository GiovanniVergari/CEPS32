/*
    ============================================================
    mBot (mCore / ATmega328P) - I2C SLAVE "Bridge" per ESP32
    ============================================================

    OBIETTIVO DIDATTICO
    -------------------
    Questo firmware trasforma l'mBot in un "nodo" controllabile via I2C.
    L'ESP32 (master) può:
        - impostare la velocità dei motori (sinistro e destro)
        - leggere la distanza misurata dal sensore ad ultrasuoni
        - leggere uno status (misura valida, safety attiva, ecc.)
        - configurare opzioni (inversione motori, soglie, safety)

    ============================================================
    CABLAGGIO HARDWARE (ESP32 <-> mBot)
    ============================================================

    1) PORTE DA USARE SULLA mCore
       --------------------------
       Dalla serigrafia sulla sua scheda (foto):
          PORT 3 e PORT 4 riportano: "SCL SDA GND 5V"

       Questo significa che PORT 3 e PORT 4 sono connesse al BUS I2C hardware.
       Quindi l'ESP32 può essere collegato a:
          - PORT 3  oppure
          - PORT 4

       PORT 3 e PORT 4 sono in parallelo (stesso bus). Ne basta una.

    2) NUMERO DI COLLEGAMENTI
       ----------------------
       Minimo (consigliato con level shifter):
          - SDA
          - SCL
          - GND

       In pratica, usando un level shifter I2C bidirezionale 5V<->3.3V:
          Lato mBot (HV=5V): SDA, SCL, GND, 5V
          Lato ESP32 (LV=3.3V): SDA, SCL, GND, 3.3V

       NON alimentare l'ESP32 dal 5V dell'mBot a meno che non sia una scelta
       consapevole e verificata (assorbimenti + cadute con motori attivi).

    3) LEVEL SHIFTING (IMPORTANTE)
       ---------------------------
       mBot lavora a 5V, ESP32 a 3.3V.
       I2C usa pull-up: se le linee SDA/SCL sono tirate a 5V, l'ESP32 rischia.

       -> Soluzione raccomandata:
          usare un level shifter bidirezionale per I2C (tipico BSS138).

    ============================================================
    CABLAGGIO SENSORE ULTRASUONI
    ============================================================

    Il sensore ultrasuoni Makeblock si collega a UNA porta "normale" del robot,
    cioè tipicamente PORT_1 o PORT_2.

    Motivo:
      - PORT_3 e PORT_4 in questa revisione sono "orientate" a I2C (SCL/SDA),
        quindi conviene dedicarle al BUS (ESP32) e usare PORT_1/2 per ultrasuoni.

    In questo sketch usiamo di default:
      - Ultrasuoni su PORT_1

    Se lei lo collega a PORT_2 basta cambiare la riga:
      MeUltrasonicSensor ultraSensor(PORT_1);
    in:
      MeUltrasonicSensor ultraSensor(PORT_2);

    ============================================================
    PROTOCOLLO I2C (STILE REGISTRI)
    ============================================================

    Indirizzo I2C dello slave (mBot): 0x12 (configurabile)

    Regole:
      - Il master scrive 1 byte: REGISTRO (selezione)
      - Se il registro è scrivibile, può seguire un payload
      - Il master legge poi N byte: il mBot risponde con i dati del registro selezionato

    Endianness per int16/uint16:
      - Little-endian: LSB prima, MSB dopo

    Registri (R=read, W=write):
      0x00  ID/VERSIONE        (R) 2 bytes
      0x01  MOTORE SX SPEED    (R/W) int16  -255..255
      0x03  MOTORE DX SPEED    (R/W) int16  -255..255
      0x05  DISTANZA mm        (R) uint16   (0 = non valida)
      0x07  STATUS FLAGS       (R) uint8
      0x08  CONFIG FLAGS       (R/W) uint8  (invert motori, safety)
      0x09  SOGLIA STOP mm     (R/W) uint16 (per safety)
      0x0B  SOGLIA RIPARTI mm  (R/W) uint16 (isteresi per safety)

    STATUS FLAGS (bit):
      bit0 = distanza valida
      bit1 = safety attiva (funzione abilitata)
      bit2 = safety in stop (motori forzati a 0 per distanza troppo bassa)

    CONFIG FLAGS (bit):
      bit0 = inverti motore sinistro
      bit1 = inverti motore destro
      bit2 = abilita safety distanza

    Safety:
      - Se abilitata e la distanza scende sotto soglia_stop -> motori forzati 0
      - Per uscire dallo stop: distanza deve superare soglia_riparti (isteresi)

    ============================================================
*/

#include <Wire.h>
#include <MeMCore.h>

/* ===========================
   CONFIG I2C
   =========================== */

static const uint8_t I2C_SLAVE_ADDRESS = 0x12;

/* ===========================
   REGISTRI
   =========================== */

static const uint8_t REG_ID_VERSION      = 0x00;
static const uint8_t REG_MOTOR_LEFT      = 0x01;
static const uint8_t REG_MOTOR_RIGHT     = 0x03;
static const uint8_t REG_DISTANCE_MM     = 0x05;
static const uint8_t REG_STATUS_FLAGS    = 0x07;
static const uint8_t REG_CONFIG_FLAGS    = 0x08;
static const uint8_t REG_STOP_MM         = 0x09;
static const uint8_t REG_RESUME_MM       = 0x0B;

/* ===========================
   FLAG BIT
   =========================== */

static const uint8_t STATUS_DISTANCE_VALID = 1;   // bit0
static const uint8_t STATUS_SAFETY_ENABLED = 2;   // bit1
static const uint8_t STATUS_SAFETY_STOP    = 4;   // bit2

static const uint8_t CFG_INVERT_LEFT       = 1;   // bit0
static const uint8_t CFG_INVERT_RIGHT      = 2;   // bit1
static const uint8_t CFG_ENABLE_SAFETY     = 4;   // bit2

/* ===========================
   mBot HARDWARE
   =========================== */

// Motori: M1 e M2 sono le uscite driver presenti su mCore
static MeDCMotor motorLeft(M1);
static MeDCMotor motorRight(M2);

// Ultrasuoni: usare PORT_1 (consigliato per lasciare PORT_3/4 al bus I2C)
static MeUltrasonicSensor ultraSensor(PORT_1);

/* ===========================
   STATO INTERNO
   =========================== */

// Registro correntemente "selezionato" dal master
static volatile uint8_t currentRegister = REG_ID_VERSION;

// Setpoint motori (comandi ricevuti dal master)
static volatile int16_t motorLeftCmd = 0;
static volatile int16_t motorRightCmd = 0;

// Configurazione
static volatile uint8_t configFlags = 0;

// Safety distanza (isteresi)
static volatile uint16_t stopThresholdMm = 120;     // esempio: 12 cm
static volatile uint16_t resumeThresholdMm = 180;   // esempio: 18 cm

// Stato sensore ultrasuoni
static volatile uint16_t distanceMm = 0;
static volatile uint8_t statusFlags = 0;

// Buffer di risposta I2C (massimo 32 byte su ATmega328, noi restiamo molto sotto)
static uint8_t txBuffer[8];
static uint8_t txLen = 0;

// Filtro semplice: media mobile su N campioni
static const uint8_t DIST_FILTER_N = 5;
static uint16_t distSamples[DIST_FILTER_N];
static uint8_t distIndex = 0;
static bool distFilled = false;

// Safety latch
static bool safetyStopLatched = false;

/* ===========================
   FUNZIONI DI SUPPORTO
   =========================== */

static int16_t clampMotor(int16_t v)
{
    if (v > 255)
    {
        return 255;
    }

    if (v < -255)
    {
        return -255;
    }

    return v;
}

static int16_t applyInvertIfNeeded(int16_t v, bool invert)
{
    if (invert)
    {
        return (int16_t)(-v);
    }

    return v;
}

static void applyMotorsWithSafety(void)
{
    int16_t left = motorLeftCmd;
    int16_t right = motorRightCmd;

    left = clampMotor(left);
    right = clampMotor(right);

    // Applica inversioni di montaggio (se configurate)
    {
        bool invL = false;
        bool invR = false;

        if ((configFlags & CFG_INVERT_LEFT) != 0)
        {
            invL = true;
        }

        if ((configFlags & CFG_INVERT_RIGHT) != 0)
        {
            invR = true;
        }

        left = applyInvertIfNeeded(left, invL);
        right = applyInvertIfNeeded(right, invR);
    }

    // Se safety attiva e latch in stop, forziamo i motori a 0
    if (safetyStopLatched)
    {
        motorLeft.run(0);
        motorRight.run(0);
        return;
    }

    motorLeft.run(left);
    motorRight.run(right);
}

static bool isDistanceValidMm(uint16_t mm)
{
    // Range tipico ultrasuoni (dipende dal modulo, ma è sufficiente per robustezza)
    if (mm == 0)
    {
        return false;
    }

    if (mm > 4000) // > 400 cm
    {
        return false;
    }

    return true;
}

static uint16_t filterDistance(uint16_t newMm)
{
    // Inserisce il campione
    distSamples[distIndex] = newMm;
    distIndex = (uint8_t)(distIndex + 1);

    if (distIndex >= DIST_FILTER_N)
    {
        distIndex = 0;
        distFilled = true;
    }

    // Calcolo media
    uint32_t sum = 0;
    uint8_t count = 0;

    if (distFilled)
    {
        count = DIST_FILTER_N;
    }
    else
    {
        count = distIndex;
        if (count == 0)
        {
            count = 1;
        }
    }

    for (uint8_t i = 0; i < count; i++)
    {
        sum += distSamples[i];
    }

    return (uint16_t)(sum / count);
}

static void updateUltrasonic(void)
{
    // Lettura in cm dalla libreria Makeblock
    double cm = ultraSensor.distanceCm();

    // Conversione cm -> mm (cm * 10)
    uint16_t mm = 0;

    if (cm > 0.0)
    {
        mm = (uint16_t)(cm * 10.0);
    }

    // Validazione
    if (!isDistanceValidMm(mm))
    {
        distanceMm = 0;

        // aggiorna flag (bit0 = distanza valida)
        statusFlags = (uint8_t)(statusFlags & (uint8_t)(~STATUS_DISTANCE_VALID));
        return;
    }

    // Filtro
    mm = filterDistance(mm);

    distanceMm = mm;
    statusFlags = (uint8_t)(statusFlags | STATUS_DISTANCE_VALID);
}

static void updateSafetyState(void)
{
    bool safetyEnabled = false;

    if ((configFlags & CFG_ENABLE_SAFETY) != 0)
    {
        safetyEnabled = true;
    }

    // Aggiorna status bit1
    if (safetyEnabled)
    {
        statusFlags = (uint8_t)(statusFlags | STATUS_SAFETY_ENABLED);
    }
    else
    {
        statusFlags = (uint8_t)(statusFlags & (uint8_t)(~STATUS_SAFETY_ENABLED));
    }

    if (!safetyEnabled)
    {
        safetyStopLatched = false;
        statusFlags = (uint8_t)(statusFlags & (uint8_t)(~STATUS_SAFETY_STOP));
        return;
    }

    // Se distanza non valida, non interveniamo (lasciamo libertà al docente)
    if ((statusFlags & STATUS_DISTANCE_VALID) == 0)
    {
        // non modifichiamo il latch
        return;
    }

    // Logica con isteresi:
    // - entra in stop se mm <= stopThresholdMm
    // - esce da stop se mm >= resumeThresholdMm
    if (!safetyStopLatched)
    {
        if (distanceMm <= stopThresholdMm)
        {
            safetyStopLatched = true;
        }
    }
    else
    {
        if (distanceMm >= resumeThresholdMm)
        {
            safetyStopLatched = false;
        }
    }

    if (safetyStopLatched)
    {
        statusFlags = (uint8_t)(statusFlags | STATUS_SAFETY_STOP);
    }
    else
    {
        statusFlags = (uint8_t)(statusFlags & (uint8_t)(~STATUS_SAFETY_STOP));
    }
}

static void prepareTx(uint8_t reg)
{
    txLen = 0;

    if (reg == REG_ID_VERSION)
    {
        // Firma semplice "MB"
        txBuffer[0] = 0x4D; // 'M'
        txBuffer[1] = 0x42; // 'B'
        txLen = 2;
        return;
    }

    if (reg == REG_MOTOR_LEFT)
    {
        int16_t v = motorLeftCmd;
        txBuffer[0] = (uint8_t)(v & 0xFF);
        txBuffer[1] = (uint8_t)((v >> 8) & 0xFF);
        txLen = 2;
        return;
    }

    if (reg == REG_MOTOR_RIGHT)
    {
        int16_t v = motorRightCmd;
        txBuffer[0] = (uint8_t)(v & 0xFF);
        txBuffer[1] = (uint8_t)((v >> 8) & 0xFF);
        txLen = 2;
        return;
    }

    if (reg == REG_DISTANCE_MM)
    {
        uint16_t v = distanceMm;
        txBuffer[0] = (uint8_t)(v & 0xFF);
        txBuffer[1] = (uint8_t)((v >> 8) & 0xFF);
        txLen = 2;
        return;
    }

    if (reg == REG_STATUS_FLAGS)
    {
        txBuffer[0] = statusFlags;
        txLen = 1;
        return;
    }

    if (reg == REG_CONFIG_FLAGS)
    {
        txBuffer[0] = configFlags;
        txLen = 1;
        return;
    }

    if (reg == REG_STOP_MM)
    {
        uint16_t v = stopThresholdMm;
        txBuffer[0] = (uint8_t)(v & 0xFF);
        txBuffer[1] = (uint8_t)((v >> 8) & 0xFF);
        txLen = 2;
        return;
    }

    if (reg == REG_RESUME_MM)
    {
        uint16_t v = resumeThresholdMm;
        txBuffer[0] = (uint8_t)(v & 0xFF);
        txBuffer[1] = (uint8_t)((v >> 8) & 0xFF);
        txLen = 2;
        return;
    }

    // Registro sconosciuto: risposta neutra
    txBuffer[0] = 0;
    txLen = 1;
}

/* ===========================
   CALLBACK I2C
   =========================== */

static void onI2CReceive(int byteCount)
{
    if (byteCount <= 0)
    {
        return;
    }

    // 1) Primo byte: selezione registro
    uint8_t reg = Wire.read();
    currentRegister = reg;
    byteCount = byteCount - 1;

    // 2) Se non c'è payload, la scrittura è solo "selezione" (per letture successive)
    if (byteCount == 0)
    {
        return;
    }

    // 3) Gestione scritture per registri configurabili
    if (reg == REG_MOTOR_LEFT)
    {
        if (byteCount >= 2)
        {
            uint8_t b0 = Wire.read();
            uint8_t b1 = Wire.read();

            int16_t v = (int16_t)((uint16_t)b0 | ((uint16_t)b1 << 8));
            motorLeftCmd = clampMotor(v);
        }

        while (Wire.available() > 0)
        {
            Wire.read();
        }
        return;
    }

    if (reg == REG_MOTOR_RIGHT)
    {
        if (byteCount >= 2)
        {
            uint8_t b0 = Wire.read();
            uint8_t b1 = Wire.read();

            int16_t v = (int16_t)((uint16_t)b0 | ((uint16_t)b1 << 8));
            motorRightCmd = clampMotor(v);
        }

        while (Wire.available() > 0)
        {
            Wire.read();
        }
        return;
    }

    if (reg == REG_CONFIG_FLAGS)
    {
        // 1 byte
        uint8_t v = Wire.read();
        configFlags = v;

        while (Wire.available() > 0)
        {
            Wire.read();
        }
        return;
    }

    if (reg == REG_STOP_MM)
    {
        if (byteCount >= 2)
        {
            uint8_t b0 = Wire.read();
            uint8_t b1 = Wire.read();
            uint16_t v = (uint16_t)b0 | ((uint16_t)b1 << 8);

            // Soglia minima ragionevole per evitare valori assurdi
            if (v < 30)
            {
                v = 30;
            }

            stopThresholdMm = v;
        }

        while (Wire.available() > 0)
        {
            Wire.read();
        }
        return;
    }

    if (reg == REG_RESUME_MM)
    {
        if (byteCount >= 2)
        {
            uint8_t b0 = Wire.read();
            uint8_t b1 = Wire.read();
            uint16_t v = (uint16_t)b0 | ((uint16_t)b1 << 8);

            // Deve essere >= stopThresholdMm per isteresi sensata
            if (v < stopThresholdMm)
            {
                v = stopThresholdMm;
            }

            resumeThresholdMm = v;
        }

        while (Wire.available() > 0)
        {
            Wire.read();
        }
        return;
    }

    // Registro non gestito in scrittura: consumiamo e ignoriamo
    while (Wire.available() > 0)
    {
        Wire.read();
    }
}

static void onI2CRequest(void)
{
    prepareTx(currentRegister);

    if (txLen == 0)
    {
        Wire.write((uint8_t)0);
        return;
    }

    Wire.write(txBuffer, txLen);
}

/* ===========================
   SETUP / LOOP
   =========================== */

void setup(void)
{
    // I2C slave
    Wire.begin(I2C_SLAVE_ADDRESS);
    Wire.onReceive(onI2CReceive);
    Wire.onRequest(onI2CRequest);

    // Stato iniziale
    motorLeftCmd = 0;
    motorRightCmd = 0;

    configFlags = 0; // default: niente inversioni, safety disabilitata

    distanceMm = 0;
    statusFlags = 0;

    // Inizializza filtro
    for (uint8_t i = 0; i < DIST_FILTER_N; i++)
    {
        distSamples[i] = 0;
    }
    distIndex = 0;
    distFilled = false;

    safetyStopLatched = false;

    // Applica motori fermi
    motorLeft.run(0);
    motorRight.run(0);
}

void loop(void)
{
    // 1) Aggiorna lettura ultrasuoni + validazione + filtro
    updateUltrasonic();

    // 2) Aggiorna logica safety (se abilitata)
    updateSafetyState();

    // 3) Applica i motori (con inversioni e safety latch)
    applyMotorsWithSafety();

    // Frequenza moderata: abbastanza reattiva ma non rumorosa
    delay(40);
}
