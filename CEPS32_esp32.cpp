/*
    ============================================================
    Ophiocordyceps32 - ESP32 Web Remote per mBot (mCore) via I2C
    ============================================================

    Obiettivo:
      - ESP32 espone una pagina Web con "telecomando"
      - ESP32 espone API HTTP (JSON) per controllare mBot da Python (requests)
      - ESP32 parla con mBot via I2C (ESP32 master, mBot slave 0x12)

    Cablaggio (mBot mCore -> ESP32):
      - Usare una porta che espone chiaramente SCL/SDA/GND (nella sua foto: PORT 3 o PORT 4)
      - Collegamenti minimi:
          SDA <-> SDA
          SCL <-> SCL
          GND <-> GND
      - Raccomandato: level shifter I2C bidirezionale 5V <-> 3.3V

    Librerie:
      - WiFi.h
      - WebServer.h
      - Wire.h

    Sicurezza (nota):
      - Questo esempio non include autenticazione.
      - In rete scolastica, valutare almeno una password o una VLAN dedicata.

    ============================================================
*/

#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>

/* ============================================================
   CONFIG WIFI
   ============================================================ */

static const char *WIFI_SSID = "INSERISCI_SSID";
static const char *WIFI_PASS = "INSERISCI_PASSWORD";

// Hostname (utile su alcune reti)
static const char *HOSTNAME = "ophiocordyceps32";

/* ============================================================
   CONFIG WEB SERVER
   ============================================================ */

static const uint16_t HTTP_PORT = 80;
static WebServer server(HTTP_PORT);

/* ============================================================
   CONFIG I2C (ESP32)
   ============================================================ */

// Molte devboard ESP32 usano SDA=21, SCL=22.
// Se necessario, cambiare qui.
static const int I2C_PIN_SDA = 21;
static const int I2C_PIN_SCL = 22;

// Standard mode per robustezza
static const uint32_t I2C_CLOCK_HZ = 100000;

/* ============================================================
   CONFIG mBot (I2C slave)
   ============================================================ */

static const uint8_t MBOT_I2C_ADDRESS = 0x12;

/* ============================================================
   REGISTRI mBot (devono combaciare con lo sketch mBot)
   ============================================================ */

static const uint8_t REG_ID_VERSION      = 0x00;
static const uint8_t REG_MOTOR_LEFT      = 0x01;
static const uint8_t REG_MOTOR_RIGHT     = 0x03;
static const uint8_t REG_DISTANCE_MM     = 0x05;
static const uint8_t REG_STATUS_FLAGS    = 0x07;
static const uint8_t REG_CONFIG_FLAGS    = 0x08;
static const uint8_t REG_STOP_MM         = 0x09;
static const uint8_t REG_RESUME_MM       = 0x0B;

/* ============================================================
   STATUS FLAGS (bit)
   ============================================================ */

static const uint8_t STATUS_DISTANCE_VALID = 1;   // bit0
static const uint8_t STATUS_SAFETY_ENABLED = 2;   // bit1
static const uint8_t STATUS_SAFETY_STOP    = 4;   // bit2

/* ============================================================
   CONFIG FLAGS (bit)
   ============================================================ */

static const uint8_t CFG_INVERT_LEFT       = 1;   // bit0
static const uint8_t CFG_INVERT_RIGHT      = 2;   // bit1
static const uint8_t CFG_ENABLE_SAFETY     = 4;   // bit2

/* ============================================================
   UTILS: parsing e validazione (no ternari, massima leggibilità)
   ============================================================ */

static int16_t clampMotor(int32_t v)
{
    if (v > 255)
    {
        return 255;
    }

    if (v < -255)
    {
        return -255;
    }

    return (int16_t)v;
}

static bool tryParseInt(const String &s, int32_t &out)
{
    // Parsing robusto: accetta solo numeri con eventuale segno.
    if (s.length() == 0)
    {
        return false;
    }

    int i = 0;
    bool negative = false;

    if (s[0] == '-')
    {
        negative = true;
        i = 1;
    }
    else if (s[0] == '+')
    {
        i = 1;
    }

    if (i >= (int)s.length())
    {
        return false;
    }

    int32_t val = 0;

    for (; i < (int)s.length(); i++)
    {
        char c = s[i];
        if (c < '0' || c > '9')
        {
            return false;
        }

        val = (val * 10) + (c - '0');
    }

    if (negative)
    {
        val = -val;
    }

    out = val;
    return true;
}

static bool tryParseUInt16(const String &s, uint16_t &out)
{
    int32_t tmp = 0;
    bool ok = tryParseInt(s, tmp);
    if (!ok)
    {
        return false;
    }

    if (tmp < 0)
    {
        return false;
    }

    if (tmp > 65535)
    {
        return false;
    }

    out = (uint16_t)tmp;
    return true;
}

static bool tryParseBool01(const String &s, bool &out)
{
    // Accetta "0" / "1"
    if (s == "0")
    {
        out = false;
        return true;
    }

    if (s == "1")
    {
        out = true;
        return true;
    }

    return false;
}

/* ============================================================
   I2C low-level
   ============================================================ */

static bool i2cWriteRegister(uint8_t reg, const uint8_t *data, uint8_t len)
{
    Wire.beginTransmission(MBOT_I2C_ADDRESS);
    Wire.write(reg);

    if (len > 0 && data != NULL)
    {
        for (uint8_t i = 0; i < len; i++)
        {
            Wire.write(data[i]);
        }
    }

    uint8_t err = Wire.endTransmission(true);

    if (err == 0)
    {
        return true;
    }

    return false;
}

static bool i2cReadRegister(uint8_t reg, uint8_t *out, uint8_t len)
{
    bool okSel = i2cWriteRegister(reg, NULL, 0);
    if (!okSel)
    {
        return false;
    }

    uint8_t received = Wire.requestFrom((int)MBOT_I2C_ADDRESS, (int)len, (int)true);

    if (received != len)
    {
        while (Wire.available() > 0)
        {
            Wire.read();
        }
        return false;
    }

    for (uint8_t i = 0; i < len; i++)
    {
        if (Wire.available() > 0)
        {
            out[i] = (uint8_t)Wire.read();
        }
        else
        {
            return false;
        }
    }

    return true;
}

/* ============================================================
   API alto livello mBot
   ============================================================ */

static bool mbotStopMotors(void)
{
    uint8_t pL[2];
    uint8_t pR[2];

    pL[0] = 0;
    pL[1] = 0;

    pR[0] = 0;
    pR[1] = 0;

    bool ok1 = i2cWriteRegister(REG_MOTOR_LEFT, pL, 2);
    bool ok2 = i2cWriteRegister(REG_MOTOR_RIGHT, pR, 2);

    if (!ok1 || !ok2)
    {
        return false;
    }

    return true;
}

static bool mbotSetMotors(int16_t left, int16_t right)
{
    uint8_t pL[2];
    uint8_t pR[2];

    left = clampMotor(left);
    right = clampMotor(right);

    pL[0] = (uint8_t)(left & 0xFF);
    pL[1] = (uint8_t)((left >> 8) & 0xFF);

    pR[0] = (uint8_t)(right & 0xFF);
    pR[1] = (uint8_t)((right >> 8) & 0xFF);

    bool ok1 = i2cWriteRegister(REG_MOTOR_LEFT, pL, 2);
    bool ok2 = i2cWriteRegister(REG_MOTOR_RIGHT, pR, 2);

    if (!ok1 || !ok2)
    {
        return false;
    }

    return true;
}

static bool mbotGetDistanceMm(uint16_t &mm, bool &valid)
{
    uint8_t buf[2];

    bool ok = i2cReadRegister(REG_DISTANCE_MM, buf, 2);
    if (!ok)
    {
        mm = 0;
        valid = false;
        return false;
    }

    mm = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);

    if (mm == 0)
    {
        valid = false;
    }
    else
    {
        valid = true;
    }

    return true;
}

static bool mbotGetStatus(uint8_t &status)
{
    uint8_t buf[1];

    bool ok = i2cReadRegister(REG_STATUS_FLAGS, buf, 1);
    if (!ok)
    {
        status = 0;
        return false;
    }

    status = buf[0];
    return true;
}

static bool mbotGetConfig(uint8_t &cfg)
{
    uint8_t buf[1];

    bool ok = i2cReadRegister(REG_CONFIG_FLAGS, buf, 1);
    if (!ok)
    {
        cfg = 0;
        return false;
    }

    cfg = buf[0];
    return true;
}

static bool mbotSetConfig(uint8_t cfg)
{
    uint8_t payload[1];
    payload[0] = cfg;

    return i2cWriteRegister(REG_CONFIG_FLAGS, payload, 1);
}

static bool mbotSetSafetyThresholds(uint16_t stopMm, uint16_t resumeMm)
{
    uint8_t pStop[2];
    uint8_t pResume[2];

    pStop[0] = (uint8_t)(stopMm & 0xFF);
    pStop[1] = (uint8_t)((stopMm >> 8) & 0xFF);

    pResume[0] = (uint8_t)(resumeMm & 0xFF);
    pResume[1] = (uint8_t)((resumeMm >> 8) & 0xFF);

    bool ok1 = i2cWriteRegister(REG_STOP_MM, pStop, 2);
    bool ok2 = i2cWriteRegister(REG_RESUME_MM, pResume, 2);

    if (!ok1 || !ok2)
    {
        return false;
    }

    return true;
}

static bool mbotGetId(char &c0, char &c1)
{
    uint8_t buf[2];

    bool ok = i2cReadRegister(REG_ID_VERSION, buf, 2);
    if (!ok)
    {
        c0 = '?';
        c1 = '?';
        return false;
    }

    c0 = (char)buf[0];
    c1 = (char)buf[1];
    return true;
}

/* ============================================================
   HTTP helpers: JSON e CORS
   ============================================================ */

static void sendCorsHeaders(void)
{
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.sendHeader("Access-Control-Allow-Methods", "GET,POST,OPTIONS");
    server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
}

static void sendJson(int code, const String &json)
{
    sendCorsHeaders();
    server.send(code, "application/json; charset=utf-8", json);
}

static void sendText(int code, const String &text)
{
    sendCorsHeaders();
    server.send(code, "text/plain; charset=utf-8", text);
}

static String jsonEscape(const String &s)
{
    // Escape minimale per virgolette e backslash.
    String out;
    out.reserve(s.length() + 8);

    for (int i = 0; i < (int)s.length(); i++)
    {
        char c = s[i];

        if (c == '\\')
        {
            out += "\\\\";
        }
        else if (c == '\"')
        {
            out += "\\\"";
        }
        else if (c == '\n')
        {
            out += "\\n";
        }
        else if (c == '\r')
        {
            out += "\\r";
        }
        else
        {
            out += c;
        }
    }

    return out;
}

/* ============================================================
   PAGINA HTML (telecomando + documentazione)
   ============================================================ */

static String buildHomePage(const String &ipStr)
{
    String html;

    html += "<!doctype html><html><head><meta charset='utf-8'>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
    html += "<title>Ophiocordyceps32 - Web Remote mBot</title>";
    html += "<style>";
    html += "body{font-family:Arial,Helvetica,sans-serif;max-width:980px;margin:20px auto;padding:0 12px;}";
    html += "h1{margin-bottom:4px;} .box{border:1px solid #ccc;border-radius:10px;padding:12px;margin:12px 0;}";
    html += "code,pre{background:#f5f5f5;border-radius:8px;padding:8px;display:block;overflow:auto;}";
    html += "button{padding:10px 12px;margin:4px;border-radius:10px;border:1px solid #888;}";
    html += "input{padding:8px;border-radius:8px;border:1px solid #888;width:90px;}";
    html += ".row{display:flex;gap:10px;flex-wrap:wrap;align-items:center;}";
    html += ".small{font-size:0.92em;color:#333;}";
    html += "</style></head><body>";

    html += "<h1>Ophiocordyceps32</h1>";
    html += "<div class='small'>ESP32 Web Remote per mBot via I2C. Base URL: <b>http://" + ipStr + "/</b></div>";

    html += "<div class='box'><h2>Telecomando rapido</h2>";
    html += "<div class='row'>";
    html += "Velocit&agrave; L: <input id='left' type='number' value='120' min='-255' max='255'>";
    html += "Velocit&agrave; R: <input id='right' type='number' value='120' min='-255' max='255'>";
    html += "<button onclick='setMotors()'>Invia MOTORI</button>";
    html += "<button onclick='stopMotors()'>STOP</button>";
    html += "<button onclick='readDistance()'>Leggi DISTANZA</button>";
    html += "<button onclick='readStatus()'>Leggi STATUS</button>";
    html += "</div>";
    html += "<pre id='out'>Pronto.</pre>";
    html += "</div>";

    html += "<div class='box'><h2>API HTTP (per Python requests)</h2>";
    html += "<p>Gli endpoint sono pensati per essere chiamati da Python (requests) come \"telecomando\".</p>";

    html += "<h3>Comando motori</h3>";
    html += "<pre>GET  /api/motors?left=120&amp;right=120\nPOST /api/motors  (stessi parametri in query)</pre>";

    html += "<h3>Stop</h3>";
    html += "<pre>GET  /api/stop\nPOST /api/stop</pre>";

    html += "<h3>Letture</h3>";
    html += "<pre>GET /api/distance\nGET /api/status\nGET /api/id</pre>";

    html += "<h3>Config (inverti motori / safety)</h3>";
    html += "<pre>GET  /api/config?invertL=0&amp;invertR=0&amp;safety=1\nPOST /api/config  (stessi parametri in query)</pre>";

    html += "<h3>Soglie safety (mm)</h3>";
    html += "<pre>GET  /api/safety?stop_mm=120&amp;resume_mm=180\nPOST /api/safety  (stessi parametri in query)</pre>";

    html += "<h3>Esempio Python (requests)</h3>";
    html += "<pre>";
    html += "import requests\n\n";
    html += "BASE = \"http://" + ipStr + "\"\n\n";
    html += "# Motori\n";
    html += "r = requests.get(f\"{BASE}/api/motors\", params={\"left\": 120, \"right\": 120}, timeout=2)\n";
    html += "print(r.json())\n\n";
    html += "# Distanza\n";
    html += "r = requests.get(f\"{BASE}/api/distance\", timeout=2)\n";
    html += "print(r.json())\n\n";
    html += "# Stop\n";
    html += "r = requests.get(f\"{BASE}/api/stop\", timeout=2)\n";
    html += "print(r.json())\n";
    html += "</pre>";

    html += "<p>Per esempi testuali pronti (anche curl): <a href='/api/help'>/api/help</a></p>";
    html += "</div>";

    html += "<script>";
    html += "async function apiGet(path){";
    html += "  const r = await fetch(path);";
    html += "  const t = await r.text();";
    html += "  return t;";
    html += "}";
    html += "function setOut(x){ document.getElementById('out').textContent = x; }";
    html += "async function setMotors(){";
    html += "  const l = document.getElementById('left').value;";
    html += "  const r = document.getElementById('right').value;";
    html += "  const txt = await apiGet('/api/motors?left=' + encodeURIComponent(l) + '&right=' + encodeURIComponent(r));";
    html += "  setOut(txt);";
    html += "}";
    html += "async function stopMotors(){";
    html += "  const txt = await apiGet('/api/stop');";
    html += "  setOut(txt);";
    html += "}";
    html += "async function readDistance(){";
    html += "  const txt = await apiGet('/api/distance');";
    html += "  setOut(txt);";
    html += "}";
    html += "async function readStatus(){";
    html += "  const txt = await apiGet('/api/status');";
    html += "  setOut(txt);";
    html += "}";
    html += "</script>";

    html += "</body></html>";

    return html;
}

/* ============================================================
   HANDLER HTTP
   ============================================================ */

static void handleOptions(void)
{
    sendCorsHeaders();
    server.send(204);
}

static void handleRoot(void)
{
    IPAddress ip = WiFi.localIP();
    String ipStr = ip.toString();

    String html = buildHomePage(ipStr);
    sendCorsHeaders();
    server.send(200, "text/html; charset=utf-8", html);
}

static void handlePing(void)
{
    String json = "{\"ok\":true,\"project\":\"Ophiocordyceps32\"}";
    sendJson(200, json);
}

static void handleId(void)
{
    char c0 = '?';
    char c1 = '?';

    bool ok = mbotGetId(c0, c1);

    String idStr;
    idStr += c0;
    idStr += c1;

    String json;

    if (ok)
    {
        json = "{\"ok\":true,\"id\":\"" + jsonEscape(idStr) + "\"}";
        sendJson(200, json);
        return;
    }

    json = "{\"ok\":false,\"error\":\"mbot_i2c_read_failed\"}";
    sendJson(500, json);
}

static void handleStatus(void)
{
    uint8_t status = 0;
    bool ok = mbotGetStatus(status);

    if (!ok)
    {
        sendJson(500, "{\"ok\":false,\"error\":\"mbot_i2c_read_failed\"}");
        return;
    }

    bool valid = false;
    bool safetyEnabled = false;
    bool safetyStop = false;

    if ((status & STATUS_DISTANCE_VALID) != 0)
    {
        valid = true;
    }

    if ((status & STATUS_SAFETY_ENABLED) != 0)
    {
        safetyEnabled = true;
    }

    if ((status & STATUS_SAFETY_STOP) != 0)
    {
        safetyStop = true;
    }

    String json = "{";
    json += "\"ok\":true,";
    json += "\"status\":";
    json += String(status);
    json += ",";
    json += "\"distance_valid\":";
    if (valid)
    {
        json += "true";
    }
    else
    {
        json += "false";
    }
    json += ",";
    json += "\"safety_enabled\":";
    if (safetyEnabled)
    {
        json += "true";
    }
    else
    {
        json += "false";
    }
    json += ",";
    json += "\"safety_stop\":";
    if (safetyStop)
    {
        json += "true";
    }
    else
    {
        json += "false";
    }
    json += "}";

    sendJson(200, json);
}

static void handleDistance(void)
{
    uint16_t mm = 0;
    bool valid = false;

    bool ok = mbotGetDistanceMm(mm, valid);

    if (!ok)
    {
        sendJson(500, "{\"ok\":false,\"error\":\"mbot_i2c_read_failed\"}");
        return;
    }

    String json = "{";
    json += "\"ok\":true,";
    json += "\"mm\":";
    json += String(mm);
    json += ",";
    json += "\"valid\":";
    if (valid)
    {
        json += "true";
    }
    else
    {
        json += "false";
    }
    json += "}";

    sendJson(200, json);
}

static void handleMotors(void)
{
    // Parametri obbligatori: left, right
    if (!server.hasArg("left") || !server.hasArg("right"))
    {
        sendJson(400, "{\"ok\":false,\"error\":\"missing_params\",\"required\":[\"left\",\"right\"]}");
        return;
    }

    int32_t leftRaw = 0;
    int32_t rightRaw = 0;

    bool okL = tryParseInt(server.arg("left"), leftRaw);
    bool okR = tryParseInt(server.arg("right"), rightRaw);

    if (!okL || !okR)
    {
        sendJson(400, "{\"ok\":false,\"error\":\"bad_params\",\"hint\":\"left/right must be integers\"}");
        return;
    }

    int16_t left = clampMotor(leftRaw);
    int16_t right = clampMotor(rightRaw);

    bool ok = mbotSetMotors(left, right);

    if (!ok)
    {
        sendJson(500, "{\"ok\":false,\"error\":\"mbot_i2c_write_failed\"}");
        return;
    }

    String json = "{";
    json += "\"ok\":true,";
    json += "\"left\":";
    json += String(left);
    json += ",";
    json += "\"right\":";
    json += String(right);
    json += "}";

    sendJson(200, json);
}

static void handleStop(void)
{
    bool ok = mbotStopMotors();

    if (!ok)
    {
        sendJson(500, "{\"ok\":false,\"error\":\"mbot_i2c_write_failed\"}");
        return;
    }

    sendJson(200, "{\"ok\":true,\"stopped\":true}");
}

static void handleConfig(void)
{
    // Se è GET senza parametri, restituiamo la config corrente
    if (!server.hasArg("invertL") && !server.hasArg("invertR") && !server.hasArg("safety"))
    {
        uint8_t cfg = 0;
        bool ok = mbotGetConfig(cfg);

        if (!ok)
        {
            sendJson(500, "{\"ok\":false,\"error\":\"mbot_i2c_read_failed\"}");
            return;
        }

        bool invL = false;
        bool invR = false;
        bool safety = false;

        if ((cfg & CFG_INVERT_LEFT) != 0)
        {
            invL = true;
        }
        if ((cfg & CFG_INVERT_RIGHT) != 0)
        {
            invR = true;
        }
        if ((cfg & CFG_ENABLE_SAFETY) != 0)
        {
            safety = true;
        }

        String json = "{";
        json += "\"ok\":true,";
        json += "\"invertL\":";
        json += (invL ? "true" : "false");
        json += ",";
        json += "\"invertR\":";
        json += (invR ? "true" : "false");
        json += ",";
        json += "\"safety\":";
        json += (safety ? "true" : "false");
        json += ",";
        json += "\"raw\":";
        json += String(cfg);
        json += "}";

        sendJson(200, json);
        return;
    }

    // Altrimenti, applichiamo i parametri presenti (0/1)
    uint8_t cfg = 0;
    bool okRead = mbotGetConfig(cfg);

    if (!okRead)
    {
        sendJson(500, "{\"ok\":false,\"error\":\"mbot_i2c_read_failed\"}");
        return;
    }

    if (server.hasArg("invertL"))
    {
        bool b = false;
        bool ok = tryParseBool01(server.arg("invertL"), b);
        if (!ok)
        {
            sendJson(400, "{\"ok\":false,\"error\":\"bad_params\",\"hint\":\"invertL must be 0 or 1\"}");
            return;
        }

        if (b)
        {
            cfg = (uint8_t)(cfg | CFG_INVERT_LEFT);
        }
        else
        {
            cfg = (uint8_t)(cfg & (uint8_t)(~CFG_INVERT_LEFT));
        }
    }

    if (server.hasArg("invertR"))
    {
        bool b = false;
        bool ok = tryParseBool01(server.arg("invertR"), b);
        if (!ok)
        {
            sendJson(400, "{\"ok\":false,\"error\":\"bad_params\",\"hint\":\"invertR must be 0 or 1\"}");
            return;
        }

        if (b)
        {
            cfg = (uint8_t)(cfg | CFG_INVERT_RIGHT);
        }
        else
        {
            cfg = (uint8_t)(cfg & (uint8_t)(~CFG_INVERT_RIGHT));
        }
    }

    if (server.hasArg("safety"))
    {
        bool b = false;
        bool ok = tryParseBool01(server.arg("safety"), b);
        if (!ok)
        {
            sendJson(400, "{\"ok\":false,\"error\":\"bad_params\",\"hint\":\"safety must be 0 or 1\"}");
            return;
        }

        if (b)
        {
            cfg = (uint8_t)(cfg | CFG_ENABLE_SAFETY);
        }
        else
        {
            cfg = (uint8_t)(cfg & (uint8_t)(~CFG_ENABLE_SAFETY));
        }
    }

    bool okWrite = mbotSetConfig(cfg);

    if (!okWrite)
    {
        sendJson(500, "{\"ok\":false,\"error\":\"mbot_i2c_write_failed\"}");
        return;
    }

    String json = "{\"ok\":true,\"raw\":" + String(cfg) + "}";
    sendJson(200, json);
}

static void handleSafety(void)
{
    // Richiede stop_mm e resume_mm se si vuole scrivere.
    // Se manca uno dei due -> errore.
    if (!server.hasArg("stop_mm") && !server.hasArg("resume_mm"))
    {
        sendJson(400, "{\"ok\":false,\"error\":\"missing_params\",\"required\":[\"stop_mm\",\"resume_mm\"]}");
        return;
    }

    if (!server.hasArg("stop_mm") || !server.hasArg("resume_mm"))
    {
        sendJson(400, "{\"ok\":false,\"error\":\"missing_params\",\"required\":[\"stop_mm\",\"resume_mm\"]}");
        return;
    }

    uint16_t stopMm = 0;
    uint16_t resumeMm = 0;

    bool okS = tryParseUInt16(server.arg("stop_mm"), stopMm);
    bool okR = tryParseUInt16(server.arg("resume_mm"), resumeMm);

    if (!okS || !okR)
    {
        sendJson(400, "{\"ok\":false,\"error\":\"bad_params\",\"hint\":\"stop_mm/resume_mm must be uint16\"}");
        return;
    }

    // Vincolo di isteresi: resume >= stop
    if (resumeMm < stopMm)
    {
        sendJson(400, "{\"ok\":false,\"error\":\"bad_params\",\"hint\":\"resume_mm must be >= stop_mm\"}");
        return;
    }

    bool okWrite = mbotSetSafetyThresholds(stopMm, resumeMm);

    if (!okWrite)
    {
        sendJson(500, "{\"ok\":false,\"error\":\"mbot_i2c_write_failed\"}");
        return;
    }

    String json = "{";
    json += "\"ok\":true,";
    json += "\"stop_mm\":";
    json += String(stopMm);
    json += ",";
    json += "\"resume_mm\":";
    json += String(resumeMm);
    json += "}";

    sendJson(200, json);
}

static void handleHelp(void)
{
    IPAddress ip = WiFi.localIP();
    String ipStr = ip.toString();

    String t;

    t += "Ophiocordyceps32 - API help\n\n";
    t += "Base URL:\n";
    t += "  http://" + ipStr + "/\n\n";

    t += "MOTORS:\n";
    t += "  GET  /api/motors?left=120&right=120\n";
    t += "  GET  /api/stop\n\n";

    t += "READ:\n";
    t += "  GET /api/distance\n";
    t += "  GET /api/status\n";
    t += "  GET /api/id\n\n";

    t += "CONFIG:\n";
    t += "  GET /api/config?invertL=0&invertR=0&safety=1\n";
    t += "  GET /api/safety?stop_mm=120&resume_mm=180\n\n";

    t += "PYTHON (requests):\n";
    t += "  import requests\n";
    t += "  BASE = \"http://" + ipStr + "\"\n";
    t += "  requests.get(f\"{BASE}/api/motors\", params={\"left\":120,\"right\":120}, timeout=2)\n";
    t += "  requests.get(f\"{BASE}/api/distance\", timeout=2).json()\n";
    t += "  requests.get(f\"{BASE}/api/stop\", timeout=2)\n";

    sendText(200, t);
}

/* ============================================================
   SETUP WIFI + ROUTES
   ============================================================ */

static void setupRoutes(void)
{
    server.on("/", HTTP_GET, handleRoot);

    server.on("/api/ping", HTTP_GET, handlePing);
    server.on("/api/id", HTTP_GET, handleId);
    server.on("/api/status", HTTP_GET, handleStatus);
    server.on("/api/distance", HTTP_GET, handleDistance);

    server.on("/api/motors", HTTP_GET, handleMotors);
    server.on("/api/motors", HTTP_POST, handleMotors);

    server.on("/api/stop", HTTP_GET, handleStop);
    server.on("/api/stop", HTTP_POST, handleStop);

    server.on("/api/config", HTTP_GET, handleConfig);
    server.on("/api/config", HTTP_POST, handleConfig);

    server.on("/api/safety", HTTP_GET, handleSafety);
    server.on("/api/safety", HTTP_POST, handleSafety);

    server.on("/api/help", HTTP_GET, handleHelp);

    // CORS preflight
    server.onNotFound([]()
    {
        // Gestione OPTIONS generica (utile per fetch da browser / CORS)
        if (server.method() == HTTP_OPTIONS)
        {
            handleOptions();
            return;
        }

        sendJson(404, "{\"ok\":false,\"error\":\"not_found\"}");
    });
}

void setup()
{
    Serial.begin(115200);
    delay(200);

    // I2C master
    Wire.begin(I2C_PIN_SDA, I2C_PIN_SCL);
    Wire.setClock(I2C_CLOCK_HZ);

    // WiFi
    WiFi.mode(WIFI_STA);
    WiFi.setHostname(HOSTNAME);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    Serial.println();
    Serial.println("Ophiocordyceps32 - avvio WiFi...");

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(300);
        Serial.print(".");

        // Timeout “umano” per evitare loop infinito
        if (millis() - start > 20000)
        {
            Serial.println();
            Serial.println("WiFi non connesso (timeout). Verificare SSID/PASS.");
            break;
        }
    }

    Serial.println();
    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.print("Connesso. IP: ");
        Serial.println(WiFi.localIP());
    }

    // Avvio server
    setupRoutes();
    server.begin();

    Serial.print("HTTP server avviato su porta ");
    Serial.println(HTTP_PORT);

    // Sicurezza minima: partenza con motori fermi
    mbotStopMotors();
}

void loop()
{
    server.handleClient();
}
