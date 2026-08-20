// Bring-up firmware for the Manta calibration rig's Teensy 4.0.
//
// Answers the questions in platformio.ini and nothing else. The real firmware,
// its wire format and its EEPROM calibration are described in SENSORS.md; none
// of that is here.
//
// The command grammar and the "#" reply prefix are borrowed from
// pico/sampler.py deliberately, so the habits transfer and so the host's
// existing reply routing works unchanged: every board-to-host reply starts
// with "#", which cannot match a sample line, and every command is short
// enough to type by hand into a serial terminal.
//
// Commands:
//   I    identify                -> "# ID dev=manta-bringup ..."
//   ?    status                  -> "# STATUS uptime_ms=... ..."
//   U    UART traffic report     -> "# UART L=... C=... R=..."
//   X    hex dump the last bytes  -> "# DUMP LEFT ..." x3
//   Z    zero the UART counters  -> "# ACK Z"
//   other                        -> "# ERR <text>"

#include <Arduino.h>

// Left, Centre, Right on the three hardware UARTs. Serial1 is pins 0/1,
// Serial2 is 7/8, Serial3 is 14/15. These are separate from the USB "Serial"
// object and do not compete with it for bandwidth.
static HardwareSerial *const PORTS[3] = {&Serial1, &Serial2, &Serial3};
static const char *const NAMES[3] = {"LEFT", "CENTRE", "RIGHT"};

// The DFRobot modules' factory default. Unverified - if a channel reports
// bytes but no plausible frames, this is the first thing to suspect.
static const uint32_t SENSOR_BAUD = 9600;

static const uint8_t MAX_COMMAND_LEN = 16;

// Last bytes seen per channel, so "X" can show the frame structure without
// any timing games on the host side. 64 is comfortably more than two WitMotion
// -style 11-byte frames, which is enough to see a header repeat.
static const uint8_t RING = 64;
static uint8_t ring[3][RING];
static uint8_t ring_pos[3] = {0, 0, 0};

static uint32_t byte_count[3] = {0, 0, 0};
static uint32_t last_byte_ms[3] = {0, 0, 0};
static uint8_t last_byte[3] = {0, 0, 0};

static String pending = "";
static uint32_t blink_at = 0;
static bool led_on = false;

static void identify() {
    // Shaped like the identity reply SENSORS.md describes, so the host-side
    // handshake can be written and tested against this before the real
    // firmware exists. chans/units/rate are what the *rig* firmware will
    // advertise; this build streams nothing, hence mode=bringup.
    Serial.println("# ID dev=manta-bringup proto=0 fw=0.1.0 hw=teensy40 "
                   "chans=LEFT,CENTRE,RIGHT units=none rate=0 cal=none "
                   "mode=bringup feat=uartprobe");
}

static void uart_report() {
    // One line, all three channels, so a glance says which is silent.
    Serial.print("# UART");
    for (int i = 0; i < 3; i++) {
        uint32_t age = last_byte_ms[i] ? (millis() - last_byte_ms[i]) : 0;
        Serial.print(' ');
        Serial.print(NAMES[i][0]);          // L, C, R
        Serial.print("=bytes:");
        Serial.print(byte_count[i]);
        Serial.print(",last:0x");
        Serial.print(last_byte[i], HEX);
        Serial.print(",age_ms:");
        Serial.print(byte_count[i] ? (long)age : -1L);
    }
    Serial.println();
}

static void apply_command(const String &raw) {
    String text = raw;
    text.trim();
    text.toUpperCase();

    if (text.length() == 0) {
        return;
    }
    // Replies are inert as commands. The host TTY line discipline echoes
    // whatever is in its buffer back at the board when the port is first
    // opened, which handed this parser its own boot banner and got an "# ERR
    // # MANTA BRINGUP" for it. Every board-to-host line starts with "#", so
    // ignoring "#" here makes that whole class of loopback harmless.
    if (text.charAt(0) == '#') {
        return;
    }
    if (text == "I") {
        identify();
    } else if (text == "?") {
        Serial.print("# STATUS uptime_ms=");
        Serial.print(millis());
        Serial.print(" sensor_baud=");
        Serial.print(SENSOR_BAUD);
        Serial.println(" mode=bringup");
    } else if (text == "U") {
        uart_report();
    } else if (text == "X") {
        // Oldest-first dump of each channel's ring. Hex, space separated, one
        // line per channel, so a header byte repeating every N bytes is
        // visible by eye and the frame length can be read straight off.
        for (int i = 0; i < 3; i++) {
            Serial.print("# DUMP ");
            Serial.print(NAMES[i]);
            for (uint8_t k = 0; k < RING; k++) {
                uint8_t b = ring[i][(ring_pos[i] + k) % RING];
                Serial.print(' ');
                if (b < 0x10) Serial.print('0');
                Serial.print(b, HEX);
            }
            Serial.println();
        }
    } else if (text == "Z") {
        for (int i = 0; i < 3; i++) {
            byte_count[i] = 0;
            last_byte_ms[i] = 0;
            last_byte[i] = 0;
            ring_pos[i] = 0;
            for (uint8_t k = 0; k < RING; k++) ring[i][k] = 0;
        }
        Serial.println("# ACK Z");
    } else {
        Serial.print("# ERR ");
        Serial.println(text);
    }
}

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);               // ignored on USB CDC; kept for habit
    for (int i = 0; i < 3; i++) {
        PORTS[i]->begin(SENSOR_BAUD);
    }
    // No wait-for-host loop: the board must run standalone on the rig, and a
    // blocking wait here would make an unattended power-on look like a hang.
    delay(200);
    Serial.println("# MANTA bringup ready mode=bringup");
    identify();
}

void loop() {
    // Drain all three UARTs every pass. Counting bytes rather than parsing
    // frames on purpose: the module's frame format is not yet confirmed, and
    // "is anything arriving on this pin at all" is the question this build
    // exists to answer.
    for (int i = 0; i < 3; i++) {
        while (PORTS[i]->available() > 0) {
            uint8_t b = (uint8_t)PORTS[i]->read();
            last_byte[i] = b;
            ring[i][ring_pos[i]] = b;
            ring_pos[i] = (uint8_t)((ring_pos[i] + 1) % RING);
            byte_count[i]++;
            last_byte_ms[i] = millis();
        }
    }

    while (Serial.available() > 0) {
        char ch = (char)Serial.read();
        if (ch == '\n' || ch == '\r') {
            apply_command(pending);
            pending = "";
        } else if (pending.length() < MAX_COMMAND_LEN) {
            pending += ch;
        }
    }

    // Heartbeat, so "is it running" is answerable without a terminal.
    if ((int32_t)(millis() - blink_at) >= 0) {
        blink_at = millis() + 250;
        led_on = !led_on;
        digitalWrite(LED_BUILTIN, led_on ? HIGH : LOW);
    }
}
