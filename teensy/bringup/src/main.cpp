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
//   P<c> passthrough one channel   -> "$ <hex> ..." until a keypress or 20 s
//   B<n> reopen sensor UARTs at n  -> "# ACK B 115200"
//   W<c>:<hex>  write bytes to a   -> "# ACK W C 5"
//        sensor. c is 0/1/2 or A.
//
// B and W exist so the sensor modules can be configured from the host without
// a second adapter, and - more to the point - so a module left at an unknown
// baud can be found again by sweeping B. Changing a module's baud is the one
// operation here that can lose contact with it, so the tool that does it also
// has to be the tool that recovers from it.
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
static uint32_t sensor_baud = 9600;

static const uint8_t MAX_COMMAND_LEN = 48;   // "W" carries a hex payload

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

static int hexval(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

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
        Serial.print(sensor_baud);
        Serial.println(" mode=bringup");
    } else if (text == "U") {
        uart_report();
    } else if (text.startsWith("P")) {
        // Raw passthrough of one sensor UART, hex, 32 bytes per line behind a
        // "$" prefix. No frame parsing here on purpose: the host already has a
        // decoder, and a board that reassembles frames would be deciding what
        // counts as one - which is exactly the thing being measured.
        //
        // "$" rather than "#" so these lines are distinguishable from replies,
        // and neither can be confused with a sample line.
        char which = text.length() > 1 ? text.charAt(1) : '0';
        int ch = (which >= '0' && which <= '2') ? which - '0' : 0;
        Serial.print("# BEGIN P ");
        Serial.println(NAMES[ch]);
        uint8_t buf[32];
        uint8_t n = 0;
        uint32_t stop_at = millis() + 20000;
        while (PORTS[ch]->available() > 0) PORTS[ch]->read();   // start clean
        while ((int32_t)(millis() - stop_at) < 0) {
            if (Serial.available() > 0) { Serial.read(); break; }
            while (PORTS[ch]->available() > 0) {
                buf[n++] = (uint8_t)PORTS[ch]->read();
                if (n == sizeof(buf)) {
                    // Written unconditionally. An earlier version skipped the
                    // line unless availableForWrite() was comfortable, which
                    // threw away 92% of the stream - the CDC buffer is small
                    // and almost never "comfortable" at 14 kB/s, even though
                    // the link drains it instantly. Passthrough is only ever
                    // run with a host actively reading, so a brief block is
                    // the right trade and a silent 12:1 decimation is not.
                    Serial.print('$');
                    for (uint8_t k = 0; k < n; k++) {
                        if (buf[k] < 0x10) Serial.print('0');
                        Serial.print(buf[k], HEX);
                    }
                    Serial.println();
                    n = 0;
                }
            }
        }
        Serial.println("# END P");
    } else if (text.startsWith("B")) {
        long b = text.substring(1).toInt();
        if (b < 1200 || b > 1000000) {
            Serial.println("# ERR B range");
        } else {
            sensor_baud = (uint32_t)b;
            for (int i = 0; i < 3; i++) {
                PORTS[i]->end();
                PORTS[i]->begin(sensor_baud);
            }
            for (int i = 0; i < 3; i++) {
                byte_count[i] = 0; last_byte_ms[i] = 0; ring_pos[i] = 0;
            }
            Serial.print("# ACK B ");
            Serial.println(sensor_baud);
        }
    } else if (text.startsWith("W")) {
        // W<c>:<hex>. Counters are not touched - a write is often followed by
        // a rate check, and clearing here would hide the before/after.
        int colon = text.indexOf(':');
        if (colon < 2) {
            Serial.println("# ERR W syntax");
            return;
        }
        char which = text.charAt(1);
        String hex = text.substring(colon + 1);
        if (hex.length() < 2 || (hex.length() % 2) != 0) {
            Serial.println("# ERR W hex");
            return;
        }
        uint8_t bytes[24];
        uint8_t n = 0;
        for (unsigned int k = 0; k + 1 < hex.length() && n < sizeof(bytes); k += 2) {
            int hi = hexval(hex.charAt(k)), lo = hexval(hex.charAt(k + 1));
            if (hi < 0 || lo < 0) { Serial.println("# ERR W hex"); return; }
            bytes[n++] = (uint8_t)((hi << 4) | lo);
        }
        int first = 0, last = 2;
        if (which >= '0' && which <= '2') { first = last = which - '0'; }
        else if (which != 'A') { Serial.println("# ERR W chan"); return; }
        for (int i = first; i <= last; i++) {
            PORTS[i]->write(bytes, n);
            PORTS[i]->flush();
        }
        Serial.print("# ACK W ");
        Serial.print(which);
        Serial.print(' ');
        Serial.println(n);
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
        PORTS[i]->begin(sensor_baud);
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
