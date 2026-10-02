// Streaming firmware for the Manta calibration rig's Teensy 4.0.
//
// It does three things, and the first one is the reason the other two are
// simple:
//
//   1. Asserts the sensor modules' configuration at every boot.
//   2. Decodes WitMotion frames from all three modules concurrently.
//   3. Always streams all three channels, at a switchable output rate.
//
// It does not yet emit degrees, and carries no stored calibration. That part is
// described in SENSORS.md ("Storage", "Writing it to the rig") and deliberately
// is not here: the six-face calibration the host wizard runs needs raw counts
// and nothing else, and a calibration store is not something to design against
// a procedure that has not been run on real hardware yet. The raw mode is the
// mode the wizard needs, and SENSORS.md commits to it being available
// regardless of what is stored.
//
// --- Why the configuration is asserted rather than assumed ------------------
//
// A power cycle after the first bench configuration pass showed it had
// persisted only in part: the RSW change stuck, the baud and rate changes did
// not. The unlock these modules require expires, and a save arriving late is
// rejected silently, because they acknowledge nothing. Tighter sequencing did
// make it persist - but a configuration that persists only when written in the
// right order, with no acknowledgement when it is not, is not a configuration
// to depend on.
//
// So it is reapplied here every boot. Three register writes per module, it is
// idempotent, and it buys three things a persisted configuration does not: a
// module swapped in from the shelf works immediately; the configuration is
// visible in source where it can be reviewed rather than invisible in a
// module's flash; and there is no silent dependency on state written by a tool
// nobody runs twice.
//
// The pass runs at 9600 first and then at 115200, so it converges whether or
// not the previous session's save took. Baud carries its own trap: the change
// applies the moment it is written, so the save that commits it has to be sent
// at the *new* baud with its own fresh unlock. Unlock, baud and save as one
// sequence at the old baud cannot work - the last of the three is transmitted
// into a receiver that has already moved.
//
// Then it measures its own input rate before declaring the sensors ready, so a
// rig that came up at 10 Hz because a module ignored its configuration says so
// rather than advertising 200 Hz and quietly delivering a twentieth of it.
//
// --- The wire format --------------------------------------------------------
//
// One line per output sample, all three channels, always in LEFT, CENTRE,
// RIGHT order:
//
//   {<t_us>:<ax>,<ay>,<az>,<gx>,<gy>,<gz>|<...>|<...>}
//
// Braces, where the Pico's position lines use brackets. SENSORS.md asks raw
// mode to carry a different delimiter from the degrees mode that will exist
// later, so a host that missed a mode-change acknowledgement - reconnected
// mid-run, restarted, lost the reply - physically cannot mistake raw counts for
// degrees. It costs one character and the failure it prevents is writing
// garbage angles into a drone's calibration.
//
// Values are the modules' own int16 counts, unscaled. Scaling them here would
// mean either floats on the wire - and floats admit "nan", which parses
// successfully and would propagate a sensor fault silently - or a units
// decision baked into firmware ahead of the calibration that should make it.
// Counts are exact, parse cheaply, and a fault must use the "x" sentinel and
// cannot masquerade as a number.
//
// t_us is micros() at the moment the line was composed. It wraps at 2^32 us,
// about 71.6 minutes; the host unwraps it. It is what lets the host prove the
// board is sampling at the rate it claims rather than merely that lines are
// arriving.
//
// Commands - newline-terminated ASCII, case-insensitive, tolerant of CRLF:
//
//   S        slow preset: 10 Hz                  -> "# ACK S 10"
//   F        fast preset: 200 Hz                 -> "# ACK F 200"
//   F<hz>    fast at a given rate, e.g. "F50"    -> "# ACK F 50"
//   H        halt the stream                     -> "# ACK H"
//   I        identify                            -> "# ID dev=manta-rig ..."
//   ?        status, including per-channel health -> "# STATUS ..."
//   C        re-assert the sensor configuration  -> "# ACK C"
//   Z        zero the frame counters             -> "# ACK Z"
//   other                                        -> "# ERR <text>"
//
// Every board-to-host reply is prefixed "#", which cannot match the sample
// grammar, so replies are inert to every parser on the host side. Every command
// is short enough to type by hand into a serial terminal, because the console
// is the debugging tool. Both invariants are carried over from
// pico/sampler.py unchanged.

#include <Arduino.h>
#include <EEPROM.h>

// Left, Centre, Right on the three hardware UARTs. Serial1 is pins 0/1,
// Serial2 is 7/8, Serial3 is 14/15. Separate from the USB "Serial" object and
// not competing with it for bandwidth: the sensor side is three real UARTs at
// 115200, the host side is CDC at 480 Mbit.
static HardwareSerial *const PORTS[3] = {&Serial1, &Serial2, &Serial3};
static const char *const NAMES[3] = {"LEFT", "CENTRE", "RIGHT"};
static const uint8_t NCHAN = 3;

static const uint32_t SENSOR_BAUD = 115200;   // what the modules are configured to
static const uint32_t SENSOR_BAUD_DEFAULT = 9600;   // what they revert to

// Output presets, mirroring pico/sampler.py's two-preset shape. SLOW is the
// boot default so plugging the board into a plain terminal gives something
// readable rather than a firehose. FAST is 200 Hz because that is the modules'
// rated ceiling and the rate they are configured to - asking for more would
// duplicate samples, not produce new ones.
static const uint32_t SLOW_HZ = 10;
static const uint32_t FAST_HZ = 200;
static const uint32_t MIN_HZ = 1;
static const uint32_t MAX_HZ = 200;

// A channel whose newest frame is older than this emits the "x" sentinel. Three
// sample periods at 200 Hz is 15 ms; 50 ms is generous enough that a single
// dropped frame does not blank the channel, and short enough that a module
// which has stopped is visible within a couple of output lines at 10 Hz.
static const uint32_t STALE_MS = 50;

// Long enough for "K0:" plus six floats with signs and decimals. The staging
// commands are the only ones that need it; everything else is one or two
// characters, and the console-typeable rule still holds for those.
static const uint8_t MAX_COMMAND_LEN = 160;

// --- WitMotion decode -------------------------------------------------------
//
// Eleven-byte frames, 55 <type> <8 data> <checksum>, checksum being the low byte
// of the sum of the first ten. With the magnetic frame switched off the cycle is
// three frames and 33 bytes per sample set:
//
//   0x51  acceleration      int16 per axis, plus die temperature
//   0x52  angular velocity  int16 per axis
//   0x53  attitude angle    int16 per axis, plus a version word
//
// 0x53 is decoded but not streamed. It is the module's own fused angle, and it
// is kept because it is a free cross-check on the accelerometer-derived one -
// on the bench they agreed to 0.03 degrees - but it is not what the six-face
// calibration consumes and putting it on the wire would widen every line for a
// number the host would ignore.
static const uint8_t FRAME_LEN = 11;
static const uint8_t FRAME_HEADER = 0x55;

struct Channel {
    uint8_t buf[FRAME_LEN];
    uint8_t len;

    int16_t acc[3];
    int16_t gyro[3];
    int16_t angle[3];

    bool have_acc;
    bool have_gyro;
    uint32_t acc_ms;          // freshness is judged on the accel frame, which
                              // is the one the calibration actually needs
    uint32_t acc_seq;         // increments per accel frame; see emit_sample
    uint32_t frames_ok;
    uint32_t frames_bad;
};

static Channel chans[NCHAN];

static uint32_t out_hz = SLOW_HZ;
static bool streaming = true;
static uint32_t next_out_us = 0;
static uint32_t lines_dropped = 0;
static uint32_t last_emit_seq[NCHAN] = {0, 0, 0};
static uint32_t last_emit_ms = 0;

static String pending = "";
static bool sensors_ready = false;
static uint32_t measured_hz[NCHAN] = {0, 0, 0};
static bool emit_corrected = false;   // false = raw counts

static void reset_channel(Channel &c) {
    c.len = 0;
    c.have_acc = false;
    c.have_gyro = false;
    c.acc_ms = 0;
    c.acc_seq = 0;
    c.frames_ok = 0;
    c.frames_bad = 0;
}

// Consume one byte into the channel's frame assembler.
//
// Resynchronisation is the whole subtlety here. On a bad checksum the frame is
// not simply discarded: the buffer is shifted past its first byte and rescanned,
// because a dropped byte on the UART leaves a 0x55 sitting somewhere inside what
// was assembled and throwing the whole eleven away would lose the real frame
// start with it. Discarding whole frames costs one good frame per glitch;
// rescanning costs nothing and recovers on the next byte.
static void feed_byte(Channel &c, uint8_t b) {
    if (c.len == 0 && b != FRAME_HEADER) {
        return;                       // hunting for a header
    }
    c.buf[c.len++] = b;
    if (c.len < FRAME_LEN) {
        return;
    }

    uint8_t sum = 0;
    for (uint8_t i = 0; i < FRAME_LEN - 1; i++) {
        sum = (uint8_t)(sum + c.buf[i]);
    }

    if (sum != c.buf[FRAME_LEN - 1]) {
        c.frames_bad++;
        // Shift past the leading 0x55 and rescan for the next header.
        uint8_t next = 0;
        for (uint8_t i = 1; i < FRAME_LEN; i++) {
            if (c.buf[i] == FRAME_HEADER) {
                next = (uint8_t)(FRAME_LEN - i);
                for (uint8_t k = 0; k < next; k++) {
                    c.buf[k] = c.buf[i + k];
                }
                break;
            }
        }
        c.len = next;
        return;
    }

    c.frames_ok++;
    int16_t v[3];
    for (uint8_t i = 0; i < 3; i++) {
        v[i] = (int16_t)((uint16_t)c.buf[2 + i * 2] |
                         ((uint16_t)c.buf[3 + i * 2] << 8));
    }

    switch (c.buf[1]) {
        case 0x51:
            for (uint8_t i = 0; i < 3; i++) c.acc[i] = v[i];
            c.have_acc = true;
            c.acc_ms = millis();
            c.acc_seq++;
            break;
        case 0x52:
            for (uint8_t i = 0; i < 3; i++) c.gyro[i] = v[i];
            c.have_gyro = true;
            break;
        case 0x53:
            for (uint8_t i = 0; i < 3; i++) c.angle[i] = v[i];
            break;
        default:
            break;                    // 0x54 magnetic, if a module ignores RSW
    }
    c.len = 0;
}

// --- The calibration store --------------------------------------------------
//
// Bias and scale per axis per sensor, from the six-face calibration, plus the
// rotation carrying CENTRE's frame onto this sensor's - which the plate's rigid
// fit measures as a by-product and which a native-format CENTRE datum
// correction needs. Stored as a rotation rather than as an angle because the
// correction is applied to the gravity vector, not to a scalar: subtracting two
// inclinations is only valid when both rotations share an axis, and an airframe
// tilted in roll and pitch does not.
//
// Names are stored alongside the coefficients even though the index implies
// them. It costs a couple of dozen bytes and buys a dump you can read plus a
// check that the record matches the wiring this firmware expects. Left/right
// swaps are exactly the class of fault that produces a plausible-looking and
// completely wrong calibration.
//
// Nothing writes except an explicit "KC". Flash endurance is finite, and a
// store that rewrote itself every boot would eventually be worn out by nothing
// happening. A commit matching what is already there is skipped entirely.

static const uint16_t CAL_VERSION = 1;
static const uint32_t CAL_ADDRESS = 0;

struct SensorCal {
    char name[8];
    float bias[3];
    float scale[3];
    float frame[9];       // rotation from CENTRE's frame to this sensor's
};

struct CalRecord {
    char magic[4];        // "MCAL"
    uint16_t version;
    uint8_t count;
    uint8_t flags;
    SensorCal sensor[NCHAN];
    uint32_t crc;
};

static CalRecord cal;         // what is stored, if valid
static CalRecord staged;      // what "K" commands are building
static bool cal_valid = false;
static bool staged_any = false;

// Bitwise CRC32, the ordinary reflected polynomial. No table: this runs on a
// 216-byte record on an explicit command, so a 1 kB table would cost more than
// the loop ever will.
static uint32_t crc32(const uint8_t *data, size_t len) {
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            crc = (crc >> 1) ^ (0xEDB88320 & (uint32_t)(-(int32_t)(crc & 1)));
        }
    }
    return ~crc;
}

static uint32_t record_crc(const CalRecord &r) {
    return crc32((const uint8_t *)&r, sizeof(CalRecord) - sizeof(uint32_t));
}

static void blank_record(CalRecord &r) {
    memset(&r, 0, sizeof(r));
    memcpy(r.magic, "MCAL", 4);
    r.version = CAL_VERSION;
    r.count = NCHAN;
    for (uint8_t i = 0; i < NCHAN; i++) {
        strncpy(r.sensor[i].name, NAMES[i], sizeof(r.sensor[i].name) - 1);
        for (uint8_t k = 0; k < 3; k++) {
            r.sensor[i].bias[k] = 0.0f;
            r.sensor[i].scale[k] = 1.0f;
        }
        for (uint8_t k = 0; k < 9; k++) {
            r.sensor[i].frame[k] = (k % 4 == 0) ? 1.0f : 0.0f;   // identity
        }
    }
}

static bool load_cal() {
    uint8_t *raw = (uint8_t *)&cal;
    for (size_t i = 0; i < sizeof(CalRecord); i++) {
        raw[i] = EEPROM.read(CAL_ADDRESS + i);
    }
    cal_valid = (memcmp(cal.magic, "MCAL", 4) == 0 &&
                 cal.version == CAL_VERSION &&
                 cal.count == NCHAN &&
                 cal.crc == record_crc(cal));
    return cal_valid;
}

static void store_cal(const CalRecord &r) {
    const uint8_t *raw = (const uint8_t *)&r;
    for (size_t i = 0; i < sizeof(CalRecord); i++) {
        // EEPROM.update semantics: only touch a byte that differs. On an
        // emulated store every write costs endurance, and a re-commit of an
        // identical calibration should cost none.
        if (EEPROM.read(CAL_ADDRESS + i) != raw[i]) {
            EEPROM.write(CAL_ADDRESS + i, raw[i]);
        }
    }
}

// Corrected gravity for one channel, in g. true = (measured - bias) / scale,
// applied per axis to the vector - the native format. The angle, if anyone
// wants one, is formed by the host at the very end.
static void corrected_g(uint8_t index, float out[3]) {
    const SensorCal &c = cal.sensor[index];
    for (uint8_t k = 0; k < 3; k++) {
        float measured = (float)chans[index].acc[k] / 2048.0f;   // 32768/16 g
        float scale = c.scale[k];
        if (scale > -1e-6f && scale < 1e-6f) scale = 1.0f;
        out[k] = (measured - c.bias[k]) / scale;
    }
}

// --- Sensor configuration ---------------------------------------------------
//
// Register writes are FF AA <reg> <lo> <hi>. Unlock and save are themselves
// register writes at 0x69 and 0x00.
static const uint8_t CMD_UNLOCK[5] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
static const uint8_t CMD_SAVE[5]   = {0xFF, 0xAA, 0x00, 0x00, 0x00};

// 0x02 RSW = 0x000E: accel + gyro + angle, magnetic frame off. The magnetic
// frame was 11 of every 44 bytes carrying nothing but zeros - this is a 6-axis
// part with no magnetometer - so dropping it recovers a quarter of the link for
// free and takes the sample set from 44 bytes to 33.
static const uint8_t CMD_RSW[5]    = {0xFF, 0xAA, 0x02, 0x0E, 0x00};
// 0x03 RRATE = 0x000B: 200 Hz, the part's rated ceiling.
static const uint8_t CMD_RRATE[5]  = {0xFF, 0xAA, 0x03, 0x0B, 0x00};
// 0x04 BAUD = 0x0006: 115200 on the sensor UARTs.
static const uint8_t CMD_BAUD[5]   = {0xFF, 0xAA, 0x04, 0x06, 0x00};

static void write_all(const uint8_t *cmd) {
    for (uint8_t i = 0; i < NCHAN; i++) {
        PORTS[i]->write(cmd, 5);
        PORTS[i]->flush();
    }
}

// One unlock-set-save group. Kept tight on purpose: the bench evidence is that
// a group completing in about 1.6 s persisted and groups with a multi-second
// verification sitting between the unlock and the save did not. Nothing may be
// inserted between these three writes.
static void assert_register(const uint8_t *cmd) {
    write_all(CMD_UNLOCK);
    delay(20);
    write_all(cmd);
    delay(20);
    write_all(CMD_SAVE);
    delay(120);                       // the save is a flash write; let it land
}

static void configure_sensors() {
    // Pass one, at the factory default. If the modules are already at 115200
    // these writes land in a receiver running a different baud and are ignored,
    // which is harmless - that is what makes running both passes safe.
    for (uint8_t i = 0; i < NCHAN; i++) {
        PORTS[i]->end();
        PORTS[i]->begin(SENSOR_BAUD_DEFAULT);
    }
    delay(50);
    assert_register(CMD_RSW);

    // Baud last in this pass, because it is the one that moves the receiver out
    // from under us. Unlock and set at 9600, then reopen at 115200 and send a
    // fresh unlock and save into the module's new baud.
    write_all(CMD_UNLOCK);
    delay(20);
    write_all(CMD_BAUD);
    delay(50);

    for (uint8_t i = 0; i < NCHAN; i++) {
        PORTS[i]->end();
        PORTS[i]->begin(SENSOR_BAUD);
    }
    delay(50);
    write_all(CMD_UNLOCK);
    delay(20);
    write_all(CMD_SAVE);
    delay(120);

    // Pass two, at the configured baud. Reasserts RSW for a module that was
    // already at 115200 and so missed pass one, then raises the rate. Rate last
    // overall: free capacity first, then demand.
    assert_register(CMD_RSW);
    assert_register(CMD_RRATE);

    for (uint8_t i = 0; i < NCHAN; i++) {
        while (PORTS[i]->available() > 0) PORTS[i]->read();
        reset_channel(chans[i]);
    }
}

// Measure the real input rate before claiming the sensors are ready. Deltas
// over a window, never against a reset - a counter cleared by a command is only
// as trustworthy as the reset, and one silently swallowed reset on this rig
// once produced a "rate" five times the real one and physically impossible at
// the link speed.
static void measure_rate() {
    uint32_t start[NCHAN];
    for (uint8_t i = 0; i < NCHAN; i++) start[i] = chans[i].frames_ok;

    uint32_t until = millis() + 500;
    while ((int32_t)(millis() - until) < 0) {
        for (uint8_t i = 0; i < NCHAN; i++) {
            while (PORTS[i]->available() > 0) {
                feed_byte(chans[i], (uint8_t)PORTS[i]->read());
            }
        }
    }

    sensors_ready = true;
    for (uint8_t i = 0; i < NCHAN; i++) {
        // Three frames per sample set with the magnetic frame off, so the
        // sample rate is a third of the frame rate.
        measured_hz[i] = (chans[i].frames_ok - start[i]) * 2 / 3;
        if (measured_hz[i] < FAST_HZ / 2) {
            sensors_ready = false;
        }
    }
}

// --- Replies ----------------------------------------------------------------

static void identify() {
    Serial.print("# ID dev=manta-rig proto=1 fw=0.2.0 hw=teensy40 "
                 "chans=LEFT,CENTRE,RIGHT feat=rawstream,rateset,calstore");
    Serial.print(emit_corrected ? " units=microg mode=corrected"
                                : " units=counts mode=raw");
    Serial.print(" cal=");
    if (cal_valid) {
        Serial.print(cal.crc, HEX);
    } else {
        Serial.print("none");
    }
    Serial.print(" rate=");
    Serial.print(out_hz);
    Serial.print(" sensors=");
    Serial.println(sensors_ready ? "ready" : "degraded");
}

static void status() {
    Serial.print("# STATUS uptime_ms=");
    Serial.print(millis());
    Serial.print(" hz=");
    Serial.print(out_hz);
    Serial.print(" streaming=");
    Serial.print(streaming ? 1 : 0);
    Serial.print(" dropped=");
    Serial.print(lines_dropped);
    Serial.print(" sensors=");
    Serial.print(sensors_ready ? "ready" : "degraded");
    for (uint8_t i = 0; i < NCHAN; i++) {
        Serial.print(' ');
        Serial.print(NAMES[i][0]);
        Serial.print("=ok:");
        Serial.print(chans[i].frames_ok);
        Serial.print(",bad:");
        Serial.print(chans[i].frames_bad);
        Serial.print(",hz:");
        Serial.print(measured_hz[i]);
        Serial.print(",age_ms:");
        Serial.print(chans[i].acc_ms ? (long)(millis() - chans[i].acc_ms) : -1L);
    }
    Serial.println();
}

static void set_rate(uint32_t hz) {
    if (hz < MIN_HZ) hz = MIN_HZ;
    if (hz > MAX_HZ) hz = MAX_HZ;
    out_hz = hz;
    streaming = true;
    next_out_us = micros();
}

static void dump_cal() {
    if (!cal_valid) {
        Serial.println("# CAL none");
        return;
    }
    Serial.print("# CAL ver=");
    Serial.print(cal.version);
    Serial.print(" crc=");
    Serial.println(cal.crc, HEX);
    for (uint8_t i = 0; i < NCHAN; i++) {
        const SensorCal &c = cal.sensor[i];
        Serial.print("# CAL ");
        Serial.print(c.name);
        Serial.print(" bias=");
        for (uint8_t k = 0; k < 3; k++) {
            if (k) Serial.print(',');
            Serial.print(c.bias[k], 6);
        }
        Serial.print(" scale=");
        for (uint8_t k = 0; k < 3; k++) {
            if (k) Serial.print(',');
            Serial.print(c.scale[k], 6);
        }
        Serial.println();

        // The datum rotation on its own line rather than widening the one
        // above. It is staged by a separate command and it is nine numbers, so
        // a host that writes it must be able to read it back and check it -
        // an earlier version stored this and never dumped it, which made the
        // one coefficient nothing could verify.
        Serial.print("# CAL ");
        Serial.print(c.name);
        Serial.print(" frame=");
        for (uint8_t k = 0; k < 9; k++) {
            if (k) Serial.print(',');
            Serial.print(c.frame[k], 6);
        }
        Serial.println();
    }
    Serial.println("# CAL end");
}

// Pull `count` comma-separated floats out of text starting at `from`.
static bool parse_floats(const String &text, int from, float *out, uint8_t count) {
    for (uint8_t k = 0; k < count; k++) {
        if (from < 0 || from > (int)text.length()) return false;
        int comma = text.indexOf(',', from);
        String field = (k == count - 1)
            ? text.substring(from)
            : text.substring(from, comma < 0 ? text.length() : comma);
        field.trim();
        if (field.length() == 0) return false;
        out[k] = field.toFloat();
        if (k < count - 1) {
            if (comma < 0) return false;
            from = comma + 1;
        }
    }
    return true;
}

static void stage_sensor(const String &text) {
    // K<i>:<bx>,<by>,<bz>,<sx>,<sy>,<sz>
    int colon = text.indexOf(':');
    if (colon != 2) { Serial.println("# ERR K syntax"); return; }
    char which = text.charAt(1);
    if (which < '0' || which >= '0' + NCHAN) { Serial.println("# ERR K chan"); return; }
    uint8_t index = (uint8_t)(which - '0');

    float v[6];
    if (!parse_floats(text, colon + 1, v, 6)) {
        Serial.println("# ERR K numbers");
        return;
    }
    // Refuse obvious nonsense here rather than storing it. The host gates on
    // the same envelope, but a board that will accept any number at all is a
    // board that can be left holding a calibration nothing ever checked.
    for (uint8_t k = 0; k < 3; k++) {
        if (v[3 + k] < 0.9f || v[3 + k] > 1.1f) {
            Serial.println("# ERR K scale range");
            return;
        }
        if (v[k] < -0.1f || v[k] > 0.1f) {
            Serial.println("# ERR K bias range");
            return;
        }
    }

    if (!staged_any) { blank_record(staged); staged_any = true; }
    for (uint8_t k = 0; k < 3; k++) {
        staged.sensor[index].bias[k] = v[k];
        staged.sensor[index].scale[k] = v[3 + k];
    }
    Serial.print("# ACK K ");
    Serial.println(index);
}

static void stage_frame(const String &text) {
    // KF<i>:<r00>,...,<r22>, the rotation from CENTRE's frame to this sensor's.
    int colon = text.indexOf(':');
    if (colon != 3) { Serial.println("# ERR KF syntax"); return; }
    char which = text.charAt(2);
    if (which < '0' || which >= '0' + NCHAN) { Serial.println("# ERR KF chan"); return; }
    uint8_t index = (uint8_t)(which - '0');

    float m[9];
    if (!parse_floats(text, colon + 1, m, 9)) {
        Serial.println("# ERR KF numbers");
        return;
    }
    if (!staged_any) { blank_record(staged); staged_any = true; }
    for (uint8_t k = 0; k < 9; k++) staged.sensor[index].frame[k] = m[k];
    Serial.print("# ACK KF ");
    Serial.println(index);
}

static void commit_cal() {
    if (!staged_any) { Serial.println("# ERR KC nothing staged"); return; }

    staged.crc = record_crc(staged);
    if (cal_valid && memcmp(&cal, &staged, sizeof(CalRecord)) == 0) {
        Serial.print("# ACK KC unchanged crc=");
        Serial.println(cal.crc, HEX);
        staged_any = false;
        return;
    }

    store_cal(staged);
    load_cal();                       // read back through the same path
    staged_any = false;

    if (!cal_valid) {
        Serial.println("# ERR KC readback failed");
        return;
    }
    Serial.print("# ACK KC crc=");
    Serial.println(cal.crc, HEX);
}

static void apply_command(const String &raw) {
    String text = raw;
    text.trim();
    text.toUpperCase();

    if (text.length() == 0) {
        return;
    }
    // Replies are inert as commands. A host TTY's line discipline echoes
    // whatever is in its buffer back at the board when the port is first
    // opened, which on the bring-up build handed the parser its own boot banner.
    // Every board-to-host line starts with "#", so ignoring it here makes that
    // whole class of loopback harmless.
    if (text.charAt(0) == '#') {
        return;
    }

    if (text == "S") {
        set_rate(SLOW_HZ);
        Serial.print("# ACK S ");
        Serial.println(out_hz);
    } else if (text.startsWith("F")) {
        long hz = text.length() > 1 ? text.substring(1).toInt() : (long)FAST_HZ;
        if (hz <= 0) hz = (long)FAST_HZ;
        set_rate((uint32_t)hz);
        Serial.print("# ACK F ");
        Serial.println(out_hz);
    } else if (text == "H") {
        streaming = false;
        Serial.println("# ACK H");
    } else if (text == "I") {
        identify();
    } else if (text == "?") {
        status();
    } else if (text == "C") {
        // Re-assert on demand. The stream stops for the duration, because the
        // modules are being reconfigured underneath it and anything emitted
        // while a baud change is in flight is not a reading.
        bool was = streaming;
        streaming = false;
        configure_sensors();
        measure_rate();
        streaming = was;
        next_out_us = micros();
        Serial.println("# ACK C");
        status();
    } else if (text == "K?") {
        dump_cal();
    } else if (text == "KC") {
        commit_cal();
    } else if (text == "KX") {
        blank_record(staged);
        staged.crc = record_crc(staged);
        // Clearing means storing a record whose magic will not validate, so a
        // half-erased store cannot read as a valid all-identity calibration.
        memset(staged.magic, 0, 4);
        store_cal(staged);
        load_cal();
        staged_any = false;
        emit_corrected = false;
        Serial.println("# ACK KX");
    } else if (text.startsWith("KF")) {
        stage_frame(text);
    } else if (text.startsWith("K")) {
        stage_sensor(text);
    } else if (text == "R") {
        emit_corrected = false;
        Serial.println("# ACK R");
    } else if (text == "D") {
        // Refuse to emit corrected values with nothing to correct by, and say
        // so. A flag would get ignored, and the consequence is a drone trimmed
        // against wrong angles.
        if (!cal_valid) {
            Serial.println("# ERR D no calibration");
        } else {
            emit_corrected = true;
            Serial.println("# ACK D");
        }
    } else if (text == "Z") {
        for (uint8_t i = 0; i < NCHAN; i++) {
            chans[i].frames_ok = 0;
            chans[i].frames_bad = 0;
        }
        lines_dropped = 0;
        Serial.println("# ACK Z");
    } else {
        Serial.print("# ERR ");
        Serial.println(text);
    }
}

// --- Sample line ------------------------------------------------------------

// Is there anything new to say?
//
// The fast preset asks for 200 Hz from modules that produce 200 Hz, so a purely
// timer-driven emit would sometimes repeat the previous reading and sometimes
// skip one, depending on how the two clocks drift against each other. The mean
// of a hold survives that, but the *scatter* does not: duplicated samples are
// perfectly correlated, so they inflate n while contributing no variance and the
// reported hold noise comes out better than the sensor really is. That figure is
// one of the three things the calibration offers as a quality measure, and a
// quality measure that flatters itself is worse than none.
//
// So a line is emitted only once at least one live channel has decoded a new
// acceleration frame. Below the sensor rate this is always true and costs
// nothing; at the sensor rate it turns the output into one line per sample set.
//
// The staleness escape matters as much: if every channel has gone silent there
// is never new data, and a board that answered by emitting nothing would look
// identical to a board that had crashed. After STALE_MS it emits anyway, which
// is a line of "x" sentinels saying the board is alive and the sensors are not.
static bool ready_to_emit() {
    for (uint8_t i = 0; i < NCHAN; i++) {
        if (chans[i].acc_seq != last_emit_seq[i]) {
            return true;
        }
    }
    return (millis() - last_emit_ms) >= STALE_MS;
}

static void emit_sample() {
    uint32_t now_ms = millis();

    // Compose into a buffer rather than printing field by field, so the
    // writability check below applies to the whole line. A line half-written
    // into a backed-up endpoint is worse than a line not written at all: it
    // parses as a different, shorter, plausible line.
    // Longest line is a timestamp at its 10-digit maximum plus three channels
    // of three signed micro-g fields (up to "-16000000") and three gyro counts,
    // which comes to about 130 characters. 224 has margin, and the append helper refuses to run past it regardless: snprintf
    // returns the length it *would* have written, so tracking that number
    // without checking it is how a buffer overrun turns into a silently
    // truncated line - and a truncated line here is not a corrupt line, it is a
    // shorter well-formed one that parses as different readings.
    char line[224];
    int n = 0;
    bool overflow = false;

    #define APPEND(...) do {                                                  \
        if (!overflow) {                                                      \
            int _w = snprintf(line + n, sizeof(line) - n, __VA_ARGS__);        \
            if (_w < 0 || _w >= (int)(sizeof(line) - n)) overflow = true;      \
            else n += _w;                                                     \
        }                                                                     \
    } while (0)

    // Braces for raw counts, brackets for corrected. A host that missed a
    // mode-change acknowledgement - reconnected mid-run, restarted, lost the
    // reply - then physically cannot mistake one for the other. It costs one
    // character and the failure it prevents is a calibration written against
    // numbers that were already corrected once.
    APPEND(emit_corrected ? "[%lu:" : "{%lu:", (unsigned long)micros());

    for (uint8_t i = 0; i < NCHAN; i++) {
        if (i) {
            APPEND("|");
        }
        Channel &c = chans[i];
        bool fresh = c.have_acc && c.have_gyro &&
                     (now_ms - c.acc_ms) <= STALE_MS;
        if (!fresh) {
            APPEND("x");
            continue;
        }
        if (!emit_corrected) {
            APPEND("%d,%d,%d,%d,%d,%d",
                   c.acc[0], c.acc[1], c.acc[2],
                   c.gyro[0], c.gyro[1], c.gyro[2]);
            continue;
        }

        // Corrected gravity, in micro-g, as a vector. Not an angle: the
        // correction belongs in the native format, and collapsing to a scalar
        // here would throw away the magnitude - which is one of the two trust
        // signals - and bake in a choice of plane that is the host's to make.
        //
        // Micro-g rather than milli-g because the modules' own noise floor is
        // 0.4 to 1.4 mg, so milli-g would quantise at the noise. One count of
        // the raw sensor is 488 ug, so this cannot lose anything either.
        float g[3];
        corrected_g(i, g);
        APPEND("%ld,%ld,%ld,%d,%d,%d",
               (long)(g[0] * 1000000.0f),
               (long)(g[1] * 1000000.0f),
               (long)(g[2] * 1000000.0f),
               c.gyro[0], c.gyro[1], c.gyro[2]);
    }
    APPEND(emit_corrected ? "]\n" : "}\n");
    #undef APPEND

    if (overflow) {
        lines_dropped++;
        return;
    }

    // Serial.print on a Teensy blocks if the USB endpoint backs up while a host
    // is attached but not reading, and that stall blocks the UART drain loop and
    // drops sensor frames - silent data loss that looks like a flaky sensor.
    // Drop the line instead and count it: a missing line is visible in the
    // timestamps, and "dropped" in the status reply names the cause outright.
    //
    // The bring-up build's passthrough learned the other half of this the hard
    // way, where an unconditional skip threw away 92% of the stream. The
    // difference is that this checks against the line's actual length rather
    // than against whatever the CDC buffer happens to feel comfortable with, and
    // at 200 Hz a ~90 byte line every 5 ms is well inside what the endpoint
    // drains.
    if (Serial.availableForWrite() < n) {
        lines_dropped++;
        return;
    }
    Serial.write((const uint8_t *)line, n);
    for (uint8_t i = 0; i < NCHAN; i++) {
        last_emit_seq[i] = chans[i].acc_seq;
    }
    last_emit_ms = now_ms;
}

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);             // ignored on USB CDC; kept for habit
    for (uint8_t i = 0; i < NCHAN; i++) {
        reset_channel(chans[i]);
        PORTS[i]->begin(SENSOR_BAUD);
    }

    // No wait-for-host loop: the board must run standalone on the rig, and a
    // blocking wait here would make an unattended power-on look like a hang.
    delay(200);
    Serial.println("# MANTA rig ready mode=raw");

    // Load before configuring, so the identity reply after configuration
    // already knows whether there is a calibration. Boot always comes up in
    // raw mode regardless: a remembered mode is a way for a host to be handed
    // corrected numbers it did not ask for.
    blank_record(staged);
    load_cal();

    configure_sensors();
    measure_rate();
    identify();
    if (!sensors_ready) {
        Serial.println("# WARN sensors below expected rate; see ? for per-channel");
    }
    next_out_us = micros();
}

void loop() {
    // Drain all three UARTs every pass, before anything else. At 200 Hz each
    // module delivers 33 bytes every 5 ms; the hardware FIFOs are small, so
    // this loop's first duty is emptying them.
    for (uint8_t i = 0; i < NCHAN; i++) {
        while (PORTS[i]->available() > 0) {
            feed_byte(chans[i], (uint8_t)PORTS[i]->read());
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

    if (streaming) {
        uint32_t period = 1000000UL / out_hz;
        if ((int32_t)(micros() - next_out_us) >= 0 && ready_to_emit()) {
            emit_sample();
            // Advance by the period rather than from now, so the output cadence
            // does not drift with however long emit_sample took. If the board
            // ever falls a whole period behind, resynchronise instead of
            // spinning to catch up - a burst of back-to-back lines would be a
            // worse lie about the sample rate than a gap is.
            next_out_us += period;
            if ((int32_t)(micros() - next_out_us) >= (int32_t)period) {
                next_out_us = micros() + period;
            }
        }
    }

    // Heartbeat, so "is it running" is answerable without a terminal.
    digitalWrite(LED_BUILTIN, (millis() / 250) % 2 ? HIGH : LOW);
}
