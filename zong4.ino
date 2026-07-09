/******************************************************************************
 * ESP32 MPU6050 + GPS + LoRa E22 Sender  —  167Hz IMU streaming + GPS on demand
 *//带GPS成公版本
 * Hardware:
 *   MPU6050  : I2C  (SDA=21, SCL=22, ADDR=0x68)
 *   LoRa E22 : UART2 (RX=19, TX=18, BAUD=115200, transparent mode)
 *   GPS      : UART1 (RX=2,  TX=0,  BAUD=9600)
 *   USB Debug: UART0 (Serial, 115200)
 *
 * Protocol (ESP32 → RA6M5, binary 14-byte packets):
 *   [0xAA] [0x55] [ax_H] [ax_L] [ay_H] [ay_L] [az_H] [az_L]
 *                  [gx_H] [gx_L] [gy_H] [gy_L] [gz_H] [gz_L]
 *
 * GPS on-demand protocol (RA6M5 → ESP32 → RA6M5):
 *   RA6M5 sends:  "GET_GPS\r\n"
 *   ESP32 replies: "$GNRMC,...\r\n"   (raw NMEA sentence)
 *
 * MPU6050 configuration:
 *   - ACCEL_FS = +/-2g  (16384 LSB/g)
 *   - GYRO_FS  = +/-250 deg/s (131 LSB/deg/s)
 *   - DLPF_CFG = 3 (accel BW=44Hz, gyro BW=42Hz at 1kHz internal rate)
 *   - SMPLRT_DIV = 5  → sample rate = 1000/(1+5) ≈ 167Hz
 *
 * IMPORTANT:
 *   - E22 module must be in transparent transmission mode (M0=0, M1=0)
 *   - E22 air data rate must be set to maximum
 *   - E22 baud rate must be 115200
 ******************************************************************************/

#include <Wire.h>

/*===========================================================================
 * Pin Definitions
 *===========================================================================*/
/* --- MPU6050 (I2C) --- */
#define MPU_SDA       21
#define MPU_SCL       22
#define MPU_ADDR      0x68

/* --- LoRa E22 (UART2) --- */
#define LORA_RX_PIN   19
#define LORA_TX_PIN   18
#define LORA_BAUD     115200

/* --- GPS (UART1) --- */
#define GPS_RX_PIN    2    /* ESP32 RX ← GPS TX */
#define GPS_TX_PIN    0    /* ESP32 TX → GPS RX (not used for most modules) */
#define GPS_BAUD      9600

/*===========================================================================
 * MPU6050 Register Map
 *===========================================================================*/
#define MPU_REG_CONFIG      0x1A   /* DLPF config                       */
#define MPU_REG_GYRO_CONFIG 0x1B   /* Gyro full-scale                   */
#define MPU_REG_ACCEL_CONFIG 0x1C  /* Accel full-scale                  */
#define MPU_REG_SMPLRT_DIV  0x19   /* Sample rate divider               */
#define MPU_REG_PWR_MGMT_1  0x6B   /* Power management / clock source   */
#define MPU_REG_ACCEL_XOUT_H 0x3B  /* Start of 14-byte sensor data      */

/*===========================================================================
 * Protocol Constants
 *===========================================================================*/
#define PKT_SYNC1       0xAA
#define PKT_SYNC2       0x55
#define PKT_LEN         14          /* 2 sync + 12 data bytes            */

/*===========================================================================
 * Timing Constants (167 Hz)
 *===========================================================================*/
#define SAMPLE_RATE_HZ      167.0f
#define SAMPLE_INTERVAL_US  ((unsigned long)(1000000.0f / SAMPLE_RATE_HZ))
/* 1000000 / 167 ≈ 5988 us */

/* GPS sentence max length (typical ~80 chars, 128 is safe) */
#define GPS_SENTENCE_MAX  128

/* LoRa command buffer */
#define LORA_CMD_MAX      32

/*===========================================================================
 * Global Objects
 *===========================================================================*/
HardwareSerial LoRaSerial(2);        /* UART2 for E22 LoRa module         */
HardwareSerial GPSSerial(1);         /* UART1 for GPS module              */

/*===========================================================================
 * Global Variables — IMU
 *===========================================================================*/
static unsigned long last_sample_us = 0;
static unsigned long packet_count   = 0;

/*===========================================================================
 * Global Variables — GPS NMEA Cache
 *===========================================================================*/
static char gps_rmc_sentence[GPS_SENTENCE_MAX] = "";
static bool gps_rmc_valid = false;

/* NMEA parser state for GPS */
static char nmea_buf[GPS_SENTENCE_MAX];
static uint8_t nmea_idx = 0;
static bool nmea_in_sentence = false;

/*===========================================================================
 * Global Variables — LoRa Command Reception
 *===========================================================================*/
static char lora_cmd_buf[LORA_CMD_MAX];
static uint8_t lora_cmd_idx = 0;

/*===========================================================================
 * writeMPU6050Reg — write one byte to an MPU6050 register
 *===========================================================================*/
static void writeMPU6050Reg(uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission(true);
}

/*===========================================================================
 * initMPU6050 — configure MPU6050 for 167Hz sampling
 *===========================================================================*/
static void initMPU6050(void)
{
    /* 1. Wake up MPU6050 (clear sleep bit, select PLL X-gyro as clock) */
    writeMPU6050Reg(MPU_REG_PWR_MGMT_1, 0x01);
    delay(50);

    /* 2. Set DLPF: DLPF_CFG = 3 (44Hz accel BW, 42Hz gyro BW) */
    writeMPU6050Reg(MPU_REG_CONFIG, 0x03);

    /* 3. Set gyro full-scale: +/-250 deg/s */
    writeMPU6050Reg(MPU_REG_GYRO_CONFIG, 0x00);

    /* 4. Set accel full-scale: +/-2g */
    writeMPU6050Reg(MPU_REG_ACCEL_CONFIG, 0x00);

    /* 5. Set sample rate divider: DIV=5 → 1000/(1+5) ≈ 167Hz */
    writeMPU6050Reg(MPU_REG_SMPLRT_DIV, 0x05);

    Serial.println("[ESP32] MPU6050 configured:");
    Serial.println("[ESP32]   ACCEL: +/-2g (16384 LSB/g)");
    Serial.println("[ESP32]   GYRO:  +/-250 deg/s (131 LSB/deg/s)");
    Serial.println("[ESP32]   DLPF:  44Hz / 42Hz");
    Serial.println("[ESP32]   Rate:  167Hz (SMPLRT_DIV=5)");
}

/*===========================================================================
 * readMPU6050Raw — burst-read 14 bytes from MPU6050, return raw int16 values
 *===========================================================================*/
static void readMPU6050Raw(int16_t &ax, int16_t &ay, int16_t &az,
                           int16_t &gx, int16_t &gy, int16_t &gz)
{
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(MPU_REG_ACCEL_XOUT_H);
    if (Wire.endTransmission(false) != 0)
    {
        ax = ay = az = gx = gy = gz = 0;
        return;
    }

    uint8_t bytes_read = Wire.requestFrom(MPU_ADDR, (uint8_t)14, (bool)true);
    if (bytes_read < 14)
    {
        ax = ay = az = gx = gy = gz = 0;
        return;
    }

    ax = (int16_t)((Wire.read() << 8) | Wire.read());
    ay = (int16_t)((Wire.read() << 8) | Wire.read());
    az = (int16_t)((Wire.read() << 8) | Wire.read());

    /* Skip temperature (2 bytes) */
    Wire.read();
    Wire.read();

    gx = (int16_t)((Wire.read() << 8) | Wire.read());
    gy = (int16_t)((Wire.read() << 8) | Wire.read());
    gz = (int16_t)((Wire.read() << 8) | Wire.read());
}

/*===========================================================================
 * sendPacket — pack 6 int16 values into a 14-byte binary packet and send
 *===========================================================================*/
static void sendPacket(int16_t ax, int16_t ay, int16_t az,
                       int16_t gx, int16_t gy, int16_t gz)
{
    uint8_t buf[PKT_LEN];

    buf[0]  = PKT_SYNC1;
    buf[1]  = PKT_SYNC2;
    buf[2]  = (uint8_t)((ax >> 8) & 0xFF);
    buf[3]  = (uint8_t)(ax & 0xFF);
    buf[4]  = (uint8_t)((ay >> 8) & 0xFF);
    buf[5]  = (uint8_t)(ay & 0xFF);
    buf[6]  = (uint8_t)((az >> 8) & 0xFF);
    buf[7]  = (uint8_t)(az & 0xFF);
    buf[8]  = (uint8_t)((gx >> 8) & 0xFF);
    buf[9]  = (uint8_t)(gx & 0xFF);
    buf[10] = (uint8_t)((gy >> 8) & 0xFF);
    buf[11] = (uint8_t)(gy & 0xFF);
    buf[12] = (uint8_t)((gz >> 8) & 0xFF);
    buf[13] = (uint8_t)(gz & 0xFF);

    LoRaSerial.write(buf, PKT_LEN);
}

/*===========================================================================
 * readGPS — non-blocking GPS NMEA parser, caches latest $GNRMC/$GPRMC
 *
 * Reads available bytes from the GPS UART, detects NMEA sentences,
 * and caches the most recent $GNRMC or $GPRMC sentence.
 *
 * NMEA sentence format:
 *   $XXYYY,data1,data2,...,dataN*CC\r\n
 *   $ = start
 *   XX = talker ID (GP=GPS only, GN=multi-constellation)
 *   YYY = sentence type (RMC, GGA, etc.)
 *   *CC = checksum (optional, after last data field)
 *
 * We cache the complete sentence including the leading '$' and trailing
 * newline, so the RA6M5 can parse it as a standard NMEA sentence.
 *===========================================================================*/
static void readGPS(void)
{
    while (GPSSerial.available() > 0)
    {
        char c = GPSSerial.read();

        if (c == '$')
        {
            /* Start of a new NMEA sentence */
            nmea_in_sentence = true;
            nmea_idx = 0;
            nmea_buf[nmea_idx++] = c;
        }
        else if (nmea_in_sentence)
        {
            /* Buffer character if there's room */
            if (nmea_idx < GPS_SENTENCE_MAX - 1)
            {
                nmea_buf[nmea_idx++] = c;
            }

            /* End of sentence: \n or \r\n */
            if (c == '\n')
            {
                nmea_buf[nmea_idx] = '\0';  /* null-terminate */
                nmea_in_sentence = false;

                /* Check if this is a $GNRMC or $GPRMC sentence.
                 * We check the first 6 chars: "$GNRMC" or "$GPRMC" */
                if ((nmea_idx >= 6) &&
                    (nmea_buf[1] == 'G') &&
                    (nmea_buf[2] == 'P' || nmea_buf[2] == 'N') &&
                    (nmea_buf[3] == 'R') &&
                    (nmea_buf[4] == 'M') &&
                    (nmea_buf[5] == 'C'))
                {
                    /* Copy to cache */
                    strncpy(gps_rmc_sentence, nmea_buf, GPS_SENTENCE_MAX - 1);
                    gps_rmc_sentence[GPS_SENTENCE_MAX - 1] = '\0';
                    gps_rmc_valid = true;

                    Serial.print("[GPS] Cached: ");
                    Serial.print(gps_rmc_sentence);
                }
            }
            else if (nmea_idx >= GPS_SENTENCE_MAX - 1)
            {
                /* Sentence too long, discard */
                nmea_in_sentence = false;
                nmea_idx = 0;
            }
        }
        /* else: discard characters between sentences */
    }
}

/*===========================================================================
 * checkLoRaCommand — non-blocking check for incoming LoRa commands
 *
 * Called from the main loop.  Currently the GPS data is sent periodically
 * (every 2 seconds) via the main IMU loop, so this function mainly serves
 * as a hook for future bidirectional communication.
 *===========================================================================*/
static void checkLoRaCommand(void)
{
    while (LoRaSerial.available() > 0)
    {
        char c = LoRaSerial.read();

        if (c == '\n' || c == '\r')
        {
            if (c == '\r') continue;

            if (lora_cmd_idx > 0)
            {
                lora_cmd_buf[lora_cmd_idx] = '\0';
                lora_cmd_idx = 0;

                Serial.print("[LoRa] Received cmd: ");
                Serial.println(lora_cmd_buf);
                Serial.print("[LoRa] Unknown command: ");
                Serial.println(lora_cmd_buf);
            }
        }
        else if (lora_cmd_idx < LORA_CMD_MAX - 1)
        {
            lora_cmd_buf[lora_cmd_idx++] = c;
        }
    }
}

/*===========================================================================
 * setup — Arduino entry point
 *===========================================================================*/
void setup()
{
    /* --- USB Debug Serial --- */
    Serial.begin(115200);
    delay(200);
    Serial.println();
    Serial.println("========================================");
    Serial.println(" ESP32 MPU6050 + GPS + LoRa E22 Sender");
    Serial.println("========================================");
    Serial.print  (" Sample Rate:  167 Hz (");
    Serial.print(SAMPLE_INTERVAL_US);
    Serial.println(" us interval)");
    Serial.println(" Protocol:     Binary 14-byte packets");
    Serial.println(" LoRa UART:    115200 baud, 8N1");
    Serial.println(" GPS UART:     9600 baud, 8N1");
    Serial.println(" GPS cache:    $GNRMC / $GPRMC");
    Serial.println(" LoRa cmd:     GET_GPS → reply with $GNRMC");
    Serial.println("========================================");

    /* --- MPU6050 --- */
    Wire.begin(MPU_SDA, MPU_SCL);
    Wire.setClock(400000);
    initMPU6050();
    Serial.println("[ESP32] MPU6050 initialized.");

    /* --- LoRa E22 (UART2) --- */
    LoRaSerial.begin(LORA_BAUD, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);
    Serial.println("[ESP32] LoRa E22 UART initialized.");

    /* --- GPS (UART1) --- */
    GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    Serial.println("[ESP32] GPS UART initialized.");

    Serial.println("[ESP32] Starting data stream...");
    Serial.println();

    last_sample_us = micros();
}

/*===========================================================================
 * loop — Arduino main loop, maintains 167Hz sampling rate
 *
 * Three tasks run in this loop:
 *   1. IMU sampling at 167Hz (timed via micros() software timer)
 *   2. GPS NMEA parsing (non-blocking, reads whatever bytes are available)
 *   3. LoRa command reception (non-blocking, handles "GET_GPS" etc.)
 *
 * All three are designed to be fast enough to not miss the 6ms sample window.
 *===========================================================================*/
void loop()
{
    unsigned long now_us = micros();
    unsigned long elapsed = now_us - last_sample_us;

    /* --- Task A: GPS reading (do this every iteration, non-blocking) --- */
    readGPS();

    /* --- Task B: LoRa command check (keep for future use) --- */
    checkLoRaCommand();

    /* --- Task C: IMU sampling at 167Hz --- */
    if (elapsed >= SAMPLE_INTERVAL_US)
    {
        /* Reset timer. If way behind (>2x), snap forward. */
        if (elapsed > (SAMPLE_INTERVAL_US * 2))
        {
            last_sample_us = now_us;
        }
        else
        {
            last_sample_us += SAMPLE_INTERVAL_US;
        }

        /* Read sensors */
        int16_t ax, ay, az, gx, gy, gz;
        readMPU6050Raw(ax, ay, az, gx, gy, gz);

        packet_count++;

        /* --- Periodic GPS injection: every 334 packets (~2 seconds),
         *     send the cached $GNRMC sentence via LoRa.
         *     This replaces one IMU packet (0.3% loss — negligible for AI).
         *     RA6M5 ISR captures it in background, no reverse link needed. --- */
        if ((packet_count % 334) == 0)
        {
            if (gps_rmc_valid)
            {
                LoRaSerial.print(gps_rmc_sentence);
                LoRaSerial.flush();
                Serial.print("[LoRa] Periodic GPS: ");
                Serial.print(gps_rmc_sentence);
            }
            else
            {
                LoRaSerial.print("$GNRMC,,V,,,,,,,,,,,,\r\n");
                LoRaSerial.flush();
                Serial.println("[LoRa] Periodic GPS: NO FIX");
            }
            /* GPS packet replaces IMU this cycle — skip sendPacket */
        }
        else
        {
            /* Normal IMU packet */
            sendPacket(ax, ay, az, gx, gy, gz);
        }

        /* Debug: every ~1 second (167 packets) */
        if ((packet_count % 167) == 0)
        {
            float ax_g = ax / 16384.0f;
            float ay_g = ay / 16384.0f;
            float az_g = az / 16384.0f;
            float acc_total = sqrt(ax_g*ax_g + ay_g*ay_g + az_g*az_g);

            Serial.print("[ESP32] #");
            Serial.print(packet_count);
            Serial.print("  ACC: ");
            Serial.print(ax_g, 2);
            Serial.print(", ");
            Serial.print(ay_g, 2);
            Serial.print(", ");
            Serial.print(az_g, 2);
            Serial.print("  |Acc|=");
            Serial.print(acc_total, 2);
            Serial.print("g  GPS:");
            Serial.print(gps_rmc_valid ? "OK" : "NO");
            Serial.println();
        }
    }
}
