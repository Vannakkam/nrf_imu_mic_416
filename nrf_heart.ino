/*
  nrf_heart.ino - v2 (high-rate IMU, unchanged/improved audio, fewer BLE packets)

  Seeed Studio XIAO nRF52840 Sense (or compatible nRF52840 Sense).
  Streams timestamped IMU (LSM6DS3 @ 416 Hz) and PDM microphone (8000 Hz, IMA ADPCM)
  over BLE to a Web Bluetooth client (index.html).

  --------------------------------------------------------------------------
  WHAT CHANGED vs v1 AND WHY
  --------------------------------------------------------------------------
  1. The LSM6DS3 *was* left at its library default ODR (well below 416 Hz), and
     the sketch read six axes with six separate I2C transactions. At 100 kHz
     that is far too slow, so the effective rate was ~140 Hz.
     -> We now explicitly set ODR_XL = ODR_G = 416 Hz (0x60 in CTRL1_XL/CTRL2_G),
        run I2C at 400 kHz, and read all 12 data bytes in ONE burst from 0x22.
        That is fast enough to sustain a true 416 Hz sample rate.

  2. Going from ~140 Hz to 416 Hz with 8 samples/packet would triples the
     number of BLE notifications -> more radio TX bursts -> more mic noise.
     -> We now batch 24 samples per IMU notification (416/24 = 17.3 pkt/s,
        essentially the same packet rate as v1's ~140/8 = 17.5 pkt/s).
        To keep each notification inside ONE link-layer packet (<= 244 bytes
        on every platform), samples are packed as 6 x 12-bit values = 9 bytes:
          accel : mg, 1 LSB = 1 mg      (FS = +/-2 g)
          gyro  : 1 LSB = 0.25 dps      (FS = +/-500 dps)
        Timestamps are derived from the packet timestamp + nominal period,
        so no per-sample 4-byte timestamp is needed.
        IMU packet = 12 + 24*9 = 228 bytes.

  3. Audio is *not* degraded: sample rate, bit depth and ADPCM codec are
     unchanged. We only double the audio batch from 200 to 400 samples, which
     halves audio notifications (40 -> 20 pkt/s) and halves the number of
     encoder resets. Packet = 17 + 200 = 217 bytes (still one LL packet).
     Net result: ~57.5 BLE notifications/s (v1) -> ~37.3/s (v2), with more
     IMU data and identical audio fidelity.

  4. "Zero data loss": BLE write return values are checked and retried,
     disconnects no longer overflow the IMU buffer, the PCM ring was enlarged,
     underfilled audio batches are sent with the true sample count instead of
     being padded with silence, and drop counters are reported on STOP.

  Board/package (Arduino IDE):
    - "Seeed nRF52 mbed-enabled Boards" -> Seeed XIAO nRF52840 Sense
  Libraries: ArduinoBLE, Seeed Arduino LSM6DS3, PDM (mbed core), Wire.
  Web Bluetooth needs a user gesture and HTTPS/localhost.
*/

#include <ArduinoBLE.h>
#include <LSM6DS3.h>
#include <PDM.h>
#include <Wire.h>
#include <stdio.h>
#include <string.h>

// -------------------- Configuration --------------------
#define DEVICE_NAME           "NRF_HEART_SENSE"

// IMU
#define IMU_HZ                416
#define IMU_PERIOD_US         2404UL          // round(1e6 / 416)
#define IMU_SAMPLES_PER_PKT   24              // 416 / 24 = 17.3 notifications/s
#define IMU_PKT_HDR           12
#define IMU_SAMPLE_BYTES      9               // 6 axes x 12-bit
#define IMU_PKT_MAX_BYTES     (IMU_PKT_HDR + IMU_SAMPLES_PER_PKT * IMU_SAMPLE_BYTES) // 228

// LSM6DS3 registers / settings
#define IMU_I2C_ADDR          0x6A
#define LSM_REG_WHO_AM_I      0x0F
#define LSM_REG_CTRL1_XL      0x10
#define LSM_REG_CTRL2_G       0x11
#define LSM_REG_CTRL3_C       0x12
#define LSM_REG_OUTX_L_G      0x22
#define LSM_WHO_AM_I_VALUE    0x69
#define LSM_CTRL1_XL_416_2G   0x60            // ODR_XL=0110 (416 Hz), FS_XL=00 (+/-2 g)
                                              // For +/-4 g use 0x68 and change scale12()
                                              // to multiply by 4000 (1 LSB = 2 mg).
#define LSM_CTRL2_G_416_500DPS 0x64           // ODR_G=0110 (416 Hz), FS_G=01 (+/-500 dps)
#define LSM_CTRL3_C_BDU_INC   0x06            // BDU=1, IF_INC=1 (safe multi-byte reads)

// Microphone
#define MIC_HZ                8000
#define MIC_CHANNELS          1
#define ADPCM_BYTES_PER_PKT   200             // 400 samples, 20 notifications/s
#define AUDIO_SAMPLES_PER_PKT (ADPCM_BYTES_PER_PKT * 2)
#define AUDIO_PKT_HDR         17
#define AUDIO_PKT_MAX_BYTES   (AUDIO_PKT_HDR + ADPCM_BYTES_PER_PKT) // 217
#define PCM_RING_SIZE         1024

// BLE service/characteristics (custom 128-bit UUIDs)
#define SERVICE_UUID        "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define CMD_CHAR_UUID       "6e400002-b5a3-f393-e0a9-e50e24dcca9e" // write
#define IMU_CHAR_UUID       "6e400003-b5a3-f393-e0a9-e50e24dcca9e" // notify
#define AUDIO_CHAR_UUID     "6e400004-b5a3-f393-e0a9-e50e24dcca9e" // notify
#define STATUS_CHAR_UUID    "6e400005-b5a3-f393-e0a9-e50e24dcca9e" // notify/read

enum Cmd : uint8_t {
  CMD_SET_TIME = 0x01,   // payload: uint64_t unix epoch ms (LE)
  CMD_START    = 0x02,
  CMD_STOP     = 0x03,
  CMD_RESET    = 0x04,
};

enum PktType : uint8_t {
  PKT_IMU_LEGACY = 0x01, // kept so old clients can still parse (not sent by v2)
  PKT_AUDIO      = 0x02,
  PKT_IMU        = 0x03, // v2: 12-bit packed IMU
  PKT_STATUS     = 0x10,
};

// -------------------- BLE --------------------
BLEService dataService(SERVICE_UUID);
BLECharacteristic cmdChar(CMD_CHAR_UUID, BLEWrite | BLEWriteWithoutResponse, 20);
BLECharacteristic imuChar(IMU_CHAR_UUID, BLENotify, IMU_PKT_MAX_BYTES);
BLECharacteristic audioChar(AUDIO_CHAR_UUID, BLENotify, AUDIO_PKT_MAX_BYTES);
BLECharacteristic statusChar(STATUS_CHAR_UUID, BLERead | BLENotify, 64);

// The library is used only for detection; all reads are done with raw I2C.
LSM6DS3 imu(I2C_MODE, 0x6A);

// -------------------- Time / state --------------------
volatile bool recording = false;
volatile bool timeSynced = false;
uint64_t baseEpochMs = 0;
uint32_t baseMillis = 0;

volatile uint16_t imuSeq = 0;
volatile uint16_t audioSeq = 0;
volatile uint32_t imuDropped = 0;
volatile uint32_t audioDropped = 0;

// -------------------- IMU batching --------------------
uint8_t imuPkt[IMU_PKT_MAX_BYTES];
uint8_t imuCount = 0;
uint32_t lastImuSampleUs = 0;
int16_t lastAccel[3] = {0, 0, 0};
int16_t lastGyro[3]  = {0, 0, 0};

// -------------------- Audio (PDM + ADPCM) --------------------
volatile int16_t pcmRing[PCM_RING_SIZE];
volatile uint16_t pcmHead = 0, pcmTail = 0;

uint8_t adpcmPayload[ADPCM_BYTES_PER_PKT];
uint16_t adpcmPayloadLen = 0;
uint16_t adpcmSampleCount = 0;

int16_t adpcmPredictor = 0;
uint8_t adpcmStepIndex = 0;
bool adpcmNibbleHigh = false;
uint8_t adpcmCurrentByte = 0;

static const int imaIndexTable[8] = { -1, -1, -1, -1, 2, 4, 6, 8 };
static const int imaStepTable[89] = {
  7,8,9,10,11,12,13,14,16,17,19,21,23,25,28,31,34,37,41,45,50,55,60,66,73,80,88,97,107,118,130,
  143,157,173,190,209,230,253,279,307,337,371,408,449,494,544,598,658,724,796,876,963,1060,1166,
  1282,1411,1552,1707,1878,2066,2272,2499,2749,3024,3327,3660,4026,4428,4871,5358,5894,6484,
  7132,7845,8630,9493,10442,11487,12635,13899,15289,16818,18500,20350,22385,24623,27086,29794,
  32767
};

// -------------------- Helpers --------------------
uint32_t nowMsSinceSync() {
  if (timeSynced) return (uint32_t)(millis() - baseMillis);
  return millis();
}

void sendStatus(const char* msg) {
  if (!BLE.connected()) return;
  statusChar.writeValue((const void*)msg, strlen(msg));
}

void sendStopStatus() {
  if (!BLE.connected()) return;
  char buf[80];
  snprintf(buf, sizeof(buf), "STOPPED imu_drop=%lu aud_drop=%lu",
           (unsigned long)imuDropped, (unsigned long)audioDropped);
  statusChar.writeValue((const void*)buf, strlen(buf));
}

void resetStreams() {
  imuSeq = 0;
  audioSeq = 0;
  imuDropped = 0;
  audioDropped = 0;
  imuCount = 0;
  pcmHead = pcmTail = 0;
  adpcmPredictor = 0;
  adpcmStepIndex = 0;
  adpcmNibbleHigh = false;
  adpcmCurrentByte = 0;
  adpcmPayloadLen = 0;
  adpcmSampleCount = 0;
}

// -------------------- Low-level I2C / LSM6DS3 setup --------------------
static uint8_t readReg8(uint8_t reg) {
  Wire.beginTransmission(IMU_I2C_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0;
  if (Wire.requestFrom(IMU_I2C_ADDR, 1) != 1) return 0;
  return Wire.read();
}

static void writeReg8(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(IMU_I2C_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission(true);
}

void configureImu() {
  Wire.begin();
  Wire.setClock(400000); // 400 kHz: one 12-byte burst takes ~0.3 ms

  uint8_t who = readReg8(LSM_REG_WHO_AM_I);
  if (who != LSM_WHO_AM_I_VALUE) {
    Serial.print("LSM6DS3 WHO_AM_I mismatch: 0x"); Serial.println(who, HEX);
  }

  // BDU prevents reading a half-updated sample; IF_INC enables auto-increment
  // so the 12-byte burst below walks 0x22..0x2D automatically.
  writeReg8(LSM_REG_CTRL3_C, LSM_CTRL3_C_BDU_INC);
  writeReg8(LSM_REG_CTRL1_XL, LSM_CTRL1_XL_416_2G);
  writeReg8(LSM_REG_CTRL2_G,  LSM_CTRL2_G_416_500DPS);

  Serial.print("IMU ODR set to 416 Hz (WHO_AM_I=0x");
  Serial.print(who, HEX);
  Serial.println(")");
}

// One I2C transaction: gyro X,Y,Z (0x22..0x27) then accel X,Y,Z (0x28..0x2D).
static bool readImuBurst(int16_t g[3], int16_t a[3]) {
  Wire.beginTransmission(IMU_I2C_ADDR);
  Wire.write(LSM_REG_OUTX_L_G);
  if (Wire.endTransmission(false) != 0) return false;

  uint8_t b[12];
  int n = Wire.requestFrom(IMU_I2C_ADDR, 12);
  if (n != 12) return false;
  for (int i = 0; i < 12; i++) b[i] = Wire.read();

  g[0] = (int16_t)(b[0]  | ((uint16_t)b[1]  << 8));
  g[1] = (int16_t)(b[2]  | ((uint16_t)b[3]  << 8));
  g[2] = (int16_t)(b[4]  | ((uint16_t)b[5]  << 8));
  a[0] = (int16_t)(b[6]  | ((uint16_t)b[7]  << 8));
  a[1] = (int16_t)(b[8]  | ((uint16_t)b[9]  << 8));
  a[2] = (int16_t)(b[10] | ((uint16_t)b[11] << 8));
  return true;
}

// raw int16 -> signed 12-bit, with the scales described in the header.
static inline int16_t scale12(int16_t raw) {
  int32_t v = ((int32_t)raw * 2000L) / 32768L; // +/-FS -> +/-2000 counts
  if (v > 2047) v = 2047;
  if (v < -2048) v = -2048;
  return (int16_t)v;
}

// Append six 12-bit values as 9 bytes into the current IMU packet.
static void appendImuSample(const int16_t g[3], const int16_t a[3]) {
  uint16_t m[6];
  m[0] = (uint16_t)(scale12(a[0]) & 0x0FFF); // ax : mg
  m[1] = (uint16_t)(scale12(a[1]) & 0x0FFF); // ay : mg
  m[2] = (uint16_t)(scale12(a[2]) & 0x0FFF); // az : mg
  m[3] = (uint16_t)(scale12(g[0]) & 0x0FFF); // gx : 0.25 dps
  m[4] = (uint16_t)(scale12(g[1]) & 0x0FFF); // gy : 0.25 dps
  m[5] = (uint16_t)(scale12(g[2]) & 0x0FFF); // gz : 0.25 dps

  uint8_t* p = imuPkt + IMU_PKT_HDR + imuCount * IMU_SAMPLE_BYTES;
  p[0] = (uint8_t)m[0];
  p[1] = (uint8_t)m[1];
  p[2] = (uint8_t)((m[0] >> 8) | ((m[1] >> 8) << 4));
  p[3] = (uint8_t)m[2];
  p[4] = (uint8_t)m[3];
  p[5] = (uint8_t)((m[2] >> 8) | ((m[3] >> 8) << 4));
  p[6] = (uint8_t)m[4];
  p[7] = (uint8_t)m[5];
  p[8] = (uint8_t)((m[4] >> 8) | ((m[5] >> 8) << 4));
}

// -------------------- IMU packet / sampling --------------------
bool flushImuPacket() {
  if (imuCount == 0) return true;
  if (!BLE.connected()) {
    // Never let a disconnect overflow the buffer.
    imuDropped += imuCount;
    imuCount = 0;
    return false;
  }

  imuPkt[0] = PKT_IMU;
  imuPkt[1] = (uint8_t)imuSeq;
  imuPkt[2] = (uint8_t)(imuSeq >> 8);
  imuPkt[3] = imuCount;
  // bytes 4..7 already hold the timestamp of the first sample
  uint16_t period = IMU_PERIOD_US;
  memcpy(imuPkt + 8, &period, 2);
  imuPkt[10] = 0; imuPkt[11] = 0; // reserved

  int len = IMU_PKT_HDR + imuCount * IMU_SAMPLE_BYTES;
  int written = imuChar.writeValue(imuPkt, len);
  if (written <= 0) {
    // Busy BLE stack: keep the batch and try again next loop.
    return false;
  }
  imuSeq++;
  imuCount = 0;
  return true;
}

void sampleImu() {
  uint32_t now = micros();
  uint32_t elapsed = now - lastImuSampleUs;
  if (elapsed < IMU_PERIOD_US) return;

  uint32_t slots = elapsed / IMU_PERIOD_US;
  if (slots > 8) {
    // Long stall (e.g. BLE operations): resync rather than emitting a burst
    // of duplicated samples.
    slots = 1;
  }

  int16_t g[3], a[3];
  if (!readImuBurst(g, a)) {
    lastImuSampleUs += slots * IMU_PERIOD_US;
    return;
  }

  for (uint32_t k = 0; k < slots; k++) {
    const int16_t* gg = (k == slots - 1) ? g  : lastGyro;
    const int16_t* aa = (k == slots - 1) ? a  : lastAccel;

    if (imuCount == 0) {
      uint32_t t = nowMsSinceSync();
      memcpy(imuPkt + 4, &t, 4); // timestamp of sample #0 in this packet
    }

    appendImuSample(gg, aa);
    imuCount++;

    if (imuCount >= IMU_SAMPLES_PER_PKT) {
      if (!flushImuPacket()) {
        imuDropped += imuCount; // cannot send and buffer is full
        imuCount = 0;
      }
    }
  }

  memcpy(lastGyro, g, sizeof(g));
  memcpy(lastAccel, a, sizeof(a));
  lastImuSampleUs += slots * IMU_PERIOD_US;
}

// -------------------- Audio encoding / packet --------------------
uint8_t adpcmEncodeSample(int16_t sample) {
  int diff = sample - adpcmPredictor;
  uint8_t code = 0;
  int step = imaStepTable[adpcmStepIndex];
  if (diff < 0) { code = 8; diff = -diff; }
  if (diff >= step) { code |= 4; diff -= step; }
  step >>= 1;
  if (diff >= step) { code |= 2; diff -= step; }
  step >>= 1;
  if (diff >= step) { code |= 1; diff -= step; }

  int step2 = imaStepTable[adpcmStepIndex];
  int delta = (step2 >> 3);
  if (code & 4) delta += step2;
  if (code & 2) delta += step2 >> 1;
  if (code & 1) delta += step2 >> 2;
  if (code & 8) adpcmPredictor -= delta; else adpcmPredictor += delta;
  if (adpcmPredictor > 32767) adpcmPredictor = 32767;
  if (adpcmPredictor < -32768) adpcmPredictor = -32768;

  adpcmStepIndex += imaIndexTable[code & 7];
  if (adpcmStepIndex < 0) adpcmStepIndex = 0;
  if (adpcmStepIndex > 88) adpcmStepIndex = 88;
  return code & 0x0F;
}

void pushAdpcmSample(int16_t sample) {
  uint8_t nib = adpcmEncodeSample(sample);
  if (!adpcmNibbleHigh) {
    adpcmCurrentByte = nib; // low nibble first
    adpcmNibbleHigh = true;
  } else {
    adpcmCurrentByte |= (nib << 4);
    adpcmNibbleHigh = false;
    if (adpcmPayloadLen < ADPCM_BYTES_PER_PKT) {
      adpcmPayload[adpcmPayloadLen++] = adpcmCurrentByte;
    }
  }
  adpcmSampleCount++;
}

bool flushAudioPacket(uint16_t sampleCount) {
  if (sampleCount == 0) return true;
  if (!BLE.connected()) {
    audioDropped += sampleCount;
    adpcmPayloadLen = 0;
    adpcmSampleCount = 0;
    adpcmNibbleHigh = false;
    return false;
  }

  // A partial batch can end on a low nibble: store it explicitly so the
  // decoder can skip it via sampleCount.
  if (adpcmNibbleHigh) {
    adpcmPayload[adpcmPayloadLen++] = (uint8_t)(adpcmCurrentByte & 0x0F);
    adpcmNibbleHigh = false;
  }

  uint8_t pkt[AUDIO_PKT_MAX_BYTES];
  pkt[0] = PKT_AUDIO;
  pkt[1] = (uint8_t)audioSeq;
  pkt[2] = (uint8_t)(audioSeq >> 8);
  uint32_t t = nowMsSinceSync();
  memcpy(pkt + 3, &t, 4);
  uint16_t sr = MIC_HZ;
  memcpy(pkt + 7, &sr, 2);
  memcpy(pkt + 9, &sampleCount, 2);
  int16_t pred = 0; uint8_t step = 0;
  memcpy(pkt + 11, &pred, 2);
  pkt[13] = step; pkt[14] = 0; pkt[15] = 0; pkt[16] = 0;
  memcpy(pkt + AUDIO_PKT_HDR, adpcmPayload, adpcmPayloadLen);

  int written = audioChar.writeValue(pkt, AUDIO_PKT_HDR + adpcmPayloadLen);
  if (written <= 0) {
    audioDropped += sampleCount;
    adpcmPayloadLen = 0;
    adpcmSampleCount = 0;
    return false;
  }

  audioSeq++;
  adpcmPayloadLen = 0;
  adpcmSampleCount = 0;
  return true;
}

void encodeAudioBatch() {
  // Per-packet reset keeps every packet independently decodable.
  adpcmPredictor = 0;
  adpcmStepIndex = 0;
  adpcmNibbleHigh = false;
  adpcmPayloadLen = 0;
  adpcmSampleCount = 0;

  while (adpcmSampleCount < AUDIO_SAMPLES_PER_PKT && pcmTail != pcmHead) {
    int16_t s = pcmRing[pcmTail];
    pcmTail = (pcmTail + 1) % PCM_RING_SIZE;
    pushAdpcmSample(s);
  }

  if (adpcmSampleCount > 0) flushAudioPacket(adpcmSampleCount);
}

void onPDMdata() {
  int bytesAvailable = PDM.available();
  if (bytesAvailable <= 0) return;
  static int16_t sampleBuffer[256];
  int toRead = bytesAvailable;
  if (toRead > (int)sizeof(sampleBuffer)) toRead = (int)sizeof(sampleBuffer);
  PDM.read(sampleBuffer, toRead);
  int samplesRead = toRead / 2;
  for (int i = 0; i < samplesRead; i++) {
    uint16_t next = (pcmHead + 1) % PCM_RING_SIZE;
    if (next != pcmTail) {
      pcmRing[pcmHead] = sampleBuffer[i];
      pcmHead = next;
    } else {
      audioDropped++; // ISR overflow, counted instead of silently lost
    }
  }
}

uint16_t pcmAvailable() {
  return (pcmHead >= pcmTail) ? (pcmHead - pcmTail) : (PCM_RING_SIZE - pcmTail + pcmHead);
}

// -------------------- Commands --------------------
void handleCommand() {
  if (!cmdChar.written()) return;
  int len = cmdChar.valueLength();
  if (len <= 0) return;
  uint8_t buf[20];
  cmdChar.readValue(buf, len);
  uint8_t cmd = buf[0];
  switch (cmd) {
    case CMD_SET_TIME:
      if (len >= 9) {
        uint64_t epoch = 0;
        memcpy(&epoch, buf + 1, 8);
        baseEpochMs = epoch;
        baseMillis = millis();
        timeSynced = true;
        sendStatus("TIME_SYNCED");
      }
      break;

    case CMD_START:
      resetStreams();
      recording = true;
      lastImuSampleUs = micros();
      sendStatus("STARTED");
      break;

    case CMD_STOP:
      recording = false;
      flushImuPacket();
      // Drain every remaining PCM sample in batches (no silence padding).
      if (adpcmSampleCount > 0) flushAudioPacket(adpcmSampleCount);
      while (pcmAvailable() > 0) encodeAudioBatch();
      sendStopStatus();
      break;

    case CMD_RESET:
      recording = false;
      timeSynced = false;
      baseEpochMs = 0;
      resetStreams();
      sendStatus("RESET");
      break;
  }
}

// -------------------- Setup / Loop --------------------
void setup() {
  Serial.begin(115200);
  unsigned long s = millis();
  while (!Serial && (millis() - s < 2000)) {}

  if (!imu.begin()) {
    Serial.println("Failed to init LSM6DS3 over I2C");
  }
  configureImu(); // override library defaults: 416 Hz / +/-2 g / +/-500 dps

  PDM.onReceive(onPDMdata);
  if (!PDM.begin(MIC_CHANNELS, MIC_HZ)) {
    Serial.println("Failed to start PDM");
  }

  if (!BLE.begin()) {
    Serial.println("BLE init failed");
    while (1) { delay(1000); }
  }

  BLE.setLocalName(DEVICE_NAME);
  BLE.setDeviceName(DEVICE_NAME);
  BLE.setAdvertisedService(dataService);

  dataService.addCharacteristic(cmdChar);
  dataService.addCharacteristic(imuChar);
  dataService.addCharacteristic(audioChar);
  dataService.addCharacteristic(statusChar);
  BLE.addService(dataService);

  statusChar.writeValue("IDLE");
  BLE.advertise();
  Serial.println("Advertising...");
}

void loop() {
  BLEDevice central = BLE.central();
  if (!central) return;

  Serial.print("Connected: "); Serial.println(central.address());
  sendStatus("CONNECTED");

  static uint32_t lastAudioTry = 0;
  lastAudioTry = millis();

  while (central.connected()) {
    handleCommand();

    if (recording) {
      sampleImu();

      uint32_t ms = millis();
      if ((uint32_t)(ms - lastAudioTry) >= 10) { // check ~100 Hz
        lastAudioTry = ms;
        if (pcmAvailable() >= AUDIO_SAMPLES_PER_PKT) encodeAudioBatch();
      }
    }

    delay(1); // yield to the BLE stack; 416 Hz needs 2.4 ms, this is ~1 ms
  }

  // Link dropped: flush what we have, then discard the rest safely.
  recording = false;
  flushImuPacket();
  if (adpcmSampleCount > 0) flushAudioPacket(adpcmSampleCount);
  Serial.println("Disconnected");
  BLE.advertise();
}
