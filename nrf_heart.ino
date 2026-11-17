/*
  nrf_heart.ino - v4 (true 416 Hz IMU via hardware FIFO, continuous ADPCM)

  Seeed Studio XIAO nRF52840 Sense (or compatible nRF52840 Sense).
  Streams timestamped IMU (LSM6DS3 @ 416 Hz) and PDM microphone (8000 Hz, IMA ADPCM)
  over BLE to a Web Bluetooth client (index.html).

  --------------------------------------------------------------------------
  WHAT CHANGED vs v1 AND WHY
  --------------------------------------------------------------------------
  1. The LSM6DS3 *was* left at its library default ODR (well below 416 Hz), and
     the sketch read six axes with six separate I2C transactions. At 100 kHz
     that is far too slow, so the effective rate was ~140 Hz.
     -> We now explicitly set ODR_XL = ODR_G = 416 Hz (0x60 in CTRL1_XL/CTRL2_G)
        and read all 12 data bytes in ONE burst from 0x22 through the LSM6DS3
        library (board's internal IMU bus, not the external header bus).
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

  3. Audio remains 8 kHz / 16-bit IMA ADPCM. The batch is 400 samples, so
     audio notifications stay at 20 pkt/s. The predictor and step index are
     now carried across packets in the audio header instead of being reset to
     zero every packet. This removes the 20 Hz "ramp from silence" artifact at
     packet boundaries while keeping every packet independently decodable.
     Packet = 17 + 200 = 217 bytes. Net rate: ~37.3 notifications/s total.

  4. "Zero data loss" and visible loss: IMU packets are queued and a busy BLE
     stack is retried instead of dropping a whole batch. Skipped samples,
     long-stall losses, write failures, queue overflows and PCM overflows are
     counted and reported in STAT. Disconnect and STOP flush safely.

  5. The peripheral asks Android for a 7.5-15 ms connection interval. The
     client re-sends system time on every START so absolute timestamps are
     valid after Reset/Stop/Start cycles.

  6. v4: the main loop cannot run every 2.4 ms because BLE notification writes
     block it. That made the old polling sampler skip ~800 samples per capture
     (STAT skip=800) even though nothing was lost on the BLE side. The LSM6DS3
     internal FIFO (4096 words) now stores gyro+accel sample sets by itself at
     the sensor's own 416 Hz ODR, independent of the loop. The loop just drains
     complete 6-word sample sets in bursts and queues the same 24-sample BLE
     packets as before. FIFO overrun/full events are counted in STAT (fo/fu).
     IMU timestamps are derived from a START anchor + sample index, so they are
     exactly 2404 us apart regardless of when the FIFO happens to be drained.

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

uint16_t pcmAvailable(); // forward declaration for STAT diagnostics

// -------------------- Configuration --------------------
#define DEVICE_NAME           "NRF_HEART_SENSE"

// IMU
#define IMU_HZ                416
#define IMU_PERIOD_US         2404UL          // round(1e6 / 416)
#define IMU_SAMPLES_PER_PKT   24              // 416 / 24 = 17.3 notifications/s
#define IMU_PKT_HDR           12
#define IMU_SAMPLE_BYTES      9               // 6 axes x 12-bit
#define IMU_PKT_MAX_BYTES     (IMU_PKT_HDR + IMU_SAMPLES_PER_PKT * IMU_SAMPLE_BYTES) // 228
#define IMU_PKT_QUEUE_LEN     4               // absorb BLE stalls without losing samples

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

// LSM6DS3 FIFO. One sample set = gyro X/Y/Z then accel X/Y/Z = 6 words = 12 bytes.
#define LSM_REG_FIFO_CTRL2    0x07
#define LSM_REG_FIFO_CTRL3    0x08
#define LSM_REG_FIFO_CTRL4    0x09
#define LSM_REG_FIFO_CTRL5    0x0A
#define LSM_REG_FIFO_STATUS1  0x3A
#define LSM_REG_FIFO_STATUS2  0x3B
#define LSM_REG_FIFO_DATA_OUT_L 0x3E
#define LSM_FIFO_CTRL3_VALUE  0x09            // XL no decimation (0x01) + G no decimation (0x08)
#define LSM_FIFO_CTRL4_VALUE  0x00            // no 3rd/4th dataset, no timestamp
// Per ST AN4650 8.3: ODR_FIFO = max(g,y ODRs) = 416 Hz (0x06 << 3 = 0x30).
// MODE=0x06 => continuous (new samples overwrite the oldest when full).
#define LSM_FIFO_CTRL5_STREAM 0x36
#define LSM_FIFO_CTRL5_BYPASS 0x30
#define LSM_FIFO_WORDS_PER_SET 6
#define LSM_FIFO_BYTES_PER_SET 12
#define LSM_FIFO_STATUS2_EMPTY 0x10
#define LSM_FIFO_STATUS2_FULL  0x20
#define LSM_FIFO_STATUS2_OVR   0x40
#define LSM_FIFO_READ_SETS     4               // 4 sets = 48 bytes per I2C burst

// Microphone. The nRF52840 mbed PDM library only supports 16000 or 41667 Hz.
// We sample the hardware at 16 kHz and keep every other sample, so the stream
// sent over BLE remains exactly 8000 Hz as before.
#define MIC_HZ                8000
#define PDM_HW_HZ             16000
#define PDM_DECIMATE          2
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
#define STATUS_CHAR_MAX_BYTES 240

enum Cmd : uint8_t {
  CMD_SET_TIME = 0x01,   // payload: uint64_t unix epoch ms (LE)
  CMD_START    = 0x02,
  CMD_STOP     = 0x03,
  CMD_RESET    = 0x04,
  CMD_PING     = 0x05,   // device sends one probe notification on each stream
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
BLECharacteristic statusChar(STATUS_CHAR_UUID, BLERead | BLENotify, STATUS_CHAR_MAX_BYTES);

// All IMU register access goes through this object so the correct internal
// I2C bus is used on the XIAO Sense.
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

// Diagnostic counters (reported in STAT status messages every 2 s).
uint32_t imuSampleTotal = 0;
uint32_t audioSampleTotal = 0;
uint32_t imuPktSent = 0;
uint32_t audioPktSent = 0;
uint32_t imuWriteFail = 0;
uint32_t audioWriteFail = 0;
uint32_t imuI2cFail = 0;
uint32_t imuSkipped = 0;     // kept for STAT compatibility (always 0 with FIFO)
uint32_t imuStallLoss = 0;   // kept for STAT compatibility (always 0 with FIFO)
uint32_t imuQueueDrops = 0;  // full-batch drops when the IMU queue overflowed
uint32_t fifoOverruns = 0;   // sensor FIFO overrun events
uint32_t fifoFull = 0;       // sensor FIFO was found full
uint32_t fifoI2cFail = 0;    // FIFO status/data read failures
bool pdmOk = false;

// -------------------- IMU batching --------------------
uint8_t imuPkt[IMU_PKT_MAX_BYTES];       // batch currently being filled
uint8_t imuCount = 0;
uint32_t imuStartMs = 0;                 // device time at START (for FIFO sample timestamps)
uint32_t imuSampleIndex = 0;             // number of FIFO samples emitted since START
bool fifoDiscardFirst = false;           // discard the first (possibly partial) FIFO set

// Completed packets wait here while BLE is busy. Sequence numbers are assigned
// when a packet is queued, so a queue overflow is visible as a client-side gap.
uint8_t imuPktQueue[IMU_PKT_QUEUE_LEN][IMU_PKT_MAX_BYTES];
uint8_t imuPktQueueLen[IMU_PKT_QUEUE_LEN];
uint8_t imuPktQueueSamples[IMU_PKT_QUEUE_LEN];
volatile uint8_t imuQueueHead = 0;
volatile uint8_t imuQueueTail = 0;
volatile uint8_t imuQueueCount = 0;

// -------------------- Audio (PDM + ADPCM) --------------------
volatile int16_t pcmRing[PCM_RING_SIZE];
volatile uint16_t pcmHead = 0, pcmTail = 0;
uint8_t pdmDecimatePhase = 0; // keep every 2nd 16 kHz sample -> 8 kHz output

uint8_t adpcmPayload[ADPCM_BYTES_PER_PKT];
uint16_t adpcmPayloadLen = 0;
uint16_t adpcmSampleCount = 0;

int16_t adpcmPredictor = 0;
uint8_t adpcmStepIndex = 0;
int16_t adpcmBatchStartPred = 0;  // state at the first sample of this packet
uint8_t adpcmBatchStartStep = 0;
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

void sendStatsStatus() {
  if (!BLE.connected()) return;
  char buf[STATUS_CHAR_MAX_BYTES];
  snprintf(buf, sizeof(buf),
           "STAT is=%lu as=%lu ip=%lu ap=%lu ie=%lu iw=%lu aw=%lu "
           "id=%lu ad=%lu skip=%lu stall=%lu iq=%lu fo=%lu fu=%lu ff=%lu "
           "pa=%u pdm=%d",
           (unsigned long)imuSampleTotal, (unsigned long)audioSampleTotal,
           (unsigned long)imuPktSent, (unsigned long)audioPktSent,
           (unsigned long)(imuI2cFail + fifoI2cFail), (unsigned long)imuWriteFail,
           (unsigned long)audioWriteFail, (unsigned long)imuDropped,
           (unsigned long)audioDropped, (unsigned long)imuSkipped,
           (unsigned long)imuStallLoss, (unsigned long)imuQueueDrops,
           (unsigned long)fifoOverruns, (unsigned long)fifoFull,
           (unsigned long)fifoI2cFail,
           (unsigned)pcmAvailable(), pdmOk ? 1 : 0);
  statusChar.writeValue((const void*)buf, strlen(buf));
}

void resetStreams() {
  imuSeq = 0;
  audioSeq = 0;
  imuDropped = 0;
  audioDropped = 0;
  imuSampleTotal = 0;
  audioSampleTotal = 0;
  imuPktSent = 0;
  audioPktSent = 0;
  imuWriteFail = 0;
  audioWriteFail = 0;
  imuI2cFail = 0;
  imuSkipped = 0;
  imuStallLoss = 0;
  imuQueueDrops = 0;
  fifoOverruns = 0;
  fifoFull = 0;
  fifoI2cFail = 0;
  imuCount = 0;
  imuStartMs = nowMsSinceSync();
  imuSampleIndex = 0;
  fifoDiscardFirst = true;
  imu.fifoClear();  // discard anything the sensor buffered before START
  imuQueueHead = imuQueueTail = imuQueueCount = 0;
  pcmHead = pcmTail = 0;
  pdmDecimatePhase = 0;
  adpcmPredictor = 0;
  adpcmStepIndex = 0;
  adpcmBatchStartPred = 0;
  adpcmBatchStartStep = 0;
  adpcmNibbleHigh = false;
  adpcmCurrentByte = 0;
  adpcmPayloadLen = 0;
  adpcmSampleCount = 0;
}

// -------------------- Low-level I2C / LSM6DS3 setup --------------------
// IMPORTANT: use the LSM6DS3 library object for all register access. On the
// XIAO Sense the library talks to the *internal* IMU bus; raw Wire calls may
// address the external header bus and silently fail.
static uint8_t readReg8(uint8_t reg) {
  uint8_t v = 0;
  imu.readRegister(&v, reg);
  return v;
}

static void writeReg8(uint8_t reg, uint8_t value) {
  imu.writeRegister(reg, value);
}

void configureImu() {
  // imu.begin() already initialized the correct I2C bus. Only change speed.
  Wire.setClock(400000);

  uint8_t who = readReg8(LSM_REG_WHO_AM_I);
  if (who != LSM_WHO_AM_I_VALUE) {
    Serial.print("LSM6DS3 WHO_AM_I mismatch: 0x"); Serial.println(who, HEX);
  }

  // BDU prevents reading a half-updated sample; IF_INC enables auto-increment
  // so the 12-byte burst below walks 0x22..0x2D automatically.
  writeReg8(LSM_REG_CTRL3_C, LSM_CTRL3_C_BDU_INC);
  writeReg8(LSM_REG_CTRL1_XL, LSM_CTRL1_XL_416_2G);
  writeReg8(LSM_REG_CTRL2_G,  LSM_CTRL2_G_416_500DPS);

  // Hardware FIFO: gyro + accel, no decimation, ODR_FIFO = 416 Hz (max sensor
  // ODR), continuous mode. The sensor now captures every 416 Hz sample set by
  // itself; the main loop only has to drain complete sets in bursts.
  writeReg8(LSM_REG_FIFO_CTRL2, 0x00);
  writeReg8(LSM_REG_FIFO_CTRL3, LSM_FIFO_CTRL3_VALUE);
  writeReg8(LSM_REG_FIFO_CTRL4, LSM_FIFO_CTRL4_VALUE);
  writeReg8(LSM_REG_FIFO_CTRL5, LSM_FIFO_CTRL5_STREAM);

  Serial.print("IMU ODR set to 416 Hz, FIFO stream enabled (WHO_AM_I=0x");
  Serial.print(who, HEX);
  Serial.println(")");
}

// One I2C transaction: gyro X,Y,Z (0x22..0x27) then accel X,Y,Z (0x28..0x2D).
static bool readImuBurst(int16_t g[3], int16_t a[3]) {
  uint8_t b[12];
  if (imu.readRegisterRegion(b, LSM_REG_OUTX_L_G, 12) != IMU_SUCCESS) {
    imuI2cFail++;
    return false;
  }

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
static void dropImuQueue() {
  while (imuQueueCount > 0) {
    imuDropped += imuPktQueueSamples[imuQueueTail];
    imuQueueTail = (imuQueueTail + 1) % IMU_PKT_QUEUE_LEN;
    imuQueueCount--;
  }
}

// Finalize the batch currently being filled and hand it to the TX queue.
// The sequence number is assigned here, so an overflow shows up as a gap.
static bool enqueueImuPacket() {
  if (imuCount == 0) return true;

  if (imuQueueCount >= IMU_PKT_QUEUE_LEN) {
    imuDropped += imuCount;
    imuQueueDrops++;
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
  memcpy(imuPktQueue[imuQueueHead], imuPkt, len);
  imuPktQueueLen[imuQueueHead] = (uint8_t)len;
  imuPktQueueSamples[imuQueueHead] = imuCount;
  imuQueueHead = (imuQueueHead + 1) % IMU_PKT_QUEUE_LEN;
  imuQueueCount++;

  imuSeq++;
  imuCount = 0;
  return true;
}

// Send queued packets; a busy BLE stack is retried next loop instead of
// throwing away the samples.
void flushImuQueue() {
  while (imuQueueCount > 0) {
    if (!BLE.connected()) {
      dropImuQueue();
      return;
    }

    uint8_t len = imuPktQueueLen[imuQueueTail];
    int written = imuChar.writeValue(imuPktQueue[imuQueueTail], len);
    if (written <= 0) {
      imuWriteFail++;
      return;
    }

    imuPktSent++;
    imuQueueTail = (imuQueueTail + 1) % IMU_PKT_QUEUE_LEN;
    imuQueueCount--;
  }
}

// Flush any in-progress batch, then drain the queue. Used by STOP/disconnect.
bool flushImuPacket() {
  bool ok = enqueueImuPacket();
  flushImuQueue();
  return ok;
}

// Drain the LSM6DS3 FIFO. Each 12-byte set is gyro X/Y/Z then accel X/Y/Z,
// sampled by the sensor itself at its own 416 Hz ODR. The main-loop rate no
// longer affects which samples are captured.
void drainImuFifo() {
  uint8_t status[2];
  if (imu.readRegisterRegion(status, LSM_REG_FIFO_STATUS1, 2) != IMU_SUCCESS) {
    fifoI2cFail++;
    return;
  }

  uint16_t words = (uint16_t)status[0] | (((uint16_t)(status[1] & 0x0F)) << 8);
  if (status[1] & LSM_FIFO_STATUS2_OVR)  fifoOverruns++;
  if (status[1] & LSM_FIFO_STATUS2_FULL) fifoFull++;

  uint16_t sets = words / LSM_FIFO_WORDS_PER_SET;
  if (sets == 0) return;

  while (sets > 0) {
    uint16_t batch = (sets > LSM_FIFO_READ_SETS) ? LSM_FIFO_READ_SETS : sets;
    uint8_t buf[LSM_FIFO_READ_SETS * LSM_FIFO_BYTES_PER_SET];
    if (imu.readRegisterRegion(buf, LSM_REG_FIFO_DATA_OUT_L,
                               (uint8_t)(batch * LSM_FIFO_BYTES_PER_SET)) != IMU_SUCCESS) {
      fifoI2cFail++;
      return;
    }

    for (uint16_t i = 0; i < batch; i++) {
      const uint8_t* p = buf + i * LSM_FIFO_BYTES_PER_SET;
      int16_t g[3], a[3];
      g[0] = (int16_t)(p[0]  | ((uint16_t)p[1]  << 8));
      g[1] = (int16_t)(p[2]  | ((uint16_t)p[3]  << 8));
      g[2] = (int16_t)(p[4]  | ((uint16_t)p[5]  << 8));
      a[0] = (int16_t)(p[6]  | ((uint16_t)p[7]  << 8));
      a[1] = (int16_t)(p[8]  | ((uint16_t)p[9]  << 8));
      a[2] = (int16_t)(p[10] | ((uint16_t)p[11] << 8));

      // The first set after leaving BYPASS can be partial/misaligned.
      if (fifoDiscardFirst) { fifoDiscardFirst = false; continue; }

      if (imuCount == 0) {
        // Exact 2404 us spacing from the START anchor, independent of when
        // the loop happens to drain the FIFO.
        uint32_t t = imuStartMs +
                     (uint32_t)(((uint64_t)imuSampleIndex * IMU_PERIOD_US) / 1000UL);
        memcpy(imuPkt + 4, &t, 4); // timestamp of sample #0 in this packet
      }

      appendImuSample(g, a);
      imuCount++;
      imuSampleIndex++;
      imuSampleTotal++;

      if (imuCount >= IMU_SAMPLES_PER_PKT) enqueueImuPacket();
    }
    sets -= batch;
  }
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
  audioSampleTotal++;
}

// If a packet cannot be delivered, roll the encoder back to the state that
// was announced in its header. The decoder never saw those samples, so the
// next packet must start from the decoder's last known state.
static void rollbackAdpcmPacket() {
  adpcmPredictor = adpcmBatchStartPred;
  adpcmStepIndex = adpcmBatchStartStep;
  adpcmPayloadLen = 0;
  adpcmSampleCount = 0;
  adpcmNibbleHigh = false;
  adpcmCurrentByte = 0;
}

bool flushAudioPacket(uint16_t sampleCount) {
  if (sampleCount == 0) return true;
  if (!BLE.connected()) {
    audioDropped += sampleCount;
    rollbackAdpcmPacket();
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

  // Encoder state at the first sample of this packet. The decoder uses these
  // values instead of resetting to zero, so audio stays continuous.
  int16_t pred = adpcmBatchStartPred;
  uint8_t step = adpcmBatchStartStep;
  memcpy(pkt + 11, &pred, 2);
  pkt[13] = step; pkt[14] = 0; pkt[15] = 0; pkt[16] = 0;
  memcpy(pkt + AUDIO_PKT_HDR, adpcmPayload, adpcmPayloadLen);

  int written = audioChar.writeValue(pkt, AUDIO_PKT_HDR + adpcmPayloadLen);
  if (written <= 0) {
    audioDropped += sampleCount;
    audioWriteFail++;
    rollbackAdpcmPacket();
    return false;
  }

  audioSeq++;
  audioPktSent++;
  adpcmPayloadLen = 0;
  adpcmSampleCount = 0;
  return true;
}

void encodeAudioBatch() {
  if (pcmTail == pcmHead) return;

  // On the very first batch, start the predictor on the first real PCM sample
  // so there is no initial ramp from zero either.
  if (audioSampleTotal == 0) {
    adpcmPredictor = pcmRing[pcmTail];
    adpcmStepIndex = 0;
  }

  adpcmBatchStartPred = adpcmPredictor;
  adpcmBatchStartStep = adpcmStepIndex;
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
    // 16 kHz hardware -> 8 kHz stream: keep every other sample.
    bool keep = (pdmDecimatePhase == 0);
    pdmDecimatePhase = (pdmDecimatePhase + 1) % PDM_DECIMATE;
    if (!keep) continue;

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
      resetStreams();   // clears the FIFO and re-anchors IMU timestamps
      recording = true;
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

    case CMD_PING: {
      // Small probe notifications confirm the client actually subscribed to
      // both stream characteristics (writeValue reports success even when
      // CCCD is not enabled, so only a received notification proves it).
      uint8_t probe[4] = { 0xFE, 0x00, 0x00, 0x00 };
      imuChar.writeValue(probe, sizeof(probe));
      audioChar.writeValue(probe, sizeof(probe));
      sendStatus("PONG");
      break;
    }
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
  pdmOk = PDM.begin(MIC_CHANNELS, PDM_HW_HZ);
  if (!pdmOk) {
    Serial.println("Failed to start PDM");
  }

  if (!BLE.begin()) {
    Serial.println("BLE init failed");
    while (1) { delay(1000); }
  }

  BLE.setLocalName(DEVICE_NAME);
  BLE.setDeviceName(DEVICE_NAME);
  BLE.setAdvertisedService(dataService);
  // Ask Android for a short connection interval (units are 1.25 ms).
  // This reduces the time a large notification can block the main loop.
  BLE.setConnectionInterval(6, 12);  // 7.5-15 ms

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
  static uint32_t lastStatMs = 0;
  lastAudioTry = millis();
  lastStatMs = millis();

  while (central.connected()) {
    handleCommand();

    if (recording && (uint32_t)(millis() - lastStatMs) >= 2000) {
      lastStatMs = millis();
      sendStatsStatus();
    }

    if (recording) {
      drainImuFifo();

      uint32_t ms = millis();
      if ((uint32_t)(ms - lastAudioTry) >= 10) { // check ~100 Hz
        lastAudioTry = ms;
        if (pcmAvailable() >= AUDIO_SAMPLES_PER_PKT) encodeAudioBatch();
      }
    }

    // Retry queued IMU packets even after STOP, so a busy BLE stack cannot
    // strand the final batches.
    flushImuQueue();

    delay(1); // yield to the BLE stack; the sensor FIFO buffers IMU samples
  }

  // Link dropped: flush what we have, then discard the rest safely.
  recording = false;
  flushImuPacket();
  if (adpcmSampleCount > 0) flushAudioPacket(adpcmSampleCount);
  Serial.println("Disconnected");
  BLE.advertise();
}
