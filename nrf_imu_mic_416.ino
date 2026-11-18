/*
  nrf_imu_mic_416.ino - v5 (corrected IMU byte order + visible FIFO-gap handling)

  Seeed Studio XIAO nRF52840 Sense (or compatible nRF52840 Sense).
  Streams timestamped IMU (LSM6DS3 @ 416 Hz) and PDM microphone (8000 Hz, IMA ADPCM)
  over BLE to a Web Bluetooth client (index.html).

  --------------------------------------------------------------------------
  WHAT CHANGED IN v5
  --------------------------------------------------------------------------
  1. CRITICAL DATA-CORRUCTION FIX: CTRL3_C was written as 0x06. On the
     LSM6DS3, CTRL3_C bits are:
         BOOT(7) BDU(6) H_LACTIVE(5) PP_OD(4) SIM(3) IF_INC(2) BLE(1) SW_RESET(0)
     0x06 sets IF_INC *and BLE*, but NOT BDU. BLE=1 puts the MSB at the lower
     address, while all sample decoding in this sketch (and the client) is
     little-endian. The result is byte-swapped accel/gyro values: a still
     sensor reads full-scale random-looking noise instead of ~1000 mg on the
     gravity axis. Verified against a 93 s静止 capture (imu_data (22).csv):
     every axis spanned +/-2000 mg / +/-5000 cdeg/s with no gravity vector.
     Correct value is 0x44 (BDU + IF_INC, BLE = 0). It is now read back and
     verified at startup, along with the ODR/FS registers.

  2. FIFO overruns are no longer silent. Continuous mode overwrites the oldest
     sets when full. Previously imuSampleIndex kept counting only emitted
     samples, so every timestamp after an overrun was too early and no packet
     gap was visible. Now an overrun:
       - flushes the FIFO (fifoClear) and re-anchors the time base,
       - discards and counts the buffered samples,
       - advances imuSeq by one (client sees a real sequence gap), and
       - sets bit 0 of IMU header byte 10 on the next packet (client logs a
         "time discontinuity" and can insert a gap instead of faking continuity).

  3. Removed the unconditional discard of the first FIFO set after START.
     fifoClear() already empties the FIFO, so that threw away one valid sample
     per recording. Misalignment is now handled by checking words % 6.

  4. I2C robustness: all IMU register writes/reads are checked and counted,
     status/data buffers are zero-initialised so a short transfer produces a
     detectable bad sample instead of stale bytes. The stray Wire.setClock()
     (which addressed the wrong bus on XIAO Sense) was removed.

  5. ADPCM partial-nibble write is bounds-checked.

  6. STAT adds tr= (time resets) and cf= (IMU config failures).

  Earlier history (v4):
  - LSM6DS3 internal FIFO at true 416 Hz; 24 samples/packet, 6x12-bit packing.
  - Audio 8 kHz IMA ADPCM, 400 samples/packet, predictor carried in header.
  - BLE write failures retried instead of dropping batches; losses counted.
  - START anchor + sample index gives exact 2404 us IMU spacing.
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
#define LSM_CTRL2_G_416_500DPS 0x64           // ODR_G=0110 (416 Hz), FS_G=01 (+/-500 dps)
// BDU (0x40) + IF_INC (0x04). BLE (0x02) must stay 0: the decoder is little-endian.
#define LSM_CTRL3_C_BDU_INC   0x44

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
// ODR_FIFO = max(g, xl ODRs) = 416 Hz (0x06 << 3 = 0x30); MODE=0x06 continuous.
#define LSM_FIFO_CTRL5_STREAM 0x36
#define LSM_FIFO_CTRL5_BYPASS 0x30
#define LSM_FIFO_WORDS_PER_SET 6
#define LSM_FIFO_BYTES_PER_SET 12
#define LSM_FIFO_STATUS2_EMPTY 0x10
#define LSM_FIFO_STATUS2_FULL  0x20
#define LSM_FIFO_STATUS2_OVR   0x40
#define LSM_FIFO_READ_SETS     4               // 4 sets = 48 bytes per I2C burst

// Microphone. nRF52840 mbed PDM supports 16000 or 41667 Hz.
// Sample at 16 kHz and keep every other sample -> 8000 Hz over BLE.
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

// IMU packet header byte 10 flags
#define IMU_FLAG_TIME_RESET   0x01            // FIFO overrun: time base was re-anchored

enum Cmd : uint8_t {
  CMD_SET_TIME = 0x01,   // payload: uint64_t unix epoch ms (LE)
  CMD_START    = 0x02,
  CMD_STOP     = 0x03,
  CMD_RESET    = 0x04,
  CMD_PING     = 0x05,   // device sends one probe notification on each stream
};

enum PktType : uint8_t {
  PKT_IMU_LEGACY = 0x01,
  PKT_AUDIO      = 0x02,
  PKT_IMU        = 0x03,
  PKT_STATUS     = 0x10,
};

// -------------------- BLE --------------------
BLEService dataService(SERVICE_UUID);
BLECharacteristic cmdChar(CMD_CHAR_UUID, BLEWrite | BLEWriteWithoutResponse, 20);
BLECharacteristic imuChar(IMU_CHAR_UUID, BLENotify, IMU_PKT_MAX_BYTES);
BLECharacteristic audioChar(AUDIO_CHAR_UUID, BLENotify, AUDIO_PKT_MAX_BYTES);
BLECharacteristic statusChar(STATUS_CHAR_UUID, BLERead | BLENotify, STATUS_CHAR_MAX_BYTES);

// All IMU register access goes through this object so the internal bus is used.
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
uint32_t imuSkipped = 0;     // FIFO words discarded to regain 6-word alignment
uint32_t imuStallLoss = 0;   // kept for STAT compatibility (always 0 with FIFO)
uint32_t imuQueueDrops = 0;  // full-batch drops when the IMU queue overflowed
uint32_t fifoOverruns = 0;   // sensor FIFO overrun events
uint32_t fifoFull = 0;       // sensor FIFO was found full
uint32_t fifoI2cFail = 0;    // FIFO status/data read failures
uint32_t imuTimeResets = 0;  // number of times the IMU time base was re-anchored
uint32_t imuCfgFail = 0;     // IMU register read/write failures (setup + runtime)
bool pdmOk = false;

// -------------------- IMU batching --------------------
uint8_t imuPkt[IMU_PKT_MAX_BYTES];       // batch currently being filled
uint8_t imuCount = 0;
uint32_t imuStartMs = 0;                 // device time at START (for FIFO sample timestamps)
uint32_t imuSampleIndex = 0;             // number of FIFO samples emitted since last anchor
uint8_t imuFlags = 0;                    // flags for the next finalized packet

// Completed packets wait here while BLE is busy.
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
uint32_t audioBatchStartMs = 0;   // device time of the first sample in this batch

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
  char buf[96];
  snprintf(buf, sizeof(buf), "STOPPED imu_drop=%lu aud_drop=%lu tr=%lu",
           (unsigned long)imuDropped, (unsigned long)audioDropped,
           (unsigned long)imuTimeResets);
  statusChar.writeValue((const void*)buf, strlen(buf));
}

void sendStatsStatus() {
  if (!BLE.connected()) return;
  char buf[STATUS_CHAR_MAX_BYTES];
  snprintf(buf, sizeof(buf),
           "STAT is=%lu as=%lu ip=%lu ap=%lu ie=%lu iw=%lu aw=%lu "
           "id=%lu ad=%lu skip=%lu stall=%lu iq=%lu fo=%lu fu=%lu ff=%lu "
           "pa=%u pdm=%d tr=%lu cf=%lu",
           (unsigned long)imuSampleTotal, (unsigned long)audioSampleTotal,
           (unsigned long)imuPktSent, (unsigned long)audioPktSent,
           (unsigned long)(imuI2cFail + fifoI2cFail), (unsigned long)imuWriteFail,
           (unsigned long)audioWriteFail, (unsigned long)imuDropped,
           (unsigned long)audioDropped, (unsigned long)imuSkipped,
           (unsigned long)imuStallLoss, (unsigned long)imuQueueDrops,
           (unsigned long)fifoOverruns, (unsigned long)fifoFull,
           (unsigned long)fifoI2cFail,
           (unsigned)pcmAvailable(), pdmOk ? 1 : 0,
           (unsigned long)imuTimeResets, (unsigned long)imuCfgFail);
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
  imuTimeResets = 0;
  imuCount = 0;
  imuFlags = 0;
  imuStartMs = nowMsSinceSync();
  imuSampleIndex = 0;
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
  audioBatchStartMs = 0;
}

// -------------------- Low-level I2C / LSM6DS3 setup --------------------
static status_t readReg8(uint8_t reg, uint8_t *v) {
  *v = 0;
  status_t st = imu.readRegister(v, reg);
  if (st != IMU_SUCCESS) imuCfgFail++;
  return st;
}

static status_t writeReg8(uint8_t reg, uint8_t value) {
  status_t st = imu.writeRegister(reg, value);
  if (st != IMU_SUCCESS) imuCfgFail++;
  return st;
}

void configureImu() {
  uint8_t who = 0;
  readReg8(LSM_REG_WHO_AM_I, &who);
  if (who != LSM_WHO_AM_I_VALUE) {
    Serial.print("LSM6DS3 WHO_AM_I mismatch: 0x"); Serial.println(who, HEX);
  }

  // BDU prevents half-updated samples; IF_INC enables the multi-byte burst.
  // BLE must stay 0 or every 16-bit sample is byte-swapped.
  writeReg8(LSM_REG_CTRL3_C, LSM_CTRL3_C_BDU_INC);
  writeReg8(LSM_REG_CTRL1_XL, LSM_CTRL1_XL_416_2G);
  writeReg8(LSM_REG_CTRL2_G,  LSM_CTRL2_G_416_500DPS);

  writeReg8(LSM_REG_FIFO_CTRL2, 0x00);
  writeReg8(LSM_REG_FIFO_CTRL3, LSM_FIFO_CTRL3_VALUE);
  writeReg8(LSM_REG_FIFO_CTRL4, LSM_FIFO_CTRL4_VALUE);
  writeReg8(LSM_REG_FIFO_CTRL5, LSM_FIFO_CTRL5_STREAM);

  // Read back and verify the critical settings. A silent write failure would
  // produce plausible-looking but wrong data (wrong ODR/FS/endianness).
  uint8_t c1 = 0, c2 = 0, c3 = 0, c5 = 0;
  readReg8(LSM_REG_CTRL1_XL, &c1);
  readReg8(LSM_REG_CTRL2_G, &c2);
  readReg8(LSM_REG_CTRL3_C, &c3);
  readReg8(LSM_REG_FIFO_CTRL5, &c5);
  bool ok = (c1 == LSM_CTRL1_XL_416_2G) &&
            (c2 == LSM_CTRL2_G_416_500DPS) &&
            ((c3 & 0x46) == LSM_CTRL3_C_BDU_INC) &&   // BDU + IF_INC set, BLE clear
            (c5 == LSM_FIFO_CTRL5_STREAM);
  if (!ok) {
    imuCfgFail++;
    Serial.print("IMU config verify FAILED: c1=0x"); Serial.print(c1, HEX);
    Serial.print(" c2=0x"); Serial.print(c2, HEX);
    Serial.print(" c3=0x"); Serial.print(c3, HEX);
    Serial.print(" c5=0x"); Serial.println(c5, HEX);
  } else {
    Serial.println("IMU 416 Hz / 2g / 500dps, FIFO stream, little-endian, BDU OK");
  }
}

// raw int16 -> signed 12-bit, with the scales described above.
static inline int16_t scale12(int16_t raw) {
  int32_t v = ((int32_t)raw * 2000L) / 32768L;
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
  imuPkt[10] = (uint8_t)imuFlags;  // bit 0: time base was re-anchored before this packet
  imuPkt[11] = 0;
  imuFlags = 0;

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

// Send queued packets; a busy BLE stack is retried next loop.
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

// A continuous-mode overrun overwrote the oldest sets. Make the loss visible
// and restart the time base instead of silently producing wrong timestamps.
static void handleFifoOverrun(uint16_t words) {
  fifoOverruns++;
  if (imuCount > 0) enqueueImuPacket();   // finalize batch carrying the old time base
  imu.fifoClear();                        // flush remaining (possibly wrapped) data
  imuDropped += (uint32_t)(words / LSM_FIFO_WORDS_PER_SET);
  imuStartMs = nowMsSinceSync();
  imuSampleIndex = 0;
  imuTimeResets++;
  imuSeq++;                               // client sees a real sequence gap
  imuFlags |= IMU_FLAG_TIME_RESET;        // client logs the discontinuity
}

// Drain the LSM6DS3 FIFO. Each 12-byte set is gyro X/Y/Z then accel X/Y/Z.
void drainImuFifo() {
  uint8_t status[2] = {0, 0};
  if (imu.readRegisterRegion(status, LSM_REG_FIFO_STATUS1, 2) != IMU_SUCCESS) {
    fifoI2cFail++;
    return;
  }

  uint16_t words = (uint16_t)status[0] | (((uint16_t)(status[1] & 0x0F)) << 8);
  if (status[1] & LSM_FIFO_STATUS2_OVR) {
    handleFifoOverrun(words);
    return;
  }
  if (status[1] & LSM_FIFO_STATUS2_FULL) fifoFull++;

  // Safety net: a partial set would rotate the axis pattern. Consume the
  // remainder so the next read starts on a 6-word boundary.
  uint16_t rem = words % LSM_FIFO_WORDS_PER_SET;
  if (rem != 0) {
    uint8_t junk[LSM_FIFO_BYTES_PER_SET] = {0};
    if (imu.readRegisterRegion(junk, LSM_REG_FIFO_DATA_OUT_L,
                               (uint8_t)(rem * 2)) != IMU_SUCCESS) {
      fifoI2cFail++;
    }
    imuSkipped += rem;
    words -= rem;
  }

  uint16_t sets = words / LSM_FIFO_WORDS_PER_SET;
  if (sets == 0) return;

  while (sets > 0) {
    uint16_t batch = (sets > LSM_FIFO_READ_SETS) ? LSM_FIFO_READ_SETS : sets;
    uint8_t buf[LSM_FIFO_READ_SETS * LSM_FIFO_BYTES_PER_SET] = {0};
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

      if (imuCount == 0) {
        // Exact 2404 us spacing from the current anchor.
        uint32_t t = imuStartMs +
                     (uint32_t)(((uint64_t)imuSampleIndex * IMU_PERIOD_US) / 1000UL);
        memcpy(imuPkt + 4, &t, 4);
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
// was announced in its header.
static void rollbackAdpcmPacket() {
  adpcmPredictor = adpcmBatchStartPred;
  adpcmStepIndex = adpcmBatchStartStep;
  adpcmPayloadLen = 0;
  adpcmSampleCount = 0;
  adpcmNibbleHigh = false;
  adpcmCurrentByte = 0;
  audioBatchStartMs = 0;
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
    if (adpcmPayloadLen < ADPCM_BYTES_PER_PKT) {
      adpcmPayload[adpcmPayloadLen++] = (uint8_t)(adpcmCurrentByte & 0x0F);
    }
    adpcmNibbleHigh = false;
  }

  uint8_t pkt[AUDIO_PKT_MAX_BYTES] = {0};
  pkt[0] = PKT_AUDIO;
  pkt[1] = (uint8_t)audioSeq;
  pkt[2] = (uint8_t)(audioSeq >> 8);
  // Use the batch start time, not the send time: a 400-sample batch is 50 ms
  // long at 8 kHz, so timestamping at writeValue() shifted all audio late.
  uint32_t t = (audioBatchStartMs != 0) ? audioBatchStartMs : nowMsSinceSync();
  memcpy(pkt + 3, &t, 4);
  uint16_t sr = MIC_HZ;
  memcpy(pkt + 7, &sr, 2);
  memcpy(pkt + 9, &sampleCount, 2);

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
  audioBatchStartMs = 0;
  return true;
}

void encodeAudioBatch() {
  if (pcmTail == pcmHead) return;

  // Approximate the time of the first sample in the ring buffer. The BLE
  // write happens up to 50 ms later; the client exports against this anchor.
  audioBatchStartMs = nowMsSinceSync();

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
  uint8_t buf[20] = {0};
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
      if (adpcmSampleCount > 0) {
        if (audioBatchStartMs == 0) audioBatchStartMs = nowMsSinceSync();
        flushAudioPacket(adpcmSampleCount);
      }
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
    imuCfgFail++;
  }
  configureImu();

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

    // Retry queued IMU packets even after STOP.
    flushImuQueue();

    delay(1); // yield to the BLE stack; the sensor FIFO buffers IMU samples
  }

  // Link dropped: flush what we have, then discard the rest safely.
  recording = false;
  flushImuPacket();
  if (adpcmSampleCount > 0) {
    if (audioBatchStartMs == 0) audioBatchStartMs = nowMsSinceSync();
    flushAudioPacket(adpcmSampleCount);
  }
  Serial.println("Disconnected");
  BLE.advertise();
}
