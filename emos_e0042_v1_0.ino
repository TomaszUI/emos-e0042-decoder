#include <Arduino.h>
#include <SPI.h>

// ======================================================
// ToM 433 Receiver v1.0
// ESP32-C3 SuperMini + E07-M1101D / CC1101
// EMOS E0042 reverse engineering
// ======================================================
//
// E07-M1101D -> ESP32-C3
//
// GDO2 -> GPIO1
// GDO0 -> GPIO3
// SCK  -> GPIO4
// MISO -> GPIO5
// MOSI -> GPIO6
// CS   -> GPIO10
//
// ======================================================

#define PIN_GDO2   1
#define PIN_GDO0   3
#define PIN_SCK    4
#define PIN_MISO   5
#define PIN_MOSI   6
#define PIN_CS    10

// ======================================================
// CC1101 registers
// ======================================================

#define IOCFG2    0x00
#define IOCFG0    0x02
#define PKTCTRL1  0x07
#define PKTCTRL0  0x08
#define FSCTRL1   0x0B
#define FSCTRL0   0x0C
#define FREQ2     0x0D
#define FREQ1     0x0E
#define FREQ0     0x0F
#define MDMCFG4   0x10
#define MDMCFG3   0x11
#define MDMCFG2   0x12
#define MDMCFG1   0x13
#define MDMCFG0   0x14
#define DEVIATN   0x15
#define MCSM0     0x18
#define FOCCFG    0x19
#define BSCFG     0x1A
#define AGCCTRL2  0x1B
#define AGCCTRL1  0x1C
#define AGCCTRL0  0x1D
#define FREND1    0x21
#define FREND0    0x22
#define FSCAL3    0x23
#define FSCAL2    0x24
#define FSCAL1    0x25
#define FSCAL0    0x26
#define TEST2     0x2C
#define TEST1     0x2D
#define TEST0     0x2E

#define PARTNUM   0x30
#define VERSION   0x31
#define RSSI_REG  0x34
#define MARCSTATE 0x35

// ======================================================
// CC1101 strobes
// ======================================================

#define SRES   0x30
#define SCAL   0x33
#define SRX    0x34
#define SIDLE  0x36

// ======================================================
// Settings
// ======================================================

#define MAX_PULSES   450
#define MAX_SYMBOLS  220

#define EXPECTED_BITS 46

// ======================================================
// Capture buffers
// ======================================================

volatile uint32_t pulseTimes[MAX_PULSES];
volatile uint8_t  pulseLevels[MAX_PULSES];

volatile uint16_t pulseCount = 0;

volatile bool frameActive = false;
volatile bool frameReady  = false;

volatile uint32_t lastEdgeUs   = 0;
volatile uint32_t frameStartUs = 0;
volatile uint32_t frameEndUs   = 0;

// ======================================================
// Previous normalized frame
// ======================================================

char previousFrame[EXPECTED_BITS + 1];
bool previousValid = false;

// ======================================================
// SPI helpers
// ======================================================

void ccSelect()
{
  digitalWrite(PIN_CS, LOW);

  uint32_t start = micros();

  while (digitalRead(PIN_MISO))
  {
    if (micros() - start > 1000)
      break;
  }
}

void ccDeselect()
{
  digitalWrite(PIN_CS, HIGH);
}

void ccWrite(uint8_t address, uint8_t value)
{
  ccSelect();

  SPI.transfer(address);
  SPI.transfer(value);

  ccDeselect();
}

uint8_t ccReadStatus(uint8_t address)
{
  ccSelect();

  SPI.transfer(address | 0xC0);
  uint8_t value = SPI.transfer(0);

  ccDeselect();

  return value;
}

uint8_t ccStrobe(uint8_t command)
{
  ccSelect();

  uint8_t status = SPI.transfer(command);

  ccDeselect();

  return status;
}

// ======================================================
// Reset CC1101
// ======================================================

void ccReset()
{
  digitalWrite(PIN_CS, HIGH);
  delay(1);

  digitalWrite(PIN_CS, LOW);
  delayMicroseconds(10);

  digitalWrite(PIN_CS, HIGH);
  delayMicroseconds(50);

  ccSelect();

  SPI.transfer(SRES);

  while (digitalRead(PIN_MISO))
  {
    delayMicroseconds(10);
  }

  ccDeselect();

  delay(5);
}

// ======================================================
// RSSI
// ======================================================

float readRSSI()
{
  uint8_t raw = ccReadStatus(RSSI_REG);

  int16_t value;

  if (raw >= 128)
    value = raw - 256;
  else
    value = raw;

  return (value / 2.0f) - 74.0f;
}

// ======================================================
// Configure CC1101
// ======================================================

void configureCC1101()
{
  ccStrobe(SIDLE);

  // GDO0 = asynchronous RX data
  ccWrite(IOCFG0, 0x0D);

  // GDO2 = Carrier Sense
  ccWrite(IOCFG2, 0x0E);

  // async serial mode
  ccWrite(PKTCTRL0, 0x32);
  ccWrite(PKTCTRL1, 0x04);

  // 433.920 MHz
  ccWrite(FREQ2, 0x10);
  ccWrite(FREQ1, 0xB0);
  ccWrite(FREQ0, 0x71);

  ccWrite(FSCTRL1, 0x06);
  ccWrite(FSCTRL0, 0x00);

  // OOK
  ccWrite(MDMCFG2, 0x30);

  // RX parameters
  ccWrite(MDMCFG4, 0x67);
  ccWrite(MDMCFG3, 0x83);

  ccWrite(MDMCFG1, 0x22);
  ccWrite(MDMCFG0, 0xF8);

  ccWrite(DEVIATN, 0x15);

  // AGC
  ccWrite(AGCCTRL2, 0x43);
  ccWrite(AGCCTRL1, 0x40);
  ccWrite(AGCCTRL0, 0x91);

  ccWrite(FOCCFG, 0x16);
  ccWrite(BSCFG, 0x6C);

  ccWrite(FREND1, 0x56);
  ccWrite(FREND0, 0x11);

  ccWrite(FSCAL3, 0xE9);
  ccWrite(FSCAL2, 0x2A);
  ccWrite(FSCAL1, 0x00);
  ccWrite(FSCAL0, 0x1F);

  ccWrite(TEST2, 0x81);
  ccWrite(TEST1, 0x35);
  ccWrite(TEST0, 0x09);

  ccWrite(MCSM0, 0x18);

  ccStrobe(SCAL);

  delay(5);

  ccStrobe(SRX);
}

// ======================================================
// Interrupt GDO0
// ======================================================

void IRAM_ATTR gdo0ISR()
{
  // ignore everything without Carrier Sense
  if (digitalRead(PIN_GDO2) == LOW)
    return;

  uint32_t now = micros();

  uint8_t newLevel =
    digitalRead(PIN_GDO0);

  if (!frameActive)
  {
    frameActive = true;
    frameReady  = false;

    pulseCount = 0;

    frameStartUs = now;
    lastEdgeUs   = now;

    return;
  }

  uint32_t width =
    now - lastEdgeUs;

  lastEdgeUs = now;

  // very short noise
  if (width < 120)
    return;

  // very long gap
  if (width > 15000)
    return;

  if (pulseCount < MAX_PULSES)
  {
    pulseTimes[pulseCount] = width;

    // width belongs to state before the edge
    pulseLevels[pulseCount] =
      !newLevel;

    pulseCount++;
  }
}

// ======================================================
// Quantize pulse
// ======================================================
//
// SHORT ~= 1955 us
// LONG  ~= 3910 us
//
// return:
// 1 = SHORT
// 2 = LONG
// 0 = invalid
//

uint8_t pulseType(uint32_t us)
{
  if (us >= 1450 && us <= 2600)
    return 1;

  if (us >= 3200 && us <= 4700)
    return 2;

  return 0;
}

// ======================================================
// Average SHORT
// ======================================================

float averageT(uint16_t count)
{
  uint64_t sum = 0;
  uint16_t n = 0;

  for (uint16_t i = 0; i < count; i++)
  {
    if (pulseType(pulseTimes[i]) == 1)
    {
      sum += pulseTimes[i];
      n++;
    }
  }

  if (!n)
    return 0;

  return (float)sum / n;
}

// ======================================================
// Build logical symbols
// ======================================================
//
// We already established:
//
// HIGH SHORT + LOW SHORT = 0
// HIGH SHORT + LOW LONG  = 1
//
// SS -> 0
// SL -> 1
//
// Other combinations -> ?
//
// ======================================================

uint16_t buildSymbols(
  char *symbols,
  uint16_t count
)
{
  uint16_t out = 0;

  for (uint16_t i = 0; i + 1 < count; i++)
  {
    // We want HIGH pulse followed by LOW pulse
    if (
      pulseLevels[i] == 1 &&
      pulseLevels[i + 1] == 0
    )
    {
      uint8_t hi =
        pulseType(pulseTimes[i]);

      uint8_t lo =
        pulseType(pulseTimes[i + 1]);

      char bit = '?';

      // SS
      if (hi == 1 && lo == 1)
        bit = '0';

      // SL
      else if (hi == 1 && lo == 2)
        bit = '1';

      if (out < MAX_SYMBOLS)
      {
        symbols[out++] = bit;
      }
    }
  }

  symbols[out] = '\0';

  return out;
}

// ======================================================
// Compare two symbol blocks
// ======================================================

float compareBlocks(
  const char *symbols,
  uint16_t count,
  uint16_t a,
  uint16_t b,
  uint16_t length
)
{
  if (
    a + length > count ||
    b + length > count
  )
    return 0;

  uint16_t same = 0;
  uint16_t valid = 0;

  for (uint16_t i = 0; i < length; i++)
  {
    char x = symbols[a + i];
    char y = symbols[b + i];

    if (x == '?' || y == '?')
      continue;

    valid++;

    if (x == y)
      same++;
  }

  if (!valid)
    return 0;

  return
    100.0f *
    same /
    valid;
}

// ======================================================
// Find symbol period
// ======================================================

uint16_t findSymbolPeriod(
  const char *symbols,
  uint16_t count,
  float &bestScore
)
{
  uint16_t bestPeriod = 0;

  bestScore = 0;

  // E0042 currently points strongly to 46
  // but scan neighborhood as safety.
  for (
    uint16_t period = 40;
    period <= 52;
    period++
  )
  {
    if (period * 2 > count)
      continue;

    uint16_t same = 0;
    uint16_t valid = 0;

    for (
      uint16_t i = 0;
      i + period < count;
      i++
    )
    {
      char a = symbols[i];
      char b = symbols[i + period];

      if (a == '?' || b == '?')
        continue;

      valid++;

      if (a == b)
        same++;
    }

    if (valid < 30)
      continue;

    float score =
      100.0f *
      same /
      valid;

    if (score > bestScore)
    {
      bestScore = score;
      bestPeriod = period;
    }
  }

  return bestPeriod;
}

// ======================================================
// Find best phase / start offset
// ======================================================
//
// We prefer an offset which gives three full copies
// and the best agreement.
//
// ======================================================

uint16_t findBestOffset(
  const char *symbols,
  uint16_t count,
  uint16_t period,
  float &scoreOut
)
{
  uint16_t bestOffset = 0;
  float best = 0;

  for (
    uint16_t offset = 0;
    offset < period;
    offset++
  )
  {
    if (
      offset + period * 2 >
      count
    )
      break;

    float score12 =
      compareBlocks(
        symbols,
        count,
        offset,
        offset + period,
        period
      );

    float score = score12;

    if (
      offset + period * 3 <=
      count
    )
    {
      float score23 =
        compareBlocks(
          symbols,
          count,
          offset + period,
          offset + period * 2,
          period
        );

      score =
        (score12 + score23) /
        2.0f;
    }

    if (score > best)
    {
      best = score;
      bestOffset = offset;
    }
  }

  scoreOut = best;

  return bestOffset;
}

// ======================================================
// Majority vote
// ======================================================

void buildMajorityFrame(
  const char *symbols,
  uint16_t count,
  uint16_t offset,
  uint16_t period,
  char *result
)
{
  for (
    uint16_t bit = 0;
    bit < period;
    bit++
  )
  {
    uint8_t zeros = 0;
    uint8_t ones  = 0;

    for (
      uint8_t copy = 0;
      copy < 3;
      copy++
    )
    {
      uint16_t p =
        offset +
        bit +
        copy * period;

      if (p >= count)
        continue;

      if (symbols[p] == '0')
        zeros++;

      if (symbols[p] == '1')
        ones++;
    }

    if (ones > zeros)
      result[bit] = '1';

    else if (zeros > ones)
      result[bit] = '0';

    else
      result[bit] = '?';
  }

  result[period] = '\0';
}

// ======================================================
// Rotate frame
// ======================================================

void rotateFrame(
  const char *source,
  char *dest,
  uint16_t len,
  uint16_t shift
)
{
  for (
    uint16_t i = 0;
    i < len;
    i++
  )
  {
    dest[i] =
      source[
        (i + shift) %
        len
      ];
  }

  dest[len] = '\0';
}

// ======================================================
// Compare lexical frames
// ======================================================

int compareFrameStrings(
  const char *a,
  const char *b,
  uint16_t len
)
{
  for (
    uint16_t i = 0;
    i < len;
    i++
  )
  {
    if (a[i] < b[i])
      return -1;

    if (a[i] > b[i])
      return 1;
  }

  return 0;
}

// ======================================================
// Canonical rotation
// ======================================================
//
// Since capture can begin in a different place in the
// cyclic transmission, choose the lexicographically
// smallest cyclic rotation.
//
// This lets consecutive frames be compared much more
// easily.
//
// ======================================================

void normalizeRotation(
  const char *source,
  char *normalized,
  uint16_t len,
  uint16_t &chosenShift
)
{
  char temp[EXPECTED_BITS + 1];

  rotateFrame(
    source,
    normalized,
    len,
    0
  );

  chosenShift = 0;

  for (
    uint16_t shift = 1;
    shift < len;
    shift++
  )
  {
    rotateFrame(
      source,
      temp,
      len,
      shift
    );

    if (
      compareFrameStrings(
        temp,
        normalized,
        len
      ) < 0
    )
    {
      strcpy(
        normalized,
        temp
      );

      chosenShift = shift;
    }
  }
}

// ======================================================
// Print bits
// ======================================================


// ======================================================
// Fixed header synchronization for EMOS E0042
// ======================================================
const char E0042_SYNC[] = "01111101010101010010";
#define E0042_SYNC_LEN 20

uint16_t syncDistanceAt(const char *src, uint16_t shift)
{
  uint16_t d = 0;
  for (uint16_t i = 0; i < E0042_SYNC_LEN; i++) {
    char b = src[(shift + i) % EXPECTED_BITS];
    if (b == '?' || b != E0042_SYNC[i]) d++;
  }
  return d;
}

bool alignToE0042Sync(const char *src, char *dst, uint16_t &shiftOut, uint16_t &distanceOut)
{
  uint16_t bestShift = 0;
  uint16_t bestDistance = E0042_SYNC_LEN + 1;
  for (uint16_t shift = 0; shift < EXPECTED_BITS; shift++) {
    uint16_t d = syncDistanceAt(src, shift);
    if (d < bestDistance) { bestDistance = d; bestShift = shift; }
  }
  rotateFrame(src, dst, EXPECTED_BITS, bestShift);
  shiftOut = bestShift;
  distanceOut = bestDistance;
  return bestDistance <= 1;
}

uint16_t extractReversed12(const char *frame, uint16_t start)
{
  uint16_t value = 0;
  for (uint8_t i = 0; i < 12; i++) if (frame[start + i] == '1') value |= (1U << i);
  return value;
}

void printField12(const char *frame, uint16_t start)
{
  for (uint8_t i = 0; i < 12; i++) Serial.print(frame[start + i]);
}
void printBits(
  const char *bits,
  uint16_t len
)
{
  for (
    uint16_t i = 0;
    i < len;
    i++
  )
  {
    Serial.print(bits[i]);

    if ((i + 1) % 8 == 0)
      Serial.print(' ');
  }

  Serial.println();
}

// ======================================================
// Print compact HEX-like data
// ======================================================

void printHex(
  const char *bits,
  uint16_t len
)
{
  Serial.print("HEX         : ");

  for (
    uint16_t i = 0;
    i + 7 < len;
    i += 8
  )
  {
    uint8_t value = 0;

    bool valid = true;

    for (
      uint8_t b = 0;
      b < 8;
      b++
    )
    {
      value <<= 1;

      if (bits[i + b] == '1')
        value |= 1;

      else if (
        bits[i + b] != '0'
      )
      {
        valid = false;
      }
    }

    if (!valid)
    {
      Serial.print("?? ");
      continue;
    }

    if (value < 0x10)
      Serial.print('0');

    Serial.print(
      value,
      HEX
    );

    Serial.print(' ');
  }

  Serial.println();

  // remaining bits
  uint16_t remainder =
    len % 8;

  if (remainder)
  {
    Serial.print(
      "Pozostale   : "
    );

    uint16_t start =
      len - remainder;

    for (
      uint16_t i = start;
      i < len;
      i++
    )
    {
      Serial.print(bits[i]);
    }

    Serial.println();
  }
}

// ======================================================
// Compare previous frame
// ======================================================

void comparePrevious(
  const char *frame
)
{
  if (!previousValid)
  {
    Serial.println();
    Serial.println(
      "Brak poprzedniej ramki do porownania."
    );

    strcpy(
      previousFrame,
      frame
    );

    previousValid = true;

    return;
  }

  uint16_t changed = 0;

  Serial.println();
  Serial.println(
    "ZMIENIONE BITY:"
  );

  for (
    uint16_t i = 0;
    i < EXPECTED_BITS;
    i++
  )
  {
    if (
      previousFrame[i] !=
      frame[i]
    )
    {
      changed++;

      Serial.print(i);
      Serial.print(' ');
    }
  }

  Serial.println();

  Serial.print(
    "Liczba zmian : "
  );

  Serial.println(changed);

  Serial.println();
  Serial.println(
    "POPZEDNIA:"
  );

  printBits(
    previousFrame,
    EXPECTED_BITS
  );

  Serial.println(
    "OBECNA:"
  );

  printBits(
    frame,
    EXPECTED_BITS
  );

  strcpy(
    previousFrame,
    frame
  );
}

// ======================================================
// Main decoder
// ======================================================

void decodeFrame()
{
  uint16_t count;

  noInterrupts();

  count = pulseCount;

  interrupts();

  if (count < 100)
    return;

  float rssi =
    readRSSI();

  // reject obvious noise
  if (rssi < -85.0f)
    return;

  // --------------------------------------------------
  // pulse quality
  // --------------------------------------------------

  uint16_t validPulse = 0;
  uint16_t badPulse   = 0;

  for (
    uint16_t i = 0;
    i < count;
    i++
  )
  {
    if (
      pulseType(
        pulseTimes[i]
      )
    )
      validPulse++;
    else
      badPulse++;
  }

  float quality =
    100.0f *
    validPulse /
    count;

  if (quality < 90.0f)
    return;

  // --------------------------------------------------
  // build SS/SL logical stream
  // --------------------------------------------------

  char symbols[
    MAX_SYMBOLS + 1
  ];

  uint16_t symbolCount =
    buildSymbols(
      symbols,
      count
    );

  if (symbolCount < 80)
    return;

  uint16_t unknownSymbols = 0;

  for (
    uint16_t i = 0;
    i < symbolCount;
    i++
  )
  {
    if (symbols[i] == '?')
      unknownSymbols++;
  }

  // --------------------------------------------------
  // find period
  // --------------------------------------------------

  float periodScore = 0;

  uint16_t period =
    findSymbolPeriod(
      symbols,
      symbolCount,
      periodScore
    );

  // We expect 46.
  // If automatic search gets confused but 46 works well,
  // prefer 46.
  if (
    symbolCount >=
    EXPECTED_BITS * 2
  )
  {
    float score46 =
      compareBlocks(
        symbols,
        symbolCount,
        0,
        EXPECTED_BITS,
        EXPECTED_BITS
      );

    if (
      score46 >= 95.0f
    )
    {
      period =
        EXPECTED_BITS;

      periodScore =
        score46;
    }
  }

  if (
    period != EXPECTED_BITS
  )
  {
    Serial.println();
    Serial.println(
      "UWAGA: wykryty okres != 46"
    );
  }

  // For further reverse engineering only use 46
  if (
    period != EXPECTED_BITS
  )
    return;

  // --------------------------------------------------
  // find best phase
  // --------------------------------------------------

  float offsetScore = 0;

  uint16_t offset =
    findBestOffset(
      symbols,
      symbolCount,
      period,
      offsetScore
    );

  // --------------------------------------------------
  // majority
  // --------------------------------------------------

  char majority[
    EXPECTED_BITS + 1
  ];

  buildMajorityFrame(
    symbols,
    symbolCount,
    offset,
    period,
    majority
  );

  // --------------------------------------------------
  // synchronize to the real E0042 fixed header
  // --------------------------------------------------

  char synced[EXPECTED_BITS + 1];
  uint16_t syncShift = 0;
  uint16_t syncDistance = 99;
  bool syncOK = alignToE0042Sync(majority, synced, syncShift, syncDistance);

  // ==================================================
  // PRODUCTION OUTPUT v1.0
  // ==================================================

  if (!syncOK) return;

  uint16_t rawA = extractReversed12(synced, 20);
  uint16_t rawB = extractReversed12(synced, 32);
  float tempC = ((float)rawA / 40.0f) - 40.0f;

  Serial.println();
  Serial.println("========================================");
  Serial.println(" ToM 433 RECEIVER v1.0");
  Serial.println(" EMOS E0042");
  Serial.println("========================================");
  Serial.print("Temperatura : "); Serial.print(tempC,1); Serial.println(" C");
  Serial.print("RSSI        : "); Serial.print(rssi,1); Serial.println(" dBm");
  Serial.print("Jakosc      : "); Serial.print(quality,1); Serial.println(" %");
  Serial.print("Raw         : "); Serial.println(rawA);
  Serial.print("Sync errors : "); Serial.println(syncDistance);
  Serial.print("Kopie       : "); Serial.print(offsetScore,1); Serial.println(" % zgodnosci");
  Serial.print("Signature   : ");
  for (uint8_t i=0;i<20;i++) Serial.print(synced[i]);
  Serial.println();

  Serial.print("JSON: {\"model\":\"EMOS E0042\",\"temperature_C\":");
  Serial.print(tempC,1);
  Serial.print(",\"rssi\":"); Serial.print(rssi,1);
  Serial.print(",\"quality\":"); Serial.print(quality,1);
  Serial.print(",\"raw\":"); Serial.print(rawA);
  Serial.print(",\"sync_errors\":"); Serial.print(syncDistance);
  Serial.print(",\"signature\":\"");
  for (uint8_t i=0;i<20;i++) Serial.print(synced[i]);
  Serial.println("\"}");
  Serial.println("========================================");
}
// ======================================================
// SETUP
// ======================================================

void setup()
{
  Serial.begin(115200);

  delay(1500);

  Serial.println();
  Serial.println(
    "========================================"
  );

  Serial.println(
    " ToM 433 Receiver v1.0"
  );

  Serial.println(
    " EMOS E0042 FRAME46"
  );

  Serial.println(
    "========================================"
  );

  pinMode(
    PIN_CS,
    OUTPUT
  );

  digitalWrite(
    PIN_CS,
    HIGH
  );

  pinMode(
    PIN_MISO,
    INPUT
  );

  pinMode(
    PIN_GDO0,
    INPUT
  );

  pinMode(
    PIN_GDO2,
    INPUT
  );

  SPI.begin(
    PIN_SCK,
    PIN_MISO,
    PIN_MOSI,
    PIN_CS
  );

  ccReset();

  uint8_t part =
    ccReadStatus(
      PARTNUM
    );

  uint8_t ver =
    ccReadStatus(
      VERSION
    );

  Serial.printf(
    "PARTNUM : 0x%02X\n",
    part
  );

  Serial.printf(
    "VERSION : 0x%02X\n",
    ver
  );

  if (
    part == 0x00 &&
    ver != 0x00 &&
    ver != 0xFF
  )
  {
    Serial.println(
      "CC1101 OK"
    );
  }
  else
  {
    Serial.println(
      "PROBLEM Z CC1101"
    );
  }

  configureCC1101();

  Serial.println();
  Serial.println(
    "433.920 MHz"
  );

  Serial.println(
    "OOK / ASK"
  );

  Serial.println(
    "SHORT ~= 1955 us"
  );

  Serial.println(
    "SS = 0"
  );

  Serial.println(
    "SL = 1"
  );

  Serial.println(
    "FRAME = 46 bit"
  );

  Serial.println();

  Serial.println(
    "Czekam na transmisje..."
  );

  Serial.println();

  attachInterrupt(
    digitalPinToInterrupt(
      PIN_GDO0
    ),
    gdo0ISR,
    CHANGE
  );
}

// ======================================================
// LOOP
// ======================================================

void loop()
{
  // --------------------------------------------------
  // finish frame after silence
  // --------------------------------------------------

  if (frameActive)
  {
    uint32_t last;

    noInterrupts();

    last =
      lastEdgeUs;

    interrupts();

    if (
      micros() - last >
      12000
    )
    {
      noInterrupts();

      frameEndUs =
        lastEdgeUs;

      frameActive =
        false;

      uint16_t countCopy =
        pulseCount;

      interrupts();

      if (
        countCopy >= 100
      )
      {
        frameReady =
          true;
      }
      else
      {
        pulseCount = 0;
      }
    }
  }

  // --------------------------------------------------
  // decode
  // --------------------------------------------------

  if (frameReady)
  {
    noInterrupts();

    frameReady =
      false;

    interrupts();

    decodeFrame();

    noInterrupts();

    pulseCount = 0;

    interrupts();
  }

  // --------------------------------------------------
  // status
  // --------------------------------------------------

  static uint32_t lastStatus = 0;

  if (
    millis() -
    lastStatus >=
    10000
  )
  {
    lastStatus =
      millis();

    Serial.printf(
      "[STATUS] RSSI=%.1f CS=%d ACTIVE=%d EDGES=%u\n",
      readRSSI(),
      digitalRead(PIN_GDO2),
      frameActive ? 1 : 0,
      pulseCount
    );
  }

  delay(1);
}




