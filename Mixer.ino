// Firmware: Epoxy dispenser A:B = 100:40 for Arduino Nano
// Hardware: HX711 (channel B, gain 32), LCD1602 I2C, encoder with button,
//           4 MOSFETs: A valve, B valve, AIR A, AIR B

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <GyverHX711.h>
#include <GyverFilters.h>
#include <EncButton.h>

// ===================== Pinout =====================
// Encoder (incremental) with push button
static const uint8_t ENCODER_PIN_A = 3;  // must support interrupts on Nano
static const uint8_t ENCODER_PIN_B = 2;  // must support interrupts on Nano
static const uint8_t ENCODER_PIN_BTN = 4;

// HX711
static const uint8_t HX_DT_PIN = A1;  // DOUT
static const uint8_t HX_SCK_PIN = A0; // SCK

// Valves
static const uint8_t VALVE_A_PIN = 12; // Resin A valve
static const uint8_t VALVE_B_PIN = 11; // Hardener B valve
static const uint8_t AIR_A_PIN = 9;    // Air for A
static const uint8_t AIR_B_PIN = 8;    // Air for B

// LCD I2C
static const uint8_t LCD_I2C_ADDR = 0x27; // change to 0x3F if needed

// ===================== Constants =====================
static const uint16_t PROP_A = 100;      // A: 100 parts
static const uint16_t PROP_B = 40;       // B: 40 parts
static const uint16_t MIX_MIN_G = 40;    // min total mix
static const uint16_t MIX_MAX_G = 350;   // max total mix
static const uint16_t MIX_DEFAULT_G = 300;

// Filtering and timing
static const uint8_t FILTER_K = 29;      // 0..31, higher = smoother
static const uint16_t FILTER_DT_MS = 100;
static const uint16_t MASS_READ_PERIOD_MS = 50;   // 20 Hz raw read
static const uint16_t LCD_UPDATE_PERIOD_MS = 250; // 4 Hz LCD refresh

// Pouring behavior
static const uint16_t AIR_PRECUT_G = 15;      // stop air 15 g early
static const uint16_t B_AIR_MIN_G = 20;       // if B target < 20, no air
static const uint16_t START_NEEDS_WEIGHT_G = 5; // require container
static const uint16_t LIFT_THRESHOLD_G = 5;   // detect lift when below
static const uint16_t RESUME_DELTA_G = 8;     // allowed difference to resume
static const uint32_t RESUME_WINDOW_MS = 60000UL; // 1 minute

// Scale calibration
// mass[g] = (filtered_raw - raw_zero) / SCALE_DIV
static const uint16_t SCALE_DIV = 146; // adjust for your sensor

// ===================== Globals =====================
LiquidCrystal_I2C lcd(LCD_I2C_ADDR, 16, 2);
EncButton eb(ENCODER_PIN_A, ENCODER_PIN_B, ENCODER_PIN_BTN);
GyverHX711 hx;
FastFilter filteredMass(FILTER_K, FILTER_DT_MS);

enum class State : uint8_t {
  IDLE,
  PREP_B,
  POUR_B_AIR,
  POUR_B_DRIP,
  PREP_A,
  POUR_A_AIR,
  POUR_A_DRIP,
  PAUSED,
  LIFTED
};

static State state = State::IDLE;

static uint16_t targetTotalG = MIX_DEFAULT_G;
static uint16_t targetAG = 0;
static uint16_t targetBG = 0;

static volatile int32_t lastRaw = 0;
static int32_t rawZero = 0;
static int32_t filteredRaw = 0;
static int32_t massG = 0; // current measured grams

// For pouring control
static int32_t massAtStartB = 0;
static int32_t massAtStartA = 0;
static int32_t bAirOffAt = 0;
static int32_t bDoneAt = 0;
static int32_t aAirOffAt = 0;
static int32_t aDoneAt = 0;

// Pause/lift handling
static int32_t massAtLift = 0;
static uint32_t liftStartMs = 0;

// Timers
static uint32_t lastReadMs = 0;
static uint32_t lastLcdMs = 0;

// ===================== Helpers =====================
static inline void setValveA(bool open) { digitalWrite(VALVE_A_PIN, open ? HIGH : LOW); }
static inline void setValveB(bool open) { digitalWrite(VALVE_B_PIN, open ? HIGH : LOW); }
static inline void setAirA(bool open)   { digitalWrite(AIR_A_PIN,   open ? HIGH : LOW); }
static inline void setAirB(bool open)   { digitalWrite(AIR_B_PIN,   open ? HIGH : LOW); }
static inline void allClosed() {
  setValveA(false); setValveB(false); setAirA(false); setAirB(false);
}

static inline uint16_t clamp16(uint16_t v, uint16_t lo, uint16_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static void computeABForTarget(uint16_t total) {
  // integer arithmetic with rounding
  targetBG = (uint32_t)total * PROP_B / (PROP_A + PROP_B);
  targetAG = total - targetBG; // ensure A+B == total
}

static void planBPhase() {
  massAtStartB = massG;
  const int32_t bTarget = targetBG;
  const bool useAir = (bTarget >= B_AIR_MIN_G);
  bAirOffAt = massAtStartB + (useAir ? max<int32_t>(0, bTarget - AIR_PRECUT_G) : -1);
  bDoneAt   = massAtStartB + bTarget;
}

static void planAPhase(uint16_t aTarget) {
  massAtStartA = massG;
  aAirOffAt = massAtStartA + max<int32_t>(0, (int32_t)aTarget - AIR_PRECUT_G);
  aDoneAt   = massAtStartA + aTarget;
}

static void updateDisplayIdle() {
  lcd.clear();
  // Line 1: target total at left, A and B at right
  char line1[17];
  snprintf(line1, sizeof(line1), "%3ug A%3u B%3u", targetTotalG, targetAG, targetBG);
  lcd.setCursor(0, 0);
  lcd.print(line1);

  // Line 2: current weight
  lcd.setCursor(0, 1);
  lcd.print("W:");
  lcd.print(massG);
  lcd.print("g      ");
}

static void updateDisplayFilling(const __FlashStringHelper* status) {
  lcd.clear();
  char line1[17];
  snprintf(line1, sizeof(line1), "%3ug A%3u B%3u", targetTotalG, targetAG, targetBG);
  lcd.setCursor(0, 0);
  lcd.print(line1);

  lcd.setCursor(0, 1);
  lcd.print("W:");
  lcd.print(massG);
  lcd.print("g ");
  if (status) {
    lcd.print(status);
  }
}

static void updateMassIfDue() {
  const uint32_t now = millis();
  if (now - lastReadMs < MASS_READ_PERIOD_MS) return;
  lastReadMs = now;

  if (hx.available()) {
    lastRaw = hx.read();
    filteredMass.setRaw(lastRaw);
    filteredMass.compute();
    filteredRaw = filteredMass.getFil();
    massG = (filteredRaw - rawZero) / (int32_t)SCALE_DIV;
  }
}

static void refreshLcdIfDue() {
  const uint32_t now = millis();
  if (now - lastLcdMs < LCD_UPDATE_PERIOD_MS) return;
  lastLcdMs = now;

  if (state == State::IDLE) {
    updateDisplayIdle();
  } else if (state == State::PAUSED) {
    updateDisplayFilling(F("PAUSE"));
  } else if (state == State::LIFTED) {
    updateDisplayFilling(F("LIFT"));
  } else {
    updateDisplayFilling(F("FILL"));
  }
}

static void startFilling() {
  allClosed();
  computeABForTarget(targetTotalG);
  planBPhase();
  // B: open valve; air optionally
  if (targetBG >= B_AIR_MIN_G) setAirB(true);
  setValveB(true);
  state = State::POUR_B_AIR; // even if air disabled, will switch quickly to DRIP
}

static void finishAndIdle() {
  allClosed();
  state = State::IDLE;
}

static void toPause() {
  allClosed();
  state = State::PAUSED;
}

static void abortToIdle() {
  finishAndIdle();
}

static bool isLiftedNow() {
  return massG <= (int32_t)LIFT_THRESHOLD_G;
}

static void handleLiftDetected() {
  allClosed();
  massAtLift = massG; // mass right after lift may be near 0, keep last good instead
  liftStartMs = millis();
  state = State::LIFTED;
}

static void handleEncoderIdle() {
  if (eb.turn()) {
    int8_t dir = eb.dir();
    uint16_t next = targetTotalG;
    if (dir > 0) {
      next = targetTotalG + 10;
    } else if (dir < 0) {
      next = (targetTotalG >= 10) ? (targetTotalG - 10) : 0;
    }
    targetTotalG = clamp16(next, MIX_MIN_G, MIX_MAX_G);
    computeABForTarget(targetTotalG);
    updateDisplayIdle();
  }

  if (eb.click()) {
    // start only if container present
    if (massG > (int32_t)START_NEEDS_WEIGHT_G) {
      startFilling();
      updateDisplayFilling(F("FILL"));
    }
  }
}

static void updateStateMachine() {
  switch (state) {
    case State::IDLE:
      break;

    case State::POUR_B_AIR: {
      // If air is not used (small B), immediately switch to DRIP logic
      const bool usingAir = (targetBG >= B_AIR_MIN_G);
      if (!usingAir) {
        state = State::POUR_B_DRIP;
        break;
      }
      if (massG >= bAirOffAt) {
        setAirB(false);
        state = State::POUR_B_DRIP;
      }
      if (massG >= bDoneAt) {
        // Safety: close B if we already reached target (possible if no air)
        setAirB(false);
        setValveB(false);
        // Calculate deviation and plan A
        const int32_t bActual = max<int32_t>(0, massG - massAtStartB);
        const int32_t dev = (int32_t)targetBG - bActual;
        uint16_t aTarget = targetAG;
        if (abs(dev) > 2) {
          // Recalculate A and total using actual B
          aTarget = (uint32_t)bActual * PROP_A / PROP_B;
          targetTotalG = aTarget + (uint16_t)bActual;
          computeABForTarget(targetTotalG); // keeps A+B==total for display
        }
        planAPhase(aTarget);
        // Start A with air
        setAirA(true);
        setValveA(true);
        state = State::POUR_A_AIR;
      }
    } break;

    case State::POUR_B_DRIP: {
      if (massG >= bDoneAt) {
        setValveB(false);
        setAirB(false);
        const int32_t bActual = max<int32_t>(0, massG - massAtStartB);
        const int32_t dev = (int32_t)targetBG - bActual;
        uint16_t aTarget = targetAG;
        if (abs(dev) > 2) {
          aTarget = (uint32_t)bActual * PROP_A / PROP_B;
          targetTotalG = aTarget + (uint16_t)bActual;
          computeABForTarget(targetTotalG);
        }
        planAPhase(aTarget);
        setAirA(true);
        setValveA(true);
        state = State::POUR_A_AIR;
      }
    } break;

    case State::POUR_A_AIR: {
      if (massG >= aAirOffAt) {
        setAirA(false);
        state = State::POUR_A_DRIP;
      }
      if (massG >= aDoneAt) {
        setAirA(false);
        setValveA(false);
        finishAndIdle();
      }
    } break;

    case State::POUR_A_DRIP: {
      if (massG >= aDoneAt) {
        setValveA(false);
        finishAndIdle();
      }
    } break;

    case State::PAUSED:
      // Stay until click (resume) or hold (abort)
      break;

    case State::LIFTED: {
      // Wait up to RESUME_WINDOW_MS for same container back, then require button
      const uint32_t now = millis();
      if (now - liftStartMs > RESUME_WINDOW_MS) {
        // Timeout, wait for long hold to abort
      }
    } break;
  }
}

static void handleButtonsDuringFill() {
  if (eb.click()) {
    if (state == State::PAUSED) {
      // Resume
      state = State::IDLE; // will be immediately advanced back into the right phase below
      // Re-open what is needed based on thresholds
      // Determine which phase should be active based on planned targets
      if (massG < bDoneAt && (massAtStartB != 0)) {
        // still on B
        if (targetBG >= B_AIR_MIN_G && massG < bAirOffAt) {
          setAirB(true);
          setValveB(true);
          state = State::POUR_B_AIR;
        } else {
          setValveB(true);
          state = State::POUR_B_DRIP;
        }
      } else if (massG < aDoneAt && (massAtStartA != 0)) {
        // in A phase
        if (massG < aAirOffAt) {
          setAirA(true);
          setValveA(true);
          state = State::POUR_A_AIR;
        } else {
          setValveA(true);
          state = State::POUR_A_DRIP;
        }
      } else {
        // nothing to resume
        state = State::IDLE;
      }
    } else if (state == State::LIFTED) {
      // Resume only if within 8 g from lift mass and weight present
      if (massG > (int32_t)START_NEEDS_WEIGHT_G && abs(massG - massAtLift) <= (int32_t)RESUME_DELTA_G) {
        // Resume similar to pause resume
        state = State::PAUSED; // route through paused to reuse logic
        handleButtonsDuringFill();
      }
    } else {
      // Pause
      toPause();
    }
  }

  if (eb.hold()) {
    // Abort any time during fill, or when lifted/paused
    abortToIdle();
  }
}

void setup() {
  pinMode(VALVE_A_PIN, OUTPUT);
  pinMode(VALVE_B_PIN, OUTPUT);
  pinMode(AIR_A_PIN, OUTPUT);
  pinMode(AIR_B_PIN, OUTPUT);
  allClosed();

  lcd.init();
  lcd.backlight();

  // HX711 init on channel B, gain 32
  hx.begin(HX_DT_PIN, HX_SCK_PIN, HX_GAIN32_B);

  // Initial readings for zeroing
  delay(200);
  int32_t sum = 0;
  const uint8_t samples = 10;
  for (uint8_t i = 0; i < samples; i++) {
    while (!hx.available()) {
      delay(2);
    }
    int32_t r = hx.read();
    sum += r;
  }
  rawZero = sum / samples;
  filteredMass.setRaw(rawZero);

  computeABForTarget(targetTotalG);
  updateDisplayIdle();
}

void loop() {
  eb.tick();

  updateMassIfDue();

  // Handle emergency lift detection during active pour (not in IDLE)
  if (state != State::IDLE && state != State::PAUSED && state != State::LIFTED) {
    if (isLiftedNow()) {
      handleLiftDetected();
    }
  }

  if (state == State::IDLE) {
    handleEncoderIdle();
  } else {
    handleButtonsDuringFill();
    updateStateMachine();
  }

  refreshLcdIfDue();
}