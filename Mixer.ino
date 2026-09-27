#include <GyverOS.h>
#include <EncButton.h>
#include <LiquidCrystal_I2C.h>
#include <GyverHX711.h>
#include <GyverFilters.h>

//encoder
#define ENC_CLK 3
#define ENC_DT 2
#define ENC_SW 4
//button
#define BTN_PIN 5
//hx711
#define HX711_SCK 6
#define HX711_DT 7

//beeper
//#define BEEPER_PIN 8  // Beeper
// Valves
#define VALVE_A_PIN 9   // Resin A valve
#define VALVE_B_PIN 10  // Hardener B valve
#define AIR_A_PIN 11    // Air for A
#define AIR_B_PIN 12    // Air for B

//А - смола, В - отвердитель
GyverOS<1> os;
Button btn(BTN_PIN);
EncButton enc(ENC_CLK, ENC_DT, ENC_SW);
LiquidCrystal_I2C lcd(0x27, 16, 2);  // адрес, столбцов, строк
GyverHX711 scales(HX711_DT, HX711_SCK, HX_GAIN32_B);
GMedian3<long> medianFilterRaw;
GKalman kalmanFilterWRaw(2, 2, 0.01);

const uint16_t PROP_A = 100;
const uint16_t PROP_B = 33;
const uint16_t MASS_BASE_SHIFT = 25399;  //исходный сдвиг показаний АЦП к нулю пустого тензодатчика
const uint16_t MASS_FACTOR = 446;        //делитель массы для пересчета из АЦП в граммы
const uint16_t AIR_STOP_AG = 20;         //использование  воздуха смола грамм
const uint16_t AIR_STOP_BG = 40;         //использование воздуха отвердитель грамм
const uint16_t SmallCupMassG = 9;        //минимальный вес стакана
const uint16_t B_AIR_PRECUT_G = 6;       //отключение воздуха B заранее
const uint16_t B_DRIP_PRECUT_G = 0;      //отключение клапана B заранее
const uint16_t A_AIR_PRECUT_G = 6;      //отключение A заранее
const uint16_t A_DRIP_PRECUT_G = 0;      //отключение A заранее


uint16_t Mass_A = 150;       //дефолтные значения массы
uint16_t Mass_B = 50;        //дефолтные значения массы
uint16_t Mass_Total = 200;   //дефолтное знач. массы смеси
long massReadG = 0;          // актуальная масса в гр. с датчика без учета сдвигов
long massReadPrevG = 0;      //предыдущее значение массы до замера, гр
long massInitOffsetRaw = 0;  //сдвиг при инициализации от актуального заначения (условно 0 g)
long massContainerG = 0;     //вес кружки при каждом тарировании (условно 0 g)
bool DispMassShow = false;   //нужно ли печатать массу при каждом измерении
long massFilled = 0;         // снапшот массы в случае паузы или подъема стакана

//стейт-машина
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

State state = State::IDLE;

// ===================== Helpers =====================
static inline void setValveA(bool open) {
  digitalWrite(VALVE_A_PIN, open ? HIGH : LOW);
}
static inline void setValveB(bool open) {
  digitalWrite(VALVE_B_PIN, open ? HIGH : LOW);
}
static inline void setAirA(bool open) {
  digitalWrite(AIR_A_PIN, open ? HIGH : LOW);
}
static inline void setAirB(bool open) {
  digitalWrite(AIR_B_PIN, open ? HIGH : LOW);
}
static inline void allClosed() {
  setValveA(false);
  setValveB(false);
  setAirA(false);
  setAirB(false);
}

void setup() {
  Serial.begin(115200);
  Mass_B = (Mass_Total * PROP_B) / (PROP_B + PROP_A);
  Mass_A = Mass_Total - Mass_B;
  pinMode(VALVE_A_PIN, OUTPUT);
  pinMode(VALVE_B_PIN, OUTPUT);
  pinMode(AIR_A_PIN, OUTPUT);
  pinMode(AIR_B_PIN, OUTPUT);
  os.attach(0, scalesReadTask, 102);
  os.stop(0);
  delay(500);
  lcd.init();
  lcd.backlight();
  driftHandler();
  os.start(0);
}

void loop() {
  btn.tick();  // Опрос кнопки
  enc.tick();
  os.tick();

  switch (state) {
    case State::IDLE:
      idleHandler();
      break;

    case State::LIFTED:
      Serial.println("LIFTED state");
      liftedHandler(massFilled);
      break;

    case State::PAUSED:
      Serial.println("PAUSED state");
      pausedHandler(massFilled);
      break;

    case State::PREP_B:
      Serial.println("PREP_B_state");
      massContainerG = massReadG;
      while (!scales.available()) {}
      scalesReadTask();
      Serial.print("Container weight: ");
      Serial.print(massContainerG);
      Serial.print(". MassReadG: ");
      Serial.println(massReadG);

      if (Mass_B <= AIR_STOP_BG) {
        Serial.println("Mass_B <= AIR_STOP go to POUR_B_DRIP");
        state = State::POUR_B_DRIP;
      } else {
        Serial.println("Mass_B > AIR_STOP >> POUR_B_AIR");
        state = State::POUR_B_AIR;
      }
      break;

    case State::POUR_B_AIR:
      Serial.println("POUR_B_AIR state");
      while (!pouringHandler(Mass_B - B_AIR_PRECUT_G, false, true)) {
      }
      state = State::POUR_B_DRIP;
      Serial.println("POUR_B_AIR done");
      break;

    case State::POUR_B_DRIP:
      Serial.println("POUR_B_DRIP state");
      while (!pouringHandler(Mass_B - B_DRIP_PRECUT_G, false, false)) {
      }
      allClosed();
      Serial.println("POUR_B_DRIP done. Allclosed. Pause 1500 now");
      Serial.println("POUR_B_DRIP finished. State::PREP_A");
      state = State::PREP_A;
      break;

    case State::PREP_A:
      Serial.print("PREP_A_state. Container: ");
      if (Mass_A <= AIR_STOP_AG) {
        Serial.println("Mass_A <= AIR_STOP >> POUR_A_DRIP");
        state = State::POUR_A_DRIP;
      } else {
        Serial.println("Mass_A > AIR_STOP >> POUR_B_AIR");
        state = State::POUR_A_AIR;
      }
      break;
    case State::POUR_A_AIR:
      Serial.println("POUR_A_AIR state");
      while (!pouringHandler(Mass_Total - A_AIR_PRECUT_G, true, true)) {
      }
      state = State::POUR_A_DRIP;
      Serial.println("POUR_A_AIR done");
      break;

    case State::POUR_A_DRIP:
      Serial.println("POUR_A_DRIP state");
      while (!pouringHandler(Mass_Total - A_DRIP_PRECUT_G, true, false)) {
      }
      allClosed();
      Serial.println("POUR_A_DRIP done, click to proceed");
      lcd.setCursor(5, 0);
      lcd.print(" OK ");
      while (true) {
        btn.tick();  // Опрос кнопки
        os.tick();
        if (btn.click()) {
          Serial.println("CLICK::PouringA_done >> IDLE");
          state = State::IDLE;
          massContainerG = 0;
          break;
        }
      }
  }
}

bool pouringHandler(uint16_t massTarget, bool ATrue, bool AirTrue) {
  bool isPouringDone = false;
  Serial.print("PourHandler. TargetMass: ");
  Serial.print(massTarget);
  Serial.print(". Atrue: ");
  Serial.print(ATrue);
  Serial.print(". AirTrue: ");
  Serial.print(AirTrue);
  Serial.print(". massReadG: ");
  Serial.println(massReadG);
  setValveB(!ATrue);
  setAirB(!ATrue and AirTrue);
  setValveA(ATrue);
  setAirA(ATrue and AirTrue);
  lcdSign();
  while (true) {
    os.tick();
    btn.tick();
    if (btn.click()) {
      Serial.println("CLICK::Pouring >> Paused");
      allClosed();
      isPouringDone = false;
      state = State::PAUSED;
      break;
    }
    if (massReadG >= massTarget) {
      Serial.println("Pouring handler done");
      isPouringDone = true;
      break;
    }
  }
  return isPouringDone;
}


void idleHandler() {
  massContainerG = 0;
  allClosed();
  lcdSign();
  while (true) {
    btn.tick();
    os.tick();
    enc.tick();
    if (enc.turn()) {
      Mass_Total += 10 * enc.dir();
      Mass_Total = (Mass_Total > 700) ? 700 : ((Mass_Total < 30) ? 30 : Mass_Total);
      Mass_B = (Mass_Total * PROP_B) / (PROP_B + PROP_A);
      Mass_A = Mass_Total - Mass_B;
      lcdProp(Mass_A, Mass_B, Mass_Total);
    }
    if (btn.click()) {
      if (massReadG <= SmallCupMassG) {
        Serial.println("Place container!");
        break;
      } else if (massReadG > SmallCupMassG) {
        state = State::PREP_B;
        Serial.println("CLICK::Idle >> Pouring");
        break;
      }
    }
    if (btn.hold()) {
      Serial.println("HOLD >> Drift");
      driftHandler();
    }
  }
}

void pausedHandler(uint16_t massFilled) {
  allClosed();
  while (true) {
    os.tick();
    btn.tick();
    if (btn.click()) {
      Serial.println("CLICK:: Paused >> IDLE");
      state = State::IDLE;
      break;
    }
  }
}
void liftedHandler(uint16_t massFilled) {
  allClosed();
}

void scalesReadTask() {
  massReadPrevG = massReadG;
  if (scales.available()) {
    long MFRAW = medianFilterRaw.filtered(scales.read() - MASS_BASE_SHIFT - massInitOffsetRaw);  //Фильтруем АЦП грубо, приводим к нулю
    long kalman = kalmanFilterWRaw.filtered((long)MFRAW);                                        //чистая фильтрация по Калману
    massReadG = ((kalman / MASS_FACTOR) - massContainerG);                                       //приведение в граммы минус тара при наливании
  }
  if (DispMassShow) {
    lcdCurrentMass(massReadG);
  }
}

void driftHandler() {
  lcd.clear();
  lcd.home();
  lcd.print("Hold to proceed!");
  uint32_t now = millis();
  while (true) {
    btn.tick();
    if (millis() - now > 102) {
      now = millis();
      DispMassShow = false;
      if (scales.available()) {
        long MFRAW = medianFilterRaw.filtered(scales.read() - MASS_BASE_SHIFT);  //Фильтруем АЦП грубо, приводим к нулю
        massInitOffsetRaw = kalmanFilterWRaw.filtered((long)MFRAW);              //чистая фильтрация по Калману
      }
    }
    if (btn.hold()) {
      DispMassShow = true;
      break;
    }
  }
  lcd.clear();
  lcd.setCursor(3, 0);
  lcd.print("g");
  lcd.setCursor(3, 1);
  lcd.print("g");
  lcd.setCursor(12, 1);
  lcd.print(":");
  lcd.setCursor(9, 0);
  lcd.print(PROP_A);
  lcd.print(":");
  lcd.print(PROP_B);
  lcdProp(Mass_A, Mass_B, Mass_Total);
}

void lcdCurrentMass(long currentMass) {
  lcd.setCursor(0, 0);
  if (currentMass < 0 or currentMass > 999) {
    lcd.print("err!");
  } else if (currentMass < 10) {
    lcd.print("  ");
    lcd.print(currentMass);
    lcd.print("g");
  } else if (currentMass > 9 and currentMass < 100) {
    lcd.print(" ");
    lcd.print(currentMass);
    lcd.print("g");
  } else lcd.print(currentMass);
}

void lcdTime(uint8_t mins, uint8_t secs) {
  lcd.setCursor(6, 0);
  if (mins < 10) {
    lcd.print(" ");
    lcd.print(mins);
  } else lcd.print(mins);

  lcd.setCursor(6, 1);
  if (secs < 10) {
    lcd.print(" ");
    lcd.print(secs);
  } else lcd.print(secs);
}

void lcdProp(uint16_t PropA, uint16_t PropB, uint16_t Mass_Total) {
  lcd.setCursor(9, 1);
  if (PropA < 10) {
    lcd.print(" ");
    lcd.print(PropA);
  } else if (PropA > 9 and PropA < 100) {
    lcd.print(" ");
    lcd.print(PropA);
  } else lcd.print(PropA);

  lcd.setCursor(13, 1);
  if (PropB < 10) {
    lcd.print(PropB);
    lcd.print("  ");
  } else if (PropB > 9 and PropB < 100) {
    lcd.print(PropB);
    lcd.print(" ");
  } else lcd.print(PropB);

  lcd.setCursor(0, 1);
  if (Mass_Total < 10) {
    lcd.print("  ");
    lcd.print(Mass_Total);
  } else if (Mass_Total > 9 and Mass_Total < 100) {
    lcd.print(" ");
    lcd.print(Mass_Total);
  } else lcd.print(Mass_Total);
}

void lcdSign() {
  lcd.setCursor(5, 0);
  if (state == State::POUR_A_AIR) lcd.print(" >A ");
  if (state == State::POUR_B_AIR) lcd.print(" >B ");
  if (state == State::POUR_A_DRIP) lcd.print(" A> ");
  if (state == State::POUR_B_DRIP) lcd.print(" B> ");
  if (state == State::LIFTED) lcd.print(" !! ");
  if (state == State::PAUSED) lcd.print(" == ");
  if (state == State::IDLE) lcd.print("    ");
}
