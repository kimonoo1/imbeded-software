/*
 *  스마트체어 (Smart Chair) - ESP32 자동 정렬 의자
 *  LG 임베디드 소프트웨어 경진대회 자율부문
 * ---------------------------------------------------------------------
 *  HW : ESP32 30pin / BTS7960 x3 / 5840-31ZY x2 / 리니어액추에이터 x1
 *       HC-SR04 x3 (전방,좌,우) / FSR (착석감지) / 3S 18650 + BMS
 *
 *  동작 흐름 (state machine)
 *    DEEP SLEEP --(FSR 착석)--> OCCUPIED --(기립)--> SETTLE(수 초 대기)
 *      -> LIFT(액추에이터로 한쪽 다리 들어 자세 정렬)
 *      -> SCAN(제자리 회전하며 전방 초음파로 책상 방향 탐색)
 *      -> TURN(최적 방향으로 복귀 회전)
 *      -> DRIVE(전진, 거리 or 전류 스톨로 정지)
 *      -> RETRACT(액추에이터 원위치) 
 *      -> DEEP SLEEP
 
 * ---------------------------------------------------------------------
 *  [필수 저항 — 총 4개]
 *
 *  1) FSR 풀다운  10kΩ x 1   
 *
 *        3.3V ─── FSR ───┬─── GPIO34
 *                        │
 *                      10kΩ
 *                        │
 *                       GND
 *
 *     단 "3.3V - FSR - GPIO34 - 저항 - GND" 순서는 지켜야 합니다.
 *
 *  2) HC-SR04 ECHO 직렬저항  1kΩ x 3   (GPIO 2 / 27 / 33 각각)
 *
 *        ECHO ───1kΩ─── GPIO
 *    직렬 1kΩ 이 ESP32 내부 클램프 다이오드로 흐르는 전류를 1.4mA 로 제한  
 */

#include <Arduino.h>
#include "esp_sleep.h"
#include "driver/rtc_io.h"


/* [1] PIN MAP   (2026-08-24 실제 배선 반영) */

// ---- BTS7960 #1 : 모터1 (좌측 주행) 
#define PIN_ML_RPWM      18
#define PIN_ML_LPWM      23   
#define PIN_ML_IS        -1   

// ---- BTS7960 #2 : 모터2 (우측 주행) 
#define PIN_MR_RPWM      16   
#define PIN_MR_LPWM      17   
#define PIN_MR_IS        -1

// ---- BTS7960 #3 : 리니어 액추에이터 
#define PIN_ACT_RPWM     25
#define PIN_ACT_LPWM     26
#define PIN_ACT_IS       -1

// ---- R_EN / L_EN : 5V 에 직결(상시 ON) 
#define PIN_EN_SHARED    -1

// ---- HC-SR04 x3 : TRIG 개별 
#define PIN_TRIG_FRONT    4
#define PIN_ECHO_FRONT   35
#define PIN_TRIG_LEFT    14
#define PIN_ECHO_LEFT    27
#define PIN_TRIG_RIGHT   32
#define PIN_ECHO_RIGHT   33

// ---- FSR (착석 감지) 
#define PIN_FSR          34
#define FSR_WAKE_GPIO    GPIO_NUM_34
#define PIN_LED          -1



/* [2] 튜닝 파라미터 */

#define FSR_INVERTED        false
#define FSR_SIT_ADC          1200   // 이 값 이상이면 착석
#define FSR_STAND_ADC         600   // 이 값 이하이면 기립 
#define FSR_DEBOUNCE_MS       800   // 이 시간 이상 유지되어야 상태 전환 인정

// --- 기립 후 대기 ---
#define SETTLE_DELAY_MS      5000   // 사람이 일어난 뒤 몇 초 후 동작 시작
#define REOCCUPY_ABORT       true   // 동작 중 다시 앉으면 즉시 중단

// --- 액추에이터 ---
#define ACT_PWM               220   // 액추에이터 구동 듀티 
#define ACT_EXTEND_MS        2500   // 뻗는 시간 
#define ACT_RETRACT_MS       2800   // 접는 시간 
#define ACT_SETTLE_MS         800   // 들어올린 뒤 흔들림 가라앉는 시간

// --- 모터 방향 반전

#define INVERT_M1           false
#define INVERT_M2            true   // 모터2 진행방향 반전
#define INVERT_ACT          false

// --- 주행 모터 ---
#define DRIVE_PWM             180   // 직진 듀티
#define TURN_PWM              150   // 제자리 회전 듀티
#define PWM_RAMP_STEP           8   // 소프트스타트 : 20ms 마다 증가폭
#define PWM_RAMP_INTERVAL_MS   20

// --- 초음파 ---
#define SONAR_MAX_CM          250.0f
#define SONAR_TIMEOUT_US     15000UL   // ≈ 250cm
#define SONAR_SAMPLES            5     // 중앙값 필터 샘플 수
#define SONAR_INTERVAL_MS       60     // 센서 간 간섭 방지 간격

// --- 목적지 판정 ---
#define DESK_STOP_CM          12.0f   // 전방 이 거리 이하 → 도착
#define DESK_VALID_MIN_CM     15.0f   // 스캔 시 이 범위 안에 있는 물체만 '책상' 후보
#define DESK_VALID_MAX_CM    200.0f

// --- 스캔(제자리 회전 탐색) ---
#define SCAN_STEP_MS           180    // 한 스텝당 회전 시간
#define SCAN_STEPS              20    // 총 스텝 수 (한 바퀴 조금 넘게 되도록 실측 조정)
#define SCAN_PAUSE_MS          120    // 스텝 후 측정 전 정지 시간

// --- 전류 스톨 감지 
#define STALL_ENABLE         false
#define STALL_BLANK_MS         700    // 기동 돌입전류 무시 구간
#define STALL_MARGIN_ADC       450    // 무부하 평균 대비 이만큼 초과하면 과부하
#define STALL_HOLD_MS          350    // 과부하가 이 시간 이상 지속되어야 스톨 인정
#define STALL_ABS_MAX_ADC     3000    // 무조건 비상정지 임계값

// --- 안전 타임아웃 
#define TIMEOUT_DRIVE_MS     12000
#define TIMEOUT_TURN_MS       8000
#define TIMEOUT_SCAN_MS      20000
#define TIMEOUT_GLOBAL_MS    60000

// --- 디버그 / 벤치 테스트 ---------------------------------------------
//    DEBUG_MODE 가 true 이면 deep sleep x
#define DEBUG_MODE          true
#define BOOT_HOLD_MS        10000   // 부팅 후 이 시간 동안은 무조건 깨어 있음
#define HEARTBEAT_MS         2000   // 살아있음 표시 출력 주기
#define BOOT_BANNER_COUNT       6   // 부팅 배너 반복 횟수 (모니터 늦게 열어도 보이게)

// --- 저전압 보호 (분압저항 달았을 때만 사용, 안 쓰면 -1) ---
#define PIN_VBAT               -1
#define VBAT_DIVIDER_RATIO   5.7f     // (R1+R2)/R2
#define VBAT_CUTOFF_V        9.6f     // 3S 기준 셀당 3.2V



/* [3] 타입 정의  (struct / enum) */

// --- 모터 1채널(BTS7960 1개) 상태 ---
struct Motor {
  uint8_t  rpwm, lpwm;
  int8_t   isPin;         // -1 이면 전류감지 미사용
  uint8_t  chR,  chL;     // LEDC 채널 
  bool     invert;        // true 면 정/역방향을 서로 뒤집음
  int16_t  target;        // -255 ~ +255 (양수 = 정방향)
  int16_t  current;       // 램프 적용된 현재 출력
  uint32_t lastRampMs;
  uint16_t isBaseline;    // 무부하 기준 ADC
  uint16_t isFiltered;    // 저역통과된 IS 값
  uint32_t startedMs;     // 마지막으로 0 → non-zero 로 바뀐 시각
  uint32_t overloadSince; // 과부하 시작 시각 (0 = 정상)
};

// --- 초음파 센서 식별자 ---
enum SonarId { SONAR_FRONT = 0, SONAR_LEFT = 1, SONAR_RIGHT = 2 };

// --- 상태 머신 ---
enum State {
  ST_BOOT,
  ST_OCCUPIED,     // 사람이 앉아 있음 — 기립 대기
  ST_SETTLE,       // 기립 확인 후 안전 대기
  ST_LIFT,         // 액추에이터로 들어올려 자세 정렬
  ST_SCAN,         // 제자리 회전하며 책상 탐색
  ST_TURN,         // 최적 방향으로 복귀 회전
  ST_DRIVE,        // 책상 앞으로 전진
  ST_RETRACT,      // 액추에이터 원위치
  ST_SLEEP_PREP    // deep sleep 진입 준비
};

/* [4] LEDC PWM 래퍼  */

#define PWM_FREQ   1000     // BTS7960 은 1~20kHz. 1kHz 가 무난, 소음 신경쓰면 16000
#define PWM_RES       8     // 8bit → duty 0~255

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  static inline void pwmAttach(uint8_t pin, uint8_t ch) {
    (void)ch;
    ledcAttach(pin, PWM_FREQ, PWM_RES);
  }
  static inline void pwmWrite(uint8_t pin, uint8_t ch, uint32_t duty) {
    (void)ch;
    ledcWrite(pin, duty);
  }
#else
  static inline void pwmAttach(uint8_t pin, uint8_t ch) {
    ledcSetup(ch, PWM_FREQ, PWM_RES);
    ledcAttachPin(pin, ch);
  }
  static inline void pwmWrite(uint8_t pin, uint8_t ch, uint32_t duty) {
    (void)pin;
    ledcWrite(ch, duty);
  }
#endif

/* [5] 모터 드라이버 추상화 */

Motor motL   = {PIN_ML_RPWM,  PIN_ML_LPWM,  PIN_ML_IS,  0, 1, INVERT_M1,  0, 0, 0, 0, 0, 0, 0};
Motor motR   = {PIN_MR_RPWM,  PIN_MR_LPWM,  PIN_MR_IS,  2, 3, INVERT_M2,  0, 0, 0, 0, 0, 0, 0};
Motor motAct = {PIN_ACT_RPWM, PIN_ACT_LPWM, PIN_ACT_IS, 4, 5, INVERT_ACT, 0, 0, 0, 0, 0, 0, 0};

void motorInit(Motor &m) {
  pwmAttach(m.rpwm, m.chR);
  pwmAttach(m.lpwm, m.chL);
  pwmWrite(m.rpwm, m.chR, 0);
  pwmWrite(m.lpwm, m.chL, 0);
  if (m.isPin >= 0) pinMode(m.isPin, INPUT);
  m.target = m.current = 0;
  m.lastRampMs = millis();
  m.isBaseline = 0;
  m.isFiltered = 0;
  m.startedMs = 0;
  m.overloadSince = 0;
}

// 목표 출력 지정 
void motorSet(Motor &m, int16_t duty) {
  duty = constrain(duty, -255, 255);
  if (m.target == 0 && duty != 0) {
    m.startedMs = millis();
    m.overloadSince = 0;
  }
  m.target = duty;
}

void motorStop(Motor &m) {
  m.target = 0;
  m.current = 0;
  pwmWrite(m.rpwm, m.chR, 0);
  pwmWrite(m.lpwm, m.chL, 0);
  m.overloadSince = 0;
}

// 소프트스타트/스톱 램프 + 실제 PWM 출력. loop() 마다 호출.
void motorService(Motor &m) {
  uint32_t now = millis();
  if (now - m.lastRampMs >= PWM_RAMP_INTERVAL_MS) {
    m.lastRampMs = now;
    if (m.current < m.target) {
      m.current += PWM_RAMP_STEP;
      if (m.current > m.target) m.current = m.target;
    } else if (m.current > m.target) {
      m.current -= PWM_RAMP_STEP;
      if (m.current < m.target) m.current = m.target;
    }
  }
  int16_t v = m.invert ? -m.current : m.current;   // 방향 반전 적용
  if (v > 0) {
    pwmWrite(m.lpwm, m.chL, 0);
    pwmWrite(m.rpwm, m.chR, v);
  } else if (v < 0) {
    pwmWrite(m.rpwm, m.chR, 0);
    pwmWrite(m.lpwm, m.chL, -v);
  } else {
    pwmWrite(m.rpwm, m.chR, 0);
    pwmWrite(m.lpwm, m.chL, 0);
  }
}

void driversEnable(bool on) {
  if (PIN_EN_SHARED >= 0) digitalWrite(PIN_EN_SHARED, on ? HIGH : LOW);
}

/* [6] 전류(IS) 감지 — 도착/스톨 판정 */

void currentService(Motor &m) {
  if (m.isPin < 0) return;
  int raw = analogRead(m.isPin);
  // 1차 IIR 저역통과 (α ≈ 1/8)
  m.isFiltered = (m.isFiltered * 7 + raw) / 8;

  if (m.current == 0) { m.overloadSince = 0; return; }
  if (millis() - m.startedMs < STALL_BLANK_MS) { m.overloadSince = 0; return; }

  bool over = (m.isFiltered > m.isBaseline + STALL_MARGIN_ADC) ||
              (m.isFiltered > STALL_ABS_MAX_ADC);
  if (over) {
    if (m.overloadSince == 0) m.overloadSince = millis();
  } else {
    m.overloadSince = 0;
  }
}

bool motorStalled(Motor &m) {
  if (!STALL_ENABLE || m.isPin < 0) return false;
  if (m.isFiltered > STALL_ABS_MAX_ADC) return true;
  return (m.overloadSince != 0) && (millis() - m.overloadSince >= STALL_HOLD_MS);
}

// 주행 시작 직후 무부하 구간에서 기준값 학습
void learnBaseline(Motor &m) {
  if (m.isPin < 0) { m.isBaseline = 0; m.isFiltered = 0; return; }
  uint32_t sum = 0;
  for (int i = 0; i < 16; i++) { sum += analogRead(m.isPin); delay(5); }
  m.isBaseline = sum / 16;
  m.isFiltered = m.isBaseline;
}

/* [7] 초음파 (HC-SR04 x3, TRIG 공용) */


const uint8_t TRIG_PINS[3] = { PIN_TRIG_FRONT, PIN_TRIG_LEFT, PIN_TRIG_RIGHT };
const uint8_t ECHO_PINS[3] = { PIN_ECHO_FRONT, PIN_ECHO_LEFT, PIN_ECHO_RIGHT };

float sonarPingOnce(SonarId id) {
  uint8_t trig = TRIG_PINS[id];
  uint8_t echo = ECHO_PINS[id];
  digitalWrite(trig, LOW);
  delayMicroseconds(4);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  unsigned long dur = pulseIn(echo, HIGH, SONAR_TIMEOUT_US);
  if (dur == 0) return SONAR_MAX_CM;          // 타임아웃 = 아무것도 없음
  float cm = dur * 0.0343f / 2.0f;
  if (cm > SONAR_MAX_CM || cm < 2.0f) return SONAR_MAX_CM;
  return cm;
}

// 중앙값 필터 
float sonarRead(SonarId id) {
  float v[SONAR_SAMPLES];
  for (int i = 0; i < SONAR_SAMPLES; i++) {
    v[i] = sonarPingOnce(id);
    delay(SONAR_INTERVAL_MS);
  }
  for (int i = 1; i < SONAR_SAMPLES; i++) {          // 삽입정렬
    float k = v[i]; int j = i - 1;
    while (j >= 0 && v[j] > k) { v[j + 1] = v[j]; j--; }
    v[j + 1] = k;
  }
  return v[SONAR_SAMPLES / 2];
}

/* [8] FSR 착석 감지 */

int  fsrRaw() {
  int v = analogRead(PIN_FSR);
  return FSR_INVERTED ? (4095 - v) : v;
}
bool fsrSeatedRaw(){ return fsrRaw() >= FSR_SIT_ADC; }
bool fsrEmptyRaw() { return fsrRaw() <= FSR_STAND_ADC; }

// 디바운스된 착석 상태
bool     seated        = false;
uint32_t seatChangeMs  = 0;
bool     pendingState  = false;

void fsrService() {
  bool inst = seated ? !fsrEmptyRaw() : fsrSeatedRaw();   // 히스테리시스
  if (inst != seated) {
    if (pendingState != inst) { pendingState = inst; seatChangeMs = millis(); }
    else if (millis() - seatChangeMs >= FSR_DEBOUNCE_MS) {
      seated = inst;
      Serial.printf("[FSR] %s (adc=%d)\n", seated ? "착석" : "기립", fsrRaw());
    }
  } else {
    pendingState = seated;
  }
}

/* [9] 상태 머신 */

State    state       = ST_BOOT;
uint32_t stateEnterMs = 0;
uint32_t missionStartMs = 0;

// SCAN 결과
int      scanBestStep   = -1;
float    scanBestDist   = SONAR_MAX_CM;
int      scanStepIdx    = 0;
bool     scanTurnRight  = true;     // 좌/우 초음파 비교로 결정된 회전 방향
bool     scanInit       = false;    // 회전방향 결정을 마쳤는지
bool     scanStepMoving = false;
uint32_t scanStepStart  = 0;

// DRIVE 상태 변수
float    driveFrontDist = SONAR_MAX_CM;
uint32_t drivePingMs    = 0;

const char* stateName(State s) {
  switch (s) {
    case ST_BOOT:       return "BOOT";
    case ST_OCCUPIED:   return "OCCUPIED";
    case ST_SETTLE:     return "SETTLE";
    case ST_LIFT:       return "LIFT";
    case ST_SCAN:       return "SCAN";
    case ST_TURN:       return "TURN";
    case ST_DRIVE:      return "DRIVE";
    case ST_RETRACT:    return "RETRACT";
    case ST_SLEEP_PREP: return "SLEEP_PREP";
  }
  return "?";
}

void gotoState(State s) {
  Serial.printf("[STATE] %s -> %s\n", stateName(state), stateName(s));

  // 상태 진입 시 해당 상태의 지역 변수 초기화 (2회차 동작에서 값이 남는 것 방지)
  if (s == ST_SCAN) {
    scanInit       = false;
    scanStepMoving = false;
    scanStepStart  = 0;
    scanStepIdx    = 0;
    scanBestStep   = -1;
    scanBestDist   = SONAR_MAX_CM;
  }
  if (s == ST_DRIVE) {
    driveFrontDist = SONAR_MAX_CM;
    drivePingMs    = 0;
  }

  state = s;
  stateEnterMs = millis();
}

uint32_t inState() { return millis() - stateEnterMs; }

void allStop() {
  motorStop(motL);
  motorStop(motR);
  motorStop(motAct);
}

// 제자리 회전 : right=true 면 시계방향
void spin(bool right, int16_t pwm) {
  motorSet(motL, right ?  pwm : -pwm);
  motorSet(motR, right ? -pwm :  pwm);
}

void driveForward(int16_t pwm) {
  motorSet(motL, pwm);
  motorSet(motR, pwm);
}

/* [10] 배터리 / 슬립 */

float batteryVolts() {
  if (PIN_VBAT < 0) return 99.0f;
  return analogRead(PIN_VBAT) * (3.3f / 4095.0f) * VBAT_DIVIDER_RATIO;
}

void enterDeepSleep() {
  Serial.println("[SLEEP] deep sleep 진입 — FSR 착석 시 기상");
  allStop();
  driversEnable(false);
  if (PIN_LED >= 0) digitalWrite(PIN_LED, LOW);
  Serial.flush();

  esp_sleep_enable_ext0_wakeup(FSR_WAKE_GPIO, 1);

  
  esp_sleep_enable_timer_wakeup(600ULL * 1000000ULL);

  esp_deep_sleep_start();
}

/* [11] 캘리브레이션 / 수동 테스트 모드 */

bool     telemetry       = false;
uint32_t lastTelemetryMs = 0;

// 자동 슬립 차단 플래그. DEBUG_MODE 이거나, 시리얼 입력이 한 번이라도 들어오면 true.
bool     sleepDisabled   = DEBUG_MODE;
uint32_t bootHoldUntil   = 0;     // 이 시각 전에는 절대 잠들지 않음
uint32_t lastHeartbeatMs = 0;

void printTelemetry() {
  Serial.printf("FSR=%4d | F=%6.1f L=%6.1f R=%6.1f cm | IS L=%4d(b%4d) R=%4d(b%4d) A=%4d(b%4d) | VBAT=%.2f\n",
    fsrRaw(),
    sonarPingOnce(SONAR_FRONT), sonarPingOnce(SONAR_LEFT), sonarPingOnce(SONAR_RIGHT),
    motL.isFiltered, motL.isBaseline,
    motR.isFiltered, motR.isBaseline,
    motAct.isFiltered, motAct.isBaseline,
    batteryVolts());
}

void printHelp() {
  Serial.println("---------------- 시리얼 명령 ----------------");
  Serial.println("  t : 센서값 스트리밍 ON/OFF (FSR/초음파/IS)");
  Serial.println("  l : 액추에이터 뻗기     k : 액추에이터 접기");
  Serial.println("  w : 전진   s : 후진   a : 좌회전   d : 우회전   x : 정지");
  Serial.println("  b : IS baseline 재학습  g : 자동 시퀀스 즉시 실행");
  Serial.println("  i : 현재 상태 출력      z : 강제 deep sleep");
  Serial.println("  h : 이 도움말");
  Serial.println("--------------------------------------------");
}

void handleSerial() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == '\r' || c == '\n' || c == ' ') return;   // 엔터/공백은 무시

  // 사람이 시리얼로 붙었다는 뜻 → 자동 슬립 차단 (테스트 중 갑자기 잠드는 것 방지)
  if (!sleepDisabled) {
    sleepDisabled = true;
    Serial.println("[CAL] 시리얼 입력 감지 — 자동 deep sleep 을 껐습니다. ('z' 로 강제 슬립)");
  }

  switch (c) {
    case 't': telemetry = !telemetry; Serial.printf("[CAL] telemetry %s\n", telemetry ? "ON" : "OFF"); break;
    case 'l': motorSet(motAct,  ACT_PWM); Serial.println("[CAL] 액추에이터 뻗기"); break;
    case 'k': motorSet(motAct, -ACT_PWM); Serial.println("[CAL] 액추에이터 접기"); break;
    case 'w': driveForward(DRIVE_PWM);    Serial.println("[CAL] 전진"); break;
    case 's': driveForward(-DRIVE_PWM);   Serial.println("[CAL] 후진"); break;
    case 'a': spin(false, TURN_PWM);      Serial.println("[CAL] 좌회전"); break;
    case 'd': spin(true,  TURN_PWM);      Serial.println("[CAL] 우회전"); break;
    case 'x': allStop();                  Serial.println("[CAL] 정지"); break;
    case 'g': missionStartMs = millis(); gotoState(ST_LIFT); break;
    case 'b':
      learnBaseline(motL); learnBaseline(motR); learnBaseline(motAct);
      Serial.printf("[CAL] baseline L=%d R=%d A=%d\n", motL.isBaseline, motR.isBaseline, motAct.isBaseline);
      break;
    case 'z':
      Serial.println("[CAL] 강제 deep sleep 진입");
      Serial.flush();
      enterDeepSleep();
      break;
    case 'i':
      Serial.printf("[INFO] state=%s seated=%d FSR=%d sleepDisabled=%d uptime=%lus\n",
                    stateName(state), (int)seated, fsrRaw(), (int)sleepDisabled, millis() / 1000);
      break;
    case 'h':
    case '?':
      printHelp();
      break;
    default:
      Serial.printf("[CAL] 알 수 없는 명령 '%c' — 'h' 로 도움말\n", c);
      break;
  }
}

/* [12] setup / loop */

void setup() {
  Serial.begin(115200);
  delay(200);

  if (PIN_LED >= 0) { pinMode(PIN_LED, OUTPUT); digitalWrite(PIN_LED, HIGH); }
  if (PIN_EN_SHARED >= 0) { pinMode(PIN_EN_SHARED, OUTPUT); digitalWrite(PIN_EN_SHARED, LOW); }

  for (int i = 0; i < 3; i++) {
    pinMode(TRIG_PINS[i], OUTPUT);
    digitalWrite(TRIG_PINS[i], LOW);
    pinMode(ECHO_PINS[i], INPUT);
  }

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_FSR, ADC_11db);      // 0~3.3V 전체 범위

  motorInit(motL);
  motorInit(motR);
  motorInit(motAct);
  driversEnable(true);
  allStop();

  learnBaseline(motL); learnBaseline(motR); learnBaseline(motAct);

  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

  
  for (int i = 0; i < BOOT_BANNER_COUNT; i++) {
    Serial.printf("\n=== 스마트체어 부팅 [%d/%d] wakeup=%d FSR=%d DEBUG=%d ===\n",
                  i + 1, BOOT_BANNER_COUNT, (int)cause, fsrRaw(), (int)DEBUG_MODE);
    Serial.flush();
    delay(500);
  }
  printHelp();
  if (sleepDisabled) Serial.println("[MODE] DEBUG_MODE=true — deep sleep 비활성 상태입니다.");

  bootHoldUntil = millis() + BOOT_HOLD_MS;

  seated = fsrSeatedRaw();
  gotoState(seated ? ST_OCCUPIED : ST_SLEEP_PREP);
}

void loop() {
  // --- 항상 돌아야 하는 서비스 ---
  fsrService();
  currentService(motL);
  currentService(motR);
  currentService(motAct);
  motorService(motL);
  motorService(motR);
  motorService(motAct);
  handleSerial();

  if (telemetry && millis() - lastTelemetryMs > 400) {
    lastTelemetryMs = millis();
    printTelemetry();
  }

  // --- 하트비트 : 보드가 살아있는지 확인용 ---
  if (!telemetry && millis() - lastHeartbeatMs >= HEARTBEAT_MS) {
    lastHeartbeatMs = millis();
    if (PIN_LED >= 0) digitalWrite(PIN_LED, !digitalRead(PIN_LED));
    Serial.printf("[HB] %lus  state=%s  FSR=%4d  seated=%d  sleep=%s\n",
                  millis() / 1000, stateName(state), fsrRaw(), (int)seated,
                  sleepDisabled ? "OFF" : "ON");
  }

  // --- 안전장치 : 동작 중 다시 앉으면 즉시 중단 ---
  bool moving = (state >= ST_LIFT && state <= ST_DRIVE);
  if (REOCCUPY_ABORT && moving && seated) {
    Serial.println("[ABORT] 착석 감지 — 동작 중단");
    allStop();
    gotoState(ST_RETRACT);
  }

  // --- 안전장치 : 전체 미션 타임아웃 ---
  if (moving && missionStartMs && millis() - missionStartMs > TIMEOUT_GLOBAL_MS) {
    Serial.println("[ABORT] 전체 타임아웃");
    allStop();
    gotoState(ST_RETRACT);
  }

  // --- 상태별 처리 ---
  switch (state) {

    case ST_BOOT:
      gotoState(ST_SLEEP_PREP);
      break;

    /* 사람이 앉아 있는 동안 대기. 일어나면 SETTLE 로. */
    case ST_OCCUPIED:
      if (!seated) gotoState(ST_SETTLE);
      break;

    /* 기립 후 유예 시간. 이 동안 다시 앉으면 취소. */
    case ST_SETTLE:
      if (seated) { gotoState(ST_OCCUPIED); break; }
      if (inState() >= SETTLE_DELAY_MS) {
        if (batteryVolts() < VBAT_CUTOFF_V) {
          Serial.println("[BATT] 전압 부족 — 동작 생략");
          gotoState(ST_SLEEP_PREP);
          break;
        }
        missionStartMs = millis();
        learnBaseline(motL); learnBaseline(motR); learnBaseline(motAct);
        gotoState(ST_LIFT);
      }
      break;

    /* 액추에이터로 다리 한쪽을 들어 의자 상판 정렬 */
    case ST_LIFT:
      if (inState() < ACT_EXTEND_MS) {
        motorSet(motAct, ACT_PWM);
        if (motorStalled(motAct)) {                 // 스트로크 끝 도달
          Serial.println("[ACT] 스트로크 끝 감지");
          motorStop(motAct);
          gotoState(ST_SCAN);
        }
      } else if (inState() < ACT_EXTEND_MS + ACT_SETTLE_MS) {
        motorStop(motAct);
      } else {
        gotoState(ST_SCAN);
      }
      break;

    /* 제자리 회전하며 전방 초음파로 가장 가까운 유효 물체(책상) 방향 탐색 */
    case ST_SCAN: {
      if (inState() >= TIMEOUT_SCAN_MS || scanStepIdx >= SCAN_STEPS) {
        allStop();
        if (scanBestStep < 0) {
          Serial.println("[SCAN] 책상 후보 없음 — 동작 취소");
          gotoState(ST_RETRACT);
        } else {
          Serial.printf("[SCAN] best step=%d dist=%.1fcm\n", scanBestStep, scanBestDist);
          gotoState(ST_TURN);
        }
        break;
      }

      if (!scanInit) {
        // 첫 진입: 좌/우 중 가까운 쪽으로 회전 방향 결정
        float dl = sonarRead(SONAR_LEFT);
        float dr = sonarRead(SONAR_RIGHT);
        scanTurnRight = (dr <= dl);
        Serial.printf("[SCAN] L=%.1f R=%.1f -> %s 방향 스캔\n", dl, dr, scanTurnRight ? "우" : "좌");
        scanInit       = true;
        scanStepStart  = millis();
        scanStepMoving = true;
        spin(scanTurnRight, TURN_PWM);
        break;
      }

      if (scanStepMoving) {
        if (millis() - scanStepStart >= SCAN_STEP_MS) {
          allStop();
          scanStepMoving = false;
          scanStepStart  = millis();
        }
      } else {
        if (millis() - scanStepStart >= SCAN_PAUSE_MS) {
          float d = sonarRead(SONAR_FRONT);
          if (d >= DESK_VALID_MIN_CM && d <= DESK_VALID_MAX_CM && d < scanBestDist) {
            scanBestDist = d;
            scanBestStep = scanStepIdx;
          }
          Serial.printf("[SCAN] step %2d : %.1f cm\n", scanStepIdx, d);
          scanStepIdx++;
          scanStepStart  = millis();
          scanStepMoving = true;
          spin(scanTurnRight, TURN_PWM);
        }
      }
      break;
    }

    /* 최적 방향으로 되돌아가기 : (총 스텝 - best) 만큼 반대로 회전 */
    case ST_TURN: {
      uint32_t backMs = (uint32_t)(scanStepIdx - scanBestStep) * SCAN_STEP_MS;
      if (backMs > TIMEOUT_TURN_MS) backMs = TIMEOUT_TURN_MS;
      if (inState() < backMs) {
        spin(!scanTurnRight, TURN_PWM);
      } else {
        allStop();
        delay(300);
        float d = sonarRead(SONAR_FRONT);
        Serial.printf("[TURN] 정렬 완료, 전방 %.1f cm\n", d);
        learnBaseline(motL); learnBaseline(motR);
        gotoState(ST_DRIVE);
      }
      break;
    }

    /* 책상 앞으로 전진. 초음파 거리 1차, 전류 스톨 2차(백업) 로 정지 */
    case ST_DRIVE: {
      driveForward(DRIVE_PWM);

      if (millis() - drivePingMs > 80) {
        drivePingMs = millis();
        driveFrontDist = sonarPingOnce(SONAR_FRONT);
      }

      bool arrivedByDistance = (driveFrontDist <= DESK_STOP_CM);
      bool arrivedByStall    = motorStalled(motL) || motorStalled(motR);
      bool timedOut          = (inState() >= TIMEOUT_DRIVE_MS);

      if (arrivedByDistance || arrivedByStall || timedOut) {
        allStop();
        Serial.printf("[DRIVE] 정지 (거리=%s 스톨=%s 타임아웃=%s, %.1fcm)\n",
                      arrivedByDistance ? "O" : "X",
                      arrivedByStall    ? "O" : "X",
                      timedOut          ? "O" : "X",
                      driveFrontDist);
        delay(300);
        gotoState(ST_RETRACT);
      }
      break;
    }

    /* 액추에이터 원위치 */
    case ST_RETRACT:
      if (inState() < ACT_RETRACT_MS) {
        motorSet(motAct, -ACT_PWM);
        if (inState() > STALL_BLANK_MS && motorStalled(motAct)) {
          motorStop(motAct);
          gotoState(ST_SLEEP_PREP);
        }
      } else {
        motorStop(motAct);
        gotoState(ST_SLEEP_PREP);
      }
      break;

    /* 정리 후 deep sleep */
    case ST_SLEEP_PREP:
      allStop();
      if (inState() < 500) break;                      // 모터 완전 정지 대기
      if (seated) { gotoState(ST_OCCUPIED); break; }   // 그새 누가 앉았으면 취소

      // 부팅 직후 관찰 시간 확보 — 이 시간 안에는 잠들지 않음
      if ((int32_t)(millis() - bootHoldUntil) < 0) break;

      // 디버그 모드이거나 시리얼 입력이 있었으면 잠들지 않고 계속 대기
      if (sleepDisabled) break;

      enterDeepSleep();
      break;
  }
}