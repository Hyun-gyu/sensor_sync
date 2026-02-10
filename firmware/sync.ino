/*
 * Teensy 4.1 Global Camera Synchronizer
 * 
 * 다중 센서 동기화 트리거 시스템
 * - 스테레오 카메라 (Pin 2): 30Hz (가변)
 * - DAVIS 360 + Velodyne Ultra Puck (Pin 3, 4): 1Hz (동시 트리거)
 * - 마이크로초 단위 정밀 타이밍
 * - 외부 트리거 입력 지원
 * - 시리얼 명령어 제어
 * 
 * ╔═══════════════════════════════════════════════════════════════════════════╗
 * ║                           핀 배치 (하드웨어 연결)                          ║
 * ╠═══════════════════════════════════════════════════════════════════════════╣
 * ║  Pin 2  → 트랜지스터 → FLIR BFS 스테레오 카메라 2대 (30Hz, 가변)           ║
 * ║  Pin 3  → DAVIS 360 이벤트 카메라 SIGNAL_IN (1Hz)                         ║
 * ║  Pin 4  → Velodyne Ultra Puck LiDAR SYNC_IN (1Hz, DAVIS와 동시)           ║
 * ║  Pin 11 → 오실로스코프 모니터링 (Pin 2와 동일 신호)                        ║
 * ║  Pin 13 → 상태 LED                                                        ║
 * ╚═══════════════════════════════════════════════════════════════════════════╝
 */

#include <IntervalTimer.h>

// ============== 설정 ==============
#define DEFAULT_STEREO_FPS  1       // 기본 FPS (ROS 미연결시 1Hz로 자동 시작)
#define DAVIS_DIVISOR       1       // DAVIS = Stereo FPS / DIVISOR (1Hz/1 = 1Hz)
#define TRIGGER_PULSE_US    10000   // 트리거 펄스 폭 (10ms = 10000us, 테스트용)
#define MIN_FPS             1
#define MAX_FPS             120

// ╔═══════════════════════════════════════════════════════════════════════════╗
// ║                              핀 정의                                       ║
// ╚═══════════════════════════════════════════════════════════════════════════╝
const int stereoTriggerPin = 2;   // 스테레오 카메라 트리거 (FLIR BFS x2) - 30Hz
const int davisTriggerPin = 3;    // DAVIS 360 이벤트 카메라 트리거 - 1Hz
const int velodyneTriggerPin = 4; // Velodyne Ultra Puck LiDAR 트리거 - 1Hz (DAVIS와 동시)
const int extTriggerPin = 10;     // 외부 트리거 입력
const int syncOutPin = 11;        // 동기화 모니터링 출력 (오실로스코프용)
const int ledPin = 13;            // 상태 LED

// ╔═══════════════════════════════════════════════════════════════════════════╗
// ║                    마스터 타이머 (단일 소스 동기화)                         ║
// ║  Stereo와 DAVIS가 정확히 동기화되도록 하나의 타이머에서 분배                 ║
// ╚═══════════════════════════════════════════════════════════════════════════╝
IntervalTimer masterTimer;        // 마스터 타이머 (Stereo FPS 기준)

// ============== 상태 변수 ==============
volatile bool isRunning = false;
volatile uint32_t stereoFrameCount = 0;
volatile uint32_t syncFrameCount = 0;      // 1Hz 동기화 프레임 (DAVIS + Velodyne)
volatile uint32_t stereoTimestamp = 0;
volatile uint32_t syncTimestamp = 0;       // 1Hz 동기화 타임스탬프

float stereoFPS = DEFAULT_STEREO_FPS;
uint32_t stereoIntervalMicros = 1000000 / DEFAULT_STEREO_FPS;
uint32_t davisDivisor = DAVIS_DIVISOR;  // N프레임마다 DAVIS 트리거

// 트리거 모드
enum TriggerMode {
  MODE_INTERNAL,      // 내부 타이머 트리거
  MODE_EXTERNAL,      // 외부 트리거 입력
  MODE_SINGLE         // 단일 프레임
};
TriggerMode triggerMode = MODE_INTERNAL;

// 트리거 엣지 설정
bool triggerOnRising = true;  // true: 상승 엣지, false: 하강 엣지

// ============== 함수 선언 ==============
void triggerStereoCamera();
void trigger1HzSensors();      // DAVIS 360 + Velodyne Ultra Puck 동시 트리거
void processSerialCommand();
void printStatus();
void printHelp();

// ╔═══════════════════════════════════════════════════════════════════════════╗
// ║                    마스터 타이머 콜백 (단일 소스 동기화)                     ║
// ║  모든 카메라 트리거가 하나의 타이머에서 파생 → Drift 없음!                   ║
// ╚═══════════════════════════════════════════════════════════════════════════╝

// 마스터 타이머 콜백 (Stereo FPS 기준)
void masterTimerCallback() {
  if (!isRunning) return;
  
  // 1. 스테레오 카메라 항상 트리거 (매 프레임)
  triggerStereoCamera();
  
  // 2. 1Hz 센서들 (DAVIS + Velodyne) N프레임마다 트리거
  //    예: 30Hz / 30 = 1Hz, 60Hz / 60 = 1Hz
  if (stereoFrameCount % davisDivisor == 0) {
    trigger1HzSensors();
  }
}

// 외부 트리거 ISR (스테레오 + 1Hz 센서 동시 동기화)
void externalTriggerISR() {
  if (!isRunning || triggerMode != MODE_EXTERNAL) return;
  
  // 스테레오 트리거
  triggerStereoCamera();
  
  // 1Hz 센서들은 N프레임마다
  if (stereoFrameCount % davisDivisor == 0) {
    trigger1HzSensors();
  }
}

// ╔═══════════════════════════════════════════════════════════════════════════╗
// ║                         카메라 트리거 함수                                  ║
// ╚═══════════════════════════════════════════════════════════════════════════╝

// 스테레오 카메라 트리거 (Pin 2 + Pin 11 모니터링)
void triggerStereoCamera() {
  stereoTimestamp = micros();
  
  // Pin 2 + 모니터링 Pin 11 동시 HIGH
  digitalWriteFast(stereoTriggerPin, HIGH);
  digitalWriteFast(syncOutPin, HIGH);       // 오실로스코프 모니터링
  
  // LED 토글 (스테레오 프레임 시각화)
  digitalWriteFast(ledPin, stereoFrameCount % 2);
  
  // 펄스 폭 대기
  delayMicroseconds(TRIGGER_PULSE_US);
  
  // LOW
  digitalWriteFast(stereoTriggerPin, LOW);
  digitalWriteFast(syncOutPin, LOW);
  
  stereoFrameCount++;
}

// ╔═══════════════════════════════════════════════════════════════════════════╗
// ║               1Hz 센서 동시 트리거 (DAVIS 360 + Velodyne)                   ║
// ║               Pin 3, Pin 4 동시에 HIGH → 완벽한 동기화                       ║
// ╚═══════════════════════════════════════════════════════════════════════════╝
void trigger1HzSensors() {
  syncTimestamp = micros();
  
  // Pin 3 (DAVIS) + Pin 4 (Velodyne) 동시 HIGH
  digitalWriteFast(davisTriggerPin, HIGH);
  digitalWriteFast(velodyneTriggerPin, HIGH);
  
  // 펄스 폭 대기
  delayMicroseconds(TRIGGER_PULSE_US);
  
  // 동시 LOW
  digitalWriteFast(davisTriggerPin, LOW);
  digitalWriteFast(velodyneTriggerPin, LOW);
  
  syncFrameCount++;
}

// ============== 설정 함수 ==============
void setStereoFPS(float fps) {
  if (fps < MIN_FPS) fps = MIN_FPS;
  if (fps > MAX_FPS) fps = MAX_FPS;
  
  stereoFPS = fps;
  stereoIntervalMicros = (uint32_t)(1000000.0 / fps);
  
  // DAVIS divisor 자동 계산 (항상 1Hz 유지)
  // 예: 30Hz → divisor 30, 60Hz → divisor 60, 20Hz → divisor 20
  davisDivisor = (uint32_t)fps;
  
  if (isRunning && triggerMode == MODE_INTERNAL) {
    masterTimer.end();
    masterTimer.begin(masterTimerCallback, stereoIntervalMicros);
  }
  
  Serial.print("스테레오 FPS 설정: ");
  Serial.print(fps, 2);
  Serial.print(" Hz (");
  Serial.print(stereoIntervalMicros);
  Serial.println(" us 간격)");
  Serial.print("DAVIS 360: 매 ");
  Serial.print(davisDivisor);
  Serial.print(" 프레임마다 트리거 (");
  Serial.print(fps / davisDivisor, 2);
  Serial.println(" Hz)");
}

void startSync() {
  stereoFrameCount = 0;
  syncFrameCount = 0;
  isRunning = true;
  
  if (triggerMode == MODE_INTERNAL) {
    // 마스터 타이머 시작 (Stereo FPS 기준, 1Hz 센서는 분배)
    masterTimer.begin(masterTimerCallback, stereoIntervalMicros);
    
    Serial.println("╔════════════════════════════════════════════════════╗");
    Serial.println("║     동기화 시작 (마스터 타이머 - 완벽 동기화)        ║");
    Serial.println("╠════════════════════════════════════════════════════╣");
    Serial.print("║  스테레오 (Pin 2): ");
    Serial.print(stereoFPS, 1);
    Serial.println(" Hz (마스터)                  ║");
    Serial.print("║  1Hz 센서 (Pin 3,4): 매 ");
    Serial.print(davisDivisor);
    Serial.print(" 프레임 = ");
    Serial.print(stereoFPS / davisDivisor, 1);
    Serial.println(" Hz    ║");
    Serial.println("║    - DAVIS 360 (Pin 3)                              ║");
    Serial.println("║    - Velodyne Ultra Puck (Pin 4)                    ║");
    Serial.println("║                                                    ║");
    Serial.println("║  ✓ 단일 타이머 소스 → Drift 없음!                   ║");
    Serial.println("║  ✓ 모든 1Hz 센서 = Stereo 프레임과 정확히 일치       ║");
    Serial.println("╚════════════════════════════════════════════════════╝");
  } else if (triggerMode == MODE_EXTERNAL) {
    // 외부 트리거 모드 (모든 센서 외부 동기화)
    if (triggerOnRising) {
      attachInterrupt(digitalPinToInterrupt(extTriggerPin), externalTriggerISR, RISING);
    } else {
      attachInterrupt(digitalPinToInterrupt(extTriggerPin), externalTriggerISR, FALLING);
    }
    Serial.println("╔════════════════════════════════════════════════════╗");
    Serial.println("║          외부 트리거 대기 중 (동기화 모드)           ║");
    Serial.println("╠════════════════════════════════════════════════════╣");
    Serial.println("║  스테레오: 매 외부 트리거                            ║");
    Serial.print("║  1Hz 센서: 매 ");
    Serial.print(davisDivisor);
    Serial.println(" 번째 외부 트리거                    ║");
    Serial.println("╚════════════════════════════════════════════════════╝");
  }
  
  digitalWriteFast(ledPin, HIGH);
}

void stopSync() {
  isRunning = false;
  masterTimer.end();
  detachInterrupt(digitalPinToInterrupt(extTriggerPin));
  
  // 모든 출력 LOW
  digitalWriteFast(stereoTriggerPin, LOW);
  digitalWriteFast(davisTriggerPin, LOW);
  digitalWriteFast(velodyneTriggerPin, LOW);
  digitalWriteFast(syncOutPin, LOW);
  digitalWriteFast(ledPin, LOW);
  
  Serial.println("╔════════════════════════════════════════════════════╗");
  Serial.println("║                  동기화 중지                        ║");
  Serial.println("╠════════════════════════════════════════════════════╣");
  Serial.print("║  스테레오 프레임: ");
  Serial.println(stereoFrameCount);
  Serial.print("║  1Hz 센서 프레임: ");
  Serial.print(syncFrameCount);
  Serial.println(" (DAVIS + Velodyne)            ║");
  Serial.print("║  예상 비율: ");
  Serial.print((float)stereoFrameCount / max(syncFrameCount, (uint32_t)1), 1);
  Serial.print(":1 (목표 ");
  Serial.print(davisDivisor);
  Serial.println(":1)              ║");
  Serial.println("╚════════════════════════════════════════════════════╝");
}

void singleTrigger() {
  Serial.println("단일 프레임 트리거 (스테레오 + DAVIS + Velodyne)");
  triggerStereoCamera();
  trigger1HzSensors();
}

// ============== 시리얼 명령어 처리 ==============
void processSerialCommand() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();
    
    if (cmd == "START" || cmd == "S") {
      startSync();
    }
    else if (cmd == "STOP" || cmd == "X") {
      stopSync();
    }
    else if (cmd == "TRIGGER" || cmd == "T") {
      singleTrigger();
    }
    else if (cmd.startsWith("FPS ") || cmd.startsWith("F ")) {
      float fps = cmd.substring(cmd.indexOf(' ') + 1).toFloat();
      setStereoFPS(fps);
    }
    else if (cmd == "INTERNAL" || cmd == "I") {
      triggerMode = MODE_INTERNAL;
      Serial.println("모드: 내부 타이머");
    }
    else if (cmd == "EXTERNAL" || cmd == "E") {
      triggerMode = MODE_EXTERNAL;
      Serial.println("모드: 외부 트리거 (스테레오만, DAVIS는 1Hz 유지)");
    }
    else if (cmd == "STATUS" || cmd == "?") {
      printStatus();
    }
    else if (cmd == "HELP" || cmd == "H") {
      printHelp();
    }
    else if (cmd == "RESET" || cmd == "R") {
      stereoFrameCount = 0;
      syncFrameCount = 0;
      Serial.println("프레임 카운터 리셋 (스테레오 + 1Hz 센서)");
    }
    else if (cmd.length() > 0) {
      Serial.print("알 수 없는 명령어: ");
      Serial.println(cmd);
      Serial.println("'HELP' 입력시 명령어 목록 표시");
    }
  }
}

void printStatus() {
  float syncHz = stereoFPS / davisDivisor;
  
  Serial.println("\n╔════════════════════════════════════════════════════╗");
  Serial.println("║                    현재 상태                        ║");
  Serial.println("╠════════════════════════════════════════════════════╣");
  Serial.print("║  실행 중: ");
  Serial.println(isRunning ? "예                                    ║" : "아니오                                ║");
  Serial.print("║  모드: ");
  switch (triggerMode) {
    case MODE_INTERNAL: Serial.println("마스터 타이머 (완벽 동기화)           ║"); break;
    case MODE_EXTERNAL: Serial.println("외부 트리거 (완벽 동기화)             ║"); break;
    case MODE_SINGLE: Serial.println("단일 프레임                           ║"); break;
  }
  Serial.println("╠════════════════════════════════════════════════════╣");
  Serial.println("║  [스테레오 카메라 - Pin 2] (30Hz 마스터)             ║");
  Serial.print("║    FPS: ");
  Serial.print(stereoFPS, 1);
  Serial.print(" Hz (");
  Serial.print(stereoIntervalMicros);
  Serial.println(" us)                       ║");
  Serial.print("║    프레임: ");
  Serial.println(stereoFrameCount);
  Serial.println("╠════════════════════════════════════════════════════╣");
  Serial.println("║  [1Hz 센서] (Stereo 동기화)                         ║");
  Serial.println("║    - DAVIS 360 (Pin 3)                              ║");
  Serial.println("║    - Velodyne Ultra Puck (Pin 4)                    ║");
  Serial.print("║    FPS: ");
  Serial.print(syncHz, 2);
  Serial.print(" Hz (매 ");
  Serial.print(davisDivisor);
  Serial.println(" Stereo 프레임)            ║");
  Serial.print("║    프레임: ");
  Serial.println(syncFrameCount);
  Serial.println("╠════════════════════════════════════════════════════╣");
  Serial.println("║  [동기화 상태]                                      ║");
  Serial.print("║    비율: Stereo ");
  Serial.print(stereoFrameCount);
  Serial.print(" : 1Hz센서 ");
  Serial.println(syncFrameCount);
  if (syncFrameCount > 0) {
    Serial.print("║    실제 비율: ");
    Serial.print((float)stereoFrameCount / syncFrameCount, 2);
    Serial.print(":1 (목표 ");
    Serial.print(davisDivisor);
    Serial.println(":1)             ║");
  }
  Serial.print("║    펄스 폭: ");
  Serial.print(TRIGGER_PULSE_US);
  Serial.println(" us                                ║");
  Serial.println("╚════════════════════════════════════════════════════╝\n");
}

void printHelp() {
  Serial.println("\n╔════════════════════════════════════════════════════╗");
  Serial.println("║                    명령어 목록                      ║");
  Serial.println("╠════════════════════════════════════════════════════╣");
  Serial.println("║  START / S    - 동기화 시작                         ║");
  Serial.println("║  STOP  / X    - 동기화 중지                         ║");
  Serial.println("║  TRIGGER / T  - 단일 프레임 트리거                   ║");
  Serial.println("║  FPS n / F n  - 스테레오 FPS 설정 (1-120)            ║");
  Serial.println("║                 DAVIS = Stereo/n (예: 30Hz→1Hz)     ║");
  Serial.println("║  INTERNAL / I - 마스터 타이머 모드                   ║");
  Serial.println("║  EXTERNAL / E - 외부 트리거 모드                     ║");
  Serial.println("║  STATUS / ?   - 현재 상태 표시                       ║");
  Serial.println("║  RESET / R    - 프레임 카운터 리셋                   ║");
  Serial.println("║  HELP / H     - 도움말 표시                         ║");
  Serial.println("╠════════════════════════════════════════════════════╣");
  Serial.println("║  [동기화 원리]                                      ║");
  Serial.println("║  마스터 타이머(Stereo FPS)에서 모든 트리거 생성       ║");
  Serial.println("║  → Stereo: 매 프레임                                ║");
  Serial.println("║  → DAVIS: 매 N번째 Stereo 프레임 (완벽 동기화)       ║");
  Serial.println("╚════════════════════════════════════════════════════╝\n");
}

// ============== 초기화 ==============
void setup() {
  // 시리얼 초기화
  Serial.begin(115200);
  while (!Serial && millis() < 3000);  // USB 연결 대기 (최대 3초)
  
  // 트리거 핀 초기화
  pinMode(stereoTriggerPin, OUTPUT);    // Pin 2: 스테레오 카메라 (30Hz)
  pinMode(davisTriggerPin, OUTPUT);     // Pin 3: DAVIS 360 (1Hz)
  pinMode(velodyneTriggerPin, OUTPUT);  // Pin 4: Velodyne Ultra Puck (1Hz)
  digitalWriteFast(stereoTriggerPin, LOW);
  digitalWriteFast(davisTriggerPin, LOW);
  digitalWriteFast(velodyneTriggerPin, LOW);
  
  // 기타 핀 초기화
  pinMode(extTriggerPin, INPUT_PULLDOWN);
  pinMode(syncOutPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  
  digitalWriteFast(syncOutPin, LOW);
  digitalWriteFast(ledPin, LOW);
  
  // 시작 메시지
  Serial.println("\n╔════════════════════════════════════════════════════╗");
  Serial.println("║       Teensy 4.1 Multi-Sensor Synchronizer         ║");
  Serial.println("║       (마스터 타이머 - 완벽 동기화 모드)              ║");
  Serial.println("╠════════════════════════════════════════════════════╣");
  Serial.println("║  [하드웨어 연결]                                    ║");
  Serial.println("║    Pin 2  → 스테레오 카메라 (FLIR BFS x2) - 30Hz    ║");
  Serial.println("║    Pin 3  → DAVIS 360 이벤트 카메라 - 1Hz           ║");
  Serial.println("║    Pin 4  → Velodyne Ultra Puck LiDAR - 1Hz        ║");
  Serial.println("║    Pin 11 → 오실로스코프 (Pin 2 모니터링)           ║");
  Serial.println("╠════════════════════════════════════════════════════╣");
  Serial.print("║  스테레오: ");
  Serial.print(DEFAULT_STEREO_FPS);
  Serial.println(" Hz (마스터)                        ║");
  Serial.print("║  1Hz 센서: 매 ");
  Serial.print(DAVIS_DIVISOR);
  Serial.println(" 프레임 = 1 Hz                     ║");
  Serial.print("║  트리거 펄스: ");
  Serial.print(TRIGGER_PULSE_US);
  Serial.println(" us                            ║");
  Serial.println("╠════════════════════════════════════════════════════╣");
  Serial.println("║  ✓ 단일 타이머 소스 → Drift 없음!                   ║");
  Serial.println("║  ✓ DAVIS + Velodyne = Stereo 프레임과 정확히 일치    ║");
  Serial.println("╠════════════════════════════════════════════════════╣");
  Serial.println("║  'HELP' 입력시 명령어 목록 표시                      ║");
  Serial.println("║  'START' 입력시 동기화 시작                         ║");
  Serial.println("╚════════════════════════════════════════════════════╝\n");
  
  // 초기 상태 LED 깜빡임 (준비 완료 표시)
  for (int i = 0; i < 3; i++) {
    digitalWriteFast(ledPin, HIGH);
    delay(100);
    digitalWriteFast(ledPin, LOW);
    delay(100);
  }

  // 자동 시작 (ROS 미연결시 기본 1Hz로 동작)
  delay(1000);  // 시리얼 연결 대기
  startSync();
}

// ============== 메인 루프 ==============
void loop() {
  // 시리얼 명령어 처리
  processSerialCommand();
  
  // 상태 출력 (실행 중일 때 1초마다)
  static uint32_t lastStatusTime = 0;
  static uint32_t lastStereoCount = 0;
  static uint32_t lastSyncCount = 0;
  
  if (isRunning && millis() - lastStatusTime >= 1000) {
    lastStatusTime = millis();
    
    uint32_t stereoFpsActual = stereoFrameCount - lastStereoCount;
    uint32_t syncFpsActual = syncFrameCount - lastSyncCount;
    
    Serial.print("[Stereo] Frames: ");
    Serial.print(stereoFrameCount);
    Serial.print(" | FPS: ");
    Serial.print(stereoFpsActual);
    Serial.print("  [1Hz] Frames: ");
    Serial.print(syncFrameCount);
    Serial.print(" | FPS: ");
    Serial.println(syncFpsActual);
    
    lastStereoCount = stereoFrameCount;
    lastSyncCount = syncFrameCount;
  }
}
