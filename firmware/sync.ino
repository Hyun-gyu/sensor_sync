/*
 * Teensy 4.1 RGB synchronizer
 *
 * Final target behavior:
 * - Pin 3 receives GPS PPS (1 Hz) as the primary timing source
 * - Pin 6 generates RGB trigger pulses derived from PPS
 * - Internal timer mode stays available for debugging without GPS
 * - External trigger mode is kept as a legacy/manual fallback
 */

#include <IntervalTimer.h>

#define FIRMWARE_NAME "sensor_sync_teensy"
#define FIRMWARE_VERSION "2026-04-20"
#define DEFAULT_STEREO_FPS 1.0f
#define MIN_FPS 1.0f
#define MAX_FPS 60.0f
// Keep the default pulse conservative until FLIR trigger timing is hardware-validated.
#define TRIGGER_PULSE_US 10000
#define GPS_PPS_TIMEOUT_MS 1500

const int stereoTriggerPin = 6;
const int gpsPpsInputPin = 3;
const int extTriggerPin = 10;
const int syncOutPin = 11;
const int ledPin = 13;

IntervalTimer stereoTimer;

enum TriggerMode {
  MODE_GPS_PPS,
  MODE_INTERNAL,
  MODE_EXTERNAL,
};

volatile TriggerMode triggerMode = MODE_GPS_PPS;
volatile bool isRunning = false;
volatile bool ppsLocked = false;
volatile uint32_t stereoFrameCount = 0;
volatile uint32_t ppsCount = 0;
volatile uint32_t framesSincePps = 0;
volatile uint32_t targetFramesPerSecond = 1;
volatile uint32_t stereoIntervalMicros = 1000000;
volatile uint32_t lastPpsMillis = 0;

float requestedStereoFps = DEFAULT_STEREO_FPS;
bool startupBannerPrinted = false;

void processSerialCommand();
void printHelp();
void printStatus();
void printStateLine();
void printStartupBanner();
void startSync();
void stopSync();
void applyRuntimeMode();
void setTriggerMode(TriggerMode newMode);
void setStereoFPS(float fps, bool emitAck = true);
void triggerStereoCamera();
void clearOutputs();

const char* modeToString(TriggerMode mode) {
  switch (mode) {
    case MODE_GPS_PPS:
      return "gps_pps";
    case MODE_INTERNAL:
      return "internal_timer";
    case MODE_EXTERNAL:
      return "external_trigger";
  }
  return "unknown";
}

uint32_t fpsToFramesPerSecond(float fps) {
  if (fps < MIN_FPS) {
    fps = MIN_FPS;
  }
  if (fps > MAX_FPS) {
    fps = MAX_FPS;
  }
  return (uint32_t)(fps + 0.5f);
}

void clearOutputs() {
  digitalWriteFast(stereoTriggerPin, LOW);
  digitalWriteFast(syncOutPin, LOW);
}

void triggerStereoCamera() {
  digitalWriteFast(stereoTriggerPin, HIGH);
  digitalWriteFast(syncOutPin, HIGH);
  digitalWriteFast(ledPin, stereoFrameCount % 2);

  delayMicroseconds(TRIGGER_PULSE_US);

  digitalWriteFast(stereoTriggerPin, LOW);
  digitalWriteFast(syncOutPin, LOW);

  stereoFrameCount++;
  if (triggerMode == MODE_GPS_PPS) {
    framesSincePps++;
  }
}

void stereoTimerCallback() {
  if (!isRunning) {
    return;
  }

  if (triggerMode == MODE_INTERNAL) {
    triggerStereoCamera();
    return;
  }

  if (triggerMode == MODE_GPS_PPS) {
    if (!ppsLocked || framesSincePps >= targetFramesPerSecond) {
      stereoTimer.end();
      return;
    }

    triggerStereoCamera();

    if (framesSincePps >= targetFramesPerSecond) {
      stereoTimer.end();
    }
  }
}

void gpsPpsISR() {
  lastPpsMillis = millis();
  ppsLocked = true;
  ppsCount++;
  framesSincePps = 0;

  if (!isRunning || triggerMode != MODE_GPS_PPS) {
    return;
  }

  stereoTimer.end();
  triggerStereoCamera();

  if (framesSincePps < targetFramesPerSecond) {
    stereoTimer.begin(stereoTimerCallback, stereoIntervalMicros);
  }
}

void externalTriggerISR() {
  if (!isRunning || triggerMode != MODE_EXTERNAL) {
    return;
  }

  triggerStereoCamera();
}

void setStereoFPS(float fps, bool emitAck) {
  if (fps < MIN_FPS) {
    fps = MIN_FPS;
  }
  if (fps > MAX_FPS) {
    fps = MAX_FPS;
  }

  requestedStereoFps = fps;
  targetFramesPerSecond = fpsToFramesPerSecond(fps);
  stereoIntervalMicros = (uint32_t)(1000000.0f / (float)targetFramesPerSecond);

  if (isRunning && triggerMode == MODE_INTERNAL) {
    stereoTimer.end();
    stereoTimer.begin(stereoTimerCallback, stereoIntervalMicros);
  }

  if (emitAck) {
    Serial.print("ACK fps=");
    Serial.print((float)targetFramesPerSecond, 2);
    Serial.print(" interval_us=");
    Serial.println(stereoIntervalMicros);
  }
}

void applyRuntimeMode() {
  stereoTimer.end();
  detachInterrupt(digitalPinToInterrupt(extTriggerPin));
  clearOutputs();

  if (!isRunning) {
    return;
  }

  if (triggerMode == MODE_INTERNAL) {
    stereoTimer.begin(stereoTimerCallback, stereoIntervalMicros);
  } else if (triggerMode == MODE_EXTERNAL) {
    attachInterrupt(digitalPinToInterrupt(extTriggerPin), externalTriggerISR, RISING);
  }
}

void setTriggerMode(TriggerMode newMode) {
  triggerMode = newMode;
  applyRuntimeMode();

  Serial.print("ACK mode=");
  Serial.println(modeToString(triggerMode));
}

void startSync() {
  stereoFrameCount = 0;
  framesSincePps = 0;
  isRunning = true;
  applyRuntimeMode();

  Serial.print("ACK running=1 mode=");
  Serial.println(modeToString(triggerMode));
}

void stopSync() {
  isRunning = false;
  applyRuntimeMode();
  digitalWriteFast(ledPin, LOW);

  Serial.println("ACK running=0");
}

void printStateLine() {
  long ppsAgeMs = -1;
  if (lastPpsMillis != 0) {
    ppsAgeMs = (long)(millis() - lastPpsMillis);
  }

  Serial.print("STATE ");
  Serial.print("mode=");
  Serial.print(modeToString(triggerMode));
  Serial.print(" ");
  Serial.print("running=");
  Serial.print(isRunning ? 1 : 0);
  Serial.print(" ");
  Serial.print("fps=");
  Serial.print((float)targetFramesPerSecond, 2);
  Serial.print(" ");
  Serial.print("pps_locked=");
  Serial.print(ppsLocked ? 1 : 0);
  Serial.print(" ");
  Serial.print("pps_age_ms=");
  Serial.print(ppsAgeMs);
  Serial.print(" ");
  Serial.print("stereo_frames=");
  Serial.print(stereoFrameCount);
  Serial.print(" ");
  Serial.print("pps_count=");
  Serial.println(ppsCount);
}

void printStatus() {
  Serial.println();
  Serial.println("=== Teensy Sync Status ===");
  Serial.print("Mode: ");
  Serial.println(modeToString(triggerMode));
  Serial.print("Running: ");
  Serial.println(isRunning ? "yes" : "no");
  Serial.print("Stereo FPS: ");
  Serial.println((float)targetFramesPerSecond, 2);
  Serial.print("PPS locked: ");
  Serial.println(ppsLocked ? "yes" : "no");
  Serial.print("PPS count: ");
  Serial.println(ppsCount);
  Serial.print("Stereo frames: ");
  Serial.println(stereoFrameCount);
  printStateLine();
}

void printHelp() {
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  START / S              Start sync");
  Serial.println("  STOP / X               Stop sync");
  Serial.println("  TRIGGER / T            Send one RGB trigger");
  Serial.println("  FPS n / F n            Set target RGB FPS");
  Serial.println("  MODE GPS_PPS           Use GPS PPS on pin 3 (default)");
  Serial.println("  MODE INTERNAL          Use Teensy internal timer");
  Serial.println("  MODE EXTERNAL          Use external trigger on pin 10");
  Serial.println("  GPS / G                Alias for MODE GPS_PPS");
  Serial.println("  INTERNAL / I           Alias for MODE INTERNAL");
  Serial.println("  EXTERNAL / E           Alias for MODE EXTERNAL");
  Serial.println("  STATUS / ?             Print status");
  Serial.println("  RESET / R              Reset counters");
  Serial.println("  HELP / H               Print this help");
}

void printStartupBanner() {
  if (!Serial || startupBannerPrinted) {
    return;
  }

  Serial.println();
  Serial.println("BOOT_OK upload_ok=1");
  Serial.print("firmware=");
  Serial.print(FIRMWARE_NAME);
  Serial.print(" version=");
  Serial.println(FIRMWARE_VERSION);
  Serial.println("Teensy 4.1 RGB synchronizer ready");
  Serial.println("GPS PPS input: Pin 3");
  Serial.println("RGB trigger output: Pin 6");
  Serial.println("Fallback mode: internal_timer");
  Serial.print("READY mode=");
  Serial.print(modeToString(triggerMode));
  Serial.print(" fps=");
  Serial.print((float)targetFramesPerSecond, 2);
  Serial.print(" running=");
  Serial.println(isRunning ? 1 : 0);
  printHelp();

  startupBannerPrinted = true;
}

void processSerialCommand() {
  if (!Serial.available()) {
    return;
  }

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  cmd.toUpperCase();

  if (cmd == "START" || cmd == "S") {
    startSync();
  } else if (cmd == "STOP" || cmd == "X") {
    stopSync();
  } else if (cmd == "TRIGGER" || cmd == "T") {
    triggerStereoCamera();
    Serial.println("ACK trigger=1");
  } else if (cmd.startsWith("FPS ") || cmd.startsWith("F ")) {
    float fps = cmd.substring(cmd.indexOf(' ') + 1).toFloat();
    setStereoFPS(fps, true);
  } else if (cmd == "GPS" || cmd == "G" || cmd == "GPS_PPS" || cmd == "MODE GPS_PPS" || cmd == "MODE GPS") {
    setTriggerMode(MODE_GPS_PPS);
  } else if (cmd == "INTERNAL" || cmd == "I" || cmd == "INTERNAL_TIMER" || cmd == "MODE INTERNAL" || cmd == "MODE INTERNAL_TIMER") {
    setTriggerMode(MODE_INTERNAL);
  } else if (cmd == "EXTERNAL" || cmd == "E" || cmd == "EXTERNAL_TRIGGER" || cmd == "MODE EXTERNAL" || cmd == "MODE EXTERNAL_TRIGGER") {
    setTriggerMode(MODE_EXTERNAL);
  } else if (cmd == "STATUS" || cmd == "?") {
    printStatus();
  } else if (cmd == "RESET" || cmd == "R") {
    stereoFrameCount = 0;
    ppsCount = 0;
    framesSincePps = 0;
    Serial.println("ACK reset=1");
  } else if (cmd == "HELP" || cmd == "H") {
    printHelp();
  } else if (cmd.length() > 0) {
    Serial.print("ERR unknown_command=");
    Serial.println(cmd);
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  pinMode(stereoTriggerPin, OUTPUT);
  pinMode(gpsPpsInputPin, INPUT_PULLDOWN);
  pinMode(extTriggerPin, INPUT_PULLDOWN);
  pinMode(syncOutPin, OUTPUT);
  pinMode(ledPin, OUTPUT);

  clearOutputs();
  digitalWriteFast(ledPin, LOW);

  attachInterrupt(digitalPinToInterrupt(gpsPpsInputPin), gpsPpsISR, RISING);

  setStereoFPS(DEFAULT_STEREO_FPS, false);
  printStartupBanner();

  for (int i = 0; i < 3; i++) {
    digitalWriteFast(ledPin, HIGH);
    delay(100);
    digitalWriteFast(ledPin, LOW);
    delay(100);
  }

  delay(1000);
  startSync();
}

void loop() {
  printStartupBanner();
  processSerialCommand();

  if (triggerMode == MODE_GPS_PPS && ppsLocked) {
    if ((millis() - lastPpsMillis) > GPS_PPS_TIMEOUT_MS) {
      ppsLocked = false;
      stereoTimer.end();
      clearOutputs();
      Serial.println("WARN pps_timeout=1");
      printStateLine();
    }
  }
}
