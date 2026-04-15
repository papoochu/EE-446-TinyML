#include <Arduino_APDS9960.h>
#include <Arduino_BMI270_BMM150.h>
#include <PDM.h>
#include <math.h>

// Microphone globals
volatile int samplesRead = 0;
short sampleBuffer[256];
float micLevel = 0.0;

// Sensor values
int clearValue = 0;
int proxValue = 0;
float motionValue = 0.0;

// Thresholds
const float MIC_THRESHOLD = 150.0;// microphone activity threshold
const int LIGHT_THRESHOLD = 125;// below this = dark
const float MOTION_THRESHOLD = 0.1;// accel magnitude deviation threshold
const int PROX_THRESHOLD = 200;// below this = near


const unsigned long UPDATE_MS = 250;
unsigned long lastUpdate = 0;

void onPDMdata() {
  int bytesAvailable = PDM.available();
  PDM.read(sampleBuffer, bytesAvailable);
  samplesRead = bytesAvailable / 2;
}

// Setup
void setup() {
  Serial.begin(115200);
  while (!Serial);


  if (!APDS.begin()) {
    Serial.println("Failed to initialize APDS9960.");
    while (1);
  }


  if (!IMU.begin()) {
    Serial.println("Failed to initialize BMI270/BMM150 IMU.");
    while (1);
  }

  // PDM microphone
  PDM.onReceive(onPDMdata);
  if (!PDM.begin(1, 16000)) {
    Serial.println("Failed to initialize PDM microphone.");
    while (1);
  }

  PDM.setGain(30);

  Serial.println("Smart Workspace Situation Classifier started.");
}

float readMicLevel() {
  if (samplesRead <= 0) return micLevel;

  long sumAbs = 0;
  int count = samplesRead;

  for (int i = 0; i < count; i++) {
    sumAbs += abs(sampleBuffer[i]);
  }

  samplesRead = 0;
  return (float)sumAbs / count;
}

int readClearChannel() {
  int r, g, b, c;

  if (APDS.colorAvailable()) {
    APDS.readColor(r, g, b, c);
    return c;
  }

  return clearValue;
}

int readProximityValue() {
  if (APDS.proximityAvailable()) {
    return APDS.readProximity();
  }

  return proxValue;
}


float readMotionMetric() {
  float x, y, z;

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);

    float mag = sqrt(x * x + y * y + z * z);
    return fabs(mag - 1.0);
  }

  return motionValue;
}

// classification

String classifyState(bool sound, bool dark, bool moving, bool nearUser) {
  
  if (sound && !dark && moving && nearUser) {
    return "NOISY_BRIGHT_MOVING_NEAR";
  }
  
  if (!sound && !dark && !moving && !nearUser) {
    return "QUIET_BRIGHT_STEADY_FAR";
  }

  if (sound && !dark && !moving && !nearUser) {
    return "NOISY_BRIGHT_STEADY_FAR";
  }

  if (!sound && dark && !moving && nearUser) {
    return "QUIET_DARK_STEADY_NEAR";
  }

  return "QUIET_BRIGHT_STEADY_FAR";
}

// Main loop
void loop() {
  if (millis() - lastUpdate < UPDATE_MS) return;
  lastUpdate = millis();

  // Raw sensor reads
  micLevel = readMicLevel();
  clearValue = readClearChannel();
  proxValue = readProximityValue();
  motionValue = readMotionMetric();

  bool sound = (micLevel > MIC_THRESHOLD);
  bool dark = (clearValue < LIGHT_THRESHOLD);
  bool moving = (motionValue > MOTION_THRESHOLD);
  bool nearUser = (proxValue < PROX_THRESHOLD);

  // Final state
  String finalLabel = classifyState(sound, dark, moving, nearUser);

  // Required output format
  Serial.print("raw,mic=");
  Serial.print(micLevel);
  Serial.print(",clear=");
  Serial.print(clearValue);
  Serial.print(",motion=");
  Serial.print(motionValue, 4);
  Serial.print(",prox=");
  Serial.println(proxValue);

  Serial.print("flags,sound=");
  Serial.print(sound ? 1 : 0);
  Serial.print(",dark=");
  Serial.print(dark ? 1 : 0);
  Serial.print(",moving=");
  Serial.print(moving ? 1 : 0);
  Serial.print(",near=");
  Serial.println(nearUser ? 1 : 0);

  Serial.print("state,");
  Serial.println(finalLabel);
}