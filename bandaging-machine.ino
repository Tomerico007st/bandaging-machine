#include <AccelStepper.h>
#include <WiFiS3.h> 

#define MOTOREV_STEP_PIN 6
#define MOTOREV_DIR_PIN 7
#define MOTOREV_ENABLE_PIN 5

#define MOTORLI_STEP_PIN 3
#define MOTORLI_DIR_PIN 4
#define MOTORLI_ENABLE_PIN 5

#define STEPS_PER_REV 800

char ssid[] = "Burgercat";     // Change to your network name
char pass[] = "12345678";

// === State variables ===
int int1 = 0;                  // Value 1 from app
int int2 = 0;                  // Value 2 from app
int int2S = 0;
bool PowerOn = false;   // Controls motor on/off
int status = WL_IDLE_STATUS;  // WiFi status tracking
  bool workDone = false;

int int1Done = 0;
bool moveforword = true;

WiFiServer server(80);        // HTTP server on port 80

AccelStepper motorrev(AccelStepper::DRIVER, MOTOREV_STEP_PIN, MOTOREV_DIR_PIN);
AccelStepper motorlli(AccelStepper::DRIVER, MOTORLI_STEP_PIN, MOTORLI_DIR_PIN);


unsigned long lastTime = 0;       // Last time the task was run
const unsigned long interval = 5000; // 5000ms = 5 seconds

bool homeDone = false;

// Rotary Encoder Pins
#define ENCODER_CLK A1
#define ENCODER_DT A2

// Proximity Sensor Pin
#define PROXIMITY_PIN A3

// Encoder state
static int lastEncoderState = 0;
int encoderValue = 0;

bool objectDetected = false;


void setup() {
config();
Serial.begin(9600);  
WIFIconfig();
sensorSetUp();

   lastEncoderState = digitalRead(ENCODER_CLK);
}

void loop() {
  int currentState = digitalRead(ENCODER_CLK);
  // sensorDIbug();
  motorWork();
// 📡 Proximity Sensor Reading
objectDetected = digitalRead(PROXIMITY_PIN) == LOW;
  autoWIFI();
  if(PowerOn) {
  digitalWrite(MOTOREV_ENABLE_PIN, LOW); // Enable motor
  digitalWrite(MOTORLI_ENABLE_PIN, LOW);
  if(homeDone == false) {
    HOME();
  }
  if(int1 != 0 && homeDone == true){
    StartWork();
  }
  }
  else{
  digitalWrite(MOTOREV_ENABLE_PIN, HIGH); // Enable motor
    digitalWrite(MOTORLI_ENABLE_PIN, HIGH);
      int1Done = 0;
  workDone = false;
  homeDone = false;
    // digitalWrite(MOTOREV_ENABLE_PIN, LOW); // Enable motor
  }
}


void HOME() {
  static bool searchingBack = true;

  if (searchingBack) {
    motorlli.moveTo(motorlli.currentPosition() + 100); // backward
    if (objectDetected) {
      searchingBack = false; // start forward phase
    }
  } else {
    motorlli.moveTo(motorlli.currentPosition() - 100); // forward
    if (!objectDetected) {
      motorlli.setCurrentPosition(0);
      homeDone = true;
    }
  }
}
void StartWork(){
  if(workDone == false ) {
    if(moveforword){
syncMove(-16);  // means 16cm forward
      if(motorlli.currentPosition() <= -25000) {
         int1Done++;
         moveforword = false;
      }
    }
    else{ 
syncMove(16);  // 16cm backward
      if(objectDetected || (int2S < motorlli.currentPosition() && motorlli.currentPosition() < -25000)) {
         int1Done++;
         moveforword = true;
      }
    } 
  }
  if(int1Done == int1){
    workDone = true;
  }
}




void motorWork(){
  motorrev.run();
  motorlli.run();
}
void config() {
  motorrev.setMaxSpeed(3000);
  motorrev.setAcceleration(3000);
  pinMode(MOTOREV_ENABLE_PIN, OUTPUT);
  digitalWrite(MOTOREV_ENABLE_PIN, LOW);

  motorlli.setMaxSpeed(2000);
  motorlli.setAcceleration(2000);
    pinMode(MOTORLI_ENABLE_PIN, OUTPUT);
    digitalWrite(MOTORLI_ENABLE_PIN, LOW);
  
}
void moveToto(long rev, long li){
  if (motorrev.distanceToGo() == 0) {
    motorrev.moveTo(rev);
  }
  if (motorlli.distanceToGo() == 0) {
    motorlli.moveTo(li);
  }
}
void moveToconstant(long rev, long li){
    motorrev.moveTo(rev);
    motorlli.moveTo(li);
}
void syncMove(float cm) {
  long lilTarget = motorlli.currentPosition() + (cm * 100);

  // Always forward REV motion (positive only)
  float revSteps = (cm * 100) / 16.0;
  if (revSteps < 0) revSteps *= -1;

  long revTarget = motorrev.currentPosition() + revSteps;

  // Clamp LIL travel
  if (lilTarget < -25000) lilTarget = -25000;
  if (lilTarget > 0) lilTarget = 0;

  motorlli.moveTo(lilTarget);
  motorrev.moveTo(revTarget);
}




void sensorSetUp() {
    // Setup Encoder and Proximity
  pinMode(ENCODER_CLK, INPUT);
  pinMode(ENCODER_DT, INPUT);
  // External 10k pull-ups already wired
  attachInterrupt(digitalPinToInterrupt(ENCODER_CLK), sensorLoop, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_DT), sensorLoop, CHANGE);

  pinMode(PROXIMITY_PIN, INPUT);
}
void sensorLoop(){
  int MSB = digitalRead(ENCODER_CLK);
  int LSB = digitalRead(ENCODER_DT);
  int encoded = (MSB << 1) | LSB;
  static int lastEncoded = 0;
  int sum = (lastEncoded << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
    encoderValue++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
    encoderValue--;

  lastEncoded = encoded;
}
void sensorDIbug() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastTime >= interval) {
    lastTime = currentMillis;

    int pulsesPerRev = 1440; // 360 PPR × 4 (if using interrupts)
    float gearRatio = 0.5;   // 20 -> 40
    int adjustedSteps = encoderValue * gearRatio;
    float degrees = fmod(adjustedSteps, pulsesPerRev) * (360.0 / pulsesPerRev);

    Serial.print("Encoder: ");
    Serial.print(encoderValue);
    Serial.print(" | Proximity: ");
    Serial.print(objectDetected ? "DETECTED" : "CLEAR");
    Serial.print(" | Angle: ");
    Serial.println(degrees);
  }
}





void WIFIconfig(){
   unsigned long currentTime = millis();
  // Attempt to connect to WiFi
  while (status != WL_CONNECTED) {
    Serial.print("Connecting to ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);   // Connect using SSID and password
  }


  // Connected!
  Serial.println("Connected to WiFi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  server.begin();   // Start listening for HTTP clients
}
void autoWIFI() {
  unsigned long currentMillis = millis();
  if(currentMillis - lastTime >= interval){
    lastTime = currentMillis;

    WiFiClient client = server.available();
  if (!client) return;

  String request = client.readStringUntil('\r'); // read first line only
  client.flush(); // clean up the rest

   int motorIndex = request.indexOf("motor=");
  if (motorIndex >= 0) {
    String state = request.substring(motorIndex + 6, request.indexOf('&', motorIndex));
    PowerOn = (state == "on");
  }

  int val1Index = request.indexOf("val1=");
  if (val1Index >= 0) {
    String temp = request.substring(val1Index + 5);
    int amp = temp.indexOf('&');
    int1 = temp.substring(0, amp >= 0 ? amp : temp.length()).toInt();
  }

  int val2Index = request.indexOf("val2=");
  if (val2Index >= 0) {
    int2 = request.substring(val2Index + 5).toInt();
    int2S = -int2 * 100; // 100 steps = 1 cm, negative = forward direction
  }

  // Debug
  Serial.print("PowerOn: ");
  Serial.println(PowerOn);
  Serial.print("int1: ");
  Serial.println(int1);
  Serial.print("int2: ");
  Serial.println(int2);

  // Response
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println("Connection: close");
  client.println();
  client.println("<html><body><h2>OK!</h2></body></html>");
  client.stop();
  }
}