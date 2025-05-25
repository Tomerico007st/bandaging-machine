#include <AccelStepper.h>
#include <WiFiS3.h>                 // WiFi support for Arduino UNO R4 WiFi
#include "Arduino_LED_Matrix.h"     // For displaying on built-in LED matrix

// === Motor control pins ===
#define MOTOREV_STEP_PIN 3
#define MOTOREV_DIR_PIN 4
#define MOTOREV_ENABLE_PIN 5

#define MOTORLI_STEP_PIN 6
#define MOTORLI_DIR_PIN 7
#define MOTORLI_ENABLE_PIN 5  // Shared with Motor 1

#define STEPS_PER_REV 800

// Rotary Encoder Pins
#define ENCODER_CLK A1
#define ENCODER_DT A2

// Proximity Sensor Pin
#define PROXIMITY_PIN A3

// === Stepper motor objects ===
AccelStepper motorrev(AccelStepper::DRIVER, MOTOREV_STEP_PIN, MOTOREV_DIR_PIN);
AccelStepper motorlli(AccelStepper::DRIVER, MOTORLI_STEP_PIN, MOTORLI_DIR_PIN);

// === WiFi credentials ===
char ssid[] = "Burgercat";     // Change to your network name
char pass[] = "12345678";      // Change to your network password

// === State variables ===
int int1 = 0;                  // Value 1 from app
int int2 = 0;                  // Value 2 from app
bool checkboxState = false;   // Controls motor on/off
int status = WL_IDLE_STATUS;  // WiFi status tracking

WiFiServer server(80);        // HTTP server on port 80
ArduinoLEDMatrix matrix;      // LED matrix display object

// Encoder state
int lastEncoderState = 0;
int encoderValue = 0;

bool objectDetected = false;



// === SETUP ===
void setup() {
  matrix.begin();             // Start the matrix
  Serial.begin(9600);         // Open serial connection for debug logs

    pinMode(MOTOREV_ENABLE_PIN, OUTPUT);
  pinMode(MOTORLI_ENABLE_PIN, OUTPUT);
  digitalWrite(MOTOREV_ENABLE_PIN, LOW);
  digitalWrite(MOTORLI_ENABLE_PIN, LOW);

    // Setup Encoder and Proximity
  pinMode(ENCODER_CLK, INPUT);
  pinMode(ENCODER_DT, INPUT);
  pinMode(PROXIMITY_PIN, INPUT);

  configur();

    lastEncoderState = digitalRead(ENCODER_CLK);

  // Attempt to connect to WiFi
  while (status != WL_CONNECTED) {
    Serial.print("Connecting to ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);   // Connect using SSID and password
    delay(1000);
  }


  // Connected!
  Serial.println("Connected to WiFi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  server.begin();   // Start listening for HTTP clients
}

// === LOOP ===
void loop() {
  sensors();
  start();
  int currentState = digitalRead(ENCODER_CLK);
  // If checkboxState is true, show a pattern on the LED matrix
  if (checkboxState) {
    uint8_t frame[8][12] = {
      { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
      { 0, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 0 },
      { 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 0 },
      { 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0 },
      { 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0 },
      { 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0 },
      { 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0 },
      { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
    };
    matrix.renderBitmap(frame, 8, 12);   // Show the frame
    moveto(motorrev.currentPosition() -5  , motorlli.currentPosition() -5 );
  } else {
    matrix.clear();    // Clear matrix if motor is off
    motorrev.stop();
    motorlli.stop();
  }

  wifiloop();  // Check for app requests and handle them
}

void start() {
  motorrev.run();
  motorlli.run();
}

void configur() {
  motorrev.setMaxSpeed(1000);
  motorrev.setAcceleration(1000);
  motorrev.moveTo(STEPS_PER_REV);

  motorlli.setMaxSpeed(1000);
  motorlli.setAcceleration(1000);
  motorlli.moveTo(STEPS_PER_REV);
}

void moveto(long rev, long li){
  if (motorrev.distanceToGo() == 0) {
    motorrev.moveTo(rev);
  }
  if (motorlli.distanceToGo() == 0) {
    motorlli.moveTo(li);
  }
}

// === WIFI HTTP REQUEST HANDLER ===
void wifiloop() {
  WiFiClient client = server.available();  // Check if client connected

  if (client) {
    Serial.println("Client connected");

    String request = "";
    unsigned long timeout = millis() + 1000;  // Timeout safety

    // Read HTTP request
    while (client.connected() && millis() < timeout) {
      if (client.available()) {
        char c = client.read();
        request += c;
        if (c == '\n' && request.endsWith("\r\n\r\n")) break;
      }
    }

    // Debug: show full request
    Serial.println("=== REQUEST START ===");
    Serial.println(request);
    Serial.println("=== REQUEST END ===");

    // === MOTOR TOGGLE HANDLING ===
    if (request.indexOf("GET /motorOn") >= 0) {
      checkboxState = true;
    } else if (request.indexOf("GET /motorOff") >= 0) {
      checkboxState = false;
    }

    // === VALUE PARSING (val1) ===
    int val1Index = request.indexOf("val1=");
    if (val1Index >= 0) {
      val1Index += 5;
      String valStr = "";
      while (val1Index < request.length() && isDigit(request[val1Index])) {
        valStr += request[val1Index];
        val1Index++;
      }
      int1 = valStr.toInt();  // Convert to integer
    }

    // === VALUE PARSING (val2) ===
    int val2Index = request.indexOf("val2=");
    if (val2Index >= 0) {
      val2Index += 5;
      String valStr = "";
      while (val2Index < request.length() && isDigit(request[val2Index])) {
        valStr += request[val2Index];
        val2Index++;
      }
      int2 = valStr.toInt();
    }

    // Print current state to Serial Monitor
    Serial.print("CheckboxState: ");
    Serial.println(checkboxState);
    Serial.print("int1: ");
    Serial.println(int1);
    Serial.print("int2: ");
    Serial.println(int2);

    // Send HTTP response
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println("Connection: close");
    client.println();
    client.println("<html><body><h2>OK!</h2></body></html>");

    delay(10);       // Small delay
    client.stop();   // Close connection
  }
}

void startCalibrate() {
  
}

void sensors(){
  // 🔄 Rotary Encoder Reading
int currentState = digitalRead(ENCODER_CLK);
if (currentState != lastEncoderState && currentState == HIGH) {
  if (digitalRead(ENCODER_DT) != currentState) {
    encoderValue++;
  } else {
    encoderValue--;
  }
}
lastEncoderState = currentState;

// 📡 Proximity Sensor Reading
objectDetected = digitalRead(PROXIMITY_PIN) == LOW;
}

