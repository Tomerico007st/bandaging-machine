#include <AccelStepper.h>
#include <WiFiS3.h>                 // WiFi support for Arduino UNO R4 WiFi
#include "Arduino_LED_Matrix.h"     // For displaying on built-in LED matrix

// === Motor control pins ===
#define MOTOR1_STEP_PIN 2
#define MOTOR1_DIR_PIN 5
#define MOTOR1_ENABLE_PIN 8

#define MOTOR2_STEP_PIN 3
#define MOTOR2_DIR_PIN 6
#define MOTOR2_ENABLE_PIN 8

#define STEPS_PER_REV 800

// === Stepper motor objects ===
AccelStepper motor1(AccelStepper::DRIVER, MOTOR1_STEP_PIN, MOTOR1_DIR_PIN);
AccelStepper motor2(AccelStepper::DRIVER, MOTOR2_STEP_PIN, MOTOR2_DIR_PIN);

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

// === SETUP ===
void setup() {
  matrix.begin();             // Start the matrix
  Serial.begin(9600);         // Open serial connection for debug logs

    pinMode(MOTOR1_ENABLE_PIN, OUTPUT);
  pinMode(MOTOR2_ENABLE_PIN, OUTPUT);
  digitalWrite(MOTOR1_ENABLE_PIN, LOW);
  digitalWrite(MOTOR2_ENABLE_PIN, LOW);

  configur();

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
  start();
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
    moveto(motor1.currentPosition() -5  , motor2.currentPosition() -5 );
  } else {
    matrix.clear();    // Clear matrix if motor is off
    motor1.stop();
    motor2.stop();
  }

  wifiloop();  // Check for app requests and handle them
}

void start() {
  motor1.run();
  motor2.run();
}

void configur() {
  motor1.setMaxSpeed(1000);
  motor1.setAcceleration(1000);
  motor1.moveTo(STEPS_PER_REV);

  motor2.setMaxSpeed(1000);
  motor2.setAcceleration(1000);
  motor2.moveTo(STEPS_PER_REV);
}

void moveto(long moveto1, long moveto2){
  if (motor1.distanceToGo() == 0) {
    motor1.moveTo(moveto1);
  }
  if (motor2.distanceToGo() == 0) {
    motor2.moveTo(moveto2);
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
