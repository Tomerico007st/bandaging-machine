#include <AccelStepper.h>
#include <WiFiS3.h>
#include "Arduino_LED_Matrix.h"
#include <WiFi.h>

#define MOTOR1_STEP_PIN 2
#define MOTOR1_DIR_PIN 5
#define MOTOR1_ENABLE_PIN 8

#define MOTOR2_STEP_PIN 3
#define MOTOR2_DIR_PIN 6
#define MOTOR2_ENABLE_PIN 8

#define STEPS_PER_REV 800

AccelStepper motor1(AccelStepper::DRIVER, MOTOR1_STEP_PIN, MOTOR1_DIR_PIN);
AccelStepper motor2(AccelStepper::DRIVER, MOTOR2_STEP_PIN, MOTOR2_DIR_PIN);


char ssid[] = "Burgercat";     // Replace with your WiFi name
char pass[] = "12345678"; // Replace with your WiFi password


// char ssid[] = "Nelya";     // Replace with your WiFi name
// char pass[] = "12345678"; // Replace with your WiFi password

int int1 = 0;
int int2 = 0;

int status = WL_IDLE_STATUS;
WiFiServer server(80);  // Web server on port 80
bool checkboxState = false;

ArduinoLEDMatrix matrix;



void setup() {
  matrix.begin();
  // pinMode(MOTOR1_ENABLE_PIN, OUTPUT);
  // pinMode(MOTOR2_ENABLE_PIN, OUTPUT);
  // digitalWrite(MOTOR1_ENABLE_PIN, LOW);
  // digitalWrite(MOTOR2_ENABLE_PIN, LOW);

  // configur();

  //wifi
Serial.begin(9600);
  while (status != WL_CONNECTED) {
    Serial.print("Connecting to ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);
    delay(1000);
  }

  Serial.println("Connected to WiFi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  server.begin();
}



void loop() {
    // start();
  if (checkboxState)
  {
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
    matrix.renderBitmap(frame, 8, 12);
    // moveto(motor1.currentPosition() -5  , 0 );
  }
  else {
    // motor1.stop();
    // motor2.stop();
  matrix.clear();  // Turn off all pixels
  }

wifiloop();

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

void wifiloop() {
  WiFiClient client = server.available();

  if (client) {
    Serial.println("Client connected");

    String request = "";
    unsigned long timeout = millis() + 1000;

    while (client.connected() && millis() < timeout) {
      if (client.available()) {
        char c = client.read();
        request += c;
        if (c == '\n' && request.endsWith("\r\n\r\n")) break;
      }
    }

    Serial.println("=== REQUEST START ===");
    Serial.println(request);
    Serial.println("=== REQUEST END ===");

    // MOTOR TOGGLE
    if (request.indexOf("GET /motorOn") >= 0) {
      checkboxState = true;
    } else if (request.indexOf("GET /motorOff") >= 0) {
      checkboxState = false;
    }

    // VALUE PARSING
    int val1Index = request.indexOf("val1=");
    if (val1Index >= 0) {
      val1Index += 5;
      String valStr = "";
      while (val1Index < request.length() && isDigit(request[val1Index])) {
        valStr += request[val1Index];
        val1Index++;
      }
      int1 = valStr.toInt();
    }

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

    // 🖨 Print state
    Serial.print("CheckboxState: ");
    Serial.println(checkboxState);
    Serial.print("int1: ");
    Serial.println(int1);
    Serial.print("int2: ");
    Serial.println(int2);

    // Reply
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println("Connection: close");
    client.println();
    client.println("<html><body><h2>OK!</h2></body></html>");

    delay(10);
    client.stop();
  }
}

