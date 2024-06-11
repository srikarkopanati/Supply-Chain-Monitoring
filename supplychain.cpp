#include <DHT.h>
#include <SPI.h>
#include <MFRC522.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <Adafruit_BMP085.h>
#include <Wire.h>
#include <MPU6050.h>

#include <GSM.h>

#define SS_PIN 10    // Slave Select Pin for RFID
#define RST_PIN 9    // Reset Pin for RFID

#define DHTPIN 2     // Pin connected to the DHT sensor
#define DHTTYPE DHT11   // DHT sensor type (DHT11 or DHT22)

#define MOISTURE_PIN A0  // Analog input pin for soil moisture sensor

#define BMP180_SDA_PIN A4   // Pin connected to BMP180 SDA (data)
#define BMP180_SCL_PIN A5   // Pin connected to BMP180 SCL (clock)

MPU6050 mpu;

DHT dht(DHTPIN, DHTTYPE);
MFRC522 rfid(SS_PIN, RST_PIN);    // Create MFRC522 instance
SoftwareSerial gpsSerial(3, 4);   // RX, TX pins for GPS module
TinyGPSPlus gps;                   // Create TinyGPS++ object
Adafruit_BMP085 bmp180;

GSM gsmAccess;
GSM_SMS sms;

bool isRFIDDetected = false;
bool isRFIDDisplayed = false;      // Flag to track if RFID tag detection has been displayed
unsigned long previousMillis = 0;
const unsigned long interval = 10000;  // Interval to display readings (10 seconds)
byte validCardUID[] = {0xA9, 0xAA, 0x04, 0x4C};  // Modify with the valid card UID

// GSM module settings
char phoneNumber[] = "+917278768888";  // Replace with the recipient's phone number

void setup() {
  Serial.begin(9600);
  dht.begin();
  SPI.begin();      // Initialize SPI bus
  rfid.PCD_Init();  // Initialize RFID reader
  gpsSerial.begin(9600);  // Initialize GPS serial communication
  bmp180.begin();  // Initialize BMP180 sensor

  Wire.begin();     // Initialize I2C bus for MPU6050
  mpu.initialize(); // Initialize MPU6050

  pinMode(MOISTURE_PIN, INPUT);  // Set the moisture sensor pin as input

  Serial.println("Place your RFID card near the reader...");

  // Initialize GSM module
  while (gsmAccess.begin() != GSM_READY) {
    Serial.println("Failed to initialize GSM module!");
    delay(1000);
  }
  Serial.println("GSM module initialized.");
}

void loop() {
  unsigned long currentMillis = millis();

  // Check for RFID card
  if (!isRFIDDetected && rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
    bool isAccessGranted = true;

    // Get RFID card UID
    byte detectedCardUID[4];
    for (byte i = 0; i < rfid.uid.size; i++) {
      detectedCardUID[i] = rfid.uid.uidByte[i];
    }

    // Compare detected UID with the valid card UID
    for (byte i = 0; i < sizeof(validCardUID); i++) {
      if (validCardUID[i] != detectedCardUID[i]) {
        isAccessGranted = false;
        break;
      }
    }

    if (isAccessGranted) {
      Serial.println("Access Granted!");
      isRFIDDetected = true;
    } else {
      Serial.println("Access Denied!");
    }

    rfid.PICC_HaltA();   // Halt PICC
    rfid.PCD_StopCrypto1(); // Stop encryption on PCD
  }

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    float humidity = dht.readHumidity();
    float temperature = dht.readTemperature();
    int moisture = analogRead(MOISTURE_PIN);
    float pressure = bmp180.readPressure() / 100.0;

    if (isnan(humidity) || isnan(temperature)) {
      Serial.println("Failed to read from DHT sensor!");
      return;
    }

    if (isRFIDDetected && !isRFIDDisplayed) {
      Serial.println("RFID Tag Detected!");
      isRFIDDisplayed = true;  // Set the flag to true after displaying the message
    }

    while (gpsSerial.available() > 0) {
      if (gps.encode(gpsSerial.read())) {
        if (gps.location.isUpdated()) {
          Serial.print("Latitude: ");
          Serial.print(gps.location.lat(), 6);
          Serial.print("° ");
          Serial.print("Longitude: ");
          Serial.print(gps.location.lng(), 6);
          Serial.println("°");
        }
      }
    }

    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.print("%\t");

    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println("°C");

    Serial.print("Pressure: ");
    Serial.print(pressure);
    Serial.println(" Pa");

    Serial.print("Moisture: ");
    Serial.println(moisture);

    if (humidity > 50) {
      Serial.println("ALERT: Humidity exceeded threshold!");
      sendSMS("ALERT: Humidity exceeded threshold!");
    }
    if (temperature > 20) {
      Serial.println("ALERT: Temperature exceeded threshold!");
      sendSMS("ALERT: Temperature exceeded threshold!");
    }
    if (pressure > 3) {
      Serial.println("ALERT: Pressure exceeded threshold!");
      sendSMS("ALERT: Pressure exceeded threshold!");
    }

    // Read and display MPU6050 data
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    Serial.print("Accelerometer: ");
    Serial.print("X = "); Serial.print(ax);
    Serial.print(" | Y = "); Serial.print(ay);
    Serial.print(" | Z = "); Serial.println(az);

    Serial.print("Gyroscope: ");
    Serial.print("X = "); Serial.print(gx);
    Serial.print(" | Y = "); Serial.print(gy);
    Serial.print(" | Z = "); Serial.println(gz);

    // Add your additional code here

    // Prepare sensor readings for SMS
    String smsContent = "Sensor Readings:\n";
    smsContent += "Humidity: " + String(humidity) + "%\n";
    smsContent += "Temperature: " + String(temperature) + "°C\n";
    smsContent += "Pressure: " + String(pressure) + "Pa\n";
    smsContent += "Moisture: " + String(moisture) + "\n";
    smsContent += "Accelerometer: X=" + String(ax) + " | Y=" + String(ay) + " | Z=" + String(az) + "\n";
    smsContent += "Gyroscope: X=" + String(gx) + " | Y=" + String(gy) + " | Z=" + String(gz) + "\n";

    // Send SMS
    sendSMS(smsContent);
  }
}

void sendSMS(const String& message) {
  sms.beginSMS(phoneNumber);
  sms.print(message);
  sms.endSMS();
  Serial.println("SMS sent.");
}