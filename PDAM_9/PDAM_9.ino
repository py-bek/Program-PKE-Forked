#include <EEPROM.h>
#include <MFRC522.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>

#define SS_PIN 5
#define RST_PIN 25
#define FLOW_SENSOR_PIN 34
#define RELAY 26 // Relay pin
#define BUZZER 17 // Buzzer pin
#define ACCESS_DELAY 2000
#define DENIED_DELAY 1000
MFRC522 mfrc522(SS_PIN, RST_PIN);   // Create MFRC522 instance

LiquidCrystal_I2C lcd(0x27, 16, 2); // Initialize LCD

// Global variables
volatile int pulseCount = 0;
float volume = 0.0;       // Volume of used water
float saldo = 0.0;
const float kalibrasi = 4.5; // Calibration factor for the flow meter
const int EEPROM_SIZE = 512; // EEPROM size sufficient for ESP32
const int EEPROM_SALDO_ADDR = 0; // Address to store saldo in EEPROM
float lastSavedSaldo = saldo;

unsigned long lastPulseTime = 0;  // To debounce the pulseCounter

// Interrupt function for flow sensor
void IRAM_ATTR pulseCounter() {
  unsigned long currentTime = millis();
  
  // Simple debounce: only count pulse if it's been more than 50ms since the last one
  if (currentTime - lastPulseTime > 50) {
    pulseCount++;
    lastPulseTime = currentTime;
  }
}

void setup() {
  Serial.begin(9600);
  EEPROM.begin(EEPROM_SIZE);

  lcd.init();
  lcd.backlight();
  delay(500);

  SPI.begin();
  mfrc522.PCD_Init();
  delay(1000);
  Serial.println("RFID Ready!");

  pinMode(FLOW_SENSOR_PIN, INPUT_PULLUP);
  pinMode(RELAY, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  digitalWrite(RELAY, HIGH);

  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), pulseCounter, RISING);

  EEPROM.get(EEPROM_SALDO_ADDR, saldo);
  if (isnan(saldo)) {
    saldo = 0.0;
  }

  Serial.print("Initial saldo from EEPROM: ");
  Serial.println(saldo);
  lastSavedSaldo = saldo;

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Sistem meter air");
  lcd.setCursor(0, 1);
  lcd.print("Saldo: ");
  lcd.print(saldo, 2);
  lcd.print(" L");
}

void loop() {
  static unsigned long lastUpdateTime = 0;

  if (saldo <= 0) {
    saldo = 0;
    digitalWrite(RELAY, HIGH);
    lcd.setCursor(0, 1);
    lcd.print("Saldo habis!    ");
    tone(BUZZER, 1000, 200);
    delay(1000);
    detachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN));

    if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
      handleRFID();
    }
    return;
  }
  
  else {
    pinMode(FLOW_SENSOR_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), pulseCounter, RISING);
  }

  if (millis() - lastUpdateTime >= 100) {
    lastUpdateTime = millis();
    if (pulseCount != 0) {
      float flowRate = (pulseCount / kalibrasi) / 60;  // L/min

      // Only reduce saldo if flowRate is above a small threshold
      if (flowRate > 0.01) {
        volume += flowRate;
        saldo -= flowRate;
        pulseCount = 0;

        lcd.setCursor(0, 1);
        lcd.print("Saldo: ");
        lcd.print(saldo, 2);
        lcd.print(" L   ");

        if (abs(saldo - lastSavedSaldo) >= 0.01) {
          EEPROM.put(EEPROM_SALDO_ADDR, saldo);
          EEPROM.commit();
          lastSavedSaldo = saldo;
        }
      }
    }

    digitalWrite(RELAY, saldo > 0 ? LOW : HIGH);
  }

  if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
    handleRFID();
  }
}

void handleRFID() {
   Serial.print("UID tag :");
   String content = "";
   byte letter;
   for (byte i = 0; i < mfrc522.uid.size; i++) {
      Serial.print(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " ");
      Serial.print(mfrc522.uid.uidByte[i], HEX);
      content.concat(String(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " "));
      content.concat(String(mfrc522.uid.uidByte[i], HEX));
   }
   Serial.println();
   content.toUpperCase();

   // Reset saldo jika kartu RFID yang cocok ditemukan
   if (content.substring(1) == "D3 02 A4 2C") {
      resetSaldo(); // Memanggil fungsi reset saldo ketika UID tertentu terdeteksi
      delay(1000);
   }

   // Tambah saldo jika kartu RFID penambah saldo yang cocok ditemukan
   else if (content.substring(1) == "A3 20 E8 2A") {
      saldo += 1;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Sistem meter air");
      lcd.setCursor(0, 1);
      lcd.print("Saldo: ");
      lcd.print(saldo, 2);
      lcd.print(" L");

      EEPROM.put(EEPROM_SALDO_ADDR, saldo);
      EEPROM.commit();
      lastSavedSaldo = saldo;

      if (saldo > 0) {
         pinMode(FLOW_SENSOR_PIN, INPUT_PULLUP);
         attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), pulseCounter, RISING);
      }
      delay(1000);

   } else {
      Serial.println("Card not valid for saldo addition.");
   }

   mfrc522.PICC_HaltA();
}


// Fungsi untuk reset saldo ke nol
void resetSaldo() {
  saldo = 0.0;
  EEPROM.put(EEPROM_SALDO_ADDR, saldo);
  EEPROM.commit();
  lastSavedSaldo = saldo;
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Sistem Meter Air");
  lcd.setCursor(0, 1);
  lcd.print("Saldo: 0.00 L");

  Serial.println("Saldo reset to zero.");

  // Siapkan kembali untuk menerima kartu RFID lain untuk penambahan saldo
  pinMode(FLOW_SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), pulseCounter, RISING);
}
