#include <MFRC522.h>
#include <MFRC522Extended.h>
#include <deprecated.h>
#include <require_cpp11.h>

#include <LiquidCrystal_I2C.h>
#include <SPI.h>

#define SS_PIN 5
#define RST_PIN 25
#define FLOW_SENSOR_PIN 34
#define RELAY 26 //relay pin
#define BUZZER 17 //buzzer pin
#define ACCESS_DELAY 2000
#define DENIED_DELAY 1000
MFRC522 mfrc522(SS_PIN, RST_PIN);   // Create MFRC522 instance.

// Inisialisasi LCD dengan alamat I2C 0x27, 16 kolom dan 2 baris
LiquidCrystal_I2C lcd(0x27, 16, 2);


// Variabel global
volatile int pulseCount = 0;
float volume = 0.0;       // Volume air yang terpakai
float saldo = 10.0;       // Saldo awal dalam liter
const float kalibrasi = 4.5; // Faktor kalibrasi untuk sensor flow meter

// Fungsi Interupsi untuk sensor flow meter
void IRAM_ATTR pulseCounter() {
  pulseCount++;
}

void setup() {
  Serial.begin(9600);

  // Inisialisasi modul LCD
  lcd.init();      // Inisialisasi LCD dengan 16 kolom dan 2 baris
  lcd.backlight();       // Mengaktifkan lampu latar LCD
  lcd.setCursor(0, 0);
  lcd.print("Sistem Meter Air");

  // Inisialisasi modul RFID
  SPI.begin();      // Initiate  SPI bus
  mfrc522.PCD_Init(); // Initiate MFRC522
  delay(1000);
  Serial.println("RFID Siap!");

  // Inisialisasi pin
  pinMode(FLOW_SENSOR_PIN, INPUT);
  pinMode(RELAY, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  digitalWrite(RELAY, LOW); // Mulai dengan valve tertutup

  // Setup interupsi untuk sensor aliran
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), pulseCounter, RISING);

  lcd.setCursor(0, 1);
  lcd.print("Saldo: ");
  lcd.print(saldo);
  lcd.print(" L");
}

void loop() {

  if (saldo <= 0)   // Cek saldo air
  {
    digitalWrite(RELAY, LOW); // Tutup valve
    lcd.setCursor(0, 1);
    lcd.print("Saldo Habis!    ");
    tone(BUZZER, 1000, 200); // Buzzer saat saldo habis
    delay(1000);
    return;
  }
    // Look for new cards
  if ( ! mfrc522.PICC_IsNewCardPresent()) 
  {
    return;
  }
  // Select one of the cards
  if ( ! mfrc522.PICC_ReadCardSerial()) 
  {
    return;
  }


// Show UID on serial monitor
  Serial.print("UID tag :");
  String content= "";
  byte letter;
  for (byte i = 0; i < mfrc522.uid.size; i++) 
  {
     Serial.print(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " ");
     Serial.print(mfrc522.uid.uidByte[i], HEX);
     content.concat(String(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " "));
     content.concat(String(mfrc522.uid.uidByte[i], HEX));
  }

  Serial.println();
  Serial.print("Message : ");
  content.toUpperCase();
    // Periksa apakah UID sesuai dengan kartu yang valid
  if (content.substring(1) == "A3 20 E8 2A") // Ganti dengan format UID yang sesuai
  { 
      saldo += 10; // Tambah saldo
      Serial.println("Authorized access");
      Serial.println();
      lcd.setCursor(0, 1);
      lcd.print("Saldo: ");
      lcd.print(saldo);
      lcd.print(" L  ");
      delay(1000); 
    } 
  else {
      lcd.print("kartu tidak dikenali");
      Serial.println("Kartu tidak valid untuk menambah saldo.");
    }

  // Hitung penggunaan air
  if (pulseCount != 0) {
    float flowRate = (pulseCount / kalibrasi) / 60; // L/min
    volume += flowRate;
    saldo -= flowRate;
    pulseCount = 0;

    // Update tampilan saldo dan volume
    lcd.setCursor(0, 1);
    lcd.print("Saldo: ");
    lcd.print(saldo, 2);
    lcd.print(" L   ");
  }

  // Kontrol valve
  if (saldo > 0) {
    digitalWrite(RELAY, HIGH); // Buka valve
  } 
  else {
    digitalWrite(RELAY, LOW); // Tutup valve jika saldo habis
  }

  delay(1000); // Penundaan untuk mengurangi frekuensi update tampilan

  
}
