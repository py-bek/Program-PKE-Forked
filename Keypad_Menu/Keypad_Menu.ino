

#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <EEPROM.h>
#include <MFRC522.h>
#include <SPI.h>

#define SS_PIN 5
#define RST_PIN 25
#define FLOW_SENSOR_PIN 34
#define RELAY 26 // Relay pin
#define BUZZER 17 // Buzzer pin
#define ACCESS_DELAY 2000
#define DENIED_DELAY 1000
MFRC522 mfrc522(SS_PIN, RST_PIN);   // Create MFRC522 instance

// Global variables
volatile int pulseCount = 0;
float volume = 0.0;       // Volume of used water
float saldo = 0.0;
const float kalibrasi = 4.5; // Calibration factor for the flow meter
const int EEPROM_SIZE = 512; // EEPROM size sufficient for ESP32
const int EEPROM_SALDO_ADDR = 0;// Address to store saldo in EEPROM
const int emoney = 1;
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

bool inmenu = false; // Apakah pengguna sedang berada di menu?


// Initialize the LCD with I2C address 0x27 (modify address if needed)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Keypad setup
const byte ROWS = 4;  // Four rows
const byte COLS = 4;  // Four columns
char keys[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};
byte rowPins[ROWS] = {4, 0, 2, 15};    // Pin baris
byte colPins[COLS] = {27, 14, 12, 13};  
// Create the Keypad object
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

String input = ""; // Stores the number being typed

String nilai_token[5] ={
  "888822112024", 
  "777723112024",
  "666624112024",
  "555525112024",
  "444426112024"};

int nilai_pulsa[5] = {
  1,
  2,
  3,
  4,
  5};

float readBalance(int address) {
  float balance = 0.0;
  EEPROM.get(address, balance);
  return balance;
}

// Function to write balance to EEPROM
void writeBalance(int address, float balance) {
  EEPROM.put(address, balance);
  EEPROM.commit();
}

static unsigned long lastUpdateTime = 0;

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

  float balance = readBalance(emoney);
  if (balance < 10.0){
    balance = balance + (10.0 - balance);
    writeBalance(emoney, balance);
    }
}

void loop() {
  check_saldo();

  evaluate_inmenu(inmenu, lastUpdateTime);
  
  char key = keypad.getKey(); // Read the key pressed
  if (key){
  inmenu = true;
  int result = pressing_key(key);

  if (result != -1) { // If a valid result is returne
    select_menu(result); // Pass the result to select_menu
  }
 }
}

int pressing_key(char key) {
    lcd.clear();
    lcd.print("Masukkan Menu :");
    inmenu=true;
    if (key == '#') { // Confirm the input on pressing '#'
      int result = input.toInt();
      input = "";          // Clear the input after selection
      lcd.setCursor(0, 1);
      lcd.print("                "); // Clear the second line
      return result;
    }
    else if (key == 'D') { // Backspace functionality
      if (input.length() > 0) {
        input.remove(input.length() - 1); // Remove the last character
        lcd.setCursor(0, 1);
        lcd.print("                "); // Clear the second line
        lcd.setCursor(0, 1);
        lcd.print(input); // Print the updated input
      }
    }
    else if (isDigit(key)) { // If it's a numeric key (0-9)
      input += key;           // Append to the input string
      lcd.setCursor(0, 1);     // Move to the second line of the LCD
      lcd.print(input);        // Display the current input
    }
  return -1;
 }

void evaluate_inmenu(bool inmenu, unsigned long &lastUpdateTime) {
    Serial.println("Entering evaluate_inmenu...");
    
    if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
        Serial.println("New RFID card detected!");
        handleRFID();  // Process the RFID card
        return;        // Exit once handled
    }

    if (!inmenu) {
        Serial.println("Displaying flow...");
        display_flow(lastUpdateTime); // Update display or perform logic
        return;
    }

    Serial.println("Inmenu is active, doing nothing.");
}

void check_saldo() {
    if (saldo <= 0) {
        saldo = 0;
        digitalWrite(RELAY, HIGH);
        lcd.setCursor(0, 1);
        lcd.print("Saldo habis!    ");

        // Buzzer non-blocking tone for 1 second
        unsigned long startMillis = millis();
        const unsigned long duration = 5000; // 1 second
        tone(BUZZER, 1000,1000); // Start buzzer tone
        char key = keypad.getKey(); // Check for key press
        while (millis() - startMillis < duration) {
            char key = keypad.getKey(); // Check for key press
            if (key) {
                noTone(BUZZER); // Stop the buzzer immediately
                int result = pressing_key(key);
                if (result != -1) {
                    select_menu(result); // Go to menu
                    return; // Exit the function
                }
            }
        }
        noTone(BUZZER); // Stop the buzzer after timeout
        if (!key){
          detachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN));
          }
    } else {
        pinMode(FLOW_SENSOR_PIN, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), pulseCounter, RISING);
    }
}



void display_flow(unsigned long &lastUpdateTime) {

  if (millis() - lastUpdateTime >= 100) {
    lastUpdateTime = millis();
    lcd.setCursor(0, 0);
    lcd.print("Sistem meter air");
    lcd.setCursor(0, 1);
    lcd.print("Saldo: ");
    lcd.print(saldo, 2);
    lcd.print(" L   ");
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
 else {
      Serial.println("Card not valid for saldo addition.");
   }

   mfrc522.PICC_HaltA();
}

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

void select_menu(int digit_key) {
  int digit_int = digit_key;
  switch(digit_int) {
    case 1:
      isitoken();
      break;
    case 2:
      monitoring();
      break;
    default:
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Invalid selection");
      delay(1000); // Delay to show message before resetting
      lcd.clear();
      inmenu = false;
      break;
  }
  return;
}

void isitoken() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Isi Nilai :");
  lcd.setCursor(0,1);
  int rfid_value = pressing_value();
  
  waitForRFIDTap(rfid_value);
  delay(2000); // Optional delay to show message before returning
  inmenu = false ;
  Serial.print(inmenu);
  lcd.clear();
  evaluate_inmenu(inmenu, lastUpdateTime);
  return;
}

void monitoring() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Masukkan Token (12 digit)");
  lcd.setCursor(0,1);
  String token = pressing_token();
  evaluate_token(token);
  lcd.clear();
  inmenu = false ;
  evaluate_inmenu(inmenu, lastUpdateTime); 
  return;
}

String pressing_token(){
  String input_token= "";
  while (true){
    char key = keypad.getKey(); // Read the key pressed
    if (key) { // If a key is pressed
      if (key == '#') { // Confirm the inpu
        lcd.clear();
        lcd.print("Token Confirmed!");
        delay(2000);  // Briefly display confirmation
        lcd.clear();
        inmenu = false; // Exit inmenu mode
        check_saldo();  // Proceed with saldo checking or other logic
        return input_token;
      } 
      else if (key == 'D') { // Backspace functionality
        if (input_token.length() > 0) {
          input_token.remove(input.length() - 1); // Remove the last character
          lcd.setCursor(0, 1);
          lcd.print("                "); // Clear the second line
          lcd.setCursor(0, 1);
          lcd.print(input_token); // Display the updated input
        }
      } 
      else if (isDigit(key)) { // If it's a numeric key (0-9)
        if (input_token.length() < 12) { // Limit input to 12 digits
          input_token += key;            // Append the key to the input
          lcd.setCursor(0, 1);     // Move to the second line
          lcd.print(input_token);        // Display the current input
        } else {
          lcd.setCursor(0, 1);
          lcd.print("Max Digits Reached");
          delay(1000);
          lcd.setCursor(0, 1);
          lcd.print(input_token); // Redisplay current input
        }
      }
    }
  }
}

void evaluate_token(String input_token){
  for (int i = 0; i < 5;++i){
    if (nilai_token[i] == input_token){
      int pulsa = nilai_pulsa[i];
      saldo = saldo + pulsa;
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Berhasil Mengisi Token!");
      lcd.setCursor(0,1);
      lcd.print("Saldo: ");
      lcd.print(saldo, 2);
      lcd.print(" L   ");
      delay(3000);
      inmenu = false;
      }
     else if (nilai_token[i] != input_token) {
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Invalid Token!");
      delay(3000);
      inmenu = false;
      }
     else{
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Error!!");
      delay(3000);
      inmenu = false;
      break;
      }
    }
  }
  
 int pressing_value(){
    String input_value = "";
    while (true){
    char key = keypad.getKey(); // Read the key pressed
    if (key) { // If a key is pressed
      if (key == '#') { // Confirm the inpu
        lcd.clear();
        lcd.print("Token Confirmed!");
        delay(2000);  // Briefly display confirmation
        lcd.clear();
        int result_value = input_value.toInt();
        return result_value;
      } 
      else if (key == 'D') { // Backspace functionality
        if (input_value.length() > 0) {
          input_value.remove(input.length() - 1); // Remove the last character
          lcd.setCursor(0, 1);
          lcd.print("                "); // Clear the second line
          lcd.setCursor(0, 1);
          lcd.print(input_value); // Display the updated input
        }
      } 
      else if (isDigit(key)) { // If it's a numeric key (0-9)
        if (input_value.length() < 12) { // Limit input to 12 digits
          input_value += key;            // Append the key to the input
          lcd.setCursor(0, 1);     // Move to the second line
          lcd.print(input_value);        // Display the current input
        } else {
          lcd.setCursor(0, 1);
          lcd.print("Max Digits Reached");
          delay(1000);
          lcd.setCursor(0, 1);
          lcd.print(input_value); // Redisplay current input
          }
        }
      }
    }
   }
   
void waitForRFIDTap(int input_value) {
  Serial.println("Tempelkan kartu RFID untuk pembayaran...");
  while (!mfrc522.PICC_IsNewCardPresent() || !mfrc522.PICC_ReadCardSerial()) {
    lcd.clear();
    lcd.print("Tempelkan Kartu");
    delay(500); // Tunggu kartu RFID
  }
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
  if (content.substring(1) == "A3 20 E8 2A") {
  handleRFIDPayment(input_value);
  inmenu = false;
  }
}

void handleRFIDPayment(int input_value) {
  float tokenValue = input_value/1000;
  float balance = readBalance(emoney);
  // Baca saldo dari EEPROM
  Serial.print("Saldo saat ini: ");
  Serial.println(balance);

  // Verifikasi apakah saldo cukup
  if (balance >= tokenValue) {
    balance -= tokenValue; // Kurangi saldo
    writeBalance(emoney, balance); // Simpan saldo baru
    Serial.println("Pembayaran berhasil!");
    Serial.print("Saldo baru: ");
    Serial.println(balance);
    saldo += tokenValue; // Masukkan token ke meteran
    writeBalance(EEPROM_SALDO_ADDR, saldo);
    lcd.clear();
    lcd.print("Pengisian Berhasil");
    delay(3000);
    inmenu = false;
    display_flow(lastUpdateTime);

  } else {
    lcd.clear();
    lcd.print("Saldo tidak cukup");
    delay(3000);
    Serial.println("Saldo tidak cukup! Kembali ke menu utama.");
    inmenu = false;
    display_flow(lastUpdateTime);
  }

  // Hentikan komunikasi dengan kartu
  mfrc522.PICC_HaltA();
  display_flow(lastUpdateTime);
}
