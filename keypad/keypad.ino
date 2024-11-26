#include <Keypad.h>

// Sesuaikan ukuran keypad (4x4 untuk keypad 16 tombol)
const byte ROWS = 4;
const byte COLS = 4;

// Definisikan layout keypad
char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};

// Definisikan pin baris dan kolom di ESP32
byte rowPins[ROWS] = {4, 0, 2, 15};    // Pin baris
byte colPins[COLS] = {27, 14, 12, 13};    // Pin kolom

// Inisialisasi keypad
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

void setup() {
  Serial.begin(9600);
}

void loop() {
  char key = keypad.getKey();

  if (key) {
    Serial.print("Tombol ditekan: ");
    Serial.println(key);
  }
}

