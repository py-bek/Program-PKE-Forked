#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>

// Initialize the LCD with I2C address 0x27 (modify address if needed)
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 16, 2);

// Define the menu items
const int numMenuItems = 3;
String menuItems[numMenuItems] = {"Cek Token", "Isi Token Code", "Monitoring Air"};
int menuIndex = 0;

// Define Sub-Menu of Cek Token
const int numSubIsiToken = 2;
String SubMenuIsiToken[numSubIsiToken] = {"Via Nomor Token", "Via Kartu Bayar"}

// Scroll delay settings

//int scrollDelay = 800; // Speed of scrolling in milliseconds
int scrollDelay = 800;
int scrollPause = 250; // Pause before starting to scroll

// Keypad setup
const byte ROWS = 4;  // Four rows
const byte COLS = 4;  // Four columns
char keys[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};
byte rowPins[ROWS] = {13, 12, 14, 27}; // Connect to the row pins of the keypad
byte colPins[COLS] = {26, 25, 33, 32}; // Connect to the column pins of the keypad

// Create the Keypad object
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

void setup() {
  lcd.init();
  lcd.backlight();
  displayMenu();
}

void loop() {
  char key = keypad.getKey();

  if (key) { // If a key is pressed
    handleKeyPress(key);
  }
}

// Function to handle key press actions
void handleKeyPress(char key) {
  if (key == 'A') {       // Scroll to the previous item
    menuIndex = (menuIndex - 1 + numMenuItems) % numMenuItems;
    lcd.clear();
    displayMenu();
  } else if (key == 'B') { // Scroll to the next item
    menuIndex = (menuIndex + 1) % numMenuItems;
    lcd.clear();
    displayMenu();
  } else if (key == '#') { // Select the current item
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(menuIndex);
    delay(100);
    selectMenuItem();
  }
}

// Function to display the current menu item with scrolling
void displayMenu() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Menu:");
  
  // Display and scroll the selected menu item
  scrollText(menuItems[menuIndex]);
}

// Function to scroll text if it's longer than 16 characters
void scrollText(String text) {
  int len = text.length();
  
  if (len <= 16) { // If the text fits, display it normally
    lcd.setCursor(0, 1);
    lcd.print(text);
  } else { // Scroll the text if it's longer than 16 characters
    lcd.setCursor(0,0);
    lcd.print("Menu :");
    for (int i = 0; i < len + 16; i++) {
      clearRow2();
      lcd.setCursor(0, 1);
      if (i < len) {
      lcd.print(text.substring(i, i + 16));
    } else {
      // Pad the end with spaces for smooth scrolling
      lcd.print(text.substring(i - len));
    }

    delay(scrollDelay); // Control scroll speed
    }
    delay(scrollPause); // Pause at the end before looping
  }
}

// Function to display selected menu item without scrolling
void selectMenuItem() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Selected:");
  lcd.setCursor(0, 1);
  lcd.print(menuItems[menuIndex].substring(0, 16)); // Display the first 16 characters
  delay(2000); // Display the selection for 2 seconds
  displayMenu(); // Return to menu display
}

void clearRow2() {
  lcd.setCursor(0, 1); // Set cursor to the first row
  lcd.print("                "); // Print spaces to clear the first row
}
