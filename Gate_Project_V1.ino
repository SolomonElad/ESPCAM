#include <SPI.h>
#include <MFRC522.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <IRremote.h>
#include <EEPROM.h>

// ==================   ==================
enum IrCode {
  IR_CH_MINUS = 0x45,
  IR_CH = 0x46,
  IR_CH_PLUS = 0x47,
  IR_PREV = 0x44,
  IR_NEXT = 0x40,
  IR_PLAY_PAUSE = 0x43,
  IR_VOL_MINUS = 0x7,
  IR_VOL_PLUS = 0x15,
  IR_EQ = 0x9,
  IR_0 = 0x16,
  IR_100_PLUS = 0x19,
  IR_200_PLUS = 0xD,
  IR_1 = 0xC,
  IR_2 = 0x18,
  IR_3 = 0x5E,
  IR_4 = 0x8,
  IR_5 = 0x1C,
  IR_6 = 0x5A,
  IR_7 = 0x42,
  IR_8 = 0x52,
  IR_9 = 0x4A
};

// ==================  ==================
#define ESP_RX 2
#define ESP_TX 3
#define TRIG_PIN 4
#define ECHO_PIN 5
#define SERVO_PIN 6
#define IR_PIN 7
#define RST_PIN 9
#define SS_PIN 10
#define LED_GREEN A0
#define LED_RED A1
#define BUZZER A2

// ==================   ==================
#define PIN_ADDR 0
#define RFID_COUNT_ADDR 10
#define RFID_START_ADDR 11

const char PROTO_START = '<';
const char PROTO_END = '>';
const char PROTO_SEP = ':';

// ==================   ==================
enum SystemState {
  STATE_WAIT_FOR_CAM,
  STATE_IDLE,
  STATE_ADMIN_AUTH,
  STATE_ADMIN_MENU,
  STATE_INPUT_PLATE,
  STATE_SET_CAR_PERM,
  STATE_RFID_ADD_WAIT,
  STATE_RFID_LIST,
  STATE_RFID_DELETE,          // NEW: State for deleting tags
  STATE_RFID_DELETE_CONFIRM,  // NEW: Confirmation state before deletion
  STATE_CHANGE_PIN,
  STATE_WAIT_ESP_ADMIN,
  STATE_ADMIN_CONFIRM
};

// ==================  ==================
SoftwareSerial espSerial(ESP_RX, ESP_TX);
MFRC522 mfrc522(SS_PIN, RST_PIN);
LiquidCrystal_I2C lcd(0x27, 16, 2);
Servo gateServo;
int currentGateAngle = 0;

// ==================  ==================
SystemState currentState = STATE_WAIT_FOR_CAM;

// Changed to char arrays
char inputBuffer[7] = "";        // 6 chars + null terminator
char currentPlateInput[9] = "";  // 8 chars + null terminator

int adminMenuOption = 1;
int rfidListIndex = 0;
unsigned long rfidTimer = 0;
bool isGateOpen = false;
unsigned long lastSonarTime = 0;
bool waitingForCameraResponse = false;
int selectedMenuAction = 0;

void moveGate(int targetAngle) {
  gateServo.attach(SERVO_PIN);
  int step = (targetAngle > currentGateAngle) ? 1 : -1;
  
  while (currentGateAngle != targetAngle) {
    currentGateAngle += step;
    gateServo.write(currentGateAngle);
    delay(10);
  }

  gateServo.detach();
}


void openGateAdmin() {
  lcd.clear();
  lcd.print(F("Opening Gate..."));
  lcd.setCursor(0, 1);
  lcd.print(F("I`m the manager!"));

  tone(BUZZER, 2000, 200);
  moveGate(90);
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, HIGH);
  delay(3000);

  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_RED, HIGH);
  moveGate(0);
  isGateOpen = false;
  lcd.clear();

  resetIR();
  currentState = STATE_ADMIN_MENU;
  updateMenuDisplay();
}

// ==================     ==================
void resetIR() {
  noTone(BUZZER);
  delay(50);
  IrReceiver.begin(IR_PIN, ENABLE_LED_FEEDBACK);
  delay(50);
}

// ==================  :   ==================
void checkCameraReady() {
  if (espSerial.available() > 0) {
    char packet[64];
    int len = espSerial.readBytesUntil(PROTO_END, packet, 63);
    packet[len] = '\0';  // Null terminate

    char* startPtr = strchr(packet, PROTO_START);
    if (startPtr != NULL) {
      if (strstr(startPtr + 1, "READY") != NULL) {
        lcd.clear();
        lcd.print(F("Camera Online!"));
        tone(BUZZER, 1000, 200);
        delay(100);
        tone(BUZZER, 2000, 300);
        delay(2000);

        resetIR();
        currentState = STATE_IDLE;
        lcd.clear();
        lcd.print(F("Welcome!"));
      }
    }
  }
}

// ==================     (IDLE) ==================
void processEspProtocol_Idle() {
  if (espSerial.available() > 0) {
    char packet[64];
    int len = espSerial.readBytesUntil(PROTO_END, packet, 63);
    packet[len] = '\0';  // Null terminate

    char* startPtr = strchr(packet, PROTO_START);
    if (startPtr == NULL) return;

    // Skip start char
    char* content = startPtr + 1;

    // Split using strtok
    // Format: ARD:TYPE:DATA
    char* header = strtok(content, ":");  // Should be ARD
    char* type = strtok(NULL, ":");
    char* data = strtok(NULL, ":");

    if (type != NULL) {
      if (strcmp(type, "GRANTED") == 0) {
        grantAccess(data != NULL ? data : "");
      } else if (strcmp(type, "DENIED") == 0) {
        denyAccess();
      } else {
        const char* errorMsg = (data != NULL && strlen(data) > 0) ? data : type;
        showErrorAndReset(errorMsg);
        waitingForCameraResponse = false;
      }
    }
  }
}

void showErrorAndReset(const char* errorMsg) {
  lcd.clear();
  lcd.print(F("Error Occurred!"));
  lcd.setCursor(0, 1);

  char tempMsg[17];
  strncpy(tempMsg, errorMsg, 16);
  tempMsg[16] = '\0';  // Ensure null termination
  lcd.print(tempMsg);

  tone(BUZZER, 200, 300);
  delay(400);
  tone(BUZZER, 150, 400);
  delay(3000);

  resetIR();

  lcd.clear();
  lcd.print(F("Welcome!"));
}

// ==================   ==================

// UPDATED: Global exit now includes CH+ for main menu return
bool checkGlobalExit(unsigned long cmd) {
  if (cmd == IR_CH || cmd == IR_CH_MINUS) {
    lcd.clear();
    lcd.print(F("Exiting..."));
    tone(BUZZER, 1000, 200);
    delay(300);
    resetIR();
    currentState = STATE_IDLE;
    closeGateState();
    lcd.clear();
    lcd.print(F("Welcome!"));
    return true;
  }

  // NEW: CH+ returns to main admin menu from anywhere
  if (cmd == IR_CH_PLUS) {
    lcd.clear();
    lcd.print(F("To Main Menu..."));
    tone(BUZZER, 1500, 200);
    delay(300);
    resetIR();
    currentState = STATE_ADMIN_MENU;
    adminMenuOption = 1;
    updateMenuDisplay();
    return true;
  }

  return false;
}

// UPDATED: Menu display now shows two lines with more options
void updateMenuDisplay() {
  lcd.clear();
  lcd.setCursor(0, 0);
  switch (adminMenuOption) {
    case 1:
      lcd.print(F("1.Check Plate"));
      lcd.setCursor(0, 1);
      lcd.print(F("2.Set Plate"));
      break;
    case 2:
      lcd.print(F("2.Set Plate"));
      lcd.setCursor(0, 1);
      lcd.print(F("3.Remove Plate"));
      break;
    case 3:
      lcd.print(F("3.Remove Plate"));
      lcd.setCursor(0, 1);
      lcd.print(F("4.Stats"));
      break;
    case 4:
      lcd.print(F("4.Stats"));
      lcd.setCursor(0, 1);
      lcd.print(F("5.RFID Manager"));
      break;
    case 5:
      lcd.print(F("5.RFID Manager"));
      lcd.setCursor(0, 1);
      lcd.print(F("6.Change PIN"));
      break;
    case 6:
      lcd.print(F("6.Change PIN"));
      lcd.setCursor(0, 1);
      lcd.print(F("7.Open Gate"));
      break;
    case 7:
      lcd.print(F("7.Open Gate"));
      lcd.setCursor(0, 1);
  }
}

void checkIR_Menu() {
  if (IrReceiver.decode()) {
    unsigned long cmd = IrReceiver.decodedIRData.command;
    if (cmd == 0 || (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT)) {
      IrReceiver.resume();
      return;
    }
    if (checkGlobalExit(cmd)) {
      IrReceiver.resume();
      return;
    }

    bool needUpdate = false;
    char digit = getDigitFromIR(cmd);
    if (digit >= '1' && digit <= '7') {
      adminMenuOption = digit - '0';
      needUpdate = true;
    }
    if (cmd == IR_NEXT && adminMenuOption < 7) {
      adminMenuOption++;
      needUpdate = true;
    }
    if (cmd == IR_PREV && adminMenuOption > 1) {
      adminMenuOption--;
      needUpdate = true;
    }

    if (cmd == IR_PLAY_PAUSE || cmd == IR_EQ) {
      handleMenuSelection(adminMenuOption);
      IrReceiver.resume();
      return;
    }
    if (needUpdate) updateMenuDisplay();
    IrReceiver.resume();
  }
}

void handleMenuSelection(int opt) {
  selectedMenuAction = opt;
  lcd.clear();
  if (opt >= 1 && opt <= 3) {
    currentState = STATE_INPUT_PLATE;
    currentPlateInput[0] = '\0';  // Reset string
    lcd.print(F("Plate:"));
    lcd.setCursor(0, 1);
  } else if (opt == 4) {
    lcd.print(F("Fetching Stats.."));
    sendToEsp("STATS", "");
    currentState = STATE_WAIT_ESP_ADMIN;
  } else if (opt == 5) {
    lcd.print(F("1.Add 2.Lst 3.Del"));
    currentState = STATE_RFID_ADD_WAIT;
  } else if (opt == 6) {
    currentState = STATE_CHANGE_PIN;
    inputBuffer[0] = '\0';  // Reset string
    lcd.print(F("New PIN:"));
  } else if (opt == 7) {
    currentState = STATE_ADMIN_CONFIRM;
    inputBuffer[0] = '\0';  // Reset string
  }
}

void checkIR_NumericInput() {
  if (IrReceiver.decode()) {
    unsigned long cmd = IrReceiver.decodedIRData.command;
    if (cmd == 0 || (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT)) {
      IrReceiver.resume();
      return;
    }
    if (checkGlobalExit(cmd)) {
      IrReceiver.resume();
      return;
    }

    int len = strlen(currentPlateInput);

    if (cmd == IR_PREV && len > 0) {
      currentPlateInput[len - 1] = '\0';
      lcd.setCursor(0, 1);
      lcd.print(F("            "));
      lcd.setCursor(0, 1);
      lcd.print(currentPlateInput);
    } else if (cmd == IR_PLAY_PAUSE || cmd == IR_EQ) {
      if (len < 7) {
        lcd.clear();
        lcd.print(F("Too Short!"));
        tone(BUZZER, 500, 300);
        delay(500);
        resetIR();
        lcd.clear();
        lcd.print(F("Plate:"));
        lcd.setCursor(0, 1);
        lcd.print(currentPlateInput);
      } else {
        if (selectedMenuAction == 1) {
          sendToEsp("CHECK", currentPlateInput);
          currentState = STATE_WAIT_ESP_ADMIN;
        } else if (selectedMenuAction == 2) {
          currentState = STATE_SET_CAR_PERM;
          lcd.clear();
          lcd.print(F("1.Allow 2.Block"));
        } else if (selectedMenuAction == 3) {
          sendToEsp("REMOVE", currentPlateInput);
          currentState = STATE_WAIT_ESP_ADMIN;
        }
      }
    } else {
      char d = getDigitFromIR(cmd);
      if (d != 'X' && len < 8) {
        currentPlateInput[len] = d;
        currentPlateInput[len + 1] = '\0';
        lcd.setCursor(0, 1);
        lcd.print(currentPlateInput);
      }
    }
    IrReceiver.resume();
  }
}

void checkIR_SetPerm() {
  if (IrReceiver.decode()) {
    unsigned long cmd = IrReceiver.decodedIRData.command;
    if (cmd == 0 || (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT)) {
      IrReceiver.resume();
      return;
    }
    if (checkGlobalExit(cmd)) {
      IrReceiver.resume();
      return;
    }

    char d = getDigitFromIR(cmd);
    if (d == '1' || d == '2') {
      char tempBuf[32];
      strcpy(tempBuf, currentPlateInput);
      if (d == '1') strcat(tempBuf, ",1");
      else strcat(tempBuf, ",0");

      sendToEsp("SET", tempBuf);
      currentState = STATE_WAIT_ESP_ADMIN;
    }
    IrReceiver.resume();
  }
}

void checkIR_Auth() {
  if (IrReceiver.decode()) {
    unsigned long cmd = IrReceiver.decodedIRData.command;
    if (cmd == 0 || (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT)) {
      IrReceiver.resume();
      return;
    }
    if (checkGlobalExit(cmd)) {
      IrReceiver.resume();
      return;
    }

    int len = strlen(inputBuffer);

    if (cmd == IR_PREV && len > 0) {
      inputBuffer[len - 1] = '\0';
      lcd.setCursor(0, 1);
      lcd.print(F("            "));
      lcd.setCursor(0, 1);
      for (unsigned int i = 0; i < strlen(inputBuffer); i++) lcd.print("*");
    } else if (cmd == IR_PLAY_PAUSE || cmd == IR_EQ) {
      char storedPin[7];
      getAdminPin(storedPin);

      if (strcmp(inputBuffer, storedPin) == 0) {
        lcd.clear();
        lcd.print(F("Hello Admin!"));
        tone(BUZZER, 2000, 200);
        delay(1000);
        resetIR();
        currentState = STATE_ADMIN_MENU;
        adminMenuOption = 1;
        updateMenuDisplay();
      } else {
        lcd.clear();
        lcd.print(F("Wrong PIN!"));
        tone(BUZZER, 500, 500);
        delay(1000);
        resetIR();
        currentState = STATE_IDLE;
        closeGateState();
        lcd.clear();
        lcd.print(F("Welcome!"));
      }
    } else {
      char d = getDigitFromIR(cmd);
      if (d != 'X' && len < 6) {
        inputBuffer[len] = d;
        inputBuffer[len + 1] = '\0';
        lcd.setCursor(len, 1);
        lcd.print("*");
      }
    }
    IrReceiver.resume();
  }
}

void waitForEspResponseAdmin() {
  static unsigned long waitStart = 0;
  if (waitStart == 0) {
    lcd.clear();
    lcd.print(F("Sending..."));
    waitStart = millis();
  }

  if (millis() - waitStart > 10000) {
    lcd.clear();
    lcd.print(F("No Response!"));
    tone(BUZZER, 500, 500);
    delay(1500);
    resetIR();
    waitStart = 0;
    currentState = STATE_ADMIN_MENU;
    updateMenuDisplay();
    return;
  }

  if (espSerial.available() > 0) {
    char packet[64];
    int len = espSerial.readBytesUntil(PROTO_END, packet, 63);
    packet[len] = '\0';

    char* startPtr = strchr(packet, PROTO_START);
    if (startPtr != NULL) {
      // Skip start char
      char* content = startPtr + 1;

      // Split ARD:TYPE:DATA
      char* header = strtok(content, ":");
      char* type = strtok(NULL, ":");
      char* data = strtok(NULL, ":");
      if (data == NULL) data = "";  // Handle empty data

      lcd.clear();

      if (strcmp(type, "ERROR") == 0) {
        lcd.print(F("ERROR:"));
        lcd.setCursor(0, 1);
        char temp[17];
        strncpy(temp, data, 16);
        temp[16] = '\0';
        lcd.print(temp);

        tone(BUZZER, 200, 500);
      } else if (strcmp(type, "OK") == 0) {
        char* arrowStr = strstr(data, "->");
        if (arrowStr != NULL) {
          *arrowStr = '\0';  // Split string at "->"
          char* part1 = data;
          char* part2 = arrowStr + 2;

          lcd.setCursor(0, 0);
          lcd.print(F("OK "));
          lcd.print(part1);
          lcd.setCursor(0, 1);
          lcd.print(part2);
        } else {
          char* spacePtr = strchr(data, ' ');
          if (spacePtr != NULL) {
            *spacePtr = '\0';  // Split at space
            lcd.setCursor(0, 0);
            lcd.print(F("OK "));
            lcd.print(data);
            lcd.setCursor(0, 1);
            lcd.print(spacePtr + 1);
          } else {
            lcd.print(F("OK "));
            lcd.print(data);
          }
        }
        tone(BUZZER, 2000, 200);
      } else {
        char* commaPtr = strchr(data, ',');
        if (commaPtr != NULL) {
          *commaPtr = '\0';  // Split at comma
          lcd.setCursor(0, 0);
          lcd.print(data);
          lcd.setCursor(0, 1);
          lcd.print(commaPtr + 1);
        } else {
          lcd.print(type);
          lcd.setCursor(0, 1);
          lcd.print(data);
        }
        tone(BUZZER, 2000, 200);
      }

      delay(4000);
      resetIR();
      waitStart = 0;
      currentState = STATE_ADMIN_MENU;
      updateMenuDisplay();
    }
  }
}

// ==================   ==================
void checkSonar() {
  if (waitingForCameraResponse) return;
  long duration, distance;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  duration = pulseIn(ECHO_PIN, HIGH, 30000);
  distance = duration * 0.034 / 2;
  if (distance < 10 && distance > 1) {
    Serial.println(F("DEBUG: Sending CHECK request"));
    lcd.clear();
    lcd.print(F("Wait for ID..."));
    sendToEsp("CHECK", "");
    lastSonarTime = millis();
    waitingForCameraResponse = true;
  }
}

void sendToEsp(const char* type, const char* data) {
  espSerial.print(PROTO_START);
  espSerial.print(F("ARD"));
  espSerial.print(PROTO_SEP);
  espSerial.print(type);
  espSerial.print(PROTO_SEP);
  espSerial.print(data);
  espSerial.print(PROTO_END);
}

void grantAccess(const char* id) {
  waitingForCameraResponse = false;
  lcd.clear();
  lcd.print(F("Access Granted!"));
  lcd.setCursor(0, 1);
  lcd.print(id);
  tone(BUZZER, 2000, 100);
  delay(150);
  tone(BUZZER, 2500, 300);
  resetIR();
  moveGate(90);
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, HIGH);
  delay(3000);
  closeGateState();
  lcd.clear();
  lcd.print(F("Welcome!"));
}

void denyAccess() {
  waitingForCameraResponse = false;
  lcd.clear();
  lcd.print(F("Access Denied!"));
  tone(BUZZER, 500);
  delay(300);
  noTone(BUZZER);
  delay(100);
  tone(BUZZER, 500);
  delay(300);
  noTone(BUZZER);
  resetIR();
  digitalWrite(LED_RED, HIGH);
  delay(2000);
  lcd.clear();
  lcd.print(F("Welcome!"));
}

void closeGateState() {
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_RED, HIGH);
  moveGate(0);
  isGateOpen = false;
}

void checkNFC_Idle() {
  if (!mfrc522.PICC_IsNewCardPresent() || !mfrc522.PICC_ReadCardSerial()) return;
  int status = checkTagInEEPROM(mfrc522.uid.uidByte);
  if (status == 1) grantAccess("RFID TAG");
  else denyAccess();
  mfrc522.PICC_HaltA();
}

void checkIR_Idle() {
  if (IrReceiver.decode()) {
    unsigned long cmd = IrReceiver.decodedIRData.command;
    if (cmd == IR_CH || cmd == IR_CH_MINUS) {
      currentState = STATE_ADMIN_AUTH;
      inputBuffer[0] = '\0';  // Reset string
      lcd.clear();
      lcd.print(F("Enter PIN:"));
      lcd.setCursor(0, 1);
    }
    IrReceiver.resume();
  }
}

// UPDATED: RFID Add/List/Delete submenu
void handleRfidAdd() {
  static bool subMenu = true;
  static bool justEntered = true;
  if (justEntered) {
    lcd.clear();
    lcd.print(F("1.Add 2.Lst 3.Del"));
    justEntered = false;
    subMenu = true;
  }
  if (subMenu) {
    if (IrReceiver.decode()) {
      unsigned long cmd = IrReceiver.decodedIRData.command;
      if (checkGlobalExit(cmd)) {
        justEntered = true;
        IrReceiver.resume();
        return;
      }
      char d = getDigitFromIR(cmd);
      if (d == '1') {
        subMenu = false;
        lcd.clear();
        lcd.print(F("Scan Tag (10s)"));
        rfidTimer = millis();
      }
      if (d == '2') {
        subMenu = false;
        currentState = STATE_RFID_LIST;
        rfidListIndex = 0;
        lcd.clear();
        lcd.print(F("Loading List.."));
        IrReceiver.resume();
        justEntered = true;
        return;
      }
      // NEW: Option 3 for deletion
      if (d == '3') {
        subMenu = false;
        currentState = STATE_RFID_DELETE;
        rfidListIndex = 0;
        lcd.clear();
        lcd.print(F("Loading List.."));
        IrReceiver.resume();
        justEntered = true;
        return;
      }
      IrReceiver.resume();
    }
    return;
  }

  // Timeout for tag scanning
  if (millis() - rfidTimer > 10000) {
    lcd.clear();
    lcd.print(F("Timeout!"));
    delay(1000);
    currentState = STATE_ADMIN_MENU;
    updateMenuDisplay();
    justEntered = true;
    return;
  }

  // UPDATED: Check if tag already exists before adding
  if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
    int existingStatus = checkTagInEEPROM(mfrc522.uid.uidByte);

    if (existingStatus == -1) {
      // Tag not found - add as new
      saveTagToEEPROM(mfrc522.uid.uidByte);
      lcd.clear();
      lcd.print(F("Tag Saved!"));
      tone(BUZZER, 2000, 500);
    } else {
      // Tag already exists
      if (existingStatus == 0) {
        // Tag was blocked - enable it
        int tagIndex = findTagIndex(mfrc522.uid.uidByte);
        if (tagIndex != -1) {
          enableTag(tagIndex);
          lcd.clear();
          lcd.print(F("Tag Enabled!"));
          tone(BUZZER, 2000, 500);
        }
      } else {
        // Tag already allowed
        lcd.clear();
        lcd.print(F("Already Exists!"));
        tone(BUZZER, 1000, 300);
      }
    }

    delay(1000);
    resetIR();
    currentState = STATE_ADMIN_MENU;
    updateMenuDisplay();
    justEntered = true;
  }
}

void checkIR_RfidList() {
  int count = EEPROM.read(RFID_COUNT_ADDR);
  static int lastIndex = -1;
  static bool firstDisplay = true;

  // Force initial display
  if (firstDisplay) {
    lastIndex = -1;
    firstDisplay = false;
  }

  if (lastIndex != rfidListIndex) {
    if (count == 0) {
      lcd.clear();
      lcd.print(F("No Tags Found"));
      delay(2000);
      firstDisplay = true;
      currentState = STATE_ADMIN_MENU;
      updateMenuDisplay();
      return;
    }

    byte uid[4];
    bool allowed = readTagFromEEPROM(rfidListIndex, uid);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("Tag "));
    lcd.print(rfidListIndex + 1);
    lcd.print(F("/"));
    lcd.print(count);
    lcd.setCursor(0, 1);
    lcd.print(allowed ? F("[OK] ") : F("[BLK] "));
    for (int i = 0; i < 4; i++) { lcd.print(uid[i], HEX); }
    lastIndex = rfidListIndex;
  }
  if (IrReceiver.decode()) {
    unsigned long cmd = IrReceiver.decodedIRData.command;
    if (checkGlobalExit(cmd)) {
      firstDisplay = true;
      IrReceiver.resume();
      return;
    }
    if (cmd == IR_NEXT) {
      rfidListIndex = (rfidListIndex + 1) % count;
      lastIndex = -1;
    }
    if (cmd == IR_PREV) {
      rfidListIndex = (rfidListIndex - 1 + count) % count;
      lastIndex = -1;
    }
    if (cmd == IR_PLAY_PAUSE) {
      toggleTagStatus(rfidListIndex);
      lcd.setCursor(0, 1);
      lcd.print(F("Saved!      "));
      delay(500);
      lastIndex = -1;
    }
    IrReceiver.resume();
  }
}

// NEW: RFID Delete browsing state
void checkIR_RfidDelete() {
  int count = EEPROM.read(RFID_COUNT_ADDR);
  static int lastIndex = -1;
  static bool firstDisplay = true;

  // Force initial display
  if (firstDisplay) {
    lastIndex = -1;
    firstDisplay = false;
  }

  if (lastIndex != rfidListIndex) {
    if (count == 0) {
      lcd.clear();
      lcd.print(F("No Tags Found"));
      delay(2000);
      firstDisplay = true;
      currentState = STATE_ADMIN_MENU;
      updateMenuDisplay();
      return;
    }

    byte uid[4];
    bool allowed = readTagFromEEPROM(rfidListIndex, uid);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("Del "));
    lcd.print(rfidListIndex + 1);
    lcd.print(F("/"));
    lcd.print(count);
    lcd.setCursor(0, 1);
    lcd.print(allowed ? F("[OK] ") : F("[BLK] "));
    for (int i = 0; i < 4; i++) { lcd.print(uid[i], HEX); }
    lastIndex = rfidListIndex;
  }

  if (IrReceiver.decode()) {
    unsigned long cmd = IrReceiver.decodedIRData.command;
    if (cmd == 0 || (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT)) {
      IrReceiver.resume();
      return;
    }
    if (checkGlobalExit(cmd)) {
      lastIndex = -1;
      firstDisplay = true;
      IrReceiver.resume();
      return;
    }

    // Browse with PREV/NEXT
    if (cmd == IR_NEXT) {
      rfidListIndex = (rfidListIndex + 1) % count;
      lastIndex = -1;
    }
    if (cmd == IR_PREV) {
      rfidListIndex = (rfidListIndex - 1 + count) % count;
      lastIndex = -1;
    }

    // Delete with EQ - go to confirmation
    if (cmd == IR_EQ) {
      currentState = STATE_RFID_DELETE_CONFIRM;
      lcd.clear();
      lcd.print(F("Confirm Delete?"));
      lcd.setCursor(0, 1);
      lcd.print(F("PLAY=Yes EQ=No"));
      lastIndex = -1;
      firstDisplay = true;
    }

    IrReceiver.resume();
  }
}

// NEW: RFID Delete confirmation state
void checkIR_RfidDeleteConfirm() {
  if (IrReceiver.decode()) {
    unsigned long cmd = IrReceiver.decodedIRData.command;
    if (cmd == 0 || (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT)) {
      IrReceiver.resume();
      return;
    }
    if (checkGlobalExit(cmd)) {
      IrReceiver.resume();
      return;
    }

    // PLAY confirms deletion
    if (cmd == IR_PLAY_PAUSE) {
      deleteTagFromEEPROM(rfidListIndex);
      lcd.clear();
      lcd.print(F("Tag Deleted!"));
      tone(BUZZER, 1500, 300);
      delay(1000);
      resetIR();
      currentState = STATE_ADMIN_MENU;
      updateMenuDisplay();
    }

    // EQ cancels deletion - return to delete browse
    if (cmd == IR_EQ) {
      lcd.clear();
      lcd.print(F("Cancelled"));
      tone(BUZZER, 1000, 200);
      resetIR();
      delay(500);
      currentState = STATE_RFID_DELETE;
    }

    IrReceiver.resume();
  }
}

// UPDATED: Change PIN now uses same input mechanism with PREV to delete
void checkIR_ChangePin() {
  if (IrReceiver.decode()) {
    unsigned long cmd = IrReceiver.decodedIRData.command;
    if (cmd == 0 || (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT)) {
      IrReceiver.resume();
      return;
    }
    if (checkGlobalExit(cmd)) {
      IrReceiver.resume();
      return;
    }

    int len = strlen(inputBuffer);

    // PREV to delete last character
    if (cmd == IR_PREV && len > 0) {
      inputBuffer[len - 1] = '\0';
      lcd.setCursor(0, 1);
      lcd.print(F("            "));
      lcd.setCursor(0, 1);
      lcd.print(inputBuffer);
    }
    // PLAY or EQ to confirm (must be exactly 4 digits)
    else if ((cmd == IR_PLAY_PAUSE || cmd == IR_EQ) && len == 4) {
      saveAdminPin(inputBuffer);
      lcd.clear();
      lcd.print(F("PIN Changed!"));
      tone(BUZZER, 2000, 500);
      delay(1000);
      resetIR();
      currentState = STATE_ADMIN_MENU;
      updateMenuDisplay();
    }
    // Add digit
    else {
      char d = getDigitFromIR(cmd);
      if (d != 'X' && len < 4) {
        inputBuffer[len] = d;
        inputBuffer[len + 1] = '\0';
        lcd.setCursor(0, 1);
        lcd.print(inputBuffer);
      }
    }
    IrReceiver.resume();
  }
}

void getAdminPin(char* buffer) {
  for (int i = 0; i < 4; i++) {
    buffer[i] = (char)EEPROM.read(PIN_ADDR + i);
  }
  buffer[4] = '\0';
}

void saveAdminPin(const char* p) {
  for (int i = 0; i < 4; i++) EEPROM.write(PIN_ADDR + i, p[i]);
}

void saveTagToEEPROM(byte* uid) {
  int count = EEPROM.read(RFID_COUNT_ADDR);
  int addr = RFID_START_ADDR + (count * 5);
  for (int i = 0; i < 4; i++) EEPROM.write(addr + i, uid[i]);
  EEPROM.write(addr + 4, 1);
  EEPROM.write(RFID_COUNT_ADDR, count + 1);
}

// UPDATED: Returns -1 if not found, 0 if blocked, 1 if allowed
int checkTagInEEPROM(byte* uid) {
  int count = EEPROM.read(RFID_COUNT_ADDR);
  for (int i = 0; i < count; i++) {
    int addr = RFID_START_ADDR + (i * 5);
    bool match = true;
    for (int j = 0; j < 4; j++) {
      if (EEPROM.read(addr + j) != uid[j]) match = false;
    }
    if (match) return EEPROM.read(addr + 4);
  }
  return -1;  // Not found
}

bool readTagFromEEPROM(int index, byte* uidOut) {
  int addr = RFID_START_ADDR + (index * 5);
  for (int i = 0; i < 4; i++) uidOut[i] = EEPROM.read(addr + i);
  return EEPROM.read(addr + 4) == 1;
}

void toggleTagStatus(int index) {
  int addr = RFID_START_ADDR + (index * 5) + 4;
  EEPROM.write(addr, !EEPROM.read(addr));
}

// NEW: Find tag index by UID
int findTagIndex(byte* uid) {
  int count = EEPROM.read(RFID_COUNT_ADDR);
  for (int i = 0; i < count; i++) {
    int addr = RFID_START_ADDR + (i * 5);
    bool match = true;
    for (int j = 0; j < 4; j++) {
      if (EEPROM.read(addr + j) != uid[j]) match = false;
    }
    if (match) return i;
  }
  return -1;
}

// NEW: Enable a tag (set status to 1)
void enableTag(int index) {
  int addr = RFID_START_ADDR + (index * 5) + 4;
  EEPROM.write(addr, 1);
}

// NEW: Delete tag from EEPROM by shifting all tags after it
void deleteTagFromEEPROM(int index) {
  int count = EEPROM.read(RFID_COUNT_ADDR);
  if (count == 0 || index >= count) return;

  // Shift all tags after the deleted one
  for (int i = index; i < count - 1; i++) {
    int srcAddr = RFID_START_ADDR + ((i + 1) * 5);
    int dstAddr = RFID_START_ADDR + (i * 5);
    for (int j = 0; j < 5; j++) {
      EEPROM.write(dstAddr + j, EEPROM.read(srcAddr + j));
    }
  }

  // Decrease count
  EEPROM.write(RFID_COUNT_ADDR, count - 1);
}

char getDigitFromIR(unsigned long cmd) {
  switch (cmd) {
    case IR_0: return '0';
    case IR_1: return '1';
    case IR_2: return '2';
    case IR_3: return '3';
    case IR_4: return '4';
    case IR_5: return '5';
    case IR_6: return '6';
    case IR_7: return '7';
    case IR_8: return '8';
    case IR_9: return '9';
    default: return 'X';
  }
}


// ================== SETUP ==================
void setup() {
  Serial.begin(9600);
  espSerial.begin(9600);
  espSerial.setTimeout(50);

  SPI.begin();
  mfrc522.PCD_Init();
  lcd.init();
  lcd.backlight();
  IrReceiver.begin(IR_PIN, ENABLE_LED_FEEDBACK);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  if (EEPROM.read(PIN_ADDR) == 0xFF) {
    saveAdminPin("1234");
    EEPROM.write(RFID_COUNT_ADDR, 0);
  }

  closeGateState();

  lcd.clear();
  lcd.print(F("Waiting for Cam"));
  lcd.setCursor(0, 1);
  lcd.print(F("System Booting.."));
}

// ================== LOOP ==================
void loop() {
  switch (currentState) {
    case STATE_WAIT_FOR_CAM:
      checkCameraReady();
      break;

    case STATE_IDLE:
      processEspProtocol_Idle();
      checkNFC_Idle();
      checkIR_Idle();
      if (!isGateOpen && (millis() - lastSonarTime > 5000)) checkSonar();
      break;

    case STATE_ADMIN_AUTH: checkIR_Auth(); break;
    case STATE_ADMIN_MENU: checkIR_Menu(); break;
    case STATE_INPUT_PLATE: checkIR_NumericInput(); break;
    case STATE_SET_CAR_PERM: checkIR_SetPerm(); break;
    case STATE_RFID_ADD_WAIT: handleRfidAdd(); break;
    case STATE_RFID_LIST: checkIR_RfidList(); break;
    case STATE_RFID_DELETE: checkIR_RfidDelete(); break;
    case STATE_RFID_DELETE_CONFIRM: checkIR_RfidDeleteConfirm(); break;
    case STATE_CHANGE_PIN: checkIR_ChangePin(); break;
    case STATE_WAIT_ESP_ADMIN: waitForEspResponseAdmin(); break;
    case STATE_ADMIN_CONFIRM: openGateAdmin(); break;
  }
}