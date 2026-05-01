#include <Bluepad32.h>

#define LED_PIN 2 

ControllerPtr myControllers[BP32_MAX_GAMEPADS];
String controlState = "O";
bool isControllerConnected = false;

void setup() {
    Serial.begin(115200);
    delay(1000); 

    Serial2.begin(115200, SERIAL_8N1, 16, 17);
    pinMode(LED_PIN, OUTPUT);

    Serial.println("Starting Bluepad32...");
    BP32.setup(&onConnectedController, &onDisconnectedController);
}

void loop() {
    BP32.update();

    isControllerConnected = false;
    String currentState = "OON"; 

    for (auto ctl : myControllers) {
        if (ctl && ctl->isConnected()) {
            isControllerConnected = true;
            
            if (ctl->hasData()) {
                char baseChar = 'O';
                uint16_t btns = ctl->buttons();
                int ry = ctl->axisRY();
                int lx = ctl->axisX();
                int ly = ctl->axisY();

                // ตรวจสอบปุ่มกดพื้นฐาน (Original Mapping)
                if (btns & 0x0001)      baseChar = 'A';
                else if (btns & 0x0002) baseChar = 'B';
                else if (btns & 0x0004) baseChar = 'Y';
                else if (btns & 0x0010) baseChar = 'M';
                else if (ly < -200)     baseChar = 'U';
                else if (ly > 200)      baseChar = 'D';
                else if (lx < -200)     baseChar = 'L';
                else if (lx > 200)      baseChar = 'R';
                else if (ry > 0)        baseChar = 'F';

                // ตรวจสอบปุ่ม P (Emergency Button)
                char emergencyChar = (btns & 0x0020) ? 'P' : 'O';

                if (emergencyChar == 'P' && baseChar == 'O') {
                    baseChar = 'X';
                }

                currentState = String(baseChar) + String(emergencyChar) + "C";
                break; 
            } else {
                currentState = controlState;
                if (currentState.length() >= 3) {
                    currentState.setCharAt(2, 'C');
                } else {
                    currentState = "OOC";
                }
            }
        }
    }

    if (!isControllerConnected) {
        currentState = "OON";
    }

    if (controlState != currentState) {
        controlState = currentState;
        Serial2.println(controlState); 
        // Serial.print("Sent to STM32: "); 
        Serial.println(controlState); 
    }

    if (isControllerConnected) {
        digitalWrite(LED_PIN, HIGH);
    } else {
        digitalWrite(LED_PIN, (millis() / 500) % 2); 
    }

    delay(20);
}

void onConnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            myControllers[i] = ctl;
            Serial.println("Controller connected!");
            break;
        }
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            myControllers[i] = ctl;
            myControllers[i] = nullptr;
            // Serial.println("Controller disconnected!");
            break;
        }
    }
}