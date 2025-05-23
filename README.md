# scenariob
const int leftPIN    = 2;
const int rightPIN   = 4;

const int DIRA       = 12;
const int PWMB       = 11;
const int DIRB       = 13;
const int PWMA       = 3;
const int leftMOTOR  = A5;
const int rightMOTOR = A3;

const int IR   = A2;
// 未使用常量已移除

// We'll keep a global ring buffer of 6 sensor readings:
int last3[6] = {999, 999, 999, 999, 999, 999};

// 移除未使用的變量 i 與 process
int wallcount  = 0;
int n          = 10;
int GapLength  = 0;
bool lastHitLeft = false;

// 感測器範圍設定
const int SENSOR_LOW  = 110;
const int SENSOR_MID  = 120;
const int SENSOR_HIGH = 250;

// 判定缺口長度的門檻
const int GAP_THRESHOLD = 30;

const byte FAST = 255;
const byte SLOW = 60;

// 移除未使用的定時相關變量


// --------------------------------------------------------
//                  Motor Control
// --------------------------------------------------------
void goForward(byte speed) {
  digitalWrite(DIRA, HIGH);
  digitalWrite(DIRB, HIGH);
  analogWrite(PWMB, speed);
  analogWrite(PWMA, speed);
}

void goBackward(byte speed) {
  digitalWrite(DIRA, LOW);
  digitalWrite(DIRB, HIGH);
  analogWrite(PWMB, speed);
  analogWrite(PWMA, speed);
}

void goLeft() {
  analogWrite(leftMOTOR, 0);
  analogWrite(rightMOTOR, 200);
}

void goRight() {
  analogWrite(leftMOTOR, 200);
  analogWrite(rightMOTOR, 0);
}

void stopMotors() {
  digitalWrite(DIRA, LOW);
  digitalWrite(DIRB, LOW);
  analogWrite(leftMOTOR, 0);
  analogWrite(rightMOTOR, 0);
  analogWrite(PWMB, 0);
  analogWrite(PWMA, 0);
}

// --------------------------------------------------------
//                       Setup
// --------------------------------------------------------
void setup() {
  pinMode(DIRA, OUTPUT);
  pinMode(DIRB, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(rightMOTOR, OUTPUT);
  pinMode(leftMOTOR, OUTPUT);
  pinMode(leftPIN, INPUT_PULLUP);
  pinMode(rightPIN, INPUT_PULLUP);

  Serial.begin(9600);
}

// --------------------------------------------------------
//                       Main Loop
// --------------------------------------------------------
void loop() {
  // We'll do the "left side" logic or "right side" logic
  // up to 'n' times, controlled by 'wallcount'.
  while (wallcount != n) {
    int LEFT  = digitalRead(leftPIN);
    int RIGHT = digitalRead(rightPIN);

    // --------------------------------------------------------
    //            1) Move while lastHitLeft == false
    //               (i.e., we have not yet pressed LEFT?)
    // --------------------------------------------------------
    while (lastHitLeft == false) {
      goLeft();        // turn left
      LEFT = digitalRead(leftPIN);

      // Read IR sensor, shift into last3 array
      int sensorValue = analogRead(IR);
      shiftReadings(sensorValue);

      // Optional debug
      // Serial.print("L: sensorValue=");
      // Serial.println(sensorValue);

      delay(20);  // small pause each iteration

      // A) 若最近三次讀數位於 SENSOR_MID 與 SENSOR_HIGH 之間，
      //    則向前行進直到感測值大於等於 SENSOR_HIGH
      if ( last3[3] < SENSOR_HIGH && last3[4] < SENSOR_HIGH && last3[5] < SENSOR_HIGH &&
           last3[3] > SENSOR_MID && last3[4] > SENSOR_MID && last3[5] > SENSOR_MID )
      {
        stopMotors();
        delay(40);
        while (analogRead(IR) < SENSOR_HIGH) {
          goForward(SLOW);
        }
        stopMotors();
      }

      // B) If left pin is HIGH => we've "hit" the left side
      if (LEFT == HIGH) {
        lastHitLeft = true;
      }
      // C) 若感測值低於 SENSOR_LOW，則累計 GapLength
      else if (sensorValue < SENSOR_LOW) {
        GapLength++;
      }
      // D) 如果感測值回到 SENSOR_LOW 以上，檢查缺口長度
      else {
        if (GapLength > GAP_THRESHOLD) {
          // big gap found => do some turn logic
          goRight();
          // turn right for some time
          delay(20 * 15);

          sensorValue = analogRead(IR);
          // move forward while sensor < SENSOR_MID
          while (sensorValue < SENSOR_MID) {
            goForward(SLOW);
            sensorValue = analogRead(IR);
            // 重新清空資料以避免舊數值干擾
            resetLast3();
          }

          GapLength = 0;
          wallcount++;
          stopMotors();
          delay(100);

          // Possibly break out if we only want to do 1 gap per iteration:
          // break;
        }
      }

      // If wallcount reached n, we can break out entirely
      if (wallcount == n) {
        break;
      }
    } // end while(!lastHitLeft)

    // If we just exited because wallcount==n, break out
    if (wallcount == n) {
      break;
    }

    // --------------------------------------------------------
    //            2) Move while lastHitLeft == true
    //               (i.e., we have pressed LEFT, so do right side?)
    // --------------------------------------------------------
    while (lastHitLeft == true) {
      goRight();       // turn right
      RIGHT = digitalRead(rightPIN);

      // Read IR sensor, shift into last3 array
      int sensorValue = analogRead(IR);
      shiftReadings(sensorValue);

      // Optional debug
      // Serial.print("R: sensorValue=");
      // Serial.println(sensorValue);

      delay(20);  // small pause

      // A) 若最近三次讀數位於 SENSOR_MID 與 SENSOR_HIGH 之間，則向前行進
      if ( last3[3] < SENSOR_HIGH && last3[4] < SENSOR_HIGH && last3[5] < SENSOR_HIGH &&
           last3[3] > SENSOR_MID && last3[4] > SENSOR_MID && last3[5] > SENSOR_MID )
      {
        stopMotors();
        delay(40);
        while (analogRead(IR) < SENSOR_HIGH) {
          goForward(SLOW);
        }
        stopMotors();
      }

      // B) If right pin is HIGH => we've "hit" the right side
      if (RIGHT == HIGH) {
        lastHitLeft = false;
      }

      // C) 若感測值低於 SENSOR_LOW，則累計 GapLength
      else if (sensorValue < SENSOR_LOW) {
        GapLength++;
      }
      // D) 如果感測值回到 SENSOR_LOW 以上，檢查缺口長度
      else {
        if (GapLength > GAP_THRESHOLD) {
          goLeft();
          delay(20 * 15);  // turn left for a bit
          sensorValue = analogRead(IR);

          while (sensorValue < SENSOR_MID) {
            goForward(SLOW);
            sensorValue = analogRead(IR);
            // 重新清空資料以避免舊數值干擾
            resetLast3();
          }

          GapLength=0;
          wallcount++;
          stopMotors();
          delay(100);

          // Possibly break if only 1 gap per iteration
          // break;
        }
      }

      if (wallcount == n) {
        break;
      }
    } // end while(lastHitLeft)

  } // end while(wallcount != n)

  // Once done, we can stop everything or do something else
  stopMotors();
  // End of loop().
}


// --------------------------------------------------------
//  Helper function to shift the 'last3' array of 6 elements
// --------------------------------------------------------
void shiftReadings(int newVal) {
  // Shift all elements down by 1
  last3[0] = last3[1];
  last3[1] = last3[2];
  last3[2] = last3[3];
  last3[3] = last3[4];
  last3[4] = last3[5];
  last3[5] = newVal;
}

// 將 last3 陣列重置為初始值
void resetLast3() {
  for (int i = 0; i < 6; i++) {
    last3[i] = 999;
  }
}
