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
const int FAR  = 350;  // not used in this snippet
const int NEAR = 500;  // not used in this snippet

// We'll keep a global ring buffer of 6 sensor readings:
int last3[6] = {999, 999, 999, 999, 999, 999};

int i          = 0;
bool process   = false;
int wallcount  = 0;
int n          = 10;
int GapLength  = 0;
bool lastHitLeft = false;

const byte FAST = 255;
const byte SLOW = 60;

// We read sensor every iteration in the 'while' loops
// so 'previousMillis' and 'interval' are not used in this version:
unsigned long previousMillis = 0;
const unsigned long interval = 200;

// Not used in your final code, but if you do need a gapcounter:
int gapcounter     = 0;
const int threshold = 8;


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

      // A) If the last 3 readings are in range (120..250),
      //    move forward until sensor >= 250
      if ( last3[3] < 250 && last3[4] < 250 && last3[5] < 250 &&
           last3[3] > 120 && last3[4] > 120 && last3[5] > 120 ) 
      {
        stopMotors();
        delay(40);
        while (analogRead(IR) < 250) {
          goForward(SLOW);
        }
        stopMotors();
      }

      // B) If left pin is HIGH => we've "hit" the left side
      if (LEFT == HIGH) {
        lastHitLeft = true;
      }
      // C) Else if sensor < 110 => increment GapLength
      else if (sensorValue < 110) {
        GapLength++;
      }
      // D) If sensor >= 110 => check if GapLength was big
      else {
        if (GapLength > 30) {
          // big gap found => do some turn logic
          goRight();
          // turn right for some time
          delay(20 * 15);

          sensorValue = analogRead(IR);
          // move forward while sensor < 120
          while (sensorValue < 120) {
            goForward(SLOW);
            sensorValue = analogRead(IR);

            // Reset the global array if you want to clear them:
            // for (int k=0; k<6; k++) {
            //   last3[k] = 999;
            // }
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

      // A) If the last 3 are in [120..250], go forward
      if ( last3[3] < 250 && last3[4] < 250 && last3[5] < 250 &&
           last3[3] > 120 && last3[4] > 120 && last3[5] > 120 ) 
      {
        stopMotors();
        delay(40);
        while (analogRead(IR) < 250) {
          goForward(SLOW);
        }
        stopMotors();
      }

      // B) If right pin is HIGH => we've "hit" the right side
      if (RIGHT == HIGH) {
        lastHitLeft = false;
      }

      // C) Else if sensor < 110 => increment gap
      else if (sensorValue < 110) {
        GapLength++;
      }
      // D) If sensor >= 110 => check if GapLength was big
      else {
        if (GapLength > 30) {
          goLeft();
          delay(20 * 15);  // turn left for a bit
          sensorValue = analogRead(IR);

          while (sensorValue < 120) {
            goForward(SLOW);
            sensorValue = analogRead(IR);
            // Optionally reset last3 here if you like
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
