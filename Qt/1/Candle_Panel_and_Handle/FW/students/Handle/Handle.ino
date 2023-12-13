const int axisButtonPin = 8;
const int stopButtonPin = 11;
int load = 2;
int clockPulse = 3;
int dataOut = 4;
int r2 = 7;
int r1 = 6;
int r0 = 5;
int previous_position[] = {0, 0, 0};

uint16_t value;

byte switchVar = 0;
int selectedAxis = 0;
int selectedDirection = 0;
int selectedEnc = 0;
int stopValue = 0;
int Step = 0;

// Promenljive za enkoder
const int encoderPinA = 9;
const int encoderPinB = 10;
int encValue = 0;
int prevEncValue = 0;
bool encoderReset = false;
bool stopButtonPressed = false;

void setup() {
  pinMode(axisButtonPin, INPUT_PULLUP);
  pinMode(stopButtonPin, INPUT);
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  pinMode(load, OUTPUT);
  pinMode(clockPulse, OUTPUT);
  pinMode(dataOut, INPUT);
  pinMode(r2, INPUT);
  pinMode(r1, INPUT);
  pinMode(r0, INPUT);
  Serial.begin(9600);
}

void loop() {
  pinMode(r2, OUTPUT);
  digitalWrite(r2, 0);
  readF(2);

  pinMode(r2, INPUT);
  pinMode(r1, OUTPUT);
  digitalWrite(r2, 1);
  digitalWrite(r1, 0);
  readF(1);

  pinMode(r1, INPUT);
  pinMode(r0, OUTPUT);
  digitalWrite(r1, 1);
  digitalWrite(r0, 0);
  pinMode(r2, OUTPUT);
  digitalWrite(r0, 1);
  readF(0);

  int encoderPos = readEncoder();
  if (encoderPos != prevEncValue) {
    encValue = encoderPos;
    handleEncoder(encValue);
    prevEncValue = encValue;
  }

  if (digitalRead(axisButtonPin) == LOW) {
    delay(180);
    selectedAxis++;
    if (selectedAxis > 3) {
      selectedAxis = 1;
    }
    selectedDirection = 0;

    // Ispis na serijski port
    printSerialOutput();
  }

  selectedEnc = 0; // Resetuj selectedEnc na 0 pre provere smera enkodera

  if (digitalRead(stopButtonPin) == HIGH && !stopButtonPressed) {
    stopButtonPressed = true;
    stopValue = 1;
    printSerialOutput();
  } else if (digitalRead(stopButtonPin) == LOW && stopButtonPressed) {
    stopButtonPressed = false;
    stopValue = 0;
    printSerialOutput();
  }

  Step = 0; // Resetuj Step na 0
}

int readEncoder() {
  static int oldEncPos = 0;
  static int encoderPos = 0;
  static int encoderPinALast = LOW;
  int encoderPinANow = digitalRead(encoderPinA);
  delay(1);
  if (encoderPinANow != encoderPinALast) {
    if (digitalRead(encoderPinB) != encoderPinANow) {
      encoderPos++;
    } else {
      encoderPos--;
    }
  }
  encoderPinALast = encoderPinANow;
  return encoderPos;
}

void handleEncoder(int value) {
  if (value > prevEncValue) {
    selectedDirection = 1;
  } else if (value < prevEncValue) {
    selectedDirection = -1;
  }
  selectedEnc = selectedDirection;

  if (selectedEnc != 0) {
    printSerialOutput();
    selectedEnc = 0;
    encoderReset = true;
  }
  if (encoderReset && selectedEnc == 0) {
    selectedEnc = 0;
    printSerialOutput();
    encoderReset = false;
  }
}

void readF(int sw) {
  uint16_t dataIn = 0;
  int position = -1;
  digitalWrite(clockPulse, 0);
  digitalWrite(load, 0);
  delay(1);
  digitalWrite(clockPulse, 0);

  delay(1);
  digitalWrite(clockPulse, 1);
  delay(1);
  digitalWrite(load, 1);

  delay(1);

  for (int j = 15; j >= 0; j--) {
    digitalWrite(clockPulse, 1);
    delay(1);
    value = digitalRead(dataOut);
    if (value) {
      int a = (1 << j);
      dataIn = dataIn | a;
    } else {
      position = j;
    }

    digitalWrite(clockPulse, 0);
    delay(1);
  }

if (previous_position[sw] != position) {
  if (sw == 1) {
    if (previous_position[sw] > position) {
      Step = -1;
      printSerialOutput();
      Step = 0; // Postavi Step na 0
      printSerialOutput();
    } else if (previous_position[sw] < position) {
      Step = 1;
      printSerialOutput();
      Step = 0; // Postavi Step na 0
      printSerialOutput();
    } else {
      Step = 0;
      printSerialOutput();
    }
  }

  previous_position[sw] = position;
}

  delay(1);
  digitalWrite(clockPulse, 1);

  //delay(100);
}

void printSerialOutput() {
  Serial.print("Axis=");
  Serial.print(selectedAxis);
  Serial.print(" Enc=");
  Serial.print(selectedEnc);
  Serial.print(" Stop=");
  Serial.print(stopValue);
  Serial.print(" Step=");
  Serial.println(Step);
}
