// motor one
const int A1B = 3;      // brown
const int A1A = 4;      // orange
// motor two
const int B1A = 5;      // yellow
const int B2A = 6;      // green

void setup() {
  // put your setup code here, to run once:
  pinMode(A1B, OUTPUT);
  pinMode(A1A, OUTPUT);
  pinMode(B1A, OUTPUT);
  pinMode(B2A, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(A1B, HIGH);
  digitalWrite(A1A, LOW);
  digitalWrite(B1A, HIGH);
  digitalWrite(B2A, LOW);
  delay(1000);
  digitalWrite(A1B, LOW);
  digitalWrite(A1A, HIGH);
  digitalWrite(B1A, LOW);
  digitalWrite(B2A, HIGH);
  delay(1000);
}