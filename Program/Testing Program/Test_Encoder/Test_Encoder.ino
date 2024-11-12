#define ENA 3
#define PWM 4
#define DIR 5

#define HALL_A  6  
#define HALL_B  7
#define HALL_C  8

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(ENA, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(PWM, OUTPUT);

  pinMode(HALL_A, INPUT);
  pinMode(HALL_B, INPUT);
  pinMode(HALL_C, INPUT);
}

void loop() {
  digitalWrite(ENA, LOW);
  digitalWrite(DIR, LOW);
  digitalWrite(PWM, LOW);
}
