#define vibrationPinOUT A0
#define Serial SerialUSB

void setup() {
  Serial.begin(9600); 
  pinMode(vibrationPinOUT, INPUT);
}

void loop(){
  int vibrationSenzorValue = analogRead(vibrationPinOUT);
  Serial.println(vibrationSenzorValue);
  delay(10);
}
