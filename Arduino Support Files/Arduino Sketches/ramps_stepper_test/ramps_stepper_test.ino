byte directionPin = 55;
byte stepPin = 54;
byte enablePin = 38;
int numberOfSteps = 150;
int pulseWidthMicros = 50;  // microseconds
int millisbetweenSteps = 30; // milliseconds
long previousMillis = 0;
long interval = 10;

void setup() 
{ 

  Serial.begin(9600);
  Serial.println("Starting StepperTest");
  
  delay(2000);

  pinMode(enablePin, OUTPUT); 
  pinMode(directionPin, OUTPUT);
  pinMode(stepPin, OUTPUT);  
  
  digitalWrite(enablePin, HIGH);
  digitalWrite(directionPin, HIGH);
  for(int n = 0; n < numberOfSteps; n++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(pulseWidthMicros);
    digitalWrite(stepPin, LOW);
    
    delay(millisbetweenSteps);
  }
  
  delay(3000);
  

  digitalWrite(directionPin, LOW);
  for(int n = 0; n < numberOfSteps; n++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(pulseWidthMicros);
    digitalWrite(stepPin, LOW);
    
    delay(millisbetweenSteps);
  
  }
  
  digitalWrite(directionPin, HIGH);
  
}
void loop() 
{ 
  
  unsigned long currentMillis = millis() * 10000; //1 ms is 1000us
  
    if(currentMillis - previousMillis > interval) { // interval is set to 10
    
    previousMillis = currentMillis;   

    digitalWrite(enablePin, HIGH);
    digitalWrite(directionPin, HIGH);
    for(int n = 0; n < numberOfSteps; n++) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(pulseWidthMicros);
      digitalWrite(stepPin, LOW);
      
      delay(millisbetweenSteps);
    }
    
    delay(3000);
    
  
    digitalWrite(directionPin, LOW);
    for(int n = 0; n < numberOfSteps; n++) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(pulseWidthMicros);
      digitalWrite(stepPin, LOW);
      
      delay(millisbetweenSteps);
    
    }
    
    digitalWrite(directionPin, HIGH);

  }
  
}
