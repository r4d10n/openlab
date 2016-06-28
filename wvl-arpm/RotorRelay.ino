// Relay Control code for PTS-306 1rpm AC Rotor

// Interfaced via 5v Relay board using 2 relays for Left and Right


const int leftPin = 12;
const int rightPin = 11;


void setup() {
  Serial.begin(9600);

  pinMode(leftPin, OUTPUT);
  pinMode(rightPin, OUTPUT);

  digitalWrite(leftPin, HIGH); 
  digitalWrite(rightPin, HIGH); 
}

void loop() {
  
  while (Serial.available() > 0)
  {

    Serial.println("Waiting for cmds....");
    
    String dir = Serial.readStringUntil(':');

    if (dir == "L")
    {      
      digitalWrite(leftPin, LOW);   
      String dur = Serial.readStringUntil(';');
      Serial.println("Left pin ON -- Dur:" + dur);
      int durInt = dur.toInt();
      delay(durInt);              
      digitalWrite(leftPin, HIGH); 
    }

    if (dir == "R")
    {
      
      digitalWrite(rightPin, LOW);  
      String dur = Serial.readStringUntil(';');
      Serial.println("Right pin ON -- Dur:" + dur);
      int durInt = dur.toInt();
      delay(durInt);              
      digitalWrite(rightPin, HIGH); 
    }
    
  }

}

