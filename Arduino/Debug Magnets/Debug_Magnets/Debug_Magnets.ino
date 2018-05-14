int magnets = 45;


void setup() {
  pinMode(magnets,OUTPUT); 
  Serial.begin(38400);
  Serial.println("Default: magnets active (1)");
  Serial.println("Type 1 to activate magnets and 0 to deactivate magnets");
  
}

void loop() {
  int input;
  if(Serial.available()){
    input = (Serial.read());
    if(input=='1'){
      activate_Magnets();
      Serial.println("Magnets have been activated");
    }
    else if(input=='0'){
      deactivate_Magnets();
      Serial.println("Magnets have been deactivated");
    }
    else{
      Serial.println("Type 1 or 0");
    }
  }
  
}
bool activate_Magnets(){
  digitalWrite(magnets,LOW);
  return true;
}
bool deactivate_Magnets(){
  digitalWrite(magnets,HIGH);
  return false;
}

