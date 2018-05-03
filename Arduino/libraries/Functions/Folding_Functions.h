
//////////////////////////////////////////////////////////////////////////////////////

// Folding Functions for Wings
void openWings(){
  digitalWrite(Wing_R_dir, HIGH);
  digitalWrite(Wing_L_dir, HIGH);
  for(int i=0; i<steps_Wings ; i++){
    digitalWrite(Wing_R_step, HIGH);
    digitalWrite(Wing_L_step, HIGH);
    delayMicroseconds(stepTime_Wings);
    digitalWrite(Wing_R_step, LOW);
    digitalWrite(Wing_L_step, LOW);
    delay(stepDelay_Wings);
  } 
}
void openWing_R(){
  digitalWrite(Wing_R_dir, HIGH);
  for(int i=0; i<steps_Wings ; i++){
    digitalWrite(Wing_R_step, HIGH);
    delayMicroseconds(stepTime_Wings);
    digitalWrite(Wing_R_step, LOW);
    delay(stepDelay_Wings);
  } 
}
void openWing_L(){
  digitalWrite(Wing_L_dir, HIGH);
  for(int i=0; i<steps_Wings ; i++){
    digitalWrite(Wing_L_step, HIGH);
    delayMicroseconds(stepTime_Wings);
    digitalWrite(Wing_L_step, LOW);
    delay(stepDelay_Wings);
  } 
}
void closeWings(){
  digitalWrite(Wing_R_dir, LOW);
  digitalWrite(Wing_L_dir, LOW);
  for(int i=0; i<steps_Wings ; i++){
    digitalWrite(Wing_R_step, HIGH);
    digitalWrite(Wing_L_step, HIGH);
    delayMicroseconds(stepTime_Wings);
    digitalWrite(Wing_R_step, LOW);
    digitalWrite(Wing_L_step, LOW);
    delay(stepDelay_Wings);
  } 
}
void closeWing_R(){
  digitalWrite(Wing_R_dir, LOW);
  for(int i=0; i<steps_Wings ; i++){
    digitalWrite(Wing_R_step, HIGH);
    delayMicroseconds(stepTime_Wings);
    digitalWrite(Wing_R_step, LOW);
    delay(stepDelay_Wings);
  } 
}
void closeWing_L(){
  digitalWrite(Wing_L_dir, LOW);
  for(int i=0; i<steps_Wings ; i++){
    digitalWrite(Wing_L_step, HIGH);
    delayMicroseconds(stepTime_Wings);
    digitalWrite(Wing_L_step, LOW);
    delay(stepDelay_Wings);
  } 
}


// Folding Functions for Seating plates
void foldSeats_up(){
  digitalWrite(Seat_R_dir, HIGH);
  digitalWrite(Seat_M_dir, HIGH);
  digitalWrite(Seat_L_dir, HIGH);
  for(int i=0; i<steps_Seats ; i++){
    digitalWrite(Seat_R_step, HIGH);
    digitalWrite(Seat_M_step, HIGH);
    digitalWrite(Seat_L_step, HIGH);
    delayMicroseconds(stepTime_Seats);
    digitalWrite(Seat_R_step, LOW);
    digitalWrite(Seat_M_step, LOW);
    digitalWrite(Seat_L_step, LOW);
    delay(stepDelay_Seats);
  } 
}
void foldSeats_ML_up(){
  digitalWrite(Seat_M_dir, HIGH);
  digitalWrite(Seat_L_dir, HIGH);
  for(int i=0; i<steps_Seats ; i++){
    digitalWrite(Seat_M_step, HIGH);
    digitalWrite(Seat_L_step, HIGH);
    delayMicroseconds(stepTime_Seats);
    digitalWrite(Seat_M_step, LOW);
    digitalWrite(Seat_L_step, LOW);
    delay(stepDelay_Seats);
  } 
}
void foldSeats_RL_up(){
  digitalWrite(Seat_R_dir, HIGH);
  digitalWrite(Seat_L_dir, HIGH);
  for(int i=0; i<steps_Seats ; i++){
    digitalWrite(Seat_R_step, HIGH);
    digitalWrite(Seat_L_step, HIGH);
    delayMicroseconds(stepTime_Seats);
    digitalWrite(Seat_R_step, LOW);
    digitalWrite(Seat_L_step, LOW);
    delay(stepDelay_Seats);
  } 
}
void foldSeats_MR_up(){
  digitalWrite(Seat_M_dir, HIGH);
  digitalWrite(Seat_R_dir, HIGH);
  for(int i=0; i<steps_Seats ; i++){
    digitalWrite(Seat_M_step, HIGH);
    digitalWrite(Seat_R_step, HIGH);
    delayMicroseconds(stepTime_Seats);
    digitalWrite(Seat_M_step, LOW);
    digitalWrite(Seat_R_step, LOW);
    delay(stepDelay_Seats);
  } 
}
void foldSeats_L_up(){
  digitalWrite(Seat_L_dir, HIGH);
  for(int i=0; i<steps_Seats ; i++){
    digitalWrite(Seat_L_step, HIGH);
    delayMicroseconds(stepTime_Seats);
    digitalWrite(Seat_L_step, LOW);
    delay(stepDelay_Seats);
  } 
}
void foldSeats_M_up(){
  digitalWrite(Seat_M_dir, HIGH);
  for(int i=0; i<steps_Seats ; i++){
    digitalWrite(Seat_M_step, HIGH);
    delayMicroseconds(stepTime_Seats);
    digitalWrite(Seat_M_step, LOW);
    delay(stepDelay_Seats);
  } 
}
void foldSeats_R_up(){
  digitalWrite(Seat_R_dir, HIGH);
  for(int i=0; i<steps_Seats ; i++){
    digitalWrite(Seat_R_step, HIGH);
    delayMicroseconds(stepTime_Seats);
    digitalWrite(Seat_R_step, LOW);
    delay(stepDelay_Seats);
  } 
}
void foldSeats_down(){
  digitalWrite(Seat_R_dir, LOW);
  digitalWrite(Seat_M_dir, LOW);
  digitalWrite(Seat_L_dir, LOW);
  for(int i=0; i<steps_Seats ; i++){
    digitalWrite(Seat_R_step, HIGH);
    digitalWrite(Seat_M_step, HIGH);
    digitalWrite(Seat_L_step, HIGH);
    delayMicroseconds(stepTime_Seats);
    digitalWrite(Seat_R_step, LOW);
    digitalWrite(Seat_M_step, LOW);
    digitalWrite(Seat_L_step, LOW);
    delay(stepDelay_Seats);
  } 
}
void foldSeats_ML_down(){
  digitalWrite(Seat_M_dir, LOW);
  digitalWrite(Seat_L_dir, LOW);
  for(int i=0; i<steps_Seats ; i++){
    digitalWrite(Seat_M_step, HIGH);
    digitalWrite(Seat_L_step, HIGH);
    delayMicroseconds(stepTime_Seats);
    digitalWrite(Seat_M_step, LOW);
    digitalWrite(Seat_L_step, LOW);
    delay(stepDelay_Seats);
  } 
}
void foldSeats_RL_down(){
  digitalWrite(Seat_R_dir, LOW);
  digitalWrite(Seat_L_dir, LOW);
  for(int i=0; i<steps_Seats ; i++){
    digitalWrite(Seat_R_step, HIGH);
    digitalWrite(Seat_L_step, HIGH);
    delayMicroseconds(stepTime_Seats);
    digitalWrite(Seat_R_step, LOW);
    digitalWrite(Seat_L_step, LOW);
    delay(stepDelay_Seats);
  } 
}
void foldSeats_MR_down(){
  digitalWrite(Seat_M_dir, LOW);
  digitalWrite(Seat_L_dir, LOW);
  for(int i=0; i<steps_Seats ; i++){
    digitalWrite(Seat_M_step, HIGH);
    digitalWrite(Seat_R_step, HIGH);
    delayMicroseconds(stepTime_Seats);
    digitalWrite(Seat_M_step, LOW);
    digitalWrite(Seat_R_step, LOW);
    delay(stepDelay_Seats);
  } 
}
void foldSeats_L_down(){
  digitalWrite(Seat_L_dir, LOW);
  for(int i=0; i<steps_Seats ; i++){
    digitalWrite(Seat_L_step, HIGH);
    delayMicroseconds(stepTime_Seats);
    digitalWrite(Seat_L_step, LOW);
    delay(stepDelay_Seats);
  } 
}
void foldSeats_M_down(){
  digitalWrite(Seat_M_dir, LOW);
  for(int i=0; i<steps_Seats ; i++){
    digitalWrite(Seat_M_step, HIGH);
    delayMicroseconds(stepTime_Seats);
    digitalWrite(Seat_M_step, LOW);
    delay(stepDelay_Seats);
  } 
}
void foldSeats_R_down(){
  digitalWrite(Seat_R_dir, LOW);
  for(int i=0; i<steps_Seats ; i++){
    digitalWrite(Seat_R_step, HIGH);
    delayMicroseconds(stepTime_Seats);
    digitalWrite(Seat_R_step, LOW);
    delay(stepDelay_Seats);
  } 
}



void do_Wings(int arg1, int arg2){ 
  if(arg1==2){
    if(arg2==1){
      openWings();
    }
    else if(arg2==0){
      closeWings();
    }      
  }
  else if(arg1==1){
    if(arg2==1){
      openWing_L();
    }
    else if(arg2==0){
      closeWing_L();
    }
  }
  else if(arg1==0){
    if(arg2==1){
      openWing_R();
    }
    else if(arg2==0){
      closeWing_R();
    }
  }
}

void do_Seats(int arg1, int arg2){
  if(arg1==3){
    if(arg2==1){
      foldSeats_up();
    }
    else if(arg2==0){
      foldSeats_down();
    }      
  }
  else if(arg1==2){
    if(arg2==1){
      foldSeats_L_up();
    }
    else if(arg2==0){
      foldSeats_L_down();
    }
  }
  else if(arg1==1){
    if(arg2==1){
      foldSeats_M_up();
    }
    else if(arg2==0){
      foldSeats_M_down();
    }
  }
  else if(arg1==0){
    if(arg2==1){
      foldSeats_R_up();
    }
    else if(arg2==0){
      foldSeats_R_down();
    }
  }
}
