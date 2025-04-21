void redLEDon() {
  digitalWrite(redledside,HIGH);
  digitalWrite(redledtop,HIGH);
}

void blueLEDon(){
  digitalWrite(blueledside,HIGH);
  digitalWrite(blueledtop,HIGH);
}

void redLEDoff() {
  digitalWrite(redledside,LOW);
  digitalWrite(redledtop,LOW);
}

void blueLEDoff(){
  digitalWrite(blueledside,LOW);
  digitalWrite(blueledtop,LOW);
}