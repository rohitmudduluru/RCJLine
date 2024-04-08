
#define MSG_LENGTH 50
char msg[MSG_LENGTH];

void read(){
  int i = 0;
  char c = 100;
  while(!Serial2.available());
  while (Serial2.available()) {
    c = Serial2.read();
    msg[i] = c;
    i++;
    delay(5);
  }
}
void write(char message[]){
  for(int i = 0; i < strlen(message); i++){
    Serial2.write(message[i]);
    delay(5);
  }
}