#define NUM_CHANNELS  11

void setup() {
  // put your setup code here, to run once:
  Serial1.begin(2000000);
}

void loop() {
  // put your main code here, to run repeatedly:
  String buffer_string = get_buffer_string();
  Serial1.println(buffer_string);
}

String get_buffer_string(){
  float data_buffer[11];
  String buffer_string = "<";
  
  data_buffer[0] = analogRead(A0);
  data_buffer[1] = analogRead(A1);
  data_buffer[2] = analogRead(A2);
  data_buffer[3] = analogRead(A3);
  data_buffer[4] = analogRead(A4);
  data_buffer[5] = analogRead(A5);
  data_buffer[6] = analogRead(A6);
  data_buffer[7] = analogRead(A7);
  data_buffer[8] = analogRead(A8);
  data_buffer[9] = analogRead(A9);
  data_buffer[10]= analogRead(A10);
  
  for (int i=0; i< NUM_CHANNELS; i++){
    buffer_string+= String((int)data_buffer[i]);
    buffer_string += ",";
  }
  buffer_string+= String((int)data_buffer[10]);
  buffer_string+= ">";

  return buffer_string;
  
}
