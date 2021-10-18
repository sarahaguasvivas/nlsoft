#define NUM_CHANNELS  11

void setup() {
  // put your setup code here, to run once:
  Serial.begin(2000000);
}

void loop() {
  // put your main code here, to run repeatedly:
  int data_buffer[NUM_CHANNELS];
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
  char to_send[128];
  sprintf(to_send, "<%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d>\n", data_buffer[0],
                    data_buffer[1], data_buffer[2], data_buffer[3], data_buffer[4],
                    data_buffer[5], data_buffer[6], data_buffer[7], data_buffer[8],
                    data_buffer[9], data_buffer[10]);
  Serial.print(to_send);             
}
