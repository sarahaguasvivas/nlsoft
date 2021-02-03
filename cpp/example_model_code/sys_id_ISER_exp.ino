void buildLayers();
float * fwdNN(float*);

struct tm timeinfo;

void setup() {
  Serial.begin(115200);
  buildLayers();
}

void forward_prediction(int N){
  unsigned long elapsed = millis();
  float * output = (float*)malloc(3*sizeof(float));
  for(int i=2; i>=0; --i) output[i]=1.;
  
  for (int i = N-1; i>=0; --i){
    float * input = (float*)malloc(26*sizeof(float));
    for (int j=3; j<26; j++) input[j] = 1.0;
    for (int j= 0; j<3; j++) input[j] = output[j];
  
    output = fwdNN(input); 
    
  }
  free(output);
  Serial.println(millis()-elapsed);
}


void first_derivative(){
  unsigned long elapsed = millis();
  float eps = 0.008; 
  

  for (int i=0; i<26; i++){
      float * output = (float*)malloc(3*sizeof(float));
  float * output1 = (float*)malloc(3*sizeof(float));
  float * output_first_der = (float*)malloc(3*sizeof(float));
  
    float * input = (float*)malloc(26*sizeof(float));
    for (int j=0; j<26; j++) input[j] = 1.0;

    float * input1 = (float*)malloc(26*sizeof(float));
    for (int j=0; j<26; j++) input1[j] = 1.0;
  
    input[i] += eps;
    input1[i] -= eps;

    output = fwdNN(input); 
    output1 = fwdNN(input1); 
    for (int jj =0; jj< 3; jj++) output_first_der[jj] = (output[jj] - output1[jj])/(2*eps);

    
  free(output);
  free(output1);
  free(output_first_der);
  }

  Serial.println(millis()-elapsed);
  
}

void second_derivative(){
  unsigned long elapsed = millis();
  float eps = 0.008; 

  
  for (int i=0; i<26; i++){
      
  float * output_first_der = (float*)malloc(3*sizeof(float));
      float * output = (float*)malloc(3*sizeof(float));
      float * output1 = (float*)malloc(3*sizeof(float));
      float * output2 = (float*)malloc(3*sizeof(float));
    float * input = (float*)malloc(26*sizeof(float));
    for (int j=0; j<26; j++) input[j] = 1.0;

    float * input1 = (float*)malloc(26*sizeof(float));
    for (int j=0; j<26; j++) input1[j] = 1.0;

    float * input2 = (float*)malloc(26*sizeof(float));
    for (int j=0; j<26; j++) input2[j] = 1.0;
  
    input[i] += eps;
    input1[i] -= eps;

    output = fwdNN(input); 
    output1 = fwdNN(input1); 
    output2 = fwdNN(input2); 
    for (int jj =0; jj< 3; jj++) output_first_der[jj] = (output[jj] -2.*output2[jj] + output1[jj])/(eps * eps);
  free(output);
  free(output1);
  free(output2);
  free(output_first_der);
  }


  Serial.println(millis()-elapsed);
  
}

void both_derivatives(){
  unsigned long elapsed = millis();
  float eps = 0.008; 

  for (int i=0; i<26; i++){
      
  float * output_second_der = (float*)malloc(3*sizeof(float));
  float * output_first_der = (float*)malloc(3*sizeof(float));
      float * output = (float*)malloc(3*sizeof(float));
      float * output1 = (float*)malloc(3*sizeof(float));
      float * output2 = (float*)malloc(3*sizeof(float));
    float * input = (float*)malloc(26*sizeof(float));
    for (int j=0; j<26; j++) input[j] = 1.0;

    float * input1 = (float*)malloc(26*sizeof(float));
    for (int j=0; j<26; j++) input1[j] = 1.0;

    float * input2 = (float*)malloc(26*sizeof(float));
    for (int j=0; j<26; j++) input2[j] = 1.0;
  
    input[i] += eps;
    input1[i] -= eps;

    output = fwdNN(input); 
    output1 = fwdNN(input1); 
    output2 = fwdNN(input2); 
    for (int jj =0; jj< 3; jj++) output_second_der[jj] = (output[jj] -2.*output2[jj] + output1[jj])/(eps * eps);
    for (int jj =0; jj< 3; jj++) output_first_der[jj] = (output[jj] - output1[jj])/(2.*eps);
  free(output);
  free(output1);
  free(output2);
  free(output_first_der);
  free(output_second_der);
  }


  Serial.println(millis()-elapsed);
  
}


void loop() {
  
   // forward_prediction(3);
   //first_derivative();
   //second_derivative();
   both_derivatives();
}
