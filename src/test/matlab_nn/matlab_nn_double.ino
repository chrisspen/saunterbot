#define numberSensorInputs 3
#define numberHiddenNeurons 5

double sensorValue[numberSensorInputs]= {0,0.4175,4.5925};
double hiddenLayerOut[numberHiddenNeurons];

double hiddenLayerWeights[numberHiddenNeurons][numberSensorInputs]=     {{38.498376246633022,4.8228789906421374,1.3602896899243053},{3.8474962878030912,-0.84371644817851899,-28.923549991793433},{-27.727414898404337,-11.923090632680966,19.537676225841405},{3.8569873599987994,-0.84731975744675336,0.17305115908902985},{0.29236102795668545,0.58525756760800285,-0.38214991270897147}};
double hiddenLayerBias[numberHiddenNeurons]= {39.11544288218294,-28.225689258527012,-16.517408802943045,0.8700678975676468,0.90940774428664883};

double outputLayerWeights[numberHiddenNeurons]= {-0.76894513764837624,-10.951307034874729,0.2457977459618178,10.887280941632419,4.3216425653566999};
double outputLayerBias = -2.8262415757548638;

double SumOfOutputNeuron(){
  double result = 0;

    for(int j=0; j<numberHiddenNeurons; j++){
      result = result+outputLayerWeights[j]*hiddenLayerOut[j];
      Serial.print("Output Neuron Product for ");Serial.print(j);Serial.print(" is ");Serial.println(result,16);
    }

  result = result+outputLayerBias;
  Serial.print("Output Neuron Sum with bias: ");Serial.println(result,16);
  return result;
}

double SumOfHiddenNeurons(int neuronNumber){
  double result = 0;

    for(int j=0; j<numberSensorInputs; j++){
      Serial.print("Hidden Neuron Weight for ");Serial.print(j);Serial.print(" is ");Serial.println(hiddenLayerWeights[neuronNumber][j],16);
      result = result+hiddenLayerWeights[neuronNumber][j]*sensorValue[j];
      Serial.print("Hidden Neuron Product for ");Serial.print(j);Serial.print(" is ");Serial.println(result,16);
    }

  result = result+hiddenLayerBias[neuronNumber];
  Serial.print("Hidden Neuron Sum with bias: ");Serial.println(result,16);
  return result;
}

/* Computing Log-Sigmoid Transfer Function*/
double ComputeTF(double neuronSum){
  double result = 0;
  result = (1.0/(1.0 + exp(-neuronSum)));
  Serial.print("Hidden Neuron output: ");Serial.println(result,16);
  return result;
}

void setup() {

  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
}

void loop() {
  // Parameter passed is the neuron number.
    for(int j=0; j<numberHiddenNeurons; j++){
      hiddenLayerOut[j] = ComputeTF(SumOfHiddenNeurons(j));
    }

    double sum = SumOfOutputNeuron();
    Serial.println(sum,16);
    Serial.println(sum);
    delay(20000);
}
