// OPEN A NEW SKETCH WINDOW IN ARDUINO
// CLICK IN THIS BOX, CTL-A, CTL-C (Copy code from text box.)
// CLICK IN SKETCH, CTL-A, CTL-V (Paste code into sketch.)

// Breathing Rate Detection System -- Final Integration
//
// Pieced together from code created by: Clark Hochgraf and David Orlicki Oct 18, 2018
// Modified by: Mark Thompson April 2020 to integrate MATLAB read and Write
//              and integrate the system

#include <MsTimer2.h>
#include <SPI.h>
#include <Tone2.h>


const int TSAMP_MSEC = 100;
const int NUM_SAMPLES = 3600;  // 3600;
const int NUM_SUBSAMPLES = 160;
const int DAC0 = 3, DAC1 = 4, DAC2 = 5, LM61 = A0, VDITH = A1;
const int V_REF = 5.0;
const int SPKR = 12; // d12  PB4

volatile boolean sampleFlag = false;

const long DATA_FXPT = 1000; // Scale value to convert from float to fixed
const float INV_FXPT = 1.0 / DATA_FXPT; // division slow: precalculate


int nSmpl = 1, sample;

float xv, yv, yLF, yMF, yHF, stdLF, stdMF, stdHF, EqLp;
float yv_low, yv_high, yv_mid;
float printArray[9];
int numValues = 0;


int loopTick = 0;
bool statsReset;
bool isToneEn = false;

unsigned long startUsec, endUsec, execUsec;

//  Define a structure to hold statistics values for each filter band
struct stats_t
{
  int tick = 1;
  float mean, var, stdev;
} statsLF, statsMF, statsHF;

Tone toneT2;
Tone toneT1;

//**********************************************************************
void setup()
{

  configureArduino();
  Serial.begin(115200);delay(5);
  toneT2.begin(13);
  toneT1.begin(SPKR);

   //Handshake with MATLAB
  Serial.println(F("%Arduino Ready"));
  while (Serial.read() != 'g'); // spin

  MsTimer2::set(TSAMP_MSEC, ISR_Sample); // Set sample msec, ISR name
  MsTimer2::start(); // start running the Timer  

}


////**********************************************************************
void loop()
{

  // syncSample();  // Wait for the interupt when actually reading ADC data

 
  // Breathing Rate Detection

  // Declare variables

  float readValue, floatOutput;  //  Input data from ADC after dither averaging or from MATLAB
  long fxdInputValue, lpfInput, lpfOutput;  
  long eqOutput;  //  Equalizer output
  int alarmCode;  //  Alarm code


  // ******************************************************************
  //  When finding the impulse responses of the filters use this as an input
  //  Create a Delta function in time with the first sample a 1 and all others 0
  //xv = (loopTick == 0) ? 1.0 : 0.0; // impulse test input

  // ******************************************************************
  //  Use this when the test vector generator is used as an input
    xv = testVector();


  // ******************************************************************
  //  Read input value in ADC counts  -- Get simulated data from MATLAB
  //readValue = ReadFromMATLAB();

  // ******************************************************************
  //  Read input value from ADC using Dithering, and averaging
  //readValue = analogReadDitherAve();


  //  Convert the floating point number to a fixed point value.  First
  //  scale the floating point value by a number to increase its resolution
  //  (use DATA_FXPT).  Then round the value and truncate to a fixed point
  //  INT datatype

    fxdInputValue = long(DATA_FXPT * xv + 0.5);
 

  //  Execute the equalizer
    eqOutput = EqualizerFIR( fxdInputValue, loopTick );
//  Serial.println(eqOutput);
  //  Execute the noise filter.  
   lpfOutput = NoiseFilter( eqOutput, loopTick );
//Serial.println("lpfOutput");
  //  Convert the output of the equalizer by scaling floating point
  EqLp = float(lpfOutput) * INV_FXPT;
//Serial.println("EqLp");

  //*******************************************************************
  // Uncomment this when measuring execution times
  startUsec = micros();

  // ******************************************************************
  //  Compute the output of the filter using the cascaded SOS sections
   yv_low = IIR_Low_Pass(xv); // second order systems cascade  
   yv_high = IIR_High_Pass(xv);
   yv_mid = IIR_Band_Pass(xv);




  //  Compute the latest output of the running stats for the output of the filters.
  //  Pass the entire set of output values, the latest stats structure and the reset flag

 
    statsReset = (statsLF.tick%100 == 0);
   
    getStats( yv_low, statsLF, statsReset);
    stdLF = statsLF.stdev;

    getStats( yv_mid, statsMF, statsReset);
    stdMF = statsMF.stdev;

    getStats( yv_high, statsHF, statsReset);
    stdHF = statsHF.stdev;


  //*******************************************************************
  // Uncomment this when measuring execution times
   endUsec = micros();
   execUsec = execUsec + (endUsec-startUsec);

  //  Call the alarm check function to determine what breathing range
  //  alarmCode = AlarmCheck( stdLF, stdMF, stdHF );

  //  Call the alarm function to turn on or off the tone
  //setAlarm(alarmCode, isToneEn );

 
 // To print data to the serial port, use the WriteToSerial function.  
 //
 //  This is a generic way to print out variable number of values
 //
 //  There are two input arguments to the function:
 //  printArray -- An array of values that are to be printed starting with the first column
 //  numValues -- An integer indicating the number of values in the array.  
 
   printArray[0] = loopTick;  //  The sample number -- always print this
   printArray[1] = xv;        //  Column 2
   printArray[2] = EqLp;
   
//   printArray[2] = stdLF;       //  Column 3
//   printArray[3] = stdMF;       //  Column 4, etc...
//   printArray[4] = stdHF;
//   printArray[5] = stdLF;
//   printArray[6] = stdMF;
//   printArray[7] = stdHF;
//   printArray[8] = float(alarmCode);

   numValues = 3;  // The number of columns to be sent to the serial monitor (or MATLAB)

 WriteToSerial( numValues, printArray );  //  Write to the serial monitor (or MATLAB)

  if (++loopTick >= NUM_SAMPLES){
    Serial.print("Average execution time (uSec) = ");Serial.println( float(execUsec)/NUM_SAMPLES );
    while(true); // spin forever
  }

} // loop()

//******************************************************************
int AlarmCheck( float stdLF, float stdMF, float stdHF)
{
//This function only checks if the system is operational and if so, it gives a go ahead
//and returns the value 1 to indicate the working operation
int retVal = 4;
int threshold = 0;
float stdarr[] = {stdMF,stdLF,stdHF};
//  Your alarm check logic code will go here.
if(stdLF > threshold && stdMF > threshold && stdHF > threshold){
  int set = stdarr[0];
  for(int i = 1; i < 3; i++)
    if (stdarr[i] > set)
      set = stdarr[i];
  if(stdLF == set)
    retVal = 1;
  if(stdMF == set)
    retVal = 0;
  if(stdHF == set)
    retVal = 2;
  else
    retVal = 3;
}
return(retVal);
}  // end AlarmCheck

//*******************************************************************
int EqualizerFIR(long inputX, int sampleNumber)
{  
  // Starting with a generic FIR filter impelementation customize only by
  // changing the length of the filter using MFILT and the values of the
  // impulse response in h

  // Filter type: FIR
 
  //
  //  Set the constant HFXPT to the sum of the values of the impulse response
  //  This is to keep the gain of the impulse response at 1.
  //
  const int HFXPT = 1, MFILT = 4;
 
  int h[] = {1, 1, -1, -1};


 
  int i;
  const float INV_HFXPT = 1.0/HFXPT;
  static long xN[MFILT] = {inputX};
  long yOutput = 0;

  //
  // Right shift old xN values. Assign new inputX to xN[0];
  //
  for ( i = (MFILT-1); i > 0; i-- )
  {
    xN[i] = xN[i-1];
  }
   xN[0] = inputX;
 
  //
  // Convolve the input sequence with the impulse response
  //
 
  for ( i = 0; i < MFILT; i++)
  {
   
    // Explicitly cast the impulse value and the input value to LONGs then multiply
    // by the input value.  Sum up the output values
   
    yOutput = yOutput + long(h[i]) * long( xN[i] );
  }

  //  Return the output, but scale by 1/HFXPT to keep the gain to 1
  //  Then cast back to an integer
  //

  // Skip the first MFILT  samples to avoid the transient at the beginning due to end effects
  if (sampleNumber < MFILT ){
    return long(0);
  }else{
    return long(float(yOutput) * INV_HFXPT);
  }
}


//*******************************************************************************

int NoiseFilter(long inputX, int sampleNumber)
{  
  // Starting with a generic FIR filter impelementation customize only by
  // changing the length of the filter using MFILT and the values of the
  // impulse response in h

  // Filter type: FIR
 
  //
  //  Set the constant HFXPT to the sum of the values of the impulse response
  //  This is to keep the gain of the impulse response at 1.
  //
  // LPF FIR Filter Coefficients MFILT = 60, Fc = 85
  const int HFXPT = 4096, MFILT = 60;
  int h[] = {3, 1, -3, -5, -4, 1, 8, 11, 4, -10, -21, -18, 4, 31,
  40, 17, -30, -67, -57, 7, 89, 121, 57, -86, -215, -209, 9, 407,
  840, 1122, 1122, 840, 407, 9, -209, -215, -86, 57, 121, 89, 7, -57,
  -67, -30, 17, 40, 31, 4, -18, -21, -10, 4, 11, 8, 1, -4,
  -5, -3, 1, 3};


 
  int i;
  const float INV_HFXPT = 1.0/HFXPT;
  static long xN[MFILT] = {inputX};
  long yOutput = 0;

  //
  // Right shift old xN values. Assign new inputX to xN[0];
  //
  for ( i = (MFILT-1); i > 0; i-- )
  {
    xN[i] = xN[i-1];
  }
   xN[0] = inputX;
 
  //
  // Convolve the input sequence with the impulse response
  //
 
  for ( i = 0; i < MFILT; i++)
  {
   
    // Explicitly cast the impulse value and the input value to LONGs then multiply
    // by the input value.  Sum up the output values
   
    yOutput = yOutput + long(h[i]) * long( xN[i] );
  }

  //  Return the output, but scale by 1/HFXPT to keep the gain to 1
  //  Then cast back to an integer
  //

  // Skip the first MFILT  samples to avoid the transient at the beginning due to end effects
  if (sampleNumber < MFILT ){
    return long(0);
  }else{
    return long(float(yOutput) * INV_HFXPT);
  }
}


//*******************************************************************************
float IIR_Low_Pass(float xv)
{  


  //  ***  Copy variable declarations from MATLAB generator to here  ****

//Filter specific variable declarations
const int numStages = 3;
static float G[numStages];
static float b[numStages][3];
static float a[numStages][3];

//  *** Stop copying MATLAB variable declarations here
 
  int stage;
  int i;
  static float xM0[numStages] = {0.0}, xM1[numStages] = {0.0}, xM2[numStages] = {0.0};
  static float yM0[numStages] = {0.0}, yM1[numStages] = {0.0}, yM2[numStages] = {0.0};
 
  float yv = 0.0;
  unsigned long startTime;



//  ***  Copy variable initialization code from MATLAB generator to here  ****

// CHEBY LOW, order 5, R = 0.5, 12 BPM
G[0] = 0.0054630;
b[0][0] = 1.0000000; b[0][1] = 0.9990472; b[0][2]= 0.0000000;
a[0][0] = 1.0000000; a[0][1] =  -0.9554256; a[0][2] =  0.0000000;
G[1] = 0.0054630;
b[1][0] = 1.0000000; b[1][1] = 2.0015407; b[1][2]= 1.0015416;
a[1][0] = 1.0000000; a[1][1] =  -1.9217194; a[1][2] =  0.9289864;
G[2] = 0.0054630;
b[2][0] = 1.0000000; b[2][1] = 1.9994122; b[2][2]= 0.9994131;
a[2][0] = 1.0000000; a[2][1] =  -1.9562202; a[2][2] =  0.9723269;

//  **** Stop copying MATLAB code here  ****



  //  Iterate over each second order stage.  For each stage shift the input data
  //  buffer ( x[kk] ) by one and the output data buffer by one ( y[k] ).  Then bring in
  //  a new sample xv into the buffer;
  //
  //  Then execute the recusive filter on the buffer
  //
  //  y[k] = -a[2]*y[k-2] + -a[1]*y[k-1] + g*b[0]*x[k] + b[1]*x[k-1] + b[2]*x[k-2]
  //
  //  Pass the output from this stage to the next stage by setting the input
  //  variable to the next stage x to the output of the current stage y
  //  
  //  Repeat this for each second order stage of the filter

 
  for (i =0; i<numStages; i++)
    {
      yM2[i] = yM1[i]; yM1[i] = yM0[i];  xM2[i] = xM1[i]; xM1[i] = xM0[i], xM0[i] = G[i]*xv;
      yv = -a[i][2]*yM2[i] - a[i][1]*yM1[i] + b[i][2]*xM2[i] + b[i][1]*xM1[i] + b[i][0]*xM0[i];
      yM0[i] = yv;
      xv = yv;
    }
//
//  execUsec += micros()-startTime;
 
  return yv;
}

//*******************************************************************************
float IIR_High_Pass(float xv)
{  


  //  ***  Copy variable declarations from MATLAB generator to here  ****

//Filter specific variable declarations
const int numStages = 4;
static float G[numStages];
static float b[numStages][3];
static float a[numStages][3];

//  *** Stop copying MATLAB variable declarations here
 
  int stage;
  int i;
  static float xM0[numStages] = {0.0}, xM1[numStages] = {0.0}, xM2[numStages] = {0.0};
  static float yM0[numStages] = {0.0}, yM1[numStages] = {0.0}, yM2[numStages] = {0.0};
 
  float yv = 0.0;
  unsigned long startTime;



//  ***  Copy variable initialization code from MATLAB generator to here  ****
// CHEBY HIGH, order 7, R = 0.5, 40 BPM
G[0] = 0.7275541;
b[0][0] = 1.0000000; b[0][1] = -0.9927070; b[0][2]= 0.0000000;
a[0][0] = 1.0000000; a[0][1] =  -0.0930467; a[0][2] =  0.0000000;
G[1] = 0.7275541;
b[1][0] = 1.0000000; b[1][1] = -2.0134915; b[1][2]= 1.0135479;
a[1][0] = 1.0000000; a[1][1] =  -1.0509112; a[1][2] =  0.5059276;
G[2] = 0.7275541;
b[2][0] = 1.0000000; b[2][1] = -2.0030945; b[2][2]= 1.0031498;
a[2][0] = 1.0000000; a[2][1] =  -1.5993244; a[2][2] =  0.8280962;
G[3] = 0.7275541;
b[3][0] = 1.0000000; b[3][1] = -1.9907069; b[3][2]= 0.9907608;
a[3][0] = 1.0000000; a[3][1] =  -1.7888693; a[3][2] =  0.9553527;

//  **** Stop copying MATLAB code here  ****



  //  Iterate over each second order stage.  For each stage shift the input data
  //  buffer ( x[kk] ) by one and the output data buffer by one ( y[k] ).  Then bring in
  //  a new sample xv into the buffer;
  //
  //  Then execute the recusive filter on the buffer
  //
  //  y[k] = -a[2]*y[k-2] + -a[1]*y[k-1] + g*b[0]*x[k] + b[1]*x[k-1] + b[2]*x[k-2]
  //
  //  Pass the output from this stage to the next stage by setting the input
  //  variable to the next stage x to the output of the current stage y
  //  
  //  Repeat this for each second order stage of the filter

 
  for (i =0; i<numStages; i++)
    {
      yM2[i] = yM1[i]; yM1[i] = yM0[i];  xM2[i] = xM1[i]; xM1[i] = xM0[i], xM0[i] = G[i]*xv;
      yv = -a[i][2]*yM2[i] - a[i][1]*yM1[i] + b[i][2]*xM2[i] + b[i][1]*xM1[i] + b[i][0]*xM0[i];
      yM0[i] = yv;
      xv = yv;
    }
//
//  execUsec += micros()-startTime;
 
  return yv;
}

//*******************************************************************************
float IIR_Band_Pass(float xv)
{  


  //  ***  Copy variable declarations from MATLAB generator to here  ****

//Filter specific variable declarations
const int numStages = 6;
static float G[numStages];
static float b[numStages][3];
static float a[numStages][3];

//  *** Stop copying MATLAB variable declarations here
 
  int stage;
  int i;
  static float xM0[numStages] = {0.0}, xM1[numStages] = {0.0}, xM2[numStages] = {0.0};
  static float yM0[numStages] = {0.0}, yM1[numStages] = {0.0}, yM2[numStages] = {0.0};
 
  float yv = 0.0;
  unsigned long startTime;



//  ***  Copy variable initialization code from MATLAB generator to here  ****

// CHEBY BANDPASS, order 6, R = 0.50, [12 40] BPM
G[0] = 0.0954800;
b[0][0] = 1.0000000; b[0][1] = 2.0000079; b[0][2]= 1.0000024;
a[0][0] = 1.0000000; a[0][1] =  -1.8335247; a[0][2] =  0.9047451;
G[1] = 0.0954800;
b[1][0] = 1.0000000; b[1][1] = -2.0000077; b[1][2]= 1.0000016;
a[1][0] = 1.0000000; a[1][1] =  -1.7901940; a[1][2] =  0.9156656;
G[2] = 0.0954800;
b[2][0] = 1.0000000; b[2][1] = 2.0023544; b[2][2]= 1.0023599;
a[2][0] = 1.0000000; a[2][1] =  -1.8956434; a[2][2] =  0.9315688;
G[3] = 0.0954800;
b[3][0] = 1.0000000; b[3][1] = 1.9976377; b[3][2]= 0.9976433;
a[3][0] = 1.0000000; a[3][1] =  -1.9441684; a[3][2] =  0.9648152;
G[4] = 0.0954800;
b[4][0] = 1.0000000; b[4][1] = -2.0024658; b[4][2]= 1.0024719;
a[4][0] = 1.0000000; a[4][1] =  -1.7945581; a[4][2] =  0.9662035;
G[5] = 0.0954800;
b[5][0] = 1.0000000; b[5][1] = -1.9975265; b[5][2]= 0.9975326;
a[5][0] = 1.0000000; a[5][1] =  -1.9740647; a[5][2] =  0.9895962;

//  **** Stop copying MATLAB code here  ****



  //  Iterate over each second order stage.  For each stage shift the input data
  //  buffer ( x[kk] ) by one and the output data buffer by one ( y[k] ).  Then bring in
  //  a new sample xv into the buffer;
  //
  //  Then execute the recusive filter on the buffer
  //
  //  y[k] = -a[2]*y[k-2] + -a[1]*y[k-1] + g*b[0]*x[k] + b[1]*x[k-1] + b[2]*x[k-2]
  //
  //  Pass the output from this stage to the next stage by setting the input
  //  variable to the next stage x to the output of the current stage y
  //  
  //  Repeat this for each second order stage of the filter

 
  for (i =0; i<numStages; i++)
    {
      yM2[i] = yM1[i]; yM1[i] = yM0[i];  xM2[i] = xM1[i]; xM1[i] = xM0[i], xM0[i] = G[i]*xv;
      yv = -a[i][2]*yM2[i] - a[i][1]*yM1[i] + b[i][2]*xM2[i] + b[i][1]*xM1[i] + b[i][0]*xM0[i];
      yM0[i] = yv;
      xv = yv;
    }
//
//  execUsec += micros()-startTime;
 
  return yv;
}


//*******************************************************************
void getStats(float xv, stats_t &s, bool reset)
{
  float oldMean, oldVar;
 
  if (reset == true)
  {
    s.stdev = sqrt(s.var/s.tick);
    s.tick = 1;
    s.mean = xv;
    s.var = 0.0;  
  }
  else
  {
    oldMean = s.mean;
    s.mean = oldMean + (xv - oldMean)/(s.tick+1);
    oldVar = s.var;
    s.var = oldVar + (xv - oldMean)*(xv - s.mean);      
  }
  s.tick++;  
}

//*******************************************************************
float analogReadDitherAve(void)
{
 
float sum = 0.0;
int index;
  for (int i = 0; i < NUM_SUBSAMPLES; i++)
  {
    index = i;
    digitalWrite(DAC0, (index & B00000001)); // LSB bit mask
    digitalWrite(DAC1, (index & B00000010));
    digitalWrite(DAC2, (index & B00000100)); // MSB bit mask
    sum += analogRead(LM61);
  }
  return sum/NUM_SUBSAMPLES; // averaged subsamples

}

//*********************************************************************
void setAlarm(int aCode, boolean isToneEn)
{
  if (isToneEn) //when tone enable is true
  {
    switch(aCode)
    {
      case 0: //System operational and normal breathing rate
      //no sound
      break;

      case 1: //System operational and low breathing rate
      toneT1.play(400);
      break;

      case 2: //System operational and high breathing rate
      toneT1.play(800);
      break;

      default: //System operation by rate is undetermined OR System not operational
      toneT1.play(200);
    }
  }
  else //when tone enable is false
  {
    toneT1.stop(); //turn off speaker
  }
}
//*************************************************************
float testVector(void)
{
  // Variable rate sinusoidal input
  // Specify segment frequencies in bpm.
  // Test each frequency for nominally 60 seconds.
  // Adjust segment intervals for nearest integer cycle count.
   
  const int NUM_BAND = 6;
  const float CAL_FBPM = 10.0, CAL_AMP = 2.0;
 
  const float FBPM[NUM_BAND] = {5.0, 10.0, 15.0, 20.0, 30.0, 70.0}; // LPF test
  static float bandAmp[NUM_BAND] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

  //  Determine the number of samples (around 600 ) that will give you an even number
  //  of full cycles of the sinewave.  This is done to avoid a large discontinuity
  //  between bands.  This forces the sinewave in each band to end near a value of zer
 
  static int bandTick = int(int(FBPM[0]+0.5)*(600/FBPM[0]));
  static int simTick = 0, band = 0;
  static float Fc = FBPM[0]/600, cycleAmp = bandAmp[0];

  //for (int i = 0; i < NUM_BAND; i++) bandAmp[i] = CAL_AMP*(CAL_FBPM/FBPM[i]);  

  //  Check to see if the simulation tick has exceeded the number of tick in each band.
  //  If it has then switch to the next frequency (band) again computing how many
  //  ticks to go through to end up at the end of a cycle.
 
  if ((simTick >= bandTick) && (FBPM[band] > 0.0))
  {

    //  The simTick got to the end of the band cycle.  Go to the next frequency
    simTick = 0;
    band++;
    Fc = FBPM[band]/600.0;
    cycleAmp = bandAmp[band];
    bandTick = int(int(FBPM[band]+0.5)*(600/FBPM[band]));
  }
 
  float degC = 0.0; // DC offset
  degC += cycleAmp*sin(TWO_PI*Fc*simTick++);  
  //degC += 1.0*(tick/100.0); // drift: degC / 10sec
  //degC += 0.1*((random(0,101)-50.0)/29.0); // stdev scaled from 1.0
  return degC;
}

//*******************************************************************
void configureArduino(void)
{
  pinMode(DAC0,OUTPUT); digitalWrite(DAC0,LOW);
  pinMode(DAC1,OUTPUT); digitalWrite(DAC1,LOW);
  pinMode(DAC2,OUTPUT); digitalWrite(DAC2,LOW);

  pinMode(SPKR, OUTPUT); digitalWrite(SPKR,LOW);


  analogReference(DEFAULT); // DEFAULT, INTERNAL
  analogRead(LM61); // read and discard to prime ADC registers
  Serial.begin(115200); // 11 char/msec
}


//**********************************************************************
void WriteToSerial( int numValues, float dataArray[] )
{

  int index=0;
  for (index = 0; index < numValues; index++)
  {
    if (index >0)
    {
      Serial.print('\t');
    }
      Serial.print(dataArray[index], DEC);
  }

  Serial.print('\n');
  delay(20);

}  // end WriteToMATLAB

////**********************************************************************
float ReadFromMATLAB()
{
  int charCount;
  bool readComplete = false;
  char inputString[80], inChar;


  // Wait for the serial port

  readComplete = false;
  charCount = 0;
  while ( !readComplete )
  {
    while ( Serial.available() <= 0);
    inChar = Serial.read();

    if ( inChar == '\n' )
    {
      readComplete = true;
    }
    else
    {
      inputString[charCount++] = inChar;
    }
  }
  inputString[charCount] = 0;
  return atof(inputString);

} // end ReadFromMATLAB

//*******************************************************************
void syncSample(void)
{
  while (sampleFlag == false); // spin until ISR trigger
  sampleFlag = false;          // disarm flag: enforce dwell  
}

//**********************************************************************
void ISR_Sample()
{
  sampleFlag = true;
}

//******************************************************************

