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
  readValue = ReadFromMATLAB();

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
   alarmCode = AlarmCheck( stdLF, stdMF, stdHF );

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
   
   printArray[2] = stdLF;       //  Column 3
   printArray[3] = stdMF;       //  Column 4, etc...
   printArray[4] = stdHF;
//   printArray[5] = stdLF;
//   printArray[6] = stdMF;
//   printArray[7] = stdHF;
   printArray[8] = float(alarmCode);

   numValues = 7;  // The number of columns to be sent to the serial monitor (or MATLAB)

 WriteToSerial( numValues, printArray );  //  Write to the serial monitor (or MATLAB)

  if (++loopTick >= NUM_SAMPLES){
    Serial.print("Average execution time (uSec) = ");Serial.println( float(execUsec)/NUM_SAMPLES );
    while(true); // spin forever
  }

} // loop()
