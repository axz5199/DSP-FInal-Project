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
