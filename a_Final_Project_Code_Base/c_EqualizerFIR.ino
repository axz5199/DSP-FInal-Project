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
