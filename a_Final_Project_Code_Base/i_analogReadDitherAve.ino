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
