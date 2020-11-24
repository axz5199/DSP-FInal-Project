int AlarmCheck( float stdLF, float stdMF, float stdHF)
{
  int code = 4;
  float threshold = 0.05;
  float thresLow = 0.03;
  float thresMed = 0.1;
  float thresHigh = 0.15;

  if (stdLF > thresLow || stdMF > thresMed || stdHF > thresHigh) //if system is operational
  {
    if ((stdLF > stdMF) and (stdLF > stdHF)) //if low rate is the greatest
    {
      code = 1;
    }
    else if ((stdMF > stdLF) and (stdMF > stdHF)) //if mid rate is the greatest
    {
      code = 0;
    }
    else if ((stdHF > stdLF) and (stdHF > stdMF)) //if high rate is the greatest
    {
      code = 2;
    }
    else //operation rate is undetermined
    {
      code = 3;
    }
  }

  return code;
}
