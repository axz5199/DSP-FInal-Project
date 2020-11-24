int AlarmCheck( float stdLF, float stdMF, float stdHF)
{
  int code = 4;
  float threshold = 0.0121;

  if (stdLF > threshold || stdMF > threshold || stdHF > threshold) //if system is operational
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
