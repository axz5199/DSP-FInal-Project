int AlarmCheck( float stdLF, float stdMF, float stdHF)
{
  int retVal = 4;
  float threshold = 0.0121;
  float stdarr[] = {stdMF,stdLF,stdHF};

  if(stdLF > threshold || stdMF > threshold || stdHF > threshold)
  {
    int highest = stdarr[0];
    
    for(int i = 1; i < 3; i++)
    {
      if (stdarr[i] > highest) { highest = stdarr[i]; }
    }
    if(stdLF == highest) { retVal = 1; }
    if(stdMF == highest) { retVal = 0; }
    if(stdHF == highest) { retVal = 2; }
    else { retVal = 3; }
  }
  
  return(retVal);
}
