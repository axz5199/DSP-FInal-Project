int AlarmCheck( float stdLF, float stdMF, float stdHF)
{
//This function only checks if the system is operational and if so, it gives a go ahead
//and returns the value 1 to indicate the working operation
int retVal = 4;
float threshold = 0.0121;
float stdarr[] = {stdMF,stdLF,stdHF};
//  Your alarm check logic code will go here.
if(stdLF > threshold || stdMF > threshold || stdHF > threshold){
  int highest = stdarr[0];
  for(int i = 1; i < 3; i++)
    if (stdarr[i] > highest)
      highest = stdarr[i];
  if(stdLF == highest)
    retVal = 1;
  if(stdMF == highest)
    retVal = 0;
  if(stdHF == highest)
    retVal = 2;
  else
    retVal = 3;
}
return(retVal);
}  // end AlarmCheck
