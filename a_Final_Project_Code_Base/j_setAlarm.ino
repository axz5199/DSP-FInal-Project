void setAlarm(int aCode, boolean isToneEn)
{
  if (isToneEn) //when tone enable is true
  {
    switch(aCode)
    {
      case 0: //System operational and normal breathing rate
      toneT1.play(0);
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
