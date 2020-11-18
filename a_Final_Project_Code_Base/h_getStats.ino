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
