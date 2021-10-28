#ifdef DEBUGGING
  #define DEBUG(x)       Serial.print (x)
  #define DEBUGLN(x)     Serial.println (x)
#else
  #define DEBUG(x)
  #define DEBUGLN(x) 
#endif


#ifdef PRINT_DATA
  #define DATA(x)       Serial.print (x)
  #define DATALN(x)     Serial.println (x)
#else
  #define DATA(x)
  #define DATALN(x) 
#endif