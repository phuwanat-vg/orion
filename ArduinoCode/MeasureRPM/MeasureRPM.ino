const int encoder = 2; // encoder interrupt pin
volatile long encoderValue = 0; //variable for encoder pulse

int interval = 250;
long previousTime = 0;
long currentTime = 0;
int pulse_per_rev = 20;
int rpm = 0;
float rps = 0;
volatile unsigned long debounce=0;
void setup() {

   Serial.begin(115200);//Initialize the serial port
   attachInterrupt(digitalPinToInterrupt(3), count, CHANGE);
   encoderValue = 0;
   previousTime = millis();
}

void loop() {
  // Update RPM value on every second
  currentTime = millis();

  if (currentTime - previousTime > interval) {
    previousTime = currentTime;
   
    rpm = (float)(encoderValue * 60 / pulse_per_rev)*4;
    rps = (rpm * 2* 3.14159)/ 60*4;

    // Only update display when there have readings
    

      Serial.println(encoderValue);
      Serial.print(" pulse / ");
      Serial.print(rpm);
      Serial.println(" RPM");
      Serial.print(rps);
      Serial.println(" rad/s");
   encoderValue = 0;
    
    
  }
  
}


void count()
{
   unsigned long m = micros();
  if(m - debounce > 1500){
    //Update Count
    encoderValue=encoderValue+1;
    }
  debounce = m;
 
}
