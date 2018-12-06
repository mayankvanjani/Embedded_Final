#define TRIG PD2  // trigger pin
#define ECHO PD3  // echo output

// riseTime and fallTime will be modified by interrupt so we need to declare them as volatile
volatile unsigned long riseTime;  // timestamp when echo signal goes high
volatile unsigned long fallTime;  // timestamp when echo signal goes low
unsigned long pulseTime;          // difference between riseTime and fallTime
unsigned long distance;           // our range

float getDistance();
int speakerPin = 9;
int speakerPin2 = 10;
int length = 15; // the number of notes
char notes[] = "ccggaagffeeddc "; // a space represents a rest
int beats[] = { 1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 2, 4 };
int tempo = 300;

void playTone(int tone, int duration) {
  for (long i = 0; i < duration * 1000L; i += tone * 2) {
    digitalWrite(speakerPin, HIGH);
    digitalWrite(speakerPin2, HIGH);
    delayMicroseconds(tone);
    digitalWrite(speakerPin, LOW);
    digitalWrite(speakerPin2, LOW);
    delayMicroseconds(tone);
  }
}


void setup() 
{
  Serial.begin(9600);

  pinMode(speakerPin, OUTPUT);
  pinMode(speakerPin2, OUTPUT);

  // Setting direction for trig and echo pins
  DDRD |= (1<<TRIG);
  DDRD &= ~(1<<ECHO);

  // Setting up pin change interrupt for echo (PD3), PCINT19
  PCICR = (1<<PCIE2); // only enabling PCIE2, because PCIE2 constains PCINT[23:16]
  PCMSK2 = (1<<PCINT19); // only enabling PCINT19  
  
}


void loop() 
{
  float range = getDistance();
  Serial.println(range);

  // Measures objects closer than 2 meters
  if (range < 200) {
     playTone(1915, range);
     delay(range*5);
  }
}

// function to calculate the distance
float getDistance()
{
  // clear trig pin
  PORTD &= ~(1<<TRIG);
  delayMicroseconds(2);
  
  // send out 10 us pulse
  PORTD = (1<<TRIG);
  delayMicroseconds(10);
  PORTD &= ~(1<<TRIG);

  // sound travels at 343 m/s which is 0.0343 cm/us
  // distance = time*velocity
  // we need to divide result by 2 because sound travels 
  // to object and back so the incoming pulse is twice longer
  distance = pulseTime*0.0343/2; // result in cm 
  return distance;
  
}



// Interrupt service vector for pin change:
// ISR (PCINT0_vect) pin change interrupt for D8 to D13
// ISR (PCINT1_vect) pin change interrupt for A0 to A5
// ISR (PCINT2_vect) pin change interrupt for D0 to D7
// We need PCINT2_vect because we are using PD3 as input (echo)

ISR(PCINT2_vect)
{  
  if ((PIND & (1<<ECHO)) == (1<<ECHO)) // check if pin PD3 is high
    { // Measure time when echo pin goes high
      riseTime = micros(); // micros() calculates the run time in us since the program's start
    }    
  else 
    {
      // measure the time when echo pin goes low
      fallTime = micros();
      pulseTime = fallTime - riseTime; // this is our echo pulse, its length is proportional to the measured distance
    }
}

