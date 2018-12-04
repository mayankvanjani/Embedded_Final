#define TRIG PD2  // trigger pin
#define ECHO PD3  // echo output

// riseTime and fallTime will be modified by interrupt so we need to declare them as volatile
volatile unsigned long riseTime;  // timestamp when echo signal goes high
volatile unsigned long fallTime;  // timestamp when echo signal goes low
unsigned long pulseTime;          // difference between riseTime and fallTime
unsigned long distance;           // our range

float getDistance();
int speakerPin = 9;
int length = 15; // the number of notes
char notes[] = "ccggaagffeeddc "; // a space represents a rest
int beats[] = { 1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 2, 4 };
int tempo = 300;

void playTone(int tone, int duration) {
  for (long i = 0; i < duration * 1000L; i += tone * 2) {
    digitalWrite(speakerPin, HIGH);
    delayMicroseconds(tone);
    digitalWrite(speakerPin, LOW);
    delayMicroseconds(tone);
  }
}

void playNote(char note, int duration) {
  char names[] = { 'c', 'd', 'e', 'f', 'g', 'a', 'b', 'C' };
  int tones[] = { 1915, 1700, 1519, 1432, 1275, 1136, 1014, 956 };
  
  // play the tone corresponding to the note name
  for (int i = 0; i < 8; i++) {
    if (names[i] == note) {
      playTone(tones[i], duration);
    }
  }
}

void setup() 
{
  Serial.begin(9600);

  pinMode(speakerPin, OUTPUT);

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

  // Measures objects closer than half a meter
  if (range < 50) {
    playNote(notes[0], beats[0] * tempo);
  }
  else {
    playNote(notes[0], beats[0] * tempo);
    delay(1000);
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

