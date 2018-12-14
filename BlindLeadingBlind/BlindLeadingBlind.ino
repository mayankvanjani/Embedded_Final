#define TRIG PD2    // trigger pin 2
#define ECHO PD3    // echo output 3
#define TRIG2 PB4   // trigger pin 12
#define ECHO2 PB5   // echo output 13

volatile unsigned long riseTime;  // timestamp when echo signal goes high
volatile unsigned long fallTime;  // timestamp when echo signal goes low
unsigned long pulseTime;          // difference between riseTime and fallTime
unsigned long distance;           // range

volatile unsigned long riseTime2;  
volatile unsigned long fallTime2;  
unsigned long pulseTime2;          
unsigned long distance2;  

float getDistance();
float getDistance2();
int speakerPin = 9;
int speakerPin2 = 10;
int buzzerPin = 11;

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



// ---------- SETUP ---------- //
void setup() {
  Serial.begin(9600);

  pinMode(13, OUTPUT);
  
  pinMode(speakerPin, OUTPUT);
  pinMode(speakerPin2, OUTPUT);
  pinMode(buzzerPin, OUTPUT);

  // Setting direction for trig and echo pins
  DDRD |= (1<<TRIG);
  DDRD &= ~(1<<ECHO);

  DDRB |= (1<<TRIG2);
  DDRB &= ~(1<<ECHO2);

  // Setting up pin change interrupt for echo (PD3), PCINT19
  PCICR |= (1<<PCIE2);    // only enabling PCIE2, because PCIE2 constains PCINT[23:16]
  PCMSK2 = (1<<PCINT19); // only enabling PCINT19  

  PCICR |= (1<<PCIE0);    // only enabling PCIE0, because PCIE0 constains PCINT[7:0]
  PCMSK0 = (1<<PCINT5);
}



// ---------- LOOP ---------- //
void loop() {

  float range = getDistance();
  float range2 = getDistance2();
  Serial.print(range);
  Serial.print("\t\t\t");
  Serial.println(range2);
  
  // Measures objects closer than 3 meters
  
  if (range < 300) {
    playTone(1915, range);
    delay(range*5);
  }
  
  if (range2 < 50) {
    // digitalWrite(13, HIGH);
    digitalWrite(buzzerPin, HIGH);
    delay( (50*5) - (range2*5) );
    digitalWrite(buzzerPin, LOW);
    // digitalWrite(13, LOW);
  }
  
  else {
    digitalWrite(buzzerPin, LOW);
    digitalWrite(13, HIGH);
  }
  
  /*
  if (range < 50) {
    playTone(1915, range);
    digitalWrite(buzzerPin, HIGH);
    delay(range*5);
    digitalWrite(buzzerPin, LOW);
  }
    
  if (range < 300) {
    playTone(1915, range);
    delay(range*5);
  }
  */
}



float getDistance() {
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

float getDistance2() {
  PORTB &= ~(1<<TRIG2);
  delayMicroseconds(2);
  
  PORTB = (1<<TRIG2);
  delayMicroseconds(10);
  PORTB &= ~(1<<TRIG2);
  
  distance2 = pulseTime2*0.0343/2; // result in cm 
  return distance2;
}





// Interrupt service vector for pin change:
// ISR (PCINT0_vect) pin change interrupt for D8 to D13
// ISR (PCINT1_vect) pin change interrupt for A0 to A5
// ISR (PCINT2_vect) pin change interrupt for D0 to D7
// We need PCINT2/0_vect because we are using PD3 and PB5 as input (echo)

ISR(PCINT2_vect) {
 
  if ( (PIND & (1<<ECHO)) == (1<<ECHO) ) {
        riseTime = micros(); // micros() calculates the run time in us since the program's start
  }
  
  else {
      // measure the time when echo pin goes low
      fallTime = micros();
      pulseTime = fallTime - riseTime; // this is our echo pulse, its length is proportional to the measured distance
    }
    
}

ISR(PCINT0_vect) { 
   
  if ( (PINB & (1<<ECHO2)) == (1<<ECHO2) ) {
    riseTime2 = micros(); // micros() calculates the run time in us since the program's start
  }
  
  else {
    fallTime2 = micros();
    pulseTime2 = fallTime2 - riseTime2; // this is our echo pulse, its length is proportional to the measured distance
  }
  
}

