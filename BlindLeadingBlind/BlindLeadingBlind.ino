/*
 * Embedded Project
 * Mayank Vanjani, Aulon Ibrahimi, Mohammed Quadir
 * Blind Leading the Blind
 * Arduino Blind Guidance Kit
 * Group 17: Team Iron Men
*/

// Arduion Pin Configuration for Hardwiring
#define TRIG PD2    // trigger pin 2
#define ECHO PD3    // echo output 3
#define TRIG2 PB4   // trigger pin 12
#define ECHO2 PB5   // echo output 13

int speakerPin = 9;    // Arduino Pin 9
int speakerPin2 = 10;  // Arduino Pin 10
int buzzerPin = 11;    // Arduino Pin 11

// Ultrasonic Sensor Variable Setup
// Ultrasonic Sensor: https://www.adafruit.com/product/3942
float getDistance();
volatile unsigned long riseTime;  // timestamp when echo signal goes high
volatile unsigned long fallTime;  // timestamp when echo signal goes low
unsigned long pulseTime;          // difference between riseTime and fallTime
unsigned long distance;           // range

float getDistance2();
volatile unsigned long riseTime2;  
volatile unsigned long fallTime2;  
unsigned long pulseTime2;          
unsigned long distance2;  

// Buzzer Activation Code for Varying Frequencies
// PiezoBuzzer: https://www.adafruit.com/product/160
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

  // LED Testing
  // pinMode(13, OUTPUT);
  
  pinMode(speakerPin, OUTPUT);
  pinMode(speakerPin2, OUTPUT);
  pinMode(buzzerPin, OUTPUT);


  // Setting direction for TRIG and ECHO pins
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
// Uses Buzzer Pin for haptic feedback
// Vibrating Disc: https://www.adafruit.com/product/1201
void loop() {

  // Seperate Ultrasonic Ranges
  float range = getDistance();
  float range2 = getDistance2();
  Serial.println(range);
  Serial.print("\t\t\t");
  Serial.println(range2);
  
  // Measures objects closer than 3 meters
  // playTone((range)*25, 100);
  
  if (range < 100) {
    // playTone((range+1), 100);
    playTone( (range+100) , 100);
  }
    
  if (range2 < 50) {
    digitalWrite(buzzerPin, HIGH);
    delay( (100*5) - (range2*5) );
    // delay(range2*5);
    digitalWrite(buzzerPin, LOW);
  }
    // delay(min(0,range-50));
   
}


// getDistance and PCI Interrupt Code for Ultrasonic Sensor provided by Dmytro Moyseyev (Embedded TA)
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

