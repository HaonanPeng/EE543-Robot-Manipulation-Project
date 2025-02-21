void setup() {
  Serial.begin(9600);
  Serial.print("Controller begin");
  Serial.println();
  // put your setup code here, to run once:
  // pinMode(13, OUTPUT);        //Set the pin to be OUTPUT
  cli();                      //stop interrupts for till we make the settings
  /*1. First we reset the control register to amke sure we start with everything disabled.*/
  TCCR1A = 0;                 // Reset entire TCCR1A to 0 
  TCCR1B = 0;                 // Reset entire TCCR1B to 0
 
  /*2. We set the prescalar to the desired value by changing the CS10 CS12 and CS12 bits. */  
  TCCR1B |= B00000100;        //Set CS12 to 1 so we get prescalar 256  
  
  /*3. We enable compare match mode on register A*/
  TIMSK1 |= B00000010;        //Set OCIE1A to 1 so we enable compare match A 
  
  /*4. Set the value of register A to 31250, about 0.5 second*/
  OCR1A = 31250;             //Finally we set compare register A to this value  
  sei();                     //Enable back the interrupts

}

void loop() {
  // put your main code here, to run repeatedly:
  //reset counter
  // TCNT1 &= 0;

  delay(3000);
  //disable timer
  TCCR1B = 0;
  delay(3000);
  //re-enable timer
  TCCR1B |= B00000100;
}

//With the settings above, this IRS will trigger each 500ms.
ISR(TIMER1_COMPA_vect){
  TCNT1  = 0;                  //First, set the timer back to 0 so it resets for next interrupt
  Serial.print("Timer interrupt");
  Serial.println();
}
