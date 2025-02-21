// Define communication protocal
#define ACK_UNO 'A' 
#define SIG_PC 'S'

// Define state machine for serial communication
enum serial_states{IDLE, INIT, READY, STREAM};
enum serial_states serial_state;

// Define flag for event
bool PC_signal_flag = false;

// Debug pin
// hook on the oscilloscope to confirm the frequency
const int debugPin_idle = 2;
const int debugPin_init = 3;
const int debugPin_ready = 4;
const int debugPin_stream = 5;


void debug_pin_state()
{

  switch(serial_state)
  {
    case IDLE:
      digitalWrite(debugPin_idle, 1);
      digitalWrite(debugPin_init, 0);
      digitalWrite(debugPin_ready, 0);
      digitalWrite(debugPin_stream, 0);
      break;

    case INIT:
      digitalWrite(debugPin_idle, 0);
      digitalWrite(debugPin_init, 1);
      digitalWrite(debugPin_ready, 0);
      digitalWrite(debugPin_stream, 0);
      break;

    case READY:
      digitalWrite(debugPin_idle, 0);
      digitalWrite(debugPin_init, 0);
      digitalWrite(debugPin_ready, 1);
      digitalWrite(debugPin_stream, 0);
      break;

    case STREAM:
      digitalWrite(debugPin_idle, 0);
      digitalWrite(debugPin_init, 0);
      digitalWrite(debugPin_ready, 0);
      digitalWrite(debugPin_stream, 1);
      break;
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  // Serial.print("START");
  //set the state machine to INIT
  serial_state = IDLE;

  // set the digital pin as output:
  pinMode(debugPin_idle, OUTPUT);
  pinMode(debugPin_init, OUTPUT);
  pinMode(debugPin_ready, OUTPUT);
  pinMode(debugPin_stream, OUTPUT);
  digitalWrite(debugPin_idle, 0);
  digitalWrite(debugPin_init, 0);
  digitalWrite(debugPin_ready, 0);
  digitalWrite(debugPin_stream, 0);
}

void loop() {
  debug_pin_state();
  // put your main code here, to run repeatedly:
  //state transition
  switch(serial_state)
  {
    case IDLE:
      //transition the state to init state
      serial_state = INIT;
      break;
    case INIT:
      serial_state = READY;
      break;
    case READY:
      //if receive signal from PC, send state to Stream
      if(PC_signal_flag)
      {
        serial_state = STREAM;
        //reset signal flag
        PC_signal_flag = false;
      }
      else
      {
        //stay in the same state otherwise
        serial_state = READY;
      }
      break;
    case STREAM:
      serial_state = STREAM;
      break;
  }  
  //state behavior
  switch(serial_state)
  {
    case IDLE:
      // do nothing
      break;
    case INIT:
      // Serial.print("INIT");
      // Serial.println();
      // clear the RX buffer
      while(Serial.available()>0){
        // read the port until it's empty
        Serial.read();
      }
      //flush the output buffer
      Serial.flush();
      // send out initialize signal
      Serial.write('I');
      
      break;
    case READY:
      // Serial.print("READY");
      // Serial.println();
      // polling for the signal bit
      if (Serial.available())
      {
        if(Serial.read() == 'S')
        {
          // set the signal flag
          PC_signal_flag = true;
          // send out acknowledge byte when receive the signal
          Serial.print(ACK_UNO);

        }
      }
      
      break;
    case STREAM:
      if(Serial.read() == 'D')
      {
        // send out acknowledge byte
        Serial.print(ACK_UNO);
      }
      break;
  }
  
}
