/*
  Cod pentru aterizarea dronei autonoma
  Acesta cod are 4 functii principale
  1. Citeste 6 canale PWM (Pulse Width Modulation) de la receiverul FS-IA6b
  2. Combina semnalele PWM sub forma de PPM (Pulse Position Modulation)
  si il trimite catre OMNIBUS F3 (controlerul de zbor)
  3. Citeste datele camerei video Pixy - Primeste valorile x,y si inaltimea suprafetei de aterizat
  4. Algoritm de aterizare autonoma cu ajutorul PID
*/

#include <SPI.h> // include libraria Serial Peripheral Interface (SPI) pentru a activa portul SPI
#include <Pixy2.h> // include libraria Pixy pentru a activa comunicarea cu Pixy
#include <PinChangeInterrupt.h> // include libraria pentru a citi valorile PWM
#include <PID_v1.h> // include libraria PID (proportional, integral and derivative)


#define CHANNEL_NUMBER 6  //seteaza numarul canalelor
#define CHANNEL_DEFAULT_VALUE 1500  //seteaza valoarea initiala PWM
#define SWITCH_OFF_VALUE 900 // seteaza valoarea implicita pentru pentru "oprit"
#define SWITCH_ON_VALUE 1800 // seteaza valoarea implicita pentru pentru "pornit"
#define FRAME_LENGTH 15000  //seteaza lungimea cardului PPM in mocrosecunde (1ms = 1000Âµs)
#define PULSE_LENGTH 300  //seteaza lungimea dintre fiecare puls PPM
#define THROTTLE_LANDING_VALUE 900 // seteaza valoarea de aterizare
#define LANDING_HEIGHT_VALUE 50 // seteaza inaltimea dorita a suprafetei de aterizat
#define LANDING_WIDTH_VALUE 50 // seteaza latimea dorita a suprafetei de aterizat
#define LANDING_SIZE_VALUE LANDING_WIDTH_VALUE * LANDING_HEIGHT_VALUE // seteaza marimea dorita a suprafetei de aterizat
#define onState 1  //seteaza polaritatea pulsurilor
#define sigPin 8  //seteaza pinul pentru iesirea PPM
#define LANDING_X_VALUE 10 // seteaza aria dorita de aterizat X
#define LANDING_Y_VALUE 10 // seteaza aria dorita de aterizat Y
#define TURNING_SPEED 70 // seteaza viteza maxima de miscare a dronei
#define LANDING_SPEED 2 // seteaza viteza maxima de coborare a dronei
#define X_CENTER (319/2) // seteaza valoarea de mijloc X a camerei Pixy    
#define Y_CENTER (199/2) // seteaza valoarea de mijloc Y a camerei Pixy
#define ledPin 9 // Led

/*variabile globale pentru valorile PWM*/
const byte channel_pin[] = {2, 3, 4, 5, 6, 7}; // folosim pini 2,3,4,5,6,7 pentru intrare PWM
volatile unsigned long rising_start[] = {0, 0, 0, 0, 0, 0};
volatile long channel_length[] = {0, 0, 0, 0, 0, 0}; //
volatile long yaw; 
volatile long throttle;
volatile long roll;
volatile long pitch;
volatile long ch5;
volatile long ch6;


Pixy2 pixy; 
int x; 
int y; 
int height; 
int width; 
int objectSize;
bool autoLand = false; 
bool objectFound = false; 
int throttleLast;  
bool descendBefore = false ;
bool cutOff = false;


int ppm[CHANNEL_NUMBER];


double rollInput; 
double pitchInput; 
double Kp = 0.2 ; 
double Ki = 0.6 ;
double Kd = 0.05 ; 
double Setpoint = 0;
double rollOutput; 
double pitchOutput; 
double rollFinal; 
double pitchFinal;

/*Clase pentru PID */
PID rollPID(&rollInput, &rollOutput, &Setpoint, Kp, Ki, Kd, REVERSE);
PID pitchPID(&pitchInput, &pitchOutput, &Setpoint, Kp, Ki, Kd, REVERSE  );

void setup()
{
  /*Setari pentru PID*/
  rollPID.SetMode(AUTOMATIC); 
  pitchPID.SetMode(AUTOMATIC); 
  rollPID.SetSampleTime(5);  
  pitchPID.SetSampleTime(5); 
  rollPID.SetOutputLimits(-1 * TURNING_SPEED, TURNING_SPEED);
  pitchPID.SetOutputLimits(-1 * TURNING_SPEED, TURNING_SPEED); 

  /*Initializare valori PPM*/
  for (int i = 0; i < CHANNEL_NUMBER; i++)
  {
    if ( i == 5 || i == 6) {
      ppm[i] = SWITCH_OFF_VALUE;
    }
    else {
      ppm[i] = CHANNEL_DEFAULT_VALUE;
    }
  }

  pinMode(ledPin, OUTPUT); 

  pinMode(sigPin, OUTPUT);
  digitalWrite(sigPin, !onState);  

  /*Setari pentru citirea PWM*/
  cli();
  TCCR1A = 0; 
  TCCR1B = 0;
  OCR1A = 100;  
  TCCR1B |= (1 << WGM12);  
  TCCR1B |= (1 << CS11);  
  TIMSK1 |= (1 << OCIE1A);
  sei();


  pinMode(channel_pin[0], INPUT);
  pinMode(channel_pin[1], INPUT);
  pinMode(channel_pin[2], INPUT);
  pinMode(channel_pin[3], INPUT);
  pinMode(channel_pin[4], INPUT);
  pinMode(channel_pin[5], INPUT);

 
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(channel_pin[0]), onRising0, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(channel_pin[1]), onRising1, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(channel_pin[2]), onRising2, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(channel_pin[3]), onRising3, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(channel_pin[4]), onRising4, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(channel_pin[5]), onRising5, CHANGE);

  /*Initializare pentru camera Pixy*/
  pixy.init();
}

/*Functie pentru a detecta schimbarile in pulsul PWM*/
void processPin(byte pin)
{
  uint8_t trigger = getPinChangeInterruptTrigger(digitalPinToPCINT(channel_pin[pin]));

  if (trigger == RISING){
    rising_start[pin] = micros();
  }
  else if (trigger == FALLING){
    channel_length[pin] = micros() - rising_start[pin];
  }
}

/*functii pentru a chema functia processPin()*/
void onRising0(void) {
  processPin(0);
}
void onRising1(void) {
  processPin(1);
}
void onRising2(void) {
  processPin(2);
}
void onRising3(void) {
  processPin(3);
}
void onRising4(void) {
  processPin(4);
}
void onRising5(void) {
  processPin(5);
}


/*Cronometru ce se ocupa de generarea pulsului PPM*/
ISR(TIMER1_COMPA_vect){
  static boolean state = true;
  TCNT1 = 0;
  if (state){
    //start pulse
    digitalWrite(sigPin, onState);
    OCR1A =  PULSE_LENGTH * 2;
    state = false;
  }
  else{
    static byte cur_chan_numb;
    static unsigned int calc_rest;
    digitalWrite(sigPin, !onState);
    state = true;
    if (cur_chan_numb >= CHANNEL_NUMBER){
      cur_chan_numb = 0;
      calc_rest = calc_rest + PULSE_LENGTH;//
      OCR1A = (FRAME_LENGTH - calc_rest) * 2;
      calc_rest = 0;
    }
    else{
      OCR1A = (ppm[cur_chan_numb] - PULSE_LENGTH) * 2;
      calc_rest = calc_rest + ppm[cur_chan_numb];
      cur_chan_numb++;
    }
  }
}

void loop() {
  /*Stocheaza valorile PWM*/
  throttle = channel_length[0];
  roll = channel_length[1];
  pitch = channel_length[2];
  yaw = channel_length[3];
  ch5 = channel_length[4];
  ch6 = channel_length[5];

  /*Veifica daca suprafata de aterizate este detectata)*/
  if ( pixy.ccc.getBlocks() > 0 ){
    objectFound = true;
    digitalWrite(ledPin, HIGH); 
  }
  else{
    objectFound = false;
    digitalWrite(ledPin, LOW);
  }

  Serial.print(" objectFound:");
  Serial.print(objectFound);
  Serial.print("   ");

  /*Verifica daca butonul de aterizare autonoma este pornit*/
  if ( ch5 <= CHANNEL_DEFAULT_VALUE){
    autoLand = false;
  }
  else if ( ch5 >= CHANNEL_DEFAULT_VALUE ){
    if (autoLand == false){
      autoLand = true;
      throttleLast = throttle;
    }
    else{
      autoLand = true;
    }
  }

  Serial.print(" autoLand:");
  Serial.print(autoLand);
  Serial.print("   ");

  if (objectFound == true){
    x = X_CENTER - pixy.ccc.blocks[0].m_x  ;
    y = (pixy.ccc.blocks[0].m_y) - Y_CENTER ;
    height = pixy.ccc.blocks[0].m_height;
    width = pixy.ccc.blocks[0].m_width;
    objectSize = height * width;
    rollInput = (double)x;
    pitchInput = (double)y;
  }

  /*Daca butonul nu este activat Arduino va comunica mai departe valorile
  receiverului nemodificate*/
  if ( autoLand == false ){
    ppm[0] = throttle;
    ppm[1] = roll;
    ppm[2] = pitch;
    ppm[3] = yaw;
    ppm[4] = ch5;
    ppm[5] = ch6;
  }
  else
  {
    ppm[3] = yaw;
    ppm[4] = ch5;
    ppm[5] = ch6;

    Serial.print(" Am intrat in primul else");
    Serial.print("   ");

    if (objectSize >= LANDING_SIZE_VALUE )
    {
      cutOff = true;
      Serial.print(" cutOff = true");
      Serial.print("   ");
      ppm[0] = THROTTLE_LANDING_VALUE;
      ppm[1] = CHANNEL_DEFAULT_VALUE;
      ppm[2] = CHANNEL_DEFAULT_VALUE;
    }
    else if (cutOff == false)
    {
          Serial.print(" cutOff = false");
          Serial.print("   ");
      
          /*PID pentru a centra drona*/
          rollPID.Compute(); 
          pitchPID.Compute(); 
          rollFinal = CHANNEL_DEFAULT_VALUE + rollOutput; 
          pitchFinal = CHANNEL_DEFAULT_VALUE + pitchOutput; 
      
          /*Elimina miscarile prea rapide*/
          if (rollFinal > CHANNEL_DEFAULT_VALUE + TURNING_SPEED){
            rollFinal = CHANNEL_DEFAULT_VALUE + TURNING_SPEED;
          }
          else if (rollFinal < CHANNEL_DEFAULT_VALUE - TURNING_SPEED){
            rollFinal = CHANNEL_DEFAULT_VALUE - TURNING_SPEED;
          }
          if (pitchFinal > CHANNEL_DEFAULT_VALUE + TURNING_SPEED){
            pitchFinal = CHANNEL_DEFAULT_VALUE + TURNING_SPEED;
          }
          else if (pitchFinal < CHANNEL_DEFAULT_VALUE - TURNING_SPEED){
            pitchFinal = CHANNEL_DEFAULT_VALUE - TURNING_SPEED;
          }
      
          ppm[1] = (int)rollFinal; 
          ppm[2] = (int)pitchFinal; 
      
          /*Daca drona este centrata, incepe coborarea */
          if ( abs(x) <= LANDING_X_VALUE && abs(y) <= LANDING_Y_VALUE){
            Serial.print(" cobor");
            Serial.print("   ");
            ppm[0] = throttleLast - LANDING_SPEED;
            descendBefore = true;
          }
          else{
            /*Daca drona nu este in pozitita buna si a coborat anterior, va incerca sa isi mentina atitudinea*/
            if (descendBefore == true) {
              Serial.print(" mentin atitudinea");
              Serial.print("   ");
              ppm[0] = throttleLast + LANDING_SPEED; // increase throttle value back to maintain its altitude
              descendBefore = false;
            }
            else{
              Serial.print(" throttle manual");
              Serial.println("   ");
              ppm[0] = throttleLast;
            }
          }
    }
  }

}
