/*  Colectat Date   */

#include <SPI.h>
#include <Servo.h>
#include<SoftwareSerial.h>

#define enA 9
#define in1 4
#define in2 5
#define enB 10
#define in3 6
#define in4 2
#define txPin3 3 //pin16 pe Jetson
#define txPin13 13 //pin18 pe Jetson
bool txPin3Val;
bool txPin13Val;


int speedA = 255;
int speedB = 255;
int ceva = 0;
int angleV = 0;
int stanga = 0;
int dreapta = 0;
int TotalspeedA = 0;
int TotalspeedB = 0;
char dataBl;

struct Data_Package {
  int viteza;
  int directie;
  
};

Data_Package data;

SoftwareSerial bt(7,8); /* (Rx,Tx) */  

void setup() {
  bt.begin(9600); //Conexiune cu modulul bluetooth HC05 
  Serial.begin(9600); //Conexiune pentru debuging

  //Initiaizat pini digitali
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(txPin3, OUTPUT);
  pinMode(txPin13, OUTPUT);

}

void loop() {

  //Daca conexiunea la bluetooth este realizata
  if (bt.available() > 0) 
      {
      dataBl = bt.read(); //citeste input de la HC05
      }

 Serial.print(dataBl); //debuging
  
 if (dataBl == '0'){

      //Comenzi drivere L298N
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
      analogWrite(enA, 255); // Trimite semnal PWM catre motorul A
      analogWrite(enB, 255); // Trimite semnal PWM catre motorul B

      Serial.println("Left");
      //Comunicarea directiei catre Jetson 
      digitalWrite(txPin11, HIGH);  
      digitalWrite(txPin12, HIGH);
        
    }
  
    if (dataBl == '1'){
      
      //Comenzi drivere L298N
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      analogWrite(enA, 255); // Trimite semnal PWM catre motorul A
      analogWrite(enB, 255); // Trimite semnal PWM catre motorul B

      Serial.println("Right");
      //Comunicarea directiei catre Jetson 
      digitalWrite(txPin11, HIGH);
      digitalWrite(txPin12, LOW);
      
    }

    if (dataBl == '2'){
      //Comenzi drivere L298N
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      analogWrite(enA, 200); // Trimite semnal PWM catre motorul A
      analogWrite(enB, 200); // Trimite semnal PWM catre motorul B
   
      Serial.println("Fata");//Fata
      //Comunicarea directiei catre Jetson 
      digitalWrite(txPin11, LOW);
      digitalWrite(txPin12, HIGH);

    }

    if (dataBl == '3'){
      //Comenzi drivere L298N
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW );
      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);
      analogWrite(enA, 0); // Trimite semnal PWM catre motorul A
      analogWrite(enB, 0); // Trimite semnal PWM catre motorul B
      
      Serial.println("Stop");//Stop
      //Comunicarea directiei catre Jetson 
      digitalWrite(txPin11, LOW);
      digitalWrite(txPin12, LOW);
    }
}
