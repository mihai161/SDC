/*  Control Rover manual + Autopilot Jetson  */

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
  pinMode(txPin3, INPUT);
  pinMode(txPin13, INPUT);

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

    }

    if (dataBl == '4'){

      //Comenzi drivere L298N
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
      analogWrite(enA, 180); // Trimite semnal PWM catre motorul A
      analogWrite(enB, 180); // Trimite semnal PWM catre motorul B

      Serial.println("Inapoi");//Inapoi
    }

    if (dataBl == '5'){
      //Citeste indicatiile prezise de reteaua neuronala primite de la Jetson
      txPin3Val = digitalRead(txPin3);
      txPin13Val = digitalRead(txPin13);
      
      Serial.print(txPin3Val);  //Debugging
      Serial.print(txPin13Val); //Debugging

      //Trimite comenzi motoarelor pe baza datelor de la Jetson
      if (txPin3Val == LOW && txPin13Val == HIGH){
        Dreapta();
        Serial.print("Dreapta");
      }
      else if(txPin3Val == HIGH && txPin13Val == HIGH){
        Fata();
        Serial.print("Fata");
      }
      else if(txPin3Val == HIGH && txPin13Val == LOW){
        Stanga();
        Serial.print("Stanga");
      }
      else if(txPin3Val == LOW && txPin13Val == LOW){
        Stop();
        Serial.print("Stop");
      }
      
    }



  
    
}
void Stanga(){
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
      analogWrite(enA, 100 ); 
      analogWrite(enB, 200); 
}

void Dreapta(){
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      analogWrite(enA, 200); 
      analogWrite(enB, 100);
}

void Fata(){
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      // Set Motor B backward
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      
      analogWrite(enA, 200); 
      analogWrite(enB, 200);
}

void Stop(){
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);
      analogWrite(enA, 0);
      analogWrite(enB, 0); 
}
