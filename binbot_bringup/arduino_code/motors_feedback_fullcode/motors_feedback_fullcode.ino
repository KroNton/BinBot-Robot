/* 
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>

////////Motors pins ////////


#define IN1 7
#define IN2 5


#define ENA 4
#define ENB 6

////////Encoders pins ////////

//left encoder
#define RH_ENCODER_A  20
#define RH_ENCODER_B 16

//right encoder
#define LH_ENCODER_A 2
#define LH_ENCODER_B 8





//ros::NodeHandle  nh;                                 //////// Motors call back functions and sub_nodes ////////
ros::NodeHandle  nh1;
void lmotor_Cb( const std_msgs::Float32& lm){
  
  left_motor_control(lm.data);
}

void rmotor_Cb( const std_msgs::Float32& rm){
  
  right_motor_control(rm.data);
}

ros::Subscriber<std_msgs::Float32> subl("lmotor_cmd", &lmotor_Cb );
ros::Subscriber<std_msgs::Float32> subr("rmotor_cmd", &rmotor_Cb );


                              //////// Encoders functions and pub_nodes ////////

std_msgs::Int16 len_msg;
std_msgs::Int16 ren_msg;
ros::Publisher pub_lencoder( "lwheel", &len_msg);
ros::Publisher pub_rencoder( "rwheel", &ren_msg);

volatile  long leftCounter = 0;
volatile  long rightCounter = 0;                              


void setup()
{ 

    nh1.initNode();
                
                         //////// Motors ////////
  //motorA
  pinMode(IN1, OUTPUT);
  pinMode(ENA, OUTPUT);
  //motorB
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);

  nh1.subscribe(subl);
  nh1.subscribe(subr);


                      //////// Encoders ////////

  //nh.initNode();
  nh1.advertise(pub_lencoder);
  nh1.advertise(pub_rencoder);


  pinMode(LH_ENCODER_A, INPUT);  //Interrupts pin declaration
  pinMode(LH_ENCODER_B, INPUT);
  pinMode(RH_ENCODER_A, INPUT);
  pinMode(RH_ENCODER_B, INPUT);
  

  attachInterrupt(digitalPinToInterrupt(2), leftEncoderISR, CHANGE);   //attach interrupt fn
  attachInterrupt(digitalPinToInterrupt(20), rightEncoderISR, CHANGE);// relates the interrupt no.
  // the ISR Function and the Interrupt Mode

  
}

void loop()
{  

  len_msg.data = leftCounter;
  ren_msg.data = rightCounter;
    
    pub_lencoder.publish( &len_msg );
    pub_rencoder.publish( &ren_msg );
    
  nh1.spinOnce();

  
 
}


void left_motor_control(float vel1){
  
 //move the wheel Forward
  if(vel1> 0.0){
    digitalWrite(IN1,1);
    
    analogWrite(ENA,vel1);
    
    }
    
   //move the wheel backward
   
  else if(vel1< 0.0){
    
    digitalWrite(IN1,0);
    vel1=abs(vel1);
    analogWrite(ENA,vel1);
    }

   else if(vel1== 0) {
        
    analogWrite(ENA,0);
      }

    
  }



  void right_motor_control(float vel2){
  
 //move the wheel Forward
  if(vel2> 0.0){
    digitalWrite(IN2,1);
    
    analogWrite(ENB,vel2);
    
    }
    
   //move the wheel backward
   
  else if(vel2< 0.0){
    
    digitalWrite(IN2,0);
    vel2=abs(vel2);
    analogWrite(ENB,vel2);
    }

   else if(vel2== 0) {
        
    analogWrite(ENB,0);
      }

    
  }




void leftEncoderISR()   //ISR for the Left Motor encoder
{         
  if (digitalRead(LH_ENCODER_A) == HIGH) {
    if (digitalRead(LH_ENCODER_B) == LOW) {
      leftCounter++;    //if output A is High and Output B is low this means forward
    } else {
      leftCounter--;   // else then the motor is rotating backward so it subtracts the counter 
    }
    
  }
} 

void rightEncoderISR() 
{
  if (digitalRead(RH_ENCODER_A) == HIGH) {
    if (digitalRead(RH_ENCODER_B) == LOW) {
      rightCounter++;
    } else {
      rightCounter--;
    }
  } 
}


  
