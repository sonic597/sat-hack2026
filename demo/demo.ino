#include "ADCS.h"

//Data variable definition
int MotSpeed1 = 0; 
int MotSpeed2 = 0;
int Speed_adjustment = 120;
int stop_bit = 0;
size_t accumulated_spin = 0;
long previous_time = 0;
long current_time = 0;
int previous_spinrate;
float distance = 0.0;
int set_dis = 30;

//Program initialization
void setup()  
{
    Serial.begin(9600); 

    pinMode(5, OUTPUT);
    pinMode(6, OUTPUT);
    pinMode(9, OUTPUT);
    pinMode(10, OUTPUT);

    pinMode(Trig,OUTPUT);   
    pinMode(Echo,INPUT); 
    // Define ultrasonic sensor pins
    

}

//Principal function
void loop()
{
  int distance;
  distance = GetDistance(); 
  Serial.println(distance);  //Print distance in serial port
  if(distance < 30)     //Judge that the distance is less than 40cm and start to decelerate
  {
	  Speed_adjustment--;
	  delay(5);     //Car deceleration speed
	  if(Speed_adjustment<=120) Speed_adjustment=120;
    MotSpeed1 = Speed_adjustment+20;  //In order to make up for the car's yaw, a value can be added according to the actual situation //as:Speed_adjustment+20
    MotSpeed2 = Speed_adjustment;
  }

  if(distance > 30)  //Judge if the distance is more than 40cm and start to accelerate
  {
	  Speed_adjustment++;
	  delay(5);
	  if(Speed_adjustment>=235)Speed_adjustment=235; //The maximum limit is 255
      MotSpeed1 = Speed_adjustment+20; //In order to make up for the car's yaw, a value can be added according to the actual situation //as:Speed_adjustment+20
      MotSpeed2 = Speed_adjustment;
  }
     
	if(distance < set_dis)  //Judge that the car is less than the obstacle avoidance distance and start to turn backward
	{
		delay(10);
		distance = GetDistance(); 
		if(distance < set_dis)
		{
			if(stop_bit==0)
			{
				//stop
        motor1(0,0);
        motor2(0,0);
				delay(300);
				stop_bit=1;
			}
			//Car back
      motor1(0,140);
      motor2(0,140);
			delay(600);
			//The car turns left
      motor1(0, 140);
      motor2(140, 0);
			delay(200);          
			Speed_adjustment=120;
		}
	}
  else
  {
      //If there is no less than the obstacle avoidance distance, move forward
      motor1(MotSpeed1,0);
      motor2(MotSpeed2,0);
		  stop_bit=0;
  }
}
