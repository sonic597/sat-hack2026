//2021.6.7
//Define the direction of car movement
#define  stop     0
#define  forward  1
#define  back     2
#define  left     3
#define  right    4

//Definition of ultrasonic pin
#define  Trig A4 
#define  Echo A5

//Data variable definition
int MotSpeed1 = 0; 
int MotSpeed2 = 0;
int Speed_adjustment = 120;
int stop_bit = 0;
size_t accumulated_spin = 0;
volatile long previous_time = 0;
volatile long current_time = 0;
volatile int previous_spinrate;
volatile long previous_spin1=0;
volatile long previous_spin2=0;
volatile long accumulated_change_spin1 = 0;
volatile long accumulated_change_spin2 = 0;


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
  //  avoidance(18);  //In brackets is the obstacle avoidance distance in cm
  motor(forward, 200,200);
  delay(100);
  motor(forward, 0,0);
  delay(100);
  Serial.print("change in 1: ");
  Serial.println(accumulated_change_spin1);
}

/*
Function: obtain ultrasonic sensor ranging data
Parameters: Trig, Echo
Parameter description: sensor connected to the motherboard pin port A4,A5
Trig -------> pin A4
Echo -------> pin A5
*/
float GetDistance()
{
    float distance;
  
	digitalWrite(Trig, LOW); 
	delayMicroseconds(2); 
	digitalWrite(Trig, HIGH); 
	delayMicroseconds(10);
	digitalWrite(Trig, LOW);

  distance = pulseIn(Echo, HIGH) / 58.00;
     
	return distance;  //Return distance
}

/*
* Function: Obstacle avoidance
* Parameter: set_dis sets the obstacle avoidance distance
*/

//Motor 1 output definition
void motor1(int steep1, int steep2) 
{
   analogWrite(5,steep1);
   analogWrite(6,steep2); 
}

//Motor 2 output definition
void motor2(int steep1, int steep2)
{
   analogWrite(9,steep1);
   analogWrite(10,steep2); 
}

//Obstacle avoidance function
void avoidance(int set_dis)
{
    int distance;
    distance = GetDistance(); 
    Serial.println(distance);  //Print distance in serial port
    if(distance < 30)     //Judge that the distance is less than 40cm and start to decelerate
    {
	  Speed_adjustment--;
	  //delay(5);     //Car deceleration speed
	  if(Speed_adjustment<=120)Speed_adjustment=120;
      MotSpeed1 = Speed_adjustment+20;  //In order to make up for the car's yaw, a value can be added according to the actual situation //as:Speed_adjustment+20
      MotSpeed2 = Speed_adjustment;
    }

   if(distance > 30)  //Judge if the distance is more than 40cm and start to accelerate
    {
	  Speed_adjustment++;
	  //delay(5);
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
				motor(stop,0,0);//stop
				delay(300);
				stop_bit=1;
			}
			motor(back,140,140); //Car back
			delay(600);
			motor(left,140,140); //The car turns left
			delay(200);          
			Speed_adjustment=120;
		}
	}
    else
    {
        motor(forward,MotSpeed1,MotSpeed2);//If there is no less than the obstacle avoidance distance, move forward
		    stop_bit=0;
    }
  
}

//Motor direction function to determine the direction of the car
void motor(int dir ,int speed1, int speed2)  
{  
    if(previous_time != 0){
      //TODO: take into account direction paramter
      // if this is not the first time calling the function
      unsigned long elapsed = millis() - previous_time;
      accumulated_spin += elapsed*previous_spinrate;
    }
    else{
      // this is the first time the function was called
      previous_time = millis();
    }
    accumulated_change_spin1 += (speed1 - previous_spin1);
    accumulated_change_spin2 += (speed2 - previous_spin2);


    previous_spinrate = speed1+speed2;
    previous_time = millis();
    previous_spin1 = speed1;
    previous_spin2 = speed2;

    switch (dir)
    {
    case stop:
        motor1(0, 0);
        motor2(0, 0);
       
        break;

    case forward:  //Go ahead
       
       motor1(speed1,0); //If the circuit is not connected correctly, modify here to correct the motor direction
       motor2(speed2,0); 
       
        break;

    case back:  //Back up
        
        motor1(0,speed1);
        motor2(0,speed2);
        
        break;

    case left:   //Turn left
        motor1(0, speed1);
        motor2(speed2, 0);
        
        break;

    case right:   //Turn right
        
        motor1(speed1, 0);
        motor2(0, speed2);
       
        break;
    
    default:
        break;
    }

    
}
