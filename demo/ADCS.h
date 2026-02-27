//Definition of ultrasonic pin
#define  Trig A4 
#define  Echo A5

long previous_spin1 = 0;
long previous_spin2 = 0;
long accumulated_change_spin1 = 0;
long accumulated_change_spin2 = 0;

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

//Motor 1 output definition
void motor1(int steep1, int steep2) 
{
   analogWrite(5,steep1);
   analogWrite(6,steep2);
   accumulated_change_spin1 = abs(steep1 + steep2 - previous_spin1);
   previous_spin1 = steep1 + steep2;
}

//Motor 2 output definition
void motor2(int steep1, int steep2)
{
   analogWrite(9,steep1);
   analogWrite(10,steep2);
   accumulated_change_spin2 = abs(steep1 + steep2 - previous_spin1);
   previous_spin2 = steep1 + steep2;
}