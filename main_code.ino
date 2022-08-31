#include <Servo.h>
#include <Wire.h>

#define LOOP_PERIOD 0.004 // 4 milliseconds
#define RAW_TO_RAD_PER_SEC 0.01526717557
#define PI 3.1415926535
#define FILTER_SIZE 5
#define NEWTONS_TO_PULSE 339.67

#define MAX_ANGLE 9.02//14.0
#define THROTTLE_RANGE 1000
#define ROLL_RANGE 500

#define MAX_THRUST 7 //1.387
#define D_X 0.7 			
#define D_Y 0.7 			
#define I_x 0.02
#define I_y 0.02
#define MASS 0.956			
#define G 9.806
#define L 0.1575  			

#define DEGTORAD 0.0174533

#define ERROR_CONSTANT 0.05


//Fast Square Root Algorithm, good for the numerical approximations. Function definition below...
double QSqrt(const double x);


double B_X=2*QSqrt(D_X*I_x);
double B_Y=2*QSqrt(D_Y*I_y);
double starting_time;

class MPU_6050{
	/*
		Wire connections for Arduino UNO:
			SDA -> A4
			SCL -> A5
	*/

	private:
		


		//Raw values
		int16_t AcX, AcY, AcZ; //Displacement Accelerations
    		int16_t GyX, GyY, GyZ; //Angular Velocities
    
		
		int16_t Tmp;		   //Temperature 

		double AccelX, AccelY, AccelZ;
		double VelocityX=0, VelocityY=0, VelocityZ=0;
		double DisplX=0, DisplY=0, DisplZ=0;
		double offset_x, offset_y, offset_z;


		void getRawValues(){
			Wire.beginTransmission(this->address);
			Wire.write(0x3B); 						//start with register 0x3B (Accel_x_H)
			Wire.endTransmission();
			Wire.requestFrom((uint8_t)this->address,(uint8_t)14,(uint8_t)true);	//Request for 14 registers
			//Check for a possible Hardware Error:
			while(Wire.available()<14){Serial.print("Error: Unavailable Registers: ");Serial.println(14 - Wire.available());}
			AcX=Wire.read()<<8|Wire.read();  // 0x3B (AcX_H) & 0x3C (AcX_L)    
			AcY=Wire.read()<<8|Wire.read();  // 0x3D (AcY_H) & 0x3E (AcY_L)
			AcZ=Wire.read()<<8|Wire.read();  // 0x3F (AcZ_H) & 0x40 (AcZ_L)
			Tmp=Wire.read()<<8|Wire.read();  // 0x41 (Tmp_H) & 0x42 (Tmp_L)
			GyX=Wire.read()<<8|Wire.read();  // 0x43 (GyX_H) & 0x44 (GyX_L)
			GyY=Wire.read()<<8|Wire.read();  // 0x45 (GyY_H) & 0x46 (GyY_L)
			GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GyZ_H) & 0x48 (GyZ_L)
		}


		//Due to vibrations etc, there is noise in the angular velocity values.
		void filter_values(){

			static double previous_values_x[FILTER_SIZE]={0};
			static double previous_values_y[FILTER_SIZE]={0};
			static double previous_values_z[FILTER_SIZE]={0};

			static double value_sum_x=0, value_sum_y=0, value_sum_z=0;		//The sums change dynamically

			
			value_sum_x=value_sum_x-previous_values_x[FILTER_SIZE-1];
			value_sum_y=value_sum_y-previous_values_y[FILTER_SIZE-1];
			value_sum_z=value_sum_z-previous_values_z[FILTER_SIZE-1];

			for(int i=FILTER_SIZE-1;i>0;i--){
				previous_values_x[i]=previous_values_x[i-1];
				previous_values_y[i]=previous_values_y[i-1];
				previous_values_z[i]=previous_values_z[i-1];	
			}

			previous_values_x[0]=((double)GyX - offset_x) * RAW_TO_RAD_PER_SEC;
			previous_values_y[0]=((double)GyY - offset_y) * RAW_TO_RAD_PER_SEC;
			previous_values_z[0]=((double)GyZ - offset_z) * RAW_TO_RAD_PER_SEC;


			value_sum_x=value_sum_x+previous_values_x[0];
			value_sum_y=value_sum_y+previous_values_y[0];
			value_sum_z=value_sum_z+previous_values_z[0];

			Alpha_x=(value_sum_x/FILTER_SIZE - Omega_x)/LOOP_PERIOD;
			Alpha_y=(value_sum_y/FILTER_SIZE - Omega_y)/LOOP_PERIOD;
			Alpha_z=(value_sum_z/FILTER_SIZE - Omega_z)/LOOP_PERIOD;
	
			Omega_x=value_sum_x/FILTER_SIZE;
			Omega_y=value_sum_y/FILTER_SIZE;
			Omega_z=value_sum_z/FILTER_SIZE;

		}


	public:
		//Must not be changed outside of the class
		double Theta_x, Theta_y, Theta_z;	//Angles
		double Omega_x, Omega_y, Omega_z;	//Angular Velocities
		double Alpha_x, Alpha_y, Alpha_z;	//Angular Accelerations
		double Wx, Wy, Wz;
		
		uint8_t address=0x68;
		

		double Total_Accel;					//Norm of the sum of the 3 acceleration vectors

		void setup_registers(){
			Wire.beginTransmission(this->address);
			Wire.write(0x6B);
			Wire.write(0x00);
			Wire.endTransmission();
			//Configure the accelerometer (+/- 8g)
			Wire.beginTransmission(this->address);
			Wire.write(0x1C);
			Wire.write(0x10);
			Wire.endTransmission();
			//Configure the gyro (500dps full scale)
			Wire.beginTransmission(this->address);
			Wire.write(0x1B);
			Wire.write(0x08);
			Wire.endTransmission();
		}


		void calibrate(unsigned long calibration_time = 5000){	//Time in milliseconds

			Serial.println("Calibrating...");
			unsigned long starting_time=millis();
			unsigned long N_of_samples=0;

			long offset_sum_x=0, offset_sum_y=0, offset_sum_z=0;
			double offset_acc_x=0,offset_acc_y=0,offset_acc_z=0;

			while(millis() - starting_time < calibration_time){
				this->getRawValues();
				offset_sum_x+=GyX;
				offset_sum_y+= GyY;
				offset_sum_z+= GyZ;
				offset_acc_x=offset_acc_x + AcX;
				offset_acc_y=offset_acc_y + AcY;
				offset_acc_z=offset_acc_z + AcZ;
				N_of_samples++;
			  delay(3);
			}

			offset_x=((double)offset_sum_x)/N_of_samples;
			offset_y=((double)offset_sum_y)/N_of_samples;
		  	offset_z=((double)offset_sum_z)/N_of_samples;	

		  	Wx = offset_acc_x/N_of_samples;
		  	Wy = offset_acc_y/N_of_samples;
			Wz = offset_acc_z/N_of_samples;


			double Total_Accel=QSqrt(((double)AcX)*AcX + ((double)AcY)*AcY + ((double)AcZ)*AcZ);
			Theta_x=0.004*asin((double)AcY/Total_Accel)*57.296;
			Theta_y=0.004*asin((double)AcX/Total_Accel)*(-57.296);
			Theta_z=0; 

			Serial.print("Done. ");Serial.print(N_of_samples);Serial.println(" samples taken");
		}	

		void refresh(){
			
			getRawValues();
			filter_values();
			
			//Adding a small angle at each iteration
			Theta_x=Theta_x + (Omega_x)*LOOP_PERIOD;
			Theta_y=Theta_y + (Omega_y)*LOOP_PERIOD;
			Theta_z=Theta_z + (Omega_z)*LOOP_PERIOD;


			double temp=Theta_y;

			//Compensating for the rotation in the z axis (using the approximation sinx = x)
			Theta_y=Theta_y - Theta_x*Omega_z*LOOP_PERIOD*0.0174532;
			Theta_x=Theta_x + temp*Omega_z*LOOP_PERIOD*0.0174532;

			Total_Accel=QSqrt(((double)AcX)*AcX + ((double)AcY)*AcY + ((double)AcZ)*AcZ);

			
		}
};

class Motor{
	private:
		double Thrust;
		int Microsecond_Pulse_Length;
		Servo thisMotor;
	public:
		const unsigned int Digital_Pin; 
		void initialize(){
			thisMotor.attach(Digital_Pin);
			thisMotor.writeMicroseconds(1000);
			return;
		}
		Motor(unsigned int pin_number):Digital_Pin(pin_number){
		}

		double getThrust(){
			return Thrust;
		}
		int getPulseLength(){
			return Microsecond_Pulse_Length;
		}
		void setThrust(double new_value){
			if(new_value<0)Thrust=0;
			else if(new_value>MAX_THRUST)Thrust=MAX_THRUST;
			else Thrust=new_value;
			Microsecond_Pulse_Length=(int)(NEWTONS_TO_PULSE*QSqrt(Thrust)+1000);
		}
		void setPulseLength(unsigned int ms){
			Microsecond_Pulse_Length = ms;
			Thrust = (((double)ms-1000)/NEWTONS_TO_PULSE)*(((double)ms-1000)/NEWTONS_TO_PULSE);
		}

		void sleep(){
			thisMotor.writeMicroseconds(Microsecond_Pulse_Length=1050);
			Thrust=0;
		}
		void applyThrust(){
			thisMotor.writeMicroseconds(getPulseLength());
		}
};

class RC_Receiver{
	private:
		

		unsigned long long throttle_timer;
		unsigned long yaw_timer;
		unsigned long pitch_timer;
		unsigned long roll_timer;



		bool pitch_last_channel;
		bool roll_last_channel;
		bool yaw_last_channel; 
		bool throttle_last_channel;




	public:

		const unsigned int pitch_pin;
		const unsigned int roll_pin;
		const unsigned int yaw_pin; 
		const unsigned int throttle_pin;

    unsigned long throttle_value;
		unsigned long yaw_value;
		unsigned long pitch_value;
		unsigned long roll_value;

		long pitch_difference;
		long roll_difference; 

		double map_value_to_throttle(){
				return (double)(throttle_value-1000)*(((double)MAX_THRUST)/THROTTLE_RANGE);
		}


		void Interrupt_Routine_(){

			//Throttle Channel
			if(throttle_last_channel==0 && ((PIND&(1<<throttle_pin))>>3)){
				throttle_last_channel=1;
				throttle_timer=micros();
			}
			else if(throttle_last_channel==1 && !((PIND&(1<<throttle_pin))>>3)){
				throttle_value=(throttle_value + micros()-throttle_timer)/2;
				throttle_last_channel=0;
			}
			
			//Pitch Channel
			if(pitch_last_channel==0 && ((PINB&(1<<1))>>1)){
				pitch_last_channel=1;
				pitch_timer=micros();
			}
			else if(pitch_last_channel==1 && !((PINB&(1<<1))>>1)){
				pitch_difference=pitch_value;
				pitch_value= (pitch_value + micros()- pitch_timer)/2;
				pitch_difference=pitch_value-pitch_difference;
				pitch_last_channel=0;
			}

			//Roll Channel
			if(roll_last_channel==0 && ((PINB&(1<<2))>>2)){
				roll_last_channel=1;
				roll_timer=micros();
			}
			else if(roll_last_channel==1 && !((PINB&(1<<2))>>2)){
				roll_difference=roll_value;
				roll_value=(roll_value + micros()-roll_timer)/2;
				roll_last_channel=0;
				roll_difference=roll_value-roll_difference;
			}
			
			//Yaw Channel
			// if(yaw_last_channel==0 && (PIND&(1<<yaw_pin)>>3)){
			// 	yaw_last_channel=0;
			// 	yaw_timer=micros();
			// }
			// else if(yaw_last_channel==0 && !(PIND&(1<<yaw_pin)>>3)){
			// 	yaw_value=micros()-yaw_timer;
			// 	yaw_last_channel=0;
			// }
		}


		RC_Receiver(const int throttle_input=3,const int pitch_input=9, const int roll_input=10, const int yaw_input=11):
		throttle_pin(throttle_input), pitch_pin(pitch_input), roll_pin(roll_input), yaw_pin(yaw_input) {}

		bool isIdle(){
			return (throttle_value<1050);
		}


};


MPU_6050 Gyro;
Motor BackRight(13);
Motor FrontRight(12);
Motor FrontLeft(7);
Motor BackLeft(6);
RC_Receiver receiver;
double Thrust_Total;
double Yaw_Input;

void Apply_Thrusts(){
	
	FrontRight.applyThrust();
	FrontLeft.applyThrust();
	BackRight.applyThrust();
	BackLeft.applyThrust();
	
}


ISR(PCINT0_vect){
	receiver.Interrupt_Routine_();
}

int loop_counter=0;
const int print_counter=80;
void MakeCorrection(){

	//Some variables related to the receiver input
	static double previous_input_pitch_angle=0;
	static double previous_input_roll_angle=0;
	double pitch_derivative_input=(map_angles(receiver.pitch_value)-previous_input_pitch_angle)/LOOP_PERIOD;
	double roll_derivative_input=(map_angles(receiver.roll_value)-previous_input_roll_angle)/LOOP_PERIOD;
	previous_input_pitch_angle=map_angles(receiver.pitch_value);
	previous_input_roll_angle=map_angles(receiver.roll_value);



	//These two variables take such values so that the drone will simulate a (rotarary) damped oscillation around the axes x and y
	//These values derive from Newton's second law, with the existence of a possible external torque (torque_sensor - torque_applied) 
	double Pitch_Oscillation_Constant=((-B_X*(Gyro.Omega_x -pitch_derivative_input) - D_X*(Gyro.Theta_x - map_angles(receiver.pitch_value))-I_x*Gyro.Alpha_x)*DEGTORAD)/L + FrontRight.getThrust() + FrontLeft.getThrust() - BackRight.getThrust() - BackLeft.getThrust();
	double Roll_Oscillation_Constant=((-B_Y*(Gyro.Omega_y -roll_derivative_input) - D_Y*(Gyro.Theta_y - map_angles(receiver.roll_value))-I_y*Gyro.Alpha_y)*DEGTORAD)/L - FrontRight.getThrust() + FrontLeft.getThrust() - BackRight.getThrust() + BackLeft.getThrust();

	//Following a similar idea in the z axis...
	double Yaw_Constant=0.05*Gyro.Omega_z+0.002*Gyro.Alpha_z -FrontRight.getThrust()+FrontLeft.getThrust() +BackRight.getThrust() -BackLeft.getThrust();
	Thrust_Total=receiver.map_value_to_throttle();///cos(DEGTORAD* QSqrt((Gyro.Theta_x)*(Gyro.Theta_x) + (Gyro.Theta_y)*(Gyro.Theta_y))); 

	//Solving a system of 4 linear equations:
	//T_FR + T_FL + T_BR + T_BL = Thrust_Total
	//T_FR + T_FL - T_BR - T_BL = Pitch_Oscillation_Constant
	//-T_FR + T_FL - T_BR + T_BL = Roll_Oscillation_Constant
	//T_FR - T_FL - T_BR + T_BL = Yaw_Constant

	FrontRight.setThrust((Pitch_Oscillation_Constant-Roll_Oscillation_Constant+Thrust_Total + Yaw_Constant)/4);
	FrontLeft.setThrust((Pitch_Oscillation_Constant + Roll_Oscillation_Constant + Thrust_Total - Yaw_Constant)/4);
	BackRight.setThrust((-Pitch_Oscillation_Constant - Roll_Oscillation_Constant + Thrust_Total - Yaw_Constant)/4);
	BackLeft.setThrust((-Pitch_Oscillation_Constant + Roll_Oscillation_Constant + Thrust_Total + Yaw_Constant)/4);
}


void stall(double starting_time){
	while(micros()-starting_time<LOOP_PERIOD*1e6);
}

double map_angles(int pulse_length){
	return (pulse_length-1500)*(MAX_ANGLE/500);
}


void helper(){
	receiver.Interrupt_Routine_();
}

void setup(){
	FrontRight.initialize();
	FrontLeft.initialize();
	BackRight.initialize();
	BackLeft.initialize();

	Serial.begin(9600); 
	attachInterrupt(digitalPinToInterrupt(3),helper,CHANGE);
	PCICR |= (1 << PCIE0);                                                                                                                                                     
	PCMSK0 |= (1 << PCINT1);    //PIN 9                                       
	PCMSK0 |= (1 << PCINT2);    //PIN 10                                         					                                   
	Gyro.setup_registers();
	Gyro.calibrate();
}

void PrintVariables(){	//for debugging
	
	// Serial.print("Alpha_x = "); Serial.print(Gyro.Alpha_x);
	// Serial.print(" | Alpha_y = "); Serial.print(Gyro.Alpha_y);
	// Serial.print(" | Alpha_z = "); Serial.println(Gyro.Alpha_z);


	// Serial.print("Omega_x = "); Serial.print(Gyro.Omega_x);
	//Serial.print(" | Omega_y = "); Serial.print(Gyro.Omega_y);
	//Serial.print(" | Omega_z = "); Serial.println(Gyro.Omega_z);


	// Serial.print("Theta_x= "); Serial.print(Gyro.Theta_x);
	// Serial.print(" | Theta_y = "); Serial.print(Gyro.Theta_y);
	// Serial.print(" | Theta_z = "); Serial.println(Gyro.Theta_z);
	// Serial.println();

	Serial.print(" | Thrust_FR = "); Serial.print(FrontRight.getPulseLength());
	Serial.print(" | Thrust_FL = "); Serial.print(FrontLeft.getPulseLength());
	Serial.print(" | Thrust_BR = "); Serial.print(BackRight.getPulseLength());
	Serial.print(" | Thrust_BL = "); Serial.println(BackLeft.getPulseLength());
	Serial.println();


}

void loop(){

	
	starting_time=micros();
	if(loop_counter==print_counter){
		//PrintVariables();
		loop_counter=0;
	}
	
	Gyro.refresh();
	if(receiver.isIdle()){
		FrontRight.sleep();
		FrontLeft.sleep();
		BackRight.sleep();
		BackLeft.sleep();
	}
	else{
		MakeCorrection();
		Apply_Thrusts();
	}
	if(micros()-starting_time>LOOP_PERIOD*1e6){Serial.println("Error: Loop exceeds the predetermined LOOP_PERIOD ");}
	else stall(starting_time);
	

	
	loop_counter++;
  
}

//Simple modification of the Quake 3 fast inverse square root algorithm :)
double QSqrt(const double x){

	long i;
	float x2,y;
	const float threehalfs = 1.5F;
	x2=x *0.5F;
	y=x;
	i=*(long *)&y;
	i=0x5F3759DF - (i>>1);			
	y=*(float*)&i;
	y=y*(threehalfs - (x2*y*y));
	y=y*(threehalfs - (x2*y*y));
	return 1/y;
}


