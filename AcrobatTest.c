/*
 * AcrobatTest.c
 *
 * Created: 11/7/2014 11:22:50 PM
 *  Author: chitvan
 */ 


#include <avr/io.h>
#include "m_imu.h"
#include "m_usb.h"
#include "m_rf.h"

#define tiltPosition 1		// ay changing with tilting
#define rotationAxis 3		// Rotation along x axis
#define AccelScale 0
#define GyroScale 0
#define gValue 16384
#define pi 3.14159265
#define del_t 0.01			// time period of readings in seconds
#define alpha 0.5			// Acceleration 
#define beta 0.1
#define dt 0.01632			// 255*1024/16M

void Timer0Init();
void printMIMU();
void printMatlab();
void findTheta(float* theta,float* data);

// Global Variables
int rawData[9],i,flag;
float data[9],mean_data[6] = {140.2580,-229.1100,276.73,-92.614,181.6275,-129.1410},*theta;
float theta_accel, theta_gyro=0, omega_gyro;

int main(void)
{
	m_clockdivide(0);
	m_red(ON);
	m_bus_init();
	m_usb_init();
	Timer0Init();
	while( !( m_imu_init(AccelScale, GyroScale) ) );
	m_red(OFF);
    while(1)
    {
// 		if(flag)
// 		{
// 			flag = 0;
// 			while(!m_imu_raw(rawData));
// 			for(i=1;i<6;i++)
// 			{
// 				data[i] = rawData[i]-mean_data[i];
// 			}
// 			//		printMIMU();
// 			//		printMatlab();
// 			m_green(TOGGLE);
// 			theta_accel = atan2(*(data+2),*(data+tiltPosition));
// 			m_usb_tx_string("\nTheta Accel : ");
// 			m_usb_tx_int(180*theta_accel/pi);
// 			if(theta_gyro==0)
// 			{
// 				theta_gyro=theta_accel;
// 			}
// 			else
// 			theta_gyro += data[rotationAxis]*pi*125*dt/(32764*180.0);
// 			m_usb_tx_string("\t\tTheta Gyro : ");
// 			m_usb_tx_int(180*theta_gyro/pi);
// 		}
//		findTheta(theta,data);
// 		m_green(TOGGLE);
// 		while(!m_imu_raw(data));
// 		printMIMU();
// 		m_wait(1000);
    }
}

void Timer0Init()
{
	sei();
	clear(TCCR0B,WGM02);						// Upto 0xFF
	TCCR0A &= ~( (1<<WGM01) | (1<<WGM00) );		// ^^
	set(TIMSK0,TOIE0);							// Overflow interrupt
	TCCR0B |= ( (1<<CS02) | (1<<CS00) );		// freq/1024
	TCCR0B &= (~(1<<CS01));						// ^^
}

ISR(TIMER0_OVF_vect)
{
	cli();
	m_green(TOGGLE);
	//flag = 1;
	while(!m_imu_raw(rawData));
	for(i=1;i<6;i++)
	{
		data[i] = rawData[i]-mean_data[i];
	}
	//		printMIMU();
	//		printMatlab();
	m_green(TOGGLE);
	theta_accel = atan2(*(data+2),*(data+tiltPosition));
	m_usb_tx_string("\nTheta Accel : ");
	m_usb_tx_int(180*theta_accel/pi);
	if(theta_gyro==0)
	{
		theta_gyro=theta_accel;
	}
	else
	theta_gyro -= data[rotationAxis]*pi*125*dt/(16384*180.0);
	m_usb_tx_string("\t\tTheta Gyro : ");
	m_usb_tx_int(180*theta_gyro/pi);
	sei();
}

void printMIMU()
{
	m_usb_tx_string("\nAcceleration : (");
	m_usb_tx_int(data[0]);
	m_usb_tx_string(" , ");
	m_usb_tx_int(data[1]);
	m_usb_tx_string(" , ");
	m_usb_tx_int(data[2]);
	m_usb_tx_string(")");
	m_usb_tx_string("\t\tGyroscope : (");
	m_usb_tx_int(data[3]);
	m_usb_tx_string(" , ");
	m_usb_tx_int(data[4]);
	m_usb_tx_string(" , ");
	m_usb_tx_int(data[5]);
	m_usb_tx_string(")");
	m_usb_tx_string("\t\tMagnetometer : (");
	m_usb_tx_int(data[6]);
	m_usb_tx_string(" , ");
	m_usb_tx_int(data[7]);
	m_usb_tx_string(" , ");
	m_usb_tx_int(data[8]);
	m_usb_tx_string(")");
}

void printMatlab()
{
	char rx_buffer; //computer interactions
	while(!m_usb_rx_available());  	//wait for an indication from the computer
	rx_buffer = m_usb_rx_char();  	//grab the computer packet
	
	m_usb_rx_flush();  				//clear buffer
	
	if(rx_buffer == 1) 
	{  			//computer wants ir data
		//write ir data as concatenated hex:  i.e. f0f1f4f5
		for (i = 0 ; i < 9 ; i++)
		{
			m_usb_tx_int(data[i]);
			m_usb_tx_char('\t');
		}
	m_usb_tx_char('\n');  //MATLAB serial command reads 1 line at a time
	}
}

void findTheta(float* theta,float* data)
{
	//static float theta_prev;
	theta_accel = atan2(*(data+2),*(data+tiltPosition));
	omega_gyro = *(data+rotationAxis)/32764*125;		// Angular rotation in degrees per second
	theta_gyro = *theta + omega_gyro*del_t*pi/180;		// theta in radians
	// Calculating Theta using moving average
	*theta = (1-alpha-beta)*(*theta) + (alpha*theta_accel) + (beta*theta_gyro);
}