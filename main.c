/************************************************************************/
/* 		PROGRAM TUGAS BESAR SISTEM MIKROPROSESOR EL3014 tahun 2017		*/
/************************************************************************/
/*		Dosen Pembimbing : Ir. Farkhad Ihsan Hariadi MASc				*/
/*		Anggota	:														*/
/*		GUSTAV ADITYA PERMANA		13214021							*/
/*		M NAJIB IRVANI				13214109							*/
/************************************************************************/
/* Program ini difokuskan untuk stabilizer sb.X--ROLL yaitu servo Hitec */
/* Untuk kombinasi roll dan pitch masih di arduino						*/
/* Kendali servo dynamixel cukup sulit dalam bahasa C					*/
/* Untuk servo Hitec hanya perlu memberikan duty cycle					*/
/* Pada program ini stabilizer sb.X--ROLL sudah baik/halus				*/
/************************************************************************/

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <Kalman.h> 
#include <math.h>
#include <i2c_master.h>

#define RESTRICT_PITCH
#define IMU_WRITE 0x19
#define IMU_READ 0x3B 

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;

double gyroXangle; // Pengukuran sudut dengan gyro
double compAngleX; // Pengukuran sudut dengan complementary filter
double kalAngleX; // Pengukuran sudut dengan Kalman filter

/* Variabel untuk Servo Hitec */
int sudut0 = 97;

/* Variabel untuk i2c IMU */
uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

/* Variabel Pushbutton */
int flagPlus = 1, flagMinus = 1;

void init_uart(uint16_t baudrate){

	uint16_t UBRR_val = (F_CPU/16)/(baudrate-1);

	UBRR0H = UBRR_val >> 8;
	UBRR0L = UBRR_val;

	UCSR0B |= (1<<TXEN0) | (1<<RXEN0) | (1<<RXCIE0); // UART TX (Transmit - senden) einschalten
	UCSR0C |= (1<<USBS0) | (3<<UCSZ00); //Modus Asynchron 8N1 (8 Datenbits, No Parity, 1 Stopbit)
}

void uart_putc(unsigned char c){

	while(!(UCSR0A & (1<<UDRE0))); // wait until sending is possible
	UDR0 = c; // output character saved in c
}

void uart_puts(char *s){
	while(*s){
		uart_putc(*s);
		s++;
	}
}

void init_IMU(void)
{
	i2c_start(IMU_WRITE);
	i2c_write(0x00); // atur pointer ke CRA
	i2c_write(0x70); // tulis 0x70 ke CRA
	i2c_stop();

	i2c_start(IMU_WRITE);
	i2c_write(0x01); // atur pointer ke CRB
	i2c_write(0xA0);
	i2c_stop();

	i2c_start(IMU_WRITE);
	i2c_write(0x02); // atur pointer ke mode pengukuran
	i2c_write(0x00); // continous measurement
	i2c_stop();
}

float setupSudut(void)
{
	i2c_start(IMU_WRITE);
	i2c_write(0x03); // atur pointer ke X axis MSB
	i2c_stop();

	i2c_start(IMU_READ);

	accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
	accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
	accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
	
	#ifdef RESTRICT_PITCH // Eq. 25 and 26
		double roll  = atan2(accY, accZ) * RAD_TO_DEG;
		double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
	#else // Eq. 28 and 29
		double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
		double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
	#endif

	setAngle(roll);
	gyroXangle = roll;
	compAngleX = roll;
	
	timer = micros();
}

void hitungSudut()
{
	while (i2cRead(0x3B, i2cData, 14));
	accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
	accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
	accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
	gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
	gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
	gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;

	double dt = (double)(micros() - timer) / 1000000; // Menghitung waktu delta
	timer = micros();

	// Konversi radian ke derajat
	#ifdef RESTRICT_PITCH
		double roll  = atan2(accY, accZ) * RAD_TO_DEG;
	#else
		double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
	#endif

	double gyroXrate = gyroX / 131.0; // Konversi ke derajat/detik

	#ifdef RESTRICT_PITCH
	if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90))
	{
		kalmanX.setAngle(roll);
		compAngleX = roll;
		kalAngleX = roll;
		gyroXangle = roll;
	} else
		kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

	gyroXangle += gyroXrate * dt; // Menghitung sudut gyro tanpa filter

	compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Menghitng sudut dengan Complimentary filter

	// Mereset gyro apabila berotasi berlebihan
	if (gyroXangle < -180 || gyroXangle > 180)
		gyroXangle = kalAngleX;
	
	// kalAngleX adalah pembacaan posisi sudut setelah melalui kalman filter
}

void moveServo(int sudut) // Menggerakkan servo Hi-Tec
{
	//Servo 2.433per degree. 97 = 0degree, 535 = 180 degree
	OCR1A=int(sudut0 + sudut*2.43);   //0 degree
}

void main()
{
	//Pengaturan TIMER1
	TCCR1A|=(1<<COM1A1)|(1<<COM1B1)|(1<<WGM11);        //NON Inverted PWM
	TCCR1B|=(1<<WGM13)|(1<<WGM12)|(1<<CS11)|(1<<CS10); //PRESCALER=64 MODE 14(FAST PWM)

	ICR1=4999;  //fPWM=50Hz (Period = 20ms Standard).

	DDRB|=(1<<PB1)|(1<<PB2);   //PWM Pins untuk data servo (output) -- Timer 1 --  OC1A servo Hitec
	DDRD|=(0<<PD3)|(0<<PD4)|(0<<PD5)|(0<<PD6)|(0<<PD7);   //Buttons Pins untuk input pushbutton (input)

	PORTD|=0b11111000; // PD3-PD7 input Pull Up
	
	init_uart(57600);
	i2c_init();
	init_IMU();
	setupSudut();
   
	while(1)
	{
		if (bit_is_clear(PIND, 4) && flagPlus) // Plus ROLL
		{
			target_roll++;
			flagPlus = 0;	// flag untuk mengatasi bouncing pushbutton
		} else
		{
			flagPlus = 1;
		}
		if (bit_is_clear(PIND, 3) && flagMinus) // Minus Roll
		{
			target_roll--;
			flagMinus = 0;
		} else
		{
			flagMinus = 1;
		}
		
		hitungSudut(); // Hasil dari hitung sudt adalah kalAngleX
		
		error_roll = (int)target_roll-kalAngleX;
		pwm_roll = pwm_roll-error_roll;
		moveServo(pwm_roll);
		
		_delay_ms(20); // Sampling time 20ms -- 1 periode servo
	}
}
