/* Membuat Library Kalman Filter bahasa C 	   */
/* Referensi Kristian Lauszus, TKJ Electronics */

void setAngle(float angle) { this->angle = angle; } // Starting angle
float getRate() { return this->rate; }; // Return the unbiased rate

/* Pengaturan Kalman filter */
void setQangle(float Q_angle) { this->Q_angle = Q_angle; }
void setQbias(float Q_bias) { this->Q_bias = Q_bias; }
void setRmeasure(float R_measure) { this->R_measure = R_measure; }

float getQangle() { return this->Q_angle; }
float getQbias() { return this->Q_bias; }
float getRmeasure() { return this->R_measure; }

void Kalman()
{
    /* Tuning konstanta default -- dapat diubah oleh pengguna */
    Q_angle = 0.001f;
    Q_bias = 0.003f;
    R_measure = 0.03f;

    angle = 0.0f; // Reset sudut
    bias = 0.0f; // Reset bias

    P[0][0] = 0.0f; 
    P[0][1] = 0.0f;
    P[1][0] = 0.0f;
    P[1][1] = 0.0f;
}

float getAngle(float newAngle, float newRate, float dt)
{
	/* Step 1 */
	rate = newRate - bias;
	angle += dt * rate;

	/* Step 2 */
	P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
	P[0][1] -= dt * P[1][1];
	P[1][0] -= dt * P[1][1];
	P[1][1] += Q_bias * dt;

	/* Step 4 */
	float S = P[0][0] + R_measure; // Estimasi error

	/* Step 5 */
	float K[2]; // Kalman gain - Matriks 2x1
	K[0] = P[0][0] / S;
	K[1] = P[1][0] / S;

	/* Step 3 */
	float y = newAngle - angle; // Perbedaan sudut
	
	/* Step 6 */
	angle += K[0] * y;
	bias += K[1] * y;

	/* Step 7 */
	float P00_temp = P[0][0];
	float P01_temp = P[0][1];

	P[0][0] -= K[0] * P00_temp;
	P[0][1] -= K[0] * P01_temp;
	P[1][0] -= K[1] * P00_temp;
	P[1][1] -= K[1] * P01_temp;

	return angle;
}
