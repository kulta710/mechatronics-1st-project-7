#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <softPwm.h>

// Define
#define ENCODERA 17
#define ENCODERB 27

#define MOTOR1 19
#define MOTOR2 26

#define PULSE 18

#define LOOPTIME 5
#define ENC2REDGEAR 216

#define PGAIN 100
#define IGAIN 10
#define DGAIN 1

// Variables
int encA;
int encB;
int encoderPosition = 0;
float redGearPosition = 0;

float referencePosition = 10;
float errorPosition = 0;
float pid = 0;

unsigned int checkTime;
unsigned int checkTimeBefore;

int pulse;

int trialNum = 0;

int trialIndex = 0;

int loop = 0;

int beforeErrorPosition = 0;
int bbeforeErrorPosition = 0;

float itae = 0;

unsigned int startTime = 0;

// Functions
void funcEncoderA() {
	encA = digitalRead(ENCODERA);
	encB = digitalRead(ENCODERB);

	if (encA == HIGH) {
		if (encB == LOW) encoderPosition++;
		else encoderPosition--;
	}
	else {
		if (encB == HIGH) encoderPosition++;
		else encoderPosition--;
	}

	redGearPosition = (float)encoderPosition / ENC2REDGEAR;

	printf("funcEncoderA() A: %d B: %d encPos: %d gearPos: %f\n",
		encA, encB, encoderPosition, redGearPosition);

	errorPosition = referencePosition - redGearPosition;

	printf("errPos: %f\n", errorPosition);
}

void funcEncoderB() {
	encA = digitalRead(ENCODERA);
	encB = digitalRead(ENCODERB);

	if (encB == HIGH) {
		if (encA == HIGH) encoderPosition++;
		else encoderPosition--;
	}
	else {
		if (encA == LOW) encoderPosition++;
		else encoderPosition--;
	}

	redGearPosition = (float)encoderPosition / ENC2REDGEAR;

	printf("funcEncoderB() A: %d B: %d encPos: %d gearPos: %f\n",
		encA, encB, encoderPosition, redGearPosition);

	errorPosition = referencePosition - redGearPosition;

	printf("errPos: %f\n", errorPosition);
}

// Main Function
int main(void) {
    // Input Test Conditions
    printf("Repitition: ");
    scanf("%d", &trialNum);
    printf("\n");

    int trialArr[trialNum];

    for (int i = 0; i < trialNum; i++) {
        printf("Number %d Target Position: ", (i + 1));
        scanf("%d", &trialArr[i]);
        printf("\n");
    }

    // Set-up
	wiringPiSetupGpio();

	pinMode(ENCODERA, INPUT);
	pinMode(ENCODERB, INPUT);

    pinMode(PULSE, INPUT);

	softPwmCreate(MOTOR1, 0, 100);
	softPwmCreate(MOTOR2, 0, 100);

	wiringPiISR(ENCODERA, INT_EDGE_BOTH, funcEncoderA);
	wiringPiISR(ENCODERB, INT_EDGE_BOTH, funcEncoderB);

    // PID Control

    
	checkTimeBefore = millis();

    int flag = 1;

	while (1) {
		checkTime = millis();

		if (checkTime - checkTimeBefore > LOOPTIME) {

			loop++;

            pulse = digitalRead(PULSE);
            
            if (pulse == 0) {
				flag = 1;
			}

            if (pulse == 1 && flag == 1) {
				flag = 0;
				
                startTime = millis();

                referencePosition = trialArr[trialIndex];

                loop = 0; trialIndex++;
            }

            errorPosition = referencePosition - redGearPosition;

            if (loop == 0) {
                beforeErrorPosition = referencePosition - redGearPosition;
                bbeforeErrorPosition = referencePosition - redGearPosition;
            }

            pid = (PGAIN + IGAIN * LOOPTIME + DGAIN / LOOPTIME) * errorPosition - (PGAIN + 2 * DGAIN / LOOPTIME) * beforeErrorPosition + (DGAIN / LOOPTIME) * bbeforeErrorPosition;

			if (errorPosition > 0) {
				
                
                softPwmWrite(MOTOR1, pid);
				softPwmWrite(MOTOR2, 0);

                bbeforeErrorPosition = beforeErrorPosition;
                beforeErrorPosition = errorPosition;

                itae += (checkTime - startTime) * errorPosition * LOOPTIME;

				// printf("errPos > 0, gearPos: %f\n", redGearPosition);
			}
			else {
				softPwmWrite(MOTOR2, pid);
				softPwmWrite(MOTOR1, 0);

                bbeforeErrorPosition = beforeErrorPosition;
                beforeErrorPosition = errorPosition;

                itae += (checkTime - startTime) * errorPosition * LOOPTIME;

				// printf("errPos < 0, gearPos: %f\n", redGearPosition);
			}

			checkTimeBefore = checkTime;

            printf("%f, %d\n", itae, pulse);
		}
	}

	return 0;
}