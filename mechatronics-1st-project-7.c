#include <stdio.h>
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

unsigned int checkTime;
unsigned int checkTimeBefore;

int trialNum = 0;

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
    printf("총 시행 횟수를 입력하세요: ");
    scanf_s("%d", &trialNum);
    printf("\n");

    int trialArr[trialNum];

    for (int i = 0; i < trialNum; i++) {
        printf("%d번째 시행의 목표 위치를 입력하세요: ", (i + 1));
        scanf_s("%d", &trialArr[i]);
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
	errorPosition = referencePosition - redGearPosition;

	checkTimeBefore = millis();

	while (1) {
		checkTime = millis();

		if (checkTime - checkTimeBefore > LOOPTIME) {
			if (errorPosition > 0) {
				softPwmWrite(MOTOR1, errorPosition * PGAIN);
				softPwmWrite(MOTOR2, 0);

				// printf("errPos > 0, gearPos: %f\n", redGearPosition);
			}
			else {
				softPwmWrite(MOTOR2, errorPosition * PGAIN);
				softPwmWrite(MOTOR1, 0);

				// printf("errPos < 0, gearPos: %f\n", redGearPosition);
			}

			checkTimeBefore = checkTime;
		}
	}

	return 0;
}