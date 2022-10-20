//////////////////////////////////////////////////
// Include
//////////////////////////////////////////////////
#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <softPwm.h>

//////////////////////////////////////////////////
// Define
//////////////////////////////////////////////////
#define ENCODERA 17
#define ENCODERB 27

#define MOTOR1 19
#define MOTOR2 26

#define PULSE 18

#define LOOPTIME 5
#define ESCAPETIME 3000

#define ENC2REDGEAR 216

#define PGAIN 80
#define IGAIN 10
#define DGAIN 1

//////////////////////////////////////////////////
// Variables
//////////////////////////////////////////////////
int encA = 0;
int encB = 0;
int pulse = 0;

int encoderPosition = 0;
float referencePosition = 0;
float redGearPosition = 0;

float errorPosition = 0;
float beforeErrorPosition = 0;
float bbeforeErrorPosition = 0;

int trialNum = 0;
int trialIndex = 0;
int loopIndex = 0;

unsigned int startTime = 0;
unsigned int checkTime = 0;
unsigned int checkTimeBefore = 0;

float pid = 0;

float itae = 0;

//////////////////////////////////////////////////
// Functions
//////////////////////////////////////////////////
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

	printf(
        "funcEncoderA() A: %d B: %d encPos: %d gearPos: %f\n",
		encA, encB, encoderPosition, redGearPosition
    );
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

	printf(
        "funcEncoderB() A: %d B: %d encPos: %d gearPos: %f\n",
		encA, encB, encoderPosition, redGearPosition
    );
}

//////////////////////////////////////////////////
// Main Function
//////////////////////////////////////////////////
int main(void) {
    // Input Test Conditions
    printf("===== Program Start =====\n");
    printf("Trial Number: ");
    scanf("%d", &trialNum);
    printf("\n");

    int trialArr[trialNum];

    for (int i = 0; i < trialNum; i++) {
        printf("Target Position at Trial %d: ", (i + 1));
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
    // 입력받은 횟수 실행
    for(trialIndex = 0; trialIndex < trialNum; trialIndex++) {           
        
        // 신호대기
        printf("Receiving Transmission...\n");

        while(1){
            pulse = digitalRead(PULSE);

            // Pulse Check
            // printf("Pulse Signal : %d\n ", pulse);

            if (pulse == 1) {
                break;
            }
        }

        // 매회 시작 출력, 초기화
        startTime = millis();
	    checkTimeBefore = millis();

        printf("----- Start %d Trial at %d -----\n", trialIndex + 1, startTime);

        loopIndex = 0;
	
        // while문 탈출을 위해 시간제한 설정 (Time_for_escape)
        while(1) {
		    checkTime = millis();
		    Time_for_escape = millis();
		
            if ((checkTime - checkTimeBefore > LOOPTIME)) {    //동시만족 시 진행
                referencePosition = trialArr[trialIndex];

                errorPosition = referencePosition - redGearPosition;

                // 초기값 설정
                if (loopIndex == 0) {
                    // pid 초기화
                    pid = 0;

                    // P-control
                    pid += errorPosition * PGAIN;

                    // I-control
                    pid += errorPosition * IGAIN * LOOPTIME;

                    // D-control
                    pid += errorPosition * DGAIN / LOOPTIME;

                    beforeErrorPosition = errorPosition;
                    bbeforeErrorPosition = 0;
                } else if (loopIndex == 1) {
                    // pid 초기화
                    pid = 0;

                    // P-control
                    pid += errorPosition * PGAIN;

                    // I-control
                    pid += (errorPosition + beforeErrorPosition) * IGAIN * LOOPTIME;

                    // D-control
                    pid += (errorPosition - beforeErrorPosition) * DGAIN / LOOPTIME;

                    beforeErrorPosition = errorPosition;
                    bbeforeErrorPosition = beforeErrorPosition;
                } else {
                    // pid 계산
                    pid += (PGAIN + IGAIN * LOOPTIME + DGAIN / LOOPTIME) * errorPosition;
                    pid -= (PGAIN + 2 * DGAIN / LOOPTIME) * beforeErrorPosition;
                    pid += (DGAIN / LOOPTIME) * bbeforeErrorPosition;

                    // 다음 진행을 위한
                    beforeErrorPosition = errorPosition;
                    bbeforeErrorPosition = beforeErrorPosition;
                }

			    if (errorPosition > 0) {
                    softPwmWrite(MOTOR1, pid);
				    softPwmWrite(MOTOR2, 0);

                    itae += (checkTime - startTime) * errorPosition * LOOPTIME;

				    // printf("errPos > 0, gearPos: %f\n", redGearPosition);
		        } else {
				    softPwmWrite(MOTOR2, pid);
				    softPwmWrite(MOTOR1, 0);

                    itae += (checkTime - startTime) * (-1) * errorPosition * LOOPTIME;

				    // printf("errPos < 0, gearPos: %f\n", redGearPosition);
			    }

		        checkTimeBefore = checkTime;
                loopIndex++;

		        // printf("%f\n", itae);
		    }
            
            if (checkTime - startTime >= ESCAPETIME) {
			    printf("Time for escape: %d\n", checkTime);

                loopIndex = 0;
			    
                break;
			}
		}
		
        // 매회 결과 출력
		printf("Complete Trial Number %d\n", trialIndex + 1);
        printf("Current ITAE Value: %f\n", itae);
	}

    return 0;
}