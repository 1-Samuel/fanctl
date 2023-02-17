#pragma clang diagnostic push
#pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection"
#pragma ide diagnostic ignored "modernize-loop-convert"
#include <Arduino.h>

// resistance at 25 degrees C
#define THERMISTORNOMINAL 10000
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25
// how many samples to take and average, more takes longer
// but is more 'smooth'
#define NUMSAMPLES 200
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3380
// the value of the 'other' resistor
#define SERIESRESISTOR 10000

#define PORTCOUNT 2

#define VENT_PIN 9

int samples[NUMSAMPLES][PORTCOUNT];

int const ports[] = { PIN_A0, PIN_A1 };

int pwmValue = 22;

int userValue = -1;

const word PWM_FREQ_HZ = 25000;
const word TCNT1_TOP = 16000000/(2*PWM_FREQ_HZ);

#define NUMRSAVG 100
#define NUMRSAVG_PORTS 2
double ravg[NUMRSAVG_PORTS][NUMRSAVG];
int rindex[NUMRSAVG_PORTS];
double ravgsum[NUMRSAVG_PORTS];
int const ravgPorts[] = { 2, 3 };

void takeSamples(int numSamples, int portCount);

void init(double *average, int length);

void calculateAverages(double *average);

double getCelcius(double d);

void pwm25kHzBegin() {
    // Clear Timer1 control and count registers
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1  = 0;

    // Set Timer1 configuration
    // COM1A(1:0) = 0b10   (Output A clear rising/set falling)
    // COM1B(1:0) = 0b00   (Output B normal operation)
    // WGM(13:10) = 0b1010 (Phase correct PWM)
    // ICNC1      = 0b0    (Input capture noise canceler disabled)
    // ICES1      = 0b0    (Input capture edge select disabled)
    // CS(12:10)  = 0b001  (Input clock select = clock/1)
    
    TCCR1A |= (1 << COM1A1) | (1 << WGM11);
    TCCR1B |= (1 << WGM13) | (1 << CS10);
    ICR1 = TCNT1_TOP;
}

void pwmDuty(byte duty) {
    OCR1A = (word) (duty*TCNT1_TOP)/100;
}


void setup(void) {
    for (int i = 0; i < NUMRSAVG_PORTS; i++) {
        for (int j = 0; j < NUMRSAVG; j++) {
            ravg[i][j] = 0;
        }
    }
    pinMode(VENT_PIN, OUTPUT);
    Serial.begin(9600);
    pwm25kHzBegin();
    pwmDuty(pwmValue);
}

void loop(void) {

    // send data only when you receive data:
    if (Serial.available() > 0) {
      userValue = Serial.parseInt();
    }

    double average[PORTCOUNT];

    init(average, PORTCOUNT);

    takeSamples(NUMSAMPLES, PORTCOUNT);

    calculateAverages(average);

    for (int i = 0; i < PORTCOUNT; i++) {
        average[i] /= NUMSAMPLES;
        average[i] = 1023 / average[i] - 1;
        average[i] = SERIESRESISTOR / average[i];
        average[i] = getCelcius(average[i]);
        Serial.print(average[i]);
        Serial.print(',');
    }

    for (int i = 0; i < NUMRSAVG_PORTS; ++i) {
        int portIndex = ravgPorts[i];
        ravg[i][rindex[i]] = average[portIndex];
        ravgsum[i] += average[portIndex];
        rindex[i] = (rindex[i] + 1) % NUMRSAVG;
        ravgsum[i] -= ravg[i][rindex[i]];
    }


    pwmValue = (int)(13 * average[0] - 380);
    pwmValue = constrain(pwmValue, 11, 100);

    if (pwmValue >= 100) {
      userValue = 100;
    }

    pwmDuty(userValue >= 0 ? userValue : pwmValue);

    Serial.print(pwmValue);

    Serial.print(" ");

    Serial.print(userValue);

    Serial.println(" ");

    delay(200);
}

double getCelcius(double d) {
    double steinhart;
    steinhart = d / THERMISTORNOMINAL;     // (R/Ro)
    steinhart = log(steinhart);                  // ln(R/Ro)
    steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
    steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
    steinhart = 1.0 / steinhart;                 // Invert
    steinhart -= 273.15;
    return (double)steinhart;
}

void calculateAverages(double *average) {
    for (int i = 0; i < NUMSAMPLES; i++) {
        for (int j = 0; j < PORTCOUNT; j++) {
            average[j] += (double)samples[i][j];
        }
    }
}

void init(double *average, int length) {
    for (int i = 0; i < length; i++) {
        average[i] = 0;
    }
}

void takeSamples(int numSamples, int portCount) {// take N samples in a row, with a slight delay
    for (int i = 0; i < numSamples; i++) {
        for (int j = 0; j < portCount; j++) {
            samples[i][j] = analogRead(ports[j]);
        }
        delay(20);
    }
}

#pragma clang diagnostic pop
