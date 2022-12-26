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
#define NUMSAMPLES 100
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3380
// the value of the 'other' resistor
#define SERIESRESISTOR 10000

#define PORTCOUNT 2

#define VENT_PIN PIN3

int samples[NUMSAMPLES][PORTCOUNT];

int const ports[] = { PIN_A0, PIN_A1 };

int pwmValue = 22;

#define NUMRSAVG 100
#define NUMRSAVG_PORTS 2
float ravg[NUMRSAVG_PORTS][NUMRSAVG];
int rindex[NUMRSAVG_PORTS];
float ravgsum[NUMRSAVG_PORTS];
int const ravgPorts[] = { 2, 3 };

void takeSamples(int numSamples, int portCount);

void init(float *average, int length);

void calculateAverages(float *average);

float getCelcius(float d);

void pwm25kHzBegin() {
    TCCR2A = 0;                               // TC2 Control Register A
    TCCR2B = 0;                               // TC2 Control Register B
    TIMSK2 = 0;                               // TC2 Interrupt Mask Register
    TIFR2 = 0;                                // TC2 Interrupt Flag Register
    TCCR2A |= (1 << COM2B1) | (1 << WGM21) | (1 << WGM20);  // OC2B cleared/set on match when up/down counting, fast PWM
    TCCR2B |= (1 << WGM22) | (1 << CS21);     // prescaler 8
    OCR2A = 79;                               // TOP overflow value (Hz)
    OCR2B = 0;
}

void pwmDuty(byte ocrb) {
    OCR2B = ocrb;                             // PWM Width (duty)
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
    float average[PORTCOUNT];

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


    pwmValue = (int)(4.625 * average[0] - 200);
    pwmValue = constrain(pwmValue, 8, 79);


    pwmDuty(pwmValue);

    Serial.print(pwmValue);

    Serial.println(" ");

    delay(200);
}

float getCelcius(float d) {
    double steinhart;
    steinhart = d / THERMISTORNOMINAL;     // (R/Ro)
    steinhart = log(steinhart);                  // ln(R/Ro)
    steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
    steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
    steinhart = 1.0 / steinhart;                 // Invert
    steinhart -= 273.15;
    return (float)steinhart;
}

void calculateAverages(float *average) {
    for (int i = 0; i < NUMSAMPLES; i++) {
        for (int j = 0; j < PORTCOUNT; j++) {
            average[j] += (float)samples[i][j];
        }
    }
}

void init(float *average, int length) {
    for (int i = 0; i < length; i++) {
        average[i] = 0;
    }
}

void takeSamples(int numSamples, int portCount) {// take N samples in a row, with a slight delay
    for (int i = 0; i < numSamples; i++) {
        for (int j = 0; j < portCount; j++) {
            samples[i][j] = analogRead(ports[j]);
        }
        delay(10);
    }
}

#pragma clang diagnostic pop
