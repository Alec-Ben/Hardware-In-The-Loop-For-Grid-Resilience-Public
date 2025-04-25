// PQ Control Version 1
// Adapted from Simulink Model
// Authors: Alec Benedict, Kendall Meienhofer, Janessa Green, Emily Ninestein

// ------------------------- PIN Definitions -------------------------
#define PWM_A 2    // PWM Output Phase A
#define PWM_B 3    // PWM Output Phase B
#define PWM_C 4    // PWM Output Phase C
#define PWM_A_o 5  // inverse PWM Output Phase A
#define PWM_B_o 6 // inverse PWM Output Phase B
#define PWM_C_o 7 // inverse PWM Output Phase C
#define P_OUT 10   // Real power output
#define Q_OUT 11   // Reactive power output

// Analog input pin numbers
const int Vd_pin = A0;
const int Vq_pin = A1;
const int Id_pin = A2;
const int Iq_pin = A3;
const int P_ref_pin = A6;
const int freq_pin = A4;
const int theta_pin = A5;

// ------------------------- Constants -------------------------
const float Pi = 3.1415926;
const float TAU = 1 / (2 * Pi * 10) * 5;  // Low-pass filter tau
const float Tc = 1e-4;                   // control loop time step (100 us)
float LPF_ALPHA = Tc / TAU;
const float PWM_FREQ = 1000000;           // 1 MHz PWM Frequency
float Q_ref = 0.0;

const float Vdc = 400;
const float LF = 125e-6 * 400;
const float RF = 53e-3;
const float CF = 3.5e-6 * 400;
const float RD = 5e-6;
const float LG = 0.0012;
const float RG = 0.6;
const float OMEGA_NOM = 2 * Pi * 60;      // 60Hz frequency

const float carrierPeriod_us = 10.0;       // 100 kHz => 10 µs period
const float halfPeriod_us = carrierPeriod_us / 2.0;  // 5 µs

// ------------------------- Variables -------------------------
float Vd_prev = 0, Vq_prev = 0, Id_prev = 0, Iq_prev = 0, P_prev, Q_prev;
volatile float theta = 0.0; // PLL Phase Angle (updated in interrupt)

// ------------------------- PID Controller Structure -------------------------
struct PIDController {
    float kp, ki, kd;
    float N, Tc;
    float usatbound, lsatbound, kt;
    float integrator_state;
    float derivative_state;
    float eprev;
    float control_out;
    float satout;

    float update(float r, float y) {
        float error = r - y;
        float es = satout - control_out;

        // Integrator with anti-windup
        integrator_state += (kt * es + ki * error) * Tc;

        // Derivative with filtering
        derivative_state = (1 - Tc / N) * derivative_state + (1 / N) * (error - eprev);
        eprev = error;

        // PID Control law
        control_out = kp * error + integrator_state + kd * derivative_state;

        // Saturation
        if (control_out > usatbound) {
            satout = usatbound;
        } else if (control_out < lsatbound) {
            satout = lsatbound;
        } else {
            satout = control_out;
        }

        return satout;
    }
};

// Initialize PID Controllers
PIDController pid1 = {10, 2, 0.0, 10, Tc, 100, -100, 0.1};   // Power P
PIDController pid2 = {10, 7, 0.0, 10, Tc, 100, -100, 0.1};   // Reactive Q
PIDController pid3 = {10, 1000, 0.0, 10, Tc, Vdc * 0.45, -0.25 * Vdc, 0.1}; // Vd control
PIDController pid4 = {10, 1000, 0.0, 10, Tc, Vdc * 0.45, -Vdc * 0.45, 0.1}; // Vq control

// ------------------------- Function Prototypes -------------------------
float lowPassFilter(float input, float prevOutput, float alpha);
float computeActivePower(float Vd, float Vq, float Id, float Iq);
float computeReactivePower(float Vd, float Vq, float Id, float Iq);
void dqToAbc(float Vd, float Vq, float theta, float &Va, float &Vb, float &Vc);
void generatePWM(float Va, float Vb, float Vc);

// ------------------------- Functions -------------------------

float lowPassFilter(float input, float prevOutput, float alpha) {
    return alpha * input + (1 - alpha) * prevOutput;
}

float computeActivePower(float Vd, float Vq, float Id, float Iq) {
    return 1.5 * (Vd * Id + Vq * Iq);
}

float computeReactivePower(float Vd, float Vq, float Id, float Iq) {
    return 1.5 * (Vq * Id - Vd * Iq);
}

void dqToAbc(float Vd, float Vq, float theta, float &Va, float &Vb, float &Vc) {
    float cosTheta = cos(theta);
    float sinTheta = sin(theta);

    Va = sqrt(2.0 / 3.0) * (Vd * cosTheta - Vq * sinTheta);
    Vb = sqrt(2.0 / 3.0) * (Vd * cos(theta - 2.0 * PI / 3.0) - Vq * sin(theta - 2.0 * PI / 3.0));
    Vc = sqrt(2.0 / 3.0) * (Vd * cos(theta + 2.0 * PI / 3.0) - Vq * sin(theta + 2.0 * PI / 3.0));
}

void generatePWM(float Va, float Vb, float Vc) {
    float now_us = (float)(micros() % 10);
    float triangle;

    Va = constrain(Va, -1.0, 1.0);
    Vb = constrain(Vb, -1.0, 1.0);
    Vc = constrain(Vc, -1.0, 1.0);

    if (now_us < 5.0) {
        triangle = -1.0 + 2.0 * (now_us / 5.0);
    } else {
        triangle = 1.0 - 2.0 * ((now_us - 5.0) / 5.0);
    }

    bool a = (Va > triangle);
    bool b = (Vb > triangle);
    bool c = (Vc > triangle);

    bool a_prime = !a;
    bool b_prime = !b;
    bool c_prime = !c;

    digitalWrite(PWM_A, a ? HIGH : LOW);
    digitalWrite(PWM_B, b ? HIGH : LOW);
    digitalWrite(PWM_C, c ? HIGH : LOW);
    digitalWrite(PWM_A_o, a_prime ? HIGH : LOW);
    digitalWrite(PWM_B_o, b_prime ? HIGH : LOW);
    digitalWrite(PWM_C_o, c_prime ? HIGH : LOW);
}

// ------------------------- Setup -------------------------
void setup() {
    pinMode(PWM_A, OUTPUT);
    pinMode(PWM_B, OUTPUT);
    pinMode(PWM_C, OUTPUT);
    pinMode(A7, OUTPUT);

    cli();
    TCCR1A = 0;
    TCCR1B = (1 << WGM12) | (1 << CS11); // CTC mode, prescaler 8
    OCR1A = 199;
    TIMSK1 |= (1 << OCIE1A);
    sei();
}

// ------------------------- Interrupt -------------------------
ISR(TIMER1_COMPA_vect) {
    float analog_theta = analogRead(A6) / 1023.0 * 5.0;
    theta = analog_theta * (2 * PI / 5.0);
}

// ------------------------- Main Loop -------------------------
void loop() {
    float t = micros() / 1e6;

    float Vd_offset = 275;
    float Vq_offset = 300;
    float Id_offset = 300;

    float Vd = (analogRead(Vd_pin) / 1023.0) * 5.0 * ((2 * Vd_offset) / 5.0) - Vd_offset;
    float Vq = (analogRead(Vq_pin) / 1023.0) * 5.0 * ((2 * Vq_offset) / 5.0) - Vq_offset;
    float Id = (analogRead(Id_pin) / 1023.0) * 5.0 * ((2 * Id_offset) / 5.0) - Id_offset;
    float Iq = (analogRead(Iq_pin) / 1023.0) * 5.0;
    float P_ref = (analogRead(P_ref_pin) / 1023.0) * 5.0 * (500 / 5.0);
    float freq = (analogRead(freq_pin) / 1023.0) * 5.0 * (100 / 5.0);
    float theta_read = (analogRead(theta_pin) / 1023.0) * 5.0 * (10 / 5.0);

    float P = computeActivePower(Vd, Vq, Id, Iq);
    float Q = computeReactivePower(Vd, Vq, Id, Iq);

    analogWrite(P_OUT, map(P, 0, 35000, 0, 255));
    analogWrite(Q_OUT, map(Q, -3000, 15000, 0, 255));

    P = lowPassFilter(P, P_prev, LPF_ALPHA);
    Q = lowPassFilter(Q, Q_prev, LPF_ALPHA);

    P_prev = P;
    Q_prev = Q;

    float P_error_1 = pid1.update(P_ref, P);
    float Q_error_1 = pid2.update(Q_ref, Q);
    Q_error_1 = Vd * OMEGA_NOM * CF - Q_error_1;

    float P_error_2 = pid3.update(P_error_1, Id);
    float Direct_out = Vd + P_error_2 - Iq * OMEGA_NOM * LF;

    float Q_error_2 = pid4.update(Q_error_1, Iq);
    float Quad_out = Id * OMEGA_NOM * LF + Q_error_2 + Vq;

    float Va, Vb, Vc;
    dqToAbc(Direct_out, Quad_out, theta, Va, Vb, Vc);

    Va = Va * 2 / Vdc;
    Vb = Vb * 2 / Vdc;
    Vc = Vc * 2 / Vdc;

    generatePWM(Va, Vb, Vc);

    Vd_prev = Vd;
    Vq_prev = Vq;
    Id_prev = Id;
    Iq_prev = Iq;
}
