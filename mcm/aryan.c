#define ENC_L_A 2
#define ENC_L_B 5
#define ENC_R_A 3
#define ENC_R_B 6

// Motors
#define PIN_LEFT_PWM   10
#define PIN_LEFT_DIR1  4
#define PIN_LEFT_DIR2  7

#define PIN_RIGHT_PWM  9
#define PIN_RIGHT_DIR1 8
#define PIN_RIGHT_DIR2 12

#define PIN_STBY       13

// Sensors
#define PIN_IR_FRONT   A1
#define PIN_IR_LEFT    A0
#define PIN_IR_RIGHT   A2

// Robot Physics
#define TICKS_PER_MM   
#define MOTOR_CPR     

// PID Gains
#define KP  
#define KI  
#define KD 

// ==========================================
// 2. STRUCTS (TYPEDEF FOR C COMPATIBILITY)
// ==========================================

typedef struct {
    volatile long ticks;      // 'volatile' because it changes in ISR
    long last_ticks;
    float integral_sum;
    float last_error;
} Motor_State;

// Global Instances
Motor_State motor_left = {0, 0, 0.0f, 0.0f};
Motor_State motor_right = {0, 0, 0.0f, 0.0f};

// ==========================================
// 3. INTERRUPT SERVICE ROUTINES
// ==========================================

void isr_left() {
    // Standard C: digitalRead is slow but safe; direct port manipulation is faster but harder to read
    if (digitalRead(ENC_L_B) == LOW) {
        motor_left.ticks++;
    } else {
        motor_left.ticks--;
    }
}

void isr_right() {
    if (digitalRead(ENC_R_B) == LOW) {
        motor_right.ticks++;
    } else {
        motor_right.ticks--;
    }
}

// ==========================================
// 4. LOW-LEVEL MOTOR DRIVER (C STYLE)
// ==========================================

void set_motor_raw(int pwm_pin, int dir1_pin, int dir2_pin, int speed) {
    // Speed clamping
    if (speed > 255) speed = 255;
    if (speed < -255) speed = -255;

    if (speed > 0) {
        digitalWrite(dir1_pin, HIGH);
        digitalWrite(dir2_pin, LOW);
        analogWrite(pwm_pin, speed);
    } 
    else if (speed < 0) {
        digitalWrite(dir1_pin, LOW);
        digitalWrite(dir2_pin, HIGH);
        analogWrite(pwm_pin, -speed); // PWM must be positive
    } 
    else {
        digitalWrite(dir1_pin, LOW);
        digitalWrite(dir2_pin, LOW);
        analogWrite(pwm_pin, 0);
    }
}

// ==========================================
// 5. PID CALCULATION HELPER
// ==========================================

// Function receives a POINTER to the struct to modify it directly
int compute_pid_output(Motor_State *m, float target_rpm, float current_rpm, float dt_sec) {
    // 1. Calculate Error
    float error = target_rpm - current_rpm;

    // 2. Integral Term (Accumulate error)
    m->integral_sum += (error * dt_sec);

    // 3. Derivative Term (Rate of change of error)
    float derivative = (error - m->last_error) / dt_sec;

    // 4. Update last error for next cycle
    m->last_error = error;

    // 5. Calculate final PWM output
    float output = (KP * error) + (KI * m->integral_sum) + (KD * derivative);

    return (int)output;
}

// ==========================================
// 6. MAIN MOVEMENT API
// ==========================================

void move_forward(float distance_mm, float speed_rpm) {
    long target_total_ticks = (long)(distance_mm * TICKS_PER_MM);
    
    // Reset Motor States
    noInterrupts();
    motor_left.ticks = 0;
    motor_right.ticks = 0;
    interrupts();
    
    // Reset PID Memory
    motor_left.last_ticks = 0;
    motor_left.integral_sum = 0.0f;
    motor_left.last_error = 0.0f;
    
    motor_right.last_ticks = 0;
    motor_right.integral_sum = 0.0f;
    motor_right.last_error = 0.0f;

    unsigned long last_time = millis();
    
    // Blocking Loop: Runs until robot reaches distance
    while ((abs(motor_left.ticks) + abs(motor_right.ticks)) / 2 < target_total_ticks) {
        
        unsigned long now = millis();
        unsigned long dt_ms = now - last_time;

        // Sampling Rate: 20ms (50Hz)
        if (dt_ms >= 20) { 
            float dt_sec = dt_ms / 1000.0f;
            last_time = now;

            // --- 1. GET CURRENT SPEEDS ---
            // Calculate change in ticks
            long delta_L = motor_left.ticks - motor_left.last_ticks;
            long delta_R = motor_right.ticks - motor_right.last_ticks;
            
            // Update history
            motor_left.last_ticks = motor_left.ticks;
            motor_right.last_ticks = motor_right.ticks;

            // Convert to RPM
            float rpm_L = (delta_L / dt_sec) * (60.0f / MOTOR_CPR);
            float rpm_R = (delta_R / dt_sec) * (60.0f / MOTOR_CPR);

            // --- 2. SYNC LOGIC (Keep robot straight) ---
            long diff_ticks = abs(motor_left.ticks) - abs(motor_right.ticks);
            float sync_adj = diff_ticks * 2.0f; // Correction Factor

            float target_L = speed_rpm - sync_adj;
            float target_R = speed_rpm + sync_adj;

            // --- 3. COMPUTE PID ---
            // Pass addresses of motor structs using '&'
            int pwm_L = compute_pid_output(&motor_left, target_L, rpm_L, dt_sec);
            int pwm_R = compute_pid_output(&motor_right, target_R, rpm_R, dt_sec);

            // --- 4. ACTUATE ---
            set_motor_raw(PIN_LEFT_PWM, PIN_LEFT_DIR1, PIN_LEFT_DIR2, pwm_L);
            set_motor_raw(PIN_RIGHT_PWM, PIN_RIGHT_DIR1, PIN_RIGHT_DIR2, pwm_R);
        }
    }

    // Stop and Brake
    set_motor_raw(PIN_LEFT_PWM, PIN_LEFT_DIR1, PIN_LEFT_DIR2, 0);
    set_motor_raw(PIN_RIGHT_PWM, PIN_RIGHT_DIR1, PIN_RIGHT_DIR2, 0);
    
    delay(200); // Wait for inertia to settle
}

// ==========================================
// 7. SENSOR APIs (Boolean Return)
// ==========================================

bool API_wallFront() {
    int val = analogRead(PIN_IR_FRONT);
    return (val <= 500) ? true : false;
}

bool API_wallLeft() {
    int val = analogRead(PIN_IR_LEFT);
    return (val <= 500) ? true : false;
}

bool API_wallRight() {
    int val = analogRead(PIN_IR_RIGHT);
    return (val <= 500) ? true : false;
}

// ==========================================
// 8. SETUP & LOOP
// ==========================================

void setup() {
    Serial.begin(9600);

    // Pin Setup
    pinMode(PIN_LEFT_PWM, OUTPUT);
    pinMode(PIN_LEFT_DIR1, OUTPUT);
    pinMode(PIN_LEFT_DIR2, OUTPUT);
    
    pinMode(PIN_RIGHT_PWM, OUTPUT);
    pinMode(PIN_RIGHT_DIR1, OUTPUT);
    pinMode(PIN_RIGHT_DIR2, OUTPUT);
    
    pinMode(PIN_STBY, OUTPUT);

    pinMode(PIN_IR_FRONT, INPUT);
    pinMode(PIN_IR_LEFT, INPUT);
    pinMode(PIN_IR_RIGHT, INPUT);

    pinMode(ENC_L_A, INPUT_PULLUP);
    pinMode(ENC_L_B, INPUT_PULLUP);
    pinMode(ENC_R_A, INPUT_PULLUP);
    pinMode(ENC_R_B, INPUT_PULLUP);

    // Initialize Interrupts
    attachInterrupt(digitalPinToInterrupt(ENC_L_A), isr_left, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_R_A), isr_right, RISING);

    // Turn on Motor Driver
    digitalWrite(PIN_STBY, HIGH);
    delay(     );
