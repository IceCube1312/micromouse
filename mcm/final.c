#include "softie.h"
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

#define INF 999
#define MAX_SIZE 256

const int MAZE_HEIGHT=16;
const int MAZE_WIDTH=16;
const int TARGET_ROW=7;      //target cell -1 because of array indexing
const int TARGET_COL=7;
const int START_ROW=0;
const int START_COL=0;
char START_DIRECTION='n';
int CURRENT_ROW;
int CURRENT_COL;
char CURRENT_DIRECTION;

// ==========================================
// 2. STRUCTS (TYPEDEF FOR C COMPATIBILITY)
// ==========================================

typedef struct {
    volatile long ticks;      // 'volatile' because it changes in ISR
    long last_ticks;
    float integral_sum;
    float last_error;
} Motor_State;

typedef struct cell{
	int DIST;
	bool NORTH_WALL;
	bool SOUTH_WALL;
	bool WEST_WALL;
	bool EAST_WALL;
}CELL;

typedef struct element{
	int row;
	int col;
}ELEMENT;

typedef struct queue{
	ELEMENT array[MAX_SIZE];
	int head;
	int tail;
	int size;
}QUEUE;

void position_MAZE_init(CELL MAZE[MAZE_HEIGHT][MAZE_WIDTH]){
	CURRENT_ROW=START_ROW;
	CURRENT_COL=START_COL;
	CURRENT_DIRECTION=START_DIRECTION;
        for(int i=0; i<MAZE_HEIGHT; i++){
               for(int j=0; j<MAZE_WIDTH; j++){
                      MAZE[i][j].NORTH_WALL = false;
                      MAZE[i][j].SOUTH_WALL = false;
                      MAZE[i][j].WEST_WALL = false;
                      MAZE[i][j].EAST_WALL = false;
               }
        }
}
void queue_init(QUEUE* queue){
	queue->head=0;
	queue->tail=0;
	queue->size=0;
}

void enqueue(QUEUE* queue,int r, int c){
	if(queue->size==MAX_SIZE){
		return;
	}
	queue->array[queue->tail].row=r;
	queue->array[queue->tail].col=c;

	queue->tail = (queue-> tail+1) % MAX_SIZE;
	queue->size ++;
}

void dequeue(QUEUE* queue){
	if(queue->size==0){
		return;
	}
	queue->head=(queue->head+1)%MAX_SIZE;
	queue->size--;
}

void floodfill(CELL MAZE[MAZE_HEIGHT][MAZE_WIDTH]){
	QUEUE queue;
	queue_init(&queue);
	MAZE[TARGET_ROW][TARGET_COL].DIST=0;
        for(int i=0;i<MAZE_HEIGHT;i++){
                for(int j=0;j<MAZE_WIDTH;j++){
                        if((i==TARGET_ROW)&&(j==TARGET_COL)){   //assigning inf to everything else not visited
                                continue;
                        }
                        MAZE[i][j].DIST=INF;
                }
        }
	int r;
	int c;
	enqueue(&queue,TARGET_ROW,TARGET_COL);
        while(true){
		r=queue.array[queue.head].row;
		c=queue.array[queue.head].col;
                if(queue.size!=0){
                        assert(MAZE[r][c].DIST!=INF);
                        if((r<MAZE_HEIGHT-1)&&(!MAZE[r][c].NORTH_WALL)&&(MAZE[r+1][c].DIST==INF) ){
                                enqueue(&queue,r+1,c);
                                MAZE[r+1][c].DIST=MAZE[r][c].DIST+1 ;
                        }
                        if((r>0)&&(!MAZE[r][c].SOUTH_WALL)&&(MAZE[r-1][c].DIST==INF) ){
                                enqueue(&queue,r-1,c);
                                MAZE[r-1][c].DIST=MAZE[r][c].DIST+1 ;
                        }
                        if((c<MAZE_WIDTH-1)&&(!MAZE[r][c].EAST_WALL)&&(MAZE[r][c+1].DIST==INF) ){
                                enqueue(&queue,r,c+1);
                                MAZE[r][c+1].DIST=MAZE[r][c].DIST+1 ;
                        }
                        if((c>0)&&(!MAZE[r][c].WEST_WALL)&&(MAZE[r][c-1].DIST==INF) ){
                                enqueue(&queue,r,c-1);
                                MAZE[r][c-1].DIST=MAZE[r][c].DIST+1 ;
                        }
                        dequeue(&queue);

                }
                else{
                        break;
                }
        }
}
void wall_check(CELL MAZE[MAZE_HEIGHT][MAZE_WIDTH],int row,int col,char direction){
	if (API_wallFront()){          //checking walls in all three directions and updating the wall in both cells only if there is an adjacent cell
		switch(direction){
			case 'n':
				MAZE[row][col].NORTH_WALL=true;
				if (row < MAZE_HEIGHT-1) MAZE[row+1][col].SOUTH_WALL = true;
				 
				break;
                        case 's':
                                MAZE[row][col].SOUTH_WALL=true;
				if (row >0) MAZE[row-1][col].NORTH_WALL = true;
                                
                                break;
                        case 'w':
                                MAZE[row][col].WEST_WALL=true;
                                if (col >0) MAZE[row][col-1].EAST_WALL = true;
                                
                                break;
                        case 'e':
                                MAZE[row][col].EAST_WALL=true;
                                if (col < MAZE_WIDTH-1) MAZE[row][col+1].WEST_WALL = true;
                              
                                break;
		}
	}
	if (API_wallLeft()){
                switch(direction){
                        case 'e':
                                MAZE[row][col].NORTH_WALL=true;
                                if (row < MAZE_HEIGHT-1) MAZE[row+1][col].SOUTH_WALL = true;
                                
                                break;
                        case 'w':
                                MAZE[row][col].SOUTH_WALL=true;
                                if (row >0) MAZE[row-1][col].NORTH_WALL = true;
                                
                                break;
                        case 'n':
                                MAZE[row][col].WEST_WALL=true;
                                if (col >0) MAZE[row][col-1].EAST_WALL = true;
                                
                                break;
                        case 's':
                                MAZE[row][col].EAST_WALL=true;
                                if (col < MAZE_WIDTH-1) MAZE[row][col+1].WEST_WALL = true;
                                break;
                }
        }
	if (API_wallRight()){
                switch(direction){
                        case 'w':
                                MAZE[row][col].NORTH_WALL=true;
                                if (row < MAZE_HEIGHT-1) MAZE[row+1][col].SOUTH_WALL = true;
                                
                                break;
                        case 'e':
                                MAZE[row][col].SOUTH_WALL=true;
                                if (row >0) MAZE[row-1][col].NORTH_WALL = true;
                                
                                break;
                        case 's':
                                MAZE[row][col].WEST_WALL=true;
                                if (col >0) MAZE[row][col-1].EAST_WALL = true;
                                
                                break;
                        case 'n':
                                MAZE[row][col].EAST_WALL=true;
                                if (col < MAZE_WIDTH-1) MAZE[row][col+1].WEST_WALL = true;
                                
                                break;
                }
        }

}
void turn(int row, int col, char *direction, CELL MAZE[MAZE_HEIGHT][MAZE_WIDTH]){
	int min_dist=MAZE[row][col].DIST;
	char target=*direction;

	if( (row<MAZE_HEIGHT-1) && (!MAZE[row][col].NORTH_WALL) && (MAZE[row+1][col].DIST < min_dist) ){
		target='n';
		min_dist=MAZE[row+1][col].DIST;
	}
	if( (row>0) && (!MAZE[row][col].SOUTH_WALL) && (MAZE[row-1][col].DIST < min_dist) ){
		target='s';
		min_dist=MAZE[row-1][col].DIST;
	}
	if( (col<MAZE_WIDTH-1) && (!MAZE[row][col].EAST_WALL) && (MAZE[row][col+1].DIST < min_dist) ){
		target='e';
		min_dist=MAZE[row][col+1].DIST;
	}
	if( (col>0) && (!MAZE[row][col].WEST_WALL) && (MAZE[row][col-1].DIST < min_dist) ){
		target='w';
		min_dist=MAZE[row][col-1].DIST;
	}

	if (*direction == target) return;

    bool is_right = (*direction == 'n' && target == 'e') ||
                    (*direction == 'e' && target == 's') ||
                    (*direction == 's' && target == 'w') ||
                    (*direction == 'w' && target == 'n');

    bool is_left = (*direction == 'n' && target == 'w') ||
                   (*direction == 'w' && target == 's') ||
                   (*direction == 's' && target == 'e') ||
                   (*direction == 'e' && target == 'n');

    if (is_right) {
        API_turnRight();
    } 
    else if (is_left) {
        API_turnLeft(); 
    } 
    else {           //u-turn
        API_turnRight();
        API_turnRight();
    }

    *direction = target;  //updating the direction variable
}

void move_one(int* CURRENT_ROW, int* CURRENT_COL,char direction,CELL MAZE[MAZE_HEIGHT][MAZE_WIDTH]){
	if(direction=='n'){
		if( (!MAZE[*CURRENT_ROW][*CURRENT_COL].NORTH_WALL) && (MAZE[*CURRENT_ROW][*CURRENT_COL].DIST > MAZE[*CURRENT_ROW+1][*CURRENT_COL].DIST) ){
			API_moveForward(200,20);
			(*CURRENT_ROW)++;
		}
	}
	else if(direction=='s'){
		if( (!MAZE[*CURRENT_ROW][*CURRENT_COL].SOUTH_WALL) && (MAZE[*CURRENT_ROW][*CURRENT_COL].DIST > MAZE[*CURRENT_ROW-1][*CURRENT_COL].DIST) ){
			API_moveForward(200,20);
			(*CURRENT_ROW)--;
		}
	}
	else if(direction=='e'){
		if( (!MAZE[*CURRENT_ROW][*CURRENT_COL].EAST_WALL) && (MAZE[*CURRENT_ROW][*CURRENT_COL].DIST > MAZE[*CURRENT_ROW][*CURRENT_COL+1].DIST) ){
			API_moveForward(200,20);
			(*CURRENT_COL)++;
		}
	}
	else if(direction=='w'){
		if( (!MAZE[*CURRENT_ROW][*CURRENT_COL].WEST_WALL) && (MAZE[*CURRENT_ROW][*CURRENT_COL].DIST > MAZE[*CURRENT_ROW][*CURRENT_COL-1].DIST) ){
			API_moveForward(200,20);
			(*CURRENT_COL)--;
		}
	}
}


// Global Instances
Motor_State motor_left = {0, 0, 0.0f, 0.0f};
Motor_State motor_right = {0, 0, 0.0f, 0.0f};

// ==========================================
// 3. INTERRUPT SERVICE ROUTINES
// ==========================================
int readavg(int input_pin){
  int sum =0;
  for(int i =1;i<=10;i++){
    sum+=analogRead(input_pin);
  }
  return sum/10;
}
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

void API_moveForward(float distance_mm, float speed_rpm) {
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
void API_turnRight() {
    // Left wheel forward, Right wheel backward
    digitalWrite(PIN_LEFT_DIR1,HIGH);
    digitalWrite(PIN_LEFT_DIR2,LOW);
    analogWrite(PIN_LEFT_PWM,100);
    digitalWrite(PIN_RIGHT_DIR1,LOW);
    digitalWrite(PIN_RIGHT_DIR2,HIGH;
    analogWrite(PIN_RIGHT_PWM,100);
    delay(1000);
    
}

void API_turnLeft() {
    // Left wheel backward, Right wheel forward
    digitalWrite(PIN_RIGHT_DIR1,HIGH);
    digitalWrite(PIN_RIGHT_DIR2,LOW);
    analogWrite(PIN_RIGHT_PWM,100);
    digitalWrite(PIN_LEFT_DIR1,LOW);
    digitalWrite(PIN_LEFT_DIR2,HIGH;
    analogWrite(PIN_LEFT_PWM,100);
    delay(1000);
}

// ==========================================
// 7. SENSOR APIs (Boolean Return)
// ==========================================

bool API_wallFront() {

    int val= readavg(PIN_IR_FRONT);
    return (val <= 500) ? true : false;
}

bool API_wallLeft() {
    int val = readavg(PIN_IR_LEFT);
    return (val <= 500) ? true : false;
}

bool API_wallRight() {
    int val = readavg(PIN_IR_RIGHT);
    return (val <= 500) ? true : false;
}
CELL MAZE[MAZE_HEIGHT][MAZE_WIDTH];
// ==========================================
// 8. SETUP & LOOP
// ==========================================

void setup() {
    Serial.begin(9600);
    
	  position_MAZE_init(MAZE);
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
    delay(    );
}
void loop(){
  wall_check(MAZE,CURRENT_ROW,CURRENT_COL,CURRENT_DIRECTION);
  floodfill(MAZE);
  turn(CURRENT_ROW,CURRENT_COL,&CURRENT_DIRECTION,MAZE);
  move_one(&CURRENT_ROW,&CURRENT_COL,CURRENT_DIRECTION,MAZE);
}
