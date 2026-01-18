#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <assert.h>
#include "API.h"

#define INF 999
#define MAX_SIZE 256

int MAZE_HEIGHT=16;
int MAZE_WIDTH=16;
int TARGET_ROW=7;      //target cell -1 because of array indexing
int TARGET_COL=7;
int START_ROW=0;
int START_COL=0;
char START_DIRECTION='n';

typedef struct cell{
	int DIST;
	bool NORTH_WALL;
	bool SOUTH_WALL;
	bool WEST_WALL;
	bool EAST_WALL;
}CELL;

/*
typedef struct queue{
	int ROW;
	int COL;
	struct queue* next;
}QUEUE;


//adding element to the last
void enqueue(QUEUE** head,QUEUE** tail,int row, int col){
	QUEUE* new=malloc(sizeof(QUEUE));
	new->next=NULL;
	new->ROW=row;
	new->COL=col;
	if(*head==NULL){
		(*head)=new;
		(*tail)=new;
	}
	else{
		(*tail)->next=new;
		(*tail)=new;
	}
}

//removing first element
void dequeue(QUEUE** head, QUEUE** tail){
	if(*head==NULL){
		return;
	}
	QUEUE* temp=*head;
	(*head)=(*head)->next;
	if(*head==NULL){
		*tail=NULL;
	}
	free(temp);
}
*/
//will use static queues instead of linked lists because something something heap memeory and gemini said so
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

void queue_init(QUEUE* queue){
	queue->head=0;
	queue->tail=0;
	queue->size=0;
}

void enqueue(QUEUE* queue,int r, int c){
	if(queue->size==MAX_SIZE){
		return;
	}
	queue->ELEMENT[tail].row=r;
	queue->ELEMENT[tail].col=c;

	queue->tail = (queue-> tail+1) % MAX_SIZE;
	queue->size ++;
}

void dequeue(QUEUE* queue){
	if(queue->size==0){
		return;
	}
	queue->head=(queue->head+1)%MAX_SIZE;
}



void floodfill(CELL MAZE[MAZE_HEIGHT][MAZE_WIDTH]){

}


void wall_check(CELL MAZE[MAZE_HEIGHT][MAZE_WIDTH],int row,int col,char direction){
	if (API_wallFront()){          //checking walls in all three directions and updating the wall in both cells only if there is an adjacent cell
		switch(direction){
			case 'n':
				MAZE[row][col].NORTH_WALL=true;
				if (row < MAZE_HEIGHT-1) MAZE[row+1][col].SOUTH_WALL = true;
				API_setWall(col,row,'n');        
				break;
                        case 's':
                                MAZE[row][col].SOUTH_WALL=true;
				if (row >0) MAZE[row-1][col].NORTH_WALL = true;
                                API_setWall(col,row,'s');
                                break;
                        case 'w':
                                MAZE[row][col].WEST_WALL=true;
                                if (col >0) MAZE[row][col-1].EAST_WALL = true;
                                API_setWall(col,row,'w');
                                break;
                        case 'e':
                                MAZE[row][col].EAST_WALL=true;
                                if (row < MAZE_WIDTH-1) MAZE[row][col+1].WEST_WALL = true;
                                API_setWall(col,row,'e');
                                break;
		}
	}
	if (API_wallLeft()){
                switch(direction){
                        case 'e':
                                MAZE[row][col].NORTH_WALL=true;
                                if (row < MAZE_HEIGHT-1) MAZE[row+1][col].SOUTH_WALL = true;
                                API_setWall(col,row,'n');
                                break;
                        case 'w':
                                MAZE[row][col].SOUTH_WALL=true;
                                if (row >0) MAZE[row-1][col].NORTH_WALL = true;
                                API_setWall(col,row,'s');
                                break;
                        case 'n':
                                MAZE[row][col].WEST_WALL=true;
                                if (col >0) MAZE[row][col-1].EAST_WALL = true;
                                API_setWall(col,row,'w');
                                break;
                        case 's':
                                MAZE[row][col].EAST_WALL=true;
                                if (row < MAZE_WIDTH-1) MAZE[row][col+1].WEST_WALL = true;
                                API_setWall(col,row,'e');
                                break;
                }
        }
	if (API_wallRight()){
                switch(direction){
                        case 'w':
                                MAZE[row][col].NORTH_WALL=true;
                                if (row < MAZE_HEIGHT-1) MAZE[row+1][col].SOUTH_WALL = true;
                                API_setWall(col,row,'n');
                                break;
                        case 'e':
                                MAZE[row][col].SOUTH_WALL=true;
                                if (row >0) MAZE[row-1][col].NORTH_WALL = true;
                                API_setWall(col,row,'s');
                                break;
                        case 's':
                                MAZE[row][col].WEST_WALL=true;
                                if (col >0) MAZE[row][col-1].EAST_WALL = true;
                                API_setWall(col,row,'w');
                                break;
                        case 'n':
                                MAZE[row][col].EAST_WALL=true;
                                if (row < MAZE_WIDTH-1) MAZE[row][col+1].WEST_WALL = true;
                                API_setWall(col,row,'e');
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
			API_moveForward();
			(*CURRENT_ROW)++;
		}
	}
	else if(direction=='s'){
		if( (!MAZE[*CURRENT_ROW][*CURRENT_COL].SOUTH_WALL) && (MAZE[*CURRENT_ROW][*CURRENT_COL].DIST > MAZE[*CURRENT_ROW-1][*CURRENT_COL].DIST) ){
			API_moveForward();
			(*CURRENT_ROW)--;
		}
	}
	else if(direction=='e'){
		if( (!MAZE[*CURRENT_ROW][*CURRENT_COL].EAST_WALL) && (MAZE[*CURRENT_ROW][*CURRENT_COL].DIST > MAZE[*CURRENT_ROW][*CURRENT_COL+1].DIST) ){
			API_moveForward();
			(*CURRENT_COL)++;
		}
	}
	else if(direction=='w'){
		if( (!MAZE[*CURRENT_ROW][*CURRENT_COL].WEST_WALL) && (MAZE[*CURRENT_ROW][*CURRENT_COL].DIST > MAZE[*CURRENT_ROW][*CURRENT_COL-1].DIST) ){
			API_moveForward();
			(*CURRENT_COL)--;
		}
	}
}


int main(){
	CELL MAZE[MAZE_HEIGHT][MAZE_WIDTH];
	int CURRENT_ROW=START_ROW;
	int CURRENT_COL=START_COL;
	char CURRENT_DIRECTION=START_DIRECTION;
	for(int i=0; i<MAZE_HEIGHT; i++){
               for(int j=0; j<MAZE_WIDTH; j++){
                      MAZE[i][j].NORTH_WALL = false;
                      MAZE[i][j].SOUTH_WALL = false;
                      MAZE[i][j].WEST_WALL = false;
                      MAZE[i][j].EAST_WALL = false;
               }
        }

	while(true){
	wall_check(MAZE,CURRENT_ROW,CURRENT_COL,CURRENT_DIRECTION);
	MAZE[TARGET_ROW][TARGET_COL].DIST=0;


        for(int i=0;i<MAZE_HEIGHT;i++){
                for(int j=0;j<MAZE_WIDTH;j++){
                        if((i==TARGET_ROW)&&(j==TARGET_COL)){   //assigning inf to everything else not visited
                                continue;
                        }
                        MAZE[i][j].DIST=INF;
                }
        }

        QUEUE* head=NULL;
        QUEUE* tail=NULL;
        enqueue(&head,&tail,TARGET_ROW,TARGET_COL);

        while(true){
                if(head!=NULL){
                        assert(MAZE[head->ROW][head->COL].DIST!=INF);
                        if((head->ROW<MAZE_HEIGHT-1)&&(!MAZE[head->ROW][head->COL].NORTH_WALL)&&(MAZE[head->ROW+1][head->COL].DIST==INF) ){
                                enqueue(&head,&tail,head->ROW+1,head->COL);
                                MAZE[head->ROW+1][head->COL].DIST=MAZE[head->ROW][head->COL].DIST+1;
                        }
                        if((head->ROW>0)&&(!MAZE[head->ROW][head->COL].SOUTH_WALL)&&(MAZE[head->ROW-1][head->COL].DIST==INF) ){
                                enqueue(&head,&tail,head->ROW-1,head->COL);
                                MAZE[head->ROW-1][head->COL].DIST=MAZE[head->ROW][head->COL].DIST+1;
                        }
                        if((head->COL<MAZE_WIDTH-1)&&(!MAZE[head->ROW][head->COL].EAST_WALL)&&(MAZE[head->ROW][head->COL+1].DIST==INF) ){
                                enqueue(&head,&tail,head->ROW,head->COL+1);
                                MAZE[head->ROW][head->COL+1].DIST=MAZE[head->ROW][head->COL].DIST+1;
                        }
                        if((head->COL>0)&&(!MAZE[head->ROW][head->COL].WEST_WALL)&&(MAZE[head->ROW][head->COL-1].DIST==INF) ){
                                enqueue(&head,&tail,head->ROW,head->COL-1);
                                MAZE[head->ROW][head->COL-1].DIST=MAZE[head->ROW][head->COL].DIST+1;
                        }

                        dequeue(&head,&tail);

                }
                else{
                        break;
                }
        }

	// Visualization Loop
        char buffer[5]; // Buffer for up to 4 digits + null
        for(int i=0; i<MAZE_HEIGHT; i++) {
               for(int j=0; j<MAZE_WIDTH; j++) {
                        // Convert integer DIST to string
                        if (MAZE[i][j].DIST == INF) {
                        sprintf(buffer, "INF");
               }else{
                        sprintf(buffer, "%d", MAZE[i][j].DIST);
                    }  

                // Pass string to API (adjust coordinate order if necessary: x,y or row,col)
               API_setText(j, i, buffer);
               }  
        }  

	turn(CURRENT_ROW,CURRENT_COL,&CURRENT_DIRECTION,MAZE);
	move_one(&CURRENT_ROW,&CURRENT_COL,CURRENT_DIRECTION,MAZE);
	}
	return 0;
}
