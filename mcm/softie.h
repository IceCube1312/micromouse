#ifndef FLOODFILL_H
#define FLOODFILL_H

#define MAZE_HEIGHT 16
#define MAZE_WIDTH 16
#define MAX_SIZE 256
#define INF 999

extern int TARGET_ROW;    
extern int TARGET_COL;
extern int START_ROW;
extern int START_COL;
extern char START_DIRECTION;
extern int CURRENT_ROW;
extern int CURRENT_COL;
extern char CURRENT_DIRECTION;

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

void position_MAZE_init(CELL MAZE[MAZE_HEIGHT][MAZE_WIDTH]);
void queue_init(QUEUE* queue);
void enqueue(QUEUE* queue,int r, int c);
void dequeue(QUEUE* queue);
void floodfill(CELL MAZE[MAZE_HEIGHT][MAZE_WIDTH]);
void wall_check(CELL MAZE[MAZE_HEIGHT][MAZE_WIDTH],int row,int col,char direction);
void turn(int row, int col, char *direction, CELL MAZE[MAZE_HEIGHT][MAZE_WIDTH]);
void move_one(int* CURRENT_ROW, int* CURRENT_COL,char direction,CELL MAZE[MAZE_HEIGHT][MAZE_WIDTH]);

#endif
