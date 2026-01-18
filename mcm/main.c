#include <stdio.h>
#include <stdlib.h>
#include "softie.h"
#include "API.h"

int main(){
        CELL MAZE[MAZE_HEIGHT][MAZE_WIDTH];
        position_MAZE_init(MAZE);

        while(true){
        wall_check(MAZE,CURRENT_ROW,CURRENT_COL,CURRENT_DIRECTION);

        floodfill(MAZE);

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
