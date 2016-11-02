#include "Arduino.h"
#include <EEPROM.h>


/* =====================================================================================
                                    Flood and Fill
====================================================================================== */

/*  This library contains the Artificial intelligence code that solves the maze.
    There are 3 Functions.

    floodMaze();
    By Peter Harrison receives the maze and the goal place and floods the maze and modifies the "maps" array
    with  the flood information.

    generatePath();
    Takes the Flooded maze and the start position, and returns a char array "outpath", with the commands for forwards and turns:
        F -> Move forward one cell
        L -> In cell turn to the left
        R -> In cell turn to the right
        G -> In place 180 degree turn
        S -> Stop, the path is finished
    example: "FFLRFS"

    floodAndFill();
    This is an encapsulation funtion that calls the last two functions and returns the path

    saveMaze();
    This function saves the current maze to the EEPROM, to the first 256 address

    restoreMaze();
    This function reads the maze from the first 256 address of the EEPROM and stores it in to the maze array.

*/

#define NORTH      1
#define EAST       2
#define SOUTH      4
#define WEST       8
#define VISITED		16
#define ONROUTE		32

#define NOTNORTH  (255-NORTH)
#define NOTEAST   (255-EAST)
#define NOTSOUTH  (255-SOUTH)
#define NOTWEST   (255-WEST)
#define ALLWALLS  (NORTH|EAST|SOUTH|WEST)

// maze storage
#define NUMCELLS 256
//We predifine the 10 by 10 labyrinth
uint8_t  maze[NUMCELLS] = { 12, 8, 8, 8, 8, 8, 8, 8, 8, 9, 0, 0, 0, 0, 0, 0,             // NORTH -->
                            4, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
                            4, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
                            4, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
                            4, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
                            4, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
                            4, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
                            4, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
                            4, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
                            6, 2, 2, 2, 2, 2, 2, 2, 2, 3, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

uint8_t  maps[NUMCELLS];

// Path generator variable
#define MAX_PATH_LENGTH 150
char path[MAX_PATH_LENGTH];




/*
  floodMaze solves the maze using wall data found in the array maze
  and places the flooding data - effectively the routemap - into
  the array map. The destination cell is passed in to the function
  to allow solving for any location.
  The state of map on entry is not important and it is not important
  whether all the walls have been found. The algorithm simply performs
  the flood based on what it knows at the time it is called.
  At worst, up to 255 passes are made through the maze date in generating
  the map. If that many passes are made, there has been an error and
  the maze should be considered to be seriously in error.
  Passes of the algorithm are made until either the limit has been reached
  or the change flag indicates that none of the map data has changed in
  that pass. At that point the maze can be considered to have been solved.
  Clearly, unless all the walls have been observed, the map will contain
  invalid paths. The mouse will discover new walls and call the solver again
  as needed.
  The function returns the number of passes required to solve the maze.
  A returned value of 255 indicates an error.
  Note that the array is allowed to wrap around in a most dangerous
  manner by relying on the 8 bit pointer.
  This should not be a real problem in as much as the existence of the outer
  walls means we never have to make use of the value
*/


uint8_t floodMaze(uint8_t goal_x, uint8_t goal_y)
{
    uint8_t i,j;
    uint8_t now,next;
    uint8_t passes;
    uint8_t cellwalls;    // the wall data for a given cell
    uint8_t changed;


    i = 0;
    do{
        maps[i--] = 255;
    } while (i);


    maps[goal_x + goal_y*16]=0;
    passes = 0;
    now = 0;
    next = now+1;


    do
    {
        changed = 0;
        i = 0;
        do
        {
            if (maps[i]==now)
            {
                cellwalls=maze[i];
                if ((cellwalls & NORTH) == 0)
                {
                    j = i+1;
                    if (maps[j] == 255){ maps[j] = next; changed=1;}
                }
                if ((cellwalls & EAST) == 0)
                {
                    j = i + 16;
                    if (maps[j] == 255){ maps[j] = next; changed=1;}
                }
                if ((cellwalls & SOUTH) == 0)
                {
                    j = i + 255;
                    if (maps[j] == 255){ maps[j] = next; changed=1;}
                }
                if ((cellwalls & WEST) == 0)
                {
                    j = i + 240;
                    if (maps[j] == 255){ maps[j] = next; changed=1;}
                }
            }
            i--;
        } while(i);
        now  = now+1;
        next = now+1;
        passes++;
    } while(changed);
    return passes;
}


void generatePath(uint8_t start_x, uint8_t start_y, char* out_path){

    // Current investigated cell, index of the char path .
    uint8_t current_cell, index = 0;
    // Variables to check where should I move.
    uint8_t go_north=0, go_east=0, go_west=0, go_south=0;
    char orientation = 'S';    // 'N' = north, 'W' = west, 'E' = east, 'S' = south.

    // Start at the goal cell and work your way toward the zero square
    current_cell = start_x + 16*start_y;

    // iterate until your reached
    while(maps[current_cell] > 0){

        // // Printing code
        // digitalWrite(13, HIGH);
        // delay(100);
        // digitalWrite(13, LOW);
        // delay(100);
        //
        // int x, y;
        // y = current_cell/16;
        // x = current_cell - y*16;
        // Serial.print("("); Serial.print(x); Serial.print(","); Serial.print(y);  Serial.print(")"); Serial.print("\t"); Serial.print(maps[current_cell]);
        // Serial.print("\t"); Serial.println(current_cell);


        // Reset check marks
        go_north = 0;
        go_east =  0;
        go_west = 0;
        go_south = 0;



        // check North to see if you should move there
        if (maps[current_cell + 1 ] == maps[current_cell] - 1 )  go_north = 1;

        // check East to see if you should move there
        if (maps[current_cell + 16 ] == maps[current_cell] - 1 )  go_east = 1;

        // check West to see if you should move there, and check if you can actually access that part of the array.
        if (current_cell >= 16)
        if (maps[current_cell - 16 ] == maps[current_cell] - 1 )  go_west = 1;

        // check South to see if you should move there, and check if you can actually access that part of the array.
        if (current_cell >= 0)
        if (maps[current_cell - 1 ] == maps[current_cell] - 1 )  go_south = 1;



        // Decide where to go based on orientation to avoid too many turns
        // If you are going North, prioritize north movement by checking it first.
        if (orientation == 'N'){

            // Check if you should move north
            if (go_north) {
                // Update orientation of the robot
                orientation = 'N';
                out_path[index++] = 'F';
                // move the robot to the corresponding cell
                current_cell += 1;
            }
            // Check if you should go East
            else if (go_east) {
                // Update orientation of the robot
                orientation = 'E';
                out_path[index++] = 'R';
                // move the robot to the corresponding cell
                current_cell += 16;
            }
            // Check if you should go West
            else if (go_west) {
                // Update orientation of the robot
                orientation = 'W';
                out_path[index++] = 'L';
                // move the robot to the corresponding cell
                current_cell -= 16;
            }
            // Lastly, check if you have to do an U turn.
            else if (go_south) {
                // Update orientation of the robot
                orientation = 'S';
                out_path[index++] = 'G'; //Turn 180 degrees on your own axis
                out_path[index++] = 'F';
                // move the robot to the corresponding cell
                current_cell -= 1;
            }
        }

        // If you are going East, prioritize north movement by checking it first.
        if (orientation == 'E'){

            // Check if you should move East
            if (go_east) {
                // Update orientation of the robot
                orientation = 'E';
                out_path[index++] = 'F';
                // move the robot to the corresponding cell
                current_cell += 16;
            }
            // Check if you should go South
            else if (go_south) {
                // Update orientation of the robot
                orientation = 'S';
                out_path[index++] = 'R';
                // move the robot to the corresponding cell
                current_cell -= 1;
            }
            // Check if you should go North
            else if (go_north) {
                // Update orientation of the robot
                orientation = 'N';
                out_path[index++] = 'L';
                // move the robot to the corresponding cell
                current_cell += 1;
            }
            // Lastly, check if you have to do an U turn.
            else if (go_west) {
                // Update orientation of the robot
                orientation = 'W';
                out_path[index++] = 'G'; //Turn 180 degrees on your own axis
                out_path[index++] = 'F';
                // move the robot to the corresponding cell
                current_cell -= 16;
            }
        }

        // If you are going East, prioritize north movement by checking it first.
        if (orientation == 'W'){

            // Check if you should move West
            if (go_west) {
                // Update orientation of the robot
                orientation = 'W';
                out_path[index++] = 'F';
                // move the robot to the corresponding cell
                current_cell -= 16;
            }
            // Check if you should go North
            else if (go_north) {
                // Update orientation of the robot
                orientation = 'N';
                out_path[index++] = 'R';
                // move the robot to the corresponding cell
                current_cell += 1;
            }
            // Check if you should go South
            else if (go_south) {
                // Update orientation of the robot
                orientation = 'S';
                out_path[index++] = 'L';
                // move the robot to the corresponding cell
                current_cell -= 1;
            }
            // Lastly, check if you have to do an U turn.
            else if (go_east) {
                // Update orientation of the robot
                orientation = 'E';
                out_path[index++] = 'G'; //Turn 180 degrees on your own axis
                out_path[index++] = 'F';
                // move the robot to the corresponding cell
                current_cell += 16;
            }
        }

        // If you are going South, prioritize north movement by checking it first.
        if (orientation == 'S'){

            // Check if you should move West
            if (go_south) {
                // Update orientation of the robot
                orientation = 'S';
                out_path[index++] = 'F';
                // move the robot to the corresponding cell
                current_cell -= 1;
            }
            // Check if you should go West
            else if (go_west) {
                // Update orientation of the robot
                orientation = 'W';
                out_path[index++] = 'R';
                // move the robot to the corresponding cell
                current_cell -= 16;
            }
            // Check if you should go East
            else if (go_east) {
                // Update orientation of the robot
                orientation = 'E';
                out_path[index++] = 'L';
                // move the robot to the corresponding cell
                current_cell += 16;
            }
            // Lastly, check if you have to do an U turn.
            else if (go_north) {
                // Update orientation of the robot
                orientation = 'N';
                out_path[index++] = 'G'; //Turn 180 degrees on your own axis
                out_path[index++] = 'F';
                // move the robot to the corresponding cell
                current_cell += 1;
            }
        }

    }

    // When you have reached your destination, stop the path.
    out_path[index++] = 'S';
    out_path[index++] = '\0';
}

// This is the public funcion of the library, it runs the maze flooder
void floodAndFill(uint8_t start_x, uint8_t start_y, uint8_t goal_x, uint8_t goal_y, char* out_path){

    int passes;

    // resets the path
    for (size_t i = 0; i < MAX_PATH_LENGTH; i++) {
        out_path[i] = 0;
    }

    //Run the flooder
    passes = floodMaze(goal_x,goal_y);
    generatePath(start_x,start_y,out_path);

}

// This function saves the current maze to the EEPROM, to the first 256 address
void saveMaze(void){

    for (size_t i = 0; i < NUMCELLS; i++) {
        EEPROM.write(i, maze[i]);
    }
}

// This function reads the maze from the first 256 address of the EEPROM and stores it in to the maze array.
void restoreMaze(void){

    for (size_t i = 0; i < NUMCELLS; i++) {
        maze[i] = EEPROM.read(i);
    }
}
