#include "icarus_pinout.h"
#include "Teensy_Encoder.h"

//USBBOTS2016_Icarus.ino

float dist[4];
// uint16_t dist[4];
// uint16_t irval[4];

extern float posErrorX, posErrorW;
extern float posPwmX, posPwmW;
extern int32_t leftBaseSpeed;
extern int32_t rightBaseSpeed;
extern float AngularSpeed;
#define calibrator 1 // multiplica los 180mm de una celda para que en la realidad recorra el valor correcto.
// #define calibrator 0.6 // multiplica los 180mm de una celda para que en la realidad recorra el valor correcto.
#define calibratorA 0.6 // multiplica los 180mm de una celda para que en la realidad recorra el valor correcto.

// Variables para chequear el flood and fill
#define NUMCELLS 256
extern uint8_t  maze[NUMCELLS];
extern uint8_t  maps[NUMCELLS];

// Path generator variable
#define MAX_PATH_LENGTH 150
// char path_s[MAX_PATH_LENGTH];
char path_s[] = "FS";

uint8_t cell_x, cell_y;
uint8_t goal_x, goal_y;
char next_step;
uint8_t start = 0;


// Check old walls
uint8_t old_wall_left = 1;
uint8_t old_wall_right = 1;


// Variables finales de estado del robot
uint8_t mapeo, corrida;


// External access for distance
extern float ir_sensor_distances[4];
extern float sensorError;

// Sequencer variables
uint8_t done = 0;
uint8_t path_index = 0;

void setup() {

  pinMode(BTN_SPEED, INPUT);   //Safe mode for onboard led
  wifiSetup();
  irSensorSetup();
  motorSetup();
  gyroSetup();
  beginController();

  motorLeftWrite(0);
  motorRightWrite(0);
  Serial.begin(115200);



  // setDistanceLeft(2000);
  // setSetPointV(3000);
  // setSetPointW(0);
  setMouseOrientation('N');
  goal_x = 4;
  goal_y = 4;

  cell_x = 0;
  cell_y = 0;


 Serial2.println("Esperando...");
  // Si se presiona el boton empieza el mapeo.
  while(digitalRead(BTN_SPEED));

  // mapeo = 1;
  corrida = 1;

  Serial2.println("Empezo el recorrido");
  delay(3000);

}  // put your setup code here, to run once:


uint8_t data;
uint8_t index_s;

void loop() {


    //
    // Serial2.print("distanceLeft = ");
    // Serial2.print(getDistanceLeft());
    // Serial2.print("   ");
    //
    // Serial2.print("angleLeft = ");
    // Serial2.print(getAngleLeft());
    // Serial2.print("   ");
    //
    // Serial2.print("posErrorW = ");
    // Serial2.print(posErrorW);
    // Serial2.print("   ");
    //
    // Serial2.print("posPwmX = ");
    // Serial2.print(posPwmX);
    // Serial2.print("   ");
    //
    // Serial2.print("posPwmW = ");
    // Serial2.print(posPwmW);
    // Serial2.print("   ");
    //
    // Serial2.print("PWM_L = ");
    // Serial2.print(leftBaseSpeed);
    // Serial2.print("   ");
    //
    // Serial2.print("PWM_R = ");
    // Serial2.print(rightBaseSpeed);
    // Serial2.print("   ");
    //
    // Serial2.print("Vmean = ");
    // Serial2.print(getVmean());
    // Serial2.println("mm/s    ");
    //
    //
    // if (getDistanceLeft() == 0) setSetPointV(0);
    //
    //
    // if (abs(getAngleLeft()) < 10.0 ) {
    //     setSetPointW(0);
    //     // stopController();
    //     // stopIRcentering();
    //     done = 0;
    // }
    //
    //
    // if (Serial2.available()) {
    //
    //     data = Serial2.read();
    //
    //     if (data == '0') {
    //         setSetPointV(0);
    //         setDistanceLeft(0);
    //     }
    //
    //     if (data == '1') {
    //         setSetPointV(3000);
    //         setSetPointW(0);
    //         setDistanceLeft(1500);
    //     }
    //
    //     if (data == '2') {
    //         startController();
    //         stopIRcentering();
    //         setSetPointW(-500);
    //         setAngleLeft(-90*calibratorA);
    //         setSetPointV(0);
    //     }
    //
    //     if (data == 'g') {
    //         gyroCalibration();
    //     }
    //
    // }
    //
    // delay(20);









        // getIRDistance(dist);
        //
        // Serial2.print("FL = ");
        // Serial2.print(dist[0]);
        // Serial2.print("   ");
        //
        // Serial2.print("DL = ");
        // Serial2.print(dist[1]);
        // Serial2.print("   ");
        //
        // Serial2.print("DR = ");
        // Serial2.print(dist[2]);
        // Serial2.print("   ");
        //
        // Serial2.print("FR = ");
        // Serial2.print(dist[3]);
        // Serial2.print("   ");
        //
        // Serial2.print("Wall_left = ");
        // Serial2.print(isWallLeft());
        // Serial2.print("   ");
        //
        // Serial2.print("Wall_left = ");
        // Serial2.print(isWallRight());
        // Serial2.print("   ");
        //
        // Serial2.print("Front Wall = ");
        // Serial2.print(isWallFront());
        // Serial2.println("   ");





    if(mapeo == 1){
        if (done == 0){
            updateMaze(cell_x, cell_y);
            floodAndFill(cell_x, cell_y, goal_x, goal_y, path_s);
            next_step = path_s[0];

            Serial2.print("Celda = (");
            Serial2.print(cell_y);
            Serial2.print(",");
            Serial2.print(cell_x);
            Serial2.print(")"); Serial2.print("  Orientacion = "); Serial2.println(getMouseOrientation());


            Serial2.print("WL = ");
            Serial2.print(isWallLeft());
            Serial2.print("   ");

            Serial2.print("WR = ");
            Serial2.print(isWallRight());
            Serial2.print("   ");

            Serial2.print("WF = ");
            Serial2.print(isWallFront());
            Serial2.println("   ");

            // // Imprimimos la informacion
            //
            // Serial2.println("");
            // Serial2.println("Laberinto =");
            // for (int i = 0; i < 256; i++) {
            //
            //     if (i%16 == 0) Serial2.println();
            //
            //     Serial2.print(maze[i]);
            //     Serial2.print("\t");
            //     delay(3);
            //
            // }


            Serial2.print("Path = ");
            for (int i = 0; i < MAX_PATH_LENGTH; i++) Serial2.print(path_s[i]);
            Serial2.println();


            if (next_step == 'F'){
                if (getMouseOrientation() == 'N') { cell_x ++; setMouseOrientation('N');}
                else if (getMouseOrientation() == 'E') { cell_y ++; setMouseOrientation('E');}
                else if (getMouseOrientation() == 'W') { cell_y --; setMouseOrientation('W');}
                else if (getMouseOrientation() == 'S') { cell_x --; setMouseOrientation('S');}
            }

            if (next_step == 'L'){
                if (getMouseOrientation() == 'N') { setMouseOrientation('W');}
                else if (getMouseOrientation() == 'E') { setMouseOrientation('N');}
                else if (getMouseOrientation() == 'W') { setMouseOrientation('S');}
                else if (getMouseOrientation() == 'S') { setMouseOrientation('E');}
            }

            if (next_step == 'R'){
                if (getMouseOrientation() == 'N') { setMouseOrientation('E');}
                else if (getMouseOrientation() == 'E') { setMouseOrientation('S');}
                else if (getMouseOrientation() == 'W') { setMouseOrientation('N');}
                else if (getMouseOrientation() == 'S') { setMouseOrientation('W');}
            }

            if (next_step == 'G'){
                if (getMouseOrientation() == 'N') { setMouseOrientation('S');}
                else if (getMouseOrientation() == 'E') { setMouseOrientation('W');}
                else if (getMouseOrientation() == 'W') { setMouseOrientation('E');}
                else if (getMouseOrientation() == 'S') { setMouseOrientation('N');}
            }

            if (next_step == 'X'){
                Serial2.println("[-] Error, Path not found");
                if (getMouseOrientation() == 'N') { setMouseOrientation('E');}
                else if (getMouseOrientation() == 'E') { setMouseOrientation('S');}
                else if (getMouseOrientation() == 'W') { setMouseOrientation('N');}
                else if (getMouseOrientation() == 'S') { setMouseOrientation('W');}
            }

            if (next_step == 'S'){
                Serial2.println("Laberinto listo");
                mapeo = 0;
                corrida = 1;
            }




            delay(300);

            executeStep(next_step);
        }


        if (done == 1) executeStep(next_step);

    }


    if(corrida == 1){



        if (done == 0 ) executeStep(path_s[index_s]);

        if (done == 1){ executeStep(path_s[index_s]); if (done == 0)  index_s++;}
    }
}

// Executes one or more steps of the planned path, returns the index of the array where it ends
uint8_t executeStep(char run_path){


    if (run_path == 'R'){

        // If not currently running, start a new command
        if (done == 0) {
            // Turn 90 degree over your own axis.
            startController();
            stopIRcentering();
            setSetPointW(-600);
            setAngleLeft(-90*calibratorA);
            setSetPointV(0);
            done = 1;   //set the flag that a new command started
        }
        else {
            if (abs(getAngleLeft()) < 10.0 ) {
                setSetPointW(0);
                // stopController();
                stopIRcentering();
                done = 0;
            }
        }
    }

    if (run_path == 'X'){

        // If not currently running, start a new command
        if (done == 0) {
            // Turn 90 degree over your own axis.
            startController();
            stopIRcentering();
            setSetPointW(-600);
            setAngleLeft(-90*calibratorA);
            setSetPointV(0);
            done = 1;   //set the flag that a new command started
        }
        else {
            if (abs(getAngleLeft()) < 10.0 ) {
                setSetPointW(0);
                // stopController();
                // stopIRcentering();
                done = 0;
            }
        }
    }


    if (run_path == 'G'){

        // If not currently running, start a new command
        if (done == 0) {
            // Turn 90 degree over your own axis.
            startController();
            stopIRcentering();
            setSetPointW(600);
            setAngleLeft(180*calibratorA);
            setSetPointV(0);
            done = 1;   //set the flag that a new command started
        }
        else {
            if (abs(getAngleLeft()) < 10.0 ) {
                setSetPointW(0);
                // stopController();
                // stopIRcentering();
                done = 0;

            }
        }
    }

        if (run_path == 'L'){

            // If not currently running, start a new command
            if (done == 0) {
                // Turn 90 degree over your own axis.
                startController();
                stopIRcentering();
                setSetPointW(600);
                setAngleLeft(90*calibratorA);
                setSetPointV(0);
                done = 1;   //set the flag that a new command started
            }
            else {
                if (abs(getAngleLeft()) < 10.0 ) {
                    setSetPointW(0);
                    // stopController();
                    stopIRcentering();
                    done = 0;

                }
            }
        }


    if (run_path == 'F'){

        // If not currently running, start a new command
        if (done == 0) {
            // Turn 90 degree over your own axis.
            startController();
            startIRcentering();
            setSetPointV(2000);
            setSetPointW(0);
            setDistanceLeft(180*calibrator);
            done = 1;   //set the flag that a new command started
        }
        else {
            if (isWallFront() == 1 && (getDistanceLeft() > 90) ) setDistanceLeft(90);
            if (isWallFront() == 2 && (getDistanceLeft() > 60) ) {setDistanceLeft(60);}
            // if (isWallLeft() == 0 &&  old_wall_left == 1) setDistanceLeft(60);
            // if (isWallRight() == 0 &&  old_wall_right == 1) setDistanceLeft(60);
            if (getDistanceLeft() == 0) {
                setSetPointV(0);
                Serial2.println("Termino");
                Serial2.println("Termino");
                Serial2.println("Termino");
                Serial2.println("Termino");
                Serial2.println("Termino");
                Serial2.println("Termino");
                Serial2.println("Termino");
                Serial2.println("Termino");
                Serial2.println("Termino");
                delay(10);
                done = 0;
                // stopController();
                // stopIRcentering();
            }
            old_wall_left = isWallLeft();
            old_wall_right = isWallRight();
        }
    }

}

/*===================================================================
                Ruta Predeterminada
====================================================================*/

        //
        //
        // Serial2.print("disLeft = ");
        // Serial2.print(getDistanceLeft());
        // Serial2.print("   ");
        //
        // Serial2.print("angLeft = ");
        // Serial2.print(getAngleLeft());
        // Serial2.print("   ");
        //
        // Serial2.print("sensorError = ");
        // Serial2.print(sensorError);
        // Serial2.print("   ");
        //
        // Serial2.print("posErrorW = ");
        // Serial2.print(posErrorW);
        // Serial2.print("   ");
        //
        // Serial2.print("posPwmW = ");
        // Serial2.print(posPwmW);
        // Serial2.print("   ");
        //
        // Serial2.print("WallL = ");
        // Serial2.print(isWallLeft());
        // Serial2.print("   ");
        //
        // Serial2.print("WallR = ");
        // Serial2.print(isWallRight());
        // Serial2.print("   ");
        //
        // Serial2.print("WallFr = ");
        // Serial2.print(isWallFront());
        // Serial2.print("   ");
        //
        // Serial2.print("DL = ");
        // Serial2.print(ir_sensor_distances[1]);
        // Serial2.print("   ");
        //
        // Serial2.print("DR = ");
        // Serial2.print(ir_sensor_distances[2]);
        // Serial2.print("   ");
        //
        // Serial2.print("Next = ");
        // Serial2.print(path_s[path_index]);
        // Serial2.print(path_index);
        // Serial2.print(" ");
        // if (done == 1) Serial2.print("GOING");
        // if (done == 0) Serial2.print("DONE");
        // Serial2.println("   ");
        //
        // if (done == 0 && start == 1) executeStep(path_s);
        //
        // if (done == 1) executeStep(path_s);
        //
        // if (Serial2.available()) {
        //
        //     data = Serial2.read();
        //
        //     //Stop everything
        //     if (data == '0') {
        //         setSetPointV(0);
        //         setSetPointW(0);
        //         setDistanceLeft(0);
        //         path_index = 0;
        //         start = 0;
        //     }
        //
        //     if (data == '1') {
        //         start = 1;
        //     }
        //
        //     if (data == 'g') {
        //         gyroCalibration();
        //     }
        //
        // }
        //
        // delay(20);
        //



/*===================================================================
                Controller tester
====================================================================*/


    //
    // Serial2.print("distanceLeft = ");
    // Serial2.print(getDistanceLeft());
    // Serial2.print("   ");
    //
    // Serial2.print("angleLeft = ");
    // Serial2.print(getAngleLeft());
    // Serial2.print("   ");
    //
    // Serial2.print("posErrorW = ");
    // Serial2.print(posErrorW);
    // Serial2.print("   ");
    //
    // Serial2.print("posPwmX = ");
    // Serial2.print(posPwmX);
    // Serial2.print("   ");
    //
    // Serial2.print("posPwmW = ");
    // Serial2.print(posPwmW);
    // Serial2.print("   ");
    //
    // Serial2.print("PWM_L = ");
    // Serial2.print(leftBaseSpeed);
    // Serial2.print("   ");
    //
    // Serial2.print("PWM_R = ");
    // Serial2.print(rightBaseSpeed);
    // Serial2.print("   ");
    //
    // Serial2.print("Vmean = ");
    // Serial2.print(getVmean());
    // Serial2.println("mm/s    ");


    // if (getDistanceLeft() == 0) setSetPointV(0);
    // if (abs(getAngleLeft()) < 10.0 ) setSetPointW(0);
    //
    //
    // if (Serial2.available()) {
    //
    //     data = Serial2.read();
    //
    //     if (data == '0') {
    //         setSetPointV(0);
    //         setDistanceLeft(0);
    //     }
    //
    //     if (data == '1') {
    //         setSetPointV(3000);
    //         setSetPointW(0);
    //         setDistanceLeft(1500);
    //     }
    //
    //     if (data == '2') {
    //         setSetPointW(500);
    //         setAngleLeft(90);
    //         setSetPointV(0);
    //     }
    //
    //     if (data == 'g') {
    //         gyroCalibration();
    //     }
    //
    // }

    // delay(20);


/*===================================================================
                Flood and Fill paso a paso
====================================================================*/

// // getIR(dist);
// getIRDistance(dist);
//
// if (Serial2.available()) {
//
//     data = Serial2.read();
//
//     if (data == '0') {
//
//         Serial2.print("Celda = (");
//         Serial2.print(cell_y);
//         Serial2.print(",");
//         Serial2.print(cell_x);
//         Serial2.print(")"); Serial2.print("  Orientacion = "); Serial2.println(getMouseOrientation());
//
//         Serial2.print("Wall_left = ");
//         Serial2.print(isWallLeft());
//         Serial2.print("   ");
//
//         Serial2.print("Wall_Right = ");
//         Serial2.print(isWallRight());
//         Serial2.print("   ");
//
//         Serial2.print("Front Wall = ");
//         Serial2.print(isWallFront());
//         Serial2.println("   ");
//
//         Serial2.println("bup");
//         updateMaze(cell_x, cell_y);
//         Serial2.println("aup");
//
//         Serial2.println("bfaf");
//         floodAndFill(cell_x, cell_y, goal_x, goal_y, path_s);
//         Serial2.println("afaf");
//
//         next_step = path_s[0];
//
//         if (next_step == 'F'){
//             if (getMouseOrientation() == 'N') { cell_x ++; setMouseOrientation('N');}
//             else if (getMouseOrientation() == 'E') { cell_y ++; setMouseOrientation('E');}
//             else if (getMouseOrientation() == 'W') { cell_y --; setMouseOrientation('W');}
//             else if (getMouseOrientation() == 'S') { cell_x --; setMouseOrientation('S');}
//         }
//
//         if (next_step == 'L'){
//             if (getMouseOrientation() == 'N') { setMouseOrientation('W');}
//             else if (getMouseOrientation() == 'E') { setMouseOrientation('N');}
//             else if (getMouseOrientation() == 'W') { setMouseOrientation('S');}
//             else if (getMouseOrientation() == 'S') { setMouseOrientation('E');}
//         }
//
//         if (next_step == 'R'){
//             if (getMouseOrientation() == 'N') { setMouseOrientation('E');}
//             else if (getMouseOrientation() == 'E') { setMouseOrientation('S');}
//             else if (getMouseOrientation() == 'W') { setMouseOrientation('N');}
//             else if (getMouseOrientation() == 'S') { setMouseOrientation('W');}
//         }
//
//         if (next_step == 'G'){
//             if (getMouseOrientation() == 'N') { setMouseOrientation('S');}
//             else if (getMouseOrientation() == 'E') { setMouseOrientation('W');}
//             else if (getMouseOrientation() == 'W') { setMouseOrientation('E');}
//             else if (getMouseOrientation() == 'S') { setMouseOrientation('N');}
//         }
//
//         if (next_step == 'X'){
//             Serial2.println("[-] Error, Path not found");
//         }
//
//
//         // Imprimimos la informacion
//
//         Serial2.println("");
//         Serial2.println("Laberinto =");
//         for (int i = 0; i < 256; i++) {
//
//             if (i%16 == 0) Serial2.println();
//
//             Serial2.print(maze[i]);
//             Serial2.print("\t");
//             delay(3);
//
//         }
//
//         Serial2.println("");
//         Serial2.println("Flood and Fill =");
//         for (int i = 0; i < 256; i++) {
//
//             if (i%16 == 0) Serial2.println();
//
//             Serial2.print(maps[i]);
//             Serial2.print("\t");
//             delay(3);
//
//         }
//
//         Serial2.println();
//         Serial2.print("Path = ");
//         for (int i = 0; i < MAX_PATH_LENGTH; i++) Serial2.print(path_s[i]);
//
//     }
//
//     if (data == '1') {
//
//         Serial2.print("Celda = (");
//         Serial2.print(cell_y);
//         Serial2.print(",");
//         Serial2.print(cell_x);
//         Serial2.print(")"); Serial2.print("  Orientacion = "); Serial2.println(getMouseOrientation());
//
//         Serial2.print("Wall_left = ");
//         Serial2.print(isWallLeft());
//         Serial2.print("   ");
//
//         Serial2.print("Wall_Right = ");
//         Serial2.print(isWallRight());
//         Serial2.print("   ");
//
//         Serial2.print("Front Wall = ");
//         Serial2.print(isWallFront());
//         Serial2.println("   ");
//
//         Serial2.println("bup");
//         updateMaze(cell_x, cell_y);
//         Serial2.println("aup");
//
//         // Imprimimos la informacion
//
//         Serial2.println("");
//         Serial2.println("Laberinto =");
//         for (int i = 0; i < 256; i++) {
//
//             if (i%16 == 0) Serial2.println();
//
//             Serial2.print(maze[i]);
//             Serial2.print("\t");
//             delay(3);
//
//         }
//
//         Serial2.println("bfaf");
//         floodAndFill(cell_x, cell_y, goal_x, goal_y, path_s);
//         Serial2.println("afaf");
//
//
//
//         Serial2.println("");
//         Serial2.println("Flood and Fill =");
//         for (int i = 0; i < 256; i++) {
//
//             if (i%16 == 0) Serial2.println();
//
//             Serial2.print(maps[i]);
//             Serial2.print("\t");
//             delay(3);
//
//         }
//
//         Serial2.println();
//         Serial2.print("Path = ");
//         for (int i = 0; i < MAX_PATH_LENGTH; i++) Serial2.print(path_s[i]);
//     }
// }
//
//
//


/*===================================================================
                Codigo de calibracion infrarrojo
====================================================================*/

    // getIR(dist);
    //
    // Serial2.print("FL = ");
    // Serial2.print(irval[0]);
    // Serial2.print("   ");
    //
    // Serial2.print("DL = ");
    // Serial2.print(irval[1]);
    // Serial2.print("   ");
    //
    // Serial2.print("DR = ");
    // Serial2.print(irval[2]);
    // Serial2.print("   ");
    //
    // Serial2.print("FR = ");
    // Serial2.print(irval[3]);
    // Serial2.print("   ");
    //
    // Serial2.print("Wall_left = ");
    // Serial2.print(isWallLeft());
    // Serial2.print("   ");
    //
    // Serial2.print("Wall_left = ");
    // Serial2.print(isWallRight());
    // Serial2.print("   ");
    //
    // Serial2.print("Front Wall = ");
    // Serial2.print(isWallFront());
    // Serial2.println("   ");




/*===================================================================
                    Codigo de chequeo de los encoders
====================================================================*/


    // Serial2.print("VL = ");
    // Serial2.print(getVL());
    // Serial2.print("mm/s      ");
    //
    // Serial2.print("VR = ");
    // Serial2.print(getVR());
    // Serial2.print("mm/s      ");
    //
    // Serial2.print("distanceLeft = ");
    // Serial2.print(getDistanceLeft());
    // Serial2.println("mm      ");
    //
    // delay(50);
    //
    // // Start in the safe range
    //
    // if (Serial2.available()) {
    //
    //     data = Serial2.read();
    //
    //     if (data == '0') {
    //         motorLeftWrite(0);
    //         motorRightWrite(0);
    //     }
    //
    //     if (data == '1') {
    //         motorLeftWrite(25000);
    //         // motorRightWrite(25000);
    //     }
    //
    //     if (data == '2') {
    //         motorLeftWrite(50000);
    //         // motorRightWrite(60000);
    //     }
    //
    //     if (data == '3') {
    //         // motorLeftWrite(-25000);
    //         motorRightWrite(25000);
    //     }
    //
    //     if (data == '4') {
    //         // motorLeftWrite(-50000);
    //         motorRightWrite(60000);
    //     }
    //
    //     if (data == 'a') {
    //         setDistanceLeft(200);
    //     }
    // }



/*===================================================================
                Codigo que agarra el dataset de los motores
====================================================================*/

//Code that writes the data for the motor balancer
//
// for (PWM = 0; PWM < 65535; PWM = PWM + 1000) {
//
//       motorLeftWrite(PWM);
//       motorRightWrite(PWM);
//
//       setRot1(0);
//       setRot2(0);
//
//       Serial2.print(PWM);
//       Serial2.print(",");
//       delay(1000);
//
//       Serial2.print(getRot1Steps());
//       Serial2.print(",");
//       Serial2.print(getRot2Steps());
//       Serial2.println();
// }
//
// for (PWM = 0; PWM > -65536; PWM = PWM - 1000) {
//
//     motorLeftWrite(PWM);
//     motorRightWrite(PWM);
//
//     setRot1(0);
//     setRot2(0);
//
//     Serial2.print(PWM);
//     Serial2.print(",");
//     delay(1000);
//
//     Serial2.print(getRot1Steps());
//     Serial2.print(",");
//     Serial2.print(getRot2Steps());
//     Serial2.println();
// }
