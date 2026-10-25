/********************************************************
 *                                                      *
 *                                                      *
 *      state_machine_student.h          	        *
 *                             		                *
 *              Algorithm bug 1 	                *
 *             			                        *
 *		Student:Roberto Miranda Meza            *
 *		FI-UNAM		                        *
 *		27-09-2026                              *
 *                                                      *
 ********************************************************/


// Student State Machine
AdvanceAngle reactive_students(Raw observations, int dest, int intensity, float Mag_Advance, float max_angle, int num_sensors){

 AdvanceAngle gen_vector;
 int obs;
 int j;
 float left_side=0;
 float right_side=0;
 int value = 0;
 static int step=0;

 step++;
 printf("\n\n **************** Student Reactive Behavior %d *********************\n",step);

 for(j=0;j<num_sensors/2;j++){
        right_side = observations.sensors[j] + right_side;
        printf("right side sensor[%d] %f\n",j,observations.sensors[j]);
 }

 for(j=num_sensors/2;j<num_sensors;j++){
        left_side = observations.sensors[j] + left_side;
        printf("left side sensor[%d] %f\n",j,observations.sensors[j]);
 }

 right_side = right_side/(num_sensors/2);
 left_side = left_side/(num_sensors/2);
 printf("Average right side %f\n",right_side);
 printf("Average left side %f\n",left_side);

 if( left_side < THRS) value = (value << 1) + 1;
 else value = (value << 1) + 0;

 if( right_side < THRS) value = (value << 1) + 1;
 else value = (value << 1) + 0;

 obs = value;
 printf("intensity %d obstacles %d dest %d\n",intensity,obs,dest);

 if (intensity == 1){
	// Constants STOP, TURN RIGHT, ETC, are defined in ../utilities/constants.h
	// generate_output function in ../utilities/utilities.h
	gen_vector=generate_output(STOP,Mag_Advance,max_angle);
        printf("STOP\n");
	printf("\n **************** Reached light source ******************************\n");
 }
 else if (obs == 0){
	// There is not obstacle
        //gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
	//printf("FORWARD\n");

	if (dest == 0){
                // go right twice
		gen_vector=generate_output(RIGHTADVANCETWICE,Mag_Advance,max_angle);
                printf("TURN RIGHT TWICE\n");
        }
        else if (dest == 1){
                // go left twice
                gen_vector=generate_output(LEFTADVANCETWICE,Mag_Advance,max_angle);
                printf("TURN LEFT TWICE\n");
        }
        else if (dest == 2){
                 // go right
                 gen_vector=generate_output(RIGHTADVANCE,Mag_Advance,max_angle);
                 printf("TURN RIGHT\n");
        }
        else if (dest == 3){
                // go left
                gen_vector=generate_output(LEFTADVANCE,Mag_Advance,max_angle);
                printf("TURN LEFT\n");
        }
 }
 else if (obs == 1){
        // Obtacle in the right
        gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
	printf("TURN LEFT\n");
 }
 else if (obs == 2){
        // obtacle in the left
	gen_vector=generate_output(RIGHT,Mag_Advance,max_angle);
	printf("TURN RIGHT\n");
 }
 else if (obs == 3){
	// obstacle in the front
        gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
	printf("TURN LEFT\n");
 }


 return gen_vector;

}





/*
// Student State Machine
AdvanceAngle state_machine_students(Raw observations, int dest, int intensity, int state, int *next_state, float Mag_Advance, float max_angle, int num_sensors, float angle_light){

 AdvanceAngle gen_vector;
 int obs;
 int j;
 float left_side=0;
 float right_side=0;
 int value = 0;

 printf("\n\n **************** Student State Machine *********************\n");

 for(j=0;j<num_sensors/2;j++){
        right_side = observations.sensors[j] + right_side;
        printf("right side sensor[%d] %f\n",j,observations.sensors[j]);
 }

 for(j=num_sensors/2;j<num_sensors;j++){
        left_side = observations.sensors[j] + left_side;
        printf("left side sensor[%d] %f\n",j,observations.sensors[j]);
 }

 right_side = right_side/(num_sensors/2);
 left_side = left_side/(num_sensors/2);
 printf("Average right side %f\n",right_side);
 printf("Average left side %f\n",left_side);

 if( left_side < THRS) value = (value << 1) + 1;
 else value = (value << 1) + 0;

 if( right_side < THRS) value = (value << 1) + 1;
 else value = (value << 1) + 0;

 obs = value;
 printf("intensity %d obstacles %d dest %d\n",intensity,obs,dest);
 printf("Angle light %f\n",angle_light);

 switch ( state ) {

        case 0:
                if (intensity == 1){
                        gen_vector=generate_output(STOP,Mag_Advance,max_angle);
                        *next_state = 1;

                        printf("Present State: %d STOP\n", state);
			printf("\n **************** Reached light source ******************************\n");
                }
                else{

			gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
                        *next_state = 1;

                        printf("Present State: %d FORWARD\n", state);
                }

                break;

        case 1:
                if (obs == 0){
			// There is not obstacle
                        gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
                        *next_state = 13;

                        printf("Present State: %d FORWARD\n", state);
                }
                else{
                        gen_vector=generate_output(STOP,Mag_Advance,max_angle);
                        printf("Present State: %d STOP\n", state);

                        if (obs == 1){
                                // obtacle in the  right
                                *next_state = 2;
                        }
                        else if (obs == 2){
                                // obtacle in the left
                                *next_state = 4;
                        }
                        else if (obs == 3){
				// obstacle in the front
                                *next_state = 6;
                        }
                }

                break;

        case 2: // Backward, obstacle in the right
                gen_vector=generate_output(BACKWARD,Mag_Advance,max_angle);
                *next_state = 3;

		printf("Present State: %d BACKWARD, obstacle right\n", state);
                break;

        case 3: // right turn
                gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
                *next_state = 0;

		printf("Present State: %d TURN LEFT\n", state);
                break;

        case 4: // Backward, obstacle in the left
                gen_vector=generate_output(BACKWARD,Mag_Advance,max_angle);
                *next_state = 5;

		printf("Present State: %d BACKWARD, obstacle left\n", state);
                break;

        case 5: // left turn
                gen_vector=generate_output(RIGHT,Mag_Advance,max_angle);
                *next_state = 0;

		printf("Present State: %d TURN RIGTH\n", state);
                break;

        case 6: // Backward, obstacle in front
                gen_vector=generate_output(BACKWARD,Mag_Advance,max_angle);
                *next_state = 7;

		printf("Present State: %d BACKWARD, obstacle FRONT\n", state);
                break;

	case 7: /// Left turn
                gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
                *next_state = 8;

		printf("Present State: %d TURN 1 LEFT\n", state);
                break;

        case 8:// Left turn
                gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
                *next_state = 9;

		printf("Present State: %d TURN 2 LEFT\n", state);
                break;

        case 9: // Forward
                gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
                *next_state = 10;

                printf("Present State: %d 1 FORWARD\n", state);
                break;

        case 10: // Forward
                gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
                *next_state = 11;

                printf("Present State: %d 2 FORWARD\n", state);
                break;

	case 11: // Right turn
                gen_vector=generate_output(RIGHT,Mag_Advance,max_angle);
                *next_state = 12;

                printf("Present State: %d turn 1 RIGHT\n", state);
                break;

        case 12: // Right turn
                gen_vector=generate_output(RIGHT,Mag_Advance,max_angle);
                *next_state = 0;

                printf("Present State: %d turn 2 RIGHT\n", state);
                break;


        case 13: // // check destination
		 if (dest == 0){
                                // go right
                                gen_vector=generate_output(RIGHT,Mag_Advance,max_angle);
                                *next_state = 5;

                                printf("Present State: %d RIGHT\n", state);
                 }
                 else if (dest == 1){
                                // go left
                                gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
                                *next_state = 3;

                                printf("Present State: %d LEFT\n", state);
                 }
                 else if (dest == 2){
                                // go right single
                                gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
                                *next_state = 5;

                                printf("Present State: %d FORWARD\n", state);
                 }
                 else if (dest == 3){
                                // go left single
                                gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
                                *next_state = 3;

                                printf("Present State: %d FORWARD\n", state);
                 }
                break;

	default:
		printf("State %d not defined used ", state);
                gen_vector=generate_output(STOP,Mag_Advance,max_angle);
                next_state = 0;
                break;


 }

 return gen_vector;

}
*/

/********************************************************
 *  Bug 1 (behavior 6)                                  *
 ********************************************************/

// Bug 1 states
#define BUG_GO_TO_GOAL  0
#define BUG_FOLLOW_WALL 1
#define BUG_RETURN_BEST 2
#define BUG_LEAVE       3
#define BUG_FAILED      4
#define BUG_TURN_AROUND 5

// Wall side
#define BUG_WALL_RIGHT 0 //indicates that the wall is on the right side
#define BUG_WALL_LEFT  1 //indicates that the wall is on the left side

// Thresholds as multiples of Mag_Advance (first guess, they must be tuned)
#define BUG_K_FRONT    1.0  // obstacle ahead (must be > 3, mvrobot refuses to move closer than 3*Mag_Advance)
#define BUG_K_NEAR     0.5   // wall on the right too close
#define BUG_K_FAR      2.0   // wall on the right too far
#define BUG_K_LOST     1.5   // wall lost (outer corner)
#define BUG_K_LEAVE    1.5   // robot must move this far from hit before a loop can be closed
#define BUG_K_CLOSE    1.5   // "back at the point" radius
#define BUG_K_PROGRESS 0.5   // minimum improvement of the distance to the goal


// minimum range of the sensors in [ini, fin)
float bug_min_sector(Raw observations, int ini, int fin){
 int j;
 float m = observations.sensors[ini];

 for(j=ini+1;j<fin;j++){
        if(observations.sensors[j] < m) m = observations.sensors[j];
 }
 return m;
}

// it limits a value to [-lim, lim]
float bug_clamp(float v, float lim){
 if(v > lim) return lim;
 if(v < -lim) return -lim;
 return v;
}

// it converts an angle to (-PI, PI]
float bug_norm_angle(float a){
 while(a > PI) a = a - 2*PI;
 while(a < -PI) a = a + 2*PI;
 return a;
}

// it follows the wall keeping it on the right (BUG_WALL_RIGHT) or on the left (BUG_WALL_LEFT)
AdvanceAngle bug_follow_wall(Raw observations, int num_sensors, float Mag_Advance, float max_angle, int side){

 AdvanceAngle out;
 float f, wf, ws; //front, wall-right-front, wall-right-side
 int toward, away, inner;

 f = bug_min_sector(observations, (3*num_sensors)/8, (5*num_sensors)/8);   // measure of the sensor with minimum distance in front zone

 if(side == BUG_WALL_RIGHT){//'side' indicates how the robot needs to move, this time when the wall is on the right side
        wf = bug_min_sector(observations, num_sensors/4, (3*num_sensors)/8);   // measure of the sensor with minimum distance in right-front zone
        ws = bug_min_sector(observations, num_sensors/8, num_sensors/4);       // measure of the sensor with minimum distance in right side zone
        toward = RIGHTADVANCE; //here 'toward' means come back right
        away   = LEFTADVANCE; //here 'away' means come back left
        inner  = LEFT; //here 'inner' means turn left
 }
 else{ //when the wall is on the left side of the robot
        wf = bug_min_sector(observations, (5*num_sensors)/8, (3*num_sensors)/4); // measure of the sensor with minimum distance in left-front zone
        ws = bug_min_sector(observations, (3*num_sensors)/4, (7*num_sensors)/8); // measure of the sensor with minimum distance in left side zone
        toward = LEFTADVANCE; //here 'toward' means come back left
        away   = RIGHTADVANCE; //here 'away' means come back right
        inner  = RIGHT; //here 'inner' means turn right
 }
//The next code chunk is about the position of the robot anent to the wall
//Using few words, this chunk means that when running out with an obstacle the robot
//will move around like a snake and the parameters announced before are the bounding conditions of that path
 if(f < BUG_K_FRONT*Mag_Advance){// if 'f' is minor to 'times' the magnitude of advance then...
        out = generate_output(inner,Mag_Advance,max_angle);          // inner corner
 }
 else if(ws > BUG_K_LOST*Mag_Advance && wf > BUG_K_LOST*Mag_Advance){ //BUG_K_LOST is leading the advance of the robot
        out = generate_output(toward,Mag_Advance,max_angle);         // outer corner, wall lost
 }
 else if(ws < BUG_K_NEAR*Mag_Advance || wf < BUG_K_NEAR*Mag_Advance){
        out = generate_output(away,Mag_Advance,max_angle);           // too close
 }
 else if(ws > BUG_K_FAR*Mag_Advance){
        out = generate_output(toward,Mag_Advance,max_angle);         // too far
 }
 else{
        out = generate_output(FORWARD,Mag_Advance,max_angle);        // if there is not obstacle the robot just move forward
 }

 return out;
}


// Bug 1 state machine, this chunk of code is the real representation of the 6th behavior using the functions stated before (bug_min_sector, bug_clamp, bug_norm_angle and bug_follow_wall
AdvanceAngle state_machine_bug1(Raw observations, int intensity, int state, int *next_state, float Mag_Advance, float max_angle, int num_sensors, float angle_light, coord coord_robot, coord coord_dest){

 AdvanceAngle gen_vector; //gen_vector is a variable of type AdvanceAngle
 static coord hit, best; // coord hit and best are declared static because of they are critical points, both are of type coord
 static float d_hit, d_best; //d_hit and d_best are distances from the hitting point with an obstacle and the best distance, both anent to the destiny, critical points too
 static float heading_target; //
 static int left_hit = 0; //left_hit
 float d_robot, d_to_hit, f, err; //these are variables and points which change every time the robot advances

 printf("\n\n **************** Bug 1 *********************\n");

 *next_state = state;

 if(num_sensors < 8){ //this is the main condition so the algorythm is able to work
        printf("Bug 1 needs at least 8 sensors (-n 16 recommended)\n");
        *next_state = BUG_FAILED;
        return generate_output(STOP,Mag_Advance,max_angle); //the robot does not advance because of the lack of sensors
 }

 // reached the light source
 if(intensity == 1){//the intensity of light is the maximum so the destiny has been reached
        printf("\n **************** Reached light source ******************************\n");
        return generate_output(STOP,Mag_Advance,max_angle); //the robot does not advance anymore because of it has arrived
 }

 d_robot = distance(coord_robot,coord_dest); //d_robot is the distance between its actual coordinates and the destiny coordinates

 switch(state){//this chunk of code answers to the question, what does the robot need to do if it has not reached its destiny yet?

        case BUG_GO_TO_GOAL://in this state the robot simply moves to its goal, the destiny
                f = bug_min_sector(observations, (3*num_sensors)/8, (5*num_sensors)/8);//'f' acquires the minor distance in the front part of the robot
                if(f < BUG_K_FRONT*Mag_Advance){//if the robot is closer than BUG_K_FRONT times its advance magnitude then...
                        // obstacle: it remembers the hit point
                        hit = coord_robot; //here the hit point to the obstacle is saved
                        d_hit = d_robot; //the actual d_robot is saved as d_hit in the moment of 'hitting' the obstacle
                        best = coord_robot; //the actual coord_robot is saved as best too
                        d_best = d_robot; //the actual d_robot is saved as d_best too
                        left_hit = 0; //means that there is not a wall on the left side, is on the right side
                        *next_state = BUG_FOLLOW_WALL; //so the next state is 'robot lets follow the wall!'
                        gen_vector = generate_output(LEFT,Mag_Advance,max_angle); //the robot advances to the left the advance magnitude and 45 degrees in this step
                        printf("State %d HIT at x %f y %f, d_hit %f\n",state,hit.xc,hit.yc,d_hit);//the state, coordinates and distances of the robot are printed
                }
                else{//if the state is other one, then...
                        err = bug_norm_angle(angle_light); //the angle anent the destiny is normalized
                        gen_vector.angle = bug_clamp(err,max_angle); //uses bug_clamp to see if the angle anent the destiny is minor or major to max_angle
                        if(fabs(err) <= max_angle) gen_vector.distance = Mag_Advance;
                        else gen_vector.distance = 0.0;
                        printf("State %d GO TO GOAL err %f\n",state,err);
                }
                break; //the switch is finished and the next step should be evaluated

        case BUG_FOLLOW_WALL://this state makes the robot move around the wall
                if(d_robot < d_best){//when the actual distance of the robot is minor to the best distance to the destiny registered then...
                        best = coord_robot; //the 'best' point is updated
                        d_best = d_robot; //the 'best' distance is updated too
                }
                d_to_hit = distance(coord_robot,hit);//using the 'hit' point from BUG_GO_TO_GOAL state and the actual coord_robot it is d_to_hit
                if(d_to_hit > BUG_K_LEAVE*Mag_Advance) left_hit = 1;
                //when the distance to the hit point is large enough then left_hit is true, here the robot is getting away from 'hit' point
                //when left_hit = 0 then the robot is getting closer to the 'hit' point

                if(left_hit && d_to_hit < BUG_K_CLOSE*Mag_Advance){//this is an imposible condition
                        // full loop completed
                        printf("State %d LOOP CLOSED, best x %f y %f d_best %f d_hit %f\n",state,best.xc,best.yc,d_best,d_hit);
                        if(d_best > d_hit - BUG_K_PROGRESS*Mag_Advance){//this cannot be so the algorythm failed, means that the robot moved almost nothing
                                printf("No progress possible: goal unreachable\n");//this means that the goal is unreachable
                                *next_state = BUG_FAILED;
                        }
                        else{//when the robot comes back to the 'hit' point then left_hit must be 0 and then...
                                // it will turn 180 degrees and go back along the wall
                                heading_target = coord_robot.anglec + PI;//heading_target is an angle
                                *next_state = BUG_TURN_AROUND;//the bug or robot turns around and returns from it came heading to the 'best' point registered
                        }
                        gen_vector = generate_output(STOP,Mag_Advance,max_angle);
                }
                else{//if robot does not fall in that condition then it follows the wall perimeter
                        gen_vector = bug_follow_wall(observations,num_sensors,Mag_Advance,max_angle,BUG_WALL_RIGHT);//the bug_follow_wall function is called
                        printf("State %d FOLLOW WALL (right), d_best %f\n",state,d_best);
                }
                break;

        case BUG_TURN_AROUND:
                err = bug_norm_angle(heading_target - coord_robot.anglec);//the error comparison with actual coord_robot is normalized
                gen_vector.distance = 0.0;
                gen_vector.angle = bug_clamp(err,max_angle);//the error should be in range of max_angle
                if(fabs(err) <= max_angle) *next_state = BUG_RETURN_BEST;//it's evaluated if the absolute value of the error is minor or equals to max_angle so the robot returns to the best point
                printf("State %d TURN AROUND, err %f\n",state,err);
                break;

        case BUG_RETURN_BEST:
                if(distance(coord_robot,best) < BUG_K_CLOSE*Mag_Advance){//When the closest point to the destiny in the perimeter of the obstacle is reached then...
                        printf("State %d reached the closest point\n",state);
                        *next_state = BUG_LEAVE;//The next state is BUG_LEAVE
                        gen_vector = generate_output(STOP,Mag_Advance,max_angle);//The robot stops
                }
                else{//if the distance to the robot after turning around is too long then...
                        gen_vector = bug_follow_wall(observations,num_sensors,Mag_Advance,max_angle,BUG_WALL_LEFT);//the robot follows the wall again but this time its left sensors are checking it
                        printf("State %d RETURN TO BEST (wall on the left)\n",state);
                }
                break;

        case BUG_LEAVE: //the robot turns heading the light or destiny direction
                err = bug_norm_angle(angle_light);
                gen_vector.angle = bug_clamp(err,max_angle);
                gen_vector.distance = 0.0;
                if(fabs(err) <= max_angle) *next_state = BUG_GO_TO_GOAL;//then the robot goes to its target
                printf("State %d LEAVE, err %f\n",state,err);
                break;

        case BUG_FAILED:
                gen_vector = generate_output(STOP,Mag_Advance,max_angle);//when the algorythm fails the robot stops movement
                printf("State %d FAILED\n",state);
                break;

        default://when robot falls in an undetermined state
                printf("State %d not defined\n",state);
                gen_vector = generate_output(STOP,Mag_Advance,max_angle);//firstly it stops and then...
                *next_state = BUG_GO_TO_GOAL;//goes to its target
                break;
 }

 return gen_vector;

}


