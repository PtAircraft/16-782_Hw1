/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <math.h>
#include "mex.h"
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <unordered_map>
#include <iostream>
#include <algorithm>
#include <chrono>

/* Input Arguments */
#define	MAP_IN      prhs[0]
#define	ROBOT_IN	prhs[1]
#define	GOAL_IN     prhs[2]


/* Output Arguments */
#define	ACTION_OUT	plhs[0]

/*access to the map is shifted to account for 0-based indexing in the map, whereas
1-based indexing in matlab (so, robotpose and goalpose are 1-indexed)*/
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

/* Primitives Information */
#define NUMOFDIRS 8
#define NUMOFPRIMS 5
#define NUMOFINTERSTATES 10
#define NUMOFDIM 3
#define RES 0.1

using namespace std;

typedef float PrimArray[NUMOFDIRS][NUMOFPRIMS][NUMOFINTERSTATES][NUMOFDIM];

int temp = 0;
// static float eps = 1;

float initial_goal_x;
float initial_goal_y;

unordered_map<int, float>heuristic;

// define node
struct node{
	float x_, y_, theta_, g_, h_, f_; 
	int parent_ID_, prim_, cur_ID_;
	node(float x, float y, float theta, float g, float h, float f, int index, int primID, int crd) {
		x_ = x;
		y_ = y;
		theta_ = theta;
		g_ = g;
		h_ = g;
		f_ = f;
		parent_ID_ = index;
		prim_ = primID;
        cur_ID_ = crd;
	}
};
// define node struct for finding heuristic
struct node_h{
	float x_, y_, f_; 
	int cur_ID_;
	node_h(float x, float y, float f, int crd) {
		x_ = x;
		y_ = y;
		f_ = f;
        cur_ID_ = crd;
	}
};
// define compare function for heap
struct compare_func{
	bool operator() (const node &node1, const node &node2) {
		return node1.f_ > node2.f_;
	}
};
struct compare_func_h{
	bool operator() (const node_h &node1, const node_h &node2) {
		return node1.f_ > node2.f_;
	}
};


bool applyaction(double *map, int x_size, int y_size, float robotposeX, float robotposeY, float robotposeTheta,
                 float *newx, float *newy, float *newtheta, PrimArray mprim, int dir, int prim)
{
    int i;
    for (i = 0; i < NUMOFINTERSTATES; i++) {
        *newx = robotposeX + mprim[dir][prim][i][0];
        *newy = robotposeY + mprim[dir][prim][i][1];
        *newtheta = mprim[dir][prim][i][2];
        
        int gridposx = (int)(*newx / RES + 0.5);
        int gridposy = (int)(*newy / RES + 0.5);

        /* check validity */
        if (gridposx < 1 || gridposx > x_size || gridposy < 1 || gridposy > y_size){
            return false;
        }
        if ((int)map[GETMAPINDEX(gridposx, gridposy, x_size, y_size)] != 0){
            return false;
        }
    }
    return true;
}

bool applyaction4(double *map, int x_size, int y_size, float robotposeX, float robotposeY,
                 float *newx, float *newy, int prim)
{
    // int i;
    // for (i = 0; i < 4; i++) {

	if (prim == 1) {
        *newx = robotposeX + 0.1;
        *newy = robotposeY + 0;		
	}
	if (prim == 1) {
        *newx = robotposeX + 0;
        *newy = robotposeY + 0.1;		
	}
	if (prim == 2) {
        *newx = robotposeX - 0.1;
        *newy = robotposeY + 0;		
	}
	if (prim == 4) {
        *newx = robotposeX + 0;
        *newy = robotposeY - 0.1;		
	}
	if (prim == 5) {
        *newx = robotposeX + 0.1;
        *newy = robotposeY + 0.1;		
	}
	if (prim == 6) {
        *newx = robotposeX + 0.1;
        *newy = robotposeY - 0.1;		
	}
	if (prim == 7) {
        *newx = robotposeX - 0.1;
        *newy = robotposeY + 0.1;		
	}
	if (prim == 8) {
        *newx = robotposeX - 0.1;
        *newy = robotposeY - 0.1;		
	}
        // *newx = robotposeX + mprim[dir][prim][i][0];
        // *newy = robotposeY + mprim[dir][prim][i][1];
        // *newtheta = mprim[dir][prim][i][2];
        
    int gridposx = (int)(*newx / RES + 0.5);
    int gridposy = (int)(*newy / RES + 0.5);

    /* check validity */
    if (gridposx < 1 || gridposx > x_size || gridposy < 1 || gridposy > y_size){
        return false;
    }
    if ((int)map[GETMAPINDEX(gridposx, gridposy, x_size, y_size)] != 0){
        return false;
    }
    // }
    return true;
}

void get_heuristic(		   
			double*	map,
		    int x_size,
 		    int y_size,
            float goalposeX,
            float goalposeY) 
{
	static node_h* n0;
	static node_h* m0;
	// start node index
    int start_index = GETMAPINDEX((int)(goalposeX / RES + 0.5), (int)(goalposeY / RES + 0.5), x_size, y_size);
    // build the start node
    float start_f = 0;

    n0 = new node_h(goalposeX, goalposeY, start_f, start_index);
    // build the open list
    vector<node_h> open_h;
    // initialize open list
    open_h.push_back(*n0);
    make_heap(open_h.begin(), open_h.end(), compare_func_h());
    // build the closed list;
    // chrono::steady_clock::time_point t1 = chrono::steady_clock::now(); 
    // Dijkstra
    while(!open_h.empty()) {
    	// take the smallest node in open list
    	n0 = new node_h(open_h.front().x_, open_h.front().y_, 
    					open_h.front().f_, open_h.front().cur_ID_);
    	// map index of this node
    	int current_index = n0->cur_ID_;
    	// remove this node in open list
    	pop_heap(open_h.begin(), open_h.end(), compare_func_h());
    	open_h.pop_back();
        // insert this node in closed list
        heuristic.insert(pair<int, float>(current_index, n0->f_));

    	//find successor from all possible primitive
    	int prim;
    	for (prim = 1; prim <= 8; prim ++) {
    		float newx, newy;
    		bool ret;
    		ret = applyaction4(map, x_size, y_size, n0->x_, n0->y_,
    		                  &newx, &newy, prim);
            if (ret) {
                // check if this node is in closed
        		int successor_index = GETMAPINDEX((int)(newx / RES + 0.5), (int)(newy / RES + 0.5), x_size, y_size);
    	    	// unordered_map<int, float>::const_iterator it = heuristic.find(successor_index);
    	    	if (heuristic.find(successor_index) == heuristic.end()) {
    	    		float cost = RES;
                    float newf = n0->f_ + cost;
    	    		// check if this node is in open;
    	    		bool found_in_open = false;
    	    		for (int i = 0; i < open_h.size(); i++) {
    	    			// if it's in open
    	    			if (successor_index == open_h[i].cur_ID_) {
    	    				found_in_open = true;
    	    				// check the g value
    	    				if (open_h[i].f_ > newf) {
    	    					open_h[i].f_ = newf;
    	    					make_heap(open_h.begin(), open_h.end(), compare_func_h());
    	    				}
    	    				break;
    	    			}
    	    		}
    	    		if (!found_in_open) {
    	    			m0 = new node_h (newx, newy, newf, successor_index);
    	    			open_h.push_back(*m0);
    	    			push_heap(open_h.begin(), open_h.end(), compare_func_h());
    	    		}
    	    	}
            }
    	}
    }
    delete n0;
    delete m0;
    return;
}

int getPrimitiveDirectionforRobotPose(float angle)
{
    /* returns the direction index with respect to the PrimArray */
    /* normalize bw 0 to 2pi */
    if (angle < 0.0) {
        angle += 2 * M_PI;
    }
    int dir = (int)(angle / (2 * M_PI / NUMOFDIRS) + 0.5);
    if (dir == 8) {
        dir = 0;
    }
    return dir;
}
int flag_h = 0;
int initial_goal_index;
int heuristic_state = 1;
static void planner(
		   double*	map,
		   int x_size,
 		   int y_size,
	       float robotposeX,
            float robotposeY,
            float robotposeTheta,
            float goalposeX,
            float goalposeY,
            PrimArray mprim,
            int *prim_id)
{   
    printf("temp=%d\n", temp);
    temp = temp+1;

    static node* n0;
    static node* m0;
    // start node index
    int start_index = GETMAPINDEX((int)(robotposeX / RES + 0.5), (int)(robotposeY / RES + 0.5), x_size, y_size);
    // goal node index
    int goal_index = GETMAPINDEX((int)(goalposeX / RES + 0.5), (int)(goalposeY / RES + 0.5), x_size, y_size);

    // compute heuristic with dijkstra
    // chrono::steady_clock::time_point th = chrono::steady_clock::now(); 

    if (flag_h == 0) {
    	flag_h = 1;
    	// cout << "check" << endl;
    	initial_goal_index = goal_index;
   		get_heuristic(map, x_size, y_size, goalposeX, goalposeY);
    }
    if (start_index == initial_goal_index && flag_h ==1) {
    	heuristic_state = 0;
    }
    // chrono::steady_clock::time_point th2 = chrono::steady_clock::now();
    // chrono::duration<double> time_in_h = chrono::duration_cast<chrono::duration<double>>(th2-th);
    // printf("time used in h:  %fs\n",time_in_h);

    chrono::steady_clock::time_point t_start = chrono::steady_clock::now(); 

    vector<node> open;
    unordered_map<int, node> closed;

    int eps = 10;
    while(eps > 2) {
        closed.clear();
        open.clear();
        // take the smallest node in open list
        // build the start node
        float start_g = 0;
        float start_h = 0;
        float start_f = eps * start_h + start_g;
        int start_parentId = 0;
        int start_prim = 0;
        n0 = new node(robotposeX, robotposeY, robotposeTheta, start_g, start_h, start_f, start_parentId, start_prim, start_index);
        // build the open list
        // initialize open list
        open.push_back(*n0);
        make_heap(open.begin(), open.end(), compare_func());
        // build the closed list;
        // A* loop
        while(!open.empty()) {
            n0 = new node(open.front().x_, open.front().y_, open.front().theta_, open.front().g_, 
                          open.front().h_, open.front().f_, open.front().parent_ID_, open.front().prim_,
                          open.front().cur_ID_);
            // map index of this node
            int current_index = n0->cur_ID_;
            // remove this node in open list
            pop_heap(open.begin(), open.end(), compare_func());
            open.pop_back();
            // insert this node in closed list
            closed.insert(pair<int, node>(current_index, *n0));
            // check if current node is goal
            if (current_index == goal_index) {
                // cout << "find in " << closed.size() << " size" << endl;
                // cout << "found " << endl;
                delete n0;
                delete m0;
                break;
            }
            //find successor from all possible primitive
            int dir;
            int prim;
            dir = getPrimitiveDirectionforRobotPose(n0->theta_);
            for (prim = 0; prim < NUMOFPRIMS; prim ++) {
                float newx, newy, newtheta;
                bool ret;
                ret = applyaction(map, x_size, y_size, n0->x_, n0->y_, n0->theta_,
                                  &newx, &newy, &newtheta, mprim, dir,prim);
                if (ret) {
                    // check if this node is in closed
                    int successor_index = GETMAPINDEX((int)(newx / RES + 0.5), (int)(newy / RES + 0.5), x_size, y_size);
                    // unordered_map<int, node>::const_iterator it = closed.find(successor_index);
                    if (closed.find(successor_index) == closed.end()) {
                        float cost = 0.9;
                        float newg = n0->g_ + cost;
                        // if initial goal is not met, use computed dijkstra heuristic
                        if(heuristic_state == 1) {
                            float newh = heuristic[successor_index];
                            float newf = newg + eps * newh;
                             // check if this node is in open;
                            bool found_in_open = false;
                            for (int i = 0; i < open.size(); i++) {
                                // if it's in open
                                if (successor_index == open[i].cur_ID_) {
                                    found_in_open = true;
                                    // check the g value
                                    if (open[i].g_ > newg) {
                                        open[i].g_ = newg;
                                        open[i].f_ = newg + eps * (newh);
                                        open[i].theta_ = newtheta;
                                        open[i].parent_ID_ = current_index;
                                        open[i].prim_ = prim;
                                        make_heap(open.begin(), open.end(), compare_func());
                                    }
                                    break;
                                }
                            }
                            if (!found_in_open) {
                                m0 = new node (newx, newy, newtheta, newg, newh, newf, current_index, prim, successor_index);
                                open.push_back(*m0);
                                push_heap(open.begin(), open.end(), compare_func());
                            }
                        } else {
                            // else use manhatan heuristic
                            float newh = fabs(newx - goalposeX) + fabs(newy - goalposeY);
                            float newf = newg + eps * newh;
                            // check if this node is in open;
                            bool found_in_open = false;
                            for (int i = 0; i < open.size(); i++) {
                                // if it's in open
                                if (successor_index == open[i].cur_ID_) {
                                    found_in_open = true;
                                    // check the g value
                                    if (open[i].g_ > newg) {
                                        open[i].g_ = newg;
                                        open[i].f_ = newg + eps * (newh);
                                        open[i].theta_ = newtheta;
                                        open[i].parent_ID_ = current_index;
                                        open[i].prim_ = prim;
                                        make_heap(open.begin(), open.end(), compare_func());
                                    }
                                    break;
                                }
                            }
                            if (!found_in_open) {
                                m0 = new node (newx, newy, newtheta, newg, newh, newf, current_index, prim, successor_index);
                                open.push_back(*m0);
                                push_heap(open.begin(), open.end(), compare_func());
                            }
                        }
                        // cout << heuristic[successor_index] << endl;
                    }
                }
            }
        }
        eps = eps - 3;
        chrono::steady_clock::time_point t_end = chrono::steady_clock::now();
        chrono::duration<double> time_in_searching = chrono::duration_cast<chrono::duration<double>>(t_end-t_start);
        if (time_in_searching.count() > 0.9) {
            break;
        }
        // cout << eps << endl;

    }
    // cout << eps << endl;
    // printf("end search\n");
    // chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    // chrono::duration<double> time_in_searching = chrono::duration_cast<chrono::duration<double>>(t2-t_start);
    // printf("time used in searching:  %fs\n",time_in_searching);

    // chrono::steady_clock::time_point t3 = chrono::steady_clock::now();
    // back track
    unordered_map<int, node>::const_iterator back_it = closed.find(goal_index);
    if (back_it != closed.end()) {
	    while (back_it->second.parent_ID_ != start_index) {
	    	int tmp_index_back = back_it->second.parent_ID_;
	    	back_it = closed.find(tmp_index_back);
	    }
        // cout << heuristic[back_it->second.cur_ID_] << endl;
	    *prim_id = back_it->second.prim_;
        // cout << *prim_id << endl;
    } else {
	    *prim_id = 0; /* arbitrary action */
		double mindisttotarget = 1000000;
	    int dir;
	    int prim;
		dir = getPrimitiveDirectionforRobotPose(robotposeTheta);
	    
	    for (prim = 0; prim < NUMOFPRIMS; prim++) {
	        float newx, newy, newtheta;
	        bool ret;
	        ret = applyaction(map, x_size, y_size, robotposeX, robotposeY, robotposeTheta, &newx, &newy, &newtheta, mprim, dir, prim);
	             //skip action that leads to collision 
	        if (ret) {
	            double disttotarget = (double)sqrt(((newx-goalposeX)*(newx-goalposeX) + (newy-goalposeY)*(newy-goalposeY)));
	            if(disttotarget < mindisttotarget){
	                mindisttotarget = disttotarget;
	                
	                *prim_id = prim;
	            }            
	        }
	    }
    }

    // chrono::steady_clock::time_point t4 = chrono::steady_clock::now();
    // chrono::duration<double> time_in_back_tracking = chrono::duration_cast<chrono::duration<double>>(t4-t3);
    // printf("time used in back tracking:  %fs\n",time_in_back_tracking);





    return;
}

/*prhs contains input parameters (3): 
1st is matrix with all the obstacles
2nd is a row vector <x,y> for the robot pose
3rd is a row vector <x,y> for the target pose
plhs should contain output parameters (1): 
1st is a row vector <dx,dy> which corresponds to the action that the robot should make*/

void parseMotionPrimitives(PrimArray mprim)
{
    FILE * fp;
    fp = fopen ("unicycle_8angles.mprim", "r+");
    char skip_c[100];
    int skip_f;
    float resolution;
    int num_angles;
    int num_mprims;
    fscanf(fp, "%s %f", skip_c, &resolution);
    fscanf(fp, "%s %d", skip_c, &num_angles);
    fscanf(fp, "%s %d", skip_c, &num_mprims);

    int i, j, k;
    for (i = 0; i < NUMOFDIRS; ++i) {
        for (j = 0; j < NUMOFPRIMS; ++j) {
            fscanf(fp, "%s %d", skip_c, &skip_f);
            for (k = 0; k < NUMOFINTERSTATES; ++k) {
                fscanf(fp, "%f %f %f", &mprim[i][j][k][0], &mprim[i][j][k][1], &mprim[i][j][k][2]);
            }

        }
    }
}

void mexFunction( int nlhs, mxArray *plhs[], 
		  int nrhs, const mxArray*prhs[] )
     
{

    /* Read motion primtives */
    PrimArray motion_primitives;
    parseMotionPrimitives(motion_primitives);

    /* Check for proper number of arguments */    
    if (nrhs != 3) { 
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Three input arguments required."); 
    } else if (nlhs != 1) {
	    mexErrMsgIdAndTxt( "MATLAB:planner:maxlhs",
                "One output argument required."); 
    } 
        
    /* get the dimensions of the map and the map matrix itself*/     
    int x_size = mxGetM(MAP_IN);
    int y_size = mxGetN(MAP_IN);
    double* map = mxGetPr(MAP_IN);
    
    /* get the dimensions of the robotpose and the robotpose itself*/     
    int robotpose_M = mxGetM(ROBOT_IN);
    int robotpose_N = mxGetN(ROBOT_IN);
    if(robotpose_M != 1 || robotpose_N != 3){
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidrobotpose",
                "robotpose vector should be 1 by 3.");         
    }
    double* robotposeV = mxGetPr(ROBOT_IN);
    float robotposeX = (float)robotposeV[0];
    float robotposeY = (float)robotposeV[1];
    float robotposeTheta = (float)robotposeV[2];
    
    /* get the dimensions of the goalpose and the goalpose itself*/     
    int goalpose_M = mxGetM(GOAL_IN);
    int goalpose_N = mxGetN(GOAL_IN);
    if(goalpose_M != 1 || goalpose_N != 3){
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidgoalpose",
                "goalpose vector should be 1 by 3.");         
    }
    double* goalposeV = mxGetPr(GOAL_IN);
    float goalposeX = (float)goalposeV[0];
    float goalposeY = (float)goalposeV[1];
        
    /* Create a matrix for the return action */ 
    ACTION_OUT = mxCreateNumericMatrix( 1, 1, mxINT8_CLASS, mxREAL); 
    int* action_ptr = (int*) mxGetData(ACTION_OUT);

    /* Do the actual planning in a subroutine */
    planner(map, x_size, y_size, robotposeX, robotposeY, robotposeTheta, goalposeX, goalposeY, motion_primitives, &action_ptr[0]);

    return;
    
}





