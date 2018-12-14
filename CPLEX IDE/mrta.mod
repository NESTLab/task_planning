/*********************************************
 * OPL 12.8.0.0 Model
 * Author: dharini
 * Creation Date: Nov 3, 2018 at 1:56:55 PM
 *********************************************/
 
/*************************************************************
*********************** initialization ***********************
*************************************************************/

// Number of robots
int R = ...;
range robots = 1..R;

// Number of tasks
int tau = ...;
range tasks = 1..tau;
range nodes = 0..tau;	//start + targets


/*********************** Graph attributes ***********************/
// Graph edges connecting start positions and all targets
tuple edge{
int i;
int j;
}
setof(edge) edges = {<i,j> | i,j in nodes};

// distance between nodes
float edist[edges];


// rechability
float g[nodes][nodes];


/*********************** Task attributes ***********************/
// task locations
tuple location{
float x;
float y;
}
location Location[nodes];

// Activation duration of tasks
tuple activation{
int start;
int end;
}
activation ak[nodes] = ...;

// activation constant
float fk[nodes];

// Quota = no. of robots required per task
int qk[nodes] = ...;

// Time required to finish the task
int dk[nodes] = ...;


/*********************** Motion attributes ***********************/
// constant velocity of all the robots
float velocity = ...;

// time taken to traverse each egde
float timetaken[nodes][nodes];

// attributes of a decision variable
tuple all_dvars{
int r;
int i;
int j;
}
setof(all_dvars) dvars = {<r,i,j> |  r in robots, i,j in nodes};



/*************************************************************
*********************** pre-processing ***********************
*************************************************************/

// generating the graph
execute{
// calculating euclidean distance

function getDistance(task1,task2){
return Opl.sqrt(Opl.pow(task1.x-task2.x,2)+Opl.pow(task1.y-task2.y,2))
}


// random position of tasks
for(var i in tasks){	// start is origin
Location[i].x = Opl.rand(100); 
Location[i].y = Opl.rand(100);
}



// storing the distances
for(var e in edges){
edist[e] = getDistance(Location[e.i],Location[e.j])
}

// activation constant
fk[0] = 0;
for(var i in tasks){
fk[i] = 1/(ak[i].end - ak[i].start);
}

// time taken to traverse each egde
for(var e in edges){
timetaken[e.i][e.j] = edist[e]/velocity;
}

// adjacency matrix
for(var e in edges)
{
	g[e.i][e.j] = timetaken[e.i][e.j];

	if(e.j == 0){
		g[e.i][e.j] = 0;}	//can't go back to start
	
	if ((e.i > 0) & (e.j > 0)){
		if((ak[e.i].start + timetaken[e.i][e.j]) > (ak[e.j].end - dk[e.j])){
			g[e.i][e.j] = 0;}
			
		if((ak[e.i].end + timetaken[e.i][e.j]) > (ak[e.j].end - dk[e.j])){
			g[e.i][e.j] = 0;}
			
		if((ak[e.i].end + timetaken[e.i][e.j]) < ak[e.j].start){
			g[e.i][e.j] = 0;} 
	}
  }
	
}

/*************************************************************
***************************** Model **************************
*************************************************************/

// Other constants
int Q2 = (min(i in tasks) dk[i]);
int Q3 = (max(i in tasks) (ak[i].end - ak[i].start));
float H = min(i,j in nodes: g[i][j] >0) (g[i][j]); 

// decision variables
dvar boolean xind[dvars];
dvar float+ xtime[dvars];


// objective function
dexpr float TotalTime = sum(j in nodes) fk[j] * (sum(r in robots, i in nodes) xtime[<r,i,j>]) ;
maximize TotalTime;


// constraints
subject to {
 
forall(r in robots, i in nodes, j in nodes){
  Quota2_new:
  xtime[<r,i,j>] >= Q2 * xind[<r,i,j>]; 
}

forall(r in robots, i in nodes, j in nodes){
  Quota3:
  xtime[<r,i,j>] <= Q3 * xind[<r,i,j>]; 
}

forall(j in nodes){
  Quota4:
  sum(r in robots, i in nodes) xind[<r,i,j>] == qk[j];
}  
  
forall(r in robots, i in nodes){
  Correct1:
  sum(j in nodes) xind[<r,i,j>] <= 1;
}

forall(r in robots, j in nodes){
  Correct2:
  sum(i in nodes) xind[<r,i,j>] <= 1; 
}
    
forall(r in robots, i in nodes, j in nodes){
  Motion:
  H*xind[<r,i,j>] <= g[i][j];
}
   
forall(r in robots, i in nodes,j in nodes){
  define:
  xtime[<r,i,j>] + xind[<r,i,j>] * (ak[i].end + dk[i] + timetaken[i][j] - ak[j].end) - sum(k in nodes)xtime[<r,k,i>] <= 0;
  }

}





