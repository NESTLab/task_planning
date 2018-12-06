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
setof(edge) edges = {<i,j> | i,j in nodes : i != j};

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
activation ak[tasks] = ...;

// activation constant
float fk[tasks];

// Quota = no. of robots required per task
int qk[nodes] = ...;

// Time required to finish the task
int dk[nodes] = ...;


/*********************** Motion attributes ***********************/
// constant velocity of all the robots
int velocity = ...;

// time taken to traverse each egde
float timetaken[nodes][nodes];

// attributes of a decision variable
tuple all_dvars{
int i;
int j;
int r;
}
setof(all_dvars) dvars = {<i,j,r> | i,j in nodes, r in robots : i!=j};



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

	if(e.j ==0){
		g[e.i][e.j] = 0;}	//can't go back to start
	
	if ((e.i > 1) & (e.j > 1)){
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
float H = min(i,j in nodes: i!=j && g[i][j] >0) (g[i][j]); 


// decision variables
dvar boolean xind[dvars];
dvar float+ xtime[dvars];
//dvar float+ y[dvars];


// objective function
dexpr float TotalTime = sum(j in tasks) fk[j] * (sum(r in robots, i in nodes : i!=j) xtime[<i,j,r>]) ;

maximize TotalTime;


// constraints
subject to{
//forall(r in robots, i in nodes, j in nodes : i!=j)
//  Quota2:
//  xind[<i,j,r>] <= 1-Q2 + xtime[<i,j,r>]; 
  
forall(r in robots, i in nodes, j in nodes : i!=j)
  Quota2_new:
  xtime[<i,j,r>] >= Q2 * xind[<i,j,r>]; 

forall(r in robots, i in nodes, j in nodes : i!=j)
  Quota3:
  xtime[<i,j,r>] <= Q3 * xind[<i,j,r>]; 

forall(j in nodes)
  Quota4:
  sum(r in robots, i in nodes : i!=j)
    xind[<i,j,r>] == qk[j];
  
forall(r in robots, i in nodes)
  Correct1:
  sum(j in nodes: j!=i)
    xind[<i,j,r>] <= 1;
     
forall(r in robots, j in nodes)
  Correct2:
  sum(i in nodes: i!=j)
    xind[<i,j,r>] <= 1;
     
//forall(r in robots, i in nodes, j in nodes : i!=j)
//  Quota1:
//  xind[<i,j,r>] <= 1;		// boolean takes care of it 
    
//forall(r in robots, i in nodes, j in nodes : i!=j)
//  Activation:
//  xtime[<i,j,r>] >= dk[i];
  
forall(r in robots, i in nodes, j in nodes : i!=j)
  Motion:
  H*xind[<i,j,r>] <= g[i][j];
  
}


