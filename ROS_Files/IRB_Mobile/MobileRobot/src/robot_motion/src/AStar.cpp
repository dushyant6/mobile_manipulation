#pragma once
#include<vector>
#include<stack>
#include<iostream>
#include<bits/stdc++.h>

#include "nav_msgs/msg/occupancy_grid.hpp"


#define GRIDROWS 1010
#define GRIDCOLS 1010


class Map
{
  public:
    int width;   
    int height;
    double resolution;
    std::vector<double> origin;
    std::vector<std::vector<int>> grid;
    void initGrid()
    {
      std::vector<int> tempRow(width,0);
      std::vector<std::vector<int>> tempGrid(height, tempRow);
      grid = tempGrid;
    }
    void fillGrid(const nav_msgs::msg::OccupancyGrid & msg)
    {
     initGrid();
     int CurrRow;
     int CurrCol;
     for (int i = 0; i < width*height; i++)
      {
       CurrRow = i / width;
       CurrCol = i % width;
       grid[CurrRow][CurrCol] = msg.data[i];
      }
    }
};

typedef std::pair<int, int> Pair;

typedef std::pair<double, std::pair<int, int> > pPair;

struct cell{
    int parent_i, parent_j;
    double f, g, h;
};

struct Node
{
  public:
    std::vector<double> coord; //x, y coordinates in x and y axis
    std::vector<int> discCoord; //discrete coordinates of the robot on the map
    std::vector<std::vector<double>> traj; //continuous trajectory of the robot from the current node to next one
    double g; // cost from start node to current node
    double h; // heuristic cost from current node to goal node
    double f; //Total cost of this node(g+h)
    std::vector<int> parentDiscCoord; //discrete coordinates of the parent  
    bool isCollision; // Node is in collision or not 
    
};

class aStarPlanner{
public:
    aStarPlanner(std::pair<int, int> start, std::pair<int, int> goal, double startOrientation, std::vector<std::vector<int>> gr);
    int gridRows, gridCols;
    int row, col;
    
    std::vector<double> mapOrigin;
    std::vector<std::vector<int>> grid;
    std::vector<std::vector<Node>> gridNodes;
    
    bool closedList[1010][1010];
    cell cellDetails[1010][1010];

    std::pair<int, int> start, goal;
    
    
    void initGridNodes();
    
    bool isValid(double row, double col){return (row>=0) && (row<GRIDROWS) && (col>= 0) &&(col < GRIDCOLS);}

    bool isUnblocked(int row, int col);

    bool isDestination(double row, double col);

    std::stack<Pair> tracePath(cell cellDetails[][GRIDCOLS]);

    std::stack<Pair> searchPath();

private:
    double calculateH(double row, double col);

};


aStarPlanner::aStarPlanner(std::pair<int, int> src, std::pair<int,int> dst, double startOrientation, std::vector<std::vector<int>> gr){
    std::cout<<"Initialize the planner\n";
    this->start.first = src.first;
    this->start.second = src.second;
    this->goal = dst;
    this->row = gr.size();
    this->col = gr[0].size();


}

void aStarPlanner::initGridNodes()
{
    double resolution = 0.1;
    goal = {goal.first/resolution, goal.second/resolution};
    start = {start.first/resolution, start.second/resolution};

    std::cout<<"Rows x cols = "<<row<<"x"<<col<<std::endl;
    for (int i = 0; i < row; i++)
    {
    std::vector<Node> tempRow;
    for (int j = 0; j < col; j++)
    { 
    
    Node temp;
    temp.discCoord = {i, j};   
    temp.g = FLT_MAX;
    temp.f = FLT_MAX;
    temp.h = FLT_MAX;  
    if (grid[i][j] == 100)
    {
        temp.isCollision = true;
    }
    else
    {
        temp.isCollision = false;
    }
    tempRow.push_back(temp);
    
    }
    gridNodes.push_back(tempRow);
    }
    
    
    memset(closedList, false, sizeof(closedList));

    
}

bool aStarPlanner::isUnblocked(int row, int col){
    return (grid[row][col] != 0);
}

bool aStarPlanner::isDestination(double row, double col){
    double goalThreshold = 0.005;
    double distFromGoal = sqrt((goal.first - row)*(goal.first - row) + (goal.second - col)*(goal.second - col));
    if (abs(distFromGoal) < goalThreshold){
        return true;
    }
    return false;
}

double aStarPlanner::calculateH(double row, double col){
    double EuclideanDist = std::hypot((goal.first - row), (goal.second-col));
    	return ((double)sqrt(
		(row - goal.first) * (row - goal.first)
		+ (col - goal.second) * (col - goal.second)));

    //return EuclideanDist;
}



std::stack<Pair> aStarPlanner::tracePath(cell cellDetails[][GRIDCOLS]){
    int row = goal.first;
    int col = goal.second;
 
    std::stack<Pair> Path;

 
    while (!(cellDetails[row][col].parent_i == row
             && cellDetails[row][col].parent_j == col)) {
        Path.push(std::make_pair(row, col));
        int temp_row = cellDetails[row][col].parent_i;
        int temp_col = cellDetails[row][col].parent_j;
        row = temp_row;
        col = temp_col;
    }
 
    Path.push(std::make_pair(row, col));
    return Path;
    /*
    while (!Path.empty()) {
        std::pair<int, int> p = Path.top();
        Path.pop();
        printf("-> (%d,%d) ", p.first, p.second);
    }
 
    return;
    */
}

std::stack<Pair> aStarPlanner::searchPath(){
    std::cout<<"Start = "<< grid[start.first][start.second]<<std::endl;
    std::cout<<"Goal = "<< grid[goal.first][goal.second]<<std::endl;
    if(isValid(start.first, start.second) == false || isValid(goal.first, goal.second) == false){
        std::cout<<"Start or goal are invalid\n";
        return {};
    }

    if(isUnblocked(start.first, start.second) == false || isUnblocked(goal.first, goal.second) == false){
        std::cout<<"Start or goal are already blocked\n";
        return {};
    }

    if(isDestination(start.first, start.second)){
        std::cout<<"Goal reached\n";
        return {};
    }


    for(int i = 0; i < GRIDROWS; i++){
        for(int j = 0; j < GRIDCOLS; j++){
            cellDetails[i][j].f = std::numeric_limits<float>::max();
            cellDetails[i][j].g = std::numeric_limits<float>::max();
            cellDetails[i][j].h = std::numeric_limits<float>::max();
            cellDetails[i][j].parent_i = -1;
            cellDetails[i][j].parent_j = -1;
        }
    }

    cellDetails[start.first][start.second].f = 0;
    cellDetails[start.first][start.second].g = 0;
    cellDetails[start.first][start.second].h = 0;
    cellDetails[start.first][start.second].parent_i = start.first;
    cellDetails[start.first][start.second].parent_j = start.second;
    
    //Create open list with coords and f where f = g + h
    std::set<std::pair<int, std::pair<int,int>>> openList;
    openList.insert(std::make_pair(0.0, std::make_pair(start.first, start.second)));

    bool foundDest = false;

    while(!openList.empty()){
        std::pair<int, std::pair<int,int>> current = *openList.begin();
        
        openList.erase(openList.begin());

        //Add this vertex to closed list
        int x = current.second.first;//this is actually y-coordinate (row num)
        int y = current.second.second;//this is actually x-coordinate (col num)
        closedList[x][y] = true; 

        //find all 8 neghbors and check their g, h, f
        double gnew, fnew, hnew;

        for(int i = -1;i<2;i++){
            for(int j = -1; j < 2; j++){
                //std::cout<<"i, j = "<<i<<", "<<j<<std::endl;
                if(i == 0 && j == 0){
                    continue;
                }
                if(isValid(x-i, y-j)){
                    //check if goal reached
                    if(isDestination(x-i, y-j)){
                        //cellDetails[x-i][y-j].g = cellDetails[x][y].g + 1;
                        cellDetails[x-i][y-j].parent_i = x;
                        cellDetails[x-i][y-j].parent_j = y;
                        foundDest = true;
                        std::cout<<"Goal found\n";
                        //tracePath(cellDetails);
                        std::stack<Pair> Path = this->tracePath(cellDetails);
                        /*
                        for(int i = 0; i < ROW; i++){
                            for (int j = 0; j< COL; j++){
                                grid[i][j] = graph[i][j];
                            }
                        }*/
                        return Path;
                    }
                    else if(closedList[x-i][y-j] == false && isUnblocked(x-i,y-j)){
                        gnew = cellDetails[x][y].g + 1;
                        hnew = calculateH(x-i,y-j);
                        //fnew = cellDetails[x-i][y-j].g + cellDetails[x-i][y-j].h;
                        fnew = gnew + hnew;
                        //If this cell is not in openList, change its parents
                        // If the f value of this cell is less than the new f values, update its attributes

                        if(cellDetails[x-i][y-j].f == std::numeric_limits<float>::max() ||
                            cellDetails[x-i][y-j].f > fnew
                            ){
                                cellDetails[x-i][y-j].g = gnew;
                                cellDetails[x-i][y-j].h = hnew;
                                cellDetails[x-i][y-j].f = fnew;
                                cellDetails[x-i][y-j].parent_i = x;
                                cellDetails[x-i][y-j].parent_j = y;
                                openList.insert(std::make_pair(fnew, std::make_pair(x-i,y-j)));
                            }
                    
                    }
                }
            }
        }

    }
    if(foundDest == false){
        std::cout<<"A feasible path could not be found\n";
        return {};
    }

}


class globPlanner
{
  public:
    globPlanner(){
        std::cout<<"Creating global planner\n";
    }

    void test(std::vector<double> start, std::vector<double> goal){
        std::cout<<"Able to call the ffunctions\n";
        std::cout<<"Start = "<<start[0]<<", "<<start[1]<<std::endl;
    }

    

    std::vector<std::vector<double>> findGlobalPath(const nav_msgs::msg::OccupancyGrid& mapMsg,
          std::vector<double> start, std::vector<double> goal)
    {
      
      std::cout<<"Inside globalPlanner\n";

      //Initialize the map
      Map map;
      map.width = mapMsg.info.width;
      map.height = mapMsg.info.height;
      map.resolution = mapMsg.info.resolution;
      map.origin = {mapMsg.info.origin.position.x, mapMsg.info.origin.position.y}; 
      map.fillGrid(mapMsg); 
      std::cout<<"W = "<<map.width<<", H = "<<map.height<<std::endl;

      std::cout<<"Start = ";
      for(auto cord: start){
        std::cout<<cord<<", ";
      }

      std::cout<<"\n";
      start[0] = (start[0] - map.origin[0]) / map.resolution;
      start[1] = (start[1] - map.origin[1]) / map.resolution;
      goal[0] = (goal[0] - map.origin[0]) / map.resolution;
      goal[1] = (goal[1] - map.origin[1]) / map.resolution;

      std::cout<<"start = "<<start[0]<<", "<<start[1]<<std::endl;

      // Initialize Hybrid A* Parameters
      std::pair<int, int> startPt;
      startPt.first = start[0];
      startPt.second= start[1];
      std::pair<int, int> goalPt;
      goalPt.first = goal[0];
      goalPt.second = goal[1];

      std::vector<std::vector<double>> pathFinal;
        
      
      aStarPlanner planner(startPt, goalPt, 0.0, map.grid);
      /*
      // Run Hybrid A*      
      std::cout<< "Running A star\n";
      std::stack<std::pair<int, int>> path = planner.searchPath();

        std::vector<std::vector<double>> pathFinal;
        while (!path.empty()) {
            std::pair<int, int> pt = path.top();
            path.pop();

            std::vector<double> temp;
            temp.push_back(pt.first);            
            temp.push_back(pt.second);  
            temp.push_back(0.0);    
            pathFinal.push_back(temp);

            //printf("-> (%d,%d) ", p.first, p.second);
        }
        */
        pathFinal = {{}};
        return pathFinal;

    }

};