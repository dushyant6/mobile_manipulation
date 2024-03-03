#pragma once
#include<vector>
#include<iostream>
#include<bits/stdc++.h>

#define ROW 9
#define COL 10

int graph[ROW][COL]
        = { { 1, 0, 1, 1, 1, 1, 0, 1, 1, 1 },
            { 1, 1, 1, 0, 1, 1, 1, 0, 1, 1 },
            { 1, 1, 1, 0, 1, 1, 0, 1, 0, 1 },
            { 0, 0, 1, 0, 1, 0, 0, 0, 0, 1 },
            { 1, 1, 1, 0, 1, 1, 1, 0, 1, 0 },
            { 1, 0, 1, 1, 1, 1, 0, 1, 0, 0 },
            { 1, 0, 0, 0, 0, 1, 0, 0, 0, 1 },
            { 1, 0, 1, 1, 1, 1, 0, 1, 1, 1 },
            { 1, 1, 1, 0, 0, 0, 1, 0, 0, 1 } };

typedef std::pair<int, int> Pair;

typedef std::pair<double, std::pair<int, int> > pPair;

struct cell{
    int parent_i, parent_j;
    double f, g, h;
};


class aStarPlanner{
public:
    aStarPlanner(std::pair<int, int> start, std::pair<int, int> goal);
    int gridRows, gridCols;

    std::pair<int, int> start, goal;
    int grid[ROW][COL];

    bool isValid(int row, int col){return (row>=0) && (row<ROW) && (col>= 0) &&(col < COL);}

    bool isUnblocked(int row, int col);

    bool isDestination(int row, int col);

    std::stack<Pair> tracePath(cell cellDetails[][COL]);

    std::stack<Pair> serachPath();

private:
    double calculateH(int row, int col);

};

class node{
public:
    double f, h;
    node* parent;
    std::vector<node*> neighbor_list;

    node();

    ~node();

private:

};

aStarPlanner::aStarPlanner(std::pair<int, int> src, std::pair<int,int> dst){
    this->start.first = src.first;
    this->start.second = src.second;
    this->goal = dst;
    this->gridRows = ROW;
    this->gridCols = COL;
    

    std::cout<<"grph "<<graph[1][0]<<", "<<graph[0][1]<<std::endl;

    for (int i = 0; i < ROW; i++) {
        for (int j = 0; j < COL; j++) {
            grid[i][j] = graph[i][j];
        }
    }
    for(int i = 0 ; i < ROW;i++){
		for(int j = 0; j < COL; j++){
			std::cout<<grid[i][j]<<", ";
		}
		std::cout<<"\n";
	}

}

bool aStarPlanner::isUnblocked(int row, int col){
    //grid[row][col] == 1 ->  free cell
    //grid[row][col] == 0 ->blocked cell
    for (int i = 0; i < ROW; i++) {
        for (int j = 0; j < COL; j++) {
            grid[i][j] = graph[i][j];
        }
    }

    return (grid[row][col] == 1);
}

bool aStarPlanner::isDestination(int row, int col){
    return (goal.first == row) && (goal.second == col);
}

double aStarPlanner::calculateH(int row, int col){
    double EuclideanDist = std::hypot((goal.first - row), (goal.second-col));
    	return ((double)sqrt(
		(row - goal.first) * (row - goal.first)
		+ (col - goal.second) * (col - goal.second)));

    //return EuclideanDist;
}



std::stack<Pair> aStarPlanner::tracePath(cell cellDetails[][COL]){
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

std::stack<Pair> aStarPlanner::serachPath(){
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

    bool closedList[ROW][COL];
    memset(closedList, false, sizeof(closedList));

    cell cellDetails[ROW][COL];

    for(int i = 0; i < ROW; i++){
        for(int j = 0; j < COL; j++){
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
    
    //Create open list with coords anf f where f = g + h
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
