/*This source code copyrighted by Lazy Foo' Productions 2004-2024
and may not be redistributed without written permission.*/

//Using SDL, SDL_image, standard IO, math, and strings
#include <SDL2/SDL_image.h>
//#include <SDL_image.h>
#include <stdio.h>
#include <string>
#include <cmath>
#include<vector>
#include<A_Star.hpp>
//Screen dimension constants
const int SCREEN_WIDTH = 640;
const int SCREEN_HEIGHT = 640;

//Starts up SDL and creates window
bool init();

//Loads media
bool loadMedia();

//Frees media and shuts down SDL
void close();

//Loads individual image as texture
SDL_Texture* loadTexture( std::string path );

//The window we'll be rendering to
SDL_Window* gWindow = NULL;

//The window renderer
SDL_Renderer* gRenderer = NULL;

bool init()
{
	//Initialization flag
	bool success = true;

	//Initialize SDL
	if( SDL_Init( SDL_INIT_VIDEO ) < 0 )
	{
		printf( "SDL could not initialize! SDL Error: %s\n", SDL_GetError() );
		success = false;
	}
	else
	{
		//Set texture filtering to linear
		if( !SDL_SetHint( SDL_HINT_RENDER_SCALE_QUALITY, "1" ) )
		{
			printf( "Warning: Linear texture filtering not enabled!" );
		}

		//Create window
		gWindow = SDL_CreateWindow( "A* Grid Search", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN );
		if( gWindow == NULL )
		{
			printf( "Window could not be created! SDL Error: %s\n", SDL_GetError() );
			success = false;
		}
		else
		{
			//Create renderer for window
			gRenderer = SDL_CreateRenderer( gWindow, -1, SDL_RENDERER_ACCELERATED );
			if( gRenderer == NULL )
			{
				printf( "Renderer could not be created! SDL Error: %s\n", SDL_GetError() );
				success = false;
			}
			else
			{
				//Initialize renderer color
				SDL_SetRenderDrawColor( gRenderer, 0xFF, 0xFF, 0xFF, 0xFF );

				//Initialize PNG loading
				int imgFlags = IMG_INIT_PNG;
				if( !( IMG_Init( imgFlags ) & imgFlags ) )
				{
					printf( "SDL_image could not initialize! SDL_image Error: %s\n", IMG_GetError() );
					success = false;
				}
			}
		}
	}

	return success;
}

bool loadMedia()
{
	//Loading success flag
	bool success = true;

	//Nothing to load
	return success;
}

void close()
{
	//Destroy window	
	SDL_DestroyRenderer( gRenderer );
	SDL_DestroyWindow( gWindow );
	gWindow = NULL;
	gRenderer = NULL;

	//Quit SDL subsystems
	IMG_Quit();
	SDL_Quit();
}

SDL_Texture* loadTexture( std::string path )
{
	//The final texture
	SDL_Texture* newTexture = NULL;

	//Load image at specified path
	SDL_Surface* loadedSurface = IMG_Load( path.c_str() );
	if( loadedSurface == NULL )
	{
		printf( "Unable to load image %s! SDL_image Error: %s\n", path.c_str(), IMG_GetError() );
	}
	else
	{
		//Create texture from surface pixels
        newTexture = SDL_CreateTextureFromSurface( gRenderer, loadedSurface );
		if( newTexture == NULL )
		{
			printf( "Unable to create texture from %s! SDL Error: %s\n", path.c_str(), SDL_GetError() );
		}

		//Get rid of old loaded surface
		SDL_FreeSurface( loadedSurface );
	}

	return newTexture;
}

int main( int argc, char* args[] )
{
	//Inspired by the SDL tutorial, lets draw a chessboard first, then we will make is out grid for A-Star search
	//Done with chessboard, lets go A*

	int srcx = 0, srcy = 0;
	int dstx = 3, dsty = 2;
	std::cout<<"Enter start point separated by space or enter\n";
	std::cin>>srcx>>srcy;
	Pair src = std::make_pair(srcx, srcy);
	std::cout<<"Enter goal point separated by space or enter\n";
	std::cin>>dstx>>dsty;

    Pair dest = std::make_pair(dstx, dsty);

	//Create planner instance
    aStarPlanner planner(src, dest);
	std::cout<<dest.first<<","<<dest.second<<std::endl;
	//get path
    std::stack<std::pair<int,int>> Path = planner.serachPath();
    //std::cout<<"grid "<<planner.grid[1][0]<<", "<<planner.grid[0][1]<<std::endl;

	//Convert path to vector as stack do not have a begin and end iterator
	src = std::make_pair(srcx, srcy);
    dest = std::make_pair(dstx, dsty);//Need to fix this
	
	std::vector<std::pair<int,int>> ptList;
	while(!Path.empty())
	{
		auto pt = Path.top();
		ptList.push_back(pt);
		Path.pop();
		std::cout<<"("<<pt.first<<","<<pt.second<<"), ";
	}
	std::cout<<"\n";

	int col = planner.gridCols;
	int rows = planner.gridRows;
	std::cout<<"test "<<planner.grid[8][0]<<std::endl;
	//SDL_Point start = {0,8};
	//SDL_Point goal = {5,5};
							
	if( !init() )
	{
		printf( "Failed to initialize!\n" );
	}
	else
	{
		//Load media
		if( !loadMedia() )
		{
			printf( "Failed to load media!\n" );
		}
		else
		{	
			//Main loop flag
			bool quit = false;

			//Event handler
			SDL_Event e;

			//While application is running
			while( !quit )
			{
				//Handle events on queue
				while( SDL_PollEvent( &e ) != 0 )
				{
					//User requests quit
					if( e.type == SDL_QUIT )
					{
						quit = true;
					}
				}

				//Clear screen
				SDL_SetRenderDrawColor( gRenderer, 0xFF, 0xFF, 0xFF, 0xFF );
				SDL_RenderClear( gRenderer );

				//Render red filled quad
				int square_width = (SCREEN_WIDTH)/(rows+2);
				int start_x  = 0 + square_width;
				int square_height = (SCREEN_HEIGHT)/(col+2);
				int start_y = 0+square_height;

				//The SDL c-sys works as X-Y and not row-col
				//Thus, first coordinate = horizontal coordinate
				//seconds coordinate - vertical coordinate
				

				for(int j = 0; j<col; j++){
					for(int i = 0 ; i <rows; i++){
						
						if(j == src.first && i == src.second){
						}
						else if(j ==dest.first && i == dest.second){
						}
						else if(planner.grid[i][j] == 1)
						{							
							SDL_Rect fillRect = {start_x+j*square_width, start_y+i*square_height, square_width, square_height};
							SDL_SetRenderDrawColor( gRenderer, 0xFF, 0xFF, 0xFF, 0xFF );		
							SDL_RenderFillRect( gRenderer, &fillRect );
						}
						else if(planner.grid[i][j] == 0)
						{							
							SDL_Rect fillRect = {start_x+j*square_width, start_y+i*square_height, square_width, square_height};
							SDL_SetRenderDrawColor( gRenderer, 0x00, 0x00, 0x00, 0xFF );		
							SDL_RenderFillRect( gRenderer, &fillRect );
						}
					}
				}
				for(auto pt: ptList){
					SDL_Rect fillRect = {start_x+pt.second*square_width, start_y+pt.first*square_height, square_width, square_height};
					SDL_SetRenderDrawColor( gRenderer, 0xB6, 0xB2, 0xA7, 0xA7 );		
					SDL_RenderFillRect( gRenderer, &fillRect );
				}
				SDL_Rect fillRect = {start_x+src.second*square_width, start_y+src.first*square_height, square_width, square_height};
				SDL_SetRenderDrawColor( gRenderer, 0x00, 0xFF, 0x00, 0xFF );		
				SDL_RenderFillRect( gRenderer, &fillRect );

				fillRect = {start_x+dest.second*square_width, start_y+dest.first*square_height, square_width, square_height};
				SDL_SetRenderDrawColor( gRenderer, 0xFF, 0xA5, 0x00, 0xFF );		
				SDL_RenderFillRect( gRenderer, &fillRect );


				SDL_Rect outlineRect = { start_x, start_y, col*square_width, rows*square_height};
				SDL_SetRenderDrawColor( gRenderer, 0xFF, 0xA5, 0x00, 0xFF );		
				SDL_RenderDrawRect( gRenderer, &outlineRect );



				//Update screen
				SDL_RenderPresent( gRenderer );
				SDL_Delay(1000);
			
			}
			
		}
	}

	//Free resources and close SDL
	close();

	for(int i = 0 ; i < rows;i++){
		for(int j = 0; j < col; j++){
			std::cout<<planner.grid[i][j]<<", ";
		}
		std::cout<<"\n";
	}

	return 0;
}
