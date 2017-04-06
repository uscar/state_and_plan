#include start_header.h
#include <vector>
#include <cmath>

#define GRIDWIDTH = 20.00 
#define GRIDHEIGHT = 20.00

using namespace std;

planner::planner()
{

}

int width, height;

struct Square{
	double width;
	double height;
	vector2 pos;
	//double score;
	double minX, maxX;
	double minY, maxY;
	//double score = 0;
};

vector<Square> planner::InitializeSquares(World world)
{
	width = GRIDWIDTH;
	height = GRIDHEIGHT;

	
	vector<Square> squares;
	for(int i = 0; i < numSquaresWidth; i++)
	{
		for(int j = 0; j < numSquaresHeight; j++)
		{
			Square s;
			s.width = GRIDWIDTH / numSquaresWidth;
			s.height = GRIDHEIGHT / numSquaresHeight;
			s.pos = new vector2(i + s.width/2, j + s.height/2);
			s.minX = i;
			s.maxX = i + s.width;
			s.minY = j;
			s.maxY = j + s.height;
			squares.push_back(s);
		}
	}

	return squares;
}

//perform all score computations for all roombas in the square, returns the highest score 
double planner::ComputeSquareScore(vector<roomba> roombas)
{

	double dropInFrontScore = 0, singleDropScore = 0, doubleDropScore = 0, tripleDropScore = 0;
	double highestScore = 0;
	for(int i = 0; i < roombas.size(); i++)
	{
		
		dropInFrontScore = drone.dropInFrontSim(2.0f);
		singleDropScore = drone.singleDropSim(2.0f);
		doubleDropScore = drone.doubleDropSim(2.0f);
		tripleDropScore = drone.tripleDropSim(2.0f);

		highestScore = dropInFrontScore;
		if(highestScore < singleDropScore)
		{
			highestScore = singleDropScore;
		}
		if(highestScore < doubleDropScore)
		{
			highestScore = doubleDropScore;
		}
		if(highestScore < tripleDropScore)
		{
			highestScore = tripleDropScore;
		}
	}	
	return highestScore;
}

double planner::MaxOfAvgSquare(vector<World> worlds)
{
	double avgScores[40];
	double squareScore = 0;
	for(int i = 0; i < 40; i++)
	{
		for(auto w : worlds)
		{
			squareScore += w.listOfSquares[i];
		}
		avgScores[i] = squareScore/40;
	}

	return get_max_pos(avgScores, 40);
}

int get_max_pos(double * array, int size)
{
    double max=array[0];
    int max_pos=0;

    int i;
    for (i=1; i<size; i++)
    {
        if (max<array[i])
        {
            max=array[i];
            max_pos=i;
        }
    }

    return max_pos;
}

//TODO
/*	computeScorePerSquare(vector<Square>)
	calculates score of each square based on orientation/time/other pertinent factors of roomba

	calculate the score of each possible action on the roomba's in the square
	Create an array of actions/scores for each square
	Get average score 
*/


/*
Takes best score of the action we can take for the action

*/

vector<Square> scoredSquares;


//may have to scrap density and fix
vector<Square> planner::ComputeDensity(World world)
{
	vector<roomba> roombas = world.GetRoombas();
	
	for(int i = 0; i < roombas.size(); i++)
	{
		for(int j = 0; j < squares.size(); j++)
		{	
			if(squares[j].minX < roombas[i].pos.x 
				&& squares[j].maxX > roombas[i].pos.x
				&& squares[j].minY < roombas[i].pos.y 
				&& squares[j].maxY > roombas[i].pos.y)
			{
				scoredSquares.push_back(squares[j]);
				squares[j].score++; // temp method of computing score, will write a better one later in this file
			}
		}
	}
	return scoredSquares;
}



