#include "Map.h"

/************************************************************************/
/* Map C'tor															*/
/************************************************************************/
Map::Map(int src_x, int src_y, int tgt_x, int tgt_y)
{
	//Set src and tgt pts
	this->src_x = src_x;
	this->src_y = src_y;
	this->tgt_x = tgt_x;
	this->tgt_y = tgt_y;
	init();
	findAngles();
	decomposite();
	setStations();
	/*After the C'tor is called, the matrix is decomposited and the matrix
	  is divided by cells, each cell has a number, and stations which the 
	  robot should follow is filled in stations matrix (can be taken calling
	  getNextStation(x, y) function.										*/
}

/************************************************************************/
/* Returns current station point                                        */
/************************************************************************/
bool Map::getNextStation(int &x, int &y)
{
	/*Keep returning the station position till there is no stations*/
	currStation++;
	for(int i(0); i<MAP_WIDTH; i++)
		for(int j(0); j<MAP_HIGHT; j++)
			if(stations[i][j] == currStation)
			{
				x=i;
				y=j;
				return true;
			}
			x=-1;
			y=-1;
			return false;
}

/************************************************************************/
/* Returns current station number                                       */
/************************************************************************/
int Map::getCurrStation()
{
	return currStation;
}

/************************************************************************/
/* Returns minimal distance obstacles and bounds from a point           */
/************************************************************************/
int Map::getDistance(int x, int y)
{
	/* In order to choose the relevant path when there are more than one
	   we calculate the distance of a point which is the minimal distance
	   from an obstacle or bound. The point with the minimal distance is
	   the desirable one.

	   When we have two paths?
	     +------------------------------------------+
		 |											|
		 |											|
		 |											|
		 |											|
		 |  (x1,y1) ------------------->			|	
		 |		|						|			|
		 |		|						|			|
		 |		|						|			|
		 |		|						|			|
		 |		|(path2)				| (Path1)	|
		 |		|						|			|
		 |		|						|			|
		 |		|						|			|
		 |		|						|			|
		 |		V						V			|
		 |		--------------------->(x2,y2)		|
		 |											|
		 |											|
		 |											|
		 |											|
		 +------------------------------------------+
	*/
	int minX = min(MAP_WIDTH-x, x);
	int minY = min(MAP_HIGHT-y, y);

	for(int i(0); i<MAP_WIDTH; i++)
		if(m[i][y] == -1)
			minX = min(minX, abs(x-i));

	for(int j(0); j<MAP_HIGHT; j++)
		if(m[x][j] == -1)
			minY = min(minY, abs(y-j));

	return min(minX, minY);
}

/************************************************************************/
/* Given a point and orientation, returns 4 distances                   */
/************************************************************************/
void Map::getDistances(int x, int y, Orientation o,
				  int &northDist, int &southDist,
				  int &eastDist , int &westDist)
{
	/*
	  In order to keep going in the same way, the robot needs to know its distance from
	  an obstacle in a given point with a given orientation, this function returns the
	  relevant distances. at first we calculates dist1, ... , dist4 as shows, then we fit
	  them for the given variables corresponding to the orientation.
	+---------------------------+
	|							|
	|							|
	|			S(2)			|
	|							|
	|	  E(3)		  W(4)		|
	|							|
	|			N(1)			|
	|							|
	|							|
	+---------------------------+
	*/
	int dist1=MAP_HIGHT-1, dist2=MAP_HIGHT-1, dist3=MAP_WIDTH-1, dist4=MAP_WIDTH-1;

	for(int j(y); j<MAP_HIGHT; j++)
		if(m[x][j] == -1)
			dist1 = min(dist1, j-y); /*North*/

	for(int j(0); j<y; j++)
		if(m[x][j] == -1)
			dist2 = min(dist2, y-j); /*South*/

	for(int i(0); i<x; i++)
		if(m[i][y] == -1)
			dist3 = min(dist3, x-i); /*East*/

	for(int i(x); i<MAP_WIDTH; i++)
		if(m[i][y] == -1)
			dist4 = min(dist4, i-x); /*West*/

	if(o == NORTH)
	{
		northDist = dist1;
		southDist = dist2;
		eastDist  = dist3;
		westDist  = dist4;
	}
	else if(o == WEST)
	{
		northDist = dist4;
		southDist = dist3;
		eastDist  = dist1;
		westDist  = dist2;
	}
	else if(o == SOUTH)
	{
		northDist = dist2;
		southDist = dist1;
		eastDist  = dist4;
		westDist  = dist3;
	}
	else if(o == EAST)
	{
		northDist = dist3;
		southDist = dist4;
		eastDist  = dist2;
		westDist  = dist1;
	}
	else
	{
		throw "Bad Orientation.";
	}
}
/************************************************************************/
/* Prints the map                                                       */
/************************************************************************/
void Map::print()
{
	printMatrix(m);
}

/************************************************************************/
/* Prints angles matrix                                                 */
/************************************************************************/
void Map::printAngles()
{
	printMatrix(angles);
}

/************************************************************************/
/* Prints stations matrix numbered                                      */
/************************************************************************/
void Map::printStations()
{
	printMatrix(stations);
}

/************************************************************************/
/* Default D'tor                                                        */
/************************************************************************/
Map::~Map()
{
	/*do nothing, since we don't new anything*/
};

/************************************************************************/
/* Init the map matrix with src, tgt and obstacles                      */
/************************************************************************/
void Map::init()
{
	/* Zero everything */
	currStation = 0;
	for(int i(0); i<MAP_WIDTH; i++)
		for(int j(0); j<MAP_HIGHT; j++)
		{
			m[i][j] = 0;
			angles[i][j] = 0;
			stations[i][j] = 0;
			changing[j] = 0;
		}
	/* Obstacles points = -1 */
		m[src_x][src_y] = 'S';
		m[tgt_x][tgt_y] = 'T';

		for(int i(0); i<OBSTACLE_WIDTH; i++)
			for(int j(0); j<OBSTACLE_HIGHT; j++)
			{
				if(isSartEndPoint(OBS_1_X+i, OBS_1_Y+j) ||
					isSartEndPoint(OBS_2_X+i, OBS_2_Y+j) ||
					isSartEndPoint(OBS_3_X+i, OBS_3_Y+j) ||
					isSartEndPoint(OBS_4_X+i, OBS_4_Y+j))
					throw "Bad Obstacle, collision with Source/Target point";


				m[OBS_1_X+i][OBS_1_Y+j] = -1;
				m[OBS_2_X+i][OBS_2_Y+j] = -1;
				m[OBS_3_X+i][OBS_3_Y+j] = -1;
				m[OBS_4_X+i][OBS_4_Y+j] = -1;
			}
}

/************************************************************************/
/* Sets stations matrix with numbers                                    */
/************************************************************************/
void Map::setStations()
{	
	int currStation(1), j;
	stations[src_x][src_y] = currStation++;
	for(int i(0); i<MAP_HIGHT; i++)
	{
		if(changing[i] == 1)
		{
			j = getNextStation(i);
			stations[j][i] = currStation++;
		}
	}
	stations[tgt_x][tgt_y] = currStation++;
}

/************************************************************************/
/* Recursive function which calculates next station X position given    */
/* changing Y position. (changing Y position is Y axis on which we move */
/* from a station to another.                                           */
/************************************************************************/
int Map::getNextStation(int i)
{
	int first, last(MAP_WIDTH-1);
	for(int j(0); j<MAP_WIDTH; j++)
		if(m[j][i] != -1)
		{
			first = j;
			break;
		}
		for(int j(0); j<MAP_WIDTH; j++)
			if(m[j][i] != -1 && m[j+1][i] == -1 && j > first)
			{
				last = j;
				break;
			}
			int diff = last - first;

			if(diff == MAP_WIDTH-1)
				return getNextStation(i-1);
			return (first + diff/2 +1);
}

/************************************************************************/
/* De-composite the map with cell numbers. After this function call the */
/* map will be numbered by cells                                        */
/************************************************************************/
void Map::decomposite()
{
	int cellNum(1);
	for(int i(0); i<MAP_HIGHT; i++)
	{
		if( (!isAngleInRow(i)) || (!isAngleInRow(i-1)) )
		{
			for(int j(0); j<MAP_WIDTH; j++)
				if(m[j][i] == 0)
					m[j][i] = cellNum;
		}
		else //isAngleInRow(i)=true && isAngleInRow(i-1)=true
		{
			changing[i] = 1;
			cellNum++;
			for(int j(0); j<MAP_WIDTH; j++)
				if(m[j][i] == 0)
					m[j][i] = cellNum;
		}

	}
}

/************************************************************************/
/* Check if a row has an angle of an obstacle                           */
/************************************************************************/
bool Map::isAngleInRow(int row)
	{
		for(int i(0); i<MAP_WIDTH; i++)
			if(angles[i][row] == 1) return true;
		return false;
	}

/************************************************************************/
/* Fills the angles matrix                                              */
/************************************************************************/
void Map::findAngles()
{
	for(int i(0); i<MAP_WIDTH; i++)
		for(int j(0); j<MAP_HIGHT; j++)
			if(isAngle(i,j))
			{
				angles[i][j] = 1;
			}
}

/************************************************************************/
/* Check if a point is an angle of an obstacle                          */
/************************************************************************/
bool Map::isAngle(int x, int y)
{
	/* 
	Angle types:
					+-------					--------+		
	|				|					|				|
	|  (1)			|  (2)		  (3)	|		   (4)	|
	|				|					|				|
	+--------					 -------+
	
	How we know if a point is an angle?
	  We check all 9 cells 0 0 0
	                       0 0 0
						   0 0 0 around the point
	  
	  if we got 5 ones and 4 zeros or vice versa means
	  it's a point, otherwise, it's not.
	*/

	int zeros = 0, ones = 0;
	Bound b;
	b = isInBounds(x,y);
	if(b == LEFT_BOUND   && m[x][y] == 0 && (m[x][y+1] == -1 || m[x][y-1] == -1))
		return true;
	if(b == RIGHT_BOUND  && m[x][y] == 0 && (m[x][y+1] == -1 || m[x][y-1] == -1))
		return true;
	if(b == TOP_BOUND	 && m[x][y] == 0 && (m[x+1][y] == -1 || m[x-1][y] == -1))
		return true;
	if(b == BUTTOM_BOUND && m[x][y] == 0 && (m[x+1][y] == -1 || m[x-1][y] == -1))
		return true;

	if(b == LEFT_BOUND || b == RIGHT_BOUND || b == TOP_BOUND || b == BUTTOM_BOUND || b == BASE_ANGLE)
		return false;

	if( m[x-1][y-1] == 0  )	zeros++;
	if( m[x-1][y-1] == -1 )	ones++;		
	if( m[x-1][y] == 0  )	zeros++;
	if( m[x-1][y] == -1 )	ones++;		
	if( m[x-1][y+1] == 0  )	zeros++;
	if( m[x-1][y+1] == -1 )	ones++;
	if( m[x][y-1] == 0 )	zeros++;
	if( m[x][y-1] == -1 )	ones++;
	if( m[x][y] == 0 )		zeros++;
	if( m[x][y] == -1 )		ones++;
	if( m[x][y+1] == 0 )	zeros++;
	if( m[x][y+1] == -1 )	ones++;
	if( m[x+1][y-1] == 0  )	zeros++;
	if( m[x+1][y-1] == -1 )	ones++;
	if( m[x+1][y] == 0  )	zeros++;
	if( m[x+1][y] == -1 )	ones++;		
	if( m[x+1][y+1] == 0  )	zeros++;
	if( m[x+1][y+1] == -1 )	ones++;		

	if( (zeros == 5 && ones == 4) ||
		(zeros == 4 && ones == 5) )
		return true;
	return false;
}

/************************************************************************/
/* Check if a point is start or target points                           */
/************************************************************************/
bool Map::isSartEndPoint(int x, int y)
{
	if( (m[x][y] == 'S') || (m[x][y] == 'T') )
		return true;
	else
		return false;
}

/************************************************************************/
/* Prints a given matrix                                                */
/************************************************************************/
void Map::printMatrix(int M[MAP_WIDTH][MAP_HIGHT])
{
	for(int i(0); i<MAP_HIGHT; i++)
	{
		for(int j(0); j<MAP_WIDTH; j++)
		{
			if(M[j][i] >=0 && M[j][i] <= 9)
			{
				cout << " " << M[j][i];
			}
			else
			{
				cout << M[j][i];
			}
		}
		cout << endl;
	}
}

/************************************************************************/
/* Returns bound type of a given point                                  */
/************************************************************************/
Bound Map::isInBounds(int x, int y)
{
	if(x == 0			&& y != 0 && y != MAP_HIGHT-1)	return LEFT_BOUND;
	if(x == MAP_WIDTH-1 && y != 0 && y != MAP_HIGHT-1)	return RIGHT_BOUND;
	if(y == 0			&& x != 0 && x != MAP_WIDTH-1)	return TOP_BOUND;
	if(y == MAP_HIGHT-1	&& x != 0 && x != MAP_WIDTH-1)	return BUTTOM_BOUND;

	if( (x == 0				&& y == 0) ||
		(x == 0				&& y == MAP_HIGHT-1) ||
		(x == MAP_WIDTH-1	&& y == 0) ||
		(x == MAP_WIDTH-1	&& y == MAP_HIGHT-1) )
		return BASE_ANGLE;

	return NOT_BOUND;
}