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
	setPath();
	/*After the C'tor is called, the matrix is decomposited and the matrix
	  is divided by cells, each cell has a number, and stations which the 
	  robot should follow is filled in stations matrix (can be taken calling
	  getNextStation(x, y) function.										*/
}

/************************************************************************/
/* Returns current station point                                        */
/************************************************************************/
StationType Map::getNextStation(int &x, int &y, bool fill)
{
	/*Keep returning the station position till there is no stations*/
	currStation++;
	for(int i(0); i<MAP_WIDTH; i++)
		for(int j(0); j<MAP_HIGHT; j++)
			if(stations[i][j] == currStation)
			{
				x=i;
				y=j;
/*
				if(currStation > 1 && fill)
				{
					int prevX, prevY;
					getStationByNum(currStation-1, prevX, prevY);
					if(x == prevX)
						fillYaxis(prevY, y, x, RED);
					else if(y == prevY)
						fillXaxis(prevX, x, y, RED);
					else
						throw "Exception in getNextStaion: still KOO3";
				}
*/
				if(currStation == numOfStations-1)
					return LAST;
				else if(currStation == numOfStations-2)
					return BEFORE_LAST;
				else if(currStation == numOfStations-3)
					return BEFORE_BEFORE_LAST;
				else if(currStation == 1)
					return FIRST;
				else
					return MIDDLE;
			}
			x=-1;
			y=-1;
			return NOT_STATION;
}

/************************************************************************/
/* get Station coordinates                                              */
/************************************************************************/
void Map::getStationByNum(int stNum, int &x, int &y)
{
	for(int i(0); i<MAP_WIDTH; i++)
		for(int j(0); j<MAP_HIGHT; j++)
			if(stations[i][j] == stNum)
			{
				x=i; y=j;
				return;
			}
	x=-1; y=-1;
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
	int dist1=MAP_HIGHT-y, dist2=y, dist3=x, dist4=MAP_WIDTH-x;

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
	cout << "======== Stations ========" << endl;
	for(int j(0); j<MAP_HIGHT; j++)
		for(int i(0); i<MAP_WIDTH; i++)
			if(stations[i][j] != 0)
				cout << "stations[" << i << "]" << "[" << j << "]" << "=" << stations[i][j] << endl;
	cout << "==========================" << endl;
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
	int firstCell = m[src_x][src_y];
	int lastCell  = m[tgt_x][tgt_y];
	int mode;	// 1 = move up
				// -1 = move down
				// 0 = both in same cell

	int currXst, currYst, nextXst, nextYst;

	if(firstCell < lastCell)
		mode = 1;
	else if (firstCell > lastCell)
		mode = -1;
	else
		mode = 0;

	int currSt(1), j;

	currXst = src_x;
	currYst = src_y;
	stations[currXst][currYst] = currSt++;

	if(mode == 0)
	{
		stations[tgt_x][tgt_y] = currSt;
		numOfStations = --currSt;
	}
	else
	{
		for(int i(mode==1 ? 0 : MAP_HIGHT-1); i<MAP_HIGHT && i>=0; i+=mode)
		{
			if(changing[i] == 1)
			{
				j = getNextStation(i);
				//station[j][i] is next station...
				nextXst = j;
				nextYst = i;
				
				if( (currXst == nextXst) || (currYst == nextYst) )
				{
					stations[nextXst][nextYst] = currSt++;
				}
				else
				{
					int res = choosePath(currXst, currYst, nextXst, nextYst, currSt);

					if(res == 1) /* _| */
					{
						stations[nextXst][currYst] = currSt++;
						stations[nextXst][nextYst] = currSt++;
					}
					else /* |_ */
					{
						stations[currXst][nextYst] = currSt++;
						stations[nextXst][nextYst] = currSt++;
					}
				}
				currXst = nextXst;
				currYst = nextYst;
			}
		}
		nextXst = tgt_x;
		nextYst = tgt_y;
		if( (currXst == nextXst) || (currYst == nextYst) )
		{
			stations[nextXst][nextYst] = currSt++;
		}
		else
		{
			int res = choosePath(currXst, currYst, nextXst, nextYst, currSt);
			if(res == 1) /* _| */
			{
				stations[nextXst][currYst] = currSt++;
				stations[nextXst][nextYst] = currSt++;
			}
			else /* |_ */
			{
				stations[currXst][nextYst] = currSt++;
				stations[nextXst][nextYst] = currSt++;
			}
		}
		numOfStations = --currSt;
		printStations();
		return;
	}
}

/************************************************************************/
/* determine path to take                                               */
/************************************************************************/
int Map::choosePath(int x1, int y1, int x2, int y2, int currSt)
{
	int distXaxis = getDistance(x2, y1),
		distYaxis = getDistance(x1, y2);

	if(distXaxis > distYaxis)
		return 1; // x then y
	else if(distXaxis < distYaxis)
		return 0; // y then x
	else //same distance => choose under the current orientation condition
	{
		if(currSt == 2)
		{
			return choosPathByFirstOrientation();
		}
		else
		{
			int temp_x1, temp_y1, temp_x2, temp_y2;
			temp_x1 = x1;
			temp_y1 = y1;
			getStationByNum(currSt-1, temp_x2, temp_y2);

			if(temp_x1 == temp_x2)
				return 0;
			else if(temp_y1 == temp_y2)
				return 1;
		}
	}
	throw "Exception in choose path";
}

int	Map::choosPathByFirstOrientation()
{
	return 1;
	/*
	if(src_x == POINT_1_X || src_x == POINT_3_X)
		return 1;
	else if(src_x == POINT_2_X || src_x == POINT_4_X)
		return 0;
	else
		throw "Exception in choosing path...";
	*/
}

/************************************************************************/
/* Set Path                                                             */
/************************************************************************/
void Map::setPath()
{
	int curr_x, curr_y, next_x, next_y;
	getNextStation(curr_x, curr_y);
	while(getNextStation(next_x, next_y, false) != NOT_STATION)
	{
		if(curr_x == next_x)
			fillYaxis(curr_y, next_y, curr_x, GREEN);
		else if(curr_y == next_y)
			fillXaxis(curr_x, next_x, curr_y, GREEN);
		else
			throw "Exception in setting path : path with Koo3";
		curr_x = next_x;
		curr_y = next_y;
	}
	currStation = 0;
}

void Map::fillYaxis(int y1, int y2, int x, char c)
{
	int start = min(y1, y2);
	int end = max(y1, y2);
	for(int j(start); j<=end; j++)
		m[x][j] = c;
}

void Map::fillXaxis(int x1, int x2, int y, char c)
{
	int start = min(x1, x2);
	int end = max(x1, x2);
	for(int i(start); i<=end; i++)
		m[i][y] = c;
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
	if( ( (x == src_x) && (y == src_y) ) ||
		( (x == tgt_x) && (y == tgt_y) ) )
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

/************************************************************************/
/*                                                                      */
/************************************************************************/
bool Map::updatePath(int newX, int newY)
{
	int closestX, closestY, closestStation;
	if(getClosestPoint(newX, newY, closestX, closestY) == false)
		return false;
	closestStation = getClosestStation(closestX, closestY);
	clearStations(closestStation-1);
	stations[closestX][closestY] = closestStation-1;
	currStation = closestStation-2;
	printStations();
	return true;
}

/************************************************************************/
/*                                                                      */
/************************************************************************/
bool Map::getClosestPoint(int x, int y, int &closestX, int &closestY)
{
	for(int i(x); i>=0; i--)
	{
		if(m[i][y] == -1)
			break;
		if(m[i][y] == GREEN)
		{
			closestX = i;
			closestY = y;
			return true;
		}
	}
	for(int i(x); i<MAP_WIDTH; i++)
	{
		if(m[i][y] == -1)
			break;
		if(m[i][y] == GREEN)
		{
			closestX = i;
			closestY = y;
			return true;
		}
	}
	for(int j(y); j>=0; j--)
	{
		if(m[x][j] == -1)
			break;
		if(m[x][j] == GREEN)
		{
			closestX = x;
			closestY = j;
			return true;
		}
	}
	for(int j(y); j<MAP_HIGHT; j++)
	{
		if(m[x][j] == -1)
			break;
		if(m[x][j] == GREEN)
		{
			closestX = x;
			closestY = j;
			return true;
		}
	}
	return false;
}

/************************************************************************/
/*                                                                      */
/************************************************************************/
void Map::clearStations(int to)
{
	for(int i(0); i<MAP_WIDTH; i++)
		for(int j(0); j<MAP_HIGHT; j++)
		{
			if(stations[i][j] <= to)
				stations[i][j] = 0;
		}
}

/************************************************************************/
/*                                                                      */
/************************************************************************/
int Map::getClosestStation(int x, int y)
{
	int minStationDist;
	for(int stNum(currStation); stNum<=numOfStations; stNum++)
	{
		int stX, stY;
		double minDist = calculateDistance(0,0, MAP_WIDTH, MAP_HIGHT);
		getStationByNum(stNum, stX, stY);
		if(stX == -1 || stY == -1)
		{
			throw "Bad Station: in getting closest station...";
		}
		if(minDist > calculateDistance(x, y, stX, stY))
		{
			minDist = calculateDistance(x, y, stX, stY);
			minStationDist = stNum;
		}
	}
	return minStationDist;
}

/************************************************************************/
/*                                                                      */
/************************************************************************/
double Map::calculateDistance(int x1, int y1, int x2, int y2)
{
	double xDiff = max(x1, x2) - min(x1, x2);
	double yDiff = max(y1, y2) - min(y1, y2); 
	return sqrt	( xDiff*xDiff + yDiff*yDiff);
}