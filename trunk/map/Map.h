#include <iostream>
using namespace std;

#define MAP_WIDTH 240
#define MAP_HIGHT 315
#define OBSTACLE_WIDTH 52
#define OBSTACLE_HIGHT 54

#define MAX_ANGLES 1000

/*
---------------------------
|							|
|	(1)				  (2)	|
|							|
|							|
|							|
|							|
|							|
|	(3)				  (4)	|
|							|
---------------------------
*/
#define POINT_1_X 30
#define POINT_1_Y 30
#define POINT_2_X 210
#define POINT_2_Y 30
#define POINT_3_X 30
#define POINT_3_Y 285
#define POINT_4_X 210
#define POINT_4_Y 285

//Obstacles' positions
#define OBS_1_X 0
#define OBS_1_Y 131
#define OBS_2_X 136
#define OBS_2_Y 131
#define OBS_3_X 188
#define OBS_3_Y 131
#define OBS_4_X 188
#define OBS_4_Y 185

typedef enum {NOT_BOUND, LEFT_BOUND, RIGHT_BOUND, TOP_BOUND, BUTTOM_BOUND, BASE_ANGLE} Bound;
typedef enum {NORTH /*0*/, SOUTH /*1*/, EAST /*2*/, WEST /*3*/} Orientation;

class Map {
public:
	Map(int src_x, int src_y, int tgt_x, int tgt_y)
	{
		this->src_x = src_x;
		this->src_y = src_y;
		this->tgt_x = tgt_x;
		this->tgt_y = tgt_y;
		init();
		findAngles();
		decomposite();
		setStations();
	}

	bool getNextStation(int &x, int &y)
	{
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

	int getCurrStation()
	{
		return currStation;
	}

	int getDistance(int x, int y)
	{
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

	void getDistances(int x, int y, Orientation o,
					  int &northDist, int &southDist,
					  int &eastDist , int &westDist)
	{
		/*
		 ---------------------------
		|							|
		|							|
		|			S(2)			|
		|							|
		|	  E(3)		  W(4)		|
		|							|
		|			N(1)			|
		|							|
		|							|
		 ---------------------------
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

	void getIntersectionPoints(int x  , int y, 
							   int &x1, int &y1, 
							   int &x2, int &y2)
	{
		
	}

	void print()
	{
		printMatrix(m);
	}

	void printAngles()
	{
		printMatrix(angles);
	}

	void printStations()
	{
		printMatrix(stations);
	}

	int src_x;
	int src_y;
	int tgt_x;
	int tgt_y;

	virtual ~Map(){};

private:
	int m[MAP_WIDTH][MAP_HIGHT];
	int angles[MAP_WIDTH][MAP_HIGHT];
	int changing[MAP_HIGHT];
	int stations[MAP_WIDTH][MAP_HIGHT];
	int currStation;
	int src;

	void init()
	{
		currStation = 0;
		for(int i(0); i<MAP_WIDTH; i++)
			for(int j(0); j<MAP_HIGHT; j++)
			{
				m[i][j] = 0;
				angles[i][j] = 0;
				stations[i][j] = 0;
				changing[j] = 0;
			}

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

	void setStations()
	{	
		int currStation(1), j;
		stations[src_x][src_y] = currStation++;
		//cout << "station: " << src_x << " " << src_y << endl;
		for(int i(0); i<MAP_HIGHT; i++)
		{
			if(changing[i] == 1)
			{
				j = getNextStation(i);
				stations[j][i] = currStation++;
				//cout << "station: " << j << " " << i << endl;
			}
		}
		stations[tgt_x][tgt_y] = currStation++;
		//cout << "station: " << TGT_X << " " << TGT_Y << endl;
	}

	int getNextStation(int i)
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

	void decomposite()
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

	bool isAngleInRow(int row)
	{
		for(int i(0); i<MAP_WIDTH; i++)
			if(angles[i][row] == 1) return true;
		return false;
	}

	void findAngles()
	{
		for(int i(0); i<MAP_WIDTH; i++)
			for(int j(0); j<MAP_HIGHT; j++)
				if(isAngle(i,j))
				{
					angles[i][j] = 1;
				}
	}

	bool isAngle(int x, int y)
	{
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

	bool isSartEndPoint(int x, int y)
	{
		if( (m[x][y] == 'S') || (m[x][y] == 'T') )
			return true;
		else
			return false;
	}

	void printMatrix(int M[MAP_WIDTH][MAP_HIGHT])
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

	Bound isInBounds(int x, int y)
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
};