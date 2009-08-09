#include <iostream>
using namespace std;

#define MAP_WIDTH 240
#define MAP_HIGHT 315
#define OBSTACLE_WIDTH 52
#define OBSTACLE_HIGHT 54

#define MAX_ANGLES 1000

#define POINT_1_X 30
#define POINT_1_Y 30
#define POINT_2_X 210
#define POINT_2_Y 30
#define POINT_3_X 30
#define POINT_3_Y 285
#define POINT_4_X 210
#define POINT_4_Y 285

//Source and Target points
#define SRC_X POINT_1_X
#define SRC_Y POINT_1_Y
#define TGT_X POINT_4_X
#define TGT_Y POINT_4_Y

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

class Map {
public:
	int m[MAP_WIDTH][MAP_HIGHT];
	int angles[MAP_WIDTH][MAP_HIGHT];
	int changing[MAP_HIGHT];
	int stations[MAP_WIDTH][MAP_HIGHT];
	Map()
	{
		for(int i(0); i<MAP_WIDTH; i++)
			for(int j(0); j<MAP_HIGHT; j++)
			{
				m[i][j] = 0;
				angles[i][j] = 0;
				stations[i][j] = 0;
				changing[j] = 0;
			}

		m[SRC_X][SRC_Y] = 'S';
		m[TGT_X][TGT_Y] = 'T';

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
		decomposite();
		int currStation(1), j;
		stations[SRC_X][SRC_Y] = currStation++;
		cout << "station: " << SRC_X << " " << SRC_Y << endl;
		for(int i(0); i<MAP_HIGHT; i++)
		{
			if(changing[i] == 1)
			{
				j = getNextStation(i);
				stations[j][i] = currStation++;
				cout << "station: " << j << " " << i << endl;
			}
		}
		stations[TGT_X][TGT_Y] = currStation++;
		cout << "station: " << TGT_X << " " << TGT_Y << endl;
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
		findAngles();
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
					//cout << "point: " << i << " " << j << " is Angle!" << endl;
					angles[i][j] = 1;
				}
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

	virtual ~Map(){};
private:
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