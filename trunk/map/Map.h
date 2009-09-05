#include <iostream>
using namespace std;

/************************************************************************/
/* Map Definitions                                                      */
/************************************************************************/
#define MAP_WIDTH 240
#define MAP_HIGHT 315
#define OBSTACLE_WIDTH 52
#define OBSTACLE_HIGHT 54

/************************************************************************/
/* Points definitions                                                   */
/************************************************************************/
#define POINT_1_X 30
#define POINT_1_Y 30
#define POINT_2_X 210
#define POINT_2_Y 30
#define POINT_3_X 30
#define POINT_3_Y 285
#define POINT_4_X 210
#define POINT_4_Y 285

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

/************************************************************************/
/* Obstacles' positions                                                 */
/************************************************************************/
#define OBS_1_X 0
#define OBS_1_Y 131
#define OBS_2_X 136
#define OBS_2_Y 131
#define OBS_3_X 188
#define OBS_3_Y 131
#define OBS_4_X 188
#define OBS_4_Y 185

/************************************************************************/
/* General Map Enums                                                    */
/************************************************************************/
typedef enum {NOT_BOUND, LEFT_BOUND, RIGHT_BOUND, TOP_BOUND, BUTTOM_BOUND, BASE_ANGLE} Bound;
typedef enum {NORTH /*0*/, SOUTH /*1*/, EAST /*2*/, WEST /*3*/} Orientation;

/************************************************************************/
/* Map Class                                                            */
/************************************************************************/
class Map {
public:
	Map(int src_x, int src_y, int tgt_x, int tgt_y);
	virtual ~Map();
	bool	getNextStation(int &x, int &y);
	int		getCurrStation();
	int		getDistance(int x, int y);
	void	getDistances(int x, int y, Orientation o, int &northDist, int &southDist, int &eastDist , int &westDist);
	void	print();
	void	printAngles();
	void	printStations();

private:
	int m[MAP_WIDTH][MAP_HIGHT];
	int angles[MAP_WIDTH][MAP_HIGHT];
	int changing[MAP_HIGHT];
	int stations[MAP_WIDTH][MAP_HIGHT];
	int currStation;

	int src_x;
	int src_y;
	int tgt_x;
	int tgt_y;

	void	init();
	void	setStations();
	int		getNextStation(int i);
	void	decomposite();
	bool	isAngleInRow(int row);
	void	findAngles(); 
	bool	isAngle(int x, int y);
	bool	isSartEndPoint(int x, int y);
	void	printMatrix(int M[MAP_WIDTH][MAP_HIGHT]);
	Bound	isInBounds(int x, int y);
};