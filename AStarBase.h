/*
 *@author: Kangkang
 *@from：NEU_Smart_Lab
 *@date: 2021-4-30
 *@description: A* algorithm used in Robocup3d
 */

/*
	A star 算法的基础处理
*/
#ifndef _A_STAR_BASE_H_
#define _A_STAR_BASE_H_
//#include "windows.h"
#include "../worldmodel/worldmodel.h"
#include<vector>
#include<algorithm>

typedef struct _APoint {
	int x; // x 坐标
	int y; // y 坐标
	int type; // 类型
	int f; // f = g + h
	int g;
	int h;
} APoint, * PAPoint;

enum APointType {
	APT_UNKNOWN, // 未知状态
	APT_OPENED, // 开放列表中
	APT_CLOSED, // 关闭列表中
	APT_STARTPOINT, // 起始点
	APT_ENDPOINT // 结束点
};


class CAStarBase {
public:
	CAStarBase();
	~CAStarBase();
private:
	// PAPoint m_pAPointArr;
    std::vector<APoint> m_pAPointArr;
	int m_nAPointArrWidth;
	int m_nAPointArrHeight;
	
	int expand_num;
	int expand_x[99];
	int expand_y[99];
	int memory_data[20];
	PAPoint m_pStartPoint, m_pEndPoint, m_pCurPoint;
	char* m_pOldArr;

public:
	bool Create(char* pDateArr, int nWidth, int nHeight);
    bool Swelling();
    bool Create_Expanded_Map();
	void SetStartPoint(int x, int y);
	void SetEndPoint(int x, int y);
	void SetOpened(int x, int y);
	void SetClosed(int x, int y);
	void SetCurrent(int x, int y);
	void PrintCharArr();
	char pBuff[201][301];
    int step;
	int plan_route_step;
	int plan_route_i[1000];
	int plan_route_j[1000];
	PAPoint CalcNextPoint(PAPoint ptCalc); // 应用迭代的办法进行查询
};

class NEUAStar{
public:
    NEUAStar(){astar_base_=CAStarBase();};

    void SetMap(WorldModel *worldmodel);
    void SetTarget(VecPosition target);
    void SetStartPoint(VecPosition pos);

    vector<VecPosition> GetTrack();

private:
    vector<int> Pos2Vec(VecPosition &pos);
    VecPosition Vec2Pos(int x,int y);
    CAStarBase astar_base_;
    bool s_valid= false;
    bool t_valid = false;
};

#endif

