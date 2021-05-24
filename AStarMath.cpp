/*
 *@author: Kangkang
 *@from：NEU_Smart_Lab
 *@date: 2021-4-30
 *@description: A* algorithm used in Robocup3d
 */
#include <iostream>
#include<vector>
#include<algorithm>
#include "stdio.h"
#include "AStarBase.h"
using namespace std;

CAStarBase::CAStarBase()
{
	m_nAPointArrWidth = 0;
	m_nAPointArrHeight = 0;
	step = 0;
	expand_num = 0;
	plan_route_step = 0;
	m_pStartPoint = NULL;
	m_pEndPoint = NULL;
	m_pCurPoint = NULL;

}

CAStarBase::~CAStarBase()
{

}

bool CAStarBase::Create(char* pDateArr, int nWidth, int nHeight)
{
	if (!pDateArr) return false;
	if (nWidth < 1 || nHeight < 1) return false;

	m_pAPointArr.clear();
    for(int x=0;x<=nWidth * nHeight;x++)
    {
        APoint point;
        point.x = 0;
        point.y = 0;
        m_pAPointArr.push_back(point);
    }

    // delete m_pAPointArr;
    // m_pAPointArr = new APoint[nWidth * nHeight];
	// if (!m_pAPointArr) return false;

	m_pOldArr = pDateArr;
	m_nAPointArrWidth = nWidth;
	m_nAPointArrHeight = nHeight;

    expand_num=0;

	// 创建存储地图数组
	for (int y = 0; y < m_nAPointArrHeight; y++)
	{
		for (int x = 0; x < m_nAPointArrWidth; x++)
		{
			m_pAPointArr[y * m_nAPointArrWidth + x].x = x;
			m_pAPointArr[y * m_nAPointArrWidth + x].y = y;
			m_pAPointArr[y * m_nAPointArrWidth + x].g = 0;
			m_pAPointArr[y * m_nAPointArrWidth + x].f = 0;
			m_pAPointArr[y * m_nAPointArrWidth + x].h = 0;

			if (m_pOldArr[y * m_nAPointArrWidth + x] == '0')
			{
				// m_pAPointArr[y * m_nAPointArrWidth + x].type = APT_OPENED;
			}
			else if (m_pOldArr[y * m_nAPointArrWidth + x] == '1')
			{
				cout << "OBSTACLE :" << x << "," << y << endl;
				expand_x[expand_num] = x;
				expand_y[expand_num] = y;
				expand_num++;
				// m_pAPointArr[y * m_nAPointArrWidth + x].type = APT_CLOSED;
			}
			else if (m_pOldArr[y * m_nAPointArrWidth + x] == 'S')
			{
				cout << "Start point:" << x << " , " << y << endl;
				memory_data[0] = x;
				memory_data[1] = y;
				// m_pAPointArr[y * m_nAPointArrWidth + x].type = APT_STARTPOINT;
				// m_pStartPoint = m_pAPointArr + y * m_nAPointArrWidth + x;
				// m_pCurPoint = m_pStartPoint;
			}
			else if (m_pOldArr[y * m_nAPointArrWidth + x] == 'E')
			{
				memory_data[2] = x;
				memory_data[3] = y;
				cout << "End point:" << x << " , " << y << endl;
				// m_pAPointArr[y * m_nAPointArrWidth + x].type = APT_ENDPOINT;
				// m_pEndPoint = m_pAPointArr + y * m_nAPointArrWidth + x;
			}
			else {
				// m_pAPointArr[y * m_nAPointArrWidth + x].type = APT_UNKNOWN;
			}

		}
	}
	//	  cout<<expand_x<<endl;
	//    cout<<expand_y<<endl;
	//    cout<<expand_num<<endl;
	return true;
}
bool CAStarBase::Swelling()
{
	int Swelling_radius;
	int radius;
	int x, y, start_x, start_y, end_x, end_y;
	int k_swl = 1; //膨胀系数
	float distance;

	start_x = memory_data[0];
	start_y = memory_data[1]; 
	end_x = memory_data[2];
	end_y = memory_data[3];
    //expand_x[]
    //expand_y[]

	//distance = abs(start_x - end_x) + abs(start_y - end_y);
	//Swelling_radius = (int)k_swl * distance;//膨胀系数与距离计算

	Swelling_radius = 20;//膨胀半径1、 2、 3 、4、 5 *0.1m
	radius = Swelling_radius - 1;
	
    //障碍物膨胀
    // PrintCharArr();

	for (int m = 0; m < 11; m++)//i<expand_num
	{
		x = expand_x[m];
		y = expand_y[m];
        // cout<<expand_num<<" " << i;
		//cout << x << "," << y << endl;
		if (Swelling_radius == 1)
		{
			//不膨胀
		}
		else
		{

/*******************************矩形膨胀××××*********************************************/
			// for (int i = x - radius; i <= x + radius; i++)
			// {
			// 	for (int j = y - radius; j <= y + radius; j++)
			// 	{
            //         int a =j*m_nAPointArrWidth+i;
            //         if(a < 0) a =0;
            //         if(a >301*201) a = 301*201;

			// 		m_pOldArr[a] = '1';//膨胀点中心位置

			// 		if (i > (301 - 1))   
            //         {
            //             a =j*m_nAPointArrWidth+i;
            //             m_pOldArr[a] = '0';
            //         }
			// 		//膨胀点越界处理***只能直接传入数据m_nAPointArrWidth会出问题,传数据没问题***

            //         // if (j>(121 -1)) a=9600;//膨胀点越界处理//y轴方向不会出现溢出问题
					
			// 	}
			// }
/*********************************圆形膨胀************************************************/
			for (int i = x - radius; i <= x + radius; i++)
			{
				for (int j = y - radius; j <= y + radius; j++)
				{
                    int a =j*m_nAPointArrWidth+i;
                    if(a < 0) a =0;
                    if(a >301*201) a = 301*201;

                    int dis =(i-x)*(i-x)+(j-y)*(j-y);//圆形膨胀处理
                    if (dis<radius*radius)
                    {
                        if(a > 0 && a <= 301*201)  m_pOldArr[a] = '1';//膨胀点中心位置
                    }
					//cout<<m_nAPointArrWidth<<endl;//81
					//cout<<m_nAPointArrHeight<<endl;//121


					if (i > (301 - 1))   
                    {
                        a =j*m_nAPointArrWidth+i;
                        m_pOldArr[a] = '0';
                    }
					//膨胀点越界处理***只能直接传入数据m_nAPointArrWidth会出问题,传数据没问题***

                    // if (j>(121 -1)) a=9600;//膨胀点越界处理//y轴方向不会出现溢出问题
					
				}
			}
/*****************************菱形膨胀**************************************************/
//菱形分割
			// for (int i = x - radius; i <= x + radius; i++)
			// {
			// 	for (int j = y - radius; j <= y + radius; j++)
			// 	{
            //         int a =j*m_nAPointArrWidth+i;
            //         if(a < 0) a =0;
            //         if(a >301*201) a = 301*201;

			// 		m_pOldArr[a] = '1';//膨胀点中心位置

			// 		if (i > (301 - 1))   
            //         {
            //             a =j*m_nAPointArrWidth+i;
            //             m_pOldArr[a] = '0';
            //         }
			// 		//膨胀点越界处理***只能直接传入数据m_nAPointArrWidth会出问题,传数据没问题***

            //         // if (j>(121 -1)) a=9600;//膨胀点越界处理//y轴方向不会出现溢出问题
					
			// 	}
			// }
			// for (int i = x - radius; i <=x-1; i++)
			// {
			// 	for (int j = y - radius; j <=y-radius+ x-1-i; j++)
			// 	{
            //         int a =j*m_nAPointArrWidth+i;
            //         if(a < 1) a =1;
            //         if(a >9801) a = 9801;

            //         if(a >= 0 && a <= 9801)  m_pOldArr[a] = '0';
					
			// 		//cout <<"左上置零"<< i << "," << j << endl;
			// 	}
			// }
			// for (int i = x + 1; i <=x+radius ; i++)
			// {
			// 	for (int j = y -radius; j <= y -radius + i - x - 1; j++)
			// 	{
            //         int a =j*m_nAPointArrWidth+i;
            //         if(a < 1) a =1;
            //         if(a >9801) a = 9801;

			// 		if(a >= 0 && a <= 9801)  m_pOldArr[a] = '0';
			// 		//cout << "右上置零" << i << "," << j << endl;
			// 	}
			// }
			// for (int i = x - radius; i <= x-1; i++)
			// {
			// 	for (int j = y+radius; j >= y + radius + i-x+1; j--)
			// 	{
            //         int a =j*m_nAPointArrWidth+i;
            //         if(a < 1) a =1;
            //         if(a >9801) a = 9801;  

			// 		if(a >= 0 && a <= 9801)  m_pOldArr[a] = '0';
			// 		//cout << "左下置零" << i << "," << j << endl;
			// 	}
			// }
			// for (int i = x + 1; i <= x + radius; i++)
			// {
			// 	for (int j = y + radius; j >= y + radius - i + x + 1; j--)
			// 	{
            //         int a =j*m_nAPointArrWidth+i;
            //         if(a < 1) a =1;
            //         if(a >9801) a = 9801;

			// 		if(a >= 0 && a <= 9801)  m_pOldArr[a] = '0';
			// 		//cout << "右下置零" << i << "," << j << endl;
			// 	}
			// }
/*****************************××××××**************************************************/
		}
		// PrintCharArr();
	}
    //重置起点、终点
	m_pOldArr[start_y * m_nAPointArrWidth + start_x] = 'S';
	m_pOldArr[end_y * m_nAPointArrWidth + end_x] = 'E';
	
    cout<<"Swelling has be done."<<endl;
    
    PrintCharArr();

    return true;
}
bool CAStarBase::Create_Expanded_Map()
{
	// 初始化数组内容
	for (int y = 0; y < m_nAPointArrHeight; y++)
	{
		for (int x = 0; x < m_nAPointArrWidth; x++)
		{
			m_pAPointArr[y * m_nAPointArrWidth + x].x = x;
			m_pAPointArr[y * m_nAPointArrWidth + x].y = y;
			m_pAPointArr[y * m_nAPointArrWidth + x].g = 0;
			m_pAPointArr[y * m_nAPointArrWidth + x].f = 0;
			m_pAPointArr[y * m_nAPointArrWidth + x].h = 0;

			if (m_pOldArr[y * m_nAPointArrWidth + x] == '0')
			{
				m_pAPointArr[y * m_nAPointArrWidth + x].type = APT_OPENED;
			}
			else if (m_pOldArr[y * m_nAPointArrWidth + x] == '1')
			{
				//cout << "OBSTACLE :" << x << "," << y << endl;
				m_pAPointArr[y * m_nAPointArrWidth + x].type = APT_CLOSED;
			}
			else if (m_pOldArr[y * m_nAPointArrWidth + x] == 'S')
			{
				//cout << "Start point:" << x << " , " << y << endl;
				m_pAPointArr[y * m_nAPointArrWidth + x].type = APT_STARTPOINT;
				m_pStartPoint =&m_pAPointArr[y * m_nAPointArrWidth + x];
				m_pCurPoint = m_pStartPoint;
			}
			else if (m_pOldArr[y * m_nAPointArrWidth + x] == 'E')
			{
				//cout << "End point:" << x << " , " << y << endl;
				m_pAPointArr[y * m_nAPointArrWidth + x].type = APT_ENDPOINT;
				m_pEndPoint = &m_pAPointArr[y * m_nAPointArrWidth + x];
			}
			else {
				m_pAPointArr[y * m_nAPointArrWidth + x].type = APT_UNKNOWN;
			}

		}
	}
    cout<<"Expanded Map has be done."<<endl;

	return true;
}
void CAStarBase::SetStartPoint(int x, int y)
{
	if (m_pStartPoint && m_pAPointArr[y * m_nAPointArrWidth + x].type != APT_CLOSED)
	{
		m_pStartPoint->type = APT_OPENED;
		// 设置新的值
		m_pStartPoint = &m_pAPointArr[y * m_nAPointArrWidth + x];
		m_pStartPoint->type = APT_STARTPOINT;
		m_pCurPoint = m_pStartPoint;
	}
}

void CAStarBase::SetEndPoint(int x, int y)
{
	if (m_pStartPoint && m_pAPointArr[y * m_nAPointArrWidth + x].type != APT_CLOSED)
	{
		m_pStartPoint->type = APT_OPENED;
		// 设置新的值
		m_pStartPoint =&m_pAPointArr[y * m_nAPointArrWidth + x];// + y * m_nAPointArrWidth + x;
		m_pStartPoint->type = APT_ENDPOINT;
	}
}

void CAStarBase::SetCurrent(int x, int y)
{
	//	if ( m_pAPointArr[y*m_nAPointArrWidth+x].type==APT_OPENED )
	{
		m_pCurPoint = &m_pAPointArr[ y * m_nAPointArrWidth + x] ;
	}
}

void CAStarBase::SetOpened(int x, int y)
{
	if (m_pAPointArr[y * m_nAPointArrWidth + x].type != APT_OPENED)
	{
		m_pAPointArr[y * m_nAPointArrWidth + x].type = APT_OPENED;
	}
}

void CAStarBase::SetClosed(int x, int y)
{
	if (m_pAPointArr[y * m_nAPointArrWidth + x].type != APT_CLOSED)
	{
		m_pAPointArr[y * m_nAPointArrWidth + x].type = APT_CLOSED;
	}
}

void CAStarBase::PrintCharArr()
{
	// ++step;
	// cout << "ARR:" << step << endl;
	if (m_pOldArr)
	{
		for (int y = 0; y < m_nAPointArrHeight; y++)
		{
			for (int x = 0; x < m_nAPointArrWidth; x++)
			{
				printf("%c ", m_pOldArr[x + m_nAPointArrWidth * y]);
			}
			printf("\r\n");
		}
		printf("\r\n");
	}

}

PAPoint CAStarBase::CalcNextPoint(PAPoint ptCalc)
{
	if (ptCalc == NULL)
	{
		ptCalc = m_pStartPoint;
	}
	int x = ptCalc->x;
	int y = ptCalc->y;
	int dx = m_pEndPoint->x;
	int dy = m_pEndPoint->y;
	int sx = m_pStartPoint->x;
	int sy = m_pStartPoint->y;
	int xmin = x, ymin = y, vmin = 0; // 最优步骤的坐标和值
	int cost[8] ;//计算步骤代价

	// 判断是否已经到了最终的位置
	if ((x == dx && abs(y - dy) == 1) || (y == dy && abs(x - dx) == 1))
	{
		return m_pEndPoint;
	}
	
    //x y轴位置注意限制幅度

	// 右
	if (m_pAPointArr[(x + 1) + m_nAPointArrWidth * y].type == APT_OPENED && (x+1) < m_nAPointArrWidth )
	{
		m_pAPointArr[(x + 1) + m_nAPointArrWidth * y].g = 10;
			//10 * (abs(x + 1 - sx)) + (abs(y - sy));
		m_pAPointArr[(x + 1) + m_nAPointArrWidth * y].h =
			10 * (abs(x + 1 - dx) + abs(y - dy));
		m_pAPointArr[(x + 1) + m_nAPointArrWidth * y].f =
			m_pAPointArr[(x + 1) + m_nAPointArrWidth * y].g + m_pAPointArr[(x + 1) + m_nAPointArrWidth * y].h;
		cost[0] =m_pAPointArr[(x + 1) + m_nAPointArrWidth * (y)].f;

	}
	else {
		cost[0] = m_nAPointArrWidth * m_nAPointArrHeight;
	}
	// 上
	if (m_pAPointArr[(x + 0) + m_nAPointArrWidth * (y - 1)].type == APT_OPENED && (y-1) > 0)
	{
		m_pAPointArr[(x + 0) + m_nAPointArrWidth * (y - 1)].g = 10;
			//10 * (abs(x + 0 - sx)) + (abs(y - 1 - sy));
		m_pAPointArr[(x + 0) + m_nAPointArrWidth * (y - 1)].h =
			10 * (abs(x - dx) + abs(y - 1 - dy));
		m_pAPointArr[(x + 0) + m_nAPointArrWidth * (y - 1)].f =
			m_pAPointArr[(x + 0) + m_nAPointArrWidth * (y - 1)].g + m_pAPointArr[(x + 0) + m_nAPointArrWidth * (y - 1)].h;

		cost[1] = m_pAPointArr[(x + 0) + m_nAPointArrWidth * (y - 1)].f;
	}
	else {
		cost[1] = m_nAPointArrWidth * m_nAPointArrHeight;
	}
	// 下
	if (m_pAPointArr[(x + 0) + m_nAPointArrWidth * (y + 1)].type == APT_OPENED && (y+1) < m_nAPointArrHeight)
	{
		m_pAPointArr[(x + 0) + m_nAPointArrWidth * (y + 1)].g = 10;
			//10 * (abs(x + 0 - sx)) + (abs(y + 1 - sy));
		m_pAPointArr[(x + 0) + m_nAPointArrWidth * (y + 1)].h =
			10 * (abs(x - dx) + abs(y + 1 - dy));
		m_pAPointArr[(x + 0) + m_nAPointArrWidth * (y + 1)].f =
			m_pAPointArr[(x + 0) + m_nAPointArrWidth * (y + 1)].g + m_pAPointArr[(x + 0) + m_nAPointArrWidth * (y + 1)].h;

		cost[2] = m_pAPointArr[(x + 0) + m_nAPointArrWidth * (y + 1)].f;
	}
	else {
		cost[2] = m_nAPointArrWidth * m_nAPointArrHeight;
	}
	// 右上
	if (m_pAPointArr[(x + 1) + m_nAPointArrWidth * (y - 1)].type == APT_OPENED && (y-1) > 0 && (x+1) < m_nAPointArrWidth)
	{
		m_pAPointArr[(x + 1) + m_nAPointArrWidth * (y - 1)].g = 14;
			//10 * (abs(x + 1 - sx)) + (abs(y - 1 - sy));
		m_pAPointArr[(x + 1) + m_nAPointArrWidth * (y - 1)].h =
			10 * (abs(x + 1 - dx) + abs(y - 1 - dy));
		m_pAPointArr[(x + 1) + m_nAPointArrWidth * (y - 1)].f =
			m_pAPointArr[(x + 1) + m_nAPointArrWidth * (y - 1)].g + m_pAPointArr[(x + 1) + m_nAPointArrWidth * (y - 1)].h;

		cost[3] = m_pAPointArr[(x + 1) + m_nAPointArrWidth * (y - 1)].f;

	}
	else {
		cost[3] = m_nAPointArrWidth * m_nAPointArrHeight;
	}
	// 右下
	if (m_pAPointArr[(x + 1) + m_nAPointArrWidth * (y + 1)].type == APT_OPENED && (y+1) < m_nAPointArrHeight && (x+1) < m_nAPointArrWidth)
	{
		m_pAPointArr[(x + 1) + m_nAPointArrWidth * (y + 1)].g = 14;
			//10 * (abs(x + 1 - sx)) + (abs(y + 1 - sy));
		m_pAPointArr[(x + 1) + m_nAPointArrWidth * (y + 1)].h =
			10 * (abs(x + 1 - dx) + abs(y + 1 - dy));
		m_pAPointArr[(x + 1) + m_nAPointArrWidth * (y + 1)].f =
			m_pAPointArr[(x + 1) + m_nAPointArrWidth * (y + 1)].g + m_pAPointArr[(x + 1) + m_nAPointArrWidth * (y + 1)].h;

		cost[4] = m_pAPointArr[(x + 1) + m_nAPointArrWidth * (y + 1)].f;
	}
	else {
		cost[4] = m_nAPointArrWidth * m_nAPointArrHeight;
	}
	// 左下
	if (m_pAPointArr[(x - 1) + m_nAPointArrWidth * (y + 1)].type == APT_OPENED && (y+1) < m_nAPointArrHeight && (x-1) > 0)
	{
		m_pAPointArr[(x - 1) + m_nAPointArrWidth * (y + 1)].g = 14;
			//10 * (abs(x - 1 - sx)) + (abs(y + 1 - sy));
		m_pAPointArr[(x - 1) + m_nAPointArrWidth * (y + 1)].h =
			10 * (abs(x - 1 - dx) + abs(y + 1 - dy));
		m_pAPointArr[(x - 1) + m_nAPointArrWidth * (y + 1)].f =
			m_pAPointArr[(x - 1) + m_nAPointArrWidth * (y + 1)].g + m_pAPointArr[(x - 1) + m_nAPointArrWidth * (y + 1)].h;

		cost[5] = m_pAPointArr[(x - 1) + m_nAPointArrWidth * (y + 1)].f;
	}
	else {
		cost[5] = m_nAPointArrWidth * m_nAPointArrHeight;
	}
    // 左上
	if (m_pAPointArr[(x - 1) + m_nAPointArrWidth * (y - 1)].type == APT_OPENED && (y-1) > 0 && (x-1) > 0)
	{
		m_pAPointArr[(x - 1) + m_nAPointArrWidth * (y - 1)].g = 14;
			//10 * (abs(x - 1 - sx)) + (abs(y - 1 - sy));
		m_pAPointArr[(x - 1) + m_nAPointArrWidth * (y - 1)].h =
			10 * (abs(x - 1 - dx) + abs(y - 1 - dy));
		m_pAPointArr[(x - 1) + m_nAPointArrWidth * (y - 1)].f =
			m_pAPointArr[(x - 1) + m_nAPointArrWidth * (y - 1)].g + m_pAPointArr[(x - 1) + m_nAPointArrWidth * (y - 1)].h;

		cost[6] = m_pAPointArr[(x - 1) + m_nAPointArrWidth * (y - 1)].f;
	}
	else {
		cost[6] = m_nAPointArrWidth * m_nAPointArrHeight;
	}
	// 左
	if (m_pAPointArr[(x - 1) + m_nAPointArrWidth * y].type == APT_OPENED && (x-1) > 0)
	{
		m_pAPointArr[(x - 1) + m_nAPointArrWidth * y].g = 10;
			//10 * (abs(x - 1 - sx)) + (abs(y - sy));
		m_pAPointArr[(x - 1) + m_nAPointArrWidth * y].h =
			10 * (abs(x - 1 - dx) + abs(y - dy));
		m_pAPointArr[(x - 1) + m_nAPointArrWidth * y].f =
			m_pAPointArr[(x - 1) + m_nAPointArrWidth * y].g + m_pAPointArr[(x - 1) + m_nAPointArrWidth * y].h;

		cost[7] = m_pAPointArr[(x - 1) + m_nAPointArrWidth * (y + 0)].f;
	}
	else {
		cost[7] = m_nAPointArrWidth* m_nAPointArrHeight;
	}

	// for (int i=0;i<=7;i++)
	// 	cout << cost[i]<<endl;
	int vmin_location;
	int arr_length = sizeof(cost) / sizeof(cost[0]);  // 数组长度

	 // max_element(arr, arr+arr_length) 计算出来是一个地址，我们需要取该地址的值
	// cout << "max val is: " << *max_element(cost, cost + arr_length) << "\t the max val index is " << max_element(cost, cost + arr_length) - cost << endl;
	// cout << "min val is: " << *min_element(cost, cost + arr_length) << "\t the min val index is " << min_element(cost, cost + arr_length) - cost << endl;
	//cout << "Min element location: " << distance(value, min_element(value, value + arr_length)) <<endl;

	vmin_location = min_element(cost, cost + arr_length) - cost;
	vmin = *min_element(cost, cost + arr_length);

	// cout << "vmin_location:" << vmin_location << endl;
	// cout << "vmin:" << vmin << endl;
		switch (vmin_location) 
		{
			case 0:xmin = x + 1; ymin = y + 0; break;//右
			case 1:xmin = x + 0; ymin = y - 1; break;//上
			case 2:xmin = x + 0; ymin = y + 1; break;//下
			case 3:xmin = x + 1; ymin = y - 1; break;//右上
			case 4:xmin = x + 1; ymin = y + 1; break;//右下
			case 5:xmin = x - 1; ymin = y + 1; break;//左下
			case 6:xmin = x - 1; ymin = y - 1; break;//左上
			case 7:xmin = x - 1; ymin = y - 0; break;//左
			default:  break;
		}
    if(step>400)//限制步数长度
    {
        step = 0;
        plan_route_step=0;
        vmin=0;
        // delete[] m_pAPointArr;
    }
	// 如果有最优点则迭代，则否就返回NULL
	if (vmin)
	{
		//cout<<vmin<<endl;
		SetCurrent(xmin, ymin);
		SetClosed(xmin, ymin);
		*(m_pOldArr + xmin + m_nAPointArrWidth * ymin) = '*';
		// cout << xmin <<','<< ymin << endl;
		plan_route_i[plan_route_step] = ymin;
		plan_route_j[plan_route_step] = xmin;

		//PrintCharArr();
		plan_route_step++;
        step++;
        // cout<<"step"<<step<<endl;
        // cout<<"plan_route_step:"<<plan_route_step<<endl;
		PAPoint pApoint = CalcNextPoint(m_pCurPoint);
		if (pApoint == NULL)
		{
			SetCurrent(x, y);
			SetClosed(xmin, ymin);
			// *(m_pOldArr + xmin + m_nAPointArrWidth * ymin) = '0';
			//return CalcNextPoint(m_pCurPoint);
		}
		return pApoint;
	}
	else 
    {
		return NULL;
	}

}

void NEUAStar::SetMap(WorldModel *worldModel) {
    for (int i = 0; i <= 200; i++)
    {
        for (int j = 0; j <= 300; j++)
        {
            astar_base_.pBuff[i][j] = '0';
        }
    }
    vector<int> temp_pos;
    for(int i = WO_OPPONENT1; i < WO_OPPONENT1+NUM_AGENTS; ++i) {
        if (!worldModel->getWorldObject(i)->validPosition){
            continue;
        }
        VecPosition oppPos = worldModel->getWorldObject(i)->pos;
        temp_pos = Pos2Vec(oppPos);
        
        if(temp_pos[0]< 0 || temp_pos[0]>300 ||
            temp_pos[1]<0 || temp_pos[1]>200){
            continue;;
        }
        astar_base_.pBuff[temp_pos[1]][temp_pos[0]] = '1';
    }
}

void NEUAStar::SetTarget(VecPosition target) {
    if(target.getX() <= (-15+0.25) || target.getX() > (15-0.25) ||
        target.getY()<(-10+0.25) || target.getY() >=(10-0.25))
    {
        t_valid = false;
        return;
    }
    vector<int> temp_pos = Pos2Vec(target);
    astar_base_.pBuff[temp_pos[1]][temp_pos[0]] = 'E';
    t_valid = true;
}

vector<int> NEUAStar::Pos2Vec(VecPosition &pos) {
    vector<int> a_pos;
    a_pos.push_back(10.f*pos.getX()+150);
    a_pos.push_back(-10.f*pos.getY()+100) ;
    return a_pos;
}

void NEUAStar::SetStartPoint(VecPosition pos) {
    if(pos.getX() <= (-15+0.25) || pos.getX() > (15-0.25) ||
        pos.getY()<(-10+0.25) || pos.getY() >=(10-0.25))
    {
        t_valid = false;
        return;
    }
    vector<int> temp_pos = Pos2Vec(pos);
    astar_base_.pBuff[temp_pos[1]][temp_pos[0]] = 'S';
    s_valid = true;
}

vector<VecPosition> NEUAStar::GetTrack() {
    if(!s_valid || !t_valid){
        return vector<VecPosition>();
    }
    astar_base_.Create(&astar_base_.pBuff[0][0], 301, 201);
    astar_base_.Swelling();
    astar_base_.Create_Expanded_Map();
    //astar_base_.PrintCharArr();
    PAPoint pPoint;
    pPoint = astar_base_.CalcNextPoint(NULL);
    vector<VecPosition> out;

    if (pPoint == NULL)
    {
        printf("no path can arrive!\r\n");
        return out;
    }
    else
    {
        printf("success arrived!\r\n");
    }

    for (int i = 0; i < astar_base_.plan_route_step; i++)
    {
        out.push_back(Vec2Pos(astar_base_.plan_route_j[i],astar_base_.plan_route_i[i]));
		// cout<<"("<<astar_base_.plan_route_j[i]<<","<<astar_base_.plan_route_i[i]<<")";
    }
	cout << endl;
    return out;
}

VecPosition NEUAStar::Vec2Pos(int x, int y) {
    return VecPosition((x-150)/10.f,(y-100)/-10.f);
}
