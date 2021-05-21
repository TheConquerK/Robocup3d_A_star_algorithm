#include "naobehavior.h"

#include "../worldmodel/worldmodel.h"
#include "../worldmodel/WorldObject.h"

#include "../rvdraw/rvdraw.h"

#include "AStarBase.h"
extern int agentBodyType;

/*
 * Real game beaming
 * .
 * Filling params x y angle
 */
void NaoBehavior::beam( double& beamX, double& beamY, double& beamAngle ) {
    /*
    beamX = -HALF_FIELD_X + worldModel->getUNum();
    beamY = 0;
    beamAngle = 0;
    */
    int pm = worldModel->getPlayMode();
    VecPosition MyPosition;
    if( (worldModel->getSide() == SIDE_LEFT && pm == PM_GOAL_LEFT ) ||
        (worldModel->getSide() == SIDE_RIGHT && pm == PM_GOAL_RIGHT ) || 
        (worldModel->getGameTime() < 1.0 && pm == PM_BEFORE_KICK_OFF && worldModel->getSide() == SIDE_RIGHT) ) {
        if (worldModel->getUNum() == 1)
        {
            MyPosition = (VecPosition(-14.0f, 2.0f, 0.0f));        
        }
        else if (worldModel->getUNum() == 2)                       
        {
            MyPosition =(VecPosition(-2.0f,7.0f,0.0f));        
        }
        else if(worldModel->getUNum() == 3)
        {
            MyPosition =(VecPosition(-2.0f,-7.0f,0.0f)); 
        }
        else if(worldModel->getUNum() == 4)
        {
            MyPosition = (VecPosition(-4.0f,0.0f, 0.0f));
        }
        else if(worldModel->getUNum()==5)
        {
            MyPosition=(VecPosition(-7.0f,0.0f,0.0f));    
        }
        else if(worldModel->getUNum() ==6)
        {
            MyPosition=(VecPosition(-9.0f,-8.0f,0.0f)); 
        }
        else if(worldModel->getUNum()==7)
        {
            MyPosition=(VecPosition(-9.0f,-3.0f,0.0f));   
        }
        else if(worldModel->getUNum()==8)
        {
            MyPosition=(VecPosition(-9.0f,3.0f,-20.0f));
        }
        else if(worldModel->getUNum()==9)
        {
            MyPosition=(VecPosition(-9.0f,8.0f,0.0f));
        }
        else if(worldModel->getUNum()==10)
        {
            MyPosition=(VecPosition(-5.0f,-6.0f,0.0f));
        }
        else if(worldModel->getUNum()==11)
        {
            MyPosition=(VecPosition(-5.0f,6.0f,0.0f));
        }
    }
       
    else {
        if (worldModel->getUNum() == 1)
        {
            MyPosition = (VecPosition(-14.0f, 0.0f, 0.0f));        
        }
        else if (worldModel->getUNum() == 2)                        
        {
            MyPosition =(VecPosition(-1.0f,8.0f,0.0f));        
        }
        else if(worldModel->getUNum() == 3)
        {
            MyPosition =(VecPosition(-3.0f,4.0f,0.0f)); 
        }
        else if(worldModel->getUNum() == 4)
        {
            MyPosition = (VecPosition(-3.0f,-4.0f, 0.0f));
        }
        else if(worldModel->getUNum()==5)
        {
            MyPosition=(VecPosition(-1.0f,-8.0f,0.0f));    
        }
        else if(worldModel->getUNum() ==6)
        {
            MyPosition=(VecPosition(-7.0f,0.0f,0.0f)); 
        }
        else if(worldModel->getUNum()==7)
        {
            MyPosition=(VecPosition(-12.0f,6.5f,0.0f));   
        }
        else if(worldModel->getUNum()==8)
        {
            MyPosition=(VecPosition(-11.0f,-3.0f,0.0f));
        }
        else if(worldModel->getUNum()==9)
        {
            MyPosition=(VecPosition(-11.0f,1.0f,0.0f));
        }
        else if(worldModel->getUNum()==10)
        {
            MyPosition=(VecPosition(-7.0f,-6.0f,0.0f));
        }
        else if(worldModel->getUNum()==11)
        {
            MyPosition=(VecPosition(-7.0f,8.0f,0.0f));
        }
    }
    
    beamX = MyPosition.getX();
    beamY = MyPosition.getY();
    beamAngle = MyPosition.getZ();   
}


SkillType NaoBehavior::selectSkill() 
{
    //opponent Kalman_Filter_test
    if(worldModel->getUNum() == 1){
        worldModel->getRVSender()->clear();
        for(int i = WO_OPPONENT1; i < WO_OPPONENT1+NUM_AGENTS; ++i) {
            VecPosition oppPos = worldModel->getWorldObject(i)->pos;
            VecPosition ballball = worldModel->getBall();
            worldModel->getRVSender()->drawPoint("pball", oppPos.getX(),oppPos.getY(), 10, RVSender::YELLOW);

            worldModel->getRVSender()->drawPoint("pball", ballball.getX(),ballball.getY(), 10, RVSender::RED);
        }
    }
    //A*_test
    if(worldModel->getUNum() == 1)
    {
        ball = worldModel->getBall();
        NEUAStar aster;
        aster.SetMap(worldModel);
        aster.SetTarget(VecPosition(14.9,0));
        aster.SetStartPoint(ball);
        vector<VecPosition> track = aster.GetTrack();

        worldModel->getRVSender()->clear();
        if(!track.empty())
        {
            worldModel->getRVSender()->drawPoint("a*", track[0].getX(),track[0].getY(), 10, RVSender::BLUE);
            worldModel->getRVSender()->drawPoint("a*", track[4].getX(),track[4].getY(), 10, RVSender::BLUE);
            worldModel->getRVSender()->drawPoint("a*", track[8].getX(),track[8].getY(), 10, RVSender::BLUE);
            worldModel->getRVSender()->drawPoint("a*", track[12].getX(),track[12].getY(), 10, RVSender::BLUE);
            worldModel->getRVSender()->drawPoint("a*", track[16].getX(),track[16].getY(), 10, RVSender::BLUE);
            worldModel->getRVSender()->drawPoint("a*", track[20].getX(),track[20].getY(), 10, RVSender::BLUE);
        }

        if(!track.empty())
        {
            for(auto &p : track)
            {
                worldModel->getRVSender()->drawPoint("a*", p.getX(),p.getY(), 10, RVSender::ORANGE);
                                                    
            }
        }
    }
    return SKILL_STAND;
}

