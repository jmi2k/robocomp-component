/*
 *    Copyright (C) 2021 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"
#include <cmath>
#include <Eigen3/Dense>


/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx, bool startup_check) : GenericWorker(mprx)
{
	this->startup_check_flag = startup_check;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
//	THE FOLLOWING IS JUST AN EXAMPLE
//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		std::string innermodel_path = par.value;
//		innerModel = std::make_shared(innermodel_path);
//	}
//	catch(const std::exception &e) { qFatal("Error reading config params"); }






	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	this->Period = period;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		timer.start(Period);
	}

}

//SIGMOIDE
static float sigmoid(float x, float param){
    return 2 / (exp(-x * param) + 1) - 1;
}

void SpecificWorker::compute( ){
    const float threshold = 200; // millimeters
    float rot = 0.6;  // rads per second
    static RoboCompRCISMousePicker::Pick target;
    static bool activo = false;
    const float sigma = -0.5 * 0.5 / log(0.3);

    try{
    	RoboCompGenericBase::TBaseState estadoLeido;
    	differentialrobot_proxy->getBaseState(estadoLeido);
        RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();

        if(auto t = target_buffer.try_get(); t.has_value()){
            target = t.value();
            activo = true;
        }

        float x_dist = target.x - estadoLeido.x;
        float z_dist = target.z - estadoLeido.z;
        float dist = sqrt(x_dist * x_dist + z_dist * z_dist);

        if(dist < 100){
            activo = false;
            differentialrobot_proxy->setSpeedBase(0, 0);
            return;
        }
#if 1
        if (activo){
            float alpha = estadoLeido.alpha;
            float theta = atan2(z_dist, x_dist);
            float beta = M_PI_2 - (alpha + theta);
            float wSpeed = sigmoid(beta, 1);
            float angular_reduction = exp( -(wSpeed * wSpeed) / sigma);
            float distance_reduction = sigmoid(dist, 1/800.);
            float vSpeed = 1000 * angular_reduction * distance_reduction;
            printf("distance_reduction = %f\n", distance_reduction);
            /*
            if (vSpeed > 1000) vSpeed = 1000;
            if(angular_reduction > 1 || angular_reduction < -1 || distance_reduction > 1 || distance_reduction < -1) {
                printf("distance_reduction = %f\n", distance_reduction);
                printf("angular_reduction = %f\n", angular_reduction);
            }
            printf("1000 * exp( -(%f * %f) / %f) * sigmoid(%f)      beta = %f\n", wSpeed, wSpeed, sigma, dist, beta);
            */
            //printf("%f  %f  %f  %f\n", beta, wSpeed, dist, vSpeed);
            differentialrobot_proxy->setSpeedBase(vSpeed, wSpeed);
        }
#else
        if (activo){
            printf("Entrando en activo!\n");
            float alpha = estadoLeido.alpha;
            float theta = atan2(z_dist, x_dist);
            float beta = M_PI_2 - (alpha + theta);
            float wSpeed = sigmoid(beta);
            differentialrobot_proxy->setSpeedBase(0, wSpeed);
        }else{
            printf("Pues no :(\n");
        }
#endif
    }catch(const Ice::Exception &ex){std::cout << ex << std::endl;}
}


int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

//SUBSCRIPTION to setPick method from RCISMousePicker interface
void SpecificWorker::RCISMousePicker_setPick(const RoboCompRCISMousePicker::Pick &myPick)
{
	qInfo() << myPick.x << myPick.z ;
	target_buffer.put(myPick);
}



/**************************************/
// From the RoboCompDifferentialRobot you can call this methods:
// this->differentialrobot_proxy->correctOdometer(...)
// this->differentialrobot_proxy->getBasePose(...)
// this->differentialrobot_proxy->getBaseState(...)
// this->differentialrobot_proxy->resetOdometer(...)
// this->differentialrobot_proxy->setOdometer(...)
// this->differentialrobot_proxy->setOdometerPose(...)
// this->differentialrobot_proxy->setSpeedBase(...)
// this->differentialrobot_proxy->stopBase(...)

/**************************************/
// From the RoboCompDifferentialRobot you can use this types:
// RoboCompDifferentialRobot::TMechParams

/**************************************/
// From the RoboCompLaser you can call this methods:
// this->laser_proxy->getLaserAndBStateData(...)
// this->laser_proxy->getLaserConfData(...)
// this->laser_proxy->getLaserData(...)

/**************************************/
// From the RoboCompLaser you can use this types:
// RoboCompLaser::LaserConfData
// RoboCompLaser::TData

