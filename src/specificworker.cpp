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

Eigen::Vector2f SpecificWorker::transform_world_to_robot(
        Eigen::Vector2f target_in_world,
        float robot_angle,
        Eigen::Vector2f robot_in_world) {
    Eigen::Matrix2f rot;
    rot << cos(robot_angle), -sin(robot_angle),
           sin(robot_angle),  cos(robot_angle);
    auto target_in_robot = rot * (target_in_world - robot_in_world);
    return target_in_robot;
}

void SpecificWorker::compute() {
    static RoboCompRCISMousePicker::Pick target;
    static bool active    = false;
    const float threshold = 200;
    const float sigma     = -0.5 * 0.5 / log(0.3);

    try {
    	RoboCompGenericBase::TBaseState base_state;
        Eigen::Vector2f target_in_world, robot_in_world, v_dist;
        float alpha, dist;

        alpha = base_state.alpha;
    	differentialrobot_proxy->getBaseState(base_state);

        if(auto t = target_buffer.try_get(); t.has_value()) {
            target = t.value();
            active = true;
        }

        target_in_world = Eigen::Vector2f(target.x, target.z);
        robot_in_world  = Eigen::Vector2f(base_state.x, base_state.z);
        v_dist          = transform_world_to_robot(target_in_world, alpha, robot_in_world);
        dist            = v_dist.norm();

        if (dist < threshold) {
            active = false;
            differentialrobot_proxy->setSpeedBase(0, 0);
            return;
        }
        if (active) {
            float beta = atan2(v_dist.x(), v_dist.y());
            cout << "beta:" << beta << endl;
            float wSpeed = sigmoid(beta, 2);

            float angular_reduction = exp( -(wSpeed * wSpeed) / sigma);
            float distance_reduction = sigmoid(dist, 1/80.);
            float vSpeed = 1000 * angular_reduction * distance_reduction;
            differentialrobot_proxy->setSpeedBase(vSpeed, wSpeed);
        }
    } catch (const Ice::Exception &ex) {
        std::cout << ex << std::endl;
    }
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

