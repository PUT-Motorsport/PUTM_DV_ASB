/*
 * continuousMonitoringWerker.cpp
 *
 *  Created on: Mar 1, 2023
 *      Author: pile
 */

#include "werker.hpp"

cmw::ContinuousMonitoringWerker(Werker ** werker_container, ErrorWerker * error_werker, StopWerker * stop_werker) : w(werker_container, WerkerType::ContinuousMonitoringWerker)
{
	this->error_werker = error_werker;
	this->stop_werker = stop_werker;
}

void cmw::run()
{

}
