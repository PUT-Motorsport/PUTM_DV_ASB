/*
 * initialCheckUpWerker.cpp
 *
 *  Created on: Feb 28, 2023
 *      Author: pile
 */

#include "Werker.hpp"

icw::InitialCheckUpWerker(Werker ** werker_container, ErrorWerker* error_werker, ContinuousMonitoringWerker * continuous_monitoring_werker) : w(werker_container, WerkerType::InitialCheckUpWerker)
{
	this->error_werker = error_werker;
	this->continuous_monitoring_werker = continuous_monitoring_werker;
}

void icw::run()
{

}
