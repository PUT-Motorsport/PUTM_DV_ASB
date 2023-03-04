/*
 * otherWerkers.cpp
 *
 *  Created on: Mar 1, 2023
 *      Author: pile
 */

#include "werker.hpp"

ew::ErrorWerker(Werker ** werker_container) : w(werker_container, WerkerType::ErrorWerker) { }

void ew::run()
{
	//hehe
}

sw::StopWerker(Werker ** werker_container) : w(werker_container, WerkerType::StopWerker) { }

void sw::run()
{
	//hehe
}

tw::TestWerker(Werker ** werker_container) : w(werker_container, WerkerType::TestWerker) { }

tw::TestWerker(Werker ** werker_container, TestWerker * test_werker) : w(werker_container, WerkerType::TestWerker), ptr(test_werker) { }

void tw::run()
{
	this->fun();
}
