/*
 * werker.cpp
 *
 *  Created on: Feb 28, 2023
 *      Author: Piotr Lesicki
 */

#include "Werker.hpp"

w::Werker(Werker ** werker_contiainer, WerkerType type) : type(type)
{
	this->werker_container = werker_contiainer;
}

void w::swapWerker(Werker * werker)
{
	*this->werker_container = werker;
}
