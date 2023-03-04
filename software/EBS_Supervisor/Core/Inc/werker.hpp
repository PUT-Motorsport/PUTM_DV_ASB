/*
 * werker.hpp
 *
 *  Created on: Feb 28, 2023
 *      Author: Piotr Lesicki
 */

#include "timer.hpp"

#include <functional>

#ifndef INC_WERKER_HPP_
#define INC_WERKER_HPP_

class Werker;
class InitialCheckUpWerker;
class ContinuousMonitoringWerker;
class ErrorWerker;
class StopWerker;
class TestWerker;

using w = Werker;
using icw = InitialCheckUpWerker;
using cmw = ContinuousMonitoringWerker;
using ew = ErrorWerker;
using sw = StopWerker;
using tw = TestWerker;

enum struct WerkerType : uint8_t
{
	InitialCheckUpWerker,
	ContinuousMonitoringWerker,
	ErrorWerker,
	StopWerker,
	TestWerker
};

class Werker
{
	protected:
		Werker ** werker_container;

		void swapWerker(Werker *);

	public:
		explicit Werker(Werker **, WerkerType);

		virtual void run() = 0;

		const WerkerType type;

		//virtual ~Werker() = 0;
};

class InitialCheckUpWerker : public Werker
{
	protected:
		ErrorWerker * error_werker;
		ContinuousMonitoringWerker * continuous_monitoring_werker;

	public:
		explicit InitialCheckUpWerker(Werker **, ErrorWerker*, ContinuousMonitoringWerker *);

		void run() override;
};

class ContinuousMonitoringWerker : public Werker
{
	protected:
		ErrorWerker * error_werker;
		StopWerker * stop_werker;

	public:
		explicit ContinuousMonitoringWerker(Werker **, ErrorWerker*, StopWerker *);

		void run() override;
};

class ErrorWerker : public Werker
{
	public:
		explicit ErrorWerker(Werker **);

		void run() override;
};

class StopWerker : public Werker
{
	public:
		explicit StopWerker(Werker **);

		void run() override;
};

class TestWerker : public Werker
{
	private:
		TestWerker * ptr;
	public:
		explicit TestWerker(Werker **);
		explicit TestWerker(Werker **, TestWerker *);

		void run() override;

		std::function <void()> fun;
};

#endif /* INC_WERKER_HPP_ */
