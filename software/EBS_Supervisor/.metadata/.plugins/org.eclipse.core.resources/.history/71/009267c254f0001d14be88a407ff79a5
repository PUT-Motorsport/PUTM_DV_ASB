/*
 * tokens.hpp
 *
 *  Created on: 27 wrz 2022
 *      Author: pile
 */

#ifndef INC_TOKENS_HPP_
#define INC_TOKENS_HPP_

class HandlerToken
{
	public:
		//void finish();

		bool enabled();

		friend class HandlerTokenSource;
	protected:
		bool _enable = false;
		//bool _finished = false;
};

class HandlerTokenSource
{
	public:
		void start();
		void stop();

		bool enabled();

		//bool finished();

		HandlerToken* getToken();

	private:
		HandlerToken token;
};

#endif /* INC_TOKENS_HPP_ */
