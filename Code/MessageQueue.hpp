/*
 * MessageQueue.hpp
 *
 *  Created on: Oct 12, 2020
 *      Author: stefano
 */

#ifndef MESSAGEQUEUE_HPP_
#define MESSAGEQUEUE_HPP_

#include <utility>

class MessageQueue {
public:
	static MessageQueue *getInstance();
	void setTarget(int, int);
	void setP1Position(int, int);
	void setP2Position(int, int);
	void setP3Position(int, int);
	std::pair<int, int> getTarget();
	std::pair<int, int> getP1Position();
	std::pair<int, int> getP2Position();
	std::pair<int, int> getP3Position();
	bool isNewTarget();
	bool isNewP1Position();
	bool isNewP2Position();
	bool isNewP3Position();

	/*
	 * Posso fare un'analisi trasformando le variabili in buffer, così se un task salta un'attivazione, il display può
	 * continuare a renderizzare.
	 */
private:
	MessageQueue();
	static MessageQueue *instance;
	int targetX;
	int targetY;
	int P1PositionX;
	int P1PositionY;
	int P2PositionX;
	int P2PositionY;
	int P3PositionX;
	int P3PositionY;
	bool newTarget;
	bool newP1Position;
	bool newP2Position;
	bool newP3Position;
};

#endif /* MESSAGEQUEUE_HPP_ */
