/*
 * MessageQueue.cpp
 *
 *  Created on: Oct 12, 2020
 *      Author: stefano
 */
#include "MessageQueue.hpp"

MessageQueue *MessageQueue::instance = 0;

MessageQueue *MessageQueue::getInstance() {
	if(instance == 0) {
		instance = new MessageQueue();
	}
	return instance;
}

MessageQueue::MessageQueue() {
	targetX = 0;
	targetY = 0;
	P1PositionX = 0;
	P1PositionY = 0;
	P2PositionX = 0;
	P2PositionY = 0;
	P3PositionX = 0;
	P3PositionY = 0;
	newTarget = false;
	newP1Position = false;
	newP2Position = false;
	newP3Position = false;
}

void MessageQueue::setTarget(int x, int y) {
	targetX = x;
	targetY = y;
	newTarget = true;
}

void MessageQueue::setP1Position(int x, int y) {
	P1PositionX = x;
	P1PositionY = y;
	newP1Position = true;
}

void MessageQueue::setP2Position(int x, int y) {
	P2PositionX = x;
	P2PositionY = y;
	newP2Position = true;
}

void MessageQueue::setP3Position(int x, int y) {
	P3PositionX = x;
	P3PositionY = y;
	newP3Position = true;
}

std::pair<int, int> MessageQueue::getTarget() {
	newTarget = false;
	return std::make_pair(targetX, targetY);
}

std::pair<int, int> MessageQueue::getP1Position() {
	newP1Position = false;
	return std::make_pair(P1PositionX, P1PositionY);
}

std::pair<int, int> MessageQueue::getP2Position() {
	newP2Position = false;
	return std::make_pair(P2PositionX, P2PositionY);
}

std::pair<int, int> MessageQueue::getP3Position() {
	newP3Position = false;
	return std::make_pair(P3PositionX, P3PositionY);
}

bool MessageQueue::isNewTarget() {
	return newTarget;
}

bool MessageQueue::isNewP1Position() {
	return newP1Position;
}

bool MessageQueue::isNewP2Position() {
	return newP2Position;
}

bool MessageQueue::isNewP3Position() {
	return newP3Position;
}
