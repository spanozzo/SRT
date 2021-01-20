#include <gui/model/Model.hpp>
#include <gui/model/ModelListener.hpp>
#include <utility>

Model::Model() : modelListener(0)
{
    msg = MessageQueue::getInstance();
}

void Model::tick()
{
    if(msg->isNewTarget()) {
        std::pair<int, int> target = msg->getTarget();
        modelListener->notifyTargetChanged(target.first, target.second);
    }
    if(msg->isNewP1Position()) {
        std::pair<int, int> position = msg->getP1Position();
        modelListener->notifyP1PositionChanged(position.first, position.second);
    }
    if(msg->isNewP2Position()) {
        std::pair<int, int> position = msg->getP2Position();
        modelListener->notifyP2PositionChanged(position.first, position.second);
    }
    if(msg->isNewP3Position()) {
        std::pair<int, int> position = msg->getP3Position();
        modelListener->notifyP3PositionChanged(position.first, position.second);
    }
}
