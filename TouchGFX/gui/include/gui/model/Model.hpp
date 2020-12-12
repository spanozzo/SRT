#ifndef MODEL_HPP
#define MODEL_HPP
#include "MessageQueue.hpp"

class ModelListener;

class Model
{
public:
    Model();

    void bind(ModelListener* listener)
    {
        modelListener = listener;
    }

    void tick();
protected:
    ModelListener* modelListener;
private:
    MessageQueue *msg;
};

#endif // MODEL_HPP
