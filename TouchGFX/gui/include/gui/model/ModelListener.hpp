#ifndef MODELLISTENER_HPP
#define MODELLISTENER_HPP

#include <gui/model/Model.hpp>

class ModelListener
{
public:
    ModelListener() : model(0) {}
    
    virtual ~ModelListener() {}

    void bind(Model* m)
    {
        model = m;
    }

    virtual void notifyTargetChanged(int, int) {}
    virtual void notifyP1PositionChanged(int, int) {}
    virtual void notifyP2PositionChanged(int, int) {}
    virtual void notifyP3PositionChanged(int, int) {}

protected:
    Model* model;
};

#endif // MODELLISTENER_HPP
