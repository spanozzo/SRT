#ifndef SCREEN1VIEW_HPP
#define SCREEN1VIEW_HPP

#include <gui_generated/screen1_screen/Screen1ViewBase.hpp>
#include <gui/screen1_screen/Screen1Presenter.hpp>

class Screen1View : public Screen1ViewBase
{
public:
    Screen1View();
    virtual ~Screen1View() {}
    virtual void setupScreen();
    virtual void tearDownScreen();

    virtual void setTarget(int, int);
    virtual void setP1Position(int, int);
    virtual void setP2Position(int, int);
    virtual void setP3Position(int, int);
protected:
};

#endif // SCREEN1VIEW_HPP
