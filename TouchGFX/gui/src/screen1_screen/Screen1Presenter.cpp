#include <gui/screen1_screen/Screen1View.hpp>
#include <gui/screen1_screen/Screen1Presenter.hpp>

Screen1Presenter::Screen1Presenter(Screen1View& v)
    : view(v)
{

}

void Screen1Presenter::activate()
{

}

void Screen1Presenter::deactivate()
{

}

void Screen1Presenter::notifyTargetChanged(int x, int y) {
    view.setTarget(x, y);
}

void Screen1Presenter::notifyP1PositionChanged(int x, int y) {
    view.setP1Position(x, y);
}

void Screen1Presenter::notifyP2PositionChanged(int x, int y) {
    view.setP2Position(x, y);
}

void Screen1Presenter::notifyP3PositionChanged(int x, int y) {
    view.setP3Position(x, y);
}
