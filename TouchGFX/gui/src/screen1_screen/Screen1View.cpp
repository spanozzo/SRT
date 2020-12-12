#include <gui/screen1_screen/Screen1View.hpp>

Screen1View::Screen1View()
{

}

void Screen1View::setupScreen()
{
    Screen1ViewBase::setupScreen();
}

void Screen1View::tearDownScreen()
{
    Screen1ViewBase::tearDownScreen();
}

void Screen1View::setTarget(int x, int y) {
    dot.moveTo(x, y);
    dot.invalidate();
}

void Screen1View::setP1Position(int x, int y) {
    player1.moveTo(x,y);
    player1.invalidate();
}

void Screen1View::setP2Position(int x, int y) {
    player2.moveTo(x,y);
    player2.invalidate();
}

void Screen1View::setP3Position(int x, int y) {
    player3.moveTo(x,y);
    player3.invalidate();
}