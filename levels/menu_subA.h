#ifndef MENU_SUBA_H_INCLUDED
#define MENU_SUBA_H_INCLUDED

#include<string>
//#include "Level.h"

//#include "LvlQM.h"
#include "LvlQM_FSW.h"
#include "LvlQM_HarmOsci.h"
#include "../button_types/buttonRect.h"

class menu_subA : public Level
{
    public:
    std::vector<button*> myButtVec;// load in init() and use to restore to button::RegisteredButtVec in reset()
    std::vector<sf::Text> msgVec;
    std::vector< sf::Rect<float> > rectVec;
    sf::Color msgClrReg;
    sf::Color msgClrMseOver;
    size_t numOptions = 0;
    size_t lvlNum = 0;
    Level* p_subLevel = nullptr;

    buttonRect gotoMeButt;// for sub level use to return here

 //   okBox levelBox;

    // functions
    menu_subA():Level() { std::cout << "Hello from menu_subA ctor\n"; }

    virtual bool init();
    virtual void reset();

    virtual bool handleEvent( sf::Event& rEvent );
    virtual void update( float dt );
    virtual void draw( sf::RenderTarget& RT ) const;

    bool loadLevel( size_t I );// true if load is good
    void cleanup();
    virtual ~menu_subA();
};

#endif // MENU_SUBA_H_INCLUDED
