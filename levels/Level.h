#ifndef LEVEL_H_INCLUDED
#define LEVEL_H_INCLUDED

#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <fstream>
#include <iostream>

#include "../button_types/buttonRect.h"

//extern bool shouldQuit;

class Level
{
    public:
    static bool run_update, step_update;
    static bool do_capture;
    static Level* pCurrLvl;
    static Level* pMM;

    static float winW, winH;
    static float mapW, mapH;
    static float viewOfstX, viewOfstY;
    static sf::View mapView;
    static bool usingMapView;
    static sf::Color clearColor;

    static buttonRect goto_MMButt;
    static buttonRect quitButt;

    // support other common stuff
    const float myPI = 3.141593f;
    static sf::Text pauseMsg;
    static void flingLineUpdate();
    static sf::Vertex pullLine[2];
    static float flingVelGrad;
    static sf::CircleShape flingMark;
    static std::function<void(sf::Vector2f,sf::Vector2f)> flingPosVelFunc;// set position and velocity

    static bool handleEvent_stat( sf::Event& rEvent );
    static void init_stat( Level* p_MnMenu );
    static void update_stat( float dt );
    static void draw_stat( sf::RenderTarget& RT );
    static bool run_stat( sf::RenderWindow& RW, float dt );// does it all
    static bool run_stat( sf::RenderWindow& RW, float dt, std::function<void(float)> pUpFunc, std::function<void(sf::RenderTarget& RT)> pDrawFunc );// does it all
    static void cleanup_stat() { if( pCurrLvl && ( pCurrLvl != pMM ) ) delete pCurrLvl; }
    static void goto_MM();

    static void init_helpMsg( std::istream& is, sf::Text& helpMsg );

    Level();

    // pure virtual funcs
    virtual bool init() = 0;
    virtual void reset() { return; }// use in mainMenu to restore the state on return from a level

    virtual bool handleEvent( sf::Event& rEvent ) = 0;
    virtual void update( float dt ) = 0;// calls virtual logic()
    virtual void draw( sf::RenderTarget& RT ) const = 0;
    virtual void draw_map( sf::RenderTarget& RT ) const { (void)RT; return; }

 //   virtual void cleanup() = 0;
    virtual ~Level();
};

bool hitRect( sf::Rect<float> hitRect, sf::Vector2f pt );
void fitAinB( sf::FloatRect& A, sf::FloatRect B );// keep A within B


#endif // LEVEL_H_INCLUDED
