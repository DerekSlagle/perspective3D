#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>

#include <vector>
#include <fstream>
//#include <algorithm>
#include<iostream>
//#include<time.h>

#include "include/levels/menu.h"
#include "include/levels/lvl_Outside.h"

unsigned int wdwW = 900, wdwH = 640;// window dimensions
float Level::winW = (float)wdwW;
float Level::winH = (float)wdwH;

Level* loadLevel( size_t I );

int main()
{
    sf::Font font;
    if (!font.loadFromFile("arial.ttf")) return 2;
    button::init( font, 30 );

    menu main_Menu;
    Level::init_stat( &main_Menu );
    if( !main_Menu.init() ) { std::cout << "main menu init fail\n"; return 2; }
    main_Menu.pLoadLvl = loadLevel;

    Level::pCurrLvl = &main_Menu;
    if( !Level::pCurrLvl ) return 2;
    button::RegisteredButtVec.push_back( &Level::quitButt );// new

    wdwW = floor( abs( Level::winW ) );
    wdwH = floor( abs( Level::winH ) );

    sf::RenderWindow RW(sf::VideoMode(wdwW, wdwH), "Various",  sf::Style::Titlebar | sf::Style::Close );
    RW.setVerticalSyncEnabled(true);
    float dt = 1.0f;


    sf::Clock clock1;
    sf::Time t1 = clock1.getElapsedTime(), t2 = t1;

     while (RW.isOpen())
     {
        button::reset();

        sf::Event event;
        while (RW.pollEvent(event))
            if( !Level::handleEvent_stat( event ) )
            { RW.close(); break; }

        t2 = clock1.getElapsedTime();
        dt = ( t2 - t1 ).asSeconds();
        t1 = t2;
        Level::update_stat(dt);

        // draw
        RW.clear( Level::clearColor );// sf::Color(0,64,128) )
        Level::draw_stat(RW);
        RW.display();
     }

    Level::cleanup_stat();

    return 0;
}

Level* loadLevel( size_t I )
{
    Level* pLvl = nullptr;

    switch( I )
    {
        case 0 :
        pLvl = new lvl_Outside;
        break;
    }

    if( !pLvl ) return nullptr;
    if( pLvl->init() ) return pLvl;
    else { delete pLvl; return nullptr; }
}
