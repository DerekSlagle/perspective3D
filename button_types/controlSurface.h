#ifndef CONTROLSURFACE_H_INCLUDED
#define CONTROLSURFACE_H_INCLUDED

#include<iostream>
#include "buttonRect.h"

class controlSurface : public button
{
    public:
    sf::Vector2f sz;
    sf::RectangleShape bkRect;// control surface
    button* pButt_mse = nullptr;
    bool onSurfaceOnly = true;
    bool ownButts = false;
    buttonRect homeButt;
    bool isOpen() const { return homeButt.sel; }
    std::vector<button*> pButtVec;// these too!
    std::vector< std::function<button*(void)> > condButtVec;// qualified calling: pButt = v(); if( pButt ) pButt->draw(RT); etc

    controlSurface(){};
    controlSurface( sf::Vector2f PosHmButt, sf::Vector2f PosSurf, sf::Vector2f Sz, const char* label ){ init( PosHmButt, PosSurf, Sz, label ); }
    bool init( sf::Vector2f PosHmButt, sf::Vector2f PosSurf, sf::Vector2f Sz, const char* label, float wHB = 50.0f, float hHB = 25.0f );
    bool init( const char* label, std::istream& is );
    virtual ~controlSurface();

    // mfs
//    virtual button* MseOverAll() = 0;
//    virtual void drawAll( sf::RenderTarget& RT )const = 0;
 //   virtual void setPositionAll( sf::Vector2f Pos ) = 0;
 //   virtual void updateAll() = 0;

    std::function<button*(void)> MseOverAll = nullptr;
    std::function<void(sf::RenderTarget& RT)> drawAll = nullptr;
    std::function<void(sf::Vector2f)> setPositionAll = nullptr;
    std::function<void(void)> updateAll = nullptr;

    // required button functions
    virtual void setSel( bool Sel );// over in buttonList to open/close list, okBox, colorPicker
    virtual void setMO( bool MO );// over in buttonRect, colorPicker
    virtual bool hit()const;
    virtual void draw( sf::RenderTarget& RT )const;
    virtual bool MseOver();// assigns mseOver - virtual due to ValOnHit live display feature
    virtual bool hitLeft();// alters sel - overload in ValOnHit
    virtual bool hitRight();// overload in joyButton
    virtual void setPosition( sf::Vector2f Pos );
    virtual void init_delayBox();
    virtual void update();
//    virtual void update() { if( sel ) updateAll(); }
};

bool hitRect( sf::FloatRect Fr );

#endif // CONTROLSURFACE_H_INCLUDED
