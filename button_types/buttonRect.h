#ifndef BUTTONRECT_H
#define BUTTONRECT_H

#include "button.h"


class buttonRect : public button
{
    public:
        static sf::Color selClr, unSelClr, moClr, unMoClr;

        sf::Vector2f sz;
        sf::RectangleShape R;
        sf::Text title;

        buttonRect();
        virtual ~buttonRect();
        virtual bool hit()const;
        virtual void draw( sf::RenderTarget& rRW )const;
        virtual void setPosition( sf::Vector2f Pos );

        virtual bool MseOver();// assigns mseOver - virtual due to ValOnHit live display feature
        virtual bool hitLeft();
        virtual void setSel( bool Sel );
        virtual void setMO( bool MO ) { mseOver = MO; R.setOutlineColor( (mseOver ? moClr : unMoClr ) ); }

        void init( float x, float y, float W, float H, std::function<void(void)> p_HitFunc, const sf::Text& Title = sf::Text() );
        buttonRect( float x, float y, float W, float H, std::function<void(void)> p_HitFunc, const sf::Text& Title = sf::Text() ) { init( x, y, W, H, p_HitFunc, Title ); }

        void init( float x, float y, float W, float H, std::function<void(void)> p_HitFunc, const char* label );
        buttonRect( float x, float y, float W, float H, std::function<void(void)> p_HitFunc, const char* label ) { init( x, y, W, H, p_HitFunc, label ); }
        void init( std::istream& is, std::function<void(void)> p_HitFunc );
};

//void initButtRects( sf::RectangleShape* R, float W, float H );// drawn
//void initButtRects( sf::RectangleShape* R, float W, float H, sf::Vector2f Pos, sf::Color unSelClr, sf::Color unSelMoClr, sf::Color selClr, sf::Color selMoClr );// drawn

#endif // BUTTONRECT_H
