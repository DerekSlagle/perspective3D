#ifndef BUTTON_H
#define BUTTON_H

#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>// to process events

#include<cmath>
#include<vector>
#include<functional>
#include<sstream>
#include<string>
#include<iostream>

class textButton;

class button
{
    public:
        // static members
        static button* pButtMse;// button under the mouse
        static textButton* pTxtButtFocus;// button under the mouse
        static std::vector<button*> RegisteredButtVec;
        static std::function<bool(void)> pOKtoRun;

        static float mseX, mseY;
        static bool mseDnLt, mseDnRt;
        static bool mseDnLt_last, mseDnRt_last;
        static bool mseMoved;

        static bool didScroll;
        static float scrollAmount;
   //     static bool usingMapView;

        // static functions
        static void unRegisterMe( button* pButton );// erase the pointer in RegisteredButtVec
        static bool registerUnique( button* pButton );// add to RegisteredButtVec only if not already present
        static void init( const sf::Font& rFont, int mo_delay, sf::Color hoverBoxColor = sf::Color::Black );// new
        static void handleEvent(  sf::Event& rEvent );
        static void reset();
        static void MseOverAll( float ofstX, float ofstY );// when mapView in use
        static void MseOverAll();// when default view only
        static void drawAll_def( sf::RenderTarget& rRT );
        static void drawAll_map( sf::RenderTarget& rRT );
        static int clickEvent_Lt();// 0= no event +1= down -1= up
        static int clickEvent_Rt();
        static bool clickEvent() { return ( clickEvent_Lt() != 0 || clickEvent_Rt() != 0 ); }
        static void grabFocus( button* pButt );
        static void setHoverBoxColor( sf::Color hbColor ) { for( size_t i=0; i<4; ++i ) moValBox[i].color = hbColor; }
        // double click event detection NEW 11/22/24
        static float dblClick_tLimit;
        static int dblClickPoll( float dt );// user may call in update(). 0 = no event, 1 = left dbl, -1 = right dbl
        static std::function<void(void)> pOnDblClickLeft, pOnDblClickRight;// will call in MseOverAll() if both assigned

        // utility
        static bool hitFloatRect( const sf::FloatRect& FR );
        static bool hitWithinR( sf::Vector2f Pos, float R );
        static sf::Vector2f msePos() { return sf::Vector2f( mseX, mseY ); }
        static bool check_anyInMapView();

        // mouse over delay box + message
        static sf::Text moValtxt;// real time value
        static sf::Vertex moValBox[4];// Quad as bkgd
        const  static sf::Font* pFont;// about time?
        static int moDelay;// # frames to trigger drawing this.
        static int delayCnt;
        static bool anyInMapView;

        enum Status : int { active, inactive, hidden };// button display and use

        // regular members
        Status status = Status::active;// NEW
        bool mseOver, sel;
        bool inDefView = true;// default view. Apply no offset
        void set_inDefView( bool InDefView );// NEW
        char mode = 'T';// 'T' = toggle, 'I' = intermittent (NOHC)
        sf::Vector2f pos;
        std::function<void(void)> pHitFunc = nullptr;// will be called in handleEvent(), so do MseOver() calls 1st?

        // regular functions
        button();
        virtual ~button();

        virtual void init_delayBox() { return; }
    //    virtual void setSel( bool Sel ) { sel = Sel; }// over in buttonList to open/close list, okBox, colorPicker
        virtual void setSel( bool Sel ) { sel = Sel; }// if( mseDnLt && pHitFunc ) pHitFunc(); }// over in buttonList to open/close list, okBox, colorPicker
        virtual void setMO( bool MO ) { mseOver = MO; }// over in buttonRect, colorPicker
        virtual bool hit()const = 0;
        virtual void draw( sf::RenderTarget& RT )const = 0;
        virtual bool MseOver();// assigns mseOver - virtual due to ValOnHit live display feature
        virtual bool hitLeft();// alters sel - overload in ValOnHit
        virtual bool hitRight() { return mseOver; }// overload in joyButton
        virtual void setPosition( sf::Vector2f Pos ) = 0;
        void setPosition( float x, float y ){ setPosition( sf::Vector2f(x,y) ); }
        virtual void update() { return; }
};

    template<class T>
    float to_SF_string(sf::Text& strArg, T x, size_t prcs = 4 )// returns position of end of strArg
    {
        std::stringstream ss;
        ss.precision( prcs );
        ss << x;
        std::string tempStr;
        ss >> tempStr;
        strArg.setString( tempStr.c_str() );
        sf::FloatRect fr = strArg.getLocalBounds();
        return fr.left + fr.width;
    }

#endif // BUTTON_H
