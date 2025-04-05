#include "button.h"
#include "textButton.h"

float button::mseX = 0.0f, button::mseY = 0.0f;
bool button::mseDnLt = false, button::mseDnRt = false;
bool button::mseDnLt_last = false, button::mseDnRt_last = false;
bool button::mseMoved = false;
bool button::didScroll = false;
bool button::anyInMapView = false;// NEW
float button::scrollAmount = 0.0f;
button* button::pButtMse = nullptr;// button under the mouse
textButton* button::pTxtButtFocus = nullptr;
std::vector<button*> button::RegisteredButtVec;
std::function<bool(void)> button::pOKtoRun = nullptr;
//bool button::usingMapView = false;
float button::dblClick_tLimit = 0.7f;
std::function<void(void)> button::pOnDblClickLeft = nullptr;
std::function<void(void)> button::pOnDblClickRight = nullptr;

// supporting mouse over delay
int button::delayCnt = 0;       // is condition for updating & drawing moVals
sf::Text button::moValtxt;// real time value
sf::Vertex button::moValBox[4];// Quad as bkgd
const  sf::Font* button::pFont = nullptr;// about time?
int button::moDelay = 30;// about 1/2 second?

button::button(): mseOver(false), sel(false), pos(0.0f,0.0f) {}

button::~button() {}// dtor

bool button::MseOver()// assigns mseOver
{
    if( status != Status::active ) { mseOver = false; return false; }// NEW

    if(  hit() ) mseOver = true;
    else mseOver = false;

    if( !mseOver && mode == 'I' && sel ) // support NOHC operation
    {
        if( pHitFunc ) pHitFunc();
   //     sel = false;// older
        setSel(false);// new
    }

    return mseOver;
}

bool button::hitLeft()// alters sel
{
    if( status != Status::active ) return false;// NEW

    if( ( mode == 'T' || mode == 'L' ) && !mseDnLt ) return false;// new

    if( hit() )
    {
        setSel( !sel );// toggle - new
        if( mseDnLt && pHitFunc ) pHitFunc();// call the function!
        return true;
    }
    return false;
}

void button::set_inDefView( bool InDefView )// NEW
{
    inDefView = InDefView;
    button::check_anyInMapView();
}

// *** static functions

void button::init( const sf::Font& rFont, int mo_delay, sf::Color hoverBoxColor )
{
    button::pFont = &rFont;
    button::moDelay = mo_delay;// about 1/2 second?

    sf::Vector2f defPos(0.0f,0.0f);
    button::moValtxt.setPosition(defPos);
    button::moValtxt.setFont(rFont);
    button::moValtxt.setCharacterSize(12);
    button::moValtxt.setFillColor( sf::Color::Cyan );
    button::moValtxt.setString("0.0");
    pButtMse = nullptr;
    pTxtButtFocus = nullptr;
    mseMoved = didScroll = false;
    scrollAmount = 0.0f;

//    sf::Vector2f& sz = buttonValOnHit::moValBoxSz;
    sf::Vertex* pV = button::moValBox;
    pV[0].color = pV[1].color = pV[2].color = pV[3].color = hoverBoxColor;
}

bool button::check_anyInMapView()
{
    for( button* pButt : button::RegisteredButtVec )
    {
        if( !pButt->inDefView )
        {
            button::anyInMapView = true;
            return true;
        }
    }

    button::anyInMapView = false;
    return false;
}

void button::handleEvent(  sf::Event& rEvent )
{
    if (rEvent.type == sf::Event::MouseMoved)// *** mouse moved
    {
        mseX = rEvent.mouseMove.x;
        mseY = rEvent.mouseMove.y;
        mseMoved = true;
    }
    else if (rEvent.type == sf::Event::MouseButtonPressed)
    {
        mseX = rEvent.mouseButton.x;
        mseY = rEvent.mouseButton.y;

        if (rEvent.mouseButton.button == sf::Mouse::Left) mseDnLt = true;
        else if (rEvent.mouseButton.button == sf::Mouse::Right) mseDnRt = true;
    }
    else if (rEvent.type == sf::Event::MouseButtonReleased)// lbutt up
    {
        mseX = rEvent.mouseButton.x;
        mseY = rEvent.mouseButton.y;

        if (rEvent.mouseButton.button == sf::Mouse::Left)  mseDnLt = false;
        else if (rEvent.mouseButton.button == sf::Mouse::Right) mseDnRt = false;
    }
    else if (rEvent.type == sf::Event::MouseWheelScrolled)
    {
        scrollAmount = (float)rEvent.mouseWheelScroll.delta;
        didScroll = true;
    }
    else if ( pTxtButtFocus && rEvent.type == sf::Event::TextEntered )
        pTxtButtFocus->pushChar( rEvent.text );

    return;
}

void button::reset()
{
    mseDnRt_last = mseDnRt;
    mseDnLt_last = mseDnLt;
    mseMoved = didScroll = false;
    scrollAmount = 0.0f;
}

// called if map view buttons present
void button::MseOverAll( float ofstX , float ofstY )
{
    static button* pButtMse_last = nullptr;
    pButtMse_last = pButtMse;

    if( pButtMse )// perhaps still under?
    {
        if( pButtMse->inDefView )
        {
            if( !pButtMse->MseOver() ) pButtMse = nullptr;
        }
        else// map view
        {
            pButtMse->mseOver = false;
            pButtMse = nullptr;
        }
    }

   if( !pButtMse )
   {
    for( auto& pButt : RegisteredButtVec )
        if( pButt->status == Status::active && pButt->inDefView && pButt->MseOver() )// break;
        {
            pButtMse = pButt;
         //   break;// found it!
        }
   }

    if( !pButtMse && button::anyInMapView )// perhaps a map view one?
    {
        mseX += ofstX; mseY += ofstY;
        for( auto& pButt : RegisteredButtVec )
            if( pButt->status == Status::active && !pButt->inDefView && pButt->MseOver() )
                pButtMse = pButt;// no break because buttons may overlap in map view. The last in the list is drawn on top

        if( !pButtMse ) { mseX -= ofstX; mseY -= ofstY; }
    }

    if( !pButtMse && pButtMse_last )// mouse slid off button. Reset buttons to up?
    {
        mseDnLt = mseDnLt_last = mseDnRt = mseDnRt_last = false;
        delayCnt = 0;
    }

    if( pButtMse_last != pButtMse ) delayCnt = 0;

    if( clickEvent_Lt() == 1 || clickEvent_Rt() == 1 )// presume no textButton hit. Call to hitLeft() will restore
    {
        if( pTxtButtFocus ) pTxtButtFocus->setSel(false);// so it will be selected if clicked again
        pTxtButtFocus = nullptr;// click missed all buttons
    }

    if( pOKtoRun && !pOKtoRun() ) { pButtMse = nullptr; return; }// ignore click activity
    if( pButtMse )// mouse is over a button
    {
        if( clickEvent_Lt() != 0 ) pButtMse->hitLeft();// press and release
        if( clickEvent_Rt() != 0 ) pButtMse->hitRight();// right button just pressed

        if( pButtMse && ( delayCnt < button::moDelay ) && ++delayCnt == button::moDelay )
        pButtMse->init_delayBox();

        if( pButtMse && !pButtMse->inDefView ) { mseX -= ofstX; mseY -= ofstY; }
    }

    for( auto& pButt : RegisteredButtVec ) if( pButt->status == Status::active && pButt->inDefView ) pButt->update();

    if( button::anyInMapView )
    {
        mseX += ofstX; mseY += ofstY;
        for( auto& pButt : RegisteredButtVec ) if( pButt->status == Status::active && !pButt->inDefView ) pButt->update();
        mseX -= ofstX; mseY -= ofstY;
    }
}

void button::MseOverAll()// all in default view
{
    static button* pButtMse_last = nullptr;
    pButtMse_last = pButtMse;

    // check if stiil over last
    if( pButtMse && !pButtMse->MseOver() ) pButtMse = nullptr;

    if( !pButtMse )// nope. Check all
        for( auto& pButt : RegisteredButtVec )
            if( pButt->status == Status::active && pButt->MseOver() ) pButtMse = pButt;

    if( !pButtMse && pButtMse_last )// mouse slid off button. Reset buttons to up?
    {
        mseDnLt = mseDnLt_last = mseDnRt = mseDnRt_last = false;
        delayCnt = 0;
    }

    if( pButtMse_last != pButtMse ) delayCnt = 0;

    if( clickEvent_Lt() == 1 || clickEvent_Rt() == 1 )// presume no textButton hit. Call to hitLeft() will restore
    {
        if( pTxtButtFocus ) pTxtButtFocus->setSel(false);// so it will be selected if clicked again
        pTxtButtFocus = nullptr;// click missed all buttons
    }

    if( pButtMse )// mouse is over a button
    {
        if( clickEvent_Lt() != 0 ) pButtMse->hitLeft();// press and release
        if( clickEvent_Rt() != 0 ) pButtMse->hitRight();// right button just pressed

        if( pButtMse && ( delayCnt < button::moDelay ) && ++delayCnt == button::moDelay )
        pButtMse->init_delayBox();
    }

    for( auto& pButt : RegisteredButtVec ) { pButt->update(); }//

    // off button double click support  NEW 11/22/24
    if( pOnDblClickLeft && pOnDblClickRight )
    {
        int DCE = dblClickPoll( 0.02f );
        if( DCE == 1 ) pOnDblClickLeft();
        else if( DCE == -1 ) pOnDblClickRight();
    }
}

void button::drawAll_def( sf::RenderTarget& rRT )
{
    for( auto& pButt : RegisteredButtVec )
        if( pButt->status != Status::hidden && pButt->inDefView ) pButt->draw(rRT);
}

void button::drawAll_map( sf::RenderTarget& rRT )
{
    if( !button::anyInMapView ) return;

    for( auto& pButt : RegisteredButtVec )
        if( pButt->status != Status::hidden && !pButt->inDefView ) pButt->draw(rRT);
}

void button::unRegisterMe( button* pButton )// erase the pointer in RegisteredButtVec
{
    std::vector<button*>::iterator itButt = RegisteredButtVec.begin();
    while( itButt != RegisteredButtVec.end() && *itButt != pButton ) ++itButt;

    if( itButt != RegisteredButtVec.end() )// found it!
    {
        RegisteredButtVec.erase(itButt);// no more automatic MseOver, draw calls for you
        if( pButtMse == pButton ) pButtMse = nullptr;// it will be honored next frame however, if you're still around
    }
}

bool button::registerUnique( button* pButton )// returns true if newly registered
{
    bool pcFound = false;
    for( auto& pButt : RegisteredButtVec ) if( pButt == pButton ){ pcFound = true; break; }
    if( !pcFound ) { button::RegisteredButtVec.push_back( pButton ); }
    return !pcFound;
}

int button::clickEvent_Lt()// 0= no event +1= down -1= up
{
    if( mseDnLt == mseDnLt_last ) return 0;
    return mseDnLt ? 1 : -1;// down: up
}

int button::clickEvent_Rt()
{
    if( mseDnRt == mseDnRt_last ) return 0;
    return mseDnRt ? 1 : -1;// down: up
}

void button::grabFocus( button* pButt )
{
    if( button::pButtMse )
    {
        button::pButtMse->setMO( false );
        button::pButtMse->setSel(false);
        button::pButtMse = pButt;
    }
    if( button::pTxtButtFocus )
    {
        button::pTxtButtFocus->setSel(false);
        button::pTxtButtFocus = nullptr;
    }
}

// utility
bool button::hitFloatRect( const sf::FloatRect& FR )
{
    if( mseX < FR.left ) return false;
    if( mseX > FR.left + FR.width ) return false;
    if( mseY < FR.top ) return false;
    if( mseY > FR.top + FR.height ) return false;
    return true;
}

bool button::hitWithinR( sf::Vector2f Pos, float R )
{
    sf::Vector2f sep = button::msePos() - Pos;
    if( sep.x*sep.x + sep.y*sep.y < R*R ) return true;
    return false;
}

// call in update(). returns: 0 = no event, 1 = left dbl, -1 = right dbl
int button::dblClickPoll( float dt )
{
    static float tElap = 0.0f;

    // increment or decrement tElap
    if( tElap != 0.0f )
    {
        int TE = tElap > 0.0f ? 1 : -1;
        tElap += TE*dt;
        if( TE*tElap > dblClick_tLimit )
        {
            tElap = 0.0f;// time out
            return 0;
        }
    }

    // handle click events
    int bce = 0;
    if( clickEvent_Lt() == 1 ) bce = 1;// left button was pressed
    else if( clickEvent_Rt() == 1 ) bce = -1;// right button was pressed

    if( bce != 0 )
    {
        if( tElap == 0.0f )// 1st click
        {
            tElap = bce*dt;// start timer
            bce = 0;// not a double
        }
        else if( bce*tElap < 0.0f )// 2nd click is contrary to 1st
        {
            tElap = 0.0f;// reset
            bce = 0;// not a double
        }
        // else go with bce
    }

    return bce;
}
