#include "buttonList.h"

buttonList::~buttonList()
{
    if( ownButts )
    {
        std::cerr << "\n list owned " << pButtVec.size() << " butts";
        for( auto& butt : pButtVec ) delete butt;
    }
}

void buttonList::init( float x, float y, float W, float H, const sf::Text& Title )
{
 //   sf::RectangleShape pR[4];
 //   initButtRects( pR, W, H );
 //   buttonRect::init( pR, x, y, Title );

    buttonRect::init( x, y, W, H, nullptr, Title );
}

void buttonList::init( float x, float y, float W, float H, float dy, const std::vector<sf::Text>& TitleVec,
    const std::vector< std::function<void(void)> > pFuncVec, std::vector<buttonRect>& recButtVec, char side )
{
    ownButts = false;
    init( x, y, W, H, TitleVec[0] );// homeButt
    pHitFunc = pFuncVec[0];
 //   pHitFunc_up = pHitFunc;
    buttonRect tmpButt;

    if( side == 'B' ) y+=dy;
    else if( side == 'L' ) x -= W;
    else if( side == 'R' ) x += W;

    for( size_t i=1; i< pFuncVec.size(); ++i )
    {
        tmpButt.init( x, y, W, H, nullptr, TitleVec[i] );
        tmpButt.pHitFunc = pFuncVec[i];
        recButtVec.push_back( tmpButt );
        y+=dy;
    }

    for( auto& butt : recButtVec )
        pButtVec.push_back( &butt );
}

void buttonList::init( float x, float y, float W, float H, float dy, const std::vector<sf::Text>& TitleVec,
    const std::vector< std::function<void(void)> > pFuncVec, char side )
{
    ownButts = true;
    init( x, y, W, H, TitleVec[0] );// homeButt
    pHitFunc = pFuncVec[0];
 //   pHitFunc_up = pHitFunc;

    if( side == 'B' ) y+=dy;
    else if( side == 'L' ) x -= W;
    else if( side == 'R' ) x += W;

    for( size_t i=1; i< pFuncVec.size() && i< TitleVec.size(); ++i )
    {
        buttonRect* pBtRct = new buttonRect;
        pBtRct->init( x, y, W, H, nullptr, TitleVec[i] );
        pBtRct->pHitFunc = pFuncVec[i];
        pButtVec.push_back( pBtRct );
        y+=dy;
    }
}

void buttonList::draw( sf::RenderTarget& rRW )const
{
    buttonRect::draw(rRW);
    if( sel ) for( auto& pButt : pButtVec ) pButt->draw(rRW);
}

/*
bool buttonList::MseOver()
{
    mseOver = false;
 //   pButt_mse = nullptr;

    if(  hit() ) { mseOver = true; if( pButt_mse ) pButt_mse->mseOver = false; pButt_mse = nullptr; }
    else if( sel )// list is open
        for( auto& pButt : pButtVec )// find button under mouse
            if( pButt->MseOver() ) pButt_mse = pButt;

    // close list if click was off control
    if( persist == 1 && sel && !mseOver && !pButt_mse && button::clickEvent_Lt() == 1 ) setSel(false);

    return mseOver || pButt_mse;
}   */


void buttonList::setPosition( sf::Vector2f Pos )
{
    sf::Vector2f dPos = Pos - pos;
    buttonRect::setPosition( Pos );
    for( auto& pButt : pButtVec ) pButt->setPosition( pButt->pos + dPos );
}

 void buttonList::init_delayBox()
 {
 //   if( openOnHover && !sel ) { openList(); return; }

    if( pButt_mse ) pButt_mse->init_delayBox();
    return;
 }

void buttonList::openList()
{
    sel = true;
    pButt_mse = nullptr;
    pButt_sel = nullptr;// new
    R.setFillColor( selClr );
    for( auto& pButt : pButtVec )
    {
    //    pButt->mseOver = false;
        pButt->MseOver();
        pButt->setSel( false );
    }
}

void buttonList::closeList()
{
    sel = false;
 //   if( pButt_mse ) pButt_mse->mseOver = false;
    if( pButt_mse ) pButt_mse->setMO( false );
    pButt_mse = nullptr;
    if( pButt_sel ) pButt_sel->setSel( false );// new
    pButt_sel = nullptr;// new
    R.setFillColor( unSelClr );
}

void buttonList::setSel( bool Sel )
{
    if( sel != Sel )
    {
        if( Sel ) openList();
        else closeList();
        sel = Sel;
    //    R.setFillColor( (sel ? selClr : unSelClr ) );
    }
}

bool buttonList::MseOver()
{
    button* pButt_mseLast = pButt_mse;

    if( sel && pButt_mse && pButt_mse->MseOver() ) return true;// NEW - skip search if still over last

    pButt_mse = nullptr;

    if(  hit() ){ if( !mseOver ) setMO( true ); }//R.setOutlineColor( moClr ); mseOver = true; }
    else
    {
        // mouse left button area before release event. Complete function call cycle.
        if( mseOver && mode == 'I' && button::mseDnLt && pHitFunc_up ) pHitFunc_up();//  mseOver value from last frame
        setMO( false );
   //     mseOver = false;
   //     R.setOutlineColor( unMoClr );
    }

    if( sel )// list is open
    {
        for( auto& pButt : pButtVec )// find button under mouse
            if( pButt->MseOver() ) pButt_mse = pButt;

        if( pButt_mseLast != pButt_mse ) button::delayCnt = 0;
    }
    // close list if click was off control
 //   if( persist == 1 && sel && !mseOver && !pButt_mse && button::clickEvent_Lt() == 1 ) setSel(false);
    bool mseOverAny = mseOver || pButt_mse;
 //   if( openOnHover && !mseOverAny ) closeList();

    return mseOverAny;
}

void buttonList::update()
{
    if( sel )
    {
        for( auto& pButt : pButtVec )
            pButt->update();
    }

    // close list if click was off control
    if( persist == 1 && sel && !mseOver && !pButt_mse && button::clickEvent_Lt() == 1 ) setSel(false);
}

/*
bool buttonList::MseOver()
{
    mseOver = false;
    pButt_mse = nullptr;

    if(  hit() ) mseOver = true;

    if( sel )// list is open
        for( auto& pButt : pButtVec )// find button under mouse
            if( pButt->MseOver() ) pButt_mse = pButt;

    // close list if click was off control
    if( persist == 1 && sel && !mseOver && !pButt_mse && button::clickEvent_Lt() == 1 ) setSel(false);

    return mseOver || pButt_mse;
}
*/

bool buttonList::hitLeft()
{
    if( hit() )// open/close button hit
    {
        if( !openOnRight && mseDnLt ) setSel( !sel );// new
        if( mseDnLt ) { if( pHitFunc ) pHitFunc(); }
        else if( pHitFunc_up ) pHitFunc_up();
        return true;
    }

    return hitList();
}

bool buttonList::hitRight()
{
    if( openOnRight && mseDnRt && hit() ) setSel( !sel );// new. No function called when openOnRight
    return hitList();
}

bool buttonList::hitList()
{
    if( sel && pButt_mse )// all open
    {
     //   if( pButt_mse && pButt_mse->sel && pButt_sel != pButt_mse ) { if( pButt_sel ) pButt_sel->setSel(false); pButt_sel = pButt_mse; }// new position
        if( pButt_mse && pButt_sel != pButt_mse ) { if( pButt_sel ) pButt_sel->setSel(false); pButt_sel = pButt_mse; }// new position

        if( button::clickEvent_Lt() != 0 ) pButt_mse->hitLeft();
        else if( button::clickEvent_Rt() != 0 ) pButt_mse->hitRight();
        else return false;// no click event

        // update selected button
     //   if( pButt_mse->sel && pButt_sel != pButt_mse ) { if( pButt_sel ) pButt_sel->setSel(false); pButt_sel = pButt_mse; }
//        if( pButt_mse && pButt_mse->sel && pButt_sel != pButt_mse ) { if( pButt_sel ) pButt_sel->setSel(false); pButt_sel = pButt_mse; }// old position
        if( persist < 3 ) setSel(false);// new
        return true;
    }

    return false;
}

/*
bool buttonList::hitRight()
{
    if( openOnRight && mseDnRt && hit() ) setSel( !sel );// new
    if( sel && pButt_mse ) return pButt_mse->hitRight();
    return false;
}   */

/*
bool buttonList::hitList()
{
    if( sel && pButt_mse )// all open
    {
        for( auto& pButt : pButtVec )
            if( pButt->sel && pButt != pButt_mse ) pButt->setSel(false);
        pButt_mse->hitLeft();
        if( persist < 3 ) setSel(false);// new
        return true;
    }

    return false;
}   */


/*
bool buttonList::hitLeft()
{
    bool Hit = false;

    if( !openOnRight ) Hit = hitList();

    if( hit() )
    {
        if( mseDnLt )
        {
            if( !openOnRight ) setSel( !sel );// new
            if( sel ) { if( pHitFunc ) pHitFunc(); }
            else if( pHitFunc_up ) pHitFunc_up();
        }

        Hit = true;
    }

    return Hit;
}

bool buttonList::hitList()
{
    bool Hit = false;

    if( hit() && mseDnLt )
    {
        setSel( !sel );
        Hit = true;
    }

    if( sel && pButt_mse )// all open
    {
        for( auto& pButt : pButtVec )
            if( pButt->sel && pButt != pButt_mse ) pButt->setSel(false);
        pButt_mse->hitLeft();
        Hit = true;
        if( persist < 3 ) setSel(false);// new
    }

    return Hit;
}   */

/*
void buttonList::closeList()
{
    sel = false;
    pButt_mse = nullptr;
    for( auto& pButt : pButtVec )
    {
        pButt->mseOver = false;
        pButt->setSel( false );
    }
}
*/

