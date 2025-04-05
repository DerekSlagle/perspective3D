#include "joyButton.h"


void joyButton::init( float R, float r, float posX, float posY )
{
    pos.x = posX; pos.y = posY;
    mseOver = sel = false;

    Bpos = pos;// no separation initially
    vtx[0].position = vtx[1].position = pos;// for center to B line
    vtx[0].color = vtx[1].color = sf::Color::Red;

    vtx[2].position.y = vtx[3].position.y = pos.y;// horiz line
    vtx[2].position.x = pos.x-hwCh; vtx[3].position.x = pos.x+hwCh;
    vtx[4].position.x = vtx[5].position.x = pos.x;// vert line
    vtx[4].position.y = pos.y-hwCh; vtx[5].position.y = pos.y+hwCh;
    vtx[2].color = vtx[3].color = vtx[4].color = vtx[5].color = sf::Color::Black;

    r_ring = R;// new
    ring.setRadius(R);
    ring.setFillColor( sf::Color::White );
    ring.setOutlineColor( sf::Color::Green );
    ring.setOutlineThickness( 4.0f );
    ring.setOrigin( R,R );
    ring.setPosition( posX, posY );
    rB = r;
    B.setRadius(r);
    B.setFillColor( sf::Color::Blue );
    B.setOrigin( r,r );
    B.setPosition( posX, posY );
}

bool joyButton::hit()const
{
    float dx = mseX - pos.x;
    float dy = mseY - pos.y;
    return dx*dx + dy*dy < (r_ring)*(r_ring);
}

void joyButton::draw( sf::RenderTarget& rRW )const
{
    rRW.draw(ring);
    rRW.draw( vtx, 6, sf::Lines );
    rRW.draw(B);
}

bool joyButton::MseOver()// update actually. All happens here
{
    bool Hit = false;

    if( hit() )
    {
        Hit = true;
        if( !mseOver )// assign?
        {
            float dx = mseX - Bpos.x;
            float dy = mseY - Bpos.y;
            mseOver = rB*rB > dx*dx + dy*dy;
        }
    }
    else mseOver = false;

    return Hit;
}

void joyButton::callBack( float x, float y ) const
{
    if( pfA ) *pfA = x;
    if( pfB ) *pfB = y;
    if( pFunc_v2d ) pFunc_v2d( vec2f(x,y) );
    if( pFunc_ff ) pFunc_ff( x, y );
    if( pFunc_r ) pFunc_r( atan2f(y,x) );
}

void joyButton::update()
{
    sf::Vector2f r = Bpos - pos;

    if( mseOver && !dropStick )// over the magnetic button B
    {
        atRest = false;

        Bpos.x = mseX;
        Bpos.y = mseY;
        B.setPosition( Bpos.x, Bpos.y );
        vtx[1].position = Bpos;
        float dx = Bpos.x - pos.x;
        float dy = Bpos.y - pos.y;
        if( dx*dx + dy*dy > rB*rB )// outside dead zone = ball radius
        {
        //    if( pfA ) *pfA = dx;
        //    if( pfB ) *pfB = dy;
        //    if( pFunc_v2d ) pFunc_v2d( vec2f(dx,dy) );
        //    if( pFunc_r ) pFunc_r( atan2f(dy,dx) );
            callBack( dx, dy );
            sel = mseDnLt;
        }
        else{ callBack( 0.0f, 0.0f ); }
    }
    else sel = false;// new


    if( !mseOver && pSel && (sel != *pSel) ) *pSel = sel;// write change if control dropped

    // animation of B falling to center
    if( !( mseOver || atRest ) )// B is falling back to center
    {
        float rMag0 = sqrtf(r.x*r.x + r.y*r.y);
        float rMagf = rMag0 - vB;
        if( rMagf < 0.0f )
        {
            rMagf = 0.0f;
            atRest = true;
            Bpos = pos;
            if( writeOnDrop ) callBack( 0.0f, 0.0f );
        }
        else
        {
            r.x *= rMagf/rMag0;
            r.y *= rMagf/rMag0;
            Bpos = r + pos;
            callBack( r.x, r.y );
        }

        B.setPosition( Bpos.x, Bpos.y );
        vtx[1].position = Bpos;
    }

    return;
}

 bool joyButton::hitLeft()
{
    sel = ( mseDnLt && mseOver );
    if( pSel && (sel != *pSel) ) *pSel = sel;
    return mseOver;
}

bool joyButton::hitRight()
{
    if( mseOver && pBoolRt && clickEvent_Rt() == 1 )
    {
        *pBoolRt = !(*pBoolRt);// toggle value
        return true;
    }
    return false;
}

 //   virtual bool hitLeft();// alters sel - overload in ValOnHit
 void joyButton::setPosition( sf::Vector2f Pos )
 {
    pos = Pos;
    mseOver = sel = false;

    Bpos = pos;// no separation initially
    vtx[0].position = vtx[1].position = pos;// for center to B line

    vtx[2].position.y = vtx[3].position.y = pos.y;// horiz line
    vtx[2].position.x = pos.x-hwCh; vtx[3].position.x = pos.x+hwCh;
    vtx[4].position.x = vtx[5].position.x = pos.x;// vert line
    vtx[4].position.y = pos.y-hwCh; vtx[5].position.y = pos.y+hwCh;

    ring.setPosition( pos.x, pos.y );
    B.setPosition( pos.x, pos.y );
 }
