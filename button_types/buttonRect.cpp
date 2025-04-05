#include "buttonRect.h"

sf::Color buttonRect::selClr = sf::Color::White, buttonRect::unSelClr = sf::Color(128,128,128);
sf::Color buttonRect::moClr = sf::Color::Green, buttonRect::unMoClr = sf::Color::Blue;

buttonRect::buttonRect(): sz(10.0f,10.0f)
{
    //ctor
}

buttonRect::~buttonRect()
{
    //dtor
}

bool buttonRect::hit()const
{
    if( mseX < pos.x || mseX > pos.x+sz.x ) return false;
    if( mseY < pos.y || mseY > pos.y+sz.y ) return false;
    return true;
}

void buttonRect::draw( sf::RenderTarget& rRW )const
{
    rRW.draw( R );
    rRW.draw(title);
}

void buttonRect::setPosition( sf::Vector2f Pos )
{
    sf::Vector2f dPos = Pos - pos;
    pos = Pos;
    title.setPosition( title.getPosition() + dPos );
    R.setPosition(pos);
}

bool buttonRect::MseOver()
{
    button::MseOver();
    R.setOutlineColor( (mseOver ? moClr : unMoClr ) );
    return mseOver;
}

bool buttonRect::hitLeft()
{
    if( mode == 'T' && !button::mseDnLt ) return false;
    bool selOld = sel;
    button::hitLeft();
    if( sel != selOld ) R.setFillColor( (sel ? selClr : unSelClr ) );
    return mseOver;
}

void buttonRect::setSel( bool Sel )
{
    button::setSel( Sel );
    R.setFillColor( (sel ? selClr : unSelClr ) );
}

void buttonRect::init( float x, float y, float W, float H, std::function<void(void)> p_HitFunc, const char* label )
{
    sf::Text Title( label, *button::pFont, 12 );
    Title.setFillColor( sf::Color::Black );
    init( x, y, W, H, p_HitFunc, Title );
}

void buttonRect::init( float x, float y, float W, float H, std::function<void(void)> p_HitFunc, const sf::Text& Title )
{
    pos.x = x;
    pos.y = y;
    sz.x = W; sz.y = H;
    pHitFunc = p_HitFunc;

    title = Title;
    sf::FloatRect titleSz = title.getLocalBounds();
    sf::Vector2f dPos;
    dPos.x = (sz.x - titleSz.width)/2.0f;
    dPos.y = (sz.y - titleSz.height)/2.0f;
    dPos += pos;// new
    dPos.x = floor( dPos.x );// new
    dPos.y = floor( dPos.y - 2.0f );// new
    title.setPosition(dPos);

    R.setSize( sf::Vector2f(W,H) );
    R.setPosition( pos );
    R.setOutlineThickness( -3.0f );
    R.setFillColor( (sel ? selClr : unSelClr) );
    R.setOutlineColor( (mseOver ? moClr : unMoClr) );
}

void buttonRect::init( std::istream& is, std::function<void(void)> p_HitFunc )
{
    std::string Name; is >> Name;
    float posX, posY, W, H; is >> posX >> posY >> W >> H;
    init( posX, posY, W, H, p_HitFunc, Name.c_str() );
}
