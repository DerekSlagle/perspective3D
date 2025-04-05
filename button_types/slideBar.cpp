#include "slideBar.h"

float slideBar::grabX = 0.0f;

void slideBar::init( std::istream& is, std::function<void(float)> p_SetFunc )
{
    std::string name; is >> name;
    float x, y, W, H, val_min, val_max, init_val, Wbutt;
    is >> x >> y >> W >> H >> val_min >> val_max >> init_val >> Wbutt;
    sf::Text Title;
    Title.setFont( *button::pFont );
    Title.setCharacterSize(12);
    Title.setString( name.c_str() );
    Title.setFillColor( sf::Color::White );

    init( x, y, W, H, Title, val_min, val_max, init_val, p_SetFunc , Wbutt );
}

void slideBar::init( float x, float y, float W, float H, const sf::Text& Title, float val_min, float val_max, float init_val, std::function<void(float)> p_SetFunc, float Wbutt )
{
    pSetFunc = p_SetFunc;
    init( x, y, W, H, Title, val_min, val_max, init_val, nullptr, Wbutt );
}

void slideBar::init( float x, float y, float W, float H, const sf::Text& Title, float val_min, float val_max, float init_val, float* p_val, float Wbutt )
{
     mseOver = sel = false;
    pos.x = x; pos.y = y;
    // buttonRect
    sz.x = W; sz.y = H;
    isHoriz = W >= H;

    buttonRect::init( x, y, W, H, nullptr, Title );
    sz = R.getSize();
    title = Title;
    sf::FloatRect titleSz = title.getLocalBounds();
    sf::Vector2f dPos;
    dPos.x = (sz.x - titleSz.width)/2.0f;
 //   dPos.y = (sz.y + titleSz.height)/2.0f;
 //   dPos.y = -2.0f*titleSz.height;
    dPos.y = sz.y + 2.0f;
    sf::Vector2f tPos = sf::Vector2f( floor(pos.x+dPos.x), floor(pos.y+dPos.y) );
 //   title.setPosition(pos+dPos);
    title.setPosition(tPos);
    title.setCharacterSize( 14 );
    title.setFillColor( sf::Color::White );

    // finally
    pVal = p_val;
    fVal_min = val_min;
    fVal_max = val_max;
    fVal = iVal = init_val;

    // the slide button
    float hButt = sz.y - 2.0f*b;
    if( Wbutt < 0.0f ) wButt = 2.0f*hButt;
    else wButt = Wbutt;

    if( !isHoriz )
    {
        wButt = sz.x - 2.0f*b;
        hButt = Wbutt < 0.0f ? 2.0f*wButt : Wbutt;
    }

    float capLen = 0.0f;//b + wButt/2.0f;
    if( isHoriz ){ capLen = b + wButt/2.0f; L = sz.x - 2.0f*capLen; }
    else { capLen = b + hButt/2.0f; L = sz.y - 2.0f*capLen; }
    float s = L*( fVal - fVal_min )/( fVal_max - fVal_min );
    s += b;//capLen;// rel to left end

    sldButt.setSize( sf::Vector2f(wButt,hButt) );
    if( isHoriz ) sldButt.setPosition( pos.x + s, pos.y + b );// new
    else sldButt.setPosition( pos.x + b, pos.y + s );// new
    sldButt.setOutlineThickness( 2.0f );
//    sldButt.setFillColor( sf::Color::White );
    sldButt.setFillColor( sf::Color(181,181,181) );
    sldButt.setOutlineColor( sf::Color::Blue );

    // the value box
 //   float boxGap = 2.0f;
    float x0 =  pos.x + sz.x + 3.0f;
    float y0 =  pos.y;
    float boxDim = sz.y - 2.0f;
    valBox[0].position.x = x0; valBox[0].position.y = y0;// up lt
    valBox[1].position.x = x0 + 2.5f*boxDim; valBox[1].position.y = y0;// up rt
    valBox[2].position.x = x0 + 2.5f*boxDim; valBox[2].position.y = y0 + boxDim;// dn rt
    valBox[3].position.x = x0; valBox[3].position.y = y0 + boxDim;// dn lt
    valBox[4].position.x = x0; valBox[4].position.y = y0;// up lt
    valBox[0].color = valBox[1].color = valBox[2].color = valBox[3].color = valBox[4].color = sf::Color::Green;

    // the 3 texts
    txtVal.setFont( *button::pFont );
    txtVal.setCharacterSize( 12 );
    sf::Vector2f POS = pos;
    POS.x += sz.x + 5.0f; POS.y += 2.0f;// right end
    if( !isHoriz ) POS.y += 20.0f;

    tPos.x = floor(POS.x); tPos.y = floor(POS.y);
    txtVal.setPosition(tPos);
    float_to_Text( txtVal, fVal );
    txtVal.setFillColor( sf::Color::Green );

    txtVal_min = txtVal_max = txtVal;
    if( isHoriz ){ POS = pos; POS.y += sz.y; }
    else { POS = pos; POS.x += sz.x + 2.0f; }

    tPos.x = floor(POS.x); tPos.y = floor(POS.y);
    txtVal_min.setPosition(tPos);
    float_to_Text( txtVal_min, fVal_min );
    txtVal_min.setFillColor( sf::Color::White );
    txtVal_min.setCharacterSize(12);

    tPos.x = floor(pos.x + sz.x); tPos.y = floor(pos.y + sz.y);
    txtVal_max.setPosition(tPos);
 //   maxValStr.setPosition(pos+sz);
    float_to_Text( txtVal_max, fVal_max );
    txtVal_max.setFillColor( sf::Color::White );
    txtVal_max.setCharacterSize(12);
}

bool slideBar::hitLeft()
{
    if( !hitR( sldButt ) ) { sel = false; return false; }

    if( button::mseDnLt )// grab
    {
        float x = 0.0f;
        sel = true;
    //    std::cerr << "\n sel = true";
        if( isHoriz )
        {
            grabX = button::mseX - sldButt.getPosition().x;
            x = button::mseX - grabX - pos.x - b;
        }
        else
        {
            grabX = button::mseY - sldButt.getPosition().y;
            x = button::mseY - grabX - pos.y - b;
        }

        fVal = (x/L)*( fVal_max - fVal_min) + fVal_min;
        float_to_Text( txtVal, fVal );
        if( pVal ) *pVal = fVal;
        else if( pSetFunc ) pSetFunc(fVal);
        return true;
    }
    else// release
    {
     //   std::cerr << "\n sel = false";
        sel = false;
        return true;
    }

    return false;
}

bool slideBar::hit()const// NEW to support wide hold
{
    if( sel )// button being dragged
    {
        if( isHoriz )
        {
            if( mseX < pos.x || mseX > pos.x+sz.x ) return false;// ends
            if( mseY < pos.y - (grabFactor-1.0f)*sz.y || mseY > pos.y + grabFactor*sz.y ) return false;
        }
        else// vertical slider
        {
            if( mseY < pos.y || mseY > pos.y+sz.y ) return false;// ends
            if( mseX < pos.x - (grabFactor-1.0f)*sz.x || mseX > pos.x + grabFactor*sz.x ) return false;
        }

        return true;
    }

    return buttonRect::hit();
}

bool slideBar::MseOver()
{
    if( hit() ) mseOver = true;
    else mseOver = false;

    R.setOutlineColor( (mseOver ? moClr : unMoClr ) );

    // xcode
    static bool selLast = false;
    if( selLast && !sel && springBack ) set_fVal(iVal);
    selLast = sel;

 //   if( button::didScroll && mseOver && !sel )// fine tune value while mouse over
  //  {
 //       float newVal = fVal + 0.001f*( fVal_max - fVal_min)*button::scrollAmount;
 //       set_fVal( newVal );
 //   }
    if( !springBack ) scrollTune();

    if( !mseDnLt ) { sel = false; return mseOver; }

    if( mseOver )
    {
        if( sel )
        {
            float x = 0.0f;
            float newPosx = button::mseX - grabX, newPosy = pos.y + b;// isHoriz true presumed
            if( !isHoriz ) { newPosy = button::mseY - grabX; newPosx = pos.x + b; }
            // then correct if past ends
            if( isHoriz )
            {
                if( newPosx < pos.x + b )
                {
                    newPosx = pos.x + b;
                    grabX = button::mseX - newPosx;
                }
                else if( newPosx > pos.x + b + L )
                {
                    newPosx = pos.x + b + L;
                    grabX = button::mseX - newPosx;
                }

                x = button::mseX - grabX - pos.x - b;
           //     std::cout << "\n newPosx = " << newPosx << " grabX = " << grabX;
            //    std::cout << "\n mseX = " << button::mseX;
            }
            else
            {
                if( newPosy < pos.y + b )
                {
                    newPosy = pos.y + b;
                    grabX = button::mseY - newPosy;
                }
                else if( newPosy > pos.y + b + L )
                {
                    newPosy = pos.y + b + L;
                    grabX = button::mseY - newPosy;
                }

                x = button::mseY - grabX - pos.y - b;
            //    std::cout << "\n newPosy = " << newPosy << " grabX = " << grabX;
            //    std::cout << "\n mseY = " << button::mseY;
            }

            sldButt.setPosition( newPosx, newPosy );// new
            fVal = (x/L)*( fVal_max - fVal_min) + fVal_min;
            float_to_Text( txtVal, fVal );
            if( pVal ) *pVal = fVal;
           else if( pSetFunc ) pSetFunc(fVal);
        }

        return true;
    }

    sel = false;
    return false;
}

/*
bool slideBar::MseOver()
{
    if( !mseDnLt )
    {
        sel = false;
        return false;
    }

    if( hit() )
    {
        mseOver = true;
    //    button::pButtMse = this;

        if( sel )
        {
            float x = 0.0f;
            float newPosx = button::mseX - grabX, newPosy = pos.y + b;// isHoriz true presumed
            if( !isHoriz ) { newPosy = button::mseY - grabX; newPosx = pos.x + b; }
            // then correct if past ends
            if( isHoriz )
            {
                if( newPosx < pos.x + b )
                {
                    newPosx = pos.x + b;
                    grabX = button::mseX - newPosx;
                }
                else if( newPosx > pos.x + b + L )
                {
                    newPosx = pos.x + b + L;
                    grabX = button::mseX - newPosx;
                }

                x = button::mseX - grabX - pos.x - b;
           //     std::cout << "\n newPosx = " << newPosx << " grabX = " << grabX;
            //    std::cout << "\n mseX = " << button::mseX;
            }
            else
            {
                if( newPosy < pos.y + b )
                {
                    newPosy = pos.y + b;
                    grabX = button::mseY - newPosy;
                }
                else if( newPosy > pos.y + b + L )
                {
                    newPosy = pos.y + b + L;
                    grabX = button::mseY - newPosy;
                }

                x = button::mseY - grabX - pos.y - b;
            //    std::cout << "\n newPosy = " << newPosy << " grabX = " << grabX;
            //    std::cout << "\n mseY = " << button::mseY;
            }

            sldButt.setPosition( newPosx, newPosy );// new
            fVal = (x/L)*( fVal_max - fVal_min) + fVal_min;
            float_to_Text( valStr, fVal );
            if( pVal ) *pVal = fVal;
           else if( pSetFunc ) pSetFunc(fVal);
        }

        return true;
    }

    sel = false;
    return false;
}   */


void slideBar::set_fVal( float f_val )
{
    if( f_val < fVal_min ) f_val = fVal_min;
    if( f_val > fVal_max ) f_val = fVal_max;
    fVal = f_val;
    float_to_Text( txtVal, fVal );
    if( pVal ) *pVal = fVal;
    else if( pSetFunc ) pSetFunc(fVal);
    float x = L*( fVal - fVal_min )/( fVal_max - fVal_min);

    float newPosx = pos.x + b + x, newPosy = pos.y + b;// isHoriz true presumed
    if( !isHoriz ) { newPosy = pos.y + b + x; newPosx = pos.x + b; }
    sldButt.setPosition( newPosx, newPosy );
}

void slideBar::draw( sf::RenderTarget& rRW )const
{
    buttonRect::draw(rRW);
    rRW.draw( sldButt );
    if( showNums )
    {
        rRW.draw( valBox, 5, sf::LinesStrip );
        rRW.draw( txtVal );
        rRW.draw( txtVal_min );
        rRW.draw( txtVal_max );
    }
}

void slideBar::setPosition( sf::Vector2f Pos )
{
    sf::Vector2f dPos = Pos - pos;
    for(size_t i=0; i<5; ++i) valBox[i].position += dPos;
    sldButt.setPosition( sldButt.getPosition() + dPos );
    txtVal.setPosition( txtVal.getPosition() + dPos );
    txtVal_min.setPosition( txtVal_min.getPosition() + dPos );
    txtVal_max.setPosition( txtVal_max.getPosition() + dPos );
    buttonRect::setPosition( Pos );
}

void slideBar::reInit( std::string label, float Min, float Max, float Curr, float* p_val )
{
    pVal = p_val;
    title.setString( label.c_str() );
    fVal_min = Min;
    to_SF_string( txtVal_min, fVal_min );
    fVal_max = Max;
    to_SF_string( txtVal_max, fVal_max );
    set_fVal( Curr );
}

// non members
float float_to_Text(sf::Text& strArg, float x)// returns position of end of strArg
{
    std::stringstream ss;
    ss.precision(4);
    ss << x;
    std::string tempStr;
    ss >> tempStr;
    strArg.setString( tempStr.c_str() );
    sf::FloatRect fr = strArg.getLocalBounds();
    return fr.left + fr.width;
}

bool hitR( const sf::RectangleShape& RS )
{
    sf::Vector2f sbSz = RS.getSize();
    sf::Vector2f sbPos = RS.getPosition();
    if(  button::mseX < sbPos.x || button::mseX > sbPos.x + sbSz.x ) return false;// off end of sldButt
    else if( button::mseY < sbPos.y || button::mseY > sbPos.y + sbSz.y ) return false;
    return true;
}
