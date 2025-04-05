#ifndef BUTTONLIST_H_INCLUDED
#define BUTTONLIST_H_INCLUDED

#include<vector>
#include "buttonRect.h"

class buttonList  : public buttonRect
{
    public:
    std::vector<button*> pButtVec;
    std::function<void(void)> pHitFunc_up = nullptr;// new call function on left button release = pHitFunc by default
    bool ownButts = false;
    button* pButt_mse = nullptr;
    button* pButt_sel = nullptr;// selected button on list
    bool openOnRight = false;
 //   bool openOnHover = false;
    int persist = 1;// 1 = close on miss, 2 = close on hit, 3 = close only on hit the open/close button
    // funcs
    buttonList() {}
    virtual ~buttonList();
    void init( float x, float y, float W, float H, const sf::Text& Title = sf::Text() );
    buttonList( float x, float y, float W, float H, const sf::Text& Title = sf::Text() ) { init( x, y, W, H, Title ); }
    // non owner of buttons
    void init( float x, float y, float W, float H, float dy, const std::vector<sf::Text>& TitleVec,
        const std::vector< std::function<void(void)> > pFuncVec, std::vector<buttonRect>& recButtVec, char side = 'B' );
    // owner of buttons
    void init( float x, float y, float W, float H, float dy, const std::vector<sf::Text>& TitleVec,
    const std::vector< std::function<void(void)> > pFuncVec, char side = 'B' );

    void openList();
    void closeList();

    virtual void draw( sf::RenderTarget& rRW )const;
    virtual bool MseOver();// assigns mseOver - virtual due to ValOnHit live display feature
    virtual bool hitLeft();
    virtual bool hitRight();
    virtual void init_delayBox();
    virtual void setSel( bool Sel );// over in buttonList to open/close list
    virtual void update();
    virtual void setPosition( sf::Vector2f Pos );

    bool hitList();
//    virtual void setPosition( sf::Vector2f Pos );
};


#endif // BUTTONLIST_H_INCLUDED
