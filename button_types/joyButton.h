#ifndef JOYBUTTON_H_INCLUDED
#define JOYBUTTON_H_INCLUDED

//#include<functional>
#include "button.h"
#include "../utility/vec2f.h"

class joyButton : public button
{
    public:
 //       static float mseX, mseY;
 //       static bool mseDnLt, mseDnRt;
 //       bool mseOver, sel;
 //       sf::Vector2f pos;
    sf::CircleShape ring, B;
    sf::Vertex vtx[6];// crosshairs + line from ring center to B
    sf::Vector2f Bpos;
    bool atRest = true;
    bool writeOnDrop = false;
    float hwCh = 10.0f;// half width of crosshairs
    float rB = 7.0f, vB = 4.0f;// B radius and fall velocity
    float r_ring = 80.0f;
    bool dropStick = false;

    float *pfA = nullptr, *pfB = nullptr;
    std::function<void(float,float)> pFunc_ff = nullptr;// B.pos - pos
    std::function<void(float)> pFunc_r = nullptr;// B.pos - pos
    std::function<void(float)> pFunc_angle = nullptr;// B.pos - pos
    std::function<void(vec2f)> pFunc_v2d = nullptr;// B.pos - pos
    std::function<void(void)> pFunc = nullptr;// B.pos - pos
    bool *pSel = nullptr;// notifies when mseOver && sel
    bool *pBoolRt = nullptr;//

    // funcs
    void init( float R, float r, float posX, float posY );
    joyButton() {};
    void init( float R, float r, float posX, float posY, std::function<void(void)> p_Func ){ pFunc = p_Func; init(R,r,posX,posY); }
    void init( float R, float r, float posX, float posY, float *pfa, float *pfb ) { pfA = pfa; pfB = pfb; init(R,r,posX,posY); }
    void callBack( float x, float y ) const;

    virtual ~joyButton() {};

    virtual bool hit()const;
    virtual void draw( sf::RenderTarget& rRW )const;
    virtual bool MseOver();// assigns mseOver - virtual due to ValOnHit live display feature
    virtual bool hitRight();// no need for hitLeft though. Base version OK
    virtual bool hitLeft();
    virtual void update();

 //   virtual bool hitLeft();// alters sel - overload in ValOnHit
    virtual void setPosition( sf::Vector2f Pos );
};

#endif // JOYBUTTON_H_INCLUDED
