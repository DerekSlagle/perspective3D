#ifndef PERSBOX_H
#define PERSBOX_H

#include "persPt.h"


class persBox : public persPt
{
    public:
    vec3f pt[8];// in basis below
 //   vec3f Xu, Yu, Zu;// local coordinate basis
    float hfSzX = 4.0f, hfSzY = 8.0f, hfSzZ = 16.0f;
    sf::VertexArray vtxArr[6];// 1 per face
    unsigned int ptIdx[24];// map vertex to pt
    bool doDrawFace[6];
    // ** face order is: right(+X), left(-X), top(+Y), bot(-Y), front(+Z), back(-Z) **
    // ie.  rt, lt, tp, bt, ft, bk
    void init_ptIdx();// fixed order
    void assignPtPos();// fixed order
    void assignVtxPos();// fixed order

    spriteSheet* pSS = nullptr;
    sf::Color faceColor[6];
    void updateShading( vec3f lightDir );// vertex.color assigned

    // change orientation of basis
    void pitch( float dAngle );
    void yaw( float dAngle );
    void roll( float dAngle );

    virtual void update( float dt );
    virtual void draw( sf::RenderTarget& RT ) const;
    virtual void setPosition( vec3f Pos );

    void init( std::istream& is, spriteSheet* p_SS = nullptr );
    persBox( std::istream& is, spriteSheet* p_SS = nullptr ){ init( is, p_SS ); }
    persBox();
    virtual ~persBox();

    protected:

    private:
};

#endif // PERSBOX_H
