#include "persBox.h"

persBox::persBox()
{
    //ctor
}

persBox::~persBox()
{
    //dtor
}

void persBox::init_ptIdx()
{
    // Z out    ^
    //          Y
    //  < X   0   1    4   5
    //        3   2    7   6
    //        front    back

    // base for this side
    unsigned int* bIdx = ptIdx;// right
    bIdx[0] = 4; bIdx[1] = 0;// as sf::Quad
    bIdx[3] = 7; bIdx[2] = 3;// swap these for 2*sf::TriangleStrip
    bIdx += 4;// left
    bIdx[0] = 5; bIdx[1] = 1;// as sf::Quad
    bIdx[3] = 6; bIdx[2] = 2;
    bIdx += 4;// top
    bIdx[0] = 1; bIdx[1] = 0;// as sf::Quad
    bIdx[3] = 5; bIdx[2] = 4;
    bIdx += 4;// bottom
    bIdx[0] = 2; bIdx[1] = 3;// as sf::Quad
    bIdx[3] = 6; bIdx[2] = 7;
    bIdx += 4;// front
    bIdx[0] = 0; bIdx[1] = 1;// as sf::Quad
    bIdx[3] = 3; bIdx[2] = 2;
    bIdx += 4;// back
    bIdx[0] = 5; bIdx[1] = 4;// as sf::Quad
    bIdx[3] = 6; bIdx[2] = 7;
}

void persBox::assignPtPos()// fixed order
{
    pt[0] = pt[4] = vec3f( hfSzX, hfSzY, hfSzZ );
    pt[1] = pt[5] = vec3f( -hfSzX, hfSzY, hfSzZ );
    pt[2] = pt[6] = vec3f( -hfSzX, -hfSzY, hfSzZ );
    pt[3] = pt[7] = vec3f( hfSzX, -hfSzY, hfSzZ );
    for( unsigned j = 4; j < 8; ++j )
        pt[j].z -= 2.0f*hfSzZ;
}

void persBox::init( std::istream& is, spriteSheet* p_SS )
{
    is >> pos.x >> pos.y >> pos.z;
    float yawAngle = 0.0f, pitchAngle = 0.0f; is >> yawAngle >> pitchAngle;
    is >> hfSzX >> hfSzY >> hfSzZ;
    Rbound = sqrtf( hfSzX*hfSzX + hfSzY*hfSzY + hfSzZ*hfSzZ );
    unsigned int rd, gn, bu; is >> rd >> gn >> bu;
    faceColor[0] = faceColor[1] = faceColor[2] = faceColor[3] = faceColor[4] = faceColor[5] = sf::Color(rd,gn,bu);

    // given yaw and pitch
    Xu = persPt::xHat;
    Yu = persPt::yHat;
    Zu = persPt::zHat;

    if( yawAngle != 0.0f )
    {
        yawAngle *= vec2f::PI/180.0f;
        yaw( yawAngle );
    }

    if( pitchAngle != 0.0f )
    {
        pitchAngle *= vec2f::PI/180.0f;
        pitch( pitchAngle );
    }

    init_ptIdx();
    assignPtPos();

    pSS = p_SS;
    if( pSS )
    {
        unsigned int SetNum; is >> SetNum;
        std::cout << "\n SetNum: " << SetNum << " FrIdx: ";
        if( SetNum >= pSS->numSets() ) SetNum = pSS->numSets() - 1;

        for( unsigned int j = 0; j < 6; ++j )
        {
            unsigned int FrIdx; is >> FrIdx;
            int Tr; is >> Tr;
            char Tf; is >> Tf;
            vtxArr[j].resize(4);
            vtxArr[j].setPrimitiveType( sf::Quads );
            vtxArr[j][0].color = faceColor[j];
            vtxArr[j][1].color = faceColor[j];
            vtxArr[j][2].color = faceColor[j];
            vtxArr[j][3].color = faceColor[j];
            std::cout << ' ' << FrIdx;
            pSS->setTxtRect( vtxArr[j], FrIdx, SetNum, Tr, Tf );
        }

    }
    else// separate face colors
    {
        for( unsigned int j = 0; j < 6; ++j )
        {
            is >> rd >> gn >> bu;
            faceColor[j] = sf::Color(rd,gn,bu);
        }
    }

    for( unsigned j = 0; j < 6; ++j )// each face
    {
        vtxArr[j].resize(4);
        vtxArr[j].setPrimitiveType( sf::Quads );
        vtxArr[j][0].color = faceColor[j];
        vtxArr[j][1].color = faceColor[j];
        vtxArr[j][2].color = faceColor[j];
        vtxArr[j][3].color = faceColor[j];
    }

    update(0.0f);
}

void persBox::pitch( float dAngle )
{
    Zu = Zu*cosf(dAngle) - Yu*sinf(dAngle);// pitch
    Zu /= Zu.mag();
    Yu = Zu.cross( Xu );
}

void persBox::yaw( float dAngle )
{
    Zu = Zu*cosf(dAngle) + Xu*sinf(dAngle);// yaw
    Zu /= Zu.mag();
    Xu = Yu.cross( Zu );
}

void persBox::roll( float dAngle )
{
    Yu = Yu*cosf(dAngle) + Xu*sinf(dAngle);// roll
    Yu /= Yu.mag();
    Xu = Yu.cross( Zu );
}

void persBox::update( float dt )
{
    if( !inUse ) return;
    if( isMoving ) pos += vel*dt;
    persPt::update_doDraw();
    if( !doDraw ) return;

    assignVtxPos();
}

void persBox::assignVtxPos()// fixed order
{
    const vec3f N[] = { Xu, -Xu, Yu, -Yu, Zu, -Zu };
    vec3f sep = persPt::camPos - pos;  // from box to camera
    for( unsigned j = 0; j < 6; ++j )// each face
    {
        if( sep.dot( N[j] ) > 0.0f )
        {
            doDrawFace[j] = true;
            for( unsigned k = 0; k < 4; ++k )
            {
                vec3f& Pt = pt[ ptIdx[ 4*j + k ] ];
                vtxArr[j][k].position = persPt::get_xyw( pos + Xu*Pt.x + Yu*Pt.y + Zu*Pt.z );
            }
        }
        else doDrawFace[j] = false;
    }
}

// call after assignVtxPos() lightDir is towards fan
void persBox::updateShading( vec3f lightDir )// vertex.color assigned
{
    const vec3f N[] = { Xu, -Xu, Yu, -Yu, Zu, -Zu };

    float minF = 0.6f, maxF = 1.0f;
 //   vec3f sep = pos - persPt::camPos;// also towards fan
    float colorF = 1.0f;// full

    for( unsigned j = 0; j < 6; ++j )// each face
    {
        if( !doDrawFace[j] ) continue;
        float LtN = N[j].dot( lightDir );
        if( LtN < 0.0f )// this side is lit
        {
            colorF = minF - LtN*( maxF - minF );
        }
        else// side is shaded
        {
             colorF = minF;
        }

        for( unsigned k = 0; k < 4; ++k )// each vertex
        {
            vtxArr[j][k].color.r = static_cast<uint8_t>( colorF*static_cast<float>( faceColor[j].r ) );
            vtxArr[j][k].color.g = static_cast<uint8_t>( colorF*static_cast<float>( faceColor[j].g ) );
            vtxArr[j][k].color.b = static_cast<uint8_t>( colorF*static_cast<float>( faceColor[j].b ) );
        }
    }

}

void persBox::draw( sf::RenderTarget& RT ) const
{
    if( !( inUse && doDraw ) ) return;

    if( pSS )
    {
        for( unsigned j = 0; j < 6; ++j )// each face
            if( doDrawFace[j] ) RT.draw( vtxArr[j], &( pSS->txt ) );
    }
    else
    {
        for( unsigned j = 0; j < 6; ++j )// each face
            if( doDrawFace[j] ) RT.draw( vtxArr[j] );
    }


}

void persBox::setPosition( vec3f Pos )
{
    pos = Pos;
}
