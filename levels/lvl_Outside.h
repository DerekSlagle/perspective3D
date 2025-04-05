#ifndef LVL_OUTSIDE_H
#define LVL_OUTSIDE_H

#include "lvl_3D.h"

#include "../button_types/joyButton.h"
#include "../button_types/pushButton.h"
#include "../button_types/buttonValOnHit.h"
#include "../button_types/controlSurface.h"

#include "../perspective_types/persBox.h"

class lvl_Outside : public lvl_3D
{
    public:
    bool keyDown_Lshift = false, keyDown_Rshift = false;

    virtual bool init();
    virtual bool handleEvent( sf::Event& rEvent );
    virtual void update( float dt );
    virtual void draw( sf::RenderTarget& RT ) const;

    std::vector<spriteSheet> SSvec;
    // persPt types
    std::vector<persQuad> PQvec;
    std::vector<persBall> ballVec;
    bool init_terrain( const char* fileName );

    std::vector<persBox> boxVec;
    std::vector<float> boxRotSpeed;// signed
    std::vector<char> boxRotMode;// yaw = 'Y', pitch = 'P', roll = 'R'
    bool init_boxes( const char* fileName );

    // controls
    joyButton jbCamButt;// navigation
    float camVelXZscale = 0.0013f;
    bool init_controls( const char* fileName );

    lvl_Outside();
    virtual ~lvl_Outside();

    protected:

    private:
};

#endif // LVL_OUTSIDE_H
