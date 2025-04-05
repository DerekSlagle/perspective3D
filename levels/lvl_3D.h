#ifndef LVL_3D_H
#define LVL_3D_H

#include<algorithm>

#include "Level.h"
#include "../perspective_types/persPt.h"

class lvl_3D : public Level
{
    public:

    bool keyDown_Lshift = false, keyDown_Rshift = false;
    // translational motion
    int move_LtRt = 0, move_UpDown = 0;// -1: Lt, Down +1: Rt Up
    float move_xyVel = 300.0f;// using arrow keys to move in xuCam or yuCam directions

    sf::Vertex viewCross[4];// cross at at persPt::X0, persPt::Yh
    float pipLength = 10.0f;
//    float camVelXZscale = 0.1f;// associates with joystick control
    float camSpeed = 1.0f, yawRate = 1.0f, pitchRate = 1.0f, rollRate = 1.0f;
    float camAccel = 400.0f;
    bool doCoast = true;// accelerate camera vs. move at constant camSpeed

    vec3f gravity;

    sf::Vertex trueHorizon[2];
    sf::Vertex upperPane[4];
 //   void updatePane();// now a static persPt method

    bool viewIsStatic = true;// default. Provide yawRate, pitchRate, camAccel
    bool keepLevel = false;// banking motion is default
    void update_global( float dt );// camera movement
    // messages updates in above
    sf::RectangleShape camDataRect;
    sf::Text camDataMsg;// use newlines etc to label each qty below
    sf::Text camSpeed_Msg, camHeading_Msg, camPitch_Msg;
    sf::Text camPosX_Msg, camPosY_Msg, camPosZ_Msg;

    // base interface
    virtual bool init();
    virtual bool handleEvent( sf::Event& rEvent );
    virtual void update( float dt );
    virtual void draw( sf::RenderTarget& RT ) const;

    void drawBackGround( sf::RenderTarget& RT ) const;
    void drawForeGround( sf::RenderTarget& RT ) const;

    lvl_3D();
    virtual ~lvl_3D();

    protected:
    std::vector<persPt*> pPt_updateVec;
    std::vector<persPt*> pPt_noUpdateVec;

    // for z ordering
 //   unsigned int numToDraw = 0;// varies with camera position and view
    std::vector<const persPt*> pPt_sortedVec;
    void update_zOrder();// numToDraw assigned
    void update_zOrder( const std::vector<const persPt*>& pPersVec );// for derived to pass other in use

    private:
};

#endif // LVL_3D_H
