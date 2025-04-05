#ifndef PERSPT_H
#define PERSPT_H

#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>

#include <vector>
#include <fstream>
#include <functional>
#include "../utility/vec2f.h"
#include "../utility/spriteSheet.h"

class persPt
{
    public:
    static float X0, Yh;// view point in window
    static float Z0;// for perspective scaling of x and y coordinates
    static vec3f camPos;// camera position in scene
    static vec3f zuCam;// direction pointed
    static vec3f yuCam;    // unit vector "up" (camera orientation)
    static vec3f xuCam;    // unit vector "to right" : xu = camDir.cross( yu ) maintain in update()
    static const vec3f xHat, yHat, zHat;// these form the "world" or "global" coordinate vector basis
 //   const vec3f yHat(0.0f,1.0f,0.0f);
 //   const vec3f zHat(0.0f,0.0f,1.0f);
    static float angle;// yaw
    static float pitchAngle;
    static float rollAngle;

    static sf::Vector2f get_xyw( vec3f Pos );
    static float set_rw( vec3f Pos, float R, sf::CircleShape& rCS );// utility. Returns scaled value of R
    static float changeCamDir( float dAy, float dAp, float dAr );// returns compass heading
    static float changeCamDir_mat( float dAy, float dAp, float dAr );// uses a matrix
    static void calibrateCameraAxes();// maintenance
    static void sortByDistance( std::vector<persPt*>& pPtVec );// this hand rolled sort function is no longer used
    static void sortByDistance( std::vector<persPt*>& pPtVec, size_t k );// sort only 1st k elements
    static bool init_stat( std::istream& is );// initialize static members

    static float zScale( vec3f P );// Z0/Z

    // automate camera motion
    static void updatePOI( float R, float angleSpeed, float dt );
    static void pitch( float dAngle );
    static void yaw( float dAngle );
    static void roll( float dAngle );
    static void keepXuLevel();// counter roll induced by pitching and yawing
    // update typical background: horizon line, skyQuad, groundQuad (if one)
    static void updateBackground( sf::Vertex* pHzLine, sf::Vertex* skyQuad, sf::Vertex* groundQuad );

    vec3f pos;// position in scene
    vec3f vel;
    vec3f Xu, Yu, Zu;// coordinate basis per instance
    float Rbound = 1.0f;// boundary in front of camera. Closer = don't draw
    bool doDraw = true;// calculate in update() and use in zSort() and draw()
    bool isMoving = false;// no need to update position of object while off screen or behind camera
    bool inUse = true;

    void init( vec3f Pos ){ pos = Pos; }
    sf::Vector2f get_xyw()const { return persPt::get_xyw( pos ); }
    float getDistance()const{ return ( pos - persPt::camPos ).mag(); }
    virtual void setPosition( vec3f Pos ){ pos = Pos; }
    void setPosition( float X, float Y, float Z ) { setPosition( vec3f(X,Y,Z) ); }
    virtual void update( float dt ) = 0;
    virtual void draw( sf::RenderTarget& RT ) const = 0;
    virtual void update_doDraw();
    virtual bool hit( vec3f posA, vec3f posB, vec3f& P, vec3f& vu )const;// also persQuad, persBox_quad, persMat
    virtual bool hit_image( float xHit, float yHit )const;// also persQuad
    bool isSighted()const;// in line with camDir?


    bool operator<( const persPt& rPt )const{ return getDistance() < rPt.getDistance(); }
    static bool compare( const persPt* pPtA, const persPt* pPtB ) { return *pPtB < *pPtA; }// for std::sort() on a container of pointers. This is operator <

    persPt(){ Xu = xHat; Yu = yHat; Zu = zHat; }
    virtual ~persPt(){}
};

class persBall : public persPt
{
public:
    sf::CircleShape B;
    float mass = 1.0f;

    void reset();
    void reset( vec3f Pos, vec3f Vel );

    void update_rw();
    bool hitFixed( vec3f posA, float r1, vec3f posB, vec3f& P, vec3f& vu, float Cr = 1.0f )const;// *this is stationary
    bool hitFree( persBall& rB, float Cr, float dt );// both moving or 1 fixed (calls above). Collision is handled within function
    static unsigned int hitAll( std::vector<persBall>& PBvec, float Cr, float dt );// mutual collision handling. Returns # of collisions
    bool isSighted( vec3f& hitPt, float R = 0.0f )const;
    virtual void init( std::istream& is );
    void init( vec3f Pos, float R0, sf::Color color, vec3f Vel = vec3f() );
    virtual void setPosition( vec3f Pos );
    void setPosition( float X, float Y, float Z ) { setPosition( vec3f(X,Y,Z) ); }
    virtual void update( float dt );
    virtual void update( vec3f grav, float dt );// other version in shotBurst, persBallTimed, which are derived from persBall
    virtual void draw( sf::RenderTarget& RT ) const;
    persBall(){}
    persBall( std::istream& is ){ init(is); }
    persBall( vec3f Pos, float R0, sf::Color color, vec3f Vel = vec3f() ){ init( Pos, R0, color, Vel ); }
    virtual ~persBall(){}
};

class persBallTimed : public persBall
{
public:
    static float tLife;
    float tElap = 0.0f;
    static std::function<bool(vec3f)> pFdone;

    virtual void update( float dt );
    virtual void update( vec3f grav, float dt );

    virtual void init( std::istream& is );
    persBallTimed(){}
    persBallTimed( std::istream& is ){ init(is); }
    virtual ~persBallTimed(){}
};

class persQuad : public persPt
{
    public:// zero encapsulation discipline on display

    // data members
    vec3f pt[4];
 //   vec3f Nu;// unit vector perpendicular to Quad
    sf::Vertex vtx[4];
    const sf::Texture* pTxt = nullptr;
    float w, h;// width and height of the quad
    bool facingCamera = false;// enables "billboarding"

    // function members
    vec3f getPosition()const{ return ( pt[0] + pt[2] )/2.0f; }
    virtual void update( float dt );
    virtual void draw( sf::RenderTarget& RT ) const;
    virtual void setPosition( vec3f Pos );
    void setNu( vec3f nu );// to default orientation: w side is horizontal, quad is tilted upwards
    void setOrientation( vec3f nu, vec3f Tu );// general orientation: pt[0], pt[1] toward Tu

    // Ta: 'R' = rotate 90 degrees, Tb = 0, 1, 2, 3 = # times
    // Ta = 'F' = flip image around Tb = 'X' or 'Y' axis
    void setTxtRect( sf::IntRect srcRect, char Ta, char Tb );// assigns texCoords with some mapping variability
    void flip( char XorY );// sometimes the required orientation needs a flip only

    // returns true when hit and writes collision point = P and unit vec3f = vu in reflected direction
    virtual bool hit( vec3f posA, vec3f posB, vec3f& P, vec3f& vu )const;// also persBox_quad
    virtual bool hit_image( float xHit, float yHit )const;
 //   bool hit( vec3f posA, vec3f posB, vec3f& P, vec3f& vu, float b = 2.0f )const;// also persBox_quad
 //   bool hitSlow( vec3f posA, vec3f posB, vec3f& P, vec3f& vu, float b = 2.0f )const;// also persBox_quad
    bool isSighted( float& dist )const;// true if aimed at. Writes distance to aim point on persQuad
    bool isSighted( float& dist, vec3f& hitPt )const;// true if aimed at. Writes distance to aim point on persQuad
    void setColor( sf::Color color );

    persQuad(){}
    virtual ~persQuad(){}

    void init( std::istream& is, sf::Texture* p_Txt = nullptr );
    persQuad( std::istream& is, sf::Texture* p_Txt = nullptr ){ init( is, p_Txt ); }

    void init( vec3f Pos, float W, float H, vec3f nu, sf::Color color, const sf::Texture* p_Txt = nullptr );
    persQuad( vec3f Pos, float W, float H, vec3f nu, sf::Color color, sf::Texture* p_Txt = nullptr ){ init( Pos, W, H, nu, color, p_Txt ); }
};

class persQuadAni : public persQuad
{
    public:
    static bool getOne( std::vector<persQuadAni>& PQAvec, vec3f Pos );
    spriteSheet* pSS = nullptr;
    size_t setNum = 0, frIdx = 0;
    int numLoops = -1, loopCount = 0;// -1 = repeat forever. numLoops > 0 loop numLoops times
 //   bool inUse = false;// until assigned

    void reset();
    void init( std::istream& is, spriteSheet& rSS );
    void init( vec3f Pos, float W, float H, vec3f nu, sf::Color color, spriteSheet& rSS, size_t SetNum, int NumLoops );
    virtual void update( float dt );
    virtual void draw( sf::RenderTarget& RT ) const;
    persQuadAni(){}
    virtual ~persQuadAni(){}
    persQuadAni( std::istream& is, spriteSheet& rSS )
    { init( is, rSS ); }
};

#endif // PERSPT_H
