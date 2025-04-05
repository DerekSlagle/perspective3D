#include "persPt.h"

float persPt::X0 = 1000.0f, persPt::Yh = 300.0f;// view point in window
float persPt::Z0 = 400.0f;// for perspective scaling of x and y coordinates
vec3f persPt::camPos = vec3f(0.0f,0.0f,0.0f);// camera position in scene
vec3f persPt::zuCam = vec3f(0.0f,0.0f,1.0f);// direction pointed
vec3f persPt::yuCam = vec3f(0.0f,1.0f,0.0f);    // unit vector "up" (camera orientation)
vec3f persPt::xuCam = vec3f(1.0f,0.0f,0.0f);    // unit vector "to right"
const vec3f persPt::xHat = persPt::xuCam;
const vec3f persPt::yHat = persPt::yuCam;
const vec3f persPt::zHat = persPt::zuCam;
float persPt::angle = 0.0f;// yaw
float persPt::pitchAngle = 0.0f;// pitch
float persPt::rollAngle = 0.0f;// roll

// static functions
// get the window position from the vec3f position in the scene
sf::Vector2f persPt::get_xyw( vec3f Pos )
{
    vec3f Rcp = Pos - camPos;// from camera to Pos
    float U = Rcp.dot( zuCam );// projection along zuCam = "z - Zcam"
    vec3f Rcp_perp = Rcp - U*zuCam;// component of Rcp in plane perpendicular to zuCam
    vec3f L = ( Z0/U )*Rcp_perp;// perspective transform applied
    if( U < 0.0f ) L *= -1.0f;
//    vec3f L = ( Z0/Rcp.mag() )*Rcp_perp;// perspective transform applied
    return sf::Vector2f( X0 + L.dot( xuCam ), Yh - L.dot( yuCam ) );
}

float persPt::changeCamDir( float dAy, float dAp, float dAr )// returns compass heading
{
    return changeCamDir_mat( dAy, dAp, dAr );// matrix based method

    // this code works fine too
    zuCam = zuCam*cosf(dAy) + xuCam*sinf(dAy);// yaw
    xuCam = yuCam.cross( zuCam );

    zuCam = zuCam*cosf(dAp) - yuCam*sinf(dAp);// pitch
    yuCam = zuCam.cross( xuCam );

    yuCam = yuCam*cosf(dAr) + xuCam*sinf(dAr);// roll
    yuCam /= yuCam.mag();// renormalize: this has proven necessary - NEW
    xuCam = yuCam.cross( zuCam );

    // compass heading
    vec3f Rh = zuCam - zuCam.y*yHat;// component of zuCam in x,z plane
    Rh /= Rh.mag();// normalize
    float a = acosf( Rh.dot( zHat ) );
    if( Rh.dot( xHat ) < 0.0f ) a *= -1.0f;
    return a*180.0f/3.14159f;
}


float persPt::changeCamDir_mat( float dAy, float dAp, float dAr )// returns compass heading
{
    float M[3][3];
    float SnAy = sinf(dAy), CsAy = cosf(dAy);
    float SnAp = sinf(dAp), CsAp = cosf(dAp);
    float SnAr = sinf(dAr), CsAr = cosf(dAr);

    M[0][0] = CsAr*CsAy - SnAr*SnAp*SnAy;    M[0][1] = -SnAr*CsAp;   M[0][2] = -CsAr*SnAy - SnAy*SnAp*SnAr;
    M[1][0] = SnAr*CsAy + SnAy*SnAp*CsAr;    M[1][1] = CsAr*CsAp;    M[1][2] = -SnAr*SnAy + SnAy*SnAp*CsAr;
    M[2][0] = CsAp*SnAy;                     M[2][1] = -SnAp;        M[2][2] = CsAp*CsAy;
    vec3f xcOld = xuCam, ycOld = yuCam, zcOld = zuCam;

    xuCam     = M[0][0]*xcOld + M[0][1]*ycOld + M[0][2]*zcOld;
    yuCam     = M[1][0]*xcOld + M[1][1]*ycOld + M[1][2]*zcOld;
    zuCam = M[2][0]*xcOld + M[2][1]*ycOld + M[2][2]*zcOld;

    // compass heading
    vec3f Rh( zuCam.x, 0.0f, zuCam.z );// = zuCam - zuCam.y*yHat;// component of zuCam in x,z plane
    Rh /= Rh.mag();// normalize
    float a = acosf( Rh.dot( zHat ) );
    if( Rh.dot( xHat ) < 0.0f ) a *= -1.0f;
    return a*180.0f/3.14159f;
}

void persPt::calibrateCameraAxes()
{
    persPt::zuCam /= persPt::zuCam.mag();
    persPt::xuCam = persPt::yuCam.cross( persPt::zuCam );
    persPt::xuCam /= persPt::xuCam.mag();
//    persPt::yuCam = persPt::xuCam.cross( persPt::zuCam );
    persPt::yuCam = persPt::zuCam.cross( persPt::xuCam );
}

void persPt::sortByDistance( std::vector<persPt*>& pPtVec )
{
    for( size_t iterCurr = 0; iterCurr + 1 < pPtVec.size(); ++ iterCurr )
    {
        if( !pPtVec[iterCurr]->doDraw ) continue;// just leave it in place and move to the next

        size_t iterMax = iterCurr;
        for( size_t i = iterCurr + 1; i < pPtVec.size(); ++i )
        {
            if( !pPtVec[iterCurr]->doDraw ) continue;
            if( pPtVec[i]->getDistance() > pPtVec[iterMax]->getDistance() )
                iterMax = i;
        }

        if( iterMax > iterCurr )// swap values
        {
            persPt* pPtTemp = pPtVec[ iterCurr ];
            pPtVec[ iterCurr ] = pPtVec[ iterMax ];
            pPtVec[ iterMax ] = pPtTemp;
        }
    }
}

void persPt::sortByDistance( std::vector<persPt*>& pPtVec, size_t k )// sort only 1st k elements
{
    for( size_t iterCurr = 0; iterCurr + 1 < k; ++ iterCurr )
    {
        if( !pPtVec[iterCurr]->doDraw ) continue;// just leave it in place and move to the next

        size_t iterMax = iterCurr;
        for( size_t i = iterCurr + 1; i < k; ++i )
        {
            if( !pPtVec[iterCurr]->doDraw ) continue;
//            if( pPtVec[i]->getDistance() > pPtVec[iterMax]->getDistance() )
            if( persPt::compare( pPtVec[i], pPtVec[iterMax] ) )
                iterMax = i;
        }

        if( iterMax > iterCurr )// swap values
        {
            persPt* pPtTemp = pPtVec[ iterCurr ];
            pPtVec[ iterCurr ] = pPtVec[ iterMax ];
            pPtVec[ iterMax ] = pPtTemp;
        }
    }
}

bool persPt::init_stat( std::istream& is )
{
    if( !is ) return false;
    is >> persPt::Z0;
    is >> persPt::camPos.x >> persPt::camPos.y >> persPt::camPos.z;
    is >> persPt::zuCam.x >> persPt::zuCam.y >> persPt::zuCam.z;
    zuCam /= zuCam.mag();
    persPt::xuCam = persPt::yHat.cross( persPt::zuCam );
    persPt::xuCam /= persPt::xuCam.mag();
    persPt::yuCam = persPt::zuCam.cross( persPt::xuCam );
    return true;
}

float persPt::set_rw( vec3f Pos, float R, sf::CircleShape& rCS )
{
    float U = ( Pos - persPt::camPos ).dot( persPt::zuCam );
    if( U < 0.0f ) U *= -1.0f;
    R *= persPt::Z0/U;
    rCS.setRadius(R);
    rCS.setOrigin(R,R);
    rCS.setPosition( get_xyw( Pos ) );
    return R;
}

float persPt::zScale( vec3f P )// Z0/Z
{
    return Z0/zuCam.dot( P - camPos );
}

void persPt::updatePOI( float R, float angleSpeed, float dt )
{
    vec3f Pc = camPos + R*zuCam;
    zuCam = zuCam*cosf( angleSpeed*dt ) - xuCam*sinf( angleSpeed*dt );// yaw
 //   vec3f xuCam0 = xuCam;
    xuCam = yuCam.cross( zuCam );
    camPos = Pc - R*zuCam;
 //   camPos += ( xuCam + xuCam0 )*R*angleSpeed*dt;
}

void persPt::pitch( float dAngle )
{
    zuCam = zuCam*cosf(dAngle) - yuCam*sinf(dAngle);// pitch
    zuCam /= zuCam.mag();
    yuCam = zuCam.cross( xuCam );
}

void persPt::yaw( float dAngle )
{
    zuCam = zuCam*cosf(dAngle) + xuCam*sinf(dAngle);// yaw
    zuCam /= zuCam.mag();
    xuCam = yuCam.cross( zuCam );
}

void persPt::roll( float dAngle )
{
    yuCam = yuCam*cosf(dAngle) + xuCam*sinf(dAngle);// roll
    yuCam /= yuCam.mag();
    xuCam = yuCam.cross( zuCam );
}

void persPt::keepXuLevel()// counter roll induced by pitching and yawing
{
    xuCam = yHat.cross( zuCam );
    float xuCamMag = xuCam.mag();
    if( xuCamMag > 0.1f ) xuCam /= xuCamMag;
    else persPt::xuCam = xHat;

    persPt::yuCam = zuCam.cross( xuCam );
}

// update typical background: horizon line, skyQuad, groundQuad (if one)
void persPt::updateBackground( sf::Vertex* pHzLine, sf::Vertex* skyQuad, sf::Vertex* groundQuad )
{
    // direction toward horizon
    float zcDotY = zuCam.dot( yHat );
    vec3f hu = zuCam - zcDotY*yHat;
    float huMag = hu.mag();
    if( huMag < 0.01f ) hu = zHat;
    else hu /= huMag;
    vec3f hu2 = yHat.cross( hu );// along horizon

    // horizon line
    sf::Vector2f hzLn0 = persPt::get_xyw( persPt::camPos + persPt::Z0*( hu - hu2 ) );// left end
    sf::Vector2f hzLn1 = persPt::get_xyw( persPt::camPos + persPt::Z0*( hu + hu2 ) );// right end

    if( pHzLine )
    {
        pHzLine[0].position = hzLn0;
        pHzLine[1].position = hzLn1;
    }

    sf::Vector2f hPerp( hzLn1.y - hzLn0.y, hzLn0.x - hzLn1.x );
    if( skyQuad )
    {
        skyQuad[0].position = hzLn0 + hPerp;
        skyQuad[1].position = hzLn1 + hPerp;
        skyQuad[2].position = hzLn1;
        skyQuad[3].position = hzLn0;
    }

    if( groundQuad )
    {
        groundQuad[0].position = hzLn0;
        groundQuad[1].position = hzLn1;
        groundQuad[2].position = hzLn1 - hPerp;
        groundQuad[3].position = hzLn0 - hPerp;
    }
}

void persPt::update_doDraw()
{
    doDraw = false;
    float U = persPt::zuCam.dot( pos - persPt::camPos );
    if( U < Rbound ) return;// don't draw objects that are behind the camera
    sf::Vector2f winPos = get_xyw();
    float dim = Rbound*persPt::Z0/U;
    if( winPos.x + dim < 0.0f ) return;// left of window
    if( winPos.x - dim > 2.0f*persPt::X0 ) return;// right of window
    if( winPos.y + dim < 0.0f ) return;// above window
    if( winPos.y - dim > 2.0f*persPt::Yh ) return;// below window
    doDraw = true;
}


// same as persBall::hitFree, but with r1 = 0 and OK if isMoving
bool persPt::hit( vec3f posA, vec3f posB, vec3f& P, vec3f& vu )const
{
    vec3f rA = posA - pos, rB = posB - pos;
    if( rB.mag() > 3.0f*( Rbound ) ) return false;// too far away
    vec3f Tu = posB - posA; Tu /= Tu.mag();
    vec3f Tn = rB.cross( Tu ); Tn /= Tn.mag();
    vec3f T1 = Tu.cross( Tn );
    float r2 = Rbound;
//    if( rB.mag() > 3.0f*( r1 + r2 ) ) return false;// too far away
    float h = rA.dot( T1 );
    if(  h >= r2 ) return false;// miss. ball centers are too far apart
    float w = sqrtf( r2*r2 - h*h );
    P = h*T1 - w*Tu;
    if( Tu.dot( P - rA ) < 0.0f || Tu.dot( P - rB ) > 0.0f ) return false;// posA and posB are on the same side of P
    // It's a hit!
    vec3f pu = P/P.mag();
    P += pos;// for return
    vec3f qu = Tn.cross( pu );
    vu = (Tu.dot(qu))*qu - (Tu.dot(pu))*pu;
    return true;
}

bool persPt::hit_image( float xHit, float yHit )const
{
    sf::Vector2f winPos = persPt::get_xyw( pos );
    float SF = persPt::Z0/persPt::zuCam.dot( pos - persPt::camPos );// scale factor
    if( ( xHit - winPos.x )*( xHit - winPos.x ) + ( yHit - winPos.y )*( yHit - winPos.y ) < SF*SF*Rbound*Rbound ) return true;
    return false;
}

bool persPt::isSighted()const// in line with zuCam?
{
    vec3f sep = pos - persPt::camPos;
    float s = persPt::zuCam.dot( sep );
    vec3f h = sep - persPt::zuCam*s;
    return Rbound > h.mag();
}

// persBall
void persBall::init( std::istream& is )
{
    vec3f Pos, Vel; is >> Pos.x >> Pos.y >> Pos.z >> Vel.x >> Vel.y >> Vel.z;
    float R0; is >> R0;
    unsigned int rd, gn, bu; is >> rd >> gn >> bu;
    init( Pos, R0, sf::Color(rd,gn,bu), Vel );
}

void persBall::init( vec3f Pos, float R0, sf::Color color, vec3f Vel )
{
    pos = Pos;
    Rbound = R0;
    vel = Vel;
    B.setRadius( R0 );
    B.setOrigin(R0,R0);
    B.setFillColor( color );
    Xu = xHat;
    Yu = yHat;
    Zu = zHat;
    update(0.0f);
}

void persBall::setPosition( vec3f Pos )
{
    pos = Pos;
    update(0.0f);
}

void persBall::update_rw()
{
    float U = ( pos - persPt::camPos ).dot( persPt::zuCam );
    if( U < 0.0f ) U *= -1.0f;
    float R = Rbound*persPt::Z0/U;
    B.setRadius(R);
    B.setOrigin(R,R);
    B.setPosition( get_xyw() );
}

void persBall::update( float dt )
{
    if( !inUse ) return;
    update_doDraw();

    if( isMoving ) pos += vel*dt;
    if( doDraw ) update_rw();
}

void persBall::update( vec3f grav, float dt )
{
    if( !inUse ) return;
    update_doDraw();

    if( isMoving )
    {
        pos += ( vel + grav*dt/2.0f )*dt;
        vel += grav*dt;
    }

    if( doDraw ) update_rw();
}

void persBall::reset()
{
    inUse = true;
}

void persBall::reset( vec3f Pos, vec3f Vel )
{
    inUse = true;
    vel = Vel;
    setPosition( Pos );
}

void persBall::draw( sf::RenderTarget& RT ) const
{
    if( inUse && doDraw ) RT.draw( B );
}

bool persBall::hitFixed( vec3f posA, float r1, vec3f posB, vec3f& P, vec3f& vu, float Cr )const// *this is stationary
{
    if( isMoving ) return false;

    vec3f rA = posA - pos, rB = posB - pos;
    if( rB.mag() > 3.0f*( r1 + Rbound ) ) return false;// too far away
    vec3f Tu = posB - posA; Tu /= Tu.mag();
    vec3f Tn = rB.cross( Tu ); Tn /= Tn.mag();
    vec3f T1 = Tu.cross( Tn );
    float r2 = Rbound;
//    if( rB.mag() > 3.0f*( r1 + r2 ) ) return false;// too far away
    float h = rA.dot( T1 );
    if(  h >= r1 + r2 ) return false;// miss. ball centers are too far apart
    float w = sqrtf( (r1+r2)*(r1+r2) - h*h );
    P = h*T1 - w*Tu;
    if( Tu.dot( P - rA ) < 0.0f || Tu.dot( P - rB ) > 0.0f ) return false;// posA and posB are on the same side of P
    // It's a hit!
    vec3f pu = P/P.mag();
    P += pos;// for return
    vec3f qu = Tn.cross( pu );
    vu = (Tu.dot(qu))*qu - Cr*(Tu.dot(pu))*pu;
    return true;
}

bool persBall::hitFree( persBall& Ball, float Cr, float dt )// both are moving. Collision is handled within function
{
    if( !Ball.isMoving ) return false;

    float Rsum = Rbound + Ball.Rbound;
    float Rmin = Rbound < Ball.Rbound ? Rbound : Ball.Rbound;
    vec3f P = Ball.pos - pos;

    if( !isMoving )// call hit fixed
    {
        if( P.dot(P) > Rsum*Rsum ) return false;// too far apart
        if( P.dot( Ball.vel ) > 0.0f ) return false;// shot is receding

        vec3f vu;
        if( Ball.vel.mag()*dt < Rmin/4.0f )// low speed case
        {
            if( P.dot(P) > Rsum*Rsum ) return false;// centers too far apart
            vec3f Pu = P/P.mag();
            Ball.setPosition( pos + Pu*Rsum );// correct spacing
            if( Ball.vel.dot(P) < 0.0f )// balls are moving together
                Ball.vel -= ( 1.0f + Cr )*( Ball.vel.dot(Pu) )*Pu;// reflect

            std::cout << "\n hitFixed() low speed";
            return true;
        }
        else if( hitFixed( Ball.pos - Ball.vel*dt, Ball.Rbound, Ball.pos, P, vu ) )
        {
            Ball.setPosition( P + 5.0f*vu );
            P -= pos;
            vec3f Pu = P/P.mag();
            Ball.vel += Ball.vel.mag()*( vu.dot(Pu) )*Pu;
        //    Ball.vel = Cr*vu*Ball.vel.mag();
            return true;
        }

        return false;
    }

    // find P in frame where *this is at rest
    vec3f Vf = Ball.vel - vel, t1 = Vf/Vf.mag();
    if( Vf.mag()*dt < Rmin/4.0f )// low speed case
    {
        P = Ball.pos - pos;
        if( P.dot( Vf ) > 0.0f ) return false;// shot is receding
        if( P.dot(P) > Rsum*Rsum ) return false;
    //    std::cout << "\n hitFree() low speed";
    }
    else// high
    {
        vec3f A = Ball.pos - Vf*dt, B = Ball.pos;
        vec3f rA = A - pos, rB = B - pos;
        vec3f tn = rB.cross( t1 ); tn /= tn.mag();
        vec3f t2 = t1.cross( tn );
        float h = rB.dot( t2 );//, hMax = Rbound + Ball.Rbound;
        if(  h > Rsum ) return false;
        float w = sqrtf( Rsum*Rsum - h*h );
        P = h*t2 - w*t1;
        if( t1.dot( P - rA ) < 0.0f ) return false;
        if( t1.dot( rB - P ) < 0.0f ) return false;
     //   std::cout << "\n hitFree() high speed";
    }

    // find velocities in cm frame
    vec3f pu = P/P.mag();
    float M = mass + Ball.mass;
    vec3f Vcm = ( Ball.mass*Ball.vel + mass*vel )/M;
    vel -= Vcm; Ball.vel -= Vcm;// shift to cm frame
    vel -= ( 1.0f + Cr )*( vel.dot( pu ) )*pu;
    Ball.vel -= ( 1.0f + Cr )*( Ball.vel.dot( pu ) )*pu;
    vel += Vcm; Ball.vel += Vcm;// shift back to lab frame
    // maintain cm position
    vec3f Rcm = ( Ball.mass*Ball.pos + mass*pos )/M;
    vec3f r1 = ( mass*Rsum/M )*pu + Rcm;
    vec3f r2 = r1 - Rsum*pu;
    Ball.setPosition( r1 );
    setPosition( r2 );

    return true;
}

// static function - mutual collision handling
 unsigned int persBall::hitAll( std::vector<persBall>& PBvec, float Cr, float dt )
{
    if( PBvec.size() < 2 ) return 0;
    unsigned int hitCnt = 0;
    for( size_t i = 0; i + 1 < PBvec.size(); ++i )
        for( size_t j = i + 1; j < PBvec.size(); ++j )
            if( PBvec[i].hitFree( PBvec[j], Cr, dt ) ) ++hitCnt;
    return hitCnt;
}

bool persBall::isSighted( vec3f& hitPt, float R )const
{
    vec3f sep = pos - camPos;
    float s = sep.dot( sep )/sep.dot( persPt::zuCam );
    vec3f rB = s*persPt::zuCam - sep;
    float h = rB.mag();
    float Rt = Rbound + R;
    if( h > Rt ) return false;// miss
    vec3f t2 = rB/h;
    float w = sqrtf( Rt*Rt - h*h );
    hitPt = h*t2 - w*persPt::zuCam;
    return true;
}

// persBallTimed
float persBallTimed::tLife = 5.0f;
std::function<bool(vec3f)> persBallTimed::pFdone = nullptr;

void persBallTimed::init( std::istream& is )
{
    persBall::init(is);
    tElap = 0.0f;
}

void persBallTimed::update( float dt )
{
    persBall::update(dt);
    if( inUse && (tElap += dt) > tLife )
    {
        inUse = doDraw = false;
        tElap = 0.0f;
        if( pFdone ) pFdone( pos );
    }
}

void persBallTimed::update( vec3f grav, float dt )
{
    persBall::update( grav, dt );
    if( inUse && (tElap += dt) > tLife )
    {
        inUse = doDraw = false;
        tElap = 0.0f;
        if( pFdone ) pFdone( pos );
    }
}


// persQuad
void persQuad::init( std::istream& is, sf::Texture* p_Txt )
{
    vec3f Pos; is >> Pos.x >> Pos.y >> Pos.z;
    float W, H; is >> W >> H;
    vec3f nu; is >> nu.x >> nu.y >> nu.z;
    unsigned int rd, gn, bu; is >> rd >> gn >> bu;
    init( Pos, W, H, nu, sf::Color(rd,gn,bu), p_Txt );
}

void persQuad::init( vec3f Pos, float W, float H, vec3f nu, sf::Color color, const sf::Texture* p_Txt )
{
    pTxt = p_Txt;
    w = W; h = H;
    pos = Pos;
    Rbound = sqrtf( W*W + H*H )/2.0f;

    for( size_t i = 0; i < 4; ++i ) vtx[i].color = color;
    setNu(nu);
}

// employs ray based method
// returns true when hit and writes collision point = P and unit vec3f = vu in reflected direction
//bool persQuad::hit( vec3f posA, vec3f posB, vec3f& P, vec3f& vu, float b )const
bool persQuad::hit( vec3f posA, vec3f posB, vec3f& P, vec3f& vu )const
{
    if( !doDraw ) return false;

    // does posB - posA cross the plane anywhere?
    float Ua = ( posA - pos ).dot( Zu );// projection a
    float Ub = ( posB - pos ).dot( Zu );// projection b
    if( Ub*Ua > 0.0f ) return false;// posA and posB are both on same side of plane
    // plane has been crossed
    vec3f Rab = posB - posA;
    P = posB - ( ( posB - pos ).dot(Zu)/Rab.dot(Zu) )*Rab;// see notes page 13

    // was plane crossed within rectangle?
    // new dot product method
    if( (P-pt[0]).dot(pt[1]-pt[0]) < 0.0f ) return false;// below
    if( (P-pt[1]).dot(pt[0]-pt[1]) < 0.0f ) return false;// above
    if( (P-pt[1]).dot(pt[2]-pt[1]) < 0.0f ) return false;// left
    if( (P-pt[2]).dot(pt[1]-pt[2]) < 0.0f ) return false;// right

    // It's a hit!!
    vu = Rab - 2.0f*( Rab.dot(Zu) )*Zu;
    vu /= vu.mag();// retval #2
    return true;
}

bool persQuad::hit_image( float xHit, float yHit )const
{
    sf::Vector2f Phit( xHit, yHit );
    sf::Vector2f Pw = Phit - vtx[0].position;// offset from upper left
    sf::Vector2f sep = vtx[1].position - vtx[0].position;// right - left
    // dot product
    if( Pw.x*sep.x + Pw.y*sep.y < 0.0f ) return false;// left
    sep = vtx[3].position - vtx[0].position;// bottom - top
    if( Pw.x*sep.x + Pw.y*sep.y < 0.0f ) return false;// above

    Pw = Phit - vtx[2].position;// offset from bottom right
    sep = vtx[0].position - vtx[1].position;// left - right
    if( Pw.x*sep.x + Pw.y*sep.y < 0.0f ) return false;// right
    sep = vtx[0].position - vtx[3].position;// top - bottom
    if( Pw.x*sep.x + Pw.y*sep.y < 0.0f ) return false;// below

    return true;
}

bool persQuad::isSighted( float& dist )const// true if aimed at. Writes distance to aim point on persQuad
{
    if( !doDraw ) return false;
    float align = persPt::zuCam.dot( Zu );
    if( align*align < 0.01f ) return false;// too close to edge on view
    float distP = Zu.dot( pos - persPt::camPos)/align;
    // in rectangle ?
    vec3f P = persPt::camPos + distP*persPt::zuCam;
    if( (P-pt[0]).dot(pt[1]-pt[0]) < 0.0f ) return false;// below
    if( (P-pt[1]).dot(pt[0]-pt[1]) < 0.0f ) return false;// above
    if( (P-pt[1]).dot(pt[2]-pt[1]) < 0.0f ) return false;// left
    if( (P-pt[2]).dot(pt[1]-pt[2]) < 0.0f ) return false;// right

 //   dist = ( pos - persPt::camPos ).mag();// actual distance to the target
    dist = ( P - persPt::camPos ).mag();
    return true;
}

bool persQuad::isSighted( float& dist, vec3f& hitPt )const
{
    if( !doDraw ) return false;
    float align = persPt::zuCam.dot( Zu );
    if( align*align < 0.01f ) return false;// too close to edge on view
    float distP = Zu.dot( pos - persPt::camPos)/align;
    // in rectangle ?
    hitPt = persPt::camPos + distP*persPt::zuCam;
    vec3f P = hitPt;
    if( (P-pt[0]).dot(pt[1]-pt[0]) < 0.0f ) return false;// below
    if( (P-pt[1]).dot(pt[0]-pt[1]) < 0.0f ) return false;// above
    if( (P-pt[1]).dot(pt[2]-pt[1]) < 0.0f ) return false;// left
    if( (P-pt[2]).dot(pt[1]-pt[2]) < 0.0f ) return false;// right

 //   dist = ( pos - persPt::camPos ).mag();// actual distance to the target
    dist = ( P - persPt::camPos ).mag();
    return true;
}

void persQuad::setColor( sf::Color color )
{
    for( size_t i = 0; i < 4; ++i )
        vtx[i].color = color;
}

void persQuad::setPosition( vec3f Pos )
{
    vec3f dPos = Pos - pos;
    pos = Pos;
    for( size_t i = 0; i < 4; ++i )
    {
        pt[i] += dPos;
        vtx[i].position = persPt::get_xyw( pt[i] );
    }
    update_doDraw();
}

void persQuad::update( float dt )
{
    update_doDraw();
    if( !doDraw ) return;

    if( facingCamera )
    {
        vec3f N = persPt::camPos - pos;
        N /= N.mag();
        setNu( N );
    }
    else
    {
        for( size_t i = 0; i < 4; ++i )
            vtx[i].position = persPt::get_xyw( pt[i] );
    }
}

void persQuad::draw( sf::RenderTarget& RT ) const
{
    if( !doDraw ) return;
    if( pTxt ) RT.draw( vtx, 4, sf::Quads, pTxt );
    else RT.draw( vtx, 4, sf::Quads );
}

void persQuad::setTxtRect( sf::IntRect srcRect, char Ta, char Tb )
{
    unsigned int iSt[4] = {0,1,2,3};
    if( Ta == 'R' )// rotate
    {
        while( Tb-- > '0' ) for( size_t i = 0; i < 4; ++i ) ++iSt[i];
        for( size_t i = 0; i < 4; ++i ) iSt[i] %= 4;
    }
    else if( Ta == 'F' )// flip
    {
        if( Tb == 'X' ) { iSt[0] = 1; iSt[1] = 0; iSt[2] = 3; iSt[3] = 2; }
        else if( Tb == 'Y' ) { iSt[0] = 2; iSt[1] = 3; iSt[2] = 0; iSt[3] = 1; }
    }

    vtx[ iSt[0] ].texCoords.x = srcRect.left;// up lt
    vtx[ iSt[0] ].texCoords.y = srcRect.top;
    vtx[ iSt[1] ].texCoords.x = srcRect.left + srcRect.width;// up rt
    vtx[ iSt[1] ].texCoords.y = srcRect.top;
    vtx[ iSt[2] ].texCoords.x = srcRect.left + srcRect.width;// dn rt
    vtx[ iSt[2] ].texCoords.y = srcRect.top + srcRect.height;
    vtx[ iSt[3] ].texCoords.x = srcRect.left;// dn lt
    vtx[ iSt[3] ].texCoords.y = srcRect.top + srcRect.height;
}

void persQuad::flip( char XorY )
{
    // vtx[0] and vtx[2] are base
    size_t pair0 = XorY == 'X' ? 3 : 1;
    size_t pair2 = XorY == 'X' ? 1 : 3;

    sf::Vector2f tempV = vtx[0].texCoords;
    vtx[0].texCoords = vtx[pair0].texCoords;
    vtx[pair0].texCoords = tempV;

    tempV = vtx[2].texCoords;
    vtx[2].texCoords = vtx[pair2].texCoords;
    vtx[pair2].texCoords = tempV;
}

void persQuad::setNu( vec3f nu )
{
    // from init() code
    Zu = nu;
    if( Zu.y > 0.9f || Zu.y < -0.9f )// vertical
    {
        Xu = persPt::xHat;
        Yu = persPt::zHat;
    }
    else// make level
    {
        Xu = Zu.cross( persPt::yHat );
        Xu /= Xu.mag();
        Yu = Xu.cross( Zu );
    }

    vec3f Pos = pos;
    Pos -= (w/2.0f)*Xu;
    Pos -= (h/2.0f)*Yu;
    pt[0].x = pt[0].y = pt[0].z = 0.0f;// lower left
    pt[1] = h*Yu;// up left
    pt[3] = w*Xu;// lower right
    pt[2] = pt[1] + pt[3];// up right
    for( size_t i = 0; i < 4; ++i )
    {
        pt[i] += Pos;
        vtx[i].position = persPt::get_xyw( pt[i] );
    }
}

// unit length of nu and Tu is assumed here
void persQuad::setOrientation( vec3f nu, vec3f Tu )
{
    Zu = nu;
    Xu = Zu.cross( Tu );// in W direction
    Yu = Xu.cross( Zu );
    float H = 0.5f*h, W = 0.5f*w;
    pt[0] = pos + H*Yu - W*Xu;
    pt[1] = pos + H*Yu + W*Xu;
    pt[2] = pos - H*Yu + W*Xu;
    pt[3] = pos - H*Yu - W*Xu;
    for( size_t i = 0; i < 4; ++i )
        vtx[i].position = persPt::get_xyw( pt[i] );
}

// persQuadAni

// static
bool persQuadAni::getOne( std::vector<persQuadAni>& PQAvec, vec3f Pos )
{
    for( persQuadAni& PQA : PQAvec )
    {
        if( !PQA.inUse )
        {
            PQA.reset();
            PQA.setPosition( Pos );
        //    anyAnis_inUse = true;
            return true;
        }
    }
    return false;// all Ani are inUse
}

// regular mfs
void persQuadAni::init( std::istream& is, spriteSheet& rSS )
{
    vec3f Pos; is >> Pos.x >> Pos.y >> Pos.z;
    float W, H; is >> W >> H;
    vec3f nu; is >> nu.x >> nu.y >> nu.z;
    unsigned int rd, gn, bu; is >> rd >> gn >> bu;
    size_t SetNum; is >> SetNum;
    int NumLoops; is >> NumLoops;
    init( Pos, W, H, nu, sf::Color(rd,gn,bu), rSS, SetNum, NumLoops );
}

void persQuadAni::init( vec3f Pos, float W, float H, vec3f nu, sf::Color color, spriteSheet& rSS, size_t SetNum, int NumLoops )
{
    setNum = SetNum;
    numLoops = NumLoops;
    loopCount = numLoops;
    pSS = &rSS;
    frIdx = 0;
//    std::cerr << "\nsetNum = " << setNum;
    persQuad::init( Pos, W, H, nu, color, &(rSS.txt) );
    setTxtRect( rSS.getFrRect( frIdx, setNum ), 'R', '0' );
    inUse = false;
}

void persQuadAni::reset()
{
    inUse = true;
    loopCount = numLoops;
    frIdx = 0;
    setTxtRect( pSS->getFrRect( frIdx, setNum ), 'R', '0' );
}

void persQuadAni::update( float dt )
{
    if( !inUse ) return;
    persQuad::update( dt );
    setTxtRect( pSS->getFrRect( frIdx, setNum, true ), 'R', '0' );
    if( frIdx == 0 && numLoops > 0 && loopCount > 0 )
        if( --loopCount == 0 ) inUse = false;
}

void persQuadAni::draw( sf::RenderTarget& RT ) const
{
    if( inUse ) persQuad::draw(RT);
}
