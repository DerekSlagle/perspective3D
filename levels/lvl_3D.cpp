#include "lvl_3D.h"

lvl_3D::lvl_3D()
{
    //ctor
}

lvl_3D::~lvl_3D()
{
    //dtor
}

bool lvl_3D::init()
{
     // static members
    persPt::camPos.x = persPt::camPos.y = persPt::camPos.z = 0.0f;
    persPt::zuCam.x = 0.0f; persPt::zuCam.y = 0.0f; persPt::zuCam.z = 1.0f;
    persPt::yuCam.x = 0.0f; persPt::yuCam.y = 1.0f; persPt::yuCam.z = 0.0f;
    persPt::xuCam.x = 1.0f; persPt::xuCam.y = 0.0f; persPt::xuCam.z = 0.0f;
 //   persPt::angle = persPt::pitchAngle = persPt::rollAngle = 0.0f;

    std::ifstream fin("include/levels/lvl_Outside/init_baseData.txt");
    if( !fin ) { std::cout << "\nNo init baseData"; return false; }

    persPt::init_stat( fin );
    fin >> persPt::angle >> persPt::pitchAngle;// in degrees
    persPt::angle *= vec2f::PI/180.0f;// to radians
    persPt::pitchAngle *= vec2f::PI/180.0f;
    float compassAngle = persPt::changeCamDir( persPt::angle, persPt::pitchAngle, 0.0f );
    to_SF_string( camHeading_Msg, compassAngle );

    fin >> pipLength;// for view direction cross
    persPt::X0 = Level::winW/2.0f; persPt::Yh = Level::winH/2.0f;
 //   fin >> camVelXZscale;
    fin >> camSpeed >> yawRate >> pitchRate >> rollRate;
    fin >> camAccel >> gravity.y;
    gravity.x = gravity.z = 0.0f;

    viewCross[0].position.y = viewCross[1].position.y = persPt::Yh;// horizontal
    viewCross[0].position.x = persPt::X0 - pipLength;
    viewCross[1].position.x = persPt::X0 + pipLength;
    viewCross[2].position.x = viewCross[3].position.x = persPt::X0;// vertical
    viewCross[2].position.y = persPt::Yh - pipLength;
    viewCross[3].position.y = persPt::Yh + pipLength;

    trueHorizon[0].color = trueHorizon[1].color = sf::Color::Blue;
    trueHorizon[0].position.y = trueHorizon[1].position.y = persPt::Yh;
    trueHorizon[0].position.x = 0.0f;
    trueHorizon[1].position.x = 2.0f*persPt::X0;

    // window panes
    unsigned int rd, gn, bu;
    fin >> rd >> gn >> bu;
    Level::clearColor = sf::Color(rd,gn,bu);
    button::setHoverBoxColor( Level::clearColor );
    fin >> rd >> gn >> bu;
    for( size_t i = 0; i < 4; ++i ) upperPane[i].color = sf::Color(rd,gn,bu);
    upperPane[0].position.x = 0.0f;        upperPane[0].position.y = 0.0f;// upper left
    upperPane[1].position.x = Level::winW; upperPane[1].position.y = 0.0f;// upper right

    persPt::updateBackground( trueHorizon, upperPane, nullptr );

    // camera messages
    sf::Vector2f cdrPos, Sz;
    fin >> cdrPos.x >> cdrPos.y >> Sz.x >> Sz.y;
    camDataRect.setPosition( cdrPos );
    camDataRect.setSize( Sz );
    float outThick;
    fin >> outThick;
    fin >> rd >> gn >> bu;
    camDataRect.setFillColor( sf::Color(rd,gn,bu) );
    fin >> rd >> gn >> bu;
    camDataRect.setOutlineColor( sf::Color(rd,gn,bu) );
    camDataRect.setOutlineThickness( outThick );
    sf::Vector2f ofst;
    fin >> ofst.x >> ofst.y;// to message origin in camDataRect
    unsigned int fontSz;
    fin >> fontSz;
    camDataMsg.setFont( *button::pFont );
    camDataMsg.setCharacterSize( fontSz );
    fin >> rd >> gn >> bu;
    camDataMsg.setFillColor( sf::Color(rd,gn,bu) );
    sf::Vector2f cdmPos = cdrPos + ofst;
    camDataMsg.setPosition( cdmPos );
    // same properties for all
    camSpeed_Msg = camHeading_Msg = camPitch_Msg = camDataMsg;
    camPosX_Msg = camPosY_Msg = camPosZ_Msg = camDataMsg;

    float offTop; fin >> offTop;
    fin >> ofst.x >> ofst.y;// to message from cdrPos
    to_SF_string( camPosX_Msg, persPt::camPos.x );
    camPosX_Msg.setPosition( cdrPos.x + ofst.x, cdrPos.y + offTop );
    to_SF_string( camPosY_Msg, persPt::camPos.y );
    camPosY_Msg.setPosition( cdrPos.x + ofst.x, cdrPos.y + offTop + ofst.y );
    to_SF_string( camPosZ_Msg, persPt::camPos.z );
    camPosZ_Msg.setPosition( cdrPos.x + ofst.x, cdrPos.y + offTop + 2.0f*ofst.y );
    fin >> ofst.x;// to right 3 messages
    camSpeed_Msg.setString("0.0");
    camSpeed_Msg.setPosition( cdrPos.x + ofst.x, cdrPos.y + offTop );
    camPitch_Msg.setString("0.0");
    camPitch_Msg.setPosition( cdrPos.x + ofst.x, cdrPos.y + offTop + ofst.y );
    camHeading_Msg.setString("0.0");
    camHeading_Msg.setPosition( cdrPos.x + ofst.x, cdrPos.y + offTop + 2.0f*ofst.y );

    // camDataMsg
    std::string helpStr, inStr;
    if( getline( fin, inStr) )
    {
        helpStr = inStr + '\n';
        while( getline( fin, inStr) ) helpStr += '\n' + inStr;
        camDataMsg.setString( helpStr.c_str() );
    }

    return true;
}

bool lvl_3D::handleEvent( sf::Event& rEvent )
{
    if ( rEvent.type == sf::Event::KeyPressed )
    {
        if( rEvent.key.code == sf::Keyboard::LShift ) keyDown_Lshift = true;
        else if( rEvent.key.code == sf::Keyboard::RShift ) keyDown_Rshift = true;
        else if( rEvent.key.code == sf::Keyboard::Left ) move_LtRt = -1;
        else if( rEvent.key.code == sf::Keyboard::Right ) move_LtRt = 1;
        else if( rEvent.key.code == sf::Keyboard::Up ) move_UpDown = 1;
        else if( rEvent.key.code == sf::Keyboard::Down ) move_UpDown = -1;
    }
    else if ( rEvent.type == sf::Event::KeyReleased )
    {
        move_LtRt = move_UpDown = 0;

        if( rEvent.key.code == sf::Keyboard::LShift ) keyDown_Lshift = false;
        else if( rEvent.key.code == sf::Keyboard::RShift ) keyDown_Rshift = false;
    }

    return true;
}

void lvl_3D::update( float dt )
{
    if( !viewIsStatic )update_global(dt);
}

void lvl_3D::draw( sf::RenderTarget& RT ) const
{
    RT.draw( upperPane, 4, sf::Quads );
    RT.draw( trueHorizon, 2, sf::Lines );

    RT.draw( viewCross, 4, sf::Lines );
    RT.draw( camDataRect );
    RT.draw( camDataMsg );
    RT.draw( camPosX_Msg );
    RT.draw( camPosY_Msg );
    RT.draw( camPosZ_Msg );
    RT.draw( camSpeed_Msg );
    RT.draw( camPitch_Msg );
    RT.draw( camHeading_Msg );
}

void lvl_3D::drawBackGround( sf::RenderTarget& RT ) const
{
    RT.draw( upperPane, 4, sf::Quads );
    RT.draw( trueHorizon, 2, sf::Lines );
}

void lvl_3D::drawForeGround( sf::RenderTarget& RT ) const
{
    RT.draw( viewCross, 4, sf::Lines );
    RT.draw( camDataRect );
    RT.draw( camDataMsg );
    RT.draw( camPosX_Msg );
    RT.draw( camPosY_Msg );
    RT.draw( camPosZ_Msg );
    RT.draw( camSpeed_Msg );
    RT.draw( camPitch_Msg );
    RT.draw( camHeading_Msg );
}

void lvl_3D::update_global( float dt )
{
    float zcDotY = persPt::zuCam.dot( persPt::yHat );
    // horizontal unit vector in view direction
    vec3f hu = persPt::zuCam - zcDotY*persPt::yHat;
    float huMag = hu.mag();
    if( huMag < 0.01f ) hu = persPt::zHat;
    else hu /= huMag;

    if( button::mseDnLt )
    {
        if( doCoast ) camSpeed += camAccel*dt;
        else persPt::camPos += hu*(camSpeed*dt);
        to_SF_string( camSpeed_Msg, camSpeed );
    }
    else if( button::mseDnRt )
    {
        if( doCoast ) camSpeed -= camAccel*dt;
        else persPt::camPos -= hu*(camSpeed*dt);

        to_SF_string( camSpeed_Msg, camSpeed );
    }

 //   if( doCoast ) persPt::camPos += hu*(camSpeed*dt);
    if( doCoast )
    {
        persPt::camPos += persPt::zuCam*(camSpeed*dt);
    }

    float dAr = 0.0f;
    if( button::didScroll ) dAr = rollRate*button::scrollAmount;

    float compassAngle = persPt::changeCamDir( yawRate*dt, pitchRate*dt, dAr );
    to_SF_string( camHeading_Msg, compassAngle );

    persPt::calibrateCameraAxes();
    // translational motion via arrow keys
    if( move_LtRt != 0 )
    {
        persPt::camPos += move_LtRt*move_xyVel*persPt::xuCam*dt;// just translate
    }

    if( move_UpDown != 0 )
    {
        persPt::camPos += move_UpDown*move_xyVel*persPt::yHat*dt;
    }

    to_SF_string( camPosX_Msg, persPt::camPos.x, 6 );
    to_SF_string( camPosY_Msg, persPt::camPos.y, 6 );
    to_SF_string( camPosZ_Msg, persPt::camPos.z, 6 );


    float angle = asinf( persPt::zuCam.dot( persPt::yHat ) )*( 180.0f/vec2f::PI );
    to_SF_string( camPitch_Msg, angle );



    if( keepLevel )
        persPt::keepXuLevel();// keep xu level
    else if( doCoast )
    {
        if( persPt::yuCam.y > 0.1f )
        {
            vec3f::bank( -gravity.y, camSpeed, yawRate, dt, persPt::xuCam, persPt::yuCam, persPt::zuCam );
        }

            persPt::zuCam /= persPt::zuCam.mag();
       //     persPt::yu = Yu_total*( 1.0f/Yu_queue.size() );
            persPt::yuCam /= persPt::yuCam.mag();
            // form xu
            persPt::xuCam = persPt::yuCam.cross( persPt::zuCam );
            persPt::yuCam = persPt::zuCam.cross( persPt::xuCam );// assure orthogonality


    }

  //  updatePane();
    persPt::updateBackground( trueHorizon, upperPane, nullptr );
}

void lvl_3D::update_zOrder()// numToDraw assigned
{
 //   numToDraw = 0;
    pPt_sortedVec.clear();

    // include all from base containers only
    for( const persPt* pPt : pPt_noUpdateVec )
        if( pPt->inUse && pPt->doDraw )
            pPt_sortedVec.push_back( pPt );

    for( const persPt* pPt : pPt_updateVec )
        if( pPt->inUse && pPt->doDraw )
            pPt_sortedVec.push_back( pPt );

    std::sort( pPt_sortedVec.begin(), pPt_sortedVec.end(), persPt::compare );
}

void lvl_3D::update_zOrder( const std::vector<const persPt*>& pPersVec )
{
 //   numToDraw = 0;
    pPt_sortedVec.clear();

    // include all from base containers only
    for( const persPt* pPt : pPt_noUpdateVec )
        if( pPt->inUse && pPt->doDraw )
            pPt_sortedVec.push_back( pPt );

    for( const persPt* pPt : pPt_updateVec )
        if( pPt->inUse && pPt->doDraw )
            pPt_sortedVec.push_back( pPt );

    // others
    for( const persPt* pPt : pPersVec )
        if( pPt->inUse && pPt->doDraw )
            pPt_sortedVec.push_back( pPt );

    std::sort( pPt_sortedVec.begin(), pPt_sortedVec.end(), persPt::compare );
}
