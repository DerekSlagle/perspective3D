#include "lvl_Outside.h"

lvl_Outside::lvl_Outside()
{
    //ctor
}

lvl_Outside::~lvl_Outside()
{
    //dtor
}

bool lvl_Outside::init()
{
    if( !lvl_3D::init() ){ std::cout << "\nbaseInit failed."; return false; }

    std::ifstream fin("include/levels/lvl_Outside/init_data.txt");
    if( !fin ) return false;

    sf::Vector2f pos;
    fin >> pos.x >> pos.y;
    Level::quitButt.setPosition( pos );
    button::RegisteredButtVec.push_back( &Level::quitButt );
    fin >> pos.x >> pos.y;
    Level::goto_MMButt.setPosition( pos );
    button::RegisteredButtVec.push_back( &Level::goto_MMButt );

    unsigned int rd, gn, bu;
    fin >> rd >> gn >> bu;
    Level::clearColor = sf::Color(rd,gn,bu);
    button::setHoverBoxColor( Level::clearColor );

    std::string fileName;
    fin >> fileName;
    if( !init_controls( fileName.c_str() ) ) return false;

    fin >> fileName;
    if( !spriteSheet::loadSpriteSheets( SSvec, fileName.c_str() ) ){ std::cout << "\n loadSpriteSheets() fail"; return false; }

    fin >> fileName;
    if( !init_terrain( fileName.c_str() ) ){ std::cout << "\n init_terrain() fail"; return false; }

    fin >> fileName;
    if( !init_boxes( fileName.c_str() ) ){ std::cout << "\n init_boxes() fail"; return false; }

    // prepare initial draw
    update_global(0.0f);

    if( !pPt_noUpdateVec.empty() )
        for( persPt* pPt : pPt_noUpdateVec )
            pPt->update(0.0f);

    if( !pPt_updateVec.empty() )
        for( persPt* pPt : pPt_updateVec )
            pPt->update(0.0f);

    update_zOrder();



//    pPt_sortedVec = pPt_noUpdateVec;// for now

    return true;
}

bool lvl_Outside::handleEvent( sf::Event& rEvent )
{
    if( !lvl_3D::handleEvent( rEvent ) ) return false;

    if ( rEvent.type == sf::Event::KeyPressed )
    {
        if( rEvent.key.code == sf::Keyboard::Z )// STOP
        {
            camSpeed = 0.0f;
            camSpeed_Msg.setString("0.0");
        }
    }

    return true;
}

void lvl_Outside::update( float dt )
{
    viewIsStatic = true;
    if( !jbCamButt.atRest )
    {
        viewIsStatic = false;
        update_global(dt);

        if( !pPt_noUpdateVec.empty() )
            for( persPt* pPt : pPt_noUpdateVec )
                pPt->update(dt);
    }

    // update due to motion or animation in place
    for( unsigned int k = 0; k < boxVec.size(); ++k )
    {
        if( boxRotMode[k] == 'Y' )
            vec3f::yaw( boxRotSpeed[k]*dt, boxVec[k].Xu, boxVec[k].Yu, boxVec[k].Zu );
        else if( boxRotMode[k] == 'P' )
            vec3f::pitch( boxRotSpeed[k]*dt, boxVec[k].Xu, boxVec[k].Yu, boxVec[k].Zu );
        else if( boxRotMode[k] == 'R' )
            vec3f::roll( boxRotSpeed[k]*dt, boxVec[k].Xu, boxVec[k].Yu, boxVec[k].Zu );
    }

    if( !pPt_updateVec.empty() )
        for( persPt* pPt : pPt_updateVec )
            pPt->update(dt);

    if( !viewIsStatic )
        update_zOrder();
}

void lvl_Outside::draw( sf::RenderTarget& RT ) const
{
 //   lvl_3D::draw(RT);

 //   if( !pPt_noUpdateVec.empty() )
 //       for( const persPt* pPt : pPt_noUpdateVec )
 //           pPt->draw(RT);
    lvl_3D::drawBackGround(RT);

    if( !pPt_sortedVec.empty() )
        for( const persPt* pPt : pPt_sortedVec )
            pPt->draw(RT);

    lvl_3D::drawForeGround(RT);
}

bool lvl_Outside::init_controls( const char* fileName )
{
    std::ifstream fin( fileName );
    if( !fin ) { std::cout << "\nNo control data"; return false; }

    float R, r, posX, posY; fin >> R >> r >> posX >> posY;
    fin >> camVelXZscale;

    jbCamButt.init( R, r, posX, posY );
    jbCamButt.pFunc_ff = [this](float x, float z)
    {
        float Sxy = camVelXZscale;
        yawRate = Sxy*x*x;
        pitchRate = -Sxy*z*z;

        if( x < 0.0f ) yawRate *= -1.0f;
        if( z < 0.0f ) pitchRate *= -1.0f;
    };
    button::RegisteredButtVec.push_back( &jbCamButt );

    return true;
}

bool lvl_Outside::init_terrain( const char* fileName )
{
    std::ifstream fin( fileName );
    if( !fin ) { std::cout << "\nNo terrain data"; return false; }



    vec3f PosA, PosB;

    // trees = A and grass = B are on separate spriteSheets
    vec3f  zuA(0.0f,0.0f,1.0f), zuB(0.0f,1.0f,0.0f);// persPt::yHat (up), persPt::zHat
    float wA, hA;
    unsigned int SSnumA, SetNumA, FrIdxA;
    fin >> SSnumA >> SetNumA >> FrIdxA >> wA >> hA;
    char Tr = 'R', RnA = '0';
    fin >> RnA;
    PosA.y = 0.5f*hA;

    float wB, hB;
    unsigned int SSnumB, SetNumB, FrIdxB;
    fin >> SSnumB >> SetNumB >> FrIdxB >> wB >> hB;
    char RnB = '0';
    fin >> RnB;
    PosB.y = 0.0f;

    unsigned int numQuads;
    fin >> numQuads;
    PQvec.reserve( 2*numQuads );

    for( unsigned int k = 0; k < numQuads; ++k )
    {
        fin >> PosB.x >> PosB.z;
        PQvec.push_back( persQuad( PosB, wB, hB, zuB, sf::Color::White, &( SSvec[SSnumB].txt ) ) );
        PQvec.back().setTxtRect( SSvec[SSnumB].getFrRect( FrIdxB, SetNumB ) , Tr, RnB );
        PosA.x = PosB.x;
        PosA.z = PosB.z;
        PQvec.push_back( persQuad( PosA, wA, hA, zuA, sf::Color::White, &( SSvec[SSnumA].txt ) ) );
        PQvec.back().setTxtRect( SSvec[SSnumA].getFrRect( FrIdxA, SetNumA ) , Tr, RnA );
    }

    unsigned int numBalls;
    fin >> numBalls;
    ballVec.reserve( numBalls );
    unsigned int rd, gn, bu;
    fin >> rd >> gn >> bu;

    for( unsigned int k = 0; k < numBalls; ++k )
    {
        fin >> PosA.x >> PosA.y >> PosA.z;
        float Rb; fin >> Rb;
        ballVec.push_back( persBall( PosA, Rb, sf::Color(rd,gn,bu) ) );
    }

    std::cout << "\ninit_terrain(): PQvec.size() = " << PQvec.size();

    // once PQvec filled
    for( persQuad& PQ : PQvec )
        pPt_noUpdateVec.push_back( &PQ );

    for( persBall& PB : ballVec )
        pPt_noUpdateVec.push_back( &PB );

    return true;
}

bool lvl_Outside::init_boxes( const char* fileName )
{
    std::ifstream fin( fileName );
    if( !fin ) { std::cout << "\nNo box data"; return false; }

    size_t numBoxes; fin >> numBoxes;
    std::cout << "\n***************numBoxes: " << numBoxes;
    boxVec.reserve( numBoxes );// temp1, temp2, etc
    boxRotSpeed.reserve( numBoxes );// temp1, temp2, etc
    boxRotMode.reserve( numBoxes );// temp1, temp2, etc

    size_t SSnum = 0;
    float rotSpd = 0.0f;
    char rotMode = 'N';// none

    for( size_t i = 0; i < numBoxes; ++i )
    {
        fin >> SSnum >> rotMode >> rotSpd;

        boxRotMode.push_back( rotMode );
        boxRotSpeed.push_back( rotSpd );

        boxVec.push_back( persBox( fin, &(SSvec[SSnum]) ) );
        pPt_updateVec.push_back( &( boxVec.back() ) );
    }

    return true;
}







