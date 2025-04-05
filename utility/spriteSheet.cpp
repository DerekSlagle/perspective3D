#include "spriteSheet.h"

spriteSheet::spriteSheet( const char* fName )
{
    std::ifstream inFile( fName );
    if( inFile ) init(inFile);
}

spriteSheet::spriteSheet( std::ifstream& inFile ) { init(inFile); }

void spriteSheet::init( std::ifstream& inFile )
{
    getline( inFile, imgFname );

    img.loadFromFile( imgFname.c_str());
    txt.loadFromImage( img );
    size_t numSets;
    inFile >> numSets;
 //   std::cout << "\nnumSets read: " << numSets;
    frSetVec.reserve(numSets);

    size_t numFrames = 0;// read in a frame set
//    for( size_t i = 0; i < numSets && inFile >> numFrames; ++i )
    for( size_t i = 0; i < numSets; ++i )
    {
        inFile >> numFrames;
 //       std::cout << "\nnumFrames read: " << numFrames;
        std::vector<sf::IntRect> setVec;
        setVec.reserve(numFrames);
        int Lt, Tp, W, H;
        for( size_t j = 0; j < numFrames; ++j )
        {
            inFile >> Lt >> Tp >> W >> H;
     //       std::cout << "\nLt: " << Lt << " Tp: " << Tp << " W: " << W << " H: " << H;
            sf::IntRect temp(Lt,Tp,W,H);
            setVec.push_back(temp);
        }

        frSetVec.push_back( setVec );
  //      std::cout << "\nfrSetVec.size(): " << frSetVec.size();
    }

 //   std::cout << "\nfrSetVec.size(): " << frSetVec.size();
}

spriteSheet::~spriteSheet()
{
    //dtor
}

sf::IntRect spriteSheet::getFrRect( size_t& frIdx, size_t setNum, bool frAdvance ) const
{
    if( setNum >= frSetVec.size() ) setNum = frSetVec.size() - 1;
    if( frAdvance && ++frIdx >= frSetVec[setNum].size() ) frIdx = 0;
    return frSetVec[setNum][frIdx];
}

void spriteSheet::copySet( sf::RectangleShape* pR, size_t setNum )const
{
    if( setNum >= frSetVec.size() ) return;

    for( size_t i=0; i<frSetVec[setNum].size(); ++i )
    {
        sf::IntRect frRect = getFrRect(i,setNum);
        sf::Vector2f Rsz( frRect.width, frRect.height );
        pR[i].setSize( Rsz );
    //    pR[i].setTexture( &img );
        pR[i].setTexture( &txt );
        pR[i].setTextureRect( frRect );
    }
}

bool spriteSheet::toFile( const char* fName )const
{
    std::ofstream fout( fName );
    if( !fout ) return false;

    fout << imgFname << '\n';
    fout << '\n' << frSetVec.size() << '\n';// number of sets

    for( size_t i=0; i<frSetVec.size(); ++i )// for each frame set
    {
        fout << '\n' << frSetVec[i].size();// number of frames
        for( size_t j=0; j<frSetVec[i].size(); ++j )// for each frame
            fout << '\n' << getFrRect( j, i ).left << ' ' << getFrRect( j, i ).top << ' ' << getFrRect( j, i ).width << ' ' << getFrRect( j, i ).height;
    }

    fout.close();
    return true;
}

// assign texCoords for a quad. Returns false if either setNum or frIdx too high
bool spriteSheet::setTxtRect( sf::Vertex* vtx, size_t frIdx, size_t setNum , char Ta, char Tb )const
{
    if( setNum >= frSetVec.size() ) return false;
    if( frIdx >= frSetVec[setNum].size() ) return false;

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

    sf::IntRect srcRect( getFrRect( frIdx, setNum ) );
    vtx[ iSt[0] ].texCoords.x = srcRect.left;// up lt
    vtx[ iSt[0] ].texCoords.y = srcRect.top;
    vtx[ iSt[1] ].texCoords.x = srcRect.left + srcRect.width;// up rt
    vtx[ iSt[1] ].texCoords.y = srcRect.top;
    vtx[ iSt[2] ].texCoords.x = srcRect.left + srcRect.width;// dn rt
    vtx[ iSt[2] ].texCoords.y = srcRect.top + srcRect.height;
    vtx[ iSt[3] ].texCoords.x = srcRect.left;// dn lt
    vtx[ iSt[3] ].texCoords.y = srcRect.top + srcRect.height;
    return true;
}

bool spriteSheet::setTxtRect( sf::VertexArray& VA, size_t frIdx, size_t setNum, int Nrotate, char Tflip )const
{
 //   return true;// temp

    if( setNum >= frSetVec.size() ) return false;
 //   return true;// temp
    if( frIdx >= frSetVec[setNum].size() ) return false;
 //   return true;// temp
    unsigned int iSt[4] = {0,1,2,3};
 //   return true;// temp

    while( Nrotate-- > 0 ) for( size_t i = 0; i < 4; ++i ) ++iSt[i];
    for( size_t i = 0; i < 4; ++i ) iSt[i] %= 4;

    if( Tflip == 'X' ) { iSt[0] = 1; iSt[1] = 0; iSt[2] = 3; iSt[3] = 2; }
    else if( Tflip == 'Y' ) { iSt[0] = 2; iSt[1] = 3; iSt[2] = 0; iSt[3] = 1; }

 //   return true;// temp

    sf::IntRect srcRect( getFrRect( frIdx, setNum ) );
    VA[ iSt[0] ].texCoords.x = srcRect.left;// up lt
    VA[ iSt[0] ].texCoords.y = srcRect.top;
    VA[ iSt[1] ].texCoords.x = srcRect.left + srcRect.width;// up rt
    VA[ iSt[1] ].texCoords.y = srcRect.top;

    if( VA.getPrimitiveType() == sf::Quads )
    {
        VA[ iSt[2] ].texCoords.x = srcRect.left + srcRect.width;// dn rt
        VA[ iSt[2] ].texCoords.y = srcRect.top + srcRect.height;
        VA[ iSt[3] ].texCoords.x = srcRect.left;// dn lt
        VA[ iSt[3] ].texCoords.y = srcRect.top + srcRect.height;
    }
    else if( VA.getPrimitiveType() == sf::TriangleStrip )
    {
        VA[ iSt[3] ].texCoords.x = srcRect.left + srcRect.width;// dn rt
        VA[ iSt[3] ].texCoords.y = srcRect.top + srcRect.height;
        VA[ iSt[2] ].texCoords.x = srcRect.left;// dn lt
        VA[ iSt[2] ].texCoords.y = srcRect.top + srcRect.height;
    }

    return true;
}

// static method
void spriteSheet::flipTexCoords( sf::Vertex* vtx, char XorY )// sometimes the required orientation needs a flip only
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

bool spriteSheet::loadSpriteSheets( std::vector<spriteSheet>& SSvec, const char* inFileName )
{
    std::ifstream finList( inFileName );
    if( !finList ) { std::cout << "\n spriteSheet::loadSpriteSheets(): no ssFileList"; return false; }

    std::string rootName;// directory path from file
    finList >> rootName;
    int numSS = 0; finList >> numSS;
    if( numSS < 1 ) return false;
    SSvec.clear();
    SSvec.reserve( numSS );

    std::string fileName;
    while( numSS-- > 0 && finList >> fileName )
    {
  //      fileName = "include/levels/lvl_perspective/images/" + fileName;
        fileName = rootName + fileName;
        std::ifstream f_in( fileName.c_str() );
        if( !f_in ) { std::cout << "\nno " << fileName; return false; }
        SSvec.push_back( spriteSheet( f_in ) );
        f_in.close();
    }

    // apply transparency masks?
    size_t numToMask = 0; finList >> numToMask;
    if( numToMask > 0 )
    {
        for( size_t i = 0; i < numToMask; ++i )
        {
            size_t idx = 0; finList >> idx;
            unsigned int rd, gn, bu; finList >> rd >> gn >> bu;
            if( idx < SSvec.size() )
            {
                SSvec[idx].img.createMaskFromColor( sf::Color(rd,gn,bu) );
                SSvec[idx].txt.loadFromImage( SSvec[idx].img );
            }
        }
    }

    finList.close();
    std::cout << "\n#SS = " << SSvec.size();
    return true;
}

// quadPack methods
void quadPack::init( std::ifstream& fin, spriteSheet* p_SS )
{
    pSS = p_SS;

    size_t numLayers = 0, setNum = 0;
    float scale = 1.0f;
    fin >> scale;
    fin >> numLayers >> setNum;
    for( size_t i=0; i<numLayers; ++i )
    {
        float x0, y0; fin >> x0 >> y0;// start position
        if( i == 0 ) { pos.x = x0; pos.y = y0; }

        char dir = 'n';// direction to expand layer in: U,D,L,R
        size_t n; fin >> n >> dir;// number this layer
        float x = x0;
        sf::Vertex vt;
        for( size_t j=0; j<n; ++j )// each quad in this layer
        {
            size_t frIdx; fin >> frIdx;// area rendered
            sf::IntRect frRect = pSS->getFrRect( frIdx, setNum );

            vt.position.x = x;// up lt
            vt.position.y = y0;
            vt.texCoords.x = frRect.left;
            vt.texCoords.y = frRect.top;
            vtxVec.push_back( vt );
            vt.position.x = x + frRect.width*scale;// tp rt
            vt.position.y = y0;
            vt.texCoords.x = frRect.left + frRect.width;
            vt.texCoords.y = frRect.top;
        //    if(  dir == 'R' && i == 0 && j+1 == n ){ L.x = vt.position.x; L.y = vt.position.y; L -= pos; }
            if(  dir == 'R' && i == 0 && j+1 == n ){ L = vt.position; L -= pos; }
            vtxVec.push_back( vt );
            vt.position.x = x + frRect.width*scale;// bt rt
            vt.position.y = y0 + frRect.height*scale;
            vt.texCoords.x = frRect.left + frRect.width;
            vt.texCoords.y = frRect.top + frRect.height;
       //     if(  dir == 'D' && i == 0 && j+1 == n ){ L.x = vt.position.x; L.y = vt.position.y; L -= pos; }
            if(  dir == 'D' && i == 0 && j+1 == n ){ L = vt.position; L -= pos; }
            vtxVec.push_back( vt );
            vt.position.x = x;// bt lt
            vt.position.y = y0 + frRect.height*scale;
            vt.texCoords.x = frRect.left;
            vt.texCoords.y = frRect.top + frRect.height;
            vtxVec.push_back( vt );

            if( dir == 'R' ) x += frRect.width*scale;
            else if( dir == 'D' ) y0 += frRect.height*scale;
        }
    }
 //   std::cerr << "quadPack.init(): L = " << L.mag() << '\n';
//    std::cerr << "quadPack.init(): L = " << L.mag() << '\n';
}
