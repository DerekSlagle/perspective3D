#ifndef SPRITESHEET_H
#define SPRITESHEET_H

#include<vector>
#include<fstream>
#include<iostream>
#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>// for quadPack::draw
//#include "vec2d.h"

class spriteSheet
{
    public:
        sf::Image img;
        sf::Texture txt;
        std::string imgFname;// file name image loaded from
        spriteSheet() {}
        spriteSheet( const char* fName );
        spriteSheet( std::ifstream& inFile );
        void init( std::ifstream& inFile );
        ~spriteSheet();

        std::vector<sf::IntRect>::const_iterator getSet_iter( size_t setNum )const { return frSetVec[setNum].begin(); }
        const std::vector<sf::IntRect>& getSetVec( size_t setNum )const { return frSetVec[setNum]; }
        sf::IntRect getFrRect( size_t& frIdx, size_t setNum, bool frAdvance ) const;
        sf::IntRect getFrRect( size_t frIdx, size_t setNum ) const { return frSetVec[setNum][frIdx]; }
        sf::IntRect& getFrRect( size_t frIdx, size_t setNum ) { return frSetVec[setNum][frIdx]; }
        size_t numSets() const { return frSetVec.size(); }
        size_t getFrCount( size_t setNum )const { return frSetVec[setNum].size(); }
        void copySet( sf::RectangleShape* pR, size_t setNum )const;
        bool toFile( const char* fName )const;
        // assign texCoords for a quad. Returns false if either setNum or frIdx too high
        bool setTxtRect( sf::Vertex* vtx, size_t frIdx, size_t setNum , char Ta, char Tb )const;
        bool setTxtRect( sf::VertexArray& VA, size_t frIdx, size_t setNum, int Nrotate, char Tflip )const;
        static void flipTexCoords( sf::Vertex* vtx, char XorY );// sometimes the required orientation needs a flip only
        static bool loadSpriteSheets( std::vector<spriteSheet>& SSvec, const char* inFileName );

    protected:
    private:
    std::vector< std::vector<sf::IntRect> > frSetVec;
};

struct quadPack
{
    std::vector<sf::Vertex> vtxVec;
    spriteSheet* pSS = nullptr;
//    vec2d pos, L;
    sf::Vector2f pos, L;
    quadPack(){}
    void init( std::ifstream& fin, spriteSheet* p_SS );
    quadPack( std::ifstream& fin, spriteSheet* p_SS )  { init( fin, p_SS ); }
    void draw( sf::RenderTarget& RT ) const { RT.draw( &(vtxVec[0]), vtxVec.size(), sf::Quads, &(pSS->txt) ); }
};

#endif // SPRITESHEET_H
