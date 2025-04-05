#include "menu.h"

int numBasketsToDo = 500;
float masses[3] = {1.f, 1.f, 1.f};

bool menu::init( )
{
    std::cout << "Hello from menu init()\n";

    std::ifstream fin("include/levels/MM_data.txt");
    if( !fin ) return false;
    fin >> Level::winW >> Level::winH;
    sf::Vector2f iPos;// 1st option message position
    fin >> iPos.x >> iPos.y;
    float msgSpacing;
    fin >> msgSpacing;
    size_t fontSz = 30;
    fin >> fontSz;
    unsigned int r,g,b;
    fin >> r >> g >> b;
    msgClrReg = sf::Color(r,g,b);
    fin >> r >> g >> b;
    msgClrMseOver = sf::Color(r,g,b);

    fin >> numOptions;
    msgVec.reserve(numOptions);
    std::string msgIn;

    std::cout << "numOptions = " << numOptions << '\n';

    sf::Text tmp;
    fin.ignore(256,'\n');
    for( size_t i=0; i<numOptions; ++i )
    {
        sf::Text tmpTxt;
        if( button::pFont ) tmpTxt.setFont( *button::pFont );
        tmpTxt.setFillColor(msgClrReg);
        tmpTxt.setOutlineColor(sf::Color::Blue);
        tmpTxt.setOutlineThickness(0.5f);
        tmpTxt.setCharacterSize(fontSz);
        getline( fin, msgIn );
        tmpTxt.setString( msgIn.c_str() );// = "Ek = ";
        tmpTxt.setPosition( iPos.x, iPos.y + msgSpacing*i );
        std::cout << msgIn.c_str() << " at " << iPos.x << ", " << iPos.y + msgSpacing*i << '\n';
        msgVec.push_back( tmpTxt );

        sf::Rect<float> tmpRect;
        tmpRect.left = iPos.x;
        tmpRect.top = iPos.y + msgSpacing*i;
        tmpRect.width = tmpTxt.getGlobalBounds().width;
        tmpRect.height = tmpTxt.getGlobalBounds().height + 5.0f;
        rectVec.push_back( tmpRect );
    }

    fin.close();

    sf::Text label("Ball list", *button::pFont, 14);

    // save the button*s
    myButtVec.push_back( &Level::quitButt );

    reset();

    return true;
}

void menu::reset()
{
    button::RegisteredButtVec.clear();
    for( button* pButt : myButtVec )
        button::RegisteredButtVec.push_back( pButt );

    Level::quitButt.setPosition( { winW/2.0f-goto_MMButt.sz.x/2.0f,20.0f } );
    button::mseDnLt = false;// so not holding ball on return from a level
}

bool menu::handleEvent( sf::Event& rEvent )// mouse input launches rB
{
//    if( levelBox.sel ) return true;

    size_t j = numOptions;

    if ( rEvent.type == sf::Event::KeyPressed )
    {
        if( rEvent.key.code == sf::Keyboard::Num1 ) { j = 0; lvlNum = 1; }
        else if( rEvent.key.code == sf::Keyboard::Num2 ) { j = 1; lvlNum = 2; }
        else if( rEvent.key.code == sf::Keyboard::Num3 ) { j = 2; lvlNum = 3; }
        else if( rEvent.key.code == sf::Keyboard::Num4 ) { j = 3; lvlNum = 4; }
        else if( rEvent.key.code == sf::Keyboard::Num5 ) { j = 4; lvlNum = 5; }
        else if( rEvent.key.code == sf::Keyboard::Num6 ) { j = 5; lvlNum = 6; }
        else if( rEvent.key.code == sf::Keyboard::Num7 ) { j = 6; lvlNum = 7; }
        else if( rEvent.key.code == sf::Keyboard::Num8 ) { j = 7; lvlNum = 8; }
        else if( rEvent.key.code == sf::Keyboard::Num9 ) { j = 8; lvlNum = 9; }
    }
    else if (rEvent.type == sf::Event::MouseMoved)
    {
        for( size_t i=0; i<numOptions; ++i )
        {
            if( hitRect( rectVec[i], sf::Vector2f( button::mseX, button::mseY ) ) ) msgVec[i].setFillColor(msgClrMseOver);
            else msgVec[i].setFillColor(msgClrReg);
        }

    }
    else if (rEvent.type == sf::Event::MouseButtonPressed)// lbutt down
    {
        if (rEvent.mouseButton.button == sf::Mouse::Left)
            for( size_t i=0; i<numOptions; ++i )
                if( hitRect( rectVec[i], sf::Vector2f( button::mseX, button::mseY ) ) ) j = i;
    }

    if( j < numOptions )// new level
    {
        if( pLoadLvl ) Level::pCurrLvl = pLoadLvl(j);
        if( !pCurrLvl )
        {
            std::cout << "\nLevel load fail";
            pCurrLvl = this;
            reset();
        }
        button::mseDnLt = false;// consume
        msgVec[j].setFillColor(msgClrReg);
    }

    return true;
}

void menu::update( float dt )
{
    return;
}

void menu::draw( sf::RenderTarget& RT ) const
{
    for( const auto& x : msgVec ) RT.draw( x );
    return;
}

void menu::cleanup()
{
    return;
}

menu::~menu()
{
    cleanup();
}
