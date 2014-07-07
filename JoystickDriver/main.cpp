#include <SFML/Graphics.hpp>
#include <iostream>
#include <sstream>
#include "Serial.h"
#include "packer.h"
#include <math.h>

using namespace std;
namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}

typedef union _data {
  float f;
  char  s[4];
} myData;

myData q1;
myData q2;
myData q3;
myData q4;

int main(){
    //Syn_serial serial("/dev/ttyUSB0");
    Serial serial("\\\\.\\COM10");
    Packer packerHandler(&serial);
    float w ,h;
    w = 600;
    h = 600;
    sf::RenderWindow window(sf::VideoMode(w, h), "Joystick control - Robocup");

    window.setFramerateLimit(100);

    bool manual = false;
    float multiplier = 1;
    float MIN_SPEED = 0.2f;
    sf::Vertex c = sf::Vector2f(w/2.f, h/2.f);
    sf::Vertex c_rot = sf::Vector2f(w/2.f, 30);
    sf::Vertex line[] ={ c, c};
    sf::Vertex rotation[] ={ c_rot, c_rot};

    sf::Font font;
    if (!font.loadFromFile("arial.ttf"))
    {
        std::cout<<"no arial.ttf font in folder";
    }

    sf::Text coorX;
    sf::Text coorY;
    sf::Text coorZ;
    sf::Text coorT;
    sf::Text log;
    coorX.setFont(font);
    coorY.setFont(font);
    coorZ.setFont(font);
    coorT.setFont(font);
    log.setFont(font);
    coorX.setPosition(50,50);
    coorY.setPosition(50,70);
    coorZ.setPosition(50,90);
    coorT.setPosition(50,120);
    coorT.setPosition(50,150);
    coorX.setCharacterSize(12);
    coorY.setCharacterSize(12);
    coorZ.setCharacterSize(12);
    coorT.setCharacterSize(12);
    log.setCharacterSize(30);
    //sf::RectangleShape line(sf::Vector2f(w/2.f, 5));arial.ttf
    //line.setOrigin(w/2.f, h/2.f);

    float hypo, x, y, z;
    while (window.isOpen()){
        if(!serial.IsConnected())
            log.setString("Impossible to connect to COM10");
        else if(!sf::Joystick::isConnected(0))
            log.setString("Joystick not connected" );
        else{
            if(manual)
                log.setString("Manual" );
            else
                log.setString("Auto" );
            x = sf::Joystick::getAxisPosition(0, sf::Joystick::X);
            y = sf::Joystick::getAxisPosition(0, sf::Joystick::Y);
            z = sf::Joystick::getAxisPosition(0, sf::Joystick::Z);
            x = x/100*multiplier;
            y = -y/100*multiplier;
            z = z/100*multiplier;

            hypo = sqrt(x*x + y*y);

            if(MIN_SPEED > hypo){
                x = 0.f;
                y = 0.f;
            }

            if(MIN_SPEED > abs(z)){
                z = 0.f;
            }

            if (sf::Joystick::isButtonPressed(0, 1))
                multiplier = 0.25;
            if (sf::Joystick::isButtonPressed(0, 2))
                multiplier = 0.5;
            if (sf::Joystick::isButtonPressed(0, 3))
                multiplier = 1;
            if (sf::Joystick::isButtonPressed(0, 8))
                manual = true;
            if (sf::Joystick::isButtonPressed(0, 9))
                manual = false;

            line[1] = sf::Vertex( sf::Vector2f( w/2.f + x * 100, h/2.f + (-y) * 100));
            rotation[1] = sf::Vertex( sf::Vector2f( w/2.f + z * 100, 30));
            coorX.setString("X ="  +  patch::to_string(x));
            coorY.setString("Y ="  +  patch::to_string(y));
            coorZ.setString("Z ="  +  patch::to_string(z));
            coorT.setString("Speed Multiplier ="  +  patch::to_string(multiplier) + " X");



            if (!manual or (sf::Joystick::isButtonPressed(0, 5) and manual)){
                packerHandler.createSpeedCommand(x,y,z,1);
                packerHandler.sendPacket();
            }
        }
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        window.clear();
       // window.draw(line);
        window.draw(line, 2, sf::Lines);
        window.draw(rotation, 2, sf::Lines);
        window.draw(coorX);
        window.draw(coorY);
        window.draw(coorZ);
        window.draw(coorT);
        window.draw(log);
        window.display();
    }

    return 0;
}
