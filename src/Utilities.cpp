#pragma once

#include <Utilities.h>
#include <chrono>
  sf::Vector3f Utilities::UnitVector(sf::Vector3f p1, sf::Vector3f p2){
    sf::Vector3f p3 = p1-p2;
    float p3_length= std::sqrt(p3.x*p3.x + p3.y*p3.y+ p3.z*p3.z);
    return p3/p3_length;
  }

  sf::Vector2f Utilities::UnitVector2D(sf::Vector2f p1, sf::Vector2f p2){
    sf::Vector2f p3 = p1 - p2 ;
    float p3_length = std::sqrt(p3.x* p3.x + p3.y * p3.y);
    return p3/p3_length;
  }

  float Utilities::xz_angle(sf::Vector3f u)
  {
    sf::Vector3f z_unitVector (0,0,-1);
    return std::acos(u.x * z_unitVector.x + u.z * z_unitVector.z);
  }

  float Utilities::xy_angle(sf::Vector3f u)
  {
    sf::Vector3f y_unitVector (0,-1,0);
    return std::acos(u.x * y_unitVector.x + u.y * y_unitVector.y);

  }

  float Utilities::angle_b_vector(sf::Vector2f v1, sf::Vector2f v2)
  {
    float v1_l = std::sqrt(v1.x*v1.x+v1.y*v1.y);
    float v2_l = std::sqrt(v2.x *v2.x + v2.y* v2.y);
    return std::acos((v1.x * v2.x + v1.y * v2.y)/(v1_l * v2_l));
  }

  std::string Utilities::time_now(){
    std::chrono::system_clock::time_point p = std::chrono::system_clock::now();
    time_t t = std::chrono::system_clock::to_time_t(p);
    struct tm * timeinfo;
    char buffer[80];

    time (&t);
    timeinfo = localtime(&t);

    strftime(buffer,sizeof(buffer),"%d-%m-%Y %H:%M:%S",timeinfo);
    std::string str(buffer);
    return str;
  }


