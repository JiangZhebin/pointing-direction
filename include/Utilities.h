#pragma once

#include <SFML/Graphics.hpp>
#include <iostream>



class Utilities
{
public:
  Utilities()=default;

  ~Utilities()
  {}

  sf::Vector3f UnitVector(sf::Vector3f p1, sf::Vector3f p2);
  sf::Vector2f UnitVector2D(sf::Vector2f p1, sf::Vector2f p2);

  float xz_angle(sf::Vector3f u);


  float xy_angle(sf::Vector3f u);


  float angle_b_vector(sf::Vector2f v1, sf::Vector2f v2);
  std::string time_now();

};


