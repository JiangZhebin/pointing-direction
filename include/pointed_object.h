
#include <SFML/Graphics.hpp>


#include <iostream>
#include <Utilities.h>




class PointedObject
{
public:
  //in xz plane


  PointedObject(sf::Vector3f MidSpine_wpos, sf::Vector3f arm_unitVector,
                  sf::Vector2f object1=sf::Vector2f(0,0), sf::Vector2f object2=sf::Vector2f(0,0),
                  sf::Vector2f object3=sf::Vector2f(0,0), sf::Vector2f object4 = sf::Vector2f(0,0),
                  sf::Vector2f object5 = sf::Vector2f(0,0));

  ~PointedObject(){}
  bool BoundaryTest();
  int whichobject() ;
  void absolute_angle_method(int armFlag, float xzAngle_R, float xzAngle_L);
  bool object_existance();

private:
  std::vector<float> angle;
  std::vector<sf::Vector2f> object;
  std::vector<sf::Vector2f> person_o;
  float xzAngle;
  float xyAngle;
  sf::Vector3f arm_unitVector_;

  enum direction {left, right, middle, none};
  direction dir = none;
};

