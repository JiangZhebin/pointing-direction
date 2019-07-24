//compare arm vector with object_personPosition object, the smaller the angle between is, more likely to be the object person is pointing to.
// x+ is on the right side towards camera
#include <pointed_object.h>
#include <algorithm>

PointedObject::PointedObject(sf::Vector3f MidSpine_wpos, sf::Vector3f arm_unitVector,
                sf::Vector2f object1, sf::Vector2f object2,
                sf::Vector2f object3, sf::Vector2f object4,
                sf::Vector2f object5 )


  {

    xzAngle = Utilities().xz_angle(arm_unitVector);
    arm_unitVector_ = arm_unitVector;
    object.push_back(object1);
    object.push_back(object2);
    object.push_back(object3);
    object.push_back(object4);
    object.push_back(object5);
    sf::Vector2f arm_unitVector2D(arm_unitVector.x,arm_unitVector.z);
    sf::Vector2f MidSpine2D(MidSpine_wpos.x,MidSpine_wpos.z);
    for(int i=object.size()-1; i>=0; --i ){
  //    std::cout <<i <<"   object coordinate "<< object.at(i).x<<","<<object.at(i).y<< std::endl;
      if(object.at(i).x==0.0 && object.at(i).y == 0.0 ){
   //     std::cout<< "object pop back" << std::endl;
        object.pop_back();

      }

    }
   // std::cout << "object size after initialization"<<object.size()<< std::endl;
    std::cout << "arm unit vector = " << arm_unitVector2D.x << "," << arm_unitVector2D.y << std::endl;
    for(unsigned long i=0; i<object.size();++i)
    {
   //   std::cout <<"person object vector: "<< Utilities().UnitVector2D(object.at(i), MidSpine2D).x<<"  ,  "<<Utilities().UnitVector2D(object.at(i), MidSpine2D).y<< std::endl;
      person_o.push_back(Utilities().UnitVector2D(object.at(i), MidSpine2D));
      angle.push_back(Utilities().angle_b_vector(arm_unitVector2D,person_o.at(i)));
     // std::cout <<i<< "  person/ angle push back" << std::endl;
    }


  }

  bool PointedObject::BoundaryTest()
  {
    bool flag = false;
    for(unsigned long i=0; i< angle.size() ; i++){
      if(angle.at(i)< 0.785)
        flag = true;

    }
    return flag;
  }


  int PointedObject::whichobject()
  {
    //std::cout << "object 1,2,3 angle" << angle.at(0) << "  , " << angle.at(1)<<"  ,  " << angle.at(2) <<std::endl;

   // unsigned long j=angle.size();
    //std::cout << "angle.size= "<<j<< std::endl;

    int minAngleIndex = std::min_element(angle.begin(), angle.end()) - angle.begin();
    float minAngle = *std::min_element(angle.begin(),angle.end());

    std::cout<< "The smallest angle is " << minAngle<<std::endl;
    std::cout<< "Pointed to Object: " << minAngleIndex+1<< std::endl;
    return minAngleIndex+1;
    }

  void PointedObject::absolute_angle_method(int armFlag, float xzAngle_R, float xzAngle_L)
  {
    switch (armFlag) {
    case(3):{
      std::cout << "Please just raise one arm!" << std::endl;
      break;}
    case(0):{
      std::cout << "Please raise one arm to give the control signal"<< std::endl;
      break;}
    case(2):
    case(1):{
      //feed the coordinates of the controllers
        if(armFlag==1)
          xzAngle = xzAngle_R;
        if(armFlag==2)
          xzAngle = xzAngle_L;
        std::cout<<"Using absolute angles to analyze"<< std::endl;
        if((xzAngle > 0.7854f && arm_unitVector_.x > 0)  ) // x-z plane pi/4
        {
          std::cout<<"choose the device on the right side"<< std::endl;
          dir = right;
          break;
        }
        else if((xzAngle>0.7854f && arm_unitVector_.x <0)){
          std::cout<<"choose the device on the left side" << std::endl;
          dir = left;
          break;
        }
        else if(xzAngle < 0.7854f ){
          std::cout<<"choose the device in the middle" << std::endl;
          dir = middle;
          break;
        }
        else{
          std::cout<<"Please point to a device" << std::endl;
          dir = none;
          break;}
      }
    }

  }

  bool PointedObject::object_existance(){
    //std::cout <<"at object_existance"<<object.size()<<std::endl;
    if(!object.empty()){
      return true;

    }
    else
      return false;
  }


