// This file is part of the Orbbec Astra SDK [https://orbbec3d.com]
// Copyright (c) 2015-2017 Orbbec 3D
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
//Modified by Zhebin Jiang@TECO KIT


#include <SFML/Graphics.hpp>
#include <astra/astra.hpp>
#include <iostream>
#include <cstring>

#include <mqtt/message.h>
#include <jsoncpp/json/value.h>
#include <jsoncpp/json/reader.h>
#include <jsoncpp/json/writer.h>
#include <mqtt/async_client.h>

#include <chrono>
#include <thread>
#include <string>
#include <cstdlib>
#include <ctime>


#include <pointed_object.h>
#include <Utilities.h>

static std::vector<sf::Vector2f> object_position(5);
static int every_n_message {5};
static std::string broker_address{"broker.hivemq.com"};

class sfLine : public sf::Drawable
{
public:
    sfLine(const sf::Vector2f& point1, const sf::Vector2f& point2, sf::Color color, float thickness)
        : color_(color)
    {
        const sf::Vector2f direction = point2 - point1;
        const sf::Vector2f unitDirection = direction / std::sqrt(direction.x*direction.x + direction.y*direction.y);
        const sf::Vector2f normal(-unitDirection.y, unitDirection.x);

        const sf::Vector2f offset = (thickness / 2.f) * normal;

        vertices_[0].position = point1 + offset;
        vertices_[1].position = point2 + offset;
        vertices_[2].position = point2 - offset;
        vertices_[3].position = point1 - offset;

        for (int i = 0; i<4; ++i)
            vertices_[i].color = color;
    }

    void draw(sf::RenderTarget &target, sf::RenderStates states) const
    {
        target.draw(vertices_, 4, sf::Quads, states);
    }

private:
    sf::Vertex vertices_[4];
    sf::Color color_;
};

class Publisher 
{
  //Publisher
public:

//  bool pubflag{false};
  std::string ADDRESS_ {"broker.hivemq.com"};
  std::string CLIENTID_ {"Turtlebot"};
  int MAX_BUFFERED_MSGS_ =100;
  std::string PERSIST_DIR_ {"data-persist"};
  int QOS_=0;
  std::string TOPIC {"intensechoi/camera/get"};
  //std::string TOPIC_2 {"intensechoi/position"};
  mqtt::async_client cli;
  mqtt::connect_options connOpts;
  mqtt::topic top_get;
//  mqtt::topic top_position;


  Publisher()
    :cli(broker_address,CLIENTID_,MAX_BUFFERED_MSGS_,PERSIST_DIR_),

    top_get(cli,TOPIC, QOS_, true)
  //  top_position(cli,TOPIC,QOS_,true)
  {
    const auto PERIOD = std::chrono::seconds(5);

    connOpts.set_keep_alive_interval(MAX_BUFFERED_MSGS_ * PERIOD);
    connOpts.set_clean_session(true);
    connOpts.set_automatic_reconnect(true);
    cli.connect(connOpts)->wait();
  }

  Publisher(std::string ADDRESS, std::string CLIENTID,int MAX_BUFFERED_MSGS,
            std::string PERSIST_DIR,int QOS, std::string TOPIC)
:   ADDRESS_{ADDRESS},
    CLIENTID_{CLIENTID},
    PERSIST_DIR_{PERSIST_DIR},
    QOS_ {QOS},
    TOPIC{TOPIC},
    //TOPIC_2{TOPIC2},
    cli(ADDRESS,CLIENTID,MAX_BUFFERED_MSGS,PERSIST_DIR),
    top_get(cli,TOPIC, QOS, true)
   // top_position(cli,TOPIC2, QOS, true)
  {
    const auto PERIOD = std::chrono::seconds(5);
    mqtt::connect_options connOpts;

    connOpts.set_keep_alive_interval(MAX_BUFFERED_MSGS * PERIOD);
    connOpts.set_clean_session(true);
    connOpts.set_automatic_reconnect(true);


//    std::cout<< "connecting to server" << ADDRESS << "....." <<std::flush;
    cli.connect(connOpts)->wait();
//    std::cout << "Connected!\n" << std::endl;
//    pubflag=true;
    
  }

  ~Publisher(){}
};

//TODO!! write the subscriber to subscribe to location of the object
//TODO!! figure out the timestamp

class action_listener : public virtual mqtt::iaction_listener
{
  std::string name_;

  void on_failure(const mqtt::token& tok) override {
    std::cout << name_ << " failure";
    if (tok.get_message_id() != 0)
      std::cout << " for token: [" << tok.get_message_id() << "]" << std::endl;
    std::cout << std::endl;
  }

  void on_success(const mqtt::token& tok) override {
    std::cout << name_ << " success";
    if (tok.get_message_id() != 0)
      std::cout << " for token: [" << tok.get_message_id() << "]" << std::endl;
    auto top = tok.get_topics();
    if (top && !top->empty())
      std::cout << "\ttoken topic: '" << (*top)[0] << "', ..." << std::endl;
    std::cout << std::endl;
  }

public:
  action_listener(const std::string& name) : name_(name) {}





};

class callback : public virtual mqtt::callback,
          public virtual mqtt::iaction_listener

{
  // Counter for the number of connection retries

  int nretry_;
  // The MQTT client
  mqtt::async_client& cli_;
  // Options to use if we need to reconnect
  mqtt::connect_options& connOpts_;
  // An action listener to display the result of actions.
  action_listener subListener_;

  // This deomonstrates manually reconnecting to the broker by calling
  // connect() again. This is a possibility for an application that keeps
  // a copy of it's original connect_options, or if the app wants to
  // reconnect with different options.
  // Another way this can be done manually, if using the same options, is
  // to just call the async_client::reconnect() method.
  void reconnect() {
    std::this_thread::sleep_for(std::chrono::milliseconds(2500));
    try {
      cli_.connect(connOpts_, nullptr, *this);
    }
    catch (const mqtt::exception& exc) {
      std::cerr << "Error: " << exc.what() << std::endl;
      exit(1);
    }
  }

  // Re-connection failure
  void on_failure(const mqtt::token& tok) override {
    std::cout << "Connection attempt failed" << std::endl;
    if (++nretry_ > N_RETRY_ATTEMPTS)
      exit(1);
    reconnect();
  }

  // (Re)connection success
  // Either this or connected() can be used for callbacks.
  void on_success(const mqtt::token& tok) override {}

  // (Re)connection success
  void connected(const std::string& cause) override {
    std::cout << "\nConnection success" << std::endl;
    std::cout << "\nSubscribing to topic '" << TOPIC_ << "'\n"
      << "\tfor client " << CLIENTID_
      << " using QoS" << QOS << "\n"
      << std::endl;

    cli_.subscribe(TOPIC_, QOS, nullptr, subListener_);
  }

  // Callback for when the connection is lost.
  // This will initiate the attempt to manually reconnect.
  void connection_lost(const std::string& cause) override {
    std::cout << "\nConnection lost" << std::endl;
    if (!cause.empty())
      std::cout << "\tcause: " << cause << std::endl;

    std::cout << "Reconnecting..." << std::endl;
    nretry_ = 0;
    reconnect();
  }

  // Callback for when a message arrives.
  void message_arrived(mqtt::const_message_ptr msg) override {
    std::cout << "Message arrived" << msg->to_string()<< std::endl;

    //parsing message received from mqtt
    Json::Value root;
    Json::Reader reader;
    bool parsingSuccessful =reader.parse(msg->to_string(),root);
    if(! parsingSuccessful)
    {
      std::cout << "Failed to parse"<< reader.getFormattedErrorMessages();
    }
    for(auto i=0 ;i<object_position.size();++i){
      if(!root["obj"+ std::to_string(i+1)].empty()){
        object_position.at(i).x= std::stod(root["obj"+ std::to_string(i+1)][0].toStyledString());
        object_position.at(i).y= std::stod(root["obj"+ std::to_string(i+1)][1].toStyledString());
      }
      else {
        std::cout<< "obj" << i+1 << " doesn't exist!"<< std::endl;
        object_position.at(i) = sf::Vector2f(0,0);
      }
    }
    std::cout<< object_position.at(0).x<< std::endl;
  }

  void delivery_complete(mqtt::delivery_token_ptr token) override {}

public:
  callback(mqtt::async_client& cli, mqtt::connect_options& connOpts)
        : nretry_(0), cli_(cli), connOpts_(connOpts), subListener_("Subscription") {}

private:
  const int QOS = 0;
  const std::string CLIENTID_ {"Turtlebot"};
  const std::string TOPIC_{"intensechoi/camera/set"};
  const int	N_RETRY_ATTEMPTS = 5;
};

class Subscriber{
public:
  const int QOS = 0;
  //const std::string ADDRESS_ {"broker.hivemq.com"};
  const std::string CLIENTID_ {"Turtlebot2"};
  const std::string TOPIC_{"intensechoi/camera/set"};
  mqtt::async_client cli_;
  mqtt::connect_options connOpts_;
  callback cb;

  Subscriber()
    :cli_(broker_address,CLIENTID_), cb(cli_, connOpts_)
  {
    connOpts_.set_keep_alive_interval(20);
    connOpts_.set_clean_session(true);

    cli_.set_callback(cb);
    cli_.connect(connOpts_,nullptr, cb);
  }


};



class BodyVisualizer : public astra::FrameListener
{
public:
  Publisher pub;

 /*****************************************/

    static sf::Color get_body_color(std::uint8_t bodyId)
    {
        if (bodyId == 0)
        {
            // Handle no body separately - transparent
            return sf::Color(0x00, 0x00, 0x00, 0x00);
        }
        // Case 0 below could mean bodyId == 25 or
        // above due to the "% 24".
        switch (bodyId % 24) {
        case 0:
            return sf::Color(0x00, 0x88, 0x00, 0xFF);
        case 1:
            return sf::Color(0x00, 0x00, 0xFF, 0xFF);
        case 2:
            return sf::Color(0x88, 0x00, 0x00, 0xFF);
        case 3:
            return sf::Color(0x00, 0xFF, 0x00, 0xFF);
        case 4:
            return sf::Color(0x00, 0x00, 0x88, 0xFF);
        case 5:
            return sf::Color(0xFF, 0x00, 0x00, 0xFF);

        case 6:
            return sf::Color(0xFF, 0x88, 0x00, 0xFF);
        case 7:
            return sf::Color(0xFF, 0x00, 0xFF, 0xFF);
        case 8:
            return sf::Color(0x88, 0x00, 0xFF, 0xFF);
        case 9:
            return sf::Color(0x00, 0xFF, 0xFF, 0xFF);
        case 10:
            return sf::Color(0x00, 0xFF, 0x88, 0xFF);
        case 11:
            return sf::Color(0xFF, 0xFF, 0x00, 0xFF);

        case 12:
            return sf::Color(0x00, 0x88, 0x88, 0xFF);
        case 13:
            return sf::Color(0x00, 0x88, 0xFF, 0xFF);
        case 14:
            return sf::Color(0x88, 0x88, 0x00, 0xFF);
        case 15:
            return sf::Color(0x88, 0xFF, 0x00, 0xFF);
        case 16:
            return sf::Color(0x88, 0x00, 0x88, 0xFF);
        case 17:
            return sf::Color(0xFF, 0x00, 0x88, 0xFF);

        case 18:
            return sf::Color(0xFF, 0x88, 0x88, 0xFF);
        case 19:
            return sf::Color(0xFF, 0x88, 0xFF, 0xFF);
        case 20:
            return sf::Color(0x88, 0x88, 0xFF, 0xFF);
        case 21:
            return sf::Color(0x88, 0xFF, 0xFF, 0xFF);
        case 22:
            return sf::Color(0x88, 0xFF, 0x88, 0xFF);
        case 23:
            return sf::Color(0xFF, 0xFF, 0x88, 0xFF);
        default:
            return sf::Color(0xAA, 0xAA, 0xAA, 0xFF);
        }
    }

    void init_depth_texture(int width, int height)
    {
        if (displayBuffer_ == nullptr || width != depthWidth_ || height != depthHeight_)
        {
            depthWidth_ = width;
            depthHeight_ = height;
            int byteLength = depthWidth_ * depthHeight_ * 4;

            displayBuffer_ = BufferPtr(new uint8_t[byteLength]);
            std::memset(displayBuffer_.get(), 0, byteLength);

            texture_.create(depthWidth_, depthHeight_);
            sprite_.setTexture(texture_, true);
            sprite_.setPosition(0, 0);
        }
    }

    void init_overlay_texture(int width, int height)
    {
        if (overlayBuffer_ == nullptr || width != overlayWidth_ || height != overlayHeight_)
        {
            overlayWidth_ = width;
            overlayHeight_ = height;
            int byteLength = overlayWidth_ * overlayHeight_ * 4;

            overlayBuffer_ = BufferPtr(new uint8_t[byteLength]);
            std::fill(&overlayBuffer_[0], &overlayBuffer_[0] + byteLength, 0);

            overlayTexture_.create(overlayWidth_, overlayHeight_);
            overlaySprite_.setTexture(overlayTexture_, true);
            overlaySprite_.setPosition(0, 0);
        }
    }

    void check_fps()
    {
        double fpsFactor = 0.02;

        std::clock_t newTimepoint= std::clock();
        long double frameDuration = (newTimepoint - lastTimepoint_) / static_cast<long double>(CLOCKS_PER_SEC);

        frameDuration_ = frameDuration * fpsFactor + frameDuration_ * (1 - fpsFactor);
        lastTimepoint_ = newTimepoint;
        double fps = 1.0 / frameDuration_;

       // printf("FPS: %3.1f (%3.4Lf ms)\n", fps, frameDuration_ * 1000);
    }

    void processDepth(astra::Frame& frame)
    {
        const astra::DepthFrame depthFrame = frame.get<astra::DepthFrame>();

        if (!depthFrame.is_valid()) { return; }

        int width = depthFrame.width();
        int height = depthFrame.height();

        init_depth_texture(width, height);

        const int16_t* depthPtr = depthFrame.data();
        for(int y = 0; y < height; y++)
        {
            for(int x = 0; x < width; x++)
            {
                int index = (x + y * width);
                int index4 = index * 4;

                int16_t depth = depthPtr[index];
                uint8_t value = depth % 255;

                displayBuffer_[index4] = value;
                displayBuffer_[index4 + 1] = value;
                displayBuffer_[index4 + 2] = value;
                displayBuffer_[index4 + 3] = 255;
            }
        }

        texture_.update(displayBuffer_.get());
    }

    void processBodies(astra::Frame& frame, mqtt::topic& topic)

    {
        astra::BodyFrame bodyFrame = frame.get<astra::BodyFrame>();

        std::vector<astra::Joint> joints;

        jointPositions_.clear();
        circles_.clear();
        circleShadows_.clear();
        boneLines_.clear();
        boneShadows_.clear();

        if (!bodyFrame.is_valid() || bodyFrame.info().width() == 0 || bodyFrame.info().height() == 0)
        {
            clear_overlay();
            return;
        }

        const float jointScale = bodyFrame.info().width() / 120.f;

        const auto& bodies = bodyFrame.bodies();

        for (auto& body : bodies)
        {
            printf("Processing frame #%d body %d left hand: %u\n",
                bodyFrame.frame_index(), body.id(), unsigned(body.hand_poses().left_hand()));
            for(auto& joint : body.joints())
            {
                jointPositions_.push_back(joint.depth_position());
                //std::cout<< "x Joint Position are" << joint.depth_position().x<<std::endl;
                 /* edited by Zhebin Jiang  */
                joints.push_back(joint);
                 /* edited by Zhebin Jiang  */
            }



            update_body(body, jointScale);
        }

        const auto& floor = bodyFrame.floor_info(); //floor
        if (floor.floor_detected())
        {
            const auto& p = floor.floor_plane();
           /* std::cout << "Floor plane: ["
                << p.a() << ", " << p.b() << ", " << p.c() << ", " << p.d()
                << "]" << std::endl; */

        }

        const auto& bodyMask = bodyFrame.body_mask();
        const auto& floorMask = floor.floor_mask();

        update_overlay(bodyMask, floorMask);


       // Calculate the direction of arm

        sf::Vector3f Rshoulder_wpos;
        sf::Vector3f Rwrist_wpos;
        sf::Vector3f RHdirection;
        sf::Vector3f Lshoulder_wpos;
        sf::Vector3f Lwrist_wpos;
        sf::Vector3f LHdirection;
        sf::Vector3f ShoulderSpine_wpos;
        sf::Vector3f MidSpine_wpos;
        sf::Vector3f z_unitVector(0,0,-1);

        enum armFlag{NoArm=0, Right,Left, TwoArms};
        armFlag arm=NoArm;

        Json::Value payload;
        Json::FastWriter fast;

        int ObjectIndex{0};

        for (const auto& joint : joints) {
          if(joint.type()==astra::JointType::RightShoulder){
            Rshoulder_wpos.x = joint.world_position().x;
            Rshoulder_wpos.y = joint.world_position().y;
            Rshoulder_wpos.z = joint.world_position().z;
          }
          if(joint.type()==astra::JointType::RightWrist){
            Rwrist_wpos.x = joint.world_position().x;
            Rwrist_wpos.y = joint.world_position().y;
            Rwrist_wpos.z = joint.world_position().z;
          }
          if(joint.type()==astra::JointType::LeftShoulder){
            Lshoulder_wpos.x = joint.world_position().x;
            Lshoulder_wpos.y = joint.world_position().y;
            Lshoulder_wpos.z = joint.world_position().z;
          }

          if(joint.type()==astra::JointType::LeftWrist){
            Lwrist_wpos.x = joint.world_position().x;
            Lwrist_wpos.y = joint.world_position().y;
            Lwrist_wpos.z = joint.world_position().z;
          }
          if(joint.type()==astra::JointType::ShoulderSpine){
            ShoulderSpine_wpos.x = joint.world_position().x;
            ShoulderSpine_wpos.y = joint.world_position().y;
            ShoulderSpine_wpos.z = joint.world_position().z;
          }
          if(joint.type()==astra::JointType::MidSpine){
            MidSpine_wpos.x = joint.world_position().x;
            MidSpine_wpos.y = joint.world_position().y;
            MidSpine_wpos.z = joint.world_position().z;
          }
        }


        //calculate xz-Angle and xy_angle of each arm
        sf::Vector3f RHunitDirection = Utilities().UnitVector(Rwrist_wpos , Rshoulder_wpos);
        sf::Vector3f LHunitDirection = Utilities().UnitVector(Lwrist_wpos, Lshoulder_wpos);

        float xzAngle_R = Utilities().xz_angle(RHunitDirection);
        float xzAngle_L = Utilities().xz_angle(LHunitDirection);

        float xyAngle_R = Utilities().xy_angle(RHunitDirection);
        float xyAngle_L = Utilities().xy_angle(LHunitDirection);


        if(bodyFrame.bodies().size()==1){ //can only recognize one body
          // choose which arm to be the control arm
          if(xyAngle_R > 0.7f && xyAngle_L < 0.7f){
            std::cout << "choose right arm as the control arm"<<std::endl;
            arm = Right;

          }
          else if(xyAngle_L>0.7f && xyAngle_R < 0.7f){
            std::cout << "choose left arm as the control arm"<< std::endl;
            arm = Left;

          }
          else if(xyAngle_L>0.7f && xyAngle_R >0.7f){
            arm = TwoArms;
          }
          else {
            arm = NoArm;
          }
         // call functions to detect which object is pointed by arm
          switch(arm){
          case Right:{
            PointedObject pointed_objectR(MidSpine_wpos,RHunitDirection,object_position.at(0), object_position.at(1),object_position.at(2),object_position.at(3),object_position.at(4));
              if(pointed_objectR.object_existance()){
                if(pointed_objectR.BoundaryTest())
                  ObjectIndex = pointed_objectR.whichobject();
                else
                  std::cout<<"You are not pointing to the given object!"<< std::endl;
              }
              else {
                pointed_objectR.absolute_angle_method(arm,xzAngle_R,xzAngle_L);
              }

              payload["result"] ="Object"+ std::to_string(ObjectIndex);
              payload["timestamp"] = Utilities().time_now();
          //    payload["timestamp"] = std::chrono::system_clock::now();
              std::string serialized = fast.write(payload);
      //        message->set_payload(serialized.c_str());
              if(every_n_message==0)
                topic.publish(serialized.c_str());
              else {
                std::cout<<every_n_message<<" left before publish a message"<< std::endl;
              }
              break;
          }
          case Left:{           
            PointedObject pointed_objectL(MidSpine_wpos, LHunitDirection,object_position.at(0), object_position.at(1),object_position.at(2),object_position.at(3),object_position.at(4));
              if(pointed_objectL.object_existance()){
                if(pointed_objectL.BoundaryTest())
                  ObjectIndex=pointed_objectL.whichobject();
                else
                  std::cout<<"You are not pointing to the given object!"<< std::endl;
              }
              else {
                pointed_objectL.absolute_angle_method(arm,xzAngle_R, xzAngle_L);
              }

              payload["result"] = "Object" + std::to_string(ObjectIndex);
              payload["timestamp"] = Utilities().time_now();
              std::string serialized = fast.write(payload);
              if(every_n_message==0)
                topic.publish(serialized.c_str());
              else {
                std::cout<<every_n_message<<" left before publish a message"<< std::endl;
              }
              break;
          }
          case NoArm:
          case TwoArms:
          {
            std::cout << "There are no or more than one person in the frame" << std::endl;
            break;
          }

          }
     }
    }

    void update_body(astra::Body body,
                     const float jointScale)


    {
        const auto& joints = body.joints();

        if (joints.empty())
        {
            return;
        }

        for (const auto& joint : joints)
        {
            astra::JointType type = joint.type();
            const auto& pos = joint.depth_position();

            if (joint.status() == astra::JointStatus::NotTracked)
            {
                continue;
            }

            auto radius = jointRadius_ * jointScale; // pixels
            sf::Color circleShadowColor(0, 0, 0, 255);

             auto color = sf::Color(0x00, 0xFF, 0x00, 0xFF);

            if ((type == astra::JointType::LeftHand && astra::HandPose::Grip==body.hand_poses().left_hand()) ||
                (type == astra::JointType::RightHand &&  astra::HandPose::Grip==body.hand_poses().right_hand()))
            {
                radius *= 1.5f;
                circleShadowColor = sf::Color(255, 255, 255, 255);
                color = sf::Color(0x00, 0xAA, 0xFF, 0xFF);
            }

            const auto shadowRadius = radius + shadowRadius_ * jointScale;
            const auto radiusDelta = shadowRadius - radius;

            sf::CircleShape circle(radius);

            circle.setFillColor(sf::Color(color.r, color.g, color.b, 255));
            circle.setPosition(pos.x - radius, pos.y - radius);
            circles_.push_back(circle);

            sf::CircleShape shadow(shadowRadius);
            shadow.setFillColor(circleShadowColor);
            shadow.setPosition(circle.getPosition() - sf::Vector2f(radiusDelta, radiusDelta));
            circleShadows_.push_back(shadow);
        }

        update_bone(joints, jointScale, astra::JointType::Head, astra::JointType::Neck);
        update_bone(joints, jointScale, astra::JointType::Neck, astra::JointType::ShoulderSpine);

        update_bone(joints, jointScale, astra::JointType::ShoulderSpine, astra::JointType::LeftShoulder);
        update_bone(joints, jointScale, astra::JointType::LeftShoulder, astra::JointType::LeftElbow);
        update_bone(joints, jointScale, astra::JointType::LeftElbow, astra::JointType::LeftWrist);
        update_bone(joints, jointScale, astra::JointType::LeftWrist, astra::JointType::LeftHand);

        update_bone(joints, jointScale, astra::JointType::ShoulderSpine, astra::JointType::RightShoulder);
        update_bone(joints, jointScale, astra::JointType::RightShoulder, astra::JointType::RightElbow);
        update_bone(joints, jointScale, astra::JointType::RightElbow, astra::JointType::RightWrist);
        update_bone(joints, jointScale, astra::JointType::RightWrist, astra::JointType::RightHand);

        update_bone(joints, jointScale, astra::JointType::ShoulderSpine, astra::JointType::MidSpine);
        update_bone(joints, jointScale, astra::JointType::MidSpine, astra::JointType::BaseSpine);

        update_bone(joints, jointScale, astra::JointType::BaseSpine, astra::JointType::LeftHip);
        update_bone(joints, jointScale, astra::JointType::LeftHip, astra::JointType::LeftKnee);
        update_bone(joints, jointScale, astra::JointType::LeftKnee, astra::JointType::LeftFoot);

        update_bone(joints, jointScale, astra::JointType::BaseSpine, astra::JointType::RightHip);
        update_bone(joints, jointScale, astra::JointType::RightHip, astra::JointType::RightKnee);
        update_bone(joints, jointScale, astra::JointType::RightKnee, astra::JointType::RightFoot);

    }

    void update_bone(const astra::JointList& joints,
                     const float jointScale,astra::JointType j1,
                     astra::JointType j2)
    {
        const auto& joint1 = joints[int(j1)];
        const auto& joint2 = joints[int(j2)];

        if (joint1.status() == astra::JointStatus::NotTracked ||
            joint2.status() == astra::JointStatus::NotTracked)
        {
            //don't render bones between untracked joints
            return;
        }

        //actually depth position, not world position
        const auto& jp1 = joint1.depth_position();
        const auto& jp2 = joint2.depth_position();

        auto p1 = sf::Vector2f(jp1.x, jp1.y);
        auto p2 = sf::Vector2f(jp2.x, jp2.y);

        sf::Color color(255, 255, 255, 255);
        float thickness = lineThickness_ * jointScale;
        if (joint1.status() == astra::JointStatus::LowConfidence ||
            joint2.status() == astra::JointStatus::LowConfidence)
        {
            color = sf::Color(128, 128, 128, 255);
            thickness *= 0.5f;
        }

        boneLines_.push_back(sfLine(p1,
            p2,
            color,
            thickness));
        const float shadowLineThickness = thickness + shadowRadius_ * jointScale * 2.f;
        boneShadows_.push_back(sfLine(p1,
            p2,
            sf::Color(0, 0, 0, 255),
            shadowLineThickness));
    }

    void update_overlay(const astra::BodyMask& bodyMask,
                        const astra::FloorMask& floorMask)
    {
        const auto* bodyData = bodyMask.data();
        const auto* floorData = floorMask.data();
        const int width = bodyMask.width();
        const int height = bodyMask.height();

        init_overlay_texture(width, height);

        const int length = width * height;

        for (int i = 0; i < length; i++)
        {
            const auto bodyId = bodyData[i];
            const auto isFloor = floorData[i];

            sf::Color color(0x0, 0x0, 0x0, 0x0);

            if (bodyId != 0)
            {
                color = get_body_color(bodyId);
            }
            else if (isFloor != 0)
            {
                color = sf::Color(0x0, 0x0, 0xFF, 0x88);
            }

            const int rgbaOffset = i * 4;
            overlayBuffer_[rgbaOffset] = color.r;
            overlayBuffer_[rgbaOffset + 1] = color.g;
            overlayBuffer_[rgbaOffset + 2] = color.b;
            overlayBuffer_[rgbaOffset + 3] = color.a;
        }

        overlayTexture_.update(overlayBuffer_.get());
    }

    void clear_overlay()
    {
        int byteLength = overlayWidth_ * overlayHeight_ * 4;
        std::fill(&overlayBuffer_[0], &overlayBuffer_[0] + byteLength, 0);

        overlayTexture_.update(overlayBuffer_.get());
    }

    virtual void on_frame_ready(astra::StreamReader& reader,
                                astra::Frame& frame) override
    //for listener, is called when a new frame of a specific type is ready for processing
    {
      if(every_n_message==0)
        every_n_message = 5;
      else
        every_n_message -- ;

      processDepth(frame);
      processBodies(frame,pub.top_get);

      check_fps();
    }

    void draw_bodies(sf::RenderWindow& window)
    {
        const float scaleX = window.getView().getSize().x / overlayWidth_;
        const float scaleY = window.getView().getSize().y / overlayHeight_;

        sf::RenderStates states;
        sf::Transform transform;
        transform.scale(scaleX, scaleY);
        states.transform *= transform;

        for (const auto& bone : boneShadows_)
            window.draw(bone, states);

        for (const auto& c : circleShadows_)
            window.draw(c, states);

        for (const auto& bone : boneLines_)
            window.draw(bone, states);

        for (auto& c : circles_)
            window.draw(c, states);

    }

    void draw_to(sf::RenderWindow& window)
    {
        if (displayBuffer_ != nullptr)
        {
            const float scaleX = window.getView().getSize().x / depthWidth_;
            const float scaleY = window.getView().getSize().y / depthHeight_;
            sprite_.setScale(scaleX, scaleY);

            window.draw(sprite_); // depth
        }

        if (overlayBuffer_ != nullptr)
        {
            const float scaleX = window.getView().getSize().x / overlayWidth_;
            const float scaleY = window.getView().getSize().y / overlayHeight_;
            overlaySprite_.setScale(scaleX, scaleY);
            window.draw(overlaySprite_); //bodymask and floormask
        }

        draw_bodies(window);
    }

private:
    long double frameDuration_{ 0 };
    std::clock_t lastTimepoint_ { 0 };
    sf::Texture texture_;
    sf::Sprite sprite_;

    using BufferPtr = std::unique_ptr < uint8_t[] >;
    BufferPtr displayBuffer_{ nullptr };

    std::vector<astra::Vector2f> jointPositions_;

    int depthWidth_{0};
    int depthHeight_{0};
    int overlayWidth_{0};
    int overlayHeight_{0};

    std::vector<sfLine> boneLines_;
    std::vector<sfLine> boneShadows_;
    std::vector<sf::CircleShape> circles_;
    std::vector<sf::CircleShape> circleShadows_;

    float lineThickness_{ 0.5f }; // pixels
    float jointRadius_{ 1.0f };   // pixels
    float shadowRadius_{ 0.5f };  // pixels

    BufferPtr overlayBuffer_{ nullptr };
    sf::Texture overlayTexture_;
    sf::Sprite overlaySprite_;

};

astra::DepthStream configure_depth(astra::StreamReader& reader)
{
    auto depthStream = reader.stream<astra::DepthStream>();

    //We don't have to set the mode to start the stream, but if you want to here is how:
    astra::ImageStreamMode depthMode;

    depthMode.set_width(640);
    depthMode.set_height(480);
    depthMode.set_pixel_format(astra_pixel_formats::ASTRA_PIXEL_FORMAT_DEPTH_MM);
    depthMode.set_fps(30);

    depthStream.set_mode(depthMode);

    return depthStream;
}

int main(int argc, char** argv)
{

    astra::initialize();

 //   const char* licenseString = "<INSERT LICENSE KEY HERE>";
 //   orbbec_body_tracking_set_license(licenseString);

    sf::RenderWindow window(sf::VideoMode(1280, 960), "Gesture Recognition");

#ifdef _WIN32
    auto fullscreenStyle = sf::Style::None;
#else
    auto fullscreenStyle = sf::Style::Fullscreen;
#endif

    const sf::VideoMode fullScreenMode = sf::VideoMode::getFullscreenModes()[0];
    const sf::VideoMode windowedMode(1280, 960);
    bool isFullScreen = false;
//get data from the sensor
    astra::StreamSet sensor;
    astra::StreamReader reader = sensor.create_reader();
    Publisher pub;
    Subscriber sub;

    BodyVisualizer listener;


    
    auto depthStream = configure_depth(reader);
    depthStream.start();

    auto bodyStream = reader.stream<astra::BodyStream>();
    bodyStream.start();
    reader.add_listener(listener);

    astra::SkeletonProfile profile = bodyStream.get_skeleton_profile();

    // HandPoses includes Joints and Segmentation
    astra::BodyTrackingFeatureFlags features = astra::BodyTrackingFeatureFlags::HandPoses;

    while (window.isOpen())
    {
        astra_update();

        sf::Event event;
        while (window.pollEvent(event))
        {
            switch (event.type)
            {
            case sf::Event::Closed:
                window.close();
                break;
            case sf::Event::KeyPressed:
            {
                if (event.key.code == sf::Keyboard::C && event.key.control)
                {
                    window.close();
                }
                switch (event.key.code)
                {
                case sf::Keyboard::D:
                {
                    auto oldMode = depthStream.mode();
                    astra::ImageStreamMode depthMode;

                    depthMode.set_width(640);
                    depthMode.set_height(400);
                    depthMode.set_pixel_format(astra_pixel_formats::ASTRA_PIXEL_FORMAT_DEPTH_MM);
                    depthMode.set_fps(30);

                    depthStream.set_mode(depthMode);
                    auto newMode = depthStream.mode();
                    printf("Changed depth mode: %dx%d @ %d -> %dx%d @ %d\n",
                           oldMode.width(), oldMode.height(), oldMode.fps(),
                           newMode.width(), newMode.height(), newMode.fps());
                    break;
                }
                case sf::Keyboard::Escape:
                    window.close();
                    break;
                case sf::Keyboard::F:
                    if (isFullScreen)
                    {
                        window.create(windowedMode, "Gesture Recognition", sf::Style::Default);
                    }
                    else
                    {
                        window.create(fullScreenMode, "Gesture Recognition", fullscreenStyle);
                    }
                    isFullScreen = !isFullScreen;
                    break;
                case sf::Keyboard::R:
                    depthStream.enable_registration(!depthStream.registration_enabled());
                    break;
                case sf::Keyboard::M:
                    depthStream.enable_mirroring(!depthStream.mirroring_enabled());
                    break;
                case sf::Keyboard::P:
                    if (profile == astra::SkeletonProfile::Full)
                    {
                        profile = astra::SkeletonProfile::Basic;
                        printf("Skeleton Profile: basic\n");
                    }
                    else
                    {
                        profile = astra::SkeletonProfile::Full;
                        printf("Skeleton Profile: full\n");
                    }
                    bodyStream.set_skeleton_profile(profile);
                    break;
                case sf::Keyboard::T:
                    if (features == astra::BodyTrackingFeatureFlags::Segmentation)
                    {
                        // Joints includes Segmentation
                        features = astra::BodyTrackingFeatureFlags::Joints;
                        printf("Default Body Features: Seg+Body\n");
                    }
                    else if (features == astra::BodyTrackingFeatureFlags::Joints)
                    {
                        // HandPoses includes Joints and Segmentation
                        features = astra::BodyTrackingFeatureFlags::HandPoses;
                        printf("Default Body Features: Seg+Body+Hand\n");
                    }
                    else
                    {
                        // HandPoses includes Joints and Segmentation
                        features = astra::BodyTrackingFeatureFlags::Segmentation;
                        printf("Default Body Features: Seg\n");
                    }
                    bodyStream.set_default_body_features(features);
                    break;
                default:
                    break;
                }
                break;
            }
            default:
                break;
            }
        }

        // clear the window with black color
        window.clear(sf::Color::Black);

        listener.draw_to(window);
        window.display();
    }

    astra::terminate();

    return 0;
}
