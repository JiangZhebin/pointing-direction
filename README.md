# gesture_recognition

This project uses Orbbec Astra SDK to detect arm gesture to determin which object the user is pointing to.

**If license expires please download the latest sdk from Orbbec.**

**Broker Address**: broker.hivemq.com

**Subscribe to topic**: intensechoi/camera/set    format: {"obj1":[x,y],"obj2":[x,y],....} max. 5 objects

**Publish to topic**: intensechoi/camera/get      format: {"result":ObjectX , "timestamp":xxx}
  


 ## Activity Graph
![Activity graph for this project](gesture_recognition_uml.jpg)
