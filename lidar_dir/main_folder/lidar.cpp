#include "lidar.h"


ILidarDriver* connectLidar(){

  ILidarDriver* lidar;
  IChannel* _channel;
	lidar = *createLidarDriver();
  _channel = (*createSerialPortChannel("/dev/ttyUSB0", 115200));
    if (SL_IS_OK((lidar)->connect(_channel))){
    	printf("Connected\n");

    	lidar->setMotorSpeed();
      //RplidarScanMode scanMode;
      //lidar->startScan(false, true, 0, &scanMode);
      return lidar;
    }
    printf("Connection failed\n");

    return NULL;

}

void disconnectLidar(ILidarDriver* lidar){
    lidar->stop();
    lidar->setMotorSpeed(0);
    delete lidar;
}
