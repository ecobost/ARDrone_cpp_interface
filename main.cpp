// Written by: Erick Cobos
// Date: 10-Nov-2015

#include "ARDrone/ARDrone.h"

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

//An example of how to use the ARDrone class
int main(){
    ARDrone myARDrone;

    // Flying demo
    myARDrone.takeoff();

    // Moves in a square.
    myARDrone.moveAhead(100);
    myARDrone.turnLeft();
    myARDrone.moveAhead(100);
    myARDrone.turnLeft();
    myARDrone.moveAhead(100);
    myARDrone.turnLeft();
    myARDrone.moveAhead(100);
    myARDrone.moveUp(100);
    cv::Mat horizontalImage = myARDrone.getImage();
    myARDrone.switchCamera();
    cv::Mat verticalImage = myARDrone.getImage();
    myARDrone.moveDown(100);
    myARDrone.land();

    cv::imshow("Horizontal", horizontalImage );
    cv::waitKey(0);
    cv::imshow("Vertical", verticalImage);
    cv::waitKey(0);

    myARDrone.printStats();
}
