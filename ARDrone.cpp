// Written by: Erick Cobos
// Date: 31-Oct-2015

/*
    Library to manage essential functions of the AR-Drone.
*/

#include "ARDrone.h"
#include "control/app.h"
#include <math.h>
#include <opencv2/core/core.hpp>

ARDrone::ARDrone(){
    // Connects with the drone and opens communication channels.
    appInit();

    // Selects the default camera
    at_zap(camera);
    usleep(250000);

    // Drone starts on the floor.
    isFlying = false;
}

ARDrone::~ARDrone(){
    // Disconnects the drone and stops transmission
    appDeinit();
}

void ARDrone::takeoff(){
    // Resets the state of the drone and takes off (it hovers at 1 meter).
    if(~isFlying){

        // Resets the state of the drone
        at_ui_reset();
        usleep(250000);

        // Recalibrates the sensors to the current flat surface.
        at_trim();
        usleep(250000);

        // Takes off (and waits 4.5 seconds until it stabilizes)
        at_ui_pad_start_pressed();
        usleep(4500000);

        // Resets communication
        at_comwdg();
        usleep(250000);

        isFlying = true;

        stopAndHover();
    }
}

void ARDrone:: land(){
    // Lands the drone
    if (isFlying){

        // Lands the drone
		at_ui_pad_start_pressed();
		usleep(250000);

		isFlying = false;
	}
}

void ARDrone::moveUp(int cm){
    // Moves up (higher) cm centimeters.

    // Centimeters travelled by the drone in a single second. Needs to be adjusted for each drone and external conditions.
    double distanceTravelledInOneSec = 28;
    double secondsNeeded = cm / distanceTravelledInOneSec;

    sendCommands(0, 0, speedUp, 0);
    usleep(secondsNeeded * 1000000);
    stopAndHover();
}

void ARDrone::moveDown(int cm){
    // Moves down (lower) cm centimeters

    // Centimeters travelled by the drone in a single second. Needs to be adjusted for each drone and external conditions.
    double distanceTravelledInOneSec = 48;
    double secondsNeeded = cm / distanceTravelledInOneSec;

    sendCommands(0, 0, -speedDown, 0);
    usleep(secondsNeeded * 1000000);
    stopAndHover();
}

void ARDrone::moveAhead(int cm){
    // Advances cm centimeters

    // Centimeters travelled by the drone in a single second. Needs to be adjusted for each drone and external conditions.
    //double distanceTravelledInOneSec = 78;
    //double secondsNeeded = cm / distanceTravelledInOneSec;

    // This function was calculated from a set of experiments with speedAhead = 0.15.
    // Needs to be adjusted for each drone and external conditions
    double secondsNeeded = log2(cm/100) + 1.415;

    sendCommands(0, -speedAhead, 0, 0);
    usleep(secondsNeeded * 1000000);
    stopAndHover();
}

void ARDrone::turnRight(){
    // Turns 90 degrees clockwise

    // The exact time needed to turn 90 degrees needs to be adjusted for each drone and external conditions.
    double secondsToTurn90Degrees = 1.99;

    sendCommands(0, 0, 0, speedRight);
    usleep(secondsToTurn90Degrees * 1000000);
    stopAndHover();
}

void ARDrone::turnLeft(){
    // Turns 90 degrees counter-clockwise

    // The exact time needed to turn 90 degrees needs to be adjusted for each drone and external conditions.
    double secondsToTurn90Degrees = 1.85;

    sendCommands(0, 0, 0, -speedLeft);
    usleep(secondsToTurn90Degrees * 1000000);
    stopAndHover();
}


void ARDrone::sendCommands(double roll, double pitch, double  gaz, double yaw){
    // Sends the specified commands to the drone. Read instructions in ARDrone.h

    // Implementation notes:
    // To use at_cmds.cpp without changing it (as a library), I had to make the following changes:
    // 1. at_set_radiogp_input() expects parameters in the -33000 to 33000 range, so I had to multiply
    //    each parameter by 33000.
    // 2. at_set_radiogp_input() changes the sign of gaz before storing it, so I add a negative sign here.
    // 3. at_set_radiogb_input() expects the arguments pitch, roll, gaz, yaw, hover (in that order) but
    //    send_command() mixes up pitch and roll before sending the commands, so I reverse the order here.
    at_set_radiogp_input(roll * 33000, pitch * 33000, -gaz * 33000, yaw * 33000, 0);
}

void ARDrone::stopAndHover(){
    // Stops movement in any direction and hovers in place. Read instructions in ARDrone.h
    at_set_radiogp_input(0, 0, 0, 0, 1);
    usleep(3000000);
}

void ARDrone::switchCamera(){
    // Switch camera between horizontal (0) and vertical (1). Default is horizontal.

    // Change camera variable
    camera = (camera + 1) % 2;

    // Change camera
    at_zap(camera);
    usleep(250000);
}

cv::Mat ARDrone::getImage(){
    // Takes a picture and returns it as an OpenCV Mat

    // Image dimensions. If the image is smaller than this, it fills the rest with black spaces.
    int imageWidth = 320;
    int imageHeight = 240;
    int channels = 3;
    int imageSize = imageWidth * imageHeight * channels;

    // Note: Code from here on is taken from Tom Krajnik. It is bad and he should feel bad.
    // I have tried to reorganize it and make it more readable but it is still quite ugly.

     // This array stores the image as a 24-bit RGB bitmap format.
    unsigned char* rawImage = (unsigned char*) calloc(imageSize, sizeof(unsigned char));

    // Create a pointer to the original picture (stored in picture_buf)
	unsigned char *picbuf = (unsigned char*)picture_buf;

    // If the picture buffer had a 320x240 image
	if (picture_width == 320){
		for(int i = 0; i < imageWidth * imageHeight; i++){
			rawImage[3*i] = (picbuf[2*i + 1]&0xf8);
			rawImage[3*i + 1] = ((picbuf[2*i + 1]&0x07) * 32) + ((picbuf[2*i]&0xe0) / 8);
			rawImage[3*i + 2] = (picbuf[2*i]&0x1f) * 8;
        }
	}
	else{
		for (int w = 0; w < picture_width; w++){
			for (int h = 0; h < picture_height; h++){
				rawImage[3*((h + 58)*imageWidth + w + 78) + 0] = (picbuf[2*(h*imageWidth + w) + 1]&0xf8);
				rawImage[3*((h + 58)*imageWidth + w + 78) + 1] = ((picbuf[2*(h*imageWidth + w) + 1]&0x07) * 32) + ((picbuf[2*(h*imageWidth + w)]&0xe0) / 8);
				rawImage[3*((h + 58)*imageWidth + w + 78) + 2] = (picbuf[2*(h*imageWidth + w)]&0x1f) * 8;
			}
		}
	}

    // Convert raw image to Mat
    cv::Mat result = cv::Mat(240, 320, CV_8UC3);

    uchar *pointerImage = result.ptr(0);
    for (int i = 0; i < 240*320; i++) {
		pointerImage[3*i] = rawImage[3*i + 2];
		pointerImage[3*i + 1] = rawImage[3*i + 1];
		pointerImage[3*i + 2] = rawImage[3*i];
	}

	delete rawImage;
	return result;
}

double ARDrone::getRoll(){
    // Get the amount of inclination from left-to-right. Ranges from -1 to 1.
    // 0: flat (horizontal); 0.5: 90 degrees to the right (vertical);, 1: 180 degrees to the right (drone on its back).
    // 0: flat (horizontal); -0.5: 90 degrees to the left (vertical);, -1: 180 degrees to the right (drone on its back).
    return helidata.phi/180000;
}

double ARDrone::getPitch(){
    // Get the amount of inclination from front-to-back. Ranges from -1 to 1.
    // 0: flat (horizontal); 1: 90 degrees to the back (vertical); 0: 180 degrees to the back (drone on its back).
    // 0: flat (horizontal); -1: 90 degrees to the front (vertical); 0: 180 degrees to the front (drone on its back).
    return helidata.theta/90000;
}

double ARDrone::getYaw(){
    // Get the amount of rotation. Range: -1 to 1
    // 0: original (pointing north); 0.5: 90 degrees counter-clockwise (pointing west); 1: 180 degrees counter-clockwise (poiniting south).
    // 0: original (pointing north); -0.5: 90 degrees clockwise (pointing east); -1: 180 degrees clockwise(pointing south).
    return helidata.psi/180000;
}

int ARDrone::getAltitude(){
    // Get the current altitude of the drone in cms. Ranges from 23 cms and up.
    // The sensor can only measure altitudes greater than 23 cms. If the drone is lower, it returns 23.
    return helidata.altitude/10;
}

double ARDrone::getBatteryLife(){
    // Gets the percentage of remaining battery life (0-100)
    return helidata.battery;
}

