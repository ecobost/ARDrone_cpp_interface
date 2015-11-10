// Written by: Erick Cobos
// Date: 31-Oct-2015

#ifndef ARDRONE_H
#define ARDRONE_H

#include <opencv2/core/core.hpp>

class ARDrone
{
    public:
        // Connects with the drone and opens communication channels.
        ARDrone();

        // Destructor: Disconnects the drone and stops transmission.
        ~ARDrone();

        // Resets the drone state and takes off (it hovers at 1 meter).
        void takeoff();

        // Lands the drone
        void land();

        // Moves up (higher) cm centimeters
        void moveUp(int cm);

        // Moves down (lower) cm centimeters
        void moveDown(int cm);

        // Advances cm centimeters
        void moveAhead(int cm);

        // Turns 90 degrees clockwise
        void turnRight();

        // Turns 90 degrees counter-clockwise
        void turnLeft();

        // Sends the specified commands to the drone.
        // This is a low-level function. Please use only if you know what you are doing.
        //  roll: Left-to-right tilt. From -1 (full leftwards) to 1 (full rightwards)
        //  pitch: Front-to-back tilt. From -1 (full frontwards) to 1 (full backwards)
        //  gaz: Vertical velocity (lift). From -1 (full downwards) to 1 (full upwards)
        //  yaw: Angular velocity (rotation). From -1 (full counter-clockwise) to 1 (full clockwise)
        //
        // IMPORTANT: When using this function, the drone will keep moving until stopAndHover() is called.
        // To avoid damaging your drone, make sure you call stopAndHover() some seconds after this function.
        void sendCommands(double roll, double pitch, double gaz, double yaw);

        // Stops movement in any direction and hovers in place.
        // This is a low-level function intended for use exclusively after sendCommands(). The other
        // functions (moveUp(), moveDown(), etc.) call it implicitly; thus, you do not need to.
        void stopAndHover();

        // Switch camera between horizontal and vertical. Default is horizontal.
        void switchCamera();

        // Takes a picture and returns it as an OpenCV Mat.
        cv::Mat getImage();

        // Get the amount of inclination from left-to-right. Range: -1 to 1
        // 0: flat (horizontal); 0.5: 90 degrees to the right (vertical); 1: 180 degrees to the right (drone on its back).
        // 0: flat (horizontal); -0.5: 90 degrees to the left (vertical); -1: 180 degrees to the left (drone on its back).
        double getRoll();

        // Get the amount of inclination from front-to-back. Range: -1 to 1
        // 0: flat (horizontal); 1: 90 degrees to the back (vertical); 0: 180 degrees to the back (drone on its back).
        // 0: flat (horizontal); -1: 90 degrees to the front (vertical); 0: 180 degrees to the front (drone on its back).
        double getPitch();

        // Get the amount of rotation. Range: -1 to 1
        // 0: original (pointing north); 0.5: 90 degrees counter-clockwise (pointing west); 1: 180 degrees counter-clockwise (poiniting south).
        // 0: original (pointing north); -0.5: 90 degrees clockwise (pointing east); -1: 180 degrees clockwise(pointing south).
        double getYaw();

        // Get the current altitude of the drone in cms. Ranges from 23 cms and up.
        // The sensor can only measure altitudes greater than 23 cms. If the drone is lower, it returns 23.
        int getAltitude();

        // Gets the percentage of remaining battery life (0-100)
        double getBatteryLife();

    protected:
    private:
        // Whether the drone is flying or not
        bool isFlying;

        // Current camera (0: horizontal, 1: vertical)
        int camera = 0;

        // Percentage of the maximum speed to which the drone is going to move (for every direction).
        // If changed, the respective function (moveUp(), moveDown(), moveAhead(), turnRight() or turnLeft())
        // should be recalibrated
        static constexpr double speedUp = 0.5;
        static constexpr double speedDown = 0.5;
        static constexpr double speedAhead = 0.15;
        static constexpr double speedRight = 0.5;
        static constexpr double speedLeft = 0.5;
};

#endif // ARDRONE_H
