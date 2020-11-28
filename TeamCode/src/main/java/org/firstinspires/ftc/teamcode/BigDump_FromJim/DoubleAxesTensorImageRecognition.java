/*
Built for me because I felt like it

James K Rumsey
03/05/2019

OVERVIEW:
Provides methods for image recognition using a webcam mounted in a dual servo assembly.   One servo
will provide rotation on the X/Y plane which is parallel to the playing field.  The other servo will
pivot the camera up and down in the Z plane.   This will effectively change the field of view on the
X/Y plane.   Image recognition will be done using the Vuforia package along with Tensor objects.
The tensor objects must be trained offline and then supplied to the project.

This class will track the camera position on both axes of rotation.   Thus the relative bearing of
the camera against the heading of the robot will always be known.   With the image recognition code
providing object distance and bearing information it will be possible to approximate the relative
distance and bearing of a recognized object from the robot.   This information can then be used to
move/position the robot by the recognized object, use an actuator on the object, etc.....


REQUIRED:

Servos    --  Two for positional control of the webcam
WebCam    --  A webcam mounted into the dual servo assembly.

ASSUMES:
Both rotational axes are free from obstruction over their defined range.
Absolute position is known on the axes control (servos).
Tensor object files are high fidelity.   It is likely the camera will see the object from many
    different perspectives.


PARAMETERS:

    LinearOpMode   opMode          -- The linear opMode to allow diagnostic messages.
    boolean        debug           -- Set to true to turn on diagnostic messages


STILL TO DO:



*/

package org.firstinspires.ftc.teamcode;

public class DoubleAxesTensorImageRecognition {
}
