/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class should be used to define all the specific hardware for a single robot.  Care should be
 * taken to only define the hardware that will exist on the robot.   Vestigial defenitions for
 * sensors and motors should be avoided.   NOTE:  The initialization of certain sensors and
 * components that are only used in AUTONOMOUS code will be controlled by boolean flags.   This is
 * to prevent computationally and time expensive initialization code from running during driver
 * modes.
 *
 * PARAMETERS
 *      HardwareMap aHwMap          -- The hardware map object that is to be initialized.   Defined
 *                                     seperately in each opMode()
 *      boolean useIMU              -- If set to true initializes the IMU so references like heading
 *                                     can be used.  Required for the Drive() class to function
 *                                     properly.   Generally set to true for autonomous code and
 *                                     false for driver code.
 *      boolean useVision           -- If set to true will initialize the vision recognition
 *                                     software.   Computationally expensive and battery expensive.
 *                                     Generally should only be used in autonomous code.  Likewise
 *                                     the vision software should be shutdown as soon as it is down.
 *      boolean useExternalCamera   -- If set to true and useVision is true enables an external USB
 *                                     camera instead of the built in camera.
 *
 *
 *
 * MAIN REV EXPANSION HUB
 *    MOTOR 0   --  left
 *    MOTOR 1   --  right
 *    I2C   0  --   imu             --  Device of IMU  type to enable use of gyro.
 *
 * SECONDARY REV EXPANSION HUB
 *
 */
public class HardwareDefinitionJK2019
{
    /* RETURN CODES */
    public enum STATUS {SUCCESS, IMU_FAIL, VISION_FAIL, };

    /* MOTORS */
    public DcMotor  left  = null;
    public DcMotor  right = null;

    /*SERVOS*/
    //public Servo    exampleServo = null;
    //public CRServo  exampleCRServo = null;

    public CRServo extension = null;

    /* SENSORS */
    public BNO055IMU imu = null;
    //public ColorSensor exampleColor = null;
    //public TouchSensor exampleTouch = null;
    //public DistanceSensor exampleDistance = null;
    //public AnalogInput  exampleAnalog = null;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();


    /* Constructor */
    public HardwareDefinitionJK2019(){

    }

    /* Initialize standard Hardware interfaces */
    public STATUS init(HardwareMap ahwMap, boolean useIMU, boolean useVision, boolean useExternalCamera) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        STATUS retCode = STATUS.SUCCESS;

        //Define and Initialize MOTORS
        left   = hwMap.get(DcMotor.class, "leftMotor");
        right  = hwMap.get(DcMotor.class, "rightMotor");

        //Take care with direction configuration.   Make sure the concept of front and rear is the
        //same between autonomous code and driver code.
        left.setDirection(DcMotor.Direction.REVERSE);
        right.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        left.setPower(0);
        right.setPower(0);

        //Setting the mode can be a little on the tricky side.   Different manufacturers motors
        //have different behaviours around their encoders based on how the code is initialized.
        //Presently STOP_AND_RESET followed by RUN_WITHOUT seems to get REV, AndyMark, and GoBilda
        //all into the correct state.
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //Define and Initialize SERVOS
        //exampleServo = hwMap.get(Servo.class, "myServo");
        //exampleCRServo = hwMap.get(CRServo.class, "myContinuousServo");
        extension = hwMap.get(CRServo.class, "extension");
        extension.setDirection(CRServo.Direction.FORWARD);
        extension.setPower(0.5);
        //exampleServo.setDirection(Servo.Direction.FORWARD);
        //exampleServo.setPosition(0.0);
        //exampleCRServo.setPower(0.0);  // Continuous Rotation servo has no concept of position

        //Define and Initialize SENSORS
        //exampleColor = hwMap.get(ColorSensor.class, "someRandomName");
        //exampleTouch = hwMap.get(TouchSensor.class, "touchMe");
        //exampleDistance = hwMap.get(DistanceSensor.class, "farFarAway");
        //exampleAnalog = hwMap.get(AnalogInput.class, "myPotentiometer");

        //exampleColor.enableLed(true);



        //ONLY INITIALIZE IMU IF REQUESTED
        // TODO  Add stop detection in gyro init() loop
        if (useIMU) {
            imu = hwMap.get(BNO055IMU.class, "imu");
            BNO055IMU.Parameters Params = new BNO055IMU.Parameters();
            Params = imu.getParameters();
            Params.mode = BNO055IMU.SensorMode.IMU;
            Params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            Params.loggingEnabled = false;
            imu.initialize(Params);
            int i;
            for (i = 0; i < 60; i++) {
                if (imu.isGyroCalibrated()) {
                    break;
                }
                SystemClock.sleep(100);
            }
            if (!imu.isGyroCalibrated()) {
                retCode = STATUS.IMU_FAIL;
            }

        }

        //ONLY INITIALIZE VISION IF REQUESTED
        if (useVision) {
            if (useExternalCamera) {

            }
            else {

            }
        }

        return (retCode);
    }
 }

