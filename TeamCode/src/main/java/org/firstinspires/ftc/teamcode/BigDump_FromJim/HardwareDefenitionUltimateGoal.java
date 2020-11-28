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
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
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
 * MAIN REV EXPANSION HUB
 *    MOTOR 0   --  left_front      --  Left front drive motor
 *    MOTOR 1   --  right_front     --  Right front drive motor
 *    MOTOR 2   --  left_rear       --  Left rear drive motor
 *    MOTOR 3   --  right_rear      --  Right rear drive motor
 *
 *    I2C   0  --   imu             --  Device of IMU  type to enable use of gyro.
 *
 *
 * SECONDARY REV EXPANSION HUB
 *   MOTOR 0   --  lift             --  Lift motor used to expand robot to full height.
 */
public class HardwareDefenitionUltimateGoal
{
    /* RETURN CODES */
    public enum STATUS {SUCCESS, IMU_FAIL, VISION_FAIL, };
    /* MOTORS */
    public DcMotor  shooterTop  = null;
    public DcMotor  shooterBottom = null;


    /*SERVOS*/
    public Servo   shooterTrigger = null;

    public BNO055IMU imu = null;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();


    /* Constructor */
    public HardwareDefenitionUltimateGoal(){

    }

    /* Initialize standard Hardware interfaces */
    public STATUS init(HardwareMap ahwMap, boolean useIMU, boolean useVision, boolean useExternalCamera) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        STATUS retCode = STATUS.SUCCESS;

        //Define and Initialize MOTORS
        shooterTop     = hwMap.get(DcMotor.class, "top");
        shooterBottom  = hwMap.get(DcMotor.class, "bottom");



        //Take care with direction configuration.   Make sure the concept of front and rear is the
        //same between autonomous code and driver code.
        shooterTop.setDirection(DcMotor.Direction.FORWARD);
        shooterBottom.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        shooterTop.setPower(0);
        shooterBottom.setPower(0);



        // SERVO
        shooterTrigger = hwMap.get(Servo.class, "trigger");

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

