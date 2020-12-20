package org.firstinspires.ftc.teamcode;

/* *
 *  Created by James Rumsey   3/31/19
 *  Updated                     9/12/19
 *
 *
 * *
 * OVERVIEW:   Driver mode code for a three sided omni wheel robot
 *
 * FORMAT:   It is important to keep in mind that this code operates in degrees and sets it frame of
 *           reference a bit different than the students might 'guess'
 *
 *           The center motor is in the 90 degree position
 *           The right motor is in the 0 degree position
 *           The left motor is in the 180 degree position
 *
 *           In theory it is possible for hte these motors to be in most any position as long as the
 *           corresponding hwMap angle information is correct.   In practice Non-90 degree angles
 *           give less than ideal performance.  (A function of how the omni wheels slip)
 *
 *
 */

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@SuppressWarnings("WeakerAccess")
@TeleOp(name = "Omni Drive H", group = "HardwarePushbot")
@Disabled
public class Driver_Omni_H extends LinearOpMode {
    private HardwareDefenitionOmni_H robot = new HardwareDefenitionOmni_H();
    private Drive robotDrive         = new Drive();
    private GamepadDrive gamepadDrive = new GamepadDrive();

    //Defenitions for rate at which each driver task executes.   Time is in milliseconds
    final long SENSORPERIOD = 20;
    final long SERVOPERIOD = 20;
    final long NAVPERIOD = 20;
    final long MOTORPERIOD = 20;
    final long CONTROLLERPERIOD = 20;
    final long TELEMETRYPERIOD = 500;

    public void runOpMode() {

        HardwareDefenitionOmni_H.STATUS retVal;

        /*
         * Initialize all of the robot hardware.
         * The init() method of the hardware class does all the work here
         */
        telemetry.addData("Status", "Initializing Robot...");
        telemetry.addData("Status", "Configuring Hardware...");
        telemetry.setAutoClear(false);
        telemetry.update();
        retVal = robot.init(hardwareMap, false, false, false);
        gamepad1.setJoystickDeadzone(0.05F);
        gamepad2.setJoystickDeadzone(0.05F);
        gamepadDrive.setZeroAxis(GamepadDrive.AXES.X_POS);
        telemetry.addData("      ", retVal);
        telemetry.update();

        // Initialize some of the timing variables
        long CurrentTime = System.currentTimeMillis();
        long LastSensor = CurrentTime;
        long LastEncoderRead = CurrentTime + 5;
        long LastServo = CurrentTime + 10;
        long LastNav = CurrentTime + 15;
        long LastMotor = CurrentTime + 20;
        long LastController = CurrentTime + 7;
        long LastTelemetry = CurrentTime + 17;

        double motorLeftCmd = 0.0;
        double motorRightCmd = 0.0;
        double motorCenterCmd = 0.0;
        final double turtleScalar = 0.35;
        final double fullSpeedScalar = 1.0;
        final double trimSpeed = 0.6;
        double magnitudeScalar = turtleScalar;

        double angleJoystick = 0.0;
        double magnitudeJoystick = 0.0;
        double angleHeading = 0.0;
        double angleActual = 0.0;
        double vX = 0.0;
        double vY = 0.0;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        ElapsedTime runtime = new ElapsedTime();
        waitForStart();
        runtime.reset();
        /*************************************************************
         *            Everything below here  \\ press START           *
         **************************************************************/

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            CurrentTime = System.currentTimeMillis();

            /****************************************************
             *                SENSORS
             *        INPUTS: Raw Sensor Values (Not encoders)
             *       OUTPUTS: parameters containing sensor values*
             ****************************************************/
            if (CurrentTime - LastSensor > SENSORPERIOD) {
                LastSensor = CurrentTime;
            }

            /* **************************************************
             *                Controller INPUT                  *
             *  INPUTS: raw controller values                   *
             *  OUTPUTS:
             *         g1_LeftX    g1_RightX
             *         g1_LeftY    g1_RightY
             *         g1_a (gamepad A)
             *         g1_b (gamepad B)
             ****************************************************/
            if (CurrentTime - LastController > CONTROLLERPERIOD) {
                LastController = CurrentTime;
            }

            /***************************************************
             *                NAV
             *      Inputs:  Gamepad positions
             *               Sensor Values (as needed)
             *      Outputs: Servo and Motor position commands
             *                         motor
             ****************************************************/
            if (CurrentTime - LastNav > NAVPERIOD) {
                LastNav = CurrentTime;

                if (gamepad2.right_stick_button) {
                    magnitudeScalar = fullSpeedScalar;
                }
                if (gamepad2.left_stick_button) {
                    magnitudeScalar = turtleScalar;
                }

                //Always reset motor commands
                motorLeftCmd = 0.0;
                motorRightCmd = 0.0;
                motorCenterCmd = 0.0;

                //Calculate angle and magnitude for Kiwi drive
                angleJoystick = gamepadDrive.angle(gamepad2.left_stick_x, gamepad2.left_stick_y);
                angleJoystick =   Math.toDegrees( Math.atan2(gamepad2.left_stick_x, gamepad2.left_stick_y) ) ;
                angleJoystick += 270;
                angleJoystick = (angleJoystick >= 360) ? angleJoystick-360 : angleJoystick;
                angleJoystick = ((int) (angleJoystick / 15)) * 15.0;   //Fixed increments, must be a factor of 90
                magnitudeJoystick = gamepadDrive.magnitude(gamepad2.left_stick_x, gamepad2.left_stick_y);
                magnitudeJoystick = (magnitudeJoystick > 1.0) ? 1.0 : magnitudeJoystick;
                //angleHeading = robot.imu.getAngularOrientation().firstAngle;
                //angleActual = mod360(angleJoystick+angleHeading);
                angleActual = angleJoystick;

                //Calculate motor commands based on angle and magnitude, use motor offset angles
                //from hardware definition.
                if (movementAllowed()) {
                    //For now just use the angle specified by the stick on the controller.
                    vX = Math.cos(Math.toRadians(angleActual))*magnitudeScalar*magnitudeJoystick;
                    vY = Math.sin(Math.toRadians(angleActual))*magnitudeScalar*magnitudeJoystick;
                    motorLeftCmd = Math.cos(Math.toRadians(robot.motorLeftAngle))*vY + Math.sin(Math.toRadians(robot.motorLeftAngle))*vX;
                    motorRightCmd = Math.cos(Math.toRadians(robot.motorRightAngle))*vY + Math.sin(Math.toRadians(robot.motorRightAngle))*vX;
                    motorCenterCmd = Math.cos(Math.toRadians(robot.motorCenterAngle))*vY + Math.sin(Math.toRadians(robot.motorCenterAngle))*vX;

                    //Build in some pivots, Go ahead and sum these on top of the other movements
                    if ((gamepad2.left_trigger > 0.1) || (gamepad2.left_bumper)) {
                        motorLeftCmd += 0.5*magnitudeScalar;
                        motorRightCmd += 0.5*magnitudeScalar;
                    }
                    if ((gamepad2.right_trigger > 0.1) || (gamepad2.right_bumper)) {
                        motorLeftCmd -= 0.5*magnitudeScalar;
                        motorRightCmd -= 0.5*magnitudeScalar;
                    }
                }
            }

            /* **************************************************
             *                SERVO OUTPUT
             *                Inputs: leftClamp position command
             *                        rightClamp position command *
             *                Outputs: Physical write to servo interface.
             ****************************************************/
            if (CurrentTime - LastServo > SERVOPERIOD) {
                LastServo = CurrentTime;
            }

            /* ***************************************************
             *                MOTOR OUTPUT
             *       Inputs:  Motor power commands
             *       Outputs: Physical interface to the motors
             ****************************************************/
            if (CurrentTime - LastMotor > MOTORPERIOD) {
                LastMotor = CurrentTime;
                // Yes, we'll set the power each time, even if it's zero.
                // this way we don't accidentally leave it somewhere.  Just simpler this way.
                robot.motorLeft.setPower(motorLeftCmd);
                robot.motorRight.setPower(motorRightCmd);
                robot.motorCenter.setPower(motorCenterCmd);

            }


            /* ***************************************************
             *                TELEMETRY
             *       Inputs:  telemetry structure
             *       Outputs: command telemetry output to phone
             ****************************************************/
            if (CurrentTime - LastTelemetry > TELEMETRYPERIOD) {
                LastTelemetry = CurrentTime;
                telemetry.clear();
                telemetry.addData("angle joystick ", angleJoystick);
                telemetry.addData("magnitude joystick", magnitudeJoystick);
                telemetry.addData("g2_leftx", gamepad2.left_stick_x);
                telemetry.addData("g2_lefty", gamepad2.left_stick_y);
                telemetry.addData("motorLeftCmd", motorLeftCmd);
                telemetry.addData("motorRightCmd", motorRightCmd);
                telemetry.addData("motorCenterCmd", motorCenterCmd);
                telemetry.update();
            }
        }

        //SAFE EXIT OF RUN OPMODE, stop motors, leave servos????
        robot.motorLeft.setPower(0);
        robot.motorRight.setPower(0);
        robot.motorCenter.setPower(0);
    }



//  Small routine that should be wrapped around all movement commands for the robot.
//  use this to check and make sure all vulnerable components of the robot are in a known state
//  before allowing any movement.
    private boolean movementAllowed() {
        boolean status = true;
        return (status);
    }


    private double mod360(double val) {
        int x = (int)val / 360;
        return (val - (val*360));
    }

}


