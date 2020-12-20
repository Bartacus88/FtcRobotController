package org.firstinspires.ftc.teamcode;

/* *
 *  Created by James Rumsey   3/31/19
 *
 *
 * *
 * OVERVIEW:   Driver mode code for a four sided omni wheel robot
 *
 * FORMAT:   It is important to keep in mind that this code operates in degrees and sets it frame of
 *           reference a bit different than the students might 'guess'
 *
 *           The forward motor is in the 90 degree position
 *           The right motor is in the 0 degree position
 *           The left motor is in the 180 degree position
 *           The rear motor is in the 270 degree position
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
@TeleOp(name = "Rosie the Riveter BOT", group = "HardwarePushbot")
@Disabled
public class Driver_Omni_X extends LinearOpMode {
    private HardwareDefenitionOmni_X robot = new HardwareDefenitionOmni_X();
    private Drive robotDrive         = new Drive();
    private GamepadDrive gamepadDrive = new GamepadDrive();



    //Defenitions for rate at which each driver task executes.   Time is in milliseconds
    final long SENSORPERIOD = 20;
    final long SERVOPERIOD = 20;
    final long NAVPERIOD = 20;
    final long MOTORPERIOD = 20;
    final long CONTROLLERPERIOD = 20;
    final long TELEMETRYPERIOD = 500;


    static double turtleScalar = 0.35;
    static double fullSpeedScalar = 1.0;
    static double trimSpeed = 0.6;
    double magnitudeScalar = turtleScalar;




    public void runOpMode() {

        HardwareDefenitionOmni_X.STATUS retVal;



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


        // variables for controller inputs.  TO DO turn this into a joystick class()
        double g1_LeftX = 0;
        double g1_LeftY = 0;
        double g1_RightX = 0;
        double g1_RightY = 0;
        boolean g1_A = false;
        boolean g1_B = false;
        boolean g1_X = false;
        boolean g1_Y = false;
        boolean g1_DD = false;
        boolean g1_DU = false;
        boolean g1_DL = false;
        boolean g1_DR = false;
        boolean g1_LB = false;
        boolean g1_RB = false;
        float g1_LT = 0;
        float g1_RT = 0;
        boolean   g1_LSB = false;
        boolean   g1_RSB = false;

        float g2_leftX = 0;
        float g2_LeftY = 0;
        float g2_RightX = 0;
        float g2_RightY = 0;
        boolean g2_A = false;
        boolean g2_B = false;
        boolean g2_X = false;
        boolean g2_Y = false;
        boolean g2_DD = false;
        boolean g2_DU = false;
        boolean g2_DL = false;
        boolean g2_DR = false;
        boolean g2_LB = false;
        boolean g2_RB = false;
        float g2_LT = 0;
        float g2_RT = 0;
        boolean   g2_LSB = false;
        boolean   g2_RSB = false;


        int g2_A_Counts = 0;
        int g2_DU_Counts = 0;
        int count=0;




        //TO DO,  make a Mecanum Driver mode class
        double g1_LT_Threshold =  0.0;
        double g1_X_Threshold =   0.0;
        double g1_Y_Threshold =   0.0;
        double g2_X_Threshold =   0.0;
        double g2_Y_Threshold =   0.0;
        double g1_Crab_Threshold =0.0;

        double motorFrontCmd = 0.0;
        double motorRearCmd = 0.0;
        double motorLeftCmd = 0.0;
        double motorRightCmd = 0.0;
        double driveMax = 1.0;
        double driveMin = -1.0;



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

                g1_LeftX = gamepad1.left_stick_x;
                g1_LeftY = gamepad1.left_stick_y;
                g1_RightX = gamepad1.right_stick_x;
                g1_RightY = gamepad1.right_stick_y;
                g1_A = gamepad1.a;
                g1_B = gamepad1.b;
                g1_X = gamepad1.x;
                g1_Y = gamepad1.y;
                g1_DD = gamepad1.dpad_down;
                g1_DU = gamepad1.dpad_up;
                g1_DL = gamepad1.dpad_left;
                g1_DR = gamepad1.dpad_right;
                g1_LB = gamepad1.left_bumper;
                g1_RB = gamepad1.right_bumper;
                g1_LT = gamepad1.left_trigger;
                g1_RT = gamepad1.right_trigger;
                g1_LSB = gamepad1.left_stick_button;
                g1_RSB = gamepad1.right_stick_button;

                g2_leftX = gamepad2.left_stick_x;
                g2_LeftY = gamepad2.left_stick_y;
                g2_RightX = gamepad2.right_stick_x;
                g2_RightY = gamepad2.right_stick_y;
                g2_A = gamepad2.a;
                g2_B = gamepad2.b;
                g2_X = gamepad2.x;
                g2_Y = gamepad2.y;
                g2_DD = gamepad2.dpad_down;
                g2_DU = gamepad2.dpad_up;
                g2_DL = gamepad2.dpad_left;
                g2_DR = gamepad2.dpad_right;
                g2_LB = gamepad2.left_bumper;
                g2_RB = gamepad2.right_bumper;
                g2_LT = gamepad2.left_trigger;
                g2_RT = gamepad2.right_trigger;
                g2_LSB = gamepad2.left_stick_button;
                g2_RSB = gamepad2.right_stick_button;


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

                if (g2_RSB) {
                    magnitudeScalar = fullSpeedScalar;
                }
                if (g2_LSB) {
                    magnitudeScalar = turtleScalar;
                }

                //Always reset motor commands
                motorFrontCmd = 0.0;
                motorRearCmd = 0.0;
                motorLeftCmd = 0.0;
                motorRightCmd = 0.0;

                //Calculate angle and magnitude for Kiwi drive
                angleJoystick = gamepadDrive.angle(g2_leftX, g2_LeftY);
                angleJoystick =   Math.toDegrees( Math.atan2(g2_leftX, g2_LeftY) ) ;
                angleJoystick += 270;
                angleJoystick = (angleJoystick >= 360) ? angleJoystick-360 : angleJoystick;
                angleJoystick = ((int) (angleJoystick / 15)) * 15.0;   //Fixed increments, must be a factor of 90
                magnitudeJoystick = gamepadDrive.magnitude(g2_leftX, g2_LeftY);
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
                    motorFrontCmd = Math.cos(Math.toRadians(robot.motorFrontAngle))*vY + Math.sin(Math.toRadians(robot.motorFrontAngle))*vX;
                    motorRearCmd = Math.cos(Math.toRadians(robot.motorRearAngle))*vY + Math.sin(Math.toRadians(robot.motorRearAngle))*vX;
                    motorLeftCmd = Math.cos(Math.toRadians(robot.motorLeftAngle))*vY + Math.sin(Math.toRadians(robot.motorLeftAngle))*vX;
                    motorRightCmd = Math.cos(Math.toRadians(robot.motorRightAngle))*vY + Math.sin(Math.toRadians(robot.motorRightAngle))*vX;

                    //Build in some pivots, Go ahead and sum these on top of the other movements
                    if ((g2_LT > 0.1) || (g2_LB)) {
                        motorFrontCmd -= 1.0*magnitudeScalar;
                        motorRearCmd -= 1.0*magnitudeScalar;
                        motorLeftCmd += 1.0*magnitudeScalar;
                        motorRightCmd += 1.0*magnitudeScalar;
                    }
                    if ((g2_RT > 0.1) || (g2_RB)) {
                        motorFrontCmd += 1.0*magnitudeScalar;
                        motorRearCmd += 1.0*magnitudeScalar;
                        motorLeftCmd -= 1.0*magnitudeScalar;
                        motorRightCmd -= 1.0*magnitudeScalar;
                    }

                    if (g2_RightX < -0.2) {
                        motorLeftCmd += trimSpeed*magnitudeScalar;
                        motorRightCmd += trimSpeed*magnitudeScalar;
                    }
                    if (g2_RightX > 0.2) {
                        motorRightCmd -= trimSpeed*magnitudeScalar;
                        motorLeftCmd -= trimSpeed*magnitudeScalar;
                    }
                    if (g2_RightY > 0.2) {
                        motorRearCmd += trimSpeed*magnitudeScalar;
                        motorFrontCmd += trimSpeed*magnitudeScalar;
                    }
                    if (g2_RightY < -0.2) {
                        motorFrontCmd -= trimSpeed*magnitudeScalar;
                        motorRearCmd -= trimSpeed*magnitudeScalar;
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
                robot.motorFront.setPower(motorFrontCmd);
                robot.motorRear.setPower(motorRearCmd);
                robot.motorLeft.setPower(motorLeftCmd);
                robot.motorRight.setPower(motorRightCmd);

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
                telemetry.addData("magnitude scalar", magnitudeScalar);
                telemetry.addData("g2_leftx", g2_leftX);
                telemetry.addData("g2_lefty", g2_LeftY);
                telemetry.addData("motorFrontCmd", motorFrontCmd);
                telemetry.addData("motorRearCmd", motorRearCmd);
                telemetry.addData("motorLeftCmd", motorLeftCmd);
                telemetry.addData("motorRightCmd", motorRightCmd);
                telemetry.update();
            }
        }

        //SAFE EXIT OF RUN OPMODE, stop motors, leave servos????
        robot.motorFront.setPower(0);
        robot.motorRear.setPower(0);
        robot.motorLeft.setPower(0);
        robot.motorRight.setPower(0);
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


