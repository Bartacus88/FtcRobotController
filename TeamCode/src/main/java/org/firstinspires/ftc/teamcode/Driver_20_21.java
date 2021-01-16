package org.firstinspires.ftc.teamcode;

/* *
 *  Created by James Rumsey   2/25/19
 *
 *  Adapted from several seasons of Juden Ki 8578 and Kernel Panic 11959 code.
 *
 */

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@SuppressWarnings("WeakerAccess")
@TeleOp(name = "JK Drive Opmode (Color Drive Test)", group = "HardwarePushbot")
//@Disabled
public class Driver_20_21 extends LinearOpMode {
    private HardwareDef_20_21 robot = new HardwareDef_20_21();
    private Drive robotDrive = new Drive();
    private GamepadDrive gamepadDrive = new GamepadDrive();

    //Defenitions for rate at which each driver task executes.   Time is in milliseconds
    final long SENSORPERIOD = 20;
    final long SERVOPERIOD = 20;
    final long NAVPERIOD = 20;
    final long MOTORPERIOD = 20;
    final long CONTROLLERPERIOD = 20;
    final long TELEMETRYPERIOD = 500;

    public enum Color {
        NOCOLOR, //0
        ERROR,   //1
        BLUE,    //2
        RED,     //3
        YELLOW   //4
    }

    float hsvValues[] = {0F, 0F, 0F};


    public void runOpMode() {

        HardwareDef_20_21.STATUS retVal;



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
        telemetry.addData("      ", retVal);
        telemetry.update();

        telemetry.addData("Status", "Configuring Robot");

        long CurrentTime = System.currentTimeMillis();
        long LastSensor = CurrentTime;
        long LastEncoderRead = CurrentTime + 5;
        long LastServo = CurrentTime + 10;
        long LastNav = CurrentTime + 15;
        long LastMotor = CurrentTime + 20;
        long LastController = CurrentTime + 7;
        long LastTelemetry = CurrentTime + 17;


        // variables for controller inputs.  TO DO turn this into a joystick class()
        //TODO,  make a Mecanum Driver mode class
        double g1_LT_Threshold = 0.0;
        double g1_X_Threshold = 0.1;
        double g1_Y_Threshold = 0.0;
        double g2_X_Threshold = 0.1;
        double g2_Y_Threshold = 0.0;
        double g1_Crab_Threshold = 0.0;

        double leftDriveCmd = 0.0;
        double rightDriveCmd = 0.0;
        double leftRearCmd = 0.0;
        double rightRearCmd = 0.0;
        double leftDriveCrab = 0.0;
        double rightDriveCrab = 0.0;
        double leftRearCrab = 0.0;
        double rightRearCrab = 0.0;
        double driveMax = 1.0;
        double driveMin = -1.0;

        double shooterPower = 0;

        double intakePower = 0;

        double angle = 0.0;
        double magnitude = 0.0;

        Color currentColor = Color.ERROR;

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
                //https://github.com/judenkirobotics/season2016-17/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/ColorSensorTest.java

                android.graphics.Color.RGBToHSV(robot.color1.red() * 8, robot.color1.green() * 8, robot.color1.blue() * 8, hsvValues);
                currentColor = DetectColor((int) hsvValues[0]);

                if (currentColor == Color.YELLOW) {
                    robot.frontRight.setPower(0);
                    robot.frontLeft.setPower(0);
                    robot.backRight.setPower(0);
                    robot.backLeft.setPower(0);
                    sleep(3000); //freeze the motors for 30 seconds when we hit the line.
                }

            }



            /* **************************************************
             *                Controller INPUT                  *
             *  INPUTS: raw controller values                   *
             *  OUTPUTS:
             ****************************************************/
            if (CurrentTime - LastController > CONTROLLERPERIOD) {
                LastController = CurrentTime;

                if (gamepad1.a) {
                    //Set to low bridge transition
                    //lift.move(0.0, LinearActuator.MOVETYPE.AUTOMATIC);
                    shooterPower = .7;
                } else {
                    shooterPower = 0;
                }
                if (gamepad1.b) {
                    currentColor = Color.NOCOLOR;
                }
                if (gamepad1.x) {
                    //Initiate placement of stone
                    //lift.move(0.5, LinearActuator.MOVETYPE.AUTOMATIC);
                }
                if (gamepad1.y) {
                    //Initiate placement of capstone
                    //lift.move(1.0, LinearActuator.MOVETYPE.AUTOMATIC);
                }

                if (gamepad2.a) {
                    //Set to low bride transition
                    //deploy.move(0.0, LinearActuator.MOVETYPE.AUTOMATIC);
                }
                if (gamepad2.b) {
                    //Set to high bridge transition
                    //deploy.move(1.0, LinearActuator.MOVETYPE.AUTOMATIC);
                }
                if (gamepad2.x) {
                    //Initiate placement of stone
                    //position.move(0.0, LinearActuator.MOVETYPE.AUTOMATIC);
                }
                if (gamepad2.y) {
                    //Initiate placement of capstone
                    //position.move(1.0, LinearActuator.MOVETYPE.AUTOMATIC);
                }


                if (gamepad1.right_bumper) {
                    /*rightBumperCnt++;
                    if(rightBumperCnt > BUMPTHRESHOLD) {
                        intakePower += 0.3334; //Icrement by 33.34%
                        Math.min(1, intakePower);
                        rightBumperCnt = 0;
                    }*/
                    intakePower = 1;
                }
                if (gamepad1.left_bumper) {
                   /* leftBumperCnt++;
                    if(leftBumperCnt > BUMPTHRESHOLD) {
                        intakePower -= 0.3334; //Decrement by 33.34%
                        Math.max(0, intakePower);
                        rightBumperCnt = 0;
                    }*/
                    intakePower = 0;
                }

/*                else if ((gamepad1.dpad_up) || (turtleInitiated)) {
                    //Engage turtle mode, approach build platform

                }
                else {

                }*/

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

                // init drive min and max to default values.  We'll reset them to other numbers
                // if conditions demand it.
                leftDriveCmd = 0;
                rightDriveCmd = 0;
                leftRearCmd = 0;
                rightRearCmd = 0;
                leftDriveCrab = 0;
                rightDriveCrab = 0;
                leftRearCrab = 0;
                rightRearCrab = 0;

                //Test code for gamepad angle and magnitude
                angle = gamepadDrive.angle(gamepad1.left_stick_x, gamepad1.left_stick_y);
                magnitude = gamepadDrive.magnitude(gamepad1.left_stick_x, gamepad1.left_stick_y);


                /**********************************************************************************
                 * PRO TIP:  It is not required (or advised) to use the same conditioning methods
                 *           for each movement type.   There are two forward and reverse, make one
                 *           full range and one turtle.   Select something precision for crab, maybe
                 *           something more mid range for pivot.   Regardless select the conditioning
                 *           filters to achieve the desired behaviour for each movement type.  One
                 *           size does not fit all.
                 *********************************************************************************/

                //if (movementAllowed()) {
                //TODO change drive style to single stick using the vector method developed
                // for the OMni robots.   Use the analog triggers to control the pivot.
                // TANK STYLE DRIVE
                //FORWARD/REVERSE
                if (Math.abs(gamepad1.left_stick_y) > g1_Y_Threshold) {
                    //Move forwards or backwards
                    leftDriveCmd = gamepadDrive.condition(gamepad1.left_stick_y, GamepadDrive.CONDITION_TYPE.POWER7);
                    rightDriveCmd = leftDriveCmd;
                    leftRearCmd = leftDriveCmd;
                    rightRearCmd = leftDriveCmd;
                }

                //PIVOT
                if (Math.abs(gamepad1.left_stick_x) > g1_X_Threshold) {
                    //Pivot
                    leftDriveCmd = -1 * gamepadDrive.condition(gamepad1.left_stick_x, GamepadDrive.CONDITION_TYPE.POWER7);
                    rightDriveCmd = -1 * leftDriveCmd;
                    leftRearCmd = leftDriveCmd;
                    rightRearCmd = rightDriveCmd;
                }


                // CRAB STYLE DRIVE
                //FORWARD/REVERSE
                if (Math.abs(gamepad1.right_stick_y) > g1_Crab_Threshold) {  //Forward, backward
                    leftDriveCrab = gamepadDrive.condition(gamepad1.right_stick_y, GamepadDrive.CONDITION_TYPE.TURTLE_HIGH_MANEUVERABILITY);
                    rightDriveCrab = leftDriveCrab;
                    leftRearCrab = leftDriveCrab;
                    rightRearCrab = leftDriveCrab;
                }

                //CRAB
                if (Math.abs(gamepad1.right_stick_x) > g1_Crab_Threshold) {
                    leftDriveCrab = -1 * gamepadDrive.condition(gamepad1.right_stick_x, GamepadDrive.CONDITION_TYPE.TURTLE_HIGH_MANEUVERABILITY);
                    rightDriveCrab = -1 * leftDriveCrab;
                    leftRearCrab = rightDriveCrab;
                    rightRearCrab = leftDriveCrab;
                }
            }

            //}



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
                robot.frontRight.setPower(-1 * (rightDriveCmd + rightDriveCrab));
                robot.frontLeft.setPower(-1 * (leftDriveCmd + leftDriveCrab));
                robot.backRight.setPower(-1 * (rightRearCmd + rightRearCrab));
                robot.backLeft.setPower(-1 * (leftRearCmd + leftRearCrab));

                robot.frontShooter.setPower(shooterPower);
                robot.backShooter.setPower(shooterPower);

                robot.transportIntake.setPower(intakePower);


            }


            /* ***************************************************
             *                TELEMETRY
             *       Inputs:  telemetry structure
             *       Outputs: command telemetry output to phone
             ****************************************************/

            if (CurrentTime - LastTelemetry > TELEMETRYPERIOD) {
                LastTelemetry = CurrentTime;
                telemetry.clear();
                //telemetry.addData("Lift Current Position ", lift.getCurrentPosition());
                //telemetry.addData("Lift Target Position ", lift.getTargetPosition());
                //telemetry.addData("Deploy Current Position ", deploy.getCurrentPosition());
                //telemetry.addData("Deploy Target Position ", deploy.getTargetPosition());
                //telemetry.addData("Position Current Position ", position.getCurrentPosition());
                //telemetry.addData("Position Target Position ", position.getTargetPosition());
                telemetry.addData("Position Target Position ", hsvValues[0]);
                //stoneManipulator.displayTarget();
                //stoneManipulator.displayPlacement();
                //stoneManipulator.displayActuators();
                //stoneManipulator.displaySensors();
                telemetry.update();
            }
        }

        //SAFE EXIT OF RUN OPMODE, stop motors, leave servos????
        robot.frontRight.setPower(0);
        robot.frontLeft.setPower(0);
        robot.backRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.transportIntake.setPower(0);
        robot.frontShooter.setPower(0);
        robot.backShooter.setPower(0);


    }

    public static int redCnt = 0;
    public static int blueCnt = 0;
    public static int yellowCnt = 0;

    public static Color DetectColor(int hueIn) {
        final int blueMin = 197; //Blue Starts at 197deg and goes to 217deg.
        final int blueMax = 217;
        final int redMin = 351; //Red Starts at 351deg and wraps around to 11deg.
        final int redMax = 11;
        final int yellowMin = 102; //Yellow Starts at 102deg and goes to 122deg. Although we want white, we named this yellow.
        final int yellowMax = 122;


        Color detectedColor = Color.ERROR;

        redCnt--;
        yellowCnt--;
        blueCnt--;

        //ensure blueCnt,redCnt,yellowCnt are always greater than 0


        if (hueIn >= blueMin && hueIn <= blueMax) {
            blueCnt += 2; //Increment twice because we already decremented
        }

        if (hueIn >= redMin || hueIn <= redMax) //Red is a special value when you consider it with the hue values since it wraps around. So we used "OR" to deterimine instead of &&
        {
            redCnt += 2; //Increment twice because we already decremented
        }

        if (hueIn >= yellowMin && hueIn <= yellowMax) {
            yellowCnt += 2; //Increment twice because we already decremented
        }

        //ensure blueCnt,redCnt,yellowCnt are always less than 5
        //                IN 65535  OUT IS 5
        //                    IN 6  OUT IS 5
        //                    IN 4  OUT IS 4
        //                    IN 3  OUT IS 3
        //                    IN 2  OUT IS 2
        //                    IN 1  OUT IS 1
        //                    IN 0  OUT IS 0
        blueCnt = Math.min(blueCnt, 5);
        redCnt = Math.min(redCnt, 5);
        yellowCnt = Math.min(yellowCnt, 5);
        blueCnt = Math.max(blueCnt, 0);
        redCnt = Math.max(redCnt, 0);
        yellowCnt = Math.max(yellowCnt, 0);

        //blueCnt = 0 redCnt = 0 yellow = 4
        if (blueCnt >= 3 && redCnt <= 2 && yellowCnt <= 2) {
            //System.out.print("Blue is your color");
            detectedColor = Color.BLUE;
        } else if (redCnt >= 3 && yellowCnt <= 2 && blueCnt <= 2) {
            //System.out.print("Red is your color");
            detectedColor = Color.RED;
        } else if (yellowCnt >= 3 && blueCnt <= 2 && redCnt <= 2) {
            //System.out.print("Yellow is your color");
            detectedColor = Color.YELLOW;
        } else {
            //System.out.print("No color detected");
            detectedColor = Color.NOCOLOR;
        }

        return (detectedColor);

    }
}


///*
// * Used to lock out movements for the robot.
// * movementAllowed() -- will prohibit movement if turtle movement has been engaged and/or the
// *           StoneManipulator is in a placement state
// * turtleAllowed() -- will prohibit turtle movement if the StoneManipulator is in a placement state
// */
//    private boolean movementAllowed() {
//        return (allowMovement);
//    }
//
//    private boolean turtleAllowed () {
//        return (allowTurtle);
//    }
//
//
//    /*
//     * Developing turtle approach in driver mode.  Eventually it should be made part of the
//     * Drive() class or perhaps a class of its own.   Want to be able to call in autonomous as well
//     * so it can not live here long term.  Having it as part of the Drive() class may make the most
//     * sense.
//     */
//
//
//    private boolean turtleApproachStarted = false;
//    private double initialLeftDistance = 0.0;
//    private double initialRightDistance = 0.0;
//    private double currentLeftDistance = 0.0;
//    private double currentRightDistance = 0.0;
//    private static double TURTLE_DESIRED_DISTANCE = 0.5;
//    private static double TURTLE_DISTANCE_TOLERANCE =0.2;
//    private static double TURTLE_INITIAL_SPEED = 0.3;
//    private static double TURTLE_MINIMUM_SPEED = 0.2;
//    private static double TURTLE_MAXIMUM_SPEED = 0.4;
//    private static double TURTLE_DELTA = 0.02;
//    private double turtleLeftPower=0.0;
//    private double turtleRightPower=0.0;
//    private void turtleApproach() {
//        if (!turtleApproachStarted) {
//            turtleApproachStarted = true;
//            initialLeftDistance = robot.distanceLeft.getDistance(DistanceUnit.INCH);
//            initialRightDistance = robot.distanceRight.getDistance(DistanceUnit.INCH);
//            if (initialLeftDistance < TURTLE_DESIRED_DISTANCE) {
//                turtleLeftPower = -1*TURTLE_MINIMUM_SPEED;
//            }
//            else {
//                turtleLeftPower = TURTLE_INITIAL_SPEED;
//            }
//            if (initialRightDistance < TURTLE_DESIRED_DISTANCE) {
//                turtleRightPower = -1*TURTLE_MINIMUM_SPEED;
//            }
//            else {
//                turtleRightPower = TURTLE_INITIAL_SPEED;
//            }
//        }
//        else {
//            currentLeftDistance = robot.distanceLeft.getDistance(DistanceUnit.INCH);
//            currentRightDistance = robot.distanceRight.getDistance(DistanceUnit.INCH);
//            if (turtleDistanceTolerance(currentLeftDistance)) {
//                turtleLeftPower = 0.0;
//            }
//            else {
//
//            }
//            if (turtleDistanceTolerance(currentRightDistance)) {
//                turtleRightPower = 0.0;
//            }
//            else {
//
//            }
//
//        }
//    }
//
//    private boolean turtleDistanceTolerance(double d) {
//        if ( (Math.abs(d)-Math.abs(TURTLE_DESIRED_DISTANCE)) < TURTLE_DISTANCE_TOLERANCE) {
//            return true;
//        }
//        return true;
//    }
//
//}


