package org.firstinspires.ftc.teamcode;

/* *
 *  Created by James Rumsey   2/25/19
 *
 *  Adapted from several seasons of Juden Ki 8578 and Kernel Panic 11959 code.
 *
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.LinearActuator;
import com.qualcomm.robotcore.util.ElapsedTime;


@SuppressWarnings("WeakerAccess")
@TeleOp(name = "JK Drive Opmode Sideways JS Swapped", group = "HardwarePushbot")
//@Disabled
public class Driver_90deg_Rotation_JS_Swapped_20_21 extends LinearOpMode {
    private HardwareDef_20_21 robot = new HardwareDef_20_21();
    private Drive robotDrive = new Drive();
    private GamepadDrive gamepadDrive = new GamepadDrive();
    LinearActuator lineAct = new LinearActuator();

    //Defenitions for rate at which each driver task executes.   Time is in milliseconds
    final long SENSORPERIOD = 20;
    final long SERVOPERIOD = 20;
    final long NAVPERIOD = 20;
    final long MOTORPERIOD = 20;
    final long CONTROLLERPERIOD = 20;
    final long TELEMETRYPERIOD = 500;

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

        telemetry.addData("Status", "Configuring StoneManipulator");
        //stoneManipulator.init(this,robot.lift, robot.deploy, robot.position,
        //                      robot.orientation, robot.clamp, robot.color1, robot.color2,
        //                      robot.distanceLeft, robot.distanceRight, robot.distanceStone);

        //lift.initialize(robot.lift, LinearActuator.ACTUATOR_TYPE.PULLEY_LINEAR_SLIDE, 28, 50.9, 30, 1.25,this, true);
        //lift.move(0.0, LinearActuator.MOVETYPE.AUTOMATIC);
        //deploy.initialize(robot.deploy, LinearActuator.ACTUATOR_TYPE.LEAD_SCREW, 28, 71.2, 8, 8.6,6, this, true);
        //deploy.move (0.0, LinearActuator.MOVETYPE.AUTOMATIC);
        //position.initialize(robot.deploy, LinearActuator.ACTUATOR_TYPE.LEAD_SCREW, 28, 13.7, 8, 8.6,6, this, true);
        //position.move (0.0, LinearActuator.MOVETYPE.AUTOMATIC);
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

        boolean loaderIsAsserted = false;

        int leftBumperCnt = 0;
        int rightBumperCnt = 0;
        final int BUMPTHRESHOLD = 5;

        double angle = 0.0;
        double magnitude = 0.0;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        ElapsedTime runtime = new ElapsedTime();

        robot.wobbleGoalMotor.setTargetPosition(0); //Must state that our initial position is refered to as "0" or the "datum"
        lineAct.initialize(robot.wobbleGoalMotor, LinearActuator.ACTUATOR_TYPE.MOTOR_ONLY, 1,
                1440, 1, this, true);


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
             ****************************************************/
            if (CurrentTime - LastController > CONTROLLERPERIOD) {
                LastController = CurrentTime;

                if (gamepad1.a || gamepad2.a) {
                    //Set to low bridge transition
                    //lift.move(0.0, LinearActuator.MOVETYPE.AUTOMATIC);
                    shooterPower = intakePower;
                }
                else
                {
                    shooterPower = 0;
                }

                if (gamepad1.right_trigger > 0.05 || gamepad2.right_trigger > 0.05) {
                    intakePower = Math.max(gamepad1.right_trigger, gamepad2.left_trigger);
                } else {
                    intakePower = 0;
                }

                if (gamepad1.b) {
                    //Move position is in percentage.  Therefore dont give it 40 for 40 deg.  Give it 40/360 = 1/9 = 0.11111111
                    lineAct.move(0.11111111, LinearActuator.MOVETYPE.AUTOMATIC);
                }
                if (gamepad1.x) {
                    //Move position is in percentage.  Therefore dont give it 160 for 160 deg.  Give it 160/360 = 4/9 = 0.44444444
                    lineAct.move(0.44444444, LinearActuator.MOVETYPE.AUTOMATIC);
                }
                if (gamepad1.y) {
                    //Move position is in percentage.  Therefore dont give it 150 for 150 deg.  Give it 150/360 = 5/12 = 0.41666666
                    lineAct.move(0.41666666, LinearActuator.MOVETYPE.AUTOMATIC);
                }
                if (gamepad1.right_bumper) {
                    //Move position is in percentage.  Therefore dont give it 140 for 140 deg.  Give it 140/360 = 7/18 = 0.38888888
                    lineAct.move(0.38888888, LinearActuator.MOVETYPE.AUTOMATIC);
                }
                if (gamepad1.left_bumper) {
                    //Move position is in percentage.  Therefore dont give it 130 for 130 deg.  Give it 130/360 = 13/36 = 0.36111111
                    lineAct.move(0.36111111, LinearActuator.MOVETYPE.AUTOMATIC);
                }
                //This is repetitive Its here merely to show that Both Controllers could launch.
                /*if (gamepad2.a || gamepad1.a) {
                    //Set to low bridge transition
                    //lift.move(0.0, LinearActuator.MOVETYPE.AUTOMATIC);
                    shooterPower = intakePower;
                }
                else
                {
                    shooterPower = 0;
                }
                //This is repetitive Its here merely to show that Both Controllers could launch.  This will probably need to be removed.
                if (gamepad2.right_trigger > 0.05 || gamepad1.right_trigger > 0.05) {
                    intakePower = Math.max(gamepad2.left_trigger, gamepad1.right_trigger);
                } else {
                    intakePower = 0;
                }*/
                if (gamepad2.b) {
                    //Move position is in percentage.  Therefore dont give it 40 for 40 deg.  Give it 40/360 = 1/9 = 0.111111111111
                    lineAct.move(0.111111111111, LinearActuator.MOVETYPE.AUTOMATIC);
                }
                if (gamepad2.x) {
                    //Initiate placement of stone
                    //position.move(0.0, LinearActuator.MOVETYPE.AUTOMATIC);
                }
                if (gamepad2.y) {
                    //Initiate placement of capstone
                    //position.move(1.0, LinearActuator.MOVETYPE.AUTOMATIC);
                }
/*
                if (gamepad1.right_bumper) {
//                    rightBumperCnt++;
//                    if(rightBumperCnt > BUMPTHRESHOLD) {
//                        intakePower += 0.3334; //Icrement by 33.34%
//                        Math.min(1, intakePower);
//                        rightBumperCnt = 0;
//                    }
                    intakePower = 1;
                }
                if (gamepad1.left_bumper) {
//                   leftBumperCnt++;
//                    if(leftBumperCnt > BUMPTHRESHOLD) {
//                        intakePower -= 0.3334; //Decrement by 33.34%
//                        Math.max(0, intakePower);
//                        rightBumperCnt = 0;
//                    }
                    intakePower = 0;
                }
*/



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
                angle = gamepadDrive.angle(gamepad1.right_stick_y, gamepad1.right_stick_x);
                magnitude = gamepadDrive.magnitude(gamepad1.right_stick_y, gamepad1.right_stick_x);


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
                if (Math.abs(gamepad1.right_stick_x) > g1_Y_Threshold) {
                    //Move forwards or backwards
                    leftDriveCmd = gamepadDrive.condition(gamepad1.right_stick_x, GamepadDrive.CONDITION_TYPE.POWER7);
                    rightDriveCmd = leftDriveCmd;
                    leftRearCmd = leftDriveCmd;
                    rightRearCmd = leftDriveCmd;
                }

                //PIVOT
                if (Math.abs(gamepad1.right_stick_y) > g1_X_Threshold) {
                    //Pivot
                    leftDriveCmd = -1 * gamepadDrive.condition(gamepad1.right_stick_y, GamepadDrive.CONDITION_TYPE.POWER7);
                    rightDriveCmd = -1 * leftDriveCmd;
                    leftRearCmd = leftDriveCmd;
                    rightRearCmd = rightDriveCmd;
                }


                // CRAB STYLE DRIVE
                //FORWARD/REVERSE
                if (Math.abs(gamepad1.left_stick_x) > g1_Crab_Threshold) {  //Forward, backward
                    leftDriveCrab = gamepadDrive.condition(gamepad1.left_stick_x, GamepadDrive.CONDITION_TYPE.TURTLE_HIGH_MANEUVERABILITY);
                    rightDriveCrab = leftDriveCrab;
                    leftRearCrab = leftDriveCrab;
                    rightRearCrab = leftDriveCrab;
                }

                //CRAB
                if (Math.abs(gamepad1.left_stick_y) > g1_Crab_Threshold) {
                    leftDriveCrab = -1 * gamepadDrive.condition(gamepad1.left_stick_y, GamepadDrive.CONDITION_TYPE.TURTLE_HIGH_MANEUVERABILITY);
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

                robot.wobbleGoalMotor.setPower(0.2);


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
        robot.wobbleGoalMotor.setPower(0);



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


