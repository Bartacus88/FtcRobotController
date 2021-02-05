package org.firstinspires.ftc.teamcode;


/*
 * Created for Juden Ki 8578 Template Autonomous Code for 2019
 *
 * October 2016
 * Initial Concept by Howard Bishop (mentor)
 * First Coding by Jeffrey and Alexis
 *
 * October 2017
 * Second Coding by Katherine amd Jeffrey
 *
 * December 2018
 * Major overhaul into single state engine fed an array of commands by Jim Rumsey (mentor)
 *
 * August 2019
 *  Turned into template file for both teams.
 *
 *  OVERVIEW
 *  Generic autonomous file that should be unique to each robot.   It is fine to copy and rename
 *  this class.   Just have the autonomous runmodes target the copy.   The MOVE, PAUSE, and WAIT
 *  states should be generic and common to all robots.   All other states are robot specific and
 *  should map directly to a task that the robot is supposed to accomplish.   Push a button, find
 *  something of a particular color, etc.....
 *
 *  REQUIRED
 *  REV Robotics Expansion Hub --  Due to IMU the only supported hub
 *  IMU                        --  Rev IMU configured as Adafruit IMU on I2C 0
 *  HardwareDefenitionJK2019   --  Robot class that MUST define and initialize all of the actuators
 *                                  and sensors.
 *
 *  FIRST TIME ON NEW ROBOT
 *      1.  Configure all of the Drive parameters for the robot  (dParm).  For a fuller explanation
 *          of these parameters refer to the Drive.java Class.   NOTE:  The Drive class is unitless
 *          for distance measurements.  The wheel diameter just needs to be configured int the
 *          desired units.    (Ex:  2.5 if inches, or 6.35 if cm)
 *      2.  Create new states that correspond to tasks the robot is to perform.
 *
 *  CREATE NEW STATE
 *
 *      1.  Add an entry to the enum AutoStates.  This should be a name that describes the purpose
 *          of this new state.   (Ex: FLIP -- flips out a robot arm.)
 *      2.  Go the the NAVIGATION section of this class.   Add the new state to the case
 *          statement.   Add the code that will perform the purpose of this state.  Take care to
 *          avoid while loops when creating this code.  Nothing should be done that will prevent the
 *          robot from responding to the STOP button on the driver station
 *      3.  Test the new state.  This can be done by making the new state the first entry on
 *          AutoCommand array that is passed to this class.  (See JK_AutonomousExample.java)  Take
 *          care to verify that all value(1-4) inputs produce the desired behaviour.
 *      4.  MOST IMPORTANT -- Once the new state has been created update the state table in the
 *          comments below.  Remember if it is not documented it does not exist!
 *
 *
 ***************************************************************************************************
 * STATE         Description and Values 1-3
 ***************************************************************************************************
 * MOVE    The movement type is defined by moveType, depending on the movement type value1 is either
 *          a distance to travel or a heading to pivot towards.   value2 is the initial speed at
 *          which the movement is to commence.
 *
 *
 * PAUSE   Essentially a sleep state for robot, but not final wait state
 *
 * WAIT    Final state should be last called.   (Not strictly necessary)
 *
 */


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.List;

//openCV needed


@Autonomous(name = "JK Auto Vuforia BART", group = "Pushbot")
@SuppressWarnings("WeakerAccess")
public class Vuforia_JK_Auto_WorkOnPhone_20210130 extends LinearOpMode {

    //Define Robot Hardware and classes here
    private HardwareDef_Bart_20_21 robot = new HardwareDef_Bart_20_21();
    private Drive robotDrive = new Drive();
    LinearActuator lineAct = new LinearActuator();

    //Vuforia needed
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    public enum Rings2 {NONE, ONE, FOUR;}

    public Rings2 retRings = Rings2.NONE;
    public static final String VUFORIA_KEY = "Afsw2oP/////AAABmfzepGuVnkr3uHCSQlCl89hgf8A+n9yEsMhH8pfA7Ttz/JfeOCGHzGmHjZMt0IHzaR5tUbVK4L59qd0RJsjAfrmrCsu/CDMm90dy3T9+eRo+zlw6aGRHD0j3EhngUWY0dc1PrRZpWXa+KCLOy3rtB+aWaZDBxILq6uCiqRtLGUwBTrDGQVH/fE1z32YZbDISS5F6actiiu9RhSyU8DKn1EEWeuTj0W+O7lAoDUIhJfJ0B0Iqk73Gfuv2ytbUq89obq9ZVMUqjq9rFsYiztVPOXQkWZsnv8P+eN/0XEgDUmQCU4XBABUw7bTsn9WW/xMDyqnuUMy+AbD/ag5+EPpQaAEGa9nT6n/ATK/Znu47ZlAe";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    //Define all of the available states for AutoState.   Add new states before PAUSE
    public enum AutoStates {MOVE, PAUSE, SHOOT_RING, MOVE_COLOR, MOVE_WOBBLE_ARM, OPEN_CV, WAIT;}


    //Define AutoState run intervals here
    private final long SENSORPERIOD = 20;
    private final long SERVOPERIOD = 20;
    private final long NAVPERIOD = 20;
    private final long MOTORPERIOD = 20;
    private final long TELEMETRYPERIOD = 500;
    private float stageTime = 0;
    int CurrentAutoState = 0;
    static final int THIRTY_SECONDS = 30 * 1000;

    public enum Color {
        NOCOLOR, //0
        ERROR,   //1
        BLUE,    //2
        RED,     //3
        YELLOW   //4
    }

    float hsvValues[] = {0F, 0F, 0F};
    Color currentColor = Color.ERROR;

    int numRings = 5;

    public void runOpMode() {

        HardwareDef_Bart_20_21.STATUS retVal;
        /*
         * Initialize all of the robot hardware.
         * The init() method of the hardware class does all the work here
         */
        telemetry.addData("Status", "Initializing Robot...");
        telemetry.addData("Status", "Configuring Hardware...");
        telemetry.setAutoClear(false);
        telemetry.update();
        retVal = robot.init(hardwareMap, true, false, false);
        telemetry.addData("      ", retVal);
        telemetry.update();


        /*****************************************************
         *  CONFIGURE THE DRIVE TRAIN  THIS IS ROBOT SPECIFIC
         *****************************************************/
        Drive.Parameters dParm = robotDrive.getParameters();
        //Why didn't the example have these two lines below?  I, spent hours with this not working, until I added these.
        dParm.cenPolarity = DcMotorSimple.Direction.FORWARD;
        dParm.center = robot.frontRight; //Just to remove error, serves no purpose as far as I can tell

        dParm.frontRight = robot.frontRight;
        dParm.frontLeft = robot.frontLeft;
        dParm.rearRight = robot.backRight;
        dParm.rearLeft = robot.backLeft;
        dParm.frPolarity = DcMotorSimple.Direction.FORWARD;
        dParm.flPolarity = DcMotorSimple.Direction.REVERSE;
        dParm.rrPolarity = DcMotorSimple.Direction.FORWARD;
        dParm.rlPolarity = DcMotorSimple.Direction.REVERSE;
        dParm.driveType = Drive.DriveType.MECANUM;
        dParm.imu = robot.imu;
        dParm.motorRatio = 28;
        dParm.gearRatio = 20;
        dParm.wheelDiameter = 3.0;
        dParm.mecanumAngle = 45;
        dParm.pivotTolerance = Drive.PivotTolerance.ONE_DEGREE;
        dParm.encoderTolerance = (int) dParm.motorRatio * 5;    //!!VERY DANGEROUS TO PLAY WITH | CAN RESULT IN STUCK STATE!!
        dParm.turnBackoff = 0.45;  // 45 percent backoff
        dParm.backoffMultiplier = 25;    // Make larger for high speed turns.
        dParm.minStartPower = 0.1;
        dParm.minTurnPower = 0.1;
        dParm.opMode = this;
        dParm.debug = true;
        dParm.useEncoderRatio = false;
        if (robotDrive.configureDrive(dParm)) {
            telemetry.addData("Status   ", "Robot Initialized!");
        } else {
            telemetry.addData("Status   ", "INITIALIZATION FAILED!!!!!");
        }

        AutoCommand cmd[] = new AutoCommand[15];

        /*
         * Perform a little range checking on the supplied array of states.  Look to see if
         * the autonomous time limit will be exceeded

        int i = 0, t = 0;
        while (i < cmd.length) {
            t += cmd[i].timeLimit;
            i++;
        }
        if (t > THIRTY_SECONDS) {
            opMode.telemetry.addLine("WARNING... Autonomous Commands may exceed time");
            opMode.telemetry.addData("   Allowed", THIRTY_SECONDS);
            opMode.telemetry.addData("   Actual ", t);
        }
        opMode.telemetry.update();
        opMode.telemetry.setAutoClear(true);
*/

        /*
         * Define and initialize all of the loop timing variables
         */
        long CurrentTime = System.currentTimeMillis();
        long LastSensor = CurrentTime;
        long LastServo = CurrentTime + 10;
        long LastNav = CurrentTime + 15;
        long LastMotor = CurrentTime + 20;
        long LastTelemetry = CurrentTime + 17;
        ElapsedTime runtime = new ElapsedTime();

        double shooterPower = 0.0;
        double intakePower = 0.0;
        double ringDeflectorPosition = 0.0;
        //robot.backShooter.setPower(0);
        //robot.frontShooter.setPower(0);
        //robot.transportIntake.setPower(0);
        //robot.wobbleGoalMotor.setTargetPosition(0); //Must state that our initial position is refered to as "0" or the "datum"
        //lineAct.initialize(robot.wobbleGoalMotor, LinearActuator.ACTUATOR_TYPE.MOTOR_ONLY, 1, 1440, 1, opMode, true);

        //long shootRingTime = 0;
        final long FIRE_RING_TIME = 10 * NAVPERIOD;
        final long SHOOTER_SPOOL_TIME = 5 * NAVPERIOD;
        //final long SHOOT_RING_MAX_TIME = 25 * NAVPERIOD;

        double wobbleTarget = 0.0;

        //Vuforia Needed
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
        }

        waitForStart();
        runtime.reset();

        /* ************************************************************
         *            Everything below here runs after START          *
         **************************************************************/
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            CurrentTime = System.currentTimeMillis();

            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    //myOpMode.telemetry.clear();
                    //myOpMode.telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        //myOpMode.telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        //myOpMode.telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f", recognition.getLeft(), recognition.getTop());
                        //myOpMode.telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",recognition.getRight(), recognition.getBottom());
                        if (i == 0 && recognition.getLabel() == "Single") {
                            retRings = Rings2.ONE;
                        }
                        if (i == 0 && recognition.getLabel() == "Quad") {
                            retRings = Rings2.FOUR;

                        }
                    }
                    //myOpMode.telemetry.addData("RetRings: ", retRings);
                    //myOpMode.telemetry.update();
                }
            }


            if (tfod != null) {
                tfod.shutdown();
            }

            if (numRings >= 5) { //Default is 5.  The below code will change to a number less than 5 allowing this section to only run once.
                if (retRings == Rings2.FOUR) {
                    cmd[0] = new AutoCommand(AutonomousStates.AutoStates.MOVE_COLOR, Drive.MoveType.CRABRIGHT, 8, 0.4, 0, 0, 6000);
                    cmd[1] = new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.CRABLEFT, 8, 0.4, 0, 0, 1000);
                    cmd[2] = new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.FORWARD, 8, 0.4, 0, 0, 1000);
                    cmd[3] = new AutoCommand(AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP, 80, 0.4, 0, 0, 1000);
                    cmd[4] = new AutoCommand(AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP, 80, 0.4, 0, 0, 1000);
                    cmd[5] = new AutoCommand(AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP, 80, 0.4, 0, 0, 1000);
                    cmd[6] = new AutoCommand(AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP, 80, 0.4, 0, 0, 1000);
                    cmd[7] = new AutoCommand(AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP, 80, 0.4, 0, 0, 1000);
                    cmd[8] = new AutoCommand(AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP, 80, 0.4, 0, 0, 1000);
                    cmd[9] = new AutoCommand(AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP, 18, 0.4, 0, 0, 100);
                    cmd[10] = new AutoCommand(AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP, 18, 0.4, 0, 0, 100);
                    cmd[11] = new AutoCommand(AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP, 18, 0.4, 0, 0, 100);
                    cmd[12] = new AutoCommand(AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP, 18, 0.4, 0, 0, 100);
                    cmd[13] = new AutoCommand(AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP, 18, 0.4, 0, 0, 100);
                    cmd[14] = new AutoCommand(AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP, 18, 0.4, 0, 0, 5000);
                    numRings = 4;
                } else if (retRings == Rings2.ONE) {
                    cmd[0] = new AutoCommand(AutonomousStates.AutoStates.MOVE_COLOR, Drive.MoveType.CRABRIGHT, 80, 0.4, 0, 0, 6000);
                    cmd[1] = new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.CRABRIGHT, 8, 0.4, 0, 0, 1000);
                    cmd[2] = new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.FORWARD, 8, 0.4, 0, 0, 1000);
                    cmd[3] = new AutoCommand(AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP, 80, 0.4, 0, 0, 1000);
                    cmd[4] = new AutoCommand(AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP, 80, 0.4, 0, 0, 1000);
                    cmd[5] = new AutoCommand(AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP, 80, 0.4, 0, 0, 1000);
                    cmd[6] = new AutoCommand(AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP, 80, 0.4, 0, 0, 1000);
                    cmd[7] = new AutoCommand(AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP, 80, 0.4, 0, 0, 1000);
                    cmd[8] = new AutoCommand(AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP, 80, 0.4, 0, 0, 1000);
                    cmd[9] = new AutoCommand(AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP, 18, 0.4, 0, 0, 100);
                    cmd[10] = new AutoCommand(AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP, 18, 0.4, 0, 0, 100);
                    cmd[11] = new AutoCommand(AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP, 18, 0.4, 0, 0, 100);
                    cmd[12] = new AutoCommand(AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP, 18, 0.4, 0, 0, 100);
                    cmd[13] = new AutoCommand(AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP, 18, 0.4, 0, 0, 100);
                    cmd[14] = new AutoCommand(AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP, 18, 0.4, 0, 0, 5000);
                    numRings = 1;
                } else {
                    cmd[0] = new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.CRABLEFT, 8, 0.4, 0, 0, 1000);
                    cmd[1] = new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.REVERSE, 8, 0.4, 0, 0, 1000);
                    cmd[2] = new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.CRABLEFT, 30, 0.4, 0, 0, 5000);
                    cmd[3] = new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.FORWARD, 11, 0.4, 0, 0, 1000);
                    //for MOVE_WOBBLE_ARM, State Value 1 = Wobble Arm Position.
                    cmd[4] = new AutoCommand(AutonomousStates.AutoStates.MOVE_WOBBLE_ARM, Drive.MoveType.REVERSE, 0.08, 0, 0, 0, 2000);
                    //for MOVE_WOBBLE_ARM, State Value 1 = Wobble Arm Position.
                    cmd[5] = new AutoCommand(AutonomousStates.AutoStates.MOVE_WOBBLE_ARM, Drive.MoveType.REVERSE, 0, 0, 0, 0, 2000);
                    cmd[6] = new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.CRABRIGHT, 12, 0.4, 0, 0, 1000);
                    //for SHOOT_RING State Value 1 = Shooter Power, Value 2 = Intake Power, Value 3 = Ring deflector Position.
                    cmd[7] = new AutoCommand(AutonomousStates.AutoStates.SHOOT_RING, Drive.MoveType.REVERSE, .80, .85, 1.0, 0, 2000);
                    cmd[8] = new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.CRABLEFT, 7, 0.4, 0, 0, 5000);
                    cmd[9] = new AutoCommand(AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP, 18, 0.4, 0, 0, 100);
                    cmd[10] = new AutoCommand(AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP, 18, 0.4, 0, 0, 100);
                    cmd[11] = new AutoCommand(AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP, 18, 0.4, 0, 0, 100);
                    cmd[12] = new AutoCommand(AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP, 18, 0.4, 0, 0, 100);
                    cmd[13] = new AutoCommand(AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP, 18, 0.4, 0, 0, 100);
                    cmd[14] = new AutoCommand(AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP, 18, 0.4, 0, 0, 5000);
                    numRings = 0;
                }
            }


            /* *******************************************************************
             *                SENSORS
             *        Inputs:  Sensor Values from robot (not drive motor encoders)
             *        OUTPUTS: parameters containing sensor values
             *
             *********************************************************************/
            if (CurrentTime - LastSensor > SENSORPERIOD) {
                LastSensor = CurrentTime;
                android.graphics.Color.RGBToHSV(robot.color1.red() * 8, robot.color1.green() * 8, robot.color1.blue() * 8, hsvValues);
                currentColor = DetectColor((int) hsvValues[0]);
            }


            /*****************************************************************
             *                NAV
             *      Inputs:  Sensor Values (as needed)
             *               Motor Encoders (accessed through Drive()
             *               IMU Heading    (accessed through Drive()
             *      Outputs: Servo and non drive train Motor commands
             *               drive train Motor commands stored in Drive() class
             ******************************************************************/
            if (CurrentTime - LastNav > NAVPERIOD) {
                LastNav = CurrentTime;
                boolean stage_complete = false;
                stageTime += NAVPERIOD;

                switch (cmd[CurrentAutoState].state) {
                    case MOVE:
                        if (robotDrive.getMoveStatus() == Drive.MoveStatus.AVAILABLE) {
                            robotDrive.move(cmd[CurrentAutoState].moveType, (int) cmd[CurrentAutoState].value1, cmd[CurrentAutoState].value2);
                        }
                        if (robotDrive.getMoveStatus() == Drive.MoveStatus.COMPLETE) {
                            robotDrive.move(Drive.MoveType.STOP, 0, 0);
                            stage_complete = true;
                        }
                        break;
                    case MOVE_COLOR:
                        if (currentColor == Color.YELLOW) {
                            robotDrive.move(Drive.MoveType.STOP, 0, 0);
                            stage_complete = true;
                        } else if (robotDrive.getMoveStatus() == Drive.MoveStatus.AVAILABLE) {
                            robotDrive.move(cmd[CurrentAutoState].moveType, (int) cmd[CurrentAutoState].value1, cmd[CurrentAutoState].value2);
                        }
                        if (robotDrive.getMoveStatus() == Drive.MoveStatus.COMPLETE) {
                            robotDrive.move(Drive.MoveType.STOP, 0, 0);
                            stage_complete = true;
                        }
                        break;
                    case MOVE_WOBBLE_ARM:
                        wobbleTarget = cmd[CurrentAutoState].value1;
                        if (stageTime >= cmd[CurrentAutoState].timeLimit) {
                            robotDrive.move(Drive.MoveType.STOP, 0, 0);
                            stage_complete = true;
                        }
                        break;
                    case SHOOT_RING:
                        //shootRingTime += NAVPERIOD;
                        shooterPower = cmd[CurrentAutoState].value1;
                        intakePower = cmd[CurrentAutoState].value2;
                        ringDeflectorPosition = cmd[CurrentAutoState].value3;

                        if (stageTime < FIRE_RING_TIME && stageTime >= SHOOTER_SPOOL_TIME) {
                            shooterPower = cmd[CurrentAutoState].value1;
                            intakePower = cmd[CurrentAutoState].value2;
                        }

                        if (stageTime >= cmd[CurrentAutoState].timeLimit) {
                            robotDrive.move(Drive.MoveType.STOP, 0, 0);
                            shooterPower = 0.0;
                            intakePower = 0.0;
                            ringDeflectorPosition = 0.0;
                            stage_complete = true;
                        }
                        break;
                            /*
                            if (stageTime < SHOOTER_SPOOL_TIME) {
                                //robotDrive.move(cmd_NONE[CurrentAutoState].moveType, (int)0, 0);
                                shooterPower = cmd_NONE[CurrentAutoState].value1;
                                intakePower = 0.0;
                                ringDeflectorPosition = cmd_NONE[CurrentAutoState].value3;
                            } else if (stageTime < FIRE_RING_TIME && stageTime >= SHOOTER_SPOOL_TIME) {
                                //robotDrive.move(cmd_NONE[CurrentAutoState].moveType, (int)0, 0);
                                shooterPower = cmd_NONE[CurrentAutoState].value1;
                                intakePower = cmd_NONE[CurrentAutoState].value2;
                            } else {
                                //robotDrive.move(cmd_NONE[CurrentAutoState].moveType, (int)0, 0);
                                shooterPower = 0;
                                intakePower = 0;
                                if (CurrentAutoState < (cmd_NONE.length - 1)) {
                                    CurrentAutoState++;
                                }
                            }
                            if (robotDrive.getMoveStatus() == Drive.MoveStatus.COMPLETE) {
                                shooterPower = 0;
                                intakePower = 0;
                                //robotDrive.move(Drive.MoveType.STOP, 0, 0);
                                stage_complete = true;
                            }
                            */
                    case OPEN_CV:

                        break;
                    // Same call, have two to make autonomous code easier to read
                    case PAUSE:
                    case WAIT:
                    default:
                        robotDrive.move(Drive.MoveType.STOP, 0, 0);
                        break;
                }

                /*
                 * Check to see if there is another state to run, Reset stage time, possibly
                 * update/clear local parameters if required (robot specific)
                 */
                if ((stageTime >= cmd[CurrentAutoState].timeLimit) || (stage_complete)) {
                    stageTime = 0;
                    //paddlePower = 0;  //CR Servo
                    if (CurrentAutoState < (cmd.length - 1)) {
                        CurrentAutoState++;
                    }
                }
            }


            /* ***************************************************
             *                MOTOR OUTPUT
             *       Inputs:  Motor power commands
             *       Outputs: Physical interface to the motors
             ****************************************************/
            if (CurrentTime - LastMotor > MOTORPERIOD) {
                LastMotor = CurrentTime;
                robotDrive.update();

                //robot.transportIntake.setPower(intakePower);
                //robot.frontShooter.setPower(shooterPower);
                //robot.backShooter.setPower(shooterPower);
                //robot.wobbleGoalMotor.setPower(0.3);
                //lineAct.move(wobbleTarget, LinearActuator.MOVETYPE.AUTOMATIC);
            }

            /* ***************************************************
             *                SERVO OUTPUT
             *       Inputs:  Motor power commands
             *       Outputs: Physical interface to the servos
             ****************************************************/
            if (CurrentTime - LastServo > SERVOPERIOD) {
                LastServo = CurrentTime;
                robot.ringDefector.setPosition(ringDeflectorPosition);
            }


            /* ***************************************************
             *                TELEMETRY
             *       Inputs:  telemetry structure
             *       Outputs: command telemetry output to phone
             ****************************************************/
            if (CurrentTime - LastTelemetry > TELEMETRYPERIOD) {
                LastTelemetry = CurrentTime;

                //OpenVC needed.
                telemetry.addData("Analysis", retRings);
                telemetry.addData("Position", retRings);
                telemetry.addData("numRings ", numRings);


                telemetry.addData("Color1       : ", currentColor);
                telemetry.addData("WobbleArm    : ", wobbleTarget);
                telemetry.addData("Shooter      : ", shooterPower);
                telemetry.addData("Intake       : ", intakePower);
                telemetry.addData("Current index: ", CurrentAutoState);
                telemetry.addData("Current State: ", cmd[CurrentAutoState].state);
                telemetry.addData("Time Limit   : ", cmd[CurrentAutoState].timeLimit);
                telemetry.addData("Value 1      : ", cmd[CurrentAutoState].value1);
                telemetry.addData("Value 2      : ", cmd[CurrentAutoState].value2);
                telemetry.addData("Value 3      : ", cmd[CurrentAutoState].value3);
                telemetry.addData("Value 4      : ", cmd[CurrentAutoState].value4);
                telemetry.update();
            }
        } // end of while opmode is active

        telemetry.addData("Path", "Complete");
        telemetry.update();
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

        //ensure blueCnt,redCnt,yellowCnt are always greater than 0


        if (hueIn >= blueMin && hueIn <= blueMax) {
            detectedColor = Color.BLUE;
        } else if (hueIn >= redMin || hueIn <= redMax) //Red is a special value when you consider it with the hue values since it wraps around. So we used "OR" to deterimine instead of &&
        {
            detectedColor = Color.RED;
        } else if (hueIn >= yellowMin && hueIn <= yellowMax) {
            detectedColor = Color.YELLOW;
        } else {
            detectedColor = Color.NOCOLOR;
        }

        return (detectedColor);
    }


    //OpenCV needed
    public void setupOpMode() {

    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        //Comment/uncomment the next two lines for an external webcam vs a phones back camera.
        //parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }


}