package org.firstinspires.ftc.teamcode;


/*
 * Created for Juden Ki 8578 and Kernel Panic 11959
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
 *  xxxHardwarePushbot         --  Robot class that MUST define and initialize all of the hardware
 *                                 that will be used in autonomous code.
 *
 *  FIRST TIME ON NEW ROBOT
 *      1.  Prune the AutoStates enum and the corresponding case statement to remove references to
 *          hardware/states that do not exists for the new robot.
 *      2.  Configure all of the Drive parameters for the robot  (dParm).  For a fuller explanation
 *          of these parameters refer to the Drive.java Class.   NOTE:  The Drive class is unitless
 *          for distance measurements.  The wheel diameter just needs to be configured int the
 *          desired units.    (Ex:  2.5 if inches, or 6.35 if cm)
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
 * STATE   MOVEMENT          Description and Values 1-3
 ***************************************************************************************************
 * MOVE    FORWARD             Moves robot forward value1 units at speed value2
 *         REVERSE             Moves robot reverse value1 units at speed value2
 *         CRABLEFT            Crabs robot left value1 units at speed value2 (mecanum only)
 *         CRABRIGHT           Crabs robot right value1 units at speed value2 (mecanum only)
 *         PIVOTLEFT           Turns robot to the left to heading value1 at speed value2
 *         PIVOTRIGHT          Turns robot to the right to heading value1 at speed value2
 *         LEFTFORWARD         Engages left side drive to move forward value1 units at speed value2
 *         LEFTREVERSE         Engages left side drive to move reverse value1 units at speed value2
 *         RIGHTFORWARD        Engages right side drive to move forward value1 units at speed value2
 *         RIGHTREVERSE        Engages right side drive to move reverse value1 units at speed value2
 *         STOP                Tells robot to stop moving
 *
 *  FLIP                       flips in or out valu1 degress
 *
 * PAUSE   N/A                  Essentially a sleep state for robot, but not final wait state
 *
 * WAIT    N/A                  Final state should be last called.   (Not strictly necessary)
 *
 */




import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;


public class AutonomousStates {
    //Define Robot Hardware and classes here
    private HardwareDefenition robot        = new HardwareDefenition();   //Remap to your robot
    private Drive robotDrive                = new Drive();
    //private GoldBlockDetection blockDetect  = new GoldBlockDetection();
    private LinearActuator lift    = new LinearActuator();
    private LinearActuator pivot    = new LinearActuator();
    private LinearActuator cascade    = new LinearActuator();





    //Define all of the available states for AutoState.   Add new states before PAUSE
    public enum AutoStates {MOVE, RAISELIFT, LOWERLIFT, GOLD_DETECT, PIVOT_ARM, RAIL_CASCADE, MOVE_COLOR, SHOOT_RING, MOVE_WOBBLE_ARM, MOVE_CAMERA_Y, MOVE_CAMERA_H, MOVE_CAMERA_X, PAUSE, WAIT;}


    //Define AutoState run intervals here
    private final long SENSORPERIOD     = 20;
    private final long SERVOPERIOD      = 20;
    private final long NAVPERIOD        = 20;
    private final long MOTORPERIOD      = 20;
    private final long TELEMETRYPERIOD  = 500;
    private float stageTime             = 0;
    int CurrentAutoState                = 0;
    static final int   THIRTY_SECONDS   = 30 * 1000;



    public void runOpMode(LinearOpMode opMode, HardwareMap hardwareMap, AutoCommand cmd[]) {

        HardwareDefenition.STATUS retVal;
        /*
         * Initialize all of the robot hardware.
         * The init() method of the hardware class does all the work here
         */
        opMode.telemetry.addData("Status", "Initializing Robot...");
        opMode.telemetry.addData("Status", "Configuring Hardware...");
        opMode.telemetry.setAutoClear(false);
        opMode.telemetry.update();
        retVal = robot.init(hardwareMap, true, true, false);
        opMode.telemetry.addData("      ", retVal);
        opMode.telemetry.update();




        /*****************************************************
         *  CONFIGURE THE DRIVE TRAIN  THIS IS ROBOT SPECIFIC
         *****************************************************/
        Drive.Parameters dParm = robotDrive.getParameters();
        dParm.frontRight       = robot.rightFront;
        dParm.frontLeft        = robot.leftFront;
        dParm.rearRight        = robot.rightRear;  //Should be unnecessary, but just keep the nulls away
        dParm.rearLeft         = robot.leftRear;
        dParm.frPolarity       = DcMotorSimple.Direction.REVERSE;
        dParm.flPolarity       = DcMotorSimple.Direction.FORWARD;
        dParm.rrPolarity       = DcMotorSimple.Direction.REVERSE;
        dParm.rlPolarity       = DcMotorSimple.Direction.FORWARD;
        dParm.driveType        = Drive.DriveType.MECANUM;
        dParm.imu              = robot.imu;
        dParm.motorRatio       = 28;
        dParm.gearRatio        = 5.2;
        dParm.wheelDiameter    = 4.0;
        dParm.mecanumAngle     = 45;
        dParm.pivotTolerance   = Drive.PivotTolerance.ONE_DEGREE;
        dParm.encoderTolerance = 75;    //!!VERY DANGEROUS TO PLAY WITH | CAN RESULT IN STUCK STATE!!
        dParm.turnBackoff      = 0.45;  // 35 percent backoff
        dParm.backoffMultiplier = 45;    // Make larger for high speed turns.
        dParm.minStartPower    = 0.1;
        dParm.minTurnPower     = 0.1;
        dParm.opMode           = opMode;
        dParm.debug            = true;
        dParm.useEncoderRatio  = true;
        if (robotDrive.configureDrive(dParm)) {
            opMode.telemetry.addData("Status   ", "Robot Initialized!");
        }
        else {
            opMode.telemetry.addData("Status   ", "INITIALIZATION FAILED!!!!!");
        }



        // Initialize Vuforia
        //blockDetect.configureDetection(opMode);

        /*
         * Perform a little range checking on the supplied array of states.  Look to see if
         * the autonomous time limit will be exceeded
         */
        int    i =0,t = 0;
        while (i < cmd.length) {
            t += cmd[i].timeLimit;
            i++;
        }
        if (t>THIRTY_SECONDS) {
            opMode.telemetry.addLine("WARNING... Autonomous Commands may exceed time");
            opMode.telemetry.addData("   Allowed", THIRTY_SECONDS);
            opMode.telemetry.addData("   Actual ", t);
        }
        opMode.telemetry.update();
        opMode.telemetry.setAutoClear(true);


        /*
         * Define and initialize all of the loop timing variables
         */
        long CurrentTime    = System.currentTimeMillis();
        long LastSensor     = CurrentTime;
        long LastServo      = CurrentTime + 10;
        long LastNav        = CurrentTime + 15;
        long LastMotor      = CurrentTime + 20;
        long LastTelemetry  = CurrentTime + 17;
        ElapsedTime runtime = new ElapsedTime();
        opMode.waitForStart();
        runtime.reset();

        /* ************************************************************
         *            Everything below here runs after START          *
         **************************************************************/
        // run until the end of the match (driver presses STOP)
        while (opMode.opModeIsActive()) {
            CurrentTime = System.currentTimeMillis();


            /* *******************************************************************
             *                SENSORS
             *        Inputs:  Sensor Values from robot (not drive motor encoders)
             *        OUTPUTS: parameters containing sensor values
             *
             *********************************************************************/
            if (CurrentTime - LastSensor > SENSORPERIOD) {
                LastSensor = CurrentTime;
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
                            robotDrive.move(cmd[CurrentAutoState].moveType, (int)cmd[CurrentAutoState].value1,cmd[CurrentAutoState].value2);
                        }
                        if (robotDrive.getMoveStatus() == Drive.MoveStatus.COMPLETE) {
                            robotDrive.move(Drive.MoveType.STOP, 0, 0);
                            stage_complete = true;
                        }
                        break;
                    // Same call, have two to make autonomous code easier to read
                    case RAISELIFT:
                    case LOWERLIFT:
                        lift.move(cmd[CurrentAutoState].value1, LinearActuator.MOVETYPE.AUTOMATIC);
                        if (Math.abs(lift.getCurrentPosition() - cmd[CurrentAutoState].value1) <= .01){
                            stage_complete = true;
                        }
                        break;
                    case PIVOT_ARM:
                        pivot.move(cmd[CurrentAutoState].value1, LinearActuator.MOVETYPE.AUTOMATIC);
                        if (Math.abs(pivot.getCurrentPosition() - cmd[CurrentAutoState].value1) <= .01){
                            stage_complete = true;
                        }
                        break;
                    case RAIL_CASCADE:
                        cascade.move(cmd[CurrentAutoState].value1, LinearActuator.MOVETYPE.AUTOMATIC);
                        if (Math.abs(cascade.getCurrentPosition() - cmd[CurrentAutoState].value1) <= .01){
                            stage_complete = true;
                        }
                        break;
//                    case GOLD_DETECT:
//                        GoldBlockDetection.LOCATION location;
//                        location = blockDetect.detectGoldBlock(cmd[CurrentAutoState].timeLimit-10);  //Subtract a little for overhead.
//                        opMode.telemetry.clear();
//                        opMode.telemetry.addData("GOLD BLOCK IS ", location);
//                        opMode.telemetry.update();
//                        SystemClock.sleep(5000);
//                        stage_complete = true;
//                        break;
                    case PAUSE:
                        break;
                    case WAIT:
                        break;
                    default:
                        robotDrive.move(Drive.MoveType.STOP, 0, 0);
                        break;
                }

                /*
                 * Check to see if there is another state to run, Reset stage time, possibly
                 * update/clear local parameters if required (robot specific)
                 */
                if ((stageTime >= cmd[CurrentAutoState].timeLimit) || (stage_complete)){
                    stageTime = 0;
                    //paddlePower = 0;  //CR Servo
                    if (CurrentAutoState  < (cmd.length - 1)) {
                        CurrentAutoState ++;
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
                //Have the Drive() class run an update on all of the drive train motors
                robotDrive.update();
                lift.update();
                pivot.update();
                cascade.update();

            }

            /* ***************************************************
             *                SERVO OUTPUT
             *       Inputs:  Motor power commands
             *       Outputs: Physical interface to the servos
             ****************************************************/
            if (CurrentTime - LastServo > SERVOPERIOD) {
                LastServo = CurrentTime;
 
            }


            /* ***************************************************
             *                TELEMETRY
             *       Inputs:  telemetry structure
             *       Outputs: command telemetry output to phone
             ****************************************************/
            if (CurrentTime - LastTelemetry > TELEMETRYPERIOD) {
                LastTelemetry = CurrentTime;
                opMode.telemetry.addData("Current index: ", CurrentAutoState);
                opMode.telemetry.addData("Current State: ", cmd[CurrentAutoState].state);
                opMode.telemetry.addData("Current Type : ", cmd[CurrentAutoState].moveType);
                opMode.telemetry.addData("Time Limit   : ", cmd[CurrentAutoState].timeLimit);
                opMode.telemetry.addData("Value 1      : ", cmd[CurrentAutoState].value1);
                opMode.telemetry.addData("Value 2      : ", cmd[CurrentAutoState].value2);
                opMode.telemetry.addData("Value 3      : ", cmd[CurrentAutoState].value3);
                opMode.telemetry.addData("Value 4      : ", cmd[CurrentAutoState].value4);
                opMode.telemetry.update();
            }
        } // end of while opmode is active

        opMode.telemetry.addData("Path", "Complete");
        opMode.telemetry.update();
    }

}
