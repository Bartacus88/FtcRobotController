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




import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


public class AutonomousStatesJK2019 {
    //Define Robot Hardware and classes here
    private HardwareDef_20_21 robot = new HardwareDef_20_21();
    private Drive robotDrive = new Drive();
    private RingStackDetection2 RingStackDetection2 = new RingStackDetection2();

    //Define all of the available states for AutoState.   Add new states before PAUSE
    public enum AutoStates {MOVE, MOVE_COLOR, SHOOT_RING, PAUSE, WAIT;}


    //Define AutoState run intervals here
    private final long SENSORPERIOD     = 20;
    private final long SERVOPERIOD      = 20;
    private final long NAVPERIOD        = 20;
    private final long MOTORPERIOD = 20;
    private final long TELEMETRYPERIOD = 500;
    private float stageTime = 0;
    int CurrentAutoState = 0;
    static final int THIRTY_SECONDS = 30 * 1000;

    //Needed to check "starter rings"
    RingStackDetection2.Rings2 numRingsTest;
    boolean ringschecked = false;
    int ringArray = 0;

    public enum Color {
        NOCOLOR, //0
        ERROR,   //1
        BLUE,    //2
        RED,     //3
        YELLOW   //4
    }

    float hsvValues[] = {0F, 0F, 0F};
    Color currentColor = Color.ERROR;

    public void runOpMode(LinearOpMode opMode, HardwareMap hardwareMap, AutoCommand cmd_FOUR[], AutoCommand cmd_ONE[], AutoCommand cmd_NONE[]) {

        HardwareDef_20_21.STATUS retVal;
        /*
         * Initialize all of the robot hardware.
         * The init() method of the hardware class does all the work here
         */
        opMode.telemetry.addData("Status", "Initializing Robot...");
        opMode.telemetry.addData("Status", "Configuring Hardware...");
        opMode.telemetry.setAutoClear(false);
        opMode.telemetry.update();
        retVal = robot.init(hardwareMap, true, false, false);
        opMode.telemetry.addData("      ", retVal);
        opMode.telemetry.update();




        /*****************************************************
         *  CONFIGURE THE DRIVE TRAIN  THIS IS ROBOT SPECIFIC
         *****************************************************/
        Drive.Parameters dParm = robotDrive.getParameters();
        //Why didn't the example have these two lines below?  I, spent hours with this not working, until I added these.
        dParm.cenPolarity      = DcMotorSimple.Direction.FORWARD;
        dParm.center           = robot.frontRight; //Just to remove error, serves no purpose as far as I can tell

        dParm.frontRight       = robot.frontRight;
        dParm.frontLeft        = robot.frontRight;
        dParm.rearRight        = robot.backRight;  //Should be unnecessary, but just keep the nulls away
        dParm.rearLeft         = robot.backLeft;
        dParm.frPolarity       = DcMotorSimple.Direction.FORWARD;
        dParm.flPolarity       = DcMotorSimple.Direction.REVERSE;
        dParm.rrPolarity       = DcMotorSimple.Direction.FORWARD;
        dParm.rlPolarity       = DcMotorSimple.Direction.REVERSE;
        dParm.driveType        = Drive.DriveType.MECANUM;
        dParm.imu              = robot.imu;
        dParm.motorRatio       = 28;
        dParm.gearRatio        = 20;
        dParm.wheelDiameter    = 3.0;
        dParm.mecanumAngle = 45;
        dParm.pivotTolerance = Drive.PivotTolerance.FIVE_DEGREES;
        dParm.encoderTolerance = (int) dParm.motorRatio * 5;    //!!VERY DANGEROUS TO PLAY WITH | CAN RESULT IN STUCK STATE!!
        dParm.turnBackoff      = 0.45;  // 45 percent backoff
        dParm.backoffMultiplier = 25;    // Make larger for high speed turns.
        dParm.minStartPower    = 0.1;
        dParm.minTurnPower     = 0.1;
        dParm.opMode           = opMode;
        dParm.debug            = true;
        dParm.useEncoderRatio  = false;
        if (robotDrive.configureDrive(dParm)) {
            opMode.telemetry.addData("Status   ", "Robot Initialized!");
        }
        else {
            opMode.telemetry.addData("Status   ", "INITIALIZATION FAILED!!!!!");
        }


        /*
         * Perform a little range checking on the supplied array of states.  Look to see if
         * the autonomous time limit will be exceeded
         */
        int    i =0,t = 0;
        while (i < cmd_NONE.length) {
            t += cmd_NONE[i].timeLimit;
            i++;
        }
        if (t>THIRTY_SECONDS) {
            opMode.telemetry.addLine("WARNING... Autonomous Commands may exceed time");
            opMode.telemetry.addData("   Allowed", THIRTY_SECONDS);
            opMode.telemetry.addData("   Actual ", t);
        }
        opMode.telemetry.update();
        opMode.telemetry.setAutoClear(true);
        while (i < cmd_ONE.length) {
            t += cmd_ONE[i].timeLimit;
            i++;
        }
        if (t>THIRTY_SECONDS) {
            opMode.telemetry.addLine("WARNING... Autonomous Commands may exceed time");
            opMode.telemetry.addData("   Allowed", THIRTY_SECONDS);
            opMode.telemetry.addData("   Actual ", t);
        }
        opMode.telemetry.update();
        opMode.telemetry.setAutoClear(true);
        while (i < cmd_FOUR.length) {
            t += cmd_FOUR[i].timeLimit;
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
        long CurrentTime = System.currentTimeMillis();
        long LastSensor = CurrentTime;
        long LastServo = CurrentTime + 10;
        long LastNav = CurrentTime + 15;
        long LastMotor = CurrentTime + 20;
        long LastTelemetry = CurrentTime + 17;
        ElapsedTime runtime = new ElapsedTime();

        RingStackDetection2.configureDetection(opMode);


        double shooterPower = 0.0;
        double intakePower = 0.0;
        double ringDeflectorPosition = 0.0;
        robot.backShooter.setPower(0);
        robot.frontShooter.setPower(0);
        robot.transportIntake.setPower(0);

        //long shootRingTime = 0;
        final long FIRE_RING_TIME = 10 * NAVPERIOD;
        final long SHOOTER_SPOOL_TIME = 5 * NAVPERIOD;
        //final long SHOOT_RING_MAX_TIME = 25 * NAVPERIOD;

        opMode.waitForStart();
        runtime.reset();

        /* ************************************************************
         *            Everything below here runs after START          *
         **************************************************************/
        // run until the end of the match (driver presses STOP)
        while (opMode.opModeIsActive()) {
            CurrentTime = System.currentTimeMillis();
            RingStackDetection2.detectRingStack(1000);
            numRingsTest = RingStackDetection2.retRings;

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

                if (numRingsTest == org.firstinspires.ftc.teamcode.RingStackDetection2.Rings2.FOUR)
                    switch (cmd_FOUR[CurrentAutoState].state) {
                        case MOVE:
                            if (robotDrive.getMoveStatus() == Drive.MoveStatus.AVAILABLE) {
                                robotDrive.move(cmd_FOUR[CurrentAutoState].moveType, (int) cmd_FOUR[CurrentAutoState].value1, cmd_FOUR[CurrentAutoState].value2);
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
                                robotDrive.move(cmd_FOUR[CurrentAutoState].moveType, (int) cmd_FOUR[CurrentAutoState].value1, cmd_FOUR[CurrentAutoState].value2);
                            }
                            if (robotDrive.getMoveStatus() == Drive.MoveStatus.COMPLETE) {
                                robotDrive.move(Drive.MoveType.STOP, 0, 0);
                                stage_complete = true;
                            }
                            break;
                        case PAUSE:
                        case WAIT:
                        default:
                            robotDrive.move(Drive.MoveType.STOP, 0, 0);
                            break;
                    }
                else if (numRingsTest == org.firstinspires.ftc.teamcode.RingStackDetection2.Rings2.ONE) {
                    switch (cmd_ONE[CurrentAutoState].state) {
                        case MOVE:
                            if (robotDrive.getMoveStatus() == Drive.MoveStatus.AVAILABLE) {
                                robotDrive.move(cmd_ONE[CurrentAutoState].moveType, (int) cmd_ONE[CurrentAutoState].value1, cmd_ONE[CurrentAutoState].value2);
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
                                robotDrive.move(cmd_ONE[CurrentAutoState].moveType, (int) cmd_ONE[CurrentAutoState].value1, cmd_ONE[CurrentAutoState].value2);
                            }
                            if (robotDrive.getMoveStatus() == Drive.MoveStatus.COMPLETE) {
                                robotDrive.move(Drive.MoveType.STOP, 0, 0);
                                stage_complete = true;
                            }
                            break;
                        case PAUSE:
                        case WAIT:
                        default:
                            robotDrive.move(Drive.MoveType.STOP, 0, 0);
                            break;
                    }
                } else {
                    switch (cmd_NONE[CurrentAutoState].state) {
                        case MOVE:
                            if (robotDrive.getMoveStatus() == Drive.MoveStatus.AVAILABLE) {
                                robotDrive.move(cmd_ONE[CurrentAutoState].moveType, (int) cmd_NONE[CurrentAutoState].value1, cmd_NONE[CurrentAutoState].value2);
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
                                robotDrive.move(cmd_NONE[CurrentAutoState].moveType, (int) cmd_NONE[CurrentAutoState].value1, cmd_NONE[CurrentAutoState].value2);
                            }
                            if (robotDrive.getMoveStatus() == Drive.MoveStatus.COMPLETE) {
                                robotDrive.move(Drive.MoveType.STOP, 0, 0);
                                stage_complete = true;
                            }
                            break;
                        case SHOOT_RING:

                            //shootRingTime += NAVPERIOD;
                            shooterPower = cmd_NONE[CurrentAutoState].value1;
                            intakePower = 0.0;
                            ringDeflectorPosition = cmd_NONE[CurrentAutoState].value3;

                            if (stageTime < FIRE_RING_TIME && stageTime >= SHOOTER_SPOOL_TIME) {
                                shooterPower = cmd_NONE[CurrentAutoState].value1;
                                intakePower = cmd_NONE[CurrentAutoState].value2;
                            }

                            if (robotDrive.getMoveStatus() == Drive.MoveStatus.COMPLETE || stageTime >= 4000) {
                                robotDrive.move(Drive.MoveType.STOP, 0, 0);
                                shooterPower = 0.0;
                                intakePower = 0.0;
                                ringDeflectorPosition = 0.0;
                                //stageTime = 0;
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
                            break;*/
                        case PAUSE:
                        case WAIT:
                        default:
                            robotDrive.move(Drive.MoveType.STOP, 0, 0);
                            break;
                    }
                }

                /*
                 * Check to see if there is another state to run, Reset stage time, possibly
                 * update/clear local parameters if required (robot specific)
                 */
                if (numRingsTest == org.firstinspires.ftc.teamcode.RingStackDetection2.Rings2.FOUR) {
                    if ((stageTime >= cmd_FOUR[CurrentAutoState].timeLimit) || (stage_complete)) {
                        stageTime = 0;
                        //paddlePower = 0;  //CR Servo
                        if (CurrentAutoState < (cmd_FOUR.length - 1)) {
                            CurrentAutoState++;
                        }
                    }
                }
                else if (numRingsTest == org.firstinspires.ftc.teamcode.RingStackDetection2.Rings2.ONE) {
                    if ((stageTime >= cmd_ONE[CurrentAutoState].timeLimit) || (stage_complete)) {
                        stageTime = 0;
                        //paddlePower = 0;  //CR Servo
                        if (CurrentAutoState < (cmd_ONE.length - 1)) {
                            CurrentAutoState++;
                        }
                    }
                }
                else
                {
                    if ((stageTime >= cmd_NONE[CurrentAutoState].timeLimit) || (stage_complete)) {
                        stageTime = 0;
                        //paddlePower = 0;  //CR Servo
                        if (CurrentAutoState < (cmd_NONE.length - 1)) {
                            CurrentAutoState++;
                        }
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
                robot.transportIntake.setPower(intakePower);
                robot.frontShooter.setPower(shooterPower);
                robot.backShooter.setPower(shooterPower);

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
                if (numRingsTest == org.firstinspires.ftc.teamcode.RingStackDetection2.Rings2.FOUR) {
                    opMode.telemetry.addData("Num Rings Test ", RingStackDetection2.retRings);
                    opMode.telemetry.addData("Current index: ", CurrentAutoState);
                    opMode.telemetry.addData("Current State: ", cmd_FOUR[CurrentAutoState].state);
                    opMode.telemetry.addData("Time Limit   : ", cmd_FOUR[CurrentAutoState].timeLimit);
                    opMode.telemetry.addData("Value 1      : ", cmd_FOUR[CurrentAutoState].value1);
                    opMode.telemetry.addData("Value 2      : ", cmd_FOUR[CurrentAutoState].value2);
                    opMode.telemetry.addData("Value 3      : ", cmd_FOUR[CurrentAutoState].value3);
                    opMode.telemetry.addData("Value 4      : ", cmd_FOUR[CurrentAutoState].value4);
                    opMode.telemetry.update();
                }
                else if (numRingsTest == org.firstinspires.ftc.teamcode.RingStackDetection2.Rings2.ONE) {
                    opMode.telemetry.addData("Num Rings Test ", RingStackDetection2.retRings);
                    opMode.telemetry.addData("Current index: ", CurrentAutoState);
                    opMode.telemetry.addData("Current State: ", cmd_ONE[CurrentAutoState].state);
                    opMode.telemetry.addData("Time Limit   : ", cmd_ONE[CurrentAutoState].timeLimit);
                    opMode.telemetry.addData("Value 1      : ", cmd_ONE[CurrentAutoState].value1);
                    opMode.telemetry.addData("Value 2      : ", cmd_ONE[CurrentAutoState].value2);
                    opMode.telemetry.addData("Value 3      : ", cmd_ONE[CurrentAutoState].value3);
                    opMode.telemetry.addData("Value 4      : ", cmd_ONE[CurrentAutoState].value4);
                    opMode.telemetry.update();
                }
                else
                {
                    opMode.telemetry.addData("Color1       : ", currentColor);
                    opMode.telemetry.addData("Num Rings Test ", RingStackDetection2.retRings);
                    opMode.telemetry.addData("Current index: ", CurrentAutoState);
                    opMode.telemetry.addData("Current State: ", cmd_NONE[CurrentAutoState].state);
                    opMode.telemetry.addData("Time Limit   : ", cmd_NONE[CurrentAutoState].timeLimit);
                    opMode.telemetry.addData("Value 1      : ", cmd_NONE[CurrentAutoState].value1);
                    opMode.telemetry.addData("Value 2      : ", cmd_NONE[CurrentAutoState].value2);
                    opMode.telemetry.addData("Value 3      : ", cmd_NONE[CurrentAutoState].value3);
                    opMode.telemetry.addData("Value 4      : ", cmd_NONE[CurrentAutoState].value4);

                    opMode.telemetry.update();
                }
            }
        } // end of while opmode is active
        //opMode.telemetry.addData("Num Rings Test ", RingStackDetection2.retRings);
        //opMode.telemetry.addData("Path", "Complete");
        //opMode.telemetry.update();
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
        }

        if (hueIn >= redMin || hueIn <= redMax) //Red is a special value when you consider it with the hue values since it wraps around. So we used "OR" to deterimine instead of &&
        {
            detectedColor = Color.RED;
        }

        if (hueIn >= yellowMin && hueIn <= yellowMax) {
            detectedColor = Color.YELLOW;
        }

        return (detectedColor);
    }


  /*
    public static Color DetectColor(int hueIn) {
        final int blueMin = 220; //Blue Starts at 220deg and goes to 240deg.
        final int blueMax = 240;
        final int redMin = 350; //Red Starts at 350deg and wraps around to 10deg.
        final int redMax = 10;
        final int yellowMin = 45; //Yellow Starts at 45deg and goes to 65deg
        final int yellowMax = 65;

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
        blueCnt = Math.min(blueCnt, 2);
        redCnt = Math.min(redCnt, 2);
        yellowCnt = Math.min(yellowCnt, 2);
        blueCnt = Math.max(blueCnt, 0);
        redCnt = Math.max(redCnt, 0);
        yellowCnt = Math.max(yellowCnt, 0);

        //blueCnt = 0 redCnt = 0 yellow = 4
        if (blueCnt >= 1 && redCnt <= 0 && yellowCnt <= 0) {
            //System.out.print("Blue is your color");
            detectedColor = Color.BLUE;
        } else if (redCnt >= 1 && yellowCnt <= 0 && blueCnt <= 0) {
            //System.out.print("Red is your color");
            detectedColor = Color.RED;
        } else if (yellowCnt >= 1 && blueCnt <= 0 && redCnt <= 0) {
            //System.out.print("Yellow is your color");
            detectedColor = Color.YELLOW;
        } else {
            //System.out.print("No color detected");
            detectedColor = Color.NOCOLOR;
        }

        return (detectedColor);

    } */


}
