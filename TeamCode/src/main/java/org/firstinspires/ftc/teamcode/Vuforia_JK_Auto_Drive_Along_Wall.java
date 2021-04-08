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


import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@Autonomous(name = "JK Auto Vuforia Drive Along Wall RED", group = "Pushbot")
@SuppressWarnings("WeakerAccess")
public class Vuforia_JK_Auto_Drive_Along_Wall extends LinearOpMode {
    //Define Robot Hardware and classes here
    private HardwareDef_20_21 robot = new HardwareDef_20_21();
    private Drive robotDrive = new Drive();
    LinearActuator lineAct = new LinearActuator();

    //Define all of the available states for AutoState.   Add new states before PAUSE
    public enum AutoStates {MOVE, RAISELIFT, LOWERLIFT, RING_DETECT, PIVOT_ARM, RAIL_CASCADE, MOVE_COLOR, SHOOT_RING, MOVE_WOBBLE_ARM, MOVE_CAMERA_Y, MOVE_CAMERA_H, MOVE_CAMERA_X, PAUSE, WAIT;}

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;
    private static final String VUFORIA_KEY =
            "Afsw2oP/////AAABmfzepGuVnkr3uHCSQlCl89hgf8A+n9yEsMhH8pfA7Ttz/JfeOCGHzGmHjZMt0IHzaR5tUbVK4L59qd0RJsjAfrmrCsu/CDMm90dy3T9+eRo+zlw6aGRHD0j3EhngUWY0dc1PrRZpWXa+KCLOy3rtB+aWaZDBxILq6uCiqRtLGUwBTrDGQVH/fE1z32YZbDISS5F6actiiu9RhSyU8DKn1EEWeuTj0W+O7lAoDUIhJfJ0B0Iqk73Gfuv2ytbUq89obq9ZVMUqjq9rFsYiztVPOXQkWZsnv8P+eN/0XEgDUmQCU4XBABUw7bTsn9WW/xMDyqnuUMy+AbD/ag5+EPpQaAEGa9nT6n/ATK/Znu47ZlAe";
    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField = 36 * mmPerInch;
    private float posX = -60;
    private float posH = 0;
    private float posY = 0;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private boolean targetVisible = false;
    private float phoneXRotate = 0;
    private float phoneYRotate = 0;
    private float phoneZRotate = 0;
    private TFObjectDetector tfod;


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
    AutonomousStatesJK2019.Color currentColor = AutonomousStatesJK2019.Color.ERROR;
    int numRings = 5;

    public enum Rings2 {NONE, ONE, FOUR;}

    public Rings2 retRings = Rings2.NONE;

    AutoCommand cmd[] = new AutoCommand[18];


    public void runOpMode() {
        cmd[0] = new AutoCommand(AutonomousStates.AutoStates.RING_DETECT, Drive.MoveType.REVERSE, 4, 0.3, 0, 0, 500);
        cmd[1] = new AutoCommand(AutonomousStates.AutoStates.RING_DETECT, Drive.MoveType.CRABLEFT, 10, 0.3, 0, 0, 1000);
        cmd[2] = new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.CRABRIGHT, 10, 0.3, 0, 0, 1000);
        HardwareDef_20_21.STATUS retVal;
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


        /*
         * Perform a little range checking on the supplied array of states.  Look to see if
         * the autonomous time limit will be exceeded
         */
      /*  Range checking will not work here since we are decideing on the array below.
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
        opMode.telemetry.setAutoClear(true);*/


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
        robot.backShooter.setPower(0);
        robot.frontShooter.setPower(0);
        robot.transportIntake.setPower(0);
        robot.wobbleGoalMotor.setTargetPosition(0); //Must state that our initial position is refered to as "0" or the "datum"
        lineAct.initialize(robot.wobbleGoalMotor, LinearActuator.ACTUATOR_TYPE.MOTOR_ONLY, 1, 1440, 1, this, true);

        //long shootRingTime = 0;
        final long FIRE_RING_TIME = 50 * NAVPERIOD;
        final long SHOOTER_SPOOL_TIME = 10 * NAVPERIOD;
        //final long SHOOT_RING_MAX_TIME = 25 * NAVPERIOD;

        double wobbleTarget = 0.0;

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CAMERA_CHOICE;
        //Comment/uncomment the next line for an external webcam vs a phones back camera.
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        // Make sure extended tracking is disabled for this example.
        parameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");
        VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
        blueTowerGoalTarget.setName("Blue Tower Goal Target");
        VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
        redTowerGoalTarget.setName("Red Tower Goal Target");
        VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
        redAllianceTarget.setName("Red Alliance Target");
        VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
        blueAllianceTarget.setName("Blue Alliance Target");
        VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
        frontWallTarget.setName("Front Wall Target");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsUltimateGoal);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        //Set the position of the perimeter targets with relation to origin (center of field)
        redAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        blueAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        frontWallTarget.setLocation(OpenGLMatrix
                .translation(-halfField, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
        blueTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
        redTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));


        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
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

                //Below is Vuforia
                targetsUltimateGoal.activate();
                // check all the trackable targets to see which one (if any) is visible.
                targetVisible = false;
                for (VuforiaTrackable trackable : allTrackables) {
                    if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                        //telemetry.addData("Visible Target", trackable.getName());
                        targetVisible = true;

                        // getUpdatedRobotLocation() will return null if no new information is available since
                        // the last time that call was made, or if the trackable is not currently visible.
                        OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                        if (robotLocationTransform != null) {
                            lastLocation = robotLocationTransform;
                        }
                        break;
                    }
                }
                // Provide feedback as to where the robot is located (if we know).
                if (targetVisible) {
                    // express position (translation) of robot in inches.
                    VectorF translation = lastLocation.getTranslation();
                    //telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f", translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
                    posY = translation.get(1) / mmPerInch;
                    posX = translation.get(0) / mmPerInch;
                    // express the rotation of the robot in degrees.
                    Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                    posH = rotation.thirdAngle;

                    //telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                } else {
                    //telemetry.addData("Visible Target", "none");
                    //robotDrive.move(Drive.MoveType.STOP, 0, 0); //We dont know where we are so dont do anything.
                }
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
                    case RING_DETECT:
                        //I really wanted to make this work with the setup that Howard and Jim made but I could not get it to compile
                        //below is TF
                        if (tfod != null) {
                            // getUpdatedRecognitions() will return null if no new information is available since
                            // the last time that call was made.
                            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                            if (updatedRecognitions != null) {
                                //myOpMode.telemetry.clear();
                                //myOpMode.telemetry.addData("# Object Detected", updatedRecognitions.size());
                                // step through the list of recognitions and display boundary info.
                                int j = 0;
                                for (Recognition recognition : updatedRecognitions) {
                                    //myOpMode.telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                                    //myOpMode.telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f", recognition.getLeft(), recognition.getTop());
                                    //myOpMode.telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",recognition.getRight(), recognition.getBottom());
                                    if (j == 0 && recognition.getLabel() == "Single") {
                                        retRings = Rings2.ONE;
                                    }
                                    if (j == 0 && recognition.getLabel() == "Quad") {
                                        retRings = Rings2.FOUR;

                                    }
                                }
                                //myOpMode.telemetry.addData("RetRings: ", retRings);
                                //myOpMode.telemetry.update();
                                //if (numRings >= 5 && CurrentAutoState >= 1) { //Default is 5.  The below code will change to a number less than 5 allowing this section to only run once.
                                if (retRings == Rings2.FOUR) {
                                    cmd[3] = new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.FORWARD, 15, 0.4, 12, 0, 1000);
                                    cmd[4] = new AutoCommand(AutonomousStates.AutoStates.MOVE_CAMERA_X, Drive.MoveType.CRABLEFT, 60, 0.4, 70, 0, 2000);
                                    cmd[5] = new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.REVERSE, 5, 0.4, 12, 0, 1000);
                                    cmd[6] = new AutoCommand(AutonomousStates.AutoStates.MOVE_WOBBLE_ARM, Drive.MoveType.REVERSE, 0.10, 0, 0, 0, 500);
                                    cmd[7] = new AutoCommand(AutonomousStates.AutoStates.MOVE_WOBBLE_ARM, Drive.MoveType.REVERSE, 0.11, 0, 0, 0, 500);
                                    //                            for MOVE_WOBBLE_ARM, State Value 1 = Wobble Arm Position.
                                    cmd[8] = new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.REVERSE, 10, 0.4, 12, 0, 1000);
                                    cmd[9] = new AutoCommand(AutonomousStates.AutoStates.MOVE_CAMERA_X, Drive.MoveType.CRABRIGHT, 30, 0.4, 18, 1, 1000);
                                    //                            for MOVE_WOBBLE_ARM, State Value 1 = Wobble Arm Position.
                                    cmd[10] = new AutoCommand(AutonomousStates.AutoStates.MOVE_WOBBLE_ARM, Drive.MoveType.REVERSE, 0, 0, 0, 0, 500);
                                    cmd[11] = new AutoCommand(AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP, 18, 0.4, 0, 0, 5000);
                                    cmd[12] = new AutoCommand(AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP, 18, 0.4, 0, 0, 5000);
                                    cmd[13] = new AutoCommand(AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP, 18, 0.4, 0, 0, 5000);
                                    cmd[14] = new AutoCommand(AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP, 18, 0.4, 0, 0, 5000);
                                    cmd[15] = new AutoCommand(AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP, 18, 0.4, 0, 0, 5000);
                                    cmd[16] = new AutoCommand(AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP, 18, 0.4, 0, 0, 5000);
                                    cmd[17] = new AutoCommand(AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP, 18, 0.4, 0, 0, 5000);
                                    numRings = 4;
                                } else if (retRings == Rings2.ONE) {
                                    cmd[3] = new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.FORWARD, 15, 0.4, 12, 0, 1000);
                                    cmd[4] = new AutoCommand(AutonomousStates.AutoStates.MOVE_CAMERA_X, Drive.MoveType.CRABLEFT, 45, 0.4, 36, 0, 2000);
                                    cmd[5] = new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.REVERSE, 25, 0.2, 12, 0, 2000);
                                    cmd[6] = new AutoCommand(AutonomousStates.AutoStates.MOVE_WOBBLE_ARM, Drive.MoveType.REVERSE, 0.10, 0, 0, 0, 500);
                                    cmd[7] = new AutoCommand(AutonomousStates.AutoStates.MOVE_WOBBLE_ARM, Drive.MoveType.REVERSE, 0.11, 0, 0, 0, 500);
                                    cmd[8] = new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.REVERSE, 10, 0.4, 12, 0, 1000);
                                    //                            for MOVE_WOBBLE_ARM, State Value 1 = Wobble Arm Position.
                                    cmd[9] = new AutoCommand(AutonomousStates.AutoStates.MOVE_CAMERA_X, Drive.MoveType.CRABRIGHT, 12, 0.4, 18, 1, 1000);
                                    cmd[10] = new AutoCommand(AutonomousStates.AutoStates.MOVE_WOBBLE_ARM, Drive.MoveType.REVERSE, 0, 0, 0, 0, 500);
                                    //                            for MOVE_WOBBLE_ARM, State Value 1 = Wobble Arm Position.
                                    cmd[11] = new AutoCommand(AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP, 18, 0.4, 0, 0, 5000);
                                    cmd[12] = new AutoCommand(AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP, 18, 0.4, 0, 0, 5000);
                                    cmd[13] = new AutoCommand(AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP, 18, 0.4, 0, 0, 5000);
                                    cmd[14] = new AutoCommand(AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP, 18, 0.4, 0, 0, 5000);
                                    cmd[15] = new AutoCommand(AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP, 18, 0.4, 0, 0, 5000);
                                    cmd[16] = new AutoCommand(AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP, 18, 0.4, 0, 0, 5000);
                                    cmd[17] = new AutoCommand(AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP, 18, 0.4, 0, 0, 5000);
                                    numRings = 1;
                                    // Always seems to see none, still trying to see if we can mechanically fix that...
                                } else {
                                    //new AutoCommand(AutonomousStates.AutoStates.MOVE_CAMERA_X, Drive.MoveType.CRABLEFT, 20, 0.4, 12, 0, 1500),
                                    cmd[3] = new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.FORWARD, 15, 0.4, 12, 0, 1000);
                                    cmd[4] = new AutoCommand(AutonomousStates.AutoStates.MOVE_CAMERA_X, Drive.MoveType.CRABLEFT, 37, 0.4, 20, 0, 1000);
                                    cmd[5] = new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.REVERSE, 5, 0.4, 12, 0, 1000);
                                    cmd[6] = new AutoCommand(AutonomousStates.AutoStates.MOVE_WOBBLE_ARM, Drive.MoveType.REVERSE, 0.10, 0, 0, 0, 500);
                                    cmd[7] = new AutoCommand(AutonomousStates.AutoStates.MOVE_WOBBLE_ARM, Drive.MoveType.REVERSE, 0.11, 0, 0, 0, 500);
                                    cmd[8] = new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.REVERSE, 10, 0.4, 12, 0, 1000);
                                    //                            for MOVE_WOBBLE_ARM, State Value 1 = Wobble Arm Position.
                                    cmd[9] = new AutoCommand(AutonomousStates.AutoStates.MOVE_WOBBLE_ARM, Drive.MoveType.REVERSE, 0, 0, 0, 0, 500);
                                    //                            for MOVE_WOBBLE_ARM, State Value 1 = Wobble Arm Position.
                                    cmd[10] = new AutoCommand(AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP, 18, 0.4, 0, 0, 5000);
                                    cmd[11] = new AutoCommand(AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP, 18, 0.4, 0, 0, 5000);
                                    cmd[12] = new AutoCommand(AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP, 18, 0.4, 0, 0, 5000);
                                    cmd[13] = new AutoCommand(AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP, 18, 0.4, 0, 0, 5000);
                                    cmd[14] = new AutoCommand(AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP, 18, 0.4, 0, 0, 5000);
                                    cmd[15] = new AutoCommand(AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP, 18, 0.4, 0, 0, 5000);
                                    cmd[16] = new AutoCommand(AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP, 18, 0.4, 0, 0, 5000);
                                    cmd[17] = new AutoCommand(AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP, 18, 0.4, 0, 0, 5000);
                                    numRings = 0;
                                }
                            }
                            //}
                        }//purposly making this part of the move.
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
                        if (currentColor == AutonomousStatesJK2019.Color.YELLOW) {
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
                        } else if (stageTime > FIRE_RING_TIME) {
                            stage_complete = true;
                        }

                        if (stageTime >= cmd[CurrentAutoState].timeLimit) {
                            robotDrive.move(Drive.MoveType.STOP, 0, 0);
                            shooterPower = 0.0;
                            intakePower = 0.0;
                            ringDeflectorPosition = 0.0;
                            stage_complete = true;
                        }
                        break;
                    // Same call, have two to make autonomous code easier to read
                    case MOVE_CAMERA_X:
                        if (posX > cmd[CurrentAutoState].value3 && cmd[CurrentAutoState].value4 == 0) {
                            robotDrive.move(Drive.MoveType.STOP, 0, 0);
                            stage_complete = true;
                        } else if (posX < cmd[CurrentAutoState].value3 && cmd[CurrentAutoState].value4 > 0) {
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
                    case MOVE_CAMERA_Y:
                        if (posY > cmd[CurrentAutoState].value3 && cmd[CurrentAutoState].value4 == 0) {
                            robotDrive.move(Drive.MoveType.STOP, 0, 0);
                            stage_complete = true;
                        } else if (posY < cmd[CurrentAutoState].value3 && cmd[CurrentAutoState].value4 > 0) {
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
                    case MOVE_CAMERA_H:
                        if (posH > cmd[CurrentAutoState].value1 && cmd[CurrentAutoState].value4 == 0) {
                            robotDrive.move(Drive.MoveType.STOP, 0, 0);
                            stage_complete = true;
                        } else if (posH < cmd[CurrentAutoState].value1 && cmd[CurrentAutoState].value4 > 0) {
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

                robot.transportIntake.setPower(intakePower);
                robot.frontShooter.setPower(shooterPower);
                robot.backShooter.setPower(shooterPower);
                robot.wobbleGoalMotor.setPower(0.3);
                lineAct.move(wobbleTarget, LinearActuator.MOVETYPE.AUTOMATIC);
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
                telemetry.addData("RetRing", retRings);
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

    public static AutonomousStatesJK2019.Color DetectColor(int hueIn) {
        final int blueMin = 197; //Blue Starts at 197deg and goes to 217deg.
        final int blueMax = 217;
        final int redMin = 351; //Red Starts at 351deg and wraps around to 11deg.
        final int redMax = 11;
        final int yellowMin = 141; //Yellow Starts at 102deg and goes to 122deg. Although we want white, we named this yellow.
        final int yellowMax = 165;


        AutonomousStatesJK2019.Color detectedColor = AutonomousStatesJK2019.Color.ERROR;

        //ensure blueCnt,redCnt,yellowCnt are always greater than 0


        if (hueIn >= blueMin && hueIn <= blueMax) {
            detectedColor = AutonomousStatesJK2019.Color.BLUE;
        } else if (hueIn >= redMin || hueIn <= redMax) //Red is a special value when you consider it with the hue values since it wraps around. So we used "OR" to deterimine instead of &&
        {
            detectedColor = AutonomousStatesJK2019.Color.RED;
        } else if (hueIn >= yellowMin && hueIn <= yellowMax) {
            detectedColor = AutonomousStatesJK2019.Color.YELLOW;
        } else {
            detectedColor = AutonomousStatesJK2019.Color.NOCOLOR;
        }

        return (detectedColor);
    }


}