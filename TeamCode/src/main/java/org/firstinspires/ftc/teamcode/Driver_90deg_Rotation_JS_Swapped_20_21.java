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

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.LinearActuator;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;


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

    double frontRightPower = 0.0;
    double frontLeftPower = 0.0;
    double backRightPower = 0.0;
    double backLeftPower = 0.0;

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

    boolean autoAlignAsserted = false;
    boolean xAxisComplete = false;
    boolean yAxisComplete = false;
    boolean hAxisComplete = false;
    int trackStep = 0;
    final double maxPosX = 21;
    final double minPosX = 11;
    final double maxPosY = 45;
    final double minPosY = 35;
    final double maxPosH = 95;
    final double minPosH = 85;


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

        double shooterIntakePowerSetting = 1.0;
        double ringDeflectorPosition = 1.0;

        double angle = 0.0;
        double magnitude = 0.0;
        double wobbleTarget = 0.0;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        ElapsedTime runtime = new ElapsedTime();
        robot.ringDefector.setPosition(ringDeflectorPosition);
        robot.wobbleGoalMotor.setTargetPosition(0); //Must state that our initial position is refered to as "0" or the "datum"
        lineAct.initialize(robot.wobbleGoalMotor, LinearActuator.ACTUATOR_TYPE.MOTOR_ONLY, 1,
                1440, 1, this, true);

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

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
                    //No Data Means we send back 65535, like Not Availible
                    posY = 65535;
                    posX = 65535;
                    posH = 65535;
                }

            }



            /* **************************************************
             *                Controller INPUT                  *
             *  INPUTS: raw controller values                   *
             *  OUTPUTS:
             ****************************************************/
            if (CurrentTime - LastController > CONTROLLERPERIOD) {
                LastController = CurrentTime;

                //Start shooter before intake moves rings to shooter.
                if (gamepad1.left_trigger > 0.05) {
                    intakePower = (gamepad1.left_trigger * -1);
                } else if (gamepad1.right_trigger > 0.05) {
                    intakePower = gamepad1.right_trigger;
                } else {
                    intakePower = 0;
                }
                if (gamepad2.x) {
                    //Move position is in percentage. Increase angle by 0.000125% for every cycle the gamepad2.a button is pressed.
                    // Equation: Incrementor*CONTROLLERPERIOD*PeriodsButtonIsPressed If "CONTROLLERPERIOD" is 20 (meaning 20ms) That means 50 periods occur in one second (1000ms/20ms) 0.000125%/period * 20 * 50periods = 12.5% increase in angle (45deg) after one second.
                    wobbleTarget += 0.000125 * CONTROLLERPERIOD;
                    wobbleTarget = Math.min(0.59, wobbleTarget);   //Limit on wobble goal arm goin up.
                    lineAct.move(wobbleTarget, LinearActuator.MOVETYPE.AUTOMATIC);
                }
                if (gamepad2.b) {
                    // same as for increment.
                    wobbleTarget -= 0.000125 * CONTROLLERPERIOD;
                    wobbleTarget = Math.max(-0.11, wobbleTarget);   //Limit on the wobble goal going down.
                    lineAct.move(wobbleTarget, LinearActuator.MOVETYPE.AUTOMATIC);
                }
                if (gamepad2.dpad_up) {
                    shooterPower = 1;
                }

                if (gamepad2.dpad_left) {

                    shooterPower = 0.95;
                }

                if (gamepad2.dpad_down) {
                    shooterPower = 0.85;
                }

                if (gamepad2.dpad_right) {
                    shooterPower = 0.75;
                }

                if (!gamepad2.dpad_left && !gamepad2.dpad_right && !gamepad2.dpad_up && !gamepad2.dpad_down) {
                    shooterPower = 0;
                }

                //Do we want to remove the ability for controller one to move the wobble arm?
/*               if (gamepad1.b && posX < 65535) {
                    autoAlignAsserted = true;

                    if (hAxisComplete && xAxisComplete && yAxisComplete) {
                        trackStep = 12;
                        frontLeftPower = 0;
                        frontRightPower = 0;
                        backLeftPower = 0;
                        backRightPower = 0;
                        trackStep = 110;
                    } else if (posX <= maxPosX && posX >= minPosX && xAxisComplete == false) {
                        //do nothing on x-axis
                        xAxisComplete = true;
                        frontLeftPower = 0;
                        frontRightPower = 0;
                        backLeftPower = 0;
                        backRightPower = 0;
                        trackStep = 1;
                    } else if (posX > maxPosX && xAxisComplete == false) {
                        //move backwards (Crab Right)
                        frontLeftPower = -.3;
                        frontRightPower = .3;
                        backLeftPower = .3;
                        backRightPower = -.3;
                        trackStep = 2;
                    } else if (posX < minPosX && xAxisComplete == false) {
                        //move forwards (Crab Left)
                        frontLeftPower = .3;
                        frontRightPower = -.3;
                        backLeftPower = -.3;
                        backRightPower = .3;
                        trackStep = 3;
                    } else if (Math.abs(posY) <= maxPosY && Math.abs(posY) >= minPosY && yAxisComplete == false) {
                        //do nothing on y-axis
                        yAxisComplete = true;
                        frontLeftPower = 0;
                        frontRightPower = 0;
                        backLeftPower = 0;
                        backRightPower = 0;
                        trackStep = 4;
                    } else if (Math.abs(posY) < minPosY && yAxisComplete == false) {
                        //move right (Forward)
                        frontLeftPower = .3;
                        frontRightPower = .3;
                        backLeftPower = .3;
                        backRightPower = .3;
                        trackStep = 5;
                    } else if (Math.abs(posY) > maxPosY && yAxisComplete == false) {
                        //move left (Reverse)
                        frontLeftPower = -.3;
                        frontRightPower = -.3;
                        backLeftPower = -.3;
                        backRightPower = -.3;
                        trackStep = 6;
                    } else if (posH <= maxPosH && posH >= minPosH && hAxisComplete == false) {
                        //do nothing on x-axis
                        hAxisComplete = true;
                        trackStep = 3;
                        frontLeftPower = 0;
                        frontRightPower = 0;
                        backLeftPower = 0;
                        backRightPower = 0;
                        trackStep = 7;

                    } else if (posH > maxPosH && hAxisComplete == false) {
                        frontLeftPower = -.3;
                        frontRightPower = .3;
                        backLeftPower = -.3;
                        backRightPower = .3;
                        trackStep = 9;

                    } else if (posH < minPosH && hAxisComplete == false) {
                        //move
                        frontLeftPower = .3;
                        frontRightPower = -.3;
                        backLeftPower = .3;
                        backRightPower = -.3;


                        trackStep = 8;
                    } else {
                        shooterPower = Math.min(.20, intakePower);
                        trackStep = 4;
                        frontLeftPower = 0;
                        frontRightPower = 0;
                        backLeftPower = 0;
                        backRightPower = 0;
                        trackStep = 10;
                    }
                } else {
                    autoAlignAsserted = false;
                    xAxisComplete = false;
                    yAxisComplete = false;
                    hAxisComplete = false;
                    trackStep = 11;
                } */

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

                if (!autoAlignAsserted) {
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
                    frontRightPower = (-1 * (rightDriveCmd + rightDriveCrab));
                    frontLeftPower = (-1 * (leftDriveCmd + leftDriveCrab));
                    backRightPower = (-1 * (rightRearCmd + rightRearCrab));
                    backLeftPower = (-1 * (leftRearCmd + leftRearCrab));
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
                robot.ringDefector.setPosition(ringDeflectorPosition);
            }


            /* ***************************************************
             *                MOTOR OUTPUT
             *       Inputs:  Motor power commands
             *       Outputs: Physical interface to the motors
             ****************************************************/
            if (CurrentTime - LastMotor > MOTORPERIOD) {
                LastMotor = CurrentTime;
                robot.frontRight.setPower(frontRightPower);
                robot.frontLeft.setPower(frontLeftPower);
                robot.backRight.setPower(backRightPower);
                robot.backLeft.setPower(backLeftPower);

                robot.frontShooter.setPower(shooterPower);
                robot.backShooter.setPower(shooterPower);

                robot.transportIntake.setPower(intakePower);

                robot.wobbleGoalMotor.setPower(0.3);


            }


            /* ***************************************************
             *                TELEMETRY
             *       Inputs:  telemetry structure
             *       Outputs: command telemetry output to phone
             ****************************************************/

            if (CurrentTime - LastTelemetry > TELEMETRYPERIOD) {
                LastTelemetry = CurrentTime;
                telemetry.clear();
                telemetry.addData("Deflector Position Var", ringDeflectorPosition);
                telemetry.addData("Deflector Position Actual", robot.ringDefector.getPosition());
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


