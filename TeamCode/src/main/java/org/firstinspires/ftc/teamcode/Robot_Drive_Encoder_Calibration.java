package org.firstinspires.ftc.teamcode;

/* *
 * Created by James Rumsey 1/18/19
 *
 *
 *
 */

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@SuppressWarnings("WeakerAccess")
@TeleOp(name = "Robot Drive Encoder Calibration", group = "Calibration")
@Disabled
public class Robot_Drive_Encoder_Calibration extends LinearOpMode {
    HardwareDefenition robot = new HardwareDefenition();
    private Drive robotDrive = new Drive();


    private enum CAL_MODE {MANUAL, ENCODER, SAVE, TEST, THRESHOLD}

    ;
    CAL_MODE mode = CAL_MODE.MANUAL;
    final long SENSORPERIOD = 20;
    final long ENCODERPERIOD = 20;
    final long SERVOPERIOD = 20;
    final long NAVPERIOD = 20;
    final long MOTORPERIOD = 20;
    final long CONTROLLERPERIOD = 20;
    final long TELEMETRYPERIOD = 500;



    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */


        /*
         * Initialize all of the robot hardware.
         * The init() method of the hardware class does all the work here
         */
        telemetry.addData("Status", "Initializing Robot...");
        telemetry.addData("Status", "Configuring Hardware...");
        telemetry.setAutoClear(false);
        telemetry.update();
        robot.init(hardwareMap, true, false, false);
        telemetry.addData("      ", "SUCCESS!");
        telemetry.update();


        /*****************************************************
         *  CONFIGURE THE DRIVE TRAIN  THIS IS ROBOT SPECIFIC
         *****************************************************/
        Drive.Parameters dParm = robotDrive.getParameters();
        dParm.frontRight = robot.rightFront;
        dParm.frontLeft = robot.leftFront;
        dParm.rearRight = robot.rightRear;  //Should be unnecessary, but just keep the nulls away
        dParm.rearLeft = robot.leftRear;
        dParm.frPolarity = DcMotorSimple.Direction.REVERSE;
        dParm.flPolarity = DcMotorSimple.Direction.FORWARD;
        dParm.rrPolarity = DcMotorSimple.Direction.REVERSE;
        dParm.rlPolarity = DcMotorSimple.Direction.FORWARD;
        dParm.driveType = Drive.DriveType.MECANUM;
        dParm.imu = robot.imu;
        dParm.motorRatio = 28;
        dParm.gearRatio = 40;
        dParm.wheelDiameter = 4.0;
        dParm.mecanumAngle = 45;
        dParm.pivotTolerance = Drive.PivotTolerance.ONE_DEGREE;
        dParm.encoderTolerance = 15;    //!!VERY DANGEROUS TO PLAY WITH | CAN RESULT IN STUCK STATE!!
        dParm.turnBackoff = 0.45;  // 35 percent backoff
        dParm.backoffMultiplier = 45;    // Make larger for high speed turns.
        dParm.minStartPower = 0.3;
        dParm.minTurnPower = 0.3;
        dParm.opMode = this;
        dParm.debug = true;
        dParm.useEncoderRatio = true;
        if (robotDrive.configureDrive(dParm)) {
            telemetry.addData("Status   ", "Robot Initialized!");
        } else {
            telemetry.addData("Status   ", "INITIALIZATION FAILED!!!!!");
        }

        long CurrentTime = System.currentTimeMillis();
        long LastSensor = CurrentTime;
        long LastEncoderRead = CurrentTime + 5;
        long LastServo = CurrentTime + 10;
        long LastNav = CurrentTime + 15;
        long LastMotor = CurrentTime + 20;
        long LastController = CurrentTime + 7;
        long LastTelemetry = CurrentTime + 17;

        // variables for controller inputs.
        float g1_LeftX = 0;
        float g1_LeftY = 0;
        float g1_RightX = 0;
        float g1_RightY = 0;
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


        int g2_A_Counts = 0;
        int g2_DU_Counts = 0;
        int count = 0;


        float leftDriveCmd = 0;
        float rightDriveCmd = 0;
        float leftRearCmd = 0;
        float rightRearCmd = 0;
        float leftDriveCrab = 0;
        float rightDriveCrab = 0;
        float leftRearCrab = 0;
        float rightRearCrab = 0;
        float g1_X_Threshold = (float) 0.0;
        float g1_Y_Threshold = (float) 0.0;
        float g1_Crab_Threshold = (float) 0.0;
        float driveMin = -1;
        float driveMax = 1;


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        ElapsedTime runtime = new ElapsedTime();
        waitForStart();
        runtime.reset();

        /* ************************************************************
         *            Everything below here  \\ press START           *
         **************************************************************/
        while (opModeIsActive()) {
            CurrentTime = System.currentTimeMillis();


            /* ***************************************************
             *                SENSORS
             *        INPUTS: Raw Sensor Values
             *       OUTPUTS: parameters containing sensor values*
             ****************************************************/
            if (CurrentTime - LastSensor > SENSORPERIOD) {
                LastSensor = CurrentTime;
            }

            /* ***************************************************
             *                ENCODERS                          *
             ****************************************************/
            if (CurrentTime - LastEncoderRead > ENCODERPERIOD) {
                LastEncoderRead = CurrentTime;

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
            }


            /* **************************************************
             *                NAV
             *      Inputs:  Gamepad positions
             *               Sensor Values (as needed)
             *      Outputs: Servo and Motor position commands
             *                         motor
             ****************************************************/
            if (CurrentTime - LastNav > NAVPERIOD) {
                LastNav = CurrentTime;

                //Select tthe calibration Mode
                if (g1_DD) {
                    mode = CAL_MODE.MANUAL;
                }
                if (g1_DU) {
                    mode = CAL_MODE.ENCODER;
                    dParm.useEncoderRatio = false;
                    if (robotDrive.configureDrive(dParm)) {
                        telemetry.addData("Status   ", "Robot Initialized!");
                    } else {
                        telemetry.addData("Status   ", "INITIALIZATION FAILED!!!!!");
                    }
                }
                if (g1_DL) {
                    mode = CAL_MODE.TEST;
                    dParm.useEncoderRatio = true;
                    if (robotDrive.configureDrive(dParm)) {
                        telemetry.addData("Status   ", "Robot Initialized!");
                    } else {
                        telemetry.addData("Status   ", "INITIALIZATION FAILED!!!!!");
                    }
                }
                if (g1_DR) {
                    mode = CAL_MODE.THRESHOLD;
                    dParm.useEncoderRatio = false;
                    if (robotDrive.configureDrive(dParm)) {
                        telemetry.addData("Status   ", "Robot Initialized!");
                    } else {
                        telemetry.addData("Status   ", "INITIALIZATION FAILED!!!!!");
                    }
                }
                if (g1_RB && g1_LB) {
                    mode = CAL_MODE.SAVE;
                }


                switch (mode) {
                    case MANUAL:
                        // TANK STYLE DRIVE
                        if (Math.abs(g1_LeftY) > g1_Y_Threshold) {
                            //Move forwards or backwards
                            leftDriveCmd = Range.clip(g1_LeftY * g1_LeftY * g1_LeftY, driveMin, driveMax);
                            rightDriveCmd = leftDriveCmd;
                            leftRearCmd = leftDriveCmd;
                            rightRearCmd = leftDriveCmd;
                        }
                        if (Math.abs(g1_LeftX) > g1_X_Threshold) {
                            //Pivot
                            leftDriveCmd = Range.clip(g1_LeftX * g1_LeftX * g1_LeftX, driveMin, driveMax);
                            rightDriveCmd = -1 * Range.clip(g1_LeftX * g1_LeftX * g1_LeftX, driveMin, driveMax);
                            leftRearCmd = leftDriveCmd;
                            rightRearCmd = rightDriveCmd;
                        }

                        // CRAB STYLE DRIVE
                        if (Math.abs(g1_RightY) > g1_Crab_Threshold) {  //Forward, backward
                            leftDriveCrab = Range.clip(g1_RightY * g1_RightY * g1_RightY, driveMin, driveMax);
                            rightDriveCrab = leftDriveCrab;
                            leftRearCrab = leftDriveCrab;
                            rightRearCrab = leftDriveCrab;
                        }
                        if (Math.abs(g1_RightX) > g1_Crab_Threshold) {
                            leftDriveCrab = -1 * Range.clip(g1_RightX * g1_RightX * g1_RightX, driveMin, driveMax);
                            rightDriveCrab = Range.clip(g1_RightX * g1_RightX * g1_RightX, driveMin, driveMax);
                            leftRearCrab = rightDriveCrab;
                            rightRearCrab = leftDriveCrab;
                        }
                        break;
                    case TEST:
                        if (g1_A) {
                            robotDrive.move(Drive.MoveType.FORWARD, 18, .9);
                            sleep(2000);
                        } else if (g1_B) {
                            robotDrive.move(Drive.MoveType.REVERSE, 18, .9);
                            sleep(2000);
                        } else if (g1_X) {
                            robotDrive.move(Drive.MoveType.CRABLEFT, 18, .9);
                            sleep(2000);
                        } else if (g1_Y) {
                            robotDrive.move(Drive.MoveType.CRABRIGHT, 18, .9);
                            sleep(2000);
                        }
                        break;
                    case ENCODER:
                        if (g1_A) {

                            robotDrive.calibrateEncoders();
                        }
                        break;
                    case THRESHOLD:
                        robotDrive.motorPowerThreshold();
                        break;
                    case SAVE:
                        if (g1_A) {
                            robotDrive.saveEncoderData();
                            sleep(2000);
                        }
                        if (g1_B) {
                            robotDrive.loadEncoderData();
                            sleep(2000);
                        }
                        if (g1_X) {
                            robotDrive.resetEncoderConfiguration();
                            sleep (2000);
                        }
                        if (g1_Y) {
                            telemetry.clear();
                            robotDrive.displayEncoderConfiguration();
                            telemetry.update();
                            sleep (4000);
                        }
                        break;
                    default:
                        break;
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

                switch (mode) {
                    case MANUAL:
                        robot.leftFront.setPower(-1 * (leftDriveCmd + leftDriveCrab));
                        robot.rightFront.setPower(-1 * (rightDriveCmd + rightDriveCrab));
                        robot.leftRear.setPower(-1 * (leftRearCmd + leftRearCrab));
                        robot.rightRear.setPower(-1 * (rightRearCmd + rightRearCrab));
                        break;
                    case ENCODER:
                        break;
                    case THRESHOLD:
                        break;
                    case TEST:
                        robotDrive.update();
                        break;
                    case SAVE:
                        break;
                    default:
                        break;
                }


            }


            /* ***************************************************
             *                TELEMETRY
             *       Inputs:  telemetry structure
             *       Outputs: command telemetry output to phone
             ****************************************************/

            if (CurrentTime - LastTelemetry > TELEMETRYPERIOD) {
                LastTelemetry = CurrentTime;

                switch (mode) {
                    case MANUAL:
                        telemetry.addLine("Manual Drive Mode Use sticks");
                        break;
                    case ENCODER:
                        telemetry.addLine("Encoder Calibration Mode -- A to start");
                        break;
                    case TEST:
                        telemetry.addLine("Encoder Test Mode A - FWD   B - REV");
                        telemetry.addLine("                  X - CRL   Y - CRR");
                        telemetry.addLine("2 Second Pause before action");
                        break;
                    case SAVE:
                        telemetry.addLine("Utility Mode A - Save Encoder Configuration");
                        telemetry.addLine("             B - Load Encoder Configuration");
                        telemetry.addLine("             X - Reset Encoder Configuration (all become 1.0)");
                        telemetry.addLine("             Y - Display Current Encoder Configuration");
                        break;
                }
                telemetry.addLine("Dpad Up      -- Encoder Calibration");
                telemetry.addLine("Dpad Down    -- Manual Drive");
                telemetry.addLine("Dpad Left    -- Test Calibration");
                telemetry.addLine("Dpad Right   -- Motor Threshold");
                telemetry.addLine("Both Bumpers -- Utility Mode");

                telemetry.update();
                telemetry.clear();
            }
        }

        //SAFE EXIT OF RUN OPMODE, stop motors, leave servos????
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftRear.setPower(0);
        robot.rightRear.setPower(0);

    }
}
