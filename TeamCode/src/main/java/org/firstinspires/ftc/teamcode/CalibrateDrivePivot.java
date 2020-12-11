package org.firstinspires.ftc.teamcode;
/*
 *  Created by James Rumsey
 *  December 2018
 *
 *  PURPOSE:  In telop() mode allows robot to have its pivot parameters for the Drive() class
 *            updated.  User specified turns can then be commanded.   Use this opMode() to
 *            determine the best pivot calibration parameters for your robot
 */
import android.os.SystemClock;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


@SuppressWarnings("WeakerAccess")
@TeleOp(name = "Calibrate Pivot", group = "K9Bot")
@Disabled
public class CalibrateDrivePivot extends LinearOpMode {
    private Drive robotDrive              = new Drive();

    private HardwareDefenition robot   = new HardwareDefenition();   //Remap to your robot    private Drive robotDrive = new Drive();

    // Timing loop parameters
    final long NAVPERIOD = 200;
    final long MOTORPERIOD = 20;
    final long CONTROLLERPERIOD = 200;
    final long TELEMETRYPERIOD = 1000;

    Gamepad g1 = new Gamepad();
    Gamepad g2 = new Gamepad();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing Hardware");
        telemetry.setAutoClear(false);
        telemetry.update();
        robot.init(hardwareMap, true, false, false);
        telemetry.addData("      ", "SUCCESS");


        //Time keeping, prime the variables
        long CurrentTime    = System.currentTimeMillis();
        long LastNav        = CurrentTime + 15;
        long LastMotor      = CurrentTime + 20;
        long LastController = CurrentTime + 7;
        long LastTelemetry  = CurrentTime + 17;

        //Storage location for the calibration variables
        Drive.PivotTolerance pivotTolerance = Drive.PivotTolerance.ONE_DEGREE;
        double               backoffMultiplier = 15;
        double               turnBackoff       = 0.45;
        double               minTurnPower      = 0.3;
        int                  targetHeading     = 90;
        double               initialPower      = 0.5;
        Drive.MoveType       moveType          = Drive.MoveType.PIVOTRIGHT;

        telemetry.addData("Status", "Initializing Drive");
        /*****************************************************
         *  CONFIGURE THE DRIVE TRAIN  THIS IS ROBOT SPECIFIC
         *****************************************************/
        Drive.Parameters dParm = robotDrive.getParameters();
        dParm.frontRight = robot.rightFront;
        dParm.frontLeft = robot.leftFront;
        dParm.rearRight = robot.rightRear;  //Should be unnecessary, but just keep the nulls away
        dParm.rearLeft = robot.leftRear;
        dParm.driveType = Drive.DriveType.MECANUM;
        dParm.imu = robot.imu;
        dParm.motorRatio = 28;
        dParm.gearRatio = 5.2;
        dParm.wheelDiameter = 4.0;
        dParm.mecanumAngle = 1;
        dParm.pivotTolerance = pivotTolerance;
        dParm.encoderTolerance = 75;
        dParm.turnBackoff = turnBackoff;
        dParm.backoffMultiplier = backoffMultiplier;
        dParm.minStartPower = 0.3;
        dParm.minTurnPower = minTurnPower;
        dParm.opMode = this;
        dParm.debug = false;


        robotDrive.configureDrive(dParm);

        telemetry.update();
        telemetry.setAutoClear(true);

        ElapsedTime runtime = new ElapsedTime();
        waitForStart();
        runtime.reset();


        /**************************************************************
         *            Everything below here press START               *
         **************************************************************/

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            CurrentTime = System.currentTimeMillis();


            /**************************************************
             *                Controller INPUT                  *
             *  INPUTS: raw controller values                   *
             *  OUTPUTS: g1, g2
             ****************************************************/
            if (CurrentTime - LastController > CONTROLLERPERIOD) {
                LastController = CurrentTime;
                g1 = gamepad1;
                g2 = gamepad2;
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

                if ((robotDrive.getMoveStatus() == Drive.MoveStatus.AVAILABLE) ||
                    (robotDrive.getMoveStatus() == Drive.MoveStatus.COMPLETE) ){

                    //PIVOT TOLERANCE  --  FIX
                    if (g1.right_bumper && (pivotTolerance.ordinal() < Drive.PivotTolerance.values().length - 1)) {
                        pivotTolerance = Drive.PivotTolerance.values()[pivotTolerance.ordinal()+1];
                    }
                    if (g1.left_bumper && (pivotTolerance.ordinal() > 0)) {
                        pivotTolerance = Drive.PivotTolerance.values()[pivotTolerance.ordinal()-1];
                    }

                    //TURN BACKOFF --- OK
                    if (g1.dpad_up) {
                        turnBackoff = (turnBackoff+.01 < 1.0) ? turnBackoff+.01 : 1.0;
                    }
                    if (g1.dpad_down) {
                        turnBackoff = (turnBackoff-.01 > 0.0) ? turnBackoff-.01 : 0.0;
                    }

                    //BACKOFF MULTIPLIER --  OK
                    if (g1.dpad_left) {
                        backoffMultiplier = (backoffMultiplier-.5 > 3) ? backoffMultiplier-.5 : 3;
                    }
                    if (g1.dpad_right) {
                        backoffMultiplier = (backoffMultiplier+.5 < 90) ? backoffMultiplier+.5 : 90;
                    }


                    //MIN TURN POWER  -- OK
                    if (g1.left_stick_y < -0.1) {
                        minTurnPower = (minTurnPower+.01<1.0) ? minTurnPower+.01 : 1.0;
                    }
                    if (g1.left_stick_y > 0.1) {
                        minTurnPower = (minTurnPower-.01>.1) ? minTurnPower-.01 : .1;
                    }

                    //TARGET HEADING -- OK
                    if (g1.a) {
                        targetHeading = (targetHeading+1<360) ? targetHeading+1 : 0;
                    }
                    if (g1.b) {
                        targetHeading = (targetHeading-1>0) ? targetHeading-1 : 359;
                    }

                    //INITIAL POWER --OK
                    if (g1.right_stick_y < -0.1) {
                        initialPower = (initialPower+.01<1.0) ? initialPower+.01 : 1.0;
                    }
                    if (g1.right_stick_y > 0.1) {
                        initialPower = (initialPower-.01>.1) ? initialPower-.01 : .1;
                    }

                    //MOVE TYPE -- OK
                    if (g1.y) {
                        if (moveType == Drive.MoveType.PIVOTRIGHT) {
                            moveType = Drive.MoveType.PIVOTLEFT;
                        }
                        else {
                            moveType = Drive.MoveType.PIVOTRIGHT;
                        }
                    }
                }


                if (g1.x) {
                    //Execute Maneuver
                    telemetry.clear();
                    telemetry.addData("Preparing to execute ", moveType);
                    telemetry.addLine("Initializing Drive");
                    telemetry.update();

                    dParm.pivotTolerance = pivotTolerance;
                    dParm.turnBackoff = turnBackoff;
                    dParm.backoffMultiplier = backoffMultiplier;
                    dParm.minTurnPower = minTurnPower;
                    robotDrive.configureDrive(dParm);
                    robotDrive.move(moveType, targetHeading,initialPower);
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

            }


            /* ***************************************************
             *                TELEMETRY
             *       Inputs:  telemetry structure
             *       Outputs: command telemetry output to phone
             ****************************************************/
            if (CurrentTime - LastTelemetry > TELEMETRYPERIOD) {
                LastTelemetry = CurrentTime;
                telemetry.addData("Current Heading          ", robotDrive.getHeading());
                telemetry.addData("Target Heading           ", targetHeading);
                telemetry.addData("Initial Power            ","%.2f", initialPower);
                telemetry.addData("Move Type                ", moveType);
                telemetry.addData("Pivot Tolerance          ", pivotTolerance);
                telemetry.addData("Backoff Power Multiplier  ","%.2f", turnBackoff);
                telemetry.addData("Backoff Window Multiplier ","%.2f", backoffMultiplier);
                telemetry.addData("Minimum       Turn Power  ","%.2f", minTurnPower);
                telemetry.update();
            }
        }

        //Graceful shutdown
        robotDrive.move(Drive.MoveType.STOP, 0, 0);
    }
}


