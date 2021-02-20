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
@TeleOp(name = "Ring Stack Detect", group = "HardwarePushbot")
@Disabled
public class Driver_RingStackDetection extends LinearOpMode {
    private HardwareDef_20_21 robot = new HardwareDef_20_21();
    private Drive robotDrive         = new Drive();
    private GamepadDrive gamepadDrive = new GamepadDrive();
    private RingStackDetection2 RingStackDetection2 = new RingStackDetection2();



    //Defenitions for rate at which each driver task executes.   Time is in milliseconds
    final long SENSORPERIOD = 20;
    final long SERVOPERIOD = 20;
    final long NAVPERIOD = 20;
    final long MOTORPERIOD = 20;
    final long CONTROLLERPERIOD = 20;
    final long TELEMETRYPERIOD = 500;

    private static double MIN_EXTENSION = -1.125;
    private static double MAX_EXTENSION = 1.825;
    private double extensionCmd = 0.0;





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
        gamepadDrive.setZeroAxis(GamepadDrive.AXES.X_POS);
        telemetry.addData("      ", retVal);
        telemetry.update();
        RingStackDetection2.Rings2 numRingsTest = org.firstinspires.ftc.teamcode.RingStackDetection2.Rings2.NONE;



        // Initialize some of the timing variables
        long CurrentTime = System.currentTimeMillis();
        long LastSensor = CurrentTime;
        long LastEncoderRead = CurrentTime + 5;
        long LastServo = CurrentTime + 10;
        long LastNav = CurrentTime + 15;
        long LastMotor = CurrentTime + 20;
        long LastController = CurrentTime + 7;
        long LastTelemetry = CurrentTime + 17;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        ElapsedTime runtime = new ElapsedTime();




        RingStackDetection2.configureDetection(this);

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
                RingStackDetection2.detectRingStack(100000);

                if (gamepad1.x) {
                    extensionCmd += 0.01;
                }
                if (gamepad1.y) {
                    extensionCmd -= 0.01;
                }

                extensionCmd = (extensionCmd > MAX_EXTENSION ? MAX_EXTENSION : extensionCmd);
                extensionCmd = (extensionCmd < MIN_EXTENSION ? MIN_EXTENSION : extensionCmd);

            }



            /* **************************************************
             *                SERVO OUTPUT
             *                Inputs: leftClamp position command
             *                        rightClamp position command *
             *                Outputs: Physical write to servo interface.
             ****************************************************/
            if (CurrentTime - LastServo > SERVOPERIOD) {
                LastServo = CurrentTime;
                //robot.extension.setPower(extensionCmd);
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

            }


            /* ***************************************************
             *                TELEMETRY
             *       Inputs:  telemetry structure
             *       Outputs: command telemetry output to phone
             ****************************************************/

            if (CurrentTime - LastTelemetry > TELEMETRYPERIOD) {
                LastTelemetry = CurrentTime;
                telemetry.clear();
                telemetry.addData("Extension Cmd ", extensionCmd);
                telemetry.addData("NumRingsTest",RingStackDetection2.retRings);
                telemetry.update();
            }
        }


    }

}


