
/*
Built for Juden Ki 8578 and Kernel Panic 11959

James K Rumsey
03/13/2019

OVERVIEW:
Provides methods to autonomously and manually control two different type sof linear actuators or
range limited motor control.  The initialization method is overloaded to support the different types
of linear actuators.

PULLEY_LINEAR_SLIDE -- Used for something like a cascading rail system that is moved by winding a
                       string on a pulley.   Can be used with drawer slides, etc...   Basically
                       anything controlled through a pulley.
LEAD_SCREW          -- Used to control any linear movement involving a lead screw and nut linear
                       actuator.   Take care to not specify a max extension greater than the
                       system can handle.  NOTE:  the max extension is not the length of the
                       threaded rod it is the length the nut can travel.
MOTOR_ONLY          -- Not really a linear actuator thing but provides an interface to provide
                       range limited control of a motor.   Useful for controlling things like
                       a motor used to swing a pivot arm, etc...


As with most support classes the length parameters are unitless.   They just all need to use the
same units.  Automatic control of the actuator is accomplished using the encoders on the motor.
When selecting a position the full range of the actuator will be represented as a percentage.
0.0 for fully retracted and 1.0 for fully extended.


REQUIRED:
Encoders -- Encoder cables to drive motor for actuator
Motors   -- Encoder capable motor geared to actuator

ASSUMES:
Actuator is fully retracted to start.
Actuator is capable of self retracting when the pulley cable is let out.  (Surgical tubing/spring)
                        PULLEY_LINEAR_SLIDE only.


INITIALIZATION PARAMETERS:
    DcMotor        motor          -- Motor for the actuator
    double         motorRatio     -- This is the number of counts for a single rotation of the motor
                                     shaft.
    double         gearRatio      -- The overall gear ratio, includes gearbox and everything else.
    ACTUATOR_TYPE  type           -- Specify which kind of linear actuator is being used, determines
                                     some of the math calculations.
    double         initialPosition -- Starting position for the actuator.   Usually 0.0, can be
                                     changed to a value up to 1.0 (100%)  Will be used to calculate
                                     an offset to the integer target provided to the motor encoder
                                     control.   Useful for dealing with a actuator not in its
                                     default state at teh end of autonomous.
******  MOTOR_ONLY  ONLY *****
    double          maxOutputShaftRotations -- the maximum number of desired rotations.
******  END MOTOR_ONLY  ONLY *****
*
******  LEAD_SCREW  ONLY *****
    double         leadScrewLength -- Length of the lead screw.  (The threaded rod).   This should
                                     be the functional length not the total length.   The functional
                                     length is the distance that the lead screw nut can travel. The
                                     length can be in any units it just needs to match the units
                                     used for pitch.
    double         leadScrewPitch  -- This should be threads per unit.
    double         leadScrewNutStarts -- The number of revolutions of nut per revolution of shaft.
******  END LEAD_SCREW  ONLY *****

******  PULLEY_LINEAR_SLIDE  ONLY *****
    double         gearRatio      -- This is the gear ration of the output shaft for the motor
    double         maxExtension   -- The maximum linear extension in whatever units are chosen.
                                     Make sure to sum all extensions if it is a cascading system.
    double         wheelDiameter  -- The diameter of hte wheel used to wind and unwind the pulley
                                      cable.   Be sure to use the same units as maxEtension
******  END PULLEY_LINEAR_SLIDE  ONLY *****


    LinearOpMode   opMode          -- The linear opMode to allow diagnostic messages.
    boolean        debug           -- Set to true to turn on diagnostic messages



OTHER USEFUL PUBLIC METHODS:
        delta -- Changes the allowed minimum and maximum delta allowed for manual movement.  Takes
                 inputs from 0.0 to 1.0 as a percentage of change to allow.
        motorPower -- Changes the initial power supplied to the motor when actuating.  Can be useful
                   to smooth out movement.


STILL TO DO:
Figure out a way to pass actuator state between autonomous and teleop.  Maybe a manual reset??  Or
perhaps add an encoder offset when initializing.   Either pass it through a save file or have it
manually passed as a ratio of deployment.   EX: Finishes at 50% deployed

FIX over-retract, causes it to be unable to extend.  Counts too low and extend does not get past
zero.


*/
package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


public class LinearActuator {
    public enum MOVETYPE {AUTOMATIC, MANUAL_EXTEND, MANUAL_RETRACT, MANUAL_STOP};
    public enum ACTUATOR_TYPE {MOTOR_ONLY, PULLEY_LINEAR_SLIDE, LEAD_SCREW};
    public enum LINEAR_ACTUATOR_STATUS {OK, ERROR, INVALID_ACTUATOR_TYPE};
    private MOVETYPE myMove = MOVETYPE.MANUAL_STOP;
    private DcMotor motor;
    private ACTUATOR_TYPE type;
    private double motorRatio;
    private double gearRatio;
    private double  maxExtension;
    private double  wheelDiameter;
    private double leadScrewLength;
    private double leadScrewPitch;
    private double leadScrewNutStart;
    private double maxOutputShaftRotations;
    private boolean init = false;
    private boolean debug;
    private LinearOpMode opMode;
    private int target   = 0;
    private int encoderPosition = 0;

    private static double MAX_PERCENTAGE_DELTA  = 0.2;
    private static double MIN_PERCENTAGE_DELTA  = 0.005;
    private double maxDelta = MAX_PERCENTAGE_DELTA;
    private double minDelta = MIN_PERCENTAGE_DELTA;

    private static double MAX_MOTOR_POWER   = 1.0;
    private static double MIN_MOTOR_POWER   = 0.2;
    private double motPower = MAX_MOTOR_POWER;
    private double targetPosition =0.0;

    private int    encoderOffset = 0;
    private double initialPosition = 0.0;
    private int    MAX_ENCODER_COUNT = 0;
    private int    MIN_ENCODER_COUNT = 0;
    private static double MAX_POSITION = 1.0;
    private static double MIN_POSITION = 0.0;








    // MOTOR_ONLY
    // Have to assume folks knew what they were doing sending the max shaft rotation.
    //
    // LEAD_SCREW
    // The total revolutions that can be traveled on the lead screw is the length of the screw
    // divided by the pitch of the threads on the screw (or nut pitch).
    // The number of lead screw revolutions per revolution of the shaft is equal to the starts
    // on the lead screw nut.
    //
    // PULLEY_LINEAR_SLIDE
    // The total revolutions that are required could be calculated by knowing the diameter of the
    // pulley wheel and measuring the maximum total amount of linear travel.   This works well on
    // systems with larger pulley wheels.  It can become problematic to accurately calculate the
    // total revolutions for small pulley wheels.   This is due to the pulley cable wrapping back on
    // itself causing a change in the wheel diameter.
    private double totalRevolutions () {

        double retVal = 0.0;
        switch (type) {
            case MOTOR_ONLY:
                retVal = maxOutputShaftRotations;
                break;
            case LEAD_SCREW:
                retVal = leadScrewLength / leadScrewPitch / leadScrewNutStart;
                break;
            case PULLEY_LINEAR_SLIDE:
                retVal = maxExtension / (Math.PI * wheelDiameter);
                break;
            default:
                break;
        }
        return (retVal);
    }

    private int encoderCountsPerRevolution () {  return (int)(motorRatio * gearRatio);    }
    private int maxEncoderCounts () { return ( (int)  (totalRevolutions() * encoderCountsPerRevolution())  );  }

    public void delta (double minDelta, double maxDelta) {
        if (minDelta < maxDelta) {
            this.minDelta = (minDelta > MIN_PERCENTAGE_DELTA) ? minDelta : MIN_PERCENTAGE_DELTA;
            this.maxDelta = (maxDelta > MAX_PERCENTAGE_DELTA) ? maxDelta : MAX_PERCENTAGE_DELTA;
        }
    }

    public void motorPower (double power) {
        motPower = power;
        motPower = (motPower > MAX_MOTOR_POWER) ? MAX_MOTOR_POWER : motPower;
        motPower = (motPower < MIN_MOTOR_POWER) ? MIN_MOTOR_POWER : motPower;
    }

    public boolean isDebug() {
        return debug;
    }
    public void setDebug(boolean debug) {
        this.debug = debug;
    }

    public void setInitialPosition(double initialPosition) {
        initialPosition = (initialPosition > MAX_POSITION) ? MAX_POSITION : initialPosition;
        initialPosition = (initialPosition < MIN_POSITION) ? MIN_POSITION : initialPosition;
        this.initialPosition = initialPosition;
        MAX_ENCODER_COUNT = (int)((MAX_POSITION-initialPosition) *  maxEncoderCounts());
        MIN_ENCODER_COUNT = (int)((MIN_POSITION-initialPosition) *  maxEncoderCounts());
        encoderOffset = (int)(initialPosition * maxEncoderCounts());
    }

    //////////////////////////////////
    // MOTOR_ONLY
    //////////////////////////////////
    public LINEAR_ACTUATOR_STATUS initialize (DcMotor motor,
                                               ACTUATOR_TYPE type,
                                               double motorRatio,
                                               double gearRatio,
                                               double maxOutputShaftRotations,
                                               LinearOpMode opMode,
                                               boolean debug) {
        LINEAR_ACTUATOR_STATUS status = LINEAR_ACTUATOR_STATUS.ERROR;

        if (type != ACTUATOR_TYPE.MOTOR_ONLY) {
            status = LINEAR_ACTUATOR_STATUS.INVALID_ACTUATOR_TYPE;
        } else {
            if (opMode != null) {
                this.debug = debug;
                this.motor = motor;
                this.type = type;
                this.motorRatio = motorRatio;
                this.gearRatio = gearRatio;
                this.maxOutputShaftRotations = maxOutputShaftRotations;
                this.opMode = opMode;
                status = LINEAR_ACTUATOR_STATUS.OK;
            }
            init2();
        }


        initPrint(status);
        return   (status);
    }

    //////////////////////////////////
    // LEAD_SCREW
    //////////////////////////////////
    public LINEAR_ACTUATOR_STATUS initialize (DcMotor motor,
                                              ACTUATOR_TYPE type,
                                              double motorRatio,
                                               double gearRatio,
                                               double leadScrewLength,
                                               double leadScrewPitch,
                                               double leadScrewNutStart,
                                               LinearOpMode opMode,
                                               boolean debug) {
        LINEAR_ACTUATOR_STATUS status = LINEAR_ACTUATOR_STATUS.ERROR;

        if (type != ACTUATOR_TYPE.LEAD_SCREW) {
            status = LINEAR_ACTUATOR_STATUS.INVALID_ACTUATOR_TYPE;
        }
        else {
            if (opMode != null) {
                this.debug = debug;
                this.motor = motor;
                this.type = type;
                this.motorRatio = motorRatio;
                this.gearRatio = gearRatio;
                this.leadScrewNutStart = leadScrewNutStart;
                this.leadScrewPitch = leadScrewPitch;
                this.leadScrewLength = leadScrewLength;
                this.opMode = opMode;
                status = LINEAR_ACTUATOR_STATUS.OK;
            }
            init2();
        }
        initPrint(status);
        return   (status);
    }

    //////////////////////////////////
    // PULLEY_LINEAR_SLIDE
    //////////////////////////////////
    public LINEAR_ACTUATOR_STATUS initialize (DcMotor motor,
                                               ACTUATOR_TYPE type,
                                               double motorRatio,
                                               double gearRatio,
                                               double maxExtension,
                                               double wheelDiameter,
                                               LinearOpMode opMode,
                                               boolean debug) {
        LINEAR_ACTUATOR_STATUS status = LINEAR_ACTUATOR_STATUS.ERROR;

        if (type != ACTUATOR_TYPE.PULLEY_LINEAR_SLIDE) {
            status = LINEAR_ACTUATOR_STATUS.INVALID_ACTUATOR_TYPE;
        }
        else {
            if (opMode != null) {
                this.debug = debug;
                this.motor = motor;
                this.type = type;
                this.motorRatio = motorRatio;
                this.gearRatio = gearRatio;
                this.maxExtension = maxExtension;
                this.wheelDiameter = wheelDiameter;
                this.opMode = opMode;
                status = LINEAR_ACTUATOR_STATUS.OK;
            }
            init2();
        }
        initPrint(status);
        return   (status);
    }

    private void init2() {
        target = 0;
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        encoderPosition = motor.getCurrentPosition();
        MAX_ENCODER_COUNT = maxEncoderCounts();
        MIN_ENCODER_COUNT =  0;
    }
    private void initPrint(LINEAR_ACTUATOR_STATUS status) {
        if (debug)  {
            opMode.telemetry.addData("Initialization Status ", status);
            opMode.telemetry.addData("Overall Motor Ratio ", encoderCountsPerRevolution());
            opMode.telemetry.addData("Maximum # Revolutions ", totalRevolutions());
            opMode.telemetry.addData("Maximum # Encoder Counts ", maxEncoderCounts());
            opMode.telemetry.update();
        }
    }


      public void move (double movePosition, MOVETYPE movetype) {
        double   finalPosition=0.0;
        myMove = movetype;

        switch (movetype) {
            case AUTOMATIC:
                movePosition = (movePosition > MAX_POSITION) ? MAX_POSITION : movePosition;
                movePosition = (movePosition < MIN_POSITION) ? MIN_POSITION : movePosition;
                target = ((int) (movePosition * maxEncoderCounts()));
                break;
            case MANUAL_RETRACT:
                movePosition = (movePosition > maxDelta)   ? maxDelta   : movePosition;
                movePosition = (movePosition < minDelta) ? minDelta : movePosition;
                finalPosition = ((getCurrentPosition() - movePosition) > MAX_POSITION) ? MAX_POSITION : (getCurrentPosition() - movePosition);
                target = (int) ((finalPosition * maxEncoderCounts()) - encoderOffset);
                break;
            case MANUAL_EXTEND:
                movePosition = (movePosition > maxDelta)   ? maxDelta   : movePosition;
                movePosition = (movePosition < minDelta) ? minDelta : movePosition;
                finalPosition = ((getCurrentPosition() + movePosition) < MIN_POSITION) ? MIN_POSITION : (getCurrentPosition() + movePosition);
                target = (int) ((finalPosition * maxEncoderCounts()) - encoderOffset);
                break;
            case MANUAL_STOP:
            default:
                target = (int) (getCurrentPosition() * maxEncoderCounts());
                break;
        }

        targetPosition = movePosition;
        target = (target < MIN_ENCODER_COUNT) ? MIN_ENCODER_COUNT : target;
        target = (target > MAX_ENCODER_COUNT) ? MAX_ENCODER_COUNT : target;

        if (debug) {
            opMode.telemetry.addData("current", getCurrentPosition());
            opMode.telemetry.addData("final", finalPosition);
            opMode.telemetry.addData("encoder current", motor.getCurrentPosition());
            opMode.telemetry.addData("encoder target  ", target);
            opMode.telemetry.addData("MAX_ENCODER_COUNTS", MAX_ENCODER_COUNT);
            opMode.telemetry.addData("MIN_ENCODER_COUNTS", MIN_ENCODER_COUNT);
            opMode.telemetry.update();
        }

        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setTargetPosition(target);

    }


    public double getCurrentPosition() {
        double myPosition = (double)(encoderPosition+encoderOffset)/(double)maxEncoderCounts();
        return myPosition;
    }

    public double getTargetPosition() {
        return targetPosition;
    }


    public void update () {
        double pow = motPower;
        encoderPosition = motor.getCurrentPosition();

        switch (myMove) {
            case AUTOMATIC:
            case MANUAL_EXTEND:
            case MANUAL_RETRACT:
                //Provide protection for over and under shoot
                if (encoderPosition > MAX_ENCODER_COUNT) {
                    motor.setTargetPosition(MAX_ENCODER_COUNT);
                }
                if (encoderPosition < MIN_ENCODER_COUNT) {
                    motor.setTargetPosition(MIN_ENCODER_COUNT);
                }
                break;
            case MANUAL_STOP:
            default:
                pow=0.0;
                break;
        }
        motor.setPower(pow);
    }
}
