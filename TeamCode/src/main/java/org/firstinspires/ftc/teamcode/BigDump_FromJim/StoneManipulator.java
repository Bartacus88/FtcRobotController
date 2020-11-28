/*
 * Created By:  James K Rumsey
 *
 * Date:  10/5/19
 *
 * Purpose:  This class will provide all of the methods to control the stone/skystone manipulator
 *      for the 2019/2020 FTC SkyStone game.    The manipulator is hardware specific to the example
 *      robot that James Rumsey built.   Neither this code nor the robot are intended to be used by
 *      students.  It is allowed for the students to reference this code to determine how to write
 *      their own version.
 *
 * Hardware Requirements:
 *
 * liftMotor        --  An encoder controlled motor hooked up to tho pulleys.   This will be used
 *                      to raise and lower the lift.
 * deployMotor      --  An encoder controlled motor hooked up to the upper 6mm lead screw.  This
 *                      will be used to extend the stone manipulation assembly.
 * positionMotor    -- An encoder controlled motor hooked up to the lower 6mm lead screw.  This
 *                      will be used to position the stone in the horizontal plane when building
 *                      the tower.
 * pivotServo       -- A standard servo that will be used to rotate the block by 90 degrees
 *                      typically.
 * clampServo       -- A sail winch servo configured to behave as a standard servo.  Used to open
 *                      and close the stone clamp.
 * clampColor1
 * clampColor2      -- Two color sensors tha tare part of the clamp assembly.  They are used to
 *                      determine if a stone is available to be clamped.  They are also used to
 *                      discriminate between a stone and a skystone.  Will also be used as distance
 *                      sensors.  Implies must be REV v2 sensor.
 * stoneDistance    -- An ultrasonic distance sensor used to determine where the stone is relative
 *                      to the clamp.   The is value is used to determine the distance the position
 *                      motor needs to travel for stone placement.
 */

package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static org.firstinspires.ftc.teamcode.LinearActuator.ACTUATOR_TYPE.LEAD_SCREW;
import static org.firstinspires.ftc.teamcode.LinearActuator.ACTUATOR_TYPE.PULLEY_LINEAR_SLIDE;

public class StoneManipulator {

    public enum StoneType  {STONE, SKYSTONE, NONE, UNKNOWN};
    public enum ManipulatorState  {INITIAL, DEPLOY, TRANSPORT_LOW, TRANSPORT_HIGH, PRE_CLAMP, CLAMP,
                                    PLACEMENT, CAPSTONE, TRANSPORT_PLATFORM, INITIAL_PLATFORM,
                                    AUTONOMOUS_PRE_CLAMP, UNKNOWN};
    public enum StoneManipulatorStatus {OK, ERROR, UNKNOWN};

    /*
     * Location enumerations for stone placement.   Each enumeration not only describes the
     * location that the stone will be placed but will be used as an index into the array of
     * actuator positions and values to execute that placement.
     */
    public enum StonePlacementLocation {LEVEL1_FAR, LEVEL1_NEAR, LEVEL2_RIGHT, LEVEL2_LEFT,
                                        LEVEL3_FAR, LEVEL3_NEAR, LEVEL4_RIGHT, LEVEL4_LEFT,
                                        LEVEL5_FAR, LEVEL5_NEAR, LEVEL6_RIGHT, LEVEL6_LEFT,
                                        LEVEL7_FAR, LEVEL7_NEAR, LEVEL8_RIGHT, LEVEL8_LEFT,
                                        LEVEL9_FAR, LEVEL9_NEAR, LEVEL10_RIGHT, LEVEL10_LEFT,
                                        NOMORE};






    private class StonePlacement {
        double liftInitial;
        double liftFinal;
        double deployInitial;
        double deployFinal;
        double positionInitial;
        double positionFinal;
        double pivotInitial;
        double pivotFinal;
        double clampInitial;
        double clampFinal;

        private StonePlacement (double liftInitial, double liftFinal, double deployInitial, double deployFinal,
                                double positionInitial, double positionFinal, double pivotInitial, double pivotFinal,
                                double clampInitial, double clampFinal) {
            this.liftInitial = liftInitial;
            this.liftFinal   = liftFinal;
            this.deployInitial = deployInitial;
            this.deployFinal   = deployFinal;
            this.positionInitial = positionInitial;
            this.positionFinal   = positionFinal;
            this.pivotInitial = pivotInitial;
            this.pivotFinal   = pivotFinal;
            this.clampInitial = clampInitial;
            this.clampFinal   = clampFinal;
        }
    }



    private StonePlacement stonePlacement [] = {
            new StonePlacement(0.15, 0.10, 1.0, 1.0, 0.8, 0.8, 0.0, 0.0, 0.0, 0.0),
            new StonePlacement(0.15, 0.10, 1.0, 1.0, 0.8, 0.8, 0.0, 0.0, 0.0, 0.0),
            new StonePlacement(0.15, 0.10, 1.0, 1.0, 0.8, 0.8, 0.0, 0.0, 0.0, 0.0),
            new StonePlacement(0.15, 0.10, 1.0, 1.0, 0.8, 0.8, 0.0, 0.0, 0.0, 0.0),
            new StonePlacement(0.15, 0.10, 1.0, 1.0, 0.8, 0.8, 0.0, 0.0, 0.0, 0.0),
            new StonePlacement(0.15, 0.10, 1.0, 1.0, 0.8, 0.8, 0.0, 0.0, 0.0, 0.0),
            new StonePlacement(0.15, 0.10, 1.0, 1.0, 0.8, 0.8, 0.0, 0.0, 0.0, 0.0),
            new StonePlacement(0.15, 0.10, 1.0, 1.0, 0.8, 0.8, 0.0, 0.0, 0.0, 0.0),
            new StonePlacement(0.15, 0.10, 1.0, 1.0, 0.8, 0.8, 0.0, 0.0, 0.0, 0.0),
            new StonePlacement(0.15, 0.10, 1.0, 1.0, 0.8, 0.8, 0.0, 0.0, 0.0, 0.0),
            new StonePlacement(0.15, 0.10, 1.0, 1.0, 0.8, 0.8, 0.0, 0.0, 0.0, 0.0),
            new StonePlacement(0.15, 0.10, 1.0, 1.0, 0.8, 0.8, 0.0, 0.0, 0.0, 0.0),
            new StonePlacement(0.15, 0.10, 1.0, 1.0, 0.8, 0.8, 0.0, 0.0, 0.0, 0.0),
            new StonePlacement(0.15, 0.10, 1.0, 1.0, 0.8, 0.8, 0.0, 0.0, 0.0, 0.0),
            new StonePlacement(0.15, 0.10, 1.0, 1.0, 0.8, 0.8, 0.0, 0.0, 0.0, 0.0),
            new StonePlacement(0.15, 0.10, 1.0, 1.0, 0.8, 0.8, 0.0, 0.0, 0.0, 0.0),
            new StonePlacement(0.15, 0.10, 1.0, 1.0, 0.8, 0.8, 0.0, 0.0, 0.0, 0.0),
            new StonePlacement(0.15, 0.10, 1.0, 1.0, 0.8, 0.8, 0.0, 0.0, 0.0, 0.0),
            new StonePlacement(0.15, 0.10, 1.0, 1.0, 0.8, 0.8, 0.0, 0.0, 0.0, 0.0),
            new StonePlacement(0.15, 0.10, 1.0, 1.0, 0.8, 0.8, 0.0, 0.0, 0.0, 0.0)
    };

    private LinearActuator lift;
    private LinearActuator deploy;
    private LinearActuator position;
    private Servo   pivotServo;
    private Servo   clampServo;
    private ColorSensor colorLeft;
    private ColorSensor colorRight;
    private DistanceSensor distanceLeft;
    private DistanceSensor distanceRight;
    private DistanceSensor stoneDistance;
    private LinearOpMode myOpMode;

    private StonePlacementLocation  targetLocation;
    private StonePlacementLocation  currentLocation;
    private StonePlacementLocation  previousLocation;
    private StonePlacementLocation  nextLocation;
    private StonePlacementLocation  capStoneLocation;
    private double positionCmpensation = 0.0;

    private static int GOLD_RED_LOWER   =        50;
    private static int GOLD_RED_UPPER   =        5000;
    private static int GOLD_GREEN_LOWER =        50;
    private static int GOLD_GREEN_UPPER =        5000;
    private static int GOLD_BLUE_LOWER  =        50;
    private static int GOLD_BLUE_UPPER  =        5000;

    private static int BLACK_RED_LOWER   =        50;
    private static int BLACK_RED_UPPER   =        5000;
    private static int BLACK_GREEN_LOWER =        50;
    private static int BLACK_GREEN_UPPER =        5000;
    private static int BLACK_BLUE_LOWER  =        50;
    private static int BLACK_BLUE_UPPER  =        5000;

    private static double DEPLOY_EXTEND    = 1.0;
    private static double DEPLOY_RETRACT   = 0.0;
    private static double DEPLOY_CLEARANCE = 0.5;

    private static double LIFT_INITIAL    = 0.0;
    private static double LIFT_DEPLOY     = 0.1;
    private static double LIFT_TRANS_LOW  = 0.0;
    private static double LIFT_TRANS_HIGH = 0.1;
    private static double LIFT_CLAMP      = 0.05;
    private static double LIFT_PRE_CLAMP  = 0.05;
    private static double LIFT_AUTO_PRE_CLAMP = 0.2;

    private static double POS_INITIAL    = 0.0;
    private static double POS_PICK       = 0.3;
    private static double POS_PLACE_FAR  = 0.8;
    private static double POS_PLACE_NEAR = 0.2;
    private static double POS_PLACE_ROTATE = 0.5;

    private static double FACE_POSITION  = 0.5;
    private static double EDGE_POSITION  = 0.0;

    private static double CLAMP_OPEN     = 1.0;
    private static double CLAMP_CLOSE    = 0.2;
    private static double CLAMP_INITIAL  = 0.0;

    public  enum    PlacementStatus    {AVAILABLE, INITIAL, FINAL, UNCLAMP, POST};
    private PlacementStatus placementStatus = PlacementStatus.AVAILABLE;
    private boolean placementSemaphore = false;

    public PlacementStatus getPlacementStatus() {
        return placementStatus;
    }
    public void setPlacementStatus (PlacementStatus p) {
        placementStatus = p;
    }
    public void placementStart() {
        placementSemaphore = true;
    }
    public void placementStop() {
        placementSemaphore = false;
    }
    private long    placementTime    = 0;

    public StoneManipulatorStatus init (LinearOpMode opMode, DcMotor liftMotor, DcMotor deployMotor,
                                        DcMotor positionMotor, Servo pivotServo, Servo clampServo,
                                        ColorSensor colorLeft, ColorSensor colorRight,
                                        DistanceSensor distanceLeft, DistanceSensor distanceRight,
                                        DistanceSensor stoneDistance) {

        StoneManipulatorStatus status = StoneManipulatorStatus.OK;
        LinearActuator.LINEAR_ACTUATOR_STATUS linStatus = LinearActuator.LINEAR_ACTUATOR_STATUS.ERROR;

        /* TODO  modify the tests so instead of null check that it tries to access a method for
         *  each device type.  Through in a trap catch if it fails so it does not crash but can
         *  inform.
         */

        if (opMode == null) {
            status = StoneManipulatorStatus.ERROR;
        }
        myOpMode = opMode;

        if (lift.initialize(liftMotor, PULLEY_LINEAR_SLIDE, 28,60,42,2, opMode,true)
                != LinearActuator.LINEAR_ACTUATOR_STATUS.OK) {
            status = StoneManipulatorStatus.ERROR;
        }
        if (deploy.initialize(deployMotor, LEAD_SCREW, 28,60,42,8, 4, opMode,true)
                != LinearActuator.LINEAR_ACTUATOR_STATUS.OK) {
            status = StoneManipulatorStatus.ERROR;
        }
        if (position.initialize(positionMotor, LEAD_SCREW, 28,60,42,8, 4,opMode,true)
                != LinearActuator.LINEAR_ACTUATOR_STATUS.OK) {
            status = StoneManipulatorStatus.ERROR;
        }



        if (pivotServo == null) {
            opMode.telemetry.addData("StoneManipulator - pivotServo invalid: ", pivotServo);
            status = StoneManipulatorStatus.ERROR;
        }
        else {
            this.pivotServo = pivotServo;
        }
        if (clampServo == null) {
            opMode.telemetry.addData("StoneManipulator - clampServo invalid: ", clampServo);
            status = StoneManipulatorStatus.ERROR;
        }
        else {
            this.pivotServo = pivotServo;
        }

        if (colorLeft == null) {
            opMode.telemetry.addData("StoneManipulator - colorLeft invalid: ", colorLeft);
            status = StoneManipulatorStatus.ERROR;
        }
        else {
            this.colorLeft = colorLeft;
        }
        this.colorRight = colorRight;
        this.distanceLeft = distanceLeft;
        this.distanceRight = distanceRight;
        this.stoneDistance = stoneDistance;

        return status;
    }

    public StoneManipulatorStatus action (ManipulatorState target) {
        StoneManipulatorStatus status = StoneManipulatorStatus.ERROR;

        switch (target) {
            case INITIAL:
                {
                    lift.move(LIFT_INITIAL, LinearActuator.MOVETYPE.AUTOMATIC);
                    deploy.move(DEPLOY_RETRACT, LinearActuator.MOVETYPE.AUTOMATIC);
                    position.move(POS_INITIAL, LinearActuator.MOVETYPE.AUTOMATIC);
                    edgePivot();
                    clampServo.setPosition(CLAMP_INITIAL);
                }
                break;
            case DEPLOY:
                {
                    lift.move(LIFT_DEPLOY, LinearActuator.MOVETYPE.AUTOMATIC);
                    deploy.move(DEPLOY_EXTEND, LinearActuator.MOVETYPE.AUTOMATIC);
                    if (deploy.getCurrentPosition() > DEPLOY_CLEARANCE) {
                        facePivot();
                    }
                }
                break;
            case TRANSPORT_LOW:
                {
                    lift.move(LIFT_TRANS_LOW, LinearActuator.MOVETYPE.AUTOMATIC);
                }
                break;
            case TRANSPORT_HIGH:
                {
                    lift.move(LIFT_TRANS_HIGH, LinearActuator.MOVETYPE.AUTOMATIC);
                }
                break;
            case PRE_CLAMP:
                {
                    lift.move(LIFT_PRE_CLAMP, LinearActuator.MOVETYPE.AUTOMATIC);
                    position.move(POS_PICK, LinearActuator.MOVETYPE.AUTOMATIC);
                    edgePivot();
                    clampOpen();
                }
                break;
            case AUTONOMOUS_PRE_CLAMP:
                {
                    lift.move(LIFT_AUTO_PRE_CLAMP, LinearActuator.MOVETYPE.AUTOMATIC);
                    position.move(POS_PICK, LinearActuator.MOVETYPE.AUTOMATIC);
                    facePivot();
                    clampOpen();
                }
                break;
            case CLAMP:
                {
                  /* Drop to clamping position, once there close clamps */
                  lift.move(LIFT_CLAMP, LinearActuator.MOVETYPE.AUTOMATIC);
                  if (atTarget(lift,LIFT_CLAMP,.05)) {
                      clampClose();
                  }
                }
                break;
            case PLACEMENT:
                /* perform all of the initial position movements first. Then perform the placement
                 * movements.   Then perform only the unclamp movement.   Return the lift to the
                 * initial height
                 */
                if ((placementStatus == PlacementStatus.AVAILABLE) && (placementSemaphore)) {
                    placementStatus = PlacementStatus.INITIAL;
                    positionCmpensation = distanceCompensation();
                    placementTime = SystemClock.currentThreadTimeMillis();
                    lift.move(stonePlacement[targetLocation.ordinal()].liftInitial, LinearActuator.MOVETYPE.AUTOMATIC);
                    deploy.move(stonePlacement[targetLocation.ordinal()].deployInitial, LinearActuator.MOVETYPE.AUTOMATIC);
                    position.move(stonePlacement[targetLocation.ordinal()].positionInitial, LinearActuator.MOVETYPE.AUTOMATIC);
                    pivotServo.setPosition(stonePlacement[targetLocation.ordinal()].pivotInitial);
                    clampServo.setPosition(stonePlacement[targetLocation.ordinal()].clampInitial);

                }

                if (placementStatus == PlacementStatus.INITIAL) {
                    if ((atTarget(lift, stonePlacement[targetLocation.ordinal()].liftInitial, 0.05)) &&
                        (atTarget(deploy, stonePlacement[targetLocation.ordinal()].deployInitial, 0.05)) &&
                        (atTarget(position, stonePlacement[targetLocation.ordinal()].positionInitial, 0.05)) &&
                        (atTargetservo(pivotServo, stonePlacement[targetLocation.ordinal()].pivotInitial, 0.05)) &&
                        (atTargetservo(clampServo, stonePlacement[targetLocation.ordinal()].clampInitial, 0.05)) ) {
                        lift.move(stonePlacement[targetLocation.ordinal()].liftFinal, LinearActuator.MOVETYPE.AUTOMATIC);
                        deploy.move(stonePlacement[targetLocation.ordinal()].deployFinal, LinearActuator.MOVETYPE.AUTOMATIC);
                        position.move(stonePlacement[targetLocation.ordinal()].positionFinal+positionCmpensation, LinearActuator.MOVETYPE.AUTOMATIC);
                        pivotServo.setPosition(stonePlacement[targetLocation.ordinal()].pivotFinal);
                        placementStatus = PlacementStatus.FINAL;
                    }
                }

                if (placementStatus == PlacementStatus.FINAL) {
                    if ((atTarget(lift, stonePlacement[targetLocation.ordinal()].liftFinal, 0.05)) &&
                        (atTarget(deploy, stonePlacement[targetLocation.ordinal()].deployFinal, 0.05)) &&
                        (atTarget(position, stonePlacement[targetLocation.ordinal()].positionFinal+positionCmpensation, 0.05)) &&
                        (atTargetservo(pivotServo, stonePlacement[targetLocation.ordinal()].pivotFinal, 0.05)) &&
                        (atTargetservo(clampServo, stonePlacement[targetLocation.ordinal()].clampInitial, 0.05)) ) {
                        clampServo.setPosition(stonePlacement[targetLocation.ordinal()].clampFinal);
                        placementStatus = PlacementStatus.UNCLAMP;
                    }
                }

                if (placementStatus == PlacementStatus.UNCLAMP) {
                    if (atTargetservo(clampServo, stonePlacement[targetLocation.ordinal()].clampInitial, 0.05)) {
                        lift.move(stonePlacement[targetLocation.ordinal()].liftInitial, LinearActuator.MOVETYPE.AUTOMATIC);
                        placementStatus = PlacementStatus.POST;
                    }
                }

                if (placementStatus == PlacementStatus.POST) {
                    if ((atTarget(lift, stonePlacement[targetLocation.ordinal()].liftInitial, 0.05))) {
                        placementStatus = PlacementStatus.AVAILABLE;
                        previousLocation = targetLocation;
                        nextLocation = nextStoneLocation(targetLocation);
                    }
                }
                break;
            case TRANSPORT_PLATFORM:
                /*
                 * Interesting case.  Want to go to the height of the last placed block and orient
                 * the manipulator so it can hold the block while the platform is pulled.   This
                 * should stabilize the tower for transport.   May need to have a couple of scenarios
                 * depending on the if the top is single or two stones.  Also depends on capstone.
                 */
                if ((placementStatus == PlacementStatus.AVAILABLE) && (placementSemaphore)) {
                    placementStatus = PlacementStatus.INITIAL;
                    placementTime = SystemClock.currentThreadTimeMillis();
                    lift.move(stonePlacement[previousLocation.ordinal()].liftInitial, LinearActuator.MOVETYPE.AUTOMATIC);
                    deploy.move(stonePlacement[previousLocation.ordinal()].deployInitial, LinearActuator.MOVETYPE.AUTOMATIC);
                    position.move(stonePlacement[previousLocation.ordinal()].positionInitial, LinearActuator.MOVETYPE.AUTOMATIC);
                    facePivot();
                    clampClose();
                }
                if (placementStatus == PlacementStatus.INITIAL) {
                    if ((atTarget(lift, stonePlacement[previousLocation.ordinal()].liftInitial, 0.05)) &&
                        (atTarget(deploy, stonePlacement[previousLocation.ordinal()].deployInitial, 0.05)) &&
                        (atTarget(position, stonePlacement[previousLocation.ordinal()].positionInitial, 0.05)))  {
                        lift.move(stonePlacement[previousLocation.ordinal()].liftFinal, LinearActuator.MOVETYPE.AUTOMATIC);
                        deploy.move(stonePlacement[previousLocation.ordinal()].deployFinal, LinearActuator.MOVETYPE.AUTOMATIC);
                        position.move(stonePlacement[previousLocation.ordinal()].positionFinal, LinearActuator.MOVETYPE.AUTOMATIC);
                        placementStatus = PlacementStatus.FINAL;
                    }
                }

                if (placementStatus == PlacementStatus.FINAL) {
                    if ((atTarget(lift, stonePlacement[previousLocation.ordinal()].liftFinal, 0.05)) &&
                        (atTarget(deploy, stonePlacement[previousLocation.ordinal()].deployFinal, 0.05)) &&
                        (atTarget(position, stonePlacement[previousLocation.ordinal()].positionFinal, 0.05))) {
                        /*Drop a tiny amount to put pressure on the tower to stabilize during transport magic number for now*/
                        lift.move(stonePlacement[previousLocation.ordinal()].liftFinal-0.02, LinearActuator.MOVETYPE.AUTOMATIC);
                        placementStatus = PlacementStatus.AVAILABLE;
                    }
                }

                break;
            case INITIAL_PLATFORM:
                lift.move(LIFT_TRANS_HIGH, LinearActuator.MOVETYPE.AUTOMATIC);
                position.move(POS_PLACE_NEAR, LinearActuator.MOVETYPE.AUTOMATIC);
                facePivot();
                clampClose();
                break;
            case CAPSTONE:
                targetLocation = capStoneLocation();
                /*
                 * FILL IN CODE, probably lift to final height, for now assume that capstone will
                 * be placed right after the platform has been transported.
                 */

                break;
            case UNKNOWN:
            default:
                    break;
        }

        return  status;
    }

    public void update () {
        lift.update();
        deploy.update();
        position.update();
    }

    public void shutdown () {

    }

    public StoneType stoneType () {
        StoneType type = StoneType.UNKNOWN;
        int red1, green1, blue1 = 0;
        int red2, green2, blue2 = 0;

        /* If both distance sensors return nan nothing between the cla(atTarget(position, stonePlacement[targetLocation.ordinal()].positionInitial, 0.05)) &&mp */
        if ((Double.isNaN(distanceRight.getDistance(DistanceUnit.INCH))) &&
            (Double.isNaN(distanceLeft.getDistance(DistanceUnit.INCH)))) {
            type = StoneType.NONE;
        }
        /* If one distance sensor returns nan likely stone not completely in clamp */
        else if ((Double.isNaN(distanceRight.getDistance(DistanceUnit.INCH))) &&
                 (Double.isNaN(distanceLeft.getDistance(DistanceUnit.INCH)))) {
            type = StoneType.UNKNOWN;
        }
        /* Something is in the clamp, determine what it is */
        else {
            red1   = colorLeft.red();
            green1 = colorLeft.green();
            blue1  = colorLeft.blue();
            red2   = colorRight.red();
            green2 = colorRight.green();
            blue2  = colorRight.blue();

            if (isGold(red1, green1, blue1) && isGold(red2,green2,blue2)) {
                type = StoneType.STONE;
            }
            else if ((isBlack(red1, green1, blue1) || isBlack(red2,green2,blue2)) &&
                      (isGold(red1, green1, blue1) || isGold(red2,green2,blue2))) {
                type = StoneType.SKYSTONE;
            }
            else {
                type = StoneType.UNKNOWN;
            }
        }

        return type;
    }

    private void facePivot() {
        pivotServo.setPosition(FACE_POSITION);
    }
    private void edgePivot() {
        pivotServo.setPosition(EDGE_POSITION);
    }

    private void clampOpen () {
        clampServo.setPosition(CLAMP_OPEN);
    }

    private void clampClose () {
        clampServo.setPosition(CLAMP_CLOSE);
    }

    private boolean isGold(int red, int green, int blue) {
        if ((red > GOLD_RED_LOWER)  && ( red < GOLD_RED_UPPER) &&
            (green > GOLD_RED_LOWER)  && ( green < GOLD_RED_UPPER) &&
            (blue > GOLD_RED_LOWER)  && ( blue < GOLD_RED_UPPER)) {
            return true;
        }
        return false;
    }

    private boolean isBlack(int red, int green, int blue) {
        if ((red > BLACK_RED_LOWER)  && ( red < BLACK_RED_UPPER) &&
                (green > BLACK_RED_LOWER)  && ( green < BLACK_RED_UPPER) &&
                (blue > BLACK_RED_LOWER)  && ( blue < BLACK_RED_UPPER)) {
            return true;
        }
        return false;
    }

    private boolean atTarget(LinearActuator linearActuator, double target, double tolerance) {
        if (Math.abs(linearActuator.getCurrentPosition() - target) <= tolerance) {
            return true;
        }
        return false;
    }

    private boolean atTargetservo(Servo s, double target, double tolerance) {
        if (Math.abs(s.getPosition() - target) <= tolerance) {
            return true;
        }
        return false;
    }

    /*
     *Calculate the distance the block is from stone distance sensor.  This will be used to
     * determine what compensation needs to be made by the position lead screw to accomodate
     * differences in where each stone is picked up.
     */
    private double distanceCompensation() {
        if (stoneDistance.getDistance(DistanceUnit.INCH) > 1 ) {

        }
        return 0.0;
    }



    public void setStoneLocation (StonePlacementLocation location) {
        targetLocation = location;
    }
    public StonePlacementLocation getStoneLocation() {
        return currentLocation;
    }

    public StonePlacementLocation getPreviousStoneLocation () {
        return previousLocation;
    }

    public StonePlacementLocation nextStoneLocation(StonePlacementLocation l) {
        switch (l) {
            case LEVEL1_FAR:
                return StonePlacementLocation.LEVEL1_NEAR;
            case LEVEL1_NEAR:
                return StonePlacementLocation.LEVEL2_RIGHT;
            case LEVEL2_RIGHT:
                return StonePlacementLocation.LEVEL2_LEFT;
            case LEVEL2_LEFT:
                return StonePlacementLocation.LEVEL3_FAR;
            case LEVEL3_FAR:
                return StonePlacementLocation.LEVEL3_NEAR;
            case LEVEL3_NEAR:
                return StonePlacementLocation.LEVEL4_RIGHT;
            case LEVEL4_RIGHT:
                return StonePlacementLocation.LEVEL4_LEFT;
            case LEVEL4_LEFT:
                return StonePlacementLocation.LEVEL5_FAR;
            case LEVEL5_FAR:
                return StonePlacementLocation.LEVEL5_NEAR;
            case LEVEL5_NEAR:
                return StonePlacementLocation.LEVEL6_RIGHT;
            case LEVEL6_RIGHT:
                return StonePlacementLocation.LEVEL6_LEFT;
            case LEVEL6_LEFT:
                return StonePlacementLocation.LEVEL7_FAR;
            case LEVEL7_FAR:
                return StonePlacementLocation.LEVEL7_NEAR;
            case LEVEL7_NEAR:
                return StonePlacementLocation.LEVEL8_RIGHT;
            case LEVEL8_RIGHT:
                return StonePlacementLocation.LEVEL8_LEFT;
            case LEVEL8_LEFT:
                return StonePlacementLocation.LEVEL9_FAR;
            case LEVEL9_FAR:
                return StonePlacementLocation.LEVEL9_NEAR;
            case LEVEL9_NEAR:
                return StonePlacementLocation.LEVEL10_RIGHT;
            case LEVEL10_RIGHT:
                return StonePlacementLocation.LEVEL10_LEFT;
            case LEVEL10_LEFT:
            case NOMORE:
            default:
                return StonePlacementLocation.NOMORE;
        }
    }

    public StonePlacementLocation capStoneLocation() {
        /*
         * Capstones should always try to go one level higher than the last placed stone.
         * The capstone by design should cover only half of the block.   The goal being
         * to allow the capstone to always be placed in the same orientation on the tower.
         * In this case no matter what level the tower is if it always targets the far right
         * corner of the tower it will always be at the top.
         */
        switch (previousLocation) {
            case LEVEL1_FAR:
            case LEVEL1_NEAR:
                return StonePlacementLocation.LEVEL2_RIGHT;
            case LEVEL2_RIGHT:
            case LEVEL2_LEFT:
                return StonePlacementLocation.LEVEL3_FAR;
            case LEVEL3_FAR:
            case LEVEL3_NEAR:
                return StonePlacementLocation.LEVEL4_RIGHT;
            case LEVEL4_RIGHT:
            case LEVEL4_LEFT:
                return StonePlacementLocation.LEVEL5_FAR;
            case LEVEL5_FAR:
            case LEVEL5_NEAR:
                return StonePlacementLocation.LEVEL6_RIGHT;
            case LEVEL6_RIGHT:
            case LEVEL6_LEFT:
                return StonePlacementLocation.LEVEL7_FAR;
            case LEVEL7_FAR:
            case LEVEL7_NEAR:
                return StonePlacementLocation.LEVEL8_RIGHT;
            case LEVEL8_RIGHT:
            case LEVEL8_LEFT:
                return StonePlacementLocation.LEVEL9_FAR;
            case LEVEL9_FAR:
            case LEVEL9_NEAR:
                return StonePlacementLocation.LEVEL10_RIGHT;
            case LEVEL10_RIGHT:
            case LEVEL10_LEFT:
            case NOMORE:
            default:
                return StonePlacementLocation.NOMORE;
        }
    }


    public void displayTarget() {
        myOpMode.telemetry.addData("Current Target ", targetLocation);
        myOpMode.telemetry.addData("Previous Target ", previousLocation);
        myOpMode.telemetry.addData("Next Target", nextStoneLocation(targetLocation));
        myOpMode.telemetry.addData("Cap Target", capStoneLocation());
    }

    public void displayPlacement() {
        myOpMode.telemetry.addData("placementSemaphore", placementSemaphore);
        myOpMode.telemetry.addData("placementStatus", placementStatus);
        myOpMode.telemetry.addData("positionCompensate", positionCmpensation);
    }

    public void displayActuators() {
        myOpMode.telemetry.addData("lift actual", lift.getCurrentPosition());
        myOpMode.telemetry.addData("lift target", lift.getTargetPosition());
        myOpMode.telemetry.addData("deploy actual", deploy.getCurrentPosition());
        myOpMode.telemetry.addData("deploy target", deploy.getTargetPosition());
        myOpMode.telemetry.addData("position actual", position.getCurrentPosition());
        myOpMode.telemetry.addData("position target", position.getTargetPosition());
        myOpMode.telemetry.addData("pivot", pivotServo.getPosition());
        myOpMode.telemetry.addData("clamp", clampServo.getPosition());
    }

    public void displaySensors() {
        myOpMode.telemetry.addData("color1", colorLeft);
        myOpMode.telemetry.addData("color2", colorRight);
        myOpMode.telemetry.addData("distanceLeft", distanceLeft);
        myOpMode.telemetry.addData("distanceRight", distanceRight);
        myOpMode.telemetry.addData("distanceStone", stoneDistance);
    }
}
