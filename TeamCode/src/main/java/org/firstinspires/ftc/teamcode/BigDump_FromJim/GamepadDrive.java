/*
Built for Juden Ki 8578 and Kernel Panic 11959

James K Rumsey
3/02/2019

OVERVIEW:
Provides various methods for conditioning the game controller inputs.


REQUIRED:


ASSUMES:

PARAMETERS:
    float          stickValue      -- This should be either the X or Y value of one of the gamepad
                                      sticks.
    CONDITION_TYPE type            -- The type of conditioning to be performed on the stick value.




STILL TO DO:
    Add some other types of filters.


*/

package org.firstinspires.ftc.teamcode;

public class GamepadDrive {
    public enum CONDITION_TYPE {POWER3, POWER5, POWER7, LINEAR, FULLRANGE_LOWBIAS, FULLRANGE_MIDBIAS, FULLRANGE_HIGH_MANEUVERABILITY, TURTLE_HIGH_MANEUVERABILITY, TURTLE_HIGH_MANEUVERABILITY_FULL_RANGE};
    public enum AXES {X_POS,X_NEG, Y_POS, Y_NEG};
    private AXES zeroAxis = AXES.Y_POS;

    //NOTE:  Lowest entries in teh arrays may not be met due to dead zone restrictions on some
    //gamepads.   If the lowest value is desired it may need to be repeated in the array.
    private final static double[] fullRangeLowBias = {0.0, 0.05, 0.09, 0.1, 0.12, 0.15, 0.18, 0.24, 0.30, 0.36, 0.43, 0.50, 0.60, 0.70, 0.80, 0.90, 1.0, 1.0};
    private final static int fullRangeLowBiasSize = fullRangeLowBias.length;
    private final static double[] fullRangeMidBias = {0.0, 0.1, 0.25, 0.30, 0.35, 0.40, 0.45, 0.48, 0.53, 0.58, 0.63, 0.68, 0.70, 0.75, 0.80, 0.85, 1.0, 1.0};
    private final static int fullRangeMidBiasSize = fullRangeMidBias.length;
    private final static double[] fullRangeHighManeuverability = {0.0, 0.15, 0.04, 0.05, 0.06, 0.07, 0.08, 0.10, 0.12, 0.15, 0.18, 0.21, 0.24, 0.27, 0.50, 0.75, 1.0, 1.0};
    private final static int fullRangeHighManeurverabilitySize = fullRangeHighManeuverability.length;
    private final static double[] turtleHighManeuverability = {0.0, 0.02, 0.04, 0.06, 0.08, 0.09, 0.10, 0.13, 0.16, 0.19, 0.22, 0.25, 0.28, 0.31, 0.34, 0.37, 0.40, 0.45};
    private final static int turtleHighManeurverabilitySize = turtleHighManeuverability.length;
    private final static double[] turtleHighManeuverabilityFullRange = {0.0, 0.02, 0.04, 0.06, 0.08, 0.09, 0.10, 0.11, 0.12, 0.13, 0.14, 0.15, 0.16, 0.17, 0.18, 0.22, 0.25, 0.30, 1.0};
    private final static int turtleHighManeurverabilityFullRangeSize = turtleHighManeuverability.length;

    public double condition (float stickValue, CONDITION_TYPE condition) {
        double retVal=0.0;
        int    index;

        switch (condition) {
            case POWER3:
                retVal = Math.pow(stickValue, 3.0);
                break;
            case POWER5:
                retVal = Math.pow(stickValue, 5.0);
                break;
            case POWER7:
                retVal = Math.pow(stickValue, 7.0);
                break;
            case FULLRANGE_LOWBIAS:
                index = (int) (Math.abs(stickValue) * fullRangeLowBiasSize);
                if (index > (fullRangeLowBiasSize-1)) {index = fullRangeLowBiasSize-1;}
                    retVal = Math.copySign(fullRangeLowBias[index], stickValue);
                break;
            case FULLRANGE_MIDBIAS:
                index = (int) (Math.abs(stickValue) * fullRangeMidBiasSize);
                if (index > (fullRangeMidBiasSize-1)) {index = fullRangeMidBiasSize-1;}
                retVal = Math.copySign(fullRangeMidBias[index], stickValue);
                break;
            case FULLRANGE_HIGH_MANEUVERABILITY:
                index = (int) (Math.abs(stickValue) * fullRangeHighManeurverabilitySize);
                if (index > (fullRangeHighManeurverabilitySize-1)) {index = fullRangeHighManeurverabilitySize-1;}
                retVal = Math.copySign(fullRangeHighManeuverability[index], stickValue);
                break;
            case TURTLE_HIGH_MANEUVERABILITY:
                index = (int) (Math.abs(stickValue) * turtleHighManeurverabilitySize);
                if (index > (turtleHighManeurverabilitySize-1)) {index = turtleHighManeurverabilitySize-1;}
                retVal = Math.copySign(turtleHighManeuverability[index], stickValue);
                break;
            case TURTLE_HIGH_MANEUVERABILITY_FULL_RANGE:
                index = (int) (Math.abs(stickValue) * turtleHighManeurverabilityFullRangeSize);
                if (index > (turtleHighManeurverabilityFullRangeSize-1)) {index = turtleHighManeurverabilityFullRangeSize-1;}
                retVal = Math.copySign(turtleHighManeuverabilityFullRange[index], stickValue);
                break;
            case LINEAR:
            default:
                retVal = stickValue;
                break;

        }

        return (retVal);
    }

    public double angle (double x, double y) {
        double offset = 0.0;
        switch (zeroAxis) {
            case X_POS:
                offset = 0;
                break;
            case X_NEG:
                offset = 180;
                break;
            case Y_POS:
                offset = 90;
                break;
            case Y_NEG:
                offset = 270;
                break;
            default:
                break;
        }



        return( mod360(Math.toDegrees(Math.atan2(y,x)))+offset);
    }

    public double magnitude (double x, double y) {
        return (Math.sqrt(x*x + y*y));
    }

    public void setZeroAxis (AXES axis) {
        zeroAxis = axis;
    }

    public AXES getZeroAxis() {
        return (zeroAxis);
    }

    private double mod360 (double val) {
        int x = (int) val / 360;
        return (val - x*val);
    }
}

