
package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class motorVelocityControl {
    public enum VELOCITY_UNITS {RPM, RADIANS_PER_SECOND};
    public enum PID_TYPE {P, PI, PIFF, SYMCED};
    private DcMotor motor;
    private VELOCITY_UNITS velocity_units;
    private PID_TYPE pid_type;
    private double gearRatio;
    private double motorRatio;

    private int currentCounts;
    private int previousCounts;
    private long currentTime;
    private long previousTime;
    private double currentVelocity;
    private double targetVelocity=0.0;

    public boolean init (DcMotor motor, double gearRatio, double motorRatio,
                         VELOCITY_UNITS velocity_units, PID_TYPE pid_type, OpMode opMode) {
        //Add some error and range checking on parameters
        this.motor = motor;
        this.gearRatio = gearRatio;
        this.motorRatio = motorRatio;
        this.velocity_units = velocity_units;
        this.pid_type = pid_type;
        currentCounts = motor.getCurrentPosition();
        previousCounts = currentCounts;
        currentTime = SystemClock.elapsedRealtime();
        previousTime = currentTime;
        return true;
    }

    //Regardless of units chosen internally always operate on RPM
    private double RPS_2_RPM (double rps) {
        return (rps*30/ Math.PI);
    }

    private double RPM_2_RPS (double rpm) {
        return (rpm*Math.PI/30);
    }

    public void setVelocity(double v) {
        targetVelocity = v;
    }

    public double currentVelocity () {
        double v=0.0;

        updateVelocity();
        switch (velocity_units) {
            case RPM:
                v = currentVelocity;
                break;
            case RADIANS_PER_SECOND:
                v = RPM_2_RPS(currentVelocity);
                break;
            default:
                v = 0.0;
        }
        return (v);
    }

    private void updateVelocity() {
        long deltaTime=0;
        int  deltaCounts=0;
        double v = 0.0;

        previousTime = currentTime;
        previousCounts = currentCounts;
        currentTime = SystemClock.elapsedRealtime();
        currentCounts = motor.getCurrentPosition();

        deltaTime = currentTime-previousTime;
        deltaCounts = currentCounts-previousCounts;

        if (deltaTime > 0) {
            v = (deltaCounts / (gearRatio*motorRatio)) / (deltaTime*1000);
        }
        currentVelocity = v;
    }

    //
    // Control the velocity based on the specified control type.
    // Currently everything is straight brute force, eventually create a target range that
    // is acceptable for velocity.    I.e. for 200 RPM 199 and 201 is fine, etc.....
    // For now use hard coded values for P, eventually make them configured values during initialization
    //
    public void update() {
        double v;
        updateVelocity();

        switch (pid_type) {
            case P:
                if (currentVelocity >targetVelocity ) {
                    motor.setPower(motor.getPower()-0.01);
                }
                if (currentVelocity < targetVelocity) {
                    motor.setPower(motor.getPower()+0.01);
                }
                break;
            case PI:
                break;
            case PIFF:
                break;
            case SYMCED:
                break;
            default:
                break;
        }

    }


}
