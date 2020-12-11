package org.firstinspires.ftc.teamcode;

/*
 * Created by
 * James K Rumsey
 * December 3/26/2019
 *
 */




import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name="Deploy", group="Pushbot")
@Disabled
@SuppressWarnings("WeakerAccess")
public class Deploy_Autonomous extends LinearOpMode {

    AutonomousStates runMe = new AutonomousStates();
    AutoCommand cmd[] = {
            new AutoCommand(AutonomousStates.AutoStates.RAISELIFT, Drive.MoveType.STOP, 1.0,0,0,0, 10000),
            new AutoCommand( AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP,         18, 0.5, 0,0, 5000),

    };

    public void runOpMode() {
        runMe.runOpMode(this, hardwareMap, cmd);
    }

}