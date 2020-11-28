package org.firstinspires.ftc.teamcode;

/*
 * Created by
 * James K Rumsey
 * August 2019
  */



import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;



@Autonomous(name="JK Template", group="Pushbot")
@SuppressWarnings("WeakerAccess")
public class AutonomousCommandJK2019_template extends LinearOpMode {

    AutonomousStatesJK2019 runMe = new AutonomousStatesJK2019();
    AutoCommand cmd[] = {
            new AutoCommand( AutonomousStates.AutoStates.MOVE, Drive.MoveType.FORWARD, 12, 0.2, 0,0, 2000),
            //new AutoCommand( AutonomousStates.AutoStates.MOVE, Drive.MoveType.REVERSE, 12, 0.2, 0,0, 2000),
            //new AutoCommand( AutonomousStates.AutoStates.MOVE, Drive.MoveType.FORWARD, 12, 0.2, 0,0, 2000),
            //new AutoCommand( AutonomousStates.AutoStates.MOVE, Drive.MoveType.PIVOTRIGHT, 90, 0.2, 0,0,2000),
            //new AutoCommand( AutonomousStates.AutoStates.MOVE, Drive.MoveType.PIVOTLEFT, 270, 0.5, 0,0,2000),
            new AutoCommand( AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP,         18, 0.5, 0,0, 5000),

    };

    public void runOpMode() {
        runMe.runOpMode(this, hardwareMap, cmd);
    }

}