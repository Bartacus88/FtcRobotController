package org.firstinspires.ftc.teamcode;

/*
 * Created by
 * James K Rumsey
 * August 2019
  */



import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name="OMNI H Template", group="Pushbot")
@Disabled
@SuppressWarnings("WeakerAccess")
public class AutonomousCommandOMNI_H_template extends LinearOpMode {

    AutonomousStatesOMNI_H runMe = new AutonomousStatesOMNI_H();
    AutoCommand cmd[] = {
            new AutoCommand( AutonomousStates.AutoStates.MOVE, Drive.MoveType.FORWARD, 12, 0.4, 0,0, 20000),
            new AutoCommand( AutonomousStates.AutoStates.MOVE, Drive.MoveType.REVERSE, 12, 0.4, 0,0, 20000),
            new AutoCommand( AutonomousStates.AutoStates.MOVE, Drive.MoveType.PIVOTRIGHT, 90, 0.4, 0,0,2000),
            new AutoCommand( AutonomousStates.AutoStates.MOVE, Drive.MoveType.PIVOTLEFT, 270, 0.4, 0,0,2000),
            new AutoCommand( AutonomousStates.AutoStates.MOVE, Drive.MoveType.CRABRIGHT, 12, 0.2, 0,0,20000),
            new AutoCommand( AutonomousStates.AutoStates.MOVE, Drive.MoveType.CRABLEFT, 12, 0.2, 0,0,20000),
            new AutoCommand( AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP,         18, 0.5, 0,0, 5000),

    };

    public void runOpMode() {
        runMe.runOpMode(this, hardwareMap, cmd);
    }

}