package org.firstinspires.ftc.teamcode;

/*
 * Created by
 * James K Rumsey
 * August 2019
  */



import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name = "JK Auto Pivot R90 L270", group = "Pushbot")
@Disabled
@SuppressWarnings("WeakerAccess")
public class AutonomousCommand_Test_JK_20_21 extends LinearOpMode {

    AutonomousStatesJK2019 runMe = new AutonomousStatesJK2019();
    AutoCommand cmd_FOUR[] = {
            //new AutoCommand( AutonomousStates.AutoStates.MOVE, Drive.MoveType.REVERSE, 12, 0.2, 0,0, 2000), //This barely moved backwards.
            //new AutoCommand( AutonomousStates.AutoStates.MOVE, Drive.MoveType.REVERSE, 12, 0.2, 0,0, 2000),
            //new AutoCommand( AutonomousStates.AutoStates.MOVE, Drive.MoveType.FORWARD, 12, 0.2, 0,0, 500),
            new AutoCommand( AutonomousStates.AutoStates.MOVE, Drive.MoveType.PIVOTRIGHT, 90, 0.2, 0,0,2000),
            new AutoCommand( AutonomousStates.AutoStates.MOVE, Drive.MoveType.PIVOTLEFT, 90, 0.2, 0,0,2000),
            //new AutoCommand( AutonomousStates.AutoStates.MOVE, Drive.MoveType.REVERSE, 12, 0.2, 0,0, 2000),
            new AutoCommand( AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP,         18, 0.2, 0,0, 5000),

    };

    AutoCommand cmd_ONE[] = {
            //new AutoCommand( AutonomousStates.AutoStates.MOVE, Drive.MoveType.REVERSE, 12, 0.2, 0,0, 2000), //This barely moved backwards.
            //new AutoCommand( AutonomousStates.AutoStates.MOVE, Drive.MoveType.REVERSE, 12, 0.2, 0,0, 2000),
            //new AutoCommand( AutonomousStates.AutoStates.MOVE, Drive.MoveType.FORWARD, 12, 0.2, 0,0, 500),
            new AutoCommand( AutonomousStates.AutoStates.MOVE, Drive.MoveType.PIVOTRIGHT, 180, 0.8, 0,0,2000),
            new AutoCommand( AutonomousStates.AutoStates.MOVE, Drive.MoveType.PIVOTLEFT, 180, 0.8, 0,0,2000),
            //new AutoCommand( AutonomousStates.AutoStates.MOVE, Drive.MoveType.REVERSE, 12, 0.2, 0,0, 2000),
            new AutoCommand( AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP,         18, 0.8, 0,0, 5000),

    };

    AutoCommand cmd_NONE[] = {
            //new AutoCommand( AutonomousStates.AutoStates.MOVE, Drive.MoveType.REVERSE, 12, 0.2, 0,0, 2000), //This barely moved backwards.
            //new AutoCommand( AutonomousStates.AutoStates.MOVE, Drive.MoveType.REVERSE, 12, 0.2, 0,0, 2000),
            //new AutoCommand( AutonomousStates.AutoStates.MOVE, Drive.MoveType.FORWARD, 12, 0.2, 0,0, 500),
            new AutoCommand( AutonomousStates.AutoStates.MOVE, Drive.MoveType.PIVOTRIGHT, 120, 0.4, 0,0,2000),
            new AutoCommand( AutonomousStates.AutoStates.MOVE, Drive.MoveType.PIVOTLEFT, 120, 0.4, 0,0,2000),
            //new AutoCommand( AutonomousStates.AutoStates.MOVE, Drive.MoveType.REVERSE, 12, 0.2, 0,0, 2000),
            new AutoCommand( AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP,         18, 0.4, 0,0, 5000),

    };
    public void runOpMode() {
                runMe.runOpMode(this, hardwareMap, cmd_NONE, cmd_ONE, cmd_FOUR);

        /*
                telemetry.clear();
        int i =0;
        if(runMe.numRingsTest == RingStackDetection2.Rings2.FOUR)
        {telemetry.addData(String.format("FOUR"),i);}
        else if(runMe.numRingsTest == RingStackDetection2.Rings2.ONE)
        {telemetry.addData(String.format("ONE"),i);}
        else
        {{telemetry.addData(String.format("NONE"),i);}}
        telemetry.update();
        sleep(10000);
         */
    }

}