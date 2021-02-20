package org.firstinspires.ftc.teamcode;

/*
 * Created by
 * James K Rumsey
 * August 2019
 */


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name = "JK Auto Color", group = "Pushbot")
@Disabled
@SuppressWarnings("WeakerAccess")
public class AutonomousCommand_Test_JK_20_21_color extends LinearOpMode {

    AutonomousStatesJK2019 runMe = new AutonomousStatesJK2019();
    AutoCommand cmd_NONE[] = {
            //                             for SHOOT_RING State Value 1 = Shooter Power, Value 2 = Intake Power, Value 3 = Ring deflector Position.

            new AutoCommand(AutonomousStates.AutoStates.MOVE_COLOR, Drive.MoveType.FORWARD, 50, 0.4, 0, 0, 5000), //This barely moved backwards.
            new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.FORWARD, 12, 0.2, 0, 0, 2000),
            new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.REVERSE, 12, 0.2, 0, 0, 2000),
            new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.PIVOTLEFT, 90, 0.4, 0, 0, 2000),
            new AutoCommand(AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP, 18, 0.4, 0, 0, 2000),
            new AutoCommand(AutonomousStates.AutoStates.SHOOT_RING, Drive.MoveType.REVERSE, 0.65, 0.65, 1.0, 0, 4000),
            new AutoCommand(AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP, 18, 0.4, 0, 0, 5000),
    };

    public void runOpMode() {
        runMe.runOpMode(this, hardwareMap, cmd_NONE, cmd_NONE, cmd_NONE);

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