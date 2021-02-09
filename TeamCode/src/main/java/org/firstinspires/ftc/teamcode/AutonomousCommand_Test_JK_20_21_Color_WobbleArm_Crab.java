package org.firstinspires.ftc.teamcode;

/*
 * Created by
 * James K Rumsey
 * August 2019
 */


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name = "JK Auto Color WobbleArm Crab", group = "Pushbot")
//@Disabled
@SuppressWarnings("WeakerAccess")
public class AutonomousCommand_Test_JK_20_21_Color_WobbleArm_Crab extends LinearOpMode {

    AutonomousStatesJK2019_color runMe = new AutonomousStatesJK2019_color();
    AutoCommand cmd[] = {
            new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.CRABLEFT, 8, 0.4, 0, 0, 1000),

            new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.REVERSE, 8, 0.4, 0, 0, 1000),
            new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.CRABLEFT, 30, 0.4, 0, 0, 5000),
            new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.FORWARD, 11, 0.4, 0, 0, 1000),
            //                            for MOVE_WOBBLE_ARM, State Value 1 = Wobble Arm Position.
            new AutoCommand(AutonomousStates.AutoStates.MOVE_WOBBLE_ARM, Drive.MoveType.REVERSE, 0.1, 0, 0, 0, 500),
            //                            for MOVE_WOBBLE_ARM, State Value 1 = Wobble Arm Position.
            new AutoCommand(AutonomousStates.AutoStates.MOVE_WOBBLE_ARM, Drive.MoveType.REVERSE, 0, 0, 0, 0, 500),
            new AutoCommand(AutonomousStates.AutoStates.MOVE, Drive.MoveType.CRABRIGHT, 12, 0.4, 0, 0, 1000),
            //                             for SHOOT_RING State Value 1 = Shooter Power, Value 2 = Intake Power, Value 3 = Ring deflector Position.
            new AutoCommand(AutonomousStates.AutoStates.SHOOT_RING, Drive.MoveType.REVERSE, .80, .85, 1.0, 0, 2000),
            new AutoCommand(AutonomousStates.AutoStates.MOVE_COLOR, Drive.MoveType.CRABLEFT, 25, 0.4, 0, 0, 5000),
            new AutoCommand(AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP, 18, 0.4, 0, 0, 5000),
    };

    public void runOpMode() {
        runMe.runOpMode(this, hardwareMap, cmd);

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