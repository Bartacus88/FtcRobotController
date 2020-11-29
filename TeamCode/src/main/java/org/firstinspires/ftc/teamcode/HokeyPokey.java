package org.firstinspires.ftc.teamcode;

/*
 * Created by
 * James K Rumsey
 * December 2018
 *
 * Purpose: Absurdity for absurdities sake has value.
 *
 */




import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;


//@Autonomous(name="Hokey Pokey", group="Pushbot")
@SuppressWarnings("WeakerAccess")
public class HokeyPokey extends LinearOpMode {

    AutonomousStates runMe = new AutonomousStates();
    AutoCommand cmd[] = {
            //Right in, out, in
            new AutoCommand(AutonomousStates.AutoStates.RAISELIFT, Drive.MoveType.STOP, 1.0,0,0,0, 5000),
            new AutoCommand(AutonomousStates.AutoStates.LOWERLIFT, Drive.MoveType.STOP, 0.0,0,0,0, 5000),
            new AutoCommand( AutonomousStates.AutoStates.MOVE, Drive.MoveType.FORWARD, 24, 0.3, 0,0, 2000),
            new AutoCommand( AutonomousStates.AutoStates.MOVE, Drive.MoveType.REVERSE, 24, 0.3, 0,0, 2000),
            new AutoCommand( AutonomousStates.AutoStates.MOVE, Drive.MoveType.FORWARD, 24, 0.3, 0,0, 2000),
            new AutoCommand( AutonomousStates.AutoStates.MOVE, Drive.MoveType.CRABLEFT, 12, 0.3, 0,0, 2000),
            new AutoCommand( AutonomousStates.AutoStates.MOVE, Drive.MoveType.CRABRIGHT, 12, 0.3, 0,0, 2000),
            new AutoCommand( AutonomousStates.AutoStates.MOVE, Drive.MoveType.PIVOTRIGHT, 90, 0.3, 0,0,2000),
            new AutoCommand( AutonomousStates.AutoStates.MOVE, Drive.MoveType.PIVOTLEFT, 270, 0.3, 0,0,2000),
            new AutoCommand( AutonomousStates.AutoStates.WAIT, Drive.MoveType.STOP,         18, 0.5, 0,0, 5000),

    };

    public void runOpMode() {
        runMe.runOpMode(this, hardwareMap, cmd);
    }

}