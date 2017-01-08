package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name="Msubscript1AutoLong", group="competition")
public class Msubscript1AutoLong extends LinearOpMode {

    AstroRobotBaseInterface robotBase;

    @Override
    public void runOpMode() throws InterruptedException {
        robotBase = new RobotBaseMsubscript1 (this);

        robotBase.init (hardwareMap);

        waitForStart();

        robotBase.resetGyro();

        robotBase.driveStraight(30, 0);
        robotBase.hanShotFirst();
        robotBase.hanShotFirst();
        robotBase.driveStraight(42, 0);

    }

}