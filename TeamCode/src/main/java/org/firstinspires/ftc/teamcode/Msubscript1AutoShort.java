package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Disabled
@Autonomous(name="Msubscript1AutoShort", group="competition")
public class Msubscript1AutoShort extends LinearOpMode {

    AstroRobotBaseInterface robotBase;

    @Override
    public void runOpMode() throws InterruptedException {
        robotBase = new RobotBaseMsubscript1 (this);

        robotBase.init (hardwareMap);

        waitForStart();

        robotBase.resetGyro();

        robotBase.driveStraight(12, 0);
        robotBase.hanShotFirst();
        robotBase.hanShotFirst();
        robotBase.driveStraight(30, 0);

    }


}