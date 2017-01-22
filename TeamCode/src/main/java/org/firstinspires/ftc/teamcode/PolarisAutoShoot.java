package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous(name="AutoShoot", group="competition")
public class PolarisAutoShoot extends LinearOpMode {

    AstroRobotBaseInterface robotBase;

    @Override
    public void runOpMode() throws InterruptedException {
        robotBase = new RobotBasePolaris ();
        robotBase.initCallingOpMode(this);
        robotBase.init (hardwareMap);

        waitForStart();

        robotBase.resetGyro();

        robotBase.driveStraight(20, 0);
        robotBase.hanShotFirst();
        robotBase.hanShotFirst();
        //robotBase.driveStraight(45, 0);

    }


}