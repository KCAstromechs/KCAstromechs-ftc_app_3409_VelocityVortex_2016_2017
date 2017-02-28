package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Disabled
@Autonomous(name="turntest", group="test")
public class turntest extends LinearOpMode {

    RobotBaseMars robotBase;

    @Override
    public void runOpMode() throws InterruptedException {
        robotBase = new RobotBaseMars();
        robotBase.init(hardwareMap, this);
        waitForStart();
        robotBase.turn(170);
        sleep(1000);
        robotBase.turn(255);
        sleep(1000);
        robotBase.turn(275);
    }
}