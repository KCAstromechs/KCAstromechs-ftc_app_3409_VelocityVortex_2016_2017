package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by N2Class1 on 2/12/2017.
 */
@Autonomous(name = "Ramp down/ramp up", group = "test")
public class AutoTestTemplate extends LinearOpMode{

    RobotBaseMars rb;

    @Override
    public void runOpMode() throws InterruptedException {
        rb = new RobotBaseMars();
        rb.init(hardwareMap, this);

        waitForStart();
        rb.driveStraight(60, 0.9, 0);
        sleep(200);

    }
}
