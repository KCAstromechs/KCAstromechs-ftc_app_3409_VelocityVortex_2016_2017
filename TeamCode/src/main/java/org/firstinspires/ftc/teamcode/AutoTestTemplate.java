package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by N2Class1 on 2/12/2017.
 */
@Autonomous(name = "Ramp down/ramp up", group = "test")
public class AutoTestTemplate extends LinearOpMode{

    RobotBasePolaris rb;

    @Override
    public void runOpMode() throws InterruptedException {
        rb = new RobotBasePolaris();
        rb.initCallingOpMode(this);
        rb.init(hardwareMap);

        waitForStart();
        rb.turn(90);
        sleep(200);
        System.out.println("zRotation = " + rb.zRotation);

    }
}
