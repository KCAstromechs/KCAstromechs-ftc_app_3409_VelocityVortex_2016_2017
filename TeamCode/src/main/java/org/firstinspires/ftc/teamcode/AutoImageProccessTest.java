package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;

/**
 * Created by N2Class1 on 1/29/2017.
 */

@Autonomous(name = "AutoImageProccessTest", group = "State testing")
public class AutoImageProccessTest extends LinearOpMode{

    RobotBasePolaris robotBase;

    @Override
    public void runOpMode() throws InterruptedException {
        robotBase = new RobotBasePolaris();
        robotBase.initCallingOpMode(this);
        robotBase.init(hardwareMap);
        waitForStart();
        int pos = robotBase.takeQuickPicture(); //pos doesn't currently contain any useful data
    }




}
