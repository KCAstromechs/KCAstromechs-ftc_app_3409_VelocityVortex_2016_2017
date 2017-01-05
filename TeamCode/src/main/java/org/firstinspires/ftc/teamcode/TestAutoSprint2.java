package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Test Autonomous", group="Test")
@Disabled
public class TestAutoSprint2 extends LinearOpMode {

    AstroRobotBaseInterface robotBase;

    @Override
    public void runOpMode() throws InterruptedException {
        robotBase = new RobotBaseOld(hardwareMap,this);
        telemetry.addData("Test", "test data");
        System.out.println("eribuaivshfx9u");
        robotBase.calibrateGyro();
        telemetry.addData("Gyro", "Calibrated");
        System.out.println("gyro calibrated");

        waitForStart();

        robotBase.gyroResetZaxisIntegrator();

        robotBase.driveStraight(8, 0.5, 0, 1.0f);
        robotBase.turn(30, 0.5);
        robotBase.driveStraight(56, 0.5, 45, 1.0f);
        robotBase.turn(80, 0.5);
        robotBase.driveStraight(24, 0.5, 90, 1.0f);
        sleep(1000);
        robotBase.driveStraight(4, 0.5, 90, -1.0f);
        robotBase.turn(15, 0.5);
        robotBase.driveStraight(40, 0.5, 0, 1.0f);
        robotBase.turn(80, 0.5);
        robotBase.driveStraight(24, 0.5, 90, 1.0f);

    }

}
