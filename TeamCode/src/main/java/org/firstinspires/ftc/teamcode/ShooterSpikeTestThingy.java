package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Kevin on 10/5/2016.
 */
@Disabled
@Autonomous(name="Test Autonomous2", group="Test")
public class ShooterSpikeTestThingy extends LinearOpMode {

    AstroRobotBaseInterface robotBase;

    @Override
    public void runOpMode() throws InterruptedException {
        robotBase = new RobotBaseOld(hardwareMap, this);
        telemetry.addData("Test", "test data");
        System.out.println("eribuaivshfx9u");
        robotBase.calibrateGyro();
        telemetry.addData("Gyro", "Calibrated");
        System.out.println("gyro calibrated");

        waitForStart();

        robotBase.gyroResetZaxisIntegrator();

        robotBase.driveStraight(29, 0.3, 0, 1.0f);

    }

}