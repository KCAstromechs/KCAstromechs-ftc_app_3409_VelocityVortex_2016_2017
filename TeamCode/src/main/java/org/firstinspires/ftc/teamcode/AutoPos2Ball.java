package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Kevin on 2/26/2017.
 */


@Autonomous(name="Auto Pos 2 + ball", group="Autonomous")
public class AutoPos2Ball extends LinearOpMode{

    RobotBaseMarsRD robotBase;
    boolean debug = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robotBase = new RobotBaseMarsRD();
        robotBase.init(hardwareMap, this);
        robotBase.setDebug(debug);

        while (!robotBase.isCocked()) {
            robotBase.shooterHandler(false, true);
        }

        if (debug)
            System.out.println("SSS left isCocked");

        robotBase.shooterHandler(false, false);

        waitForStart();

        if (debug)
            System.out.println("SSS reloaderPos @Start: " + robotBase.reloaderServo.getPosition());

        //initial drive

        if (opModeIsActive()) robotBase.driveStraight(25, 0);

        while (opModeIsActive() && robotBase.shooterHandler(true, false)) ;
        while (opModeIsActive() && robotBase.reloadHandler(true)) ;
        while (opModeIsActive() && robotBase.shooterHandler(true, false)) ;

        if (opModeIsActive()) sleep(20000);
        if (opModeIsActive()) robotBase.driveStraight(25, 0);
    }
}
