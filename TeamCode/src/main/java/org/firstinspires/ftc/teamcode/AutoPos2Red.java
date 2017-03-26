package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Kevin on 2/26/2017.
 */


@Autonomous(name="Red Pos 2", group="Red")
public class AutoPos2Red extends LinearOpMode{

    RobotBaseMarsRD robotBase;

    //determines whether we want to dump data
    boolean debug = false;

    @Override
    public void runOpMode() throws InterruptedException {
        //gives us instance of robot base
        robotBase = new RobotBaseMarsRD();

        //sets up robot variables, moves servos to starting positions, and gives robotbase this instance of OpMode
        robotBase.init(hardwareMap, this);

        //tells robot base whether we want to dump data
        robotBase.setDebug(debug);

        //while touch sensor isn't pressed, manually drive shooter
        while (!robotBase.isCocked()) {
            robotBase.shooterHandler(false, true);
        }
        //dump that we finished cocking if we want to dump data
        if (debug)
            System.out.println("SSS left isCocked");

        //stops shooter after we are finished cocking
        robotBase.shooterHandler(false, false);

        waitForStart();

        //dumps reloader position if we want to dump data
        if (debug)
            System.out.println("SSS reloaderPos @Start: " + robotBase.reloaderServo.getPosition());

        //NOTE: if we are about to execute a method that may take some time,
        // then we make sure we are still active by calling opModeIsActive()

        //drives forward to an optimal position to take a shot
        if (opModeIsActive()) robotBase.driveStraight(28, 0);

        //takes two shots
        while (opModeIsActive() && robotBase.shooterHandler(true, false)) ;
        while (opModeIsActive() && robotBase.reloadHandler(true)) ;
        while (opModeIsActive() && robotBase.shooterHandler(true, false)) ;


    }
}
