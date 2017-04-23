package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Kevin on 2/26/2017.
 */


@Autonomous(name="Blue Pos 2 Center", group="Blue")
public class NovaPos2BlueCenter extends LinearOpMode{

    RobotBaseNova robotBase;
    boolean debug = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robotBase = new RobotBaseNova();
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
