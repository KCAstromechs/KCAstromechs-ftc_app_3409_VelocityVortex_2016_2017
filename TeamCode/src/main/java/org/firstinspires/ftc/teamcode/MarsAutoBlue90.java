package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Mars Blue 90", group="Blue")
public class MarsAutoBlue90 extends LinearOpMode {

    RobotBaseMarsRD robotBase;
    boolean debug = true;

    @Override
    public void runOpMode() throws InterruptedException {
        robotBase = new RobotBaseMarsRD();
        robotBase.init(hardwareMap, this);
        robotBase.initVuforia();
        robotBase.setDebug(debug);
        int pos = -1;
        double shiftedAvg;
        double deltaX;
        double correctionAngle;

        while(!robotBase.isCocked()){
            robotBase.shooterHandler(false, true);
        }

        if(debug)
            System.out.println("SSS left isCocked");

        robotBase.shooterHandler(false, false);

        //All of that is set-up doing things like setting up the robotBase, cocking the shooter, setting up variables

        waitForStart();

        if (debug)
            System.out.println("SSS reloaderPos @Start: " + robotBase.reloaderServo.getPosition());

        //initial drive

        if(opModeIsActive()) robotBase.driveStraight(23, 0);

        //turns parallel to ramp
        if(opModeIsActive()) robotBase.turn(38);

        //second drive to align the robot to the first beacon one one axis
        if(opModeIsActive()) robotBase.driveStraight(24, 38);

        //turns robot to face beacon
        if(opModeIsActive()) robotBase.turn(85);

        //waits for robot to come to rest, then takes picture to determine beacon orientation
        if(opModeIsActive()) sleep(500);
        if (debug)
            System.out.println("SSS heading after turn to take picture: " + robotBase.zRotation);
        if(opModeIsActive()) pos = robotBase.takePicture();

        //do some math to determine the angle the robot should drive to the beacon with
        shiftedAvg = ((90 - robotBase.getZRotation()) * robotBase.PIXELS_PER_DEGREE) + robotBase.getLastPicBeaconAvg();
        deltaX = (340 - shiftedAvg)/robotBase.PIXELS_PER_INCH;
        correctionAngle = Math.toDegrees(Math.atan(deltaX/30.));

        //outputs beacon info for testing purposes
        if (debug) {
            System.out.println("px per degree: " + robotBase.PIXELS_PER_DEGREE);
            System.out.println("lastPicBeaconAvg: " + robotBase.getLastPicBeaconAvg());
            System.out.println("zRotation: " + robotBase.getZRotation());
            System.out.println("shiftedAvg: " + shiftedAvg);
            System.out.println("deltaX: " + deltaX);
            System.out.println("correctionAngle: " + correctionAngle);
            System.out.println(pos);
        }

        telemetry.addData("pos ", pos);
        telemetry.update();

        //drive to actually hit the beacon's button
        if (pos == RobotBasePolaris.BEACON_RED_BLUE){
            if(opModeIsActive()) robotBase.turn(105 +(float)correctionAngle);
            try {
                if(opModeIsActive()) robotBase.pushButton(105 + (int)correctionAngle, 100, 2);
            }
            catch (TimeoutException e) {
                if(opModeIsActive()) robotBase.driveStraight(-20, -0.5, 100);
            }
        }
        else if (pos == RobotBasePolaris.BEACON_BLUE_RED) {
            try {
                if(opModeIsActive()) robotBase.pushButton(90 + (int)correctionAngle, 90, 2);
            }
            catch (TimeoutException e) {
                if(opModeIsActive()) robotBase.driveStraight(-20, -0.5, 90);
            }
        }

        //turn around to shoot into the center vortex
        if(opModeIsActive()) robotBase.turn(314);


        //shoot into center vortex
        while (opModeIsActive() && robotBase.shooterHandler(true, false));
        while (opModeIsActive() && robotBase.reloadHandler(true));
        while (opModeIsActive() && robotBase.shooterHandler(true, false));

        if (opModeIsActive()) sleep(451);

        if (debug)
            System.out.println("SSS reloaderPos @end: " + robotBase.reloaderServo.getPosition());

        //drive to second beacon
        if(opModeIsActive()) robotBase.turn(0);
        if(opModeIsActive()) robotBase.driveStraight(43, 0);

        //turn to beacon and take picture
        if(opModeIsActive()) robotBase.turn(85);
        if(opModeIsActive()) sleep(500);
        if(opModeIsActive()) pos = robotBase.takePicture();

        //do some math to determine the angle the robot should drive to the beacon with
        shiftedAvg = ((90 - robotBase.getZRotation()) * robotBase.PIXELS_PER_DEGREE) + robotBase.getLastPicBeaconAvg();
        deltaX = (340 - shiftedAvg)/robotBase.PIXELS_PER_INCH;
        correctionAngle = Math.toDegrees(Math.atan(deltaX/20.));

        //outputs beacon info for testing purposes
        if (debug) {
            System.out.println("px per degree: " + robotBase.PIXELS_PER_DEGREE);
            System.out.println("lastPicBeaconAvg: " + robotBase.getLastPicBeaconAvg());
            System.out.println("zRotation: " + robotBase.getZRotation());
            System.out.println("shiftedAvg: " + shiftedAvg);
            System.out.println("deltaX: " + deltaX);
            System.out.println("correctionAngle: " + correctionAngle);
            System.out.println(pos);
        }

        telemetry.addData("pos ", pos);
        telemetry.update();

        //drive to actually hit the beacon's button
        if (pos == RobotBasePolaris.BEACON_RED_BLUE){
            if(opModeIsActive()) robotBase.turn(100 +(float)correctionAngle);
            try {
                if(opModeIsActive()) robotBase.pushButton(100 + (int)correctionAngle, 100, 3);
            }
            catch (TimeoutException e) {
                if(opModeIsActive()) robotBase.driveStraight(-20, -0.5, 100);
            }
        }
        else if (pos == RobotBasePolaris.BEACON_BLUE_RED) {
            try {
                if(opModeIsActive()) robotBase.pushButton(90 + (int)correctionAngle, 90, 3);
            }
            catch (TimeoutException e) {
                if(opModeIsActive()) robotBase.driveStraight(-20, -0.5, 90);
            }
        }

        if(opModeIsActive()) sleep(1000);

        //clean up the mess we made
        robotBase.deconstruct();
        robotBase = null;
    }

}