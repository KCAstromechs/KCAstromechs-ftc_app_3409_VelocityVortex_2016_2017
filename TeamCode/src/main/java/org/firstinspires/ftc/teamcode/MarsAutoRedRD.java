package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Mars RedRD", group="red")
public class MarsAutoRedRD extends LinearOpMode {

    RobotBaseMarsRD robotBase;
    boolean debug = false;

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

        waitForStart();

        if (debug)
            System.out.println("SSS reloaderPos @Start: " + robotBase.reloaderServo.getPosition());

        //initial drive

        if(opModeIsActive()) robotBase.driveStraight(25, 0);

        //turns parallel to ramp
        if(opModeIsActive()) robotBase.turn(322);

        //second drive to align the robot to the first beacon on one axis
        if(opModeIsActive()) robotBase.driveStraight(24, 322);

        //turns robot to face beacon
        if(opModeIsActive()) robotBase.turn(275);

        //waits for robot to come to rest, then takes picture to determine beacon orientation
        if(opModeIsActive()) sleep(500);
        if (debug)
            System.out.println("SSS heading after turn to take picture: " + robotBase.zRotation);
        if(opModeIsActive()) pos = robotBase.takePicture();

        //do some math to determine the angle the robot should drive to the beacon with
        shiftedAvg = ((270 - robotBase.getZRotation()) * robotBase.PIXELS_PER_DEGREE) + robotBase.getLastPicBeaconAvg();
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

        if (pos == RobotBasePolaris.BEACON_RED_BLUE){
            if(opModeIsActive()) robotBase.turn(270 +(float)correctionAngle);
            try {
                if(opModeIsActive()) robotBase.pushButton(270 + (int)correctionAngle, 270, 2);
            }
            catch (TimeoutException e) {
                if(opModeIsActive()) robotBase.driveStraight(-20, -0.5, 270);
            }
        }
        else if (pos == RobotBasePolaris.BEACON_BLUE_RED) {
            try {
                if(opModeIsActive()) robotBase.pushButton(280 + (int)correctionAngle, 280, 2);
            }
            catch (TimeoutException e) {
                if(opModeIsActive()) robotBase.driveStraight(-20, -0.5, 280);
            }
        }

        if(opModeIsActive()) robotBase.turn(122);


        while(opModeIsActive() && robotBase.shooterHandler(true, false)) ;
        while(opModeIsActive() && robotBase.reloadHandler(true));
        while(opModeIsActive() && robotBase.shooterHandler(true, false)) ;

        if(opModeIsActive()) sleep(1000);

        if (debug)
            System.out.println("SSS reloaderPos @end: " + robotBase.reloaderServo.getPosition());


        /*
        robotBase.turn(0);
        robotBase.driveStraight(48, 0);

        robotBase.turn(90);
        sleep(2000);
        pos = robotBase.takePicture();

        telemetry.addData("",pos);
        telemetry.update();


        shiftedAvg = ((90 - robotBase.getZRotation()) * robotBase.PIXELS_PER_DEGREE) + robotBase.getLastPicBeaconAvg();
        deltaX = (340 - shiftedAvg)/robotBase.PIXELS_PER_INCH;
        correctionAngle = Math.toDegrees(Math.atan(deltaX/30.));


        //push chosen button0
        if (pos == RobotBasePolaris.BEACON_RED_BLUE){
            robotBase.turn(108 + (float)correctionAngle);
            try {robotBase.pushButton(108 + (int)correctionAngle, 108, 2);}
            catch (TimeoutException e) {
                robotBase.driveStraight(-12, -0.5, 108);
            }
        }
        else if (pos == RobotBasePolaris.BEACON_BLUE_RED) {
            try {robotBase.pushButton(90 + (int)correctionAngle, 90, 2);}
            catch (TimeoutException e) {
                robotBase.driveStraight(-12, -0.5, 90);
            }
        }

        robotBase.deconstruct();
        robotBase = null;

        */
    }

}