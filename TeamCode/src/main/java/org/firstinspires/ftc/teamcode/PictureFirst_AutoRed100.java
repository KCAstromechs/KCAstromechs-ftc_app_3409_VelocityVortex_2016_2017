package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Red Pos 1 Long Distance", group="Red")
public class PictureFirst_AutoRed100 extends LinearOpMode {

    RobotBaseMarsRD robotBase;
    boolean debug = true;

    @Override
    public void runOpMode() throws InterruptedException {
        robotBase = new RobotBaseMarsRD();
        robotBase.init(hardwareMap, this);
        robotBase.initVuforia();
        robotBase.setDebug(debug);
        int pos = -1;
        BeaconOrientation picData;
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

        picData = robotBase.takeLongDistancePicture();

        //initial drive out to first beacon
        if(opModeIsActive()) robotBase.driveStraight(40, 315);

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

        //if the current picture matches the last one and they both say blue is on the left, go for the left!
        if ((pos == RobotBaseMarsRD.BEACON_BLUE_RED || pos == 0) && (picData == BeaconOrientation.BLUE_RED_BLUE_RED || picData == BeaconOrientation.RED_BLUE_BLUE_RED)){
            try {
                if(opModeIsActive()) robotBase.pushButton(280 + (int)correctionAngle, 280, 2);
            }
            catch (TimeoutException e) {
                if(opModeIsActive()) robotBase.driveStraight(-20, -0.5, 280);
            }
        }
        //If they match and say blue is on the right, go for the right
        else if ((pos == RobotBaseMarsRD.BEACON_RED_BLUE || pos == 0) && (picData == BeaconOrientation.BLUE_RED_RED_BLUE || picData == BeaconOrientation.RED_BLUE_RED_BLUE)){
            if(opModeIsActive()) robotBase.turn(275 +(float)correctionAngle);
            try {
                if(opModeIsActive()) robotBase.pushButton(270 + (int)correctionAngle, 270, 2);
            }
            catch (TimeoutException e) {
                if(opModeIsActive()) robotBase.driveStraight(-20, -0.5, 270);
            }
        }
        //Now if they don't match, we trust the picture we just took
        else {
            //If it says blue is on the right, we go right
            if (pos == RobotBasePolaris.BEACON_RED_BLUE) {
                if(opModeIsActive()) robotBase.turn(275 +(float)correctionAngle);
                try {
                    if(opModeIsActive()) robotBase.pushButton(270 + (int)correctionAngle, 270, 2);
                }
                catch (TimeoutException e) {
                    if(opModeIsActive()) robotBase.driveStraight(-20, -0.5, 270);
                }
            }
            //If it says blue is on the left, we go left
            else if (pos == RobotBasePolaris.BEACON_BLUE_RED) {
                try {
                    if(opModeIsActive()) robotBase.pushButton(280 + (int)correctionAngle, 280, 2);
                }
                catch (TimeoutException e) {
                    if(opModeIsActive()) robotBase.driveStraight(-20, -0.5, 280);
                }
            }
        }

        //turn around to shoot into the center vortex
        if(opModeIsActive()) robotBase.turn(70);

        //shoot into center vortex
        while (opModeIsActive() && robotBase.shooterHandler(true, false));
        while (opModeIsActive() && robotBase.reloadHandler(true));
        while (opModeIsActive() && robotBase.shooterHandler(true, false));

        //Turn to drive to the center beacon
        if (opModeIsActive()) robotBase.turn(0);

        //Determine how far to drive depending on the orientations of both beacons.
        if (picData == BeaconOrientation.RED_BLUE_BLUE_RED){
            if (opModeIsActive()) robotBase.driveStraight(53, 0);
        } else if (picData == BeaconOrientation.BLUE_RED_BLUE_RED || picData == BeaconOrientation.RED_BLUE_RED_BLUE){
            if (opModeIsActive()) robotBase.driveStraight(47, 0);
        } else if (picData == BeaconOrientation.BLUE_RED_RED_BLUE){
            if (opModeIsActive()) robotBase.driveStraight(41, 0);
        }

        //turn to face second beacon
        if (opModeIsActive()) robotBase.turn(275);

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

        //drive to actually hit the second beacon's button
        //if the current picture matches the last one and they both say blue is on the left, go for the left!
        if ((pos == RobotBaseMarsRD.BEACON_BLUE_RED || pos == 0) && (picData == BeaconOrientation.BLUE_RED_BLUE_RED || picData == BeaconOrientation.RED_BLUE_BLUE_RED)){
            try {
                if(opModeIsActive()) robotBase.pushButton(280 + (int)correctionAngle, 280, 2);
            }
            catch (TimeoutException e) {
                if(opModeIsActive()) robotBase.driveStraight(-20, -0.5, 280);
            }
        }
        //If they match and say blue is on the right, go for the right
        else if ((pos == RobotBaseMarsRD.BEACON_RED_BLUE || pos == 0) && (picData == BeaconOrientation.BLUE_RED_RED_BLUE || picData == BeaconOrientation.RED_BLUE_RED_BLUE)){
            if(opModeIsActive()) robotBase.turn(275 +(float)correctionAngle);
            try {
                if(opModeIsActive()) robotBase.pushButton(270 + (int)correctionAngle, 270, 2);
            }
            catch (TimeoutException e) {
                if(opModeIsActive()) robotBase.driveStraight(-20, -0.5, 270);
            }
        }
        //Now if they don't match, we trust the picture we just took
        else {
            //If it says blue is on the right, we go right
            if (pos == RobotBasePolaris.BEACON_RED_BLUE) {
                if(opModeIsActive()) robotBase.turn(275 +(float)correctionAngle);
                try {
                    if(opModeIsActive()) robotBase.pushButton(270 + (int)correctionAngle, 270, 2);
                }
                catch (TimeoutException e) {
                    if(opModeIsActive()) robotBase.driveStraight(-20, -0.5, 270);
                }
            }
            //If it says blue is on the left, we go left
            else if (pos == RobotBasePolaris.BEACON_BLUE_RED) {
                try {
                    if(opModeIsActive()) robotBase.pushButton(280 + (int)correctionAngle, 280, 2);
                }
                catch (TimeoutException e) {
                    if(opModeIsActive()) robotBase.driveStraight(-20, -0.5, 280);
                }
            }
        }

        //Turn to go hit cap ball
        if (opModeIsActive()) robotBase.turn(142);

        //go hit the ball and park on the center
        if(opModeIsActive()) robotBase.driveStraight(48, 142);
        if(opModeIsActive()) sleep(1000);

        //clean up the mess we made
        robotBase.deconstruct();
        robotBase = null;

    }
}