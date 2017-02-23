package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Mars Red", group="red")
public class MarsAutoRed extends LinearOpMode {

    RobotBaseMars robotBase;

    @Override
    public void runOpMode() throws InterruptedException {
        robotBase = new RobotBaseMars();
        robotBase.init(hardwareMap, this);
        int pos;
        double shiftedAvg;
        double deltaX;
        double correctionAngle;
        robotBase.cockShooter();
        waitForStart();

        System.out.println("SSS reloaderPos @Start: " + robotBase.reloaderServo.getPosition());

        //initial drive

        robotBase.driveStraight(23, 0);

        //turns parallel to ramp
        robotBase.turn(322);

        //second drive to align the robot to the first beacon one one axis
        robotBase.driveStraight(24, 322);

        //turns robot to face beacon
        robotBase.turn(275);

        //waits for robot to come to rest, then takes picture to determine beacon orientation
        sleep(2000);
        System.out.println("SSS heading after turn to take picture: " + robotBase.zRotation);
        pos = robotBase.takePicture();

        //do some math to determine the angle the robot should drive to the beacon with
        shiftedAvg = ((90 - robotBase.getZRotation()) * robotBase.PIXELS_PER_DEGREE) + robotBase.getLastPicBeaconAvg();
        deltaX = (340 - shiftedAvg)/robotBase.PIXELS_PER_INCH;
        correctionAngle = Math.toDegrees(Math.atan(deltaX/30.));

        //outputs beacon info for testing purposes
        System.out.println("px per degree: " + robotBase.PIXELS_PER_DEGREE);
        System.out.println("lastPicBeaconAvg: " + robotBase.getLastPicBeaconAvg());
        System.out.println("zRotation: " + robotBase.getZRotation());
        System.out.println("shiftedAvg: " + shiftedAvg);
        System.out.println("deltaX: " + deltaX);
        System.out.println("correctionAngle: " + correctionAngle);

        telemetry.addData("pos ",pos);
        telemetry.update();

        System.out.println(pos);

        if (pos == RobotBasePolaris.BEACON_RED_BLUE){
            robotBase.turn(260 +(float)correctionAngle);
            try {
                robotBase.pushButton(260 + (int)correctionAngle, 260, 2);
            }
            catch (TimeoutException e) {
                robotBase.driveStraight(-8, -0.5, 260);
            }
        }
        else if (pos == RobotBasePolaris.BEACON_BLUE_RED) {
            try {robotBase.pushButton(270 + (int)correctionAngle, 270, 2);}
            catch (TimeoutException e) {
                robotBase.driveStraight(-8, -0.5, 270);
            }
        }

        robotBase.turn(112);

        robotBase.hanShotFirst();

        robotBase.reloadShooter();

        robotBase.hanShotFirst();

        sleep(1000);

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