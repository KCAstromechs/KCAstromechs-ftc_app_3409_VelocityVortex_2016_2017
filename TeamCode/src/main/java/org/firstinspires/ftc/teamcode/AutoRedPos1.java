package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.sql.Time;

@Disabled
@Autonomous(name="BeaconAutoRedBackup", group="Backup")
public class AutoRedPos1 extends LinearOpMode {

    AstroRobotBaseInterface robotBase;

    @Override
    public void runOpMode() throws InterruptedException {
        robotBase = new RobotBasePolaris();
        robotBase.initCallingOpMode(this);
        robotBase.init(hardwareMap);
        int pos;
        double shiftedAvg;
        double deltaX;
        double correctionAngle;
        waitForStart();

        //initial drive
        robotBase.driveStraight(21, 0);

        //turns parallel to ramp
        robotBase.turn(322);

        //second drive to align the robot to the first beacon one one axis
        robotBase.driveStraight(29.6,322);

        //turns robot to face beacon
        robotBase.turn(275);

        //waits for robot to come to rest, then takes picture to determine beacon orientation
        sleep(500);
        pos = robotBase.takePicture();

        //do some math to determine the angle the robot should drive to the beacon with
        shiftedAvg = ((270 - robotBase.getZRotation()) * robotBase.PIXELS_PER_DEGREE) + robotBase.getLastPicBeaconAvg();
        deltaX = (340 - shiftedAvg)/robotBase.PIXELS_PER_INCH;
        correctionAngle = Math.toDegrees(Math.atan(deltaX/30.));

        //outputs beacon info for testing purposes
        System.out.println("Beginning first beacon");
        System.out.println(pos);
        System.out.println("px per degree: " + robotBase.PIXELS_PER_DEGREE);
        System.out.println("lastPicBeaconAvg: " + robotBase.getLastPicBeaconAvg());
        System.out.println("zRotation: " + robotBase.getZRotation());
        System.out.println("shiftedAvg: " + shiftedAvg);
        System.out.println("deltaX: " + deltaX);
        System.out.println("correctionAngle: " + correctionAngle);

        telemetry.addData("",pos);
        telemetry.update();

        //if statement drives to the correct side of the beacon depending on the beacon orientation, then drives backwards away from beacon
        if (pos == RobotBasePolaris.BEACON_RED_BLUE){
            try {
                robotBase.pushButton(270 + (int)correctionAngle, 270, 2);
            }
            catch (TimeoutException e) {
                robotBase.driveStraight(-12, -0.5, 270);
            }
        }
        else if (pos == RobotBasePolaris.BEACON_BLUE_RED) {
            robotBase.turn(280 + (float)correctionAngle);
            try {robotBase.pushButton(280 + (int)correctionAngle, 280, 2);}
            catch (TimeoutException e) {
                robotBase.driveStraight(-12, -0.5, 280);
            }
        }

        //turns to position the robot for shooting
        robotBase.turn(115);

        //takes two shots into the center vortex
        robotBase.hanShotFirst();
        robotBase.hanShotFirst();

        //begins journey to second beacon
        robotBase.turn(0);
        robotBase.driveStraight(45, 0);

        //turns to face second beacon
        robotBase.turn(275);

        //waits for robot to come to rest, then takes picture to determine beacon orientation
        sleep(500);
        pos = robotBase.takePicture();

        System.out.println("Beginning second beacon");
        System.out.println(pos);
        System.out.println(pos);
        System.out.println("px per degree: " + robotBase.PIXELS_PER_DEGREE);
        System.out.println("lastPicBeaconAvg: " + robotBase.getLastPicBeaconAvg());
        System.out.println("zRotation: " + robotBase.getZRotation());
        System.out.println("shiftedAvg: " + shiftedAvg);
        System.out.println("deltaX: " + deltaX);
        System.out.println("correctionAngle: " + correctionAngle);

        telemetry.addData("",pos);
        telemetry.update();

        //if statement drives to the correct side of the beacon depending on the beacon orientation, then drives backwards away from beacon
        if (pos == RobotBasePolaris.BEACON_RED_BLUE){
            try {robotBase.pushButton(270 + (int)correctionAngle, 270, 2);}
            catch (TimeoutException e) {
                robotBase.driveStraight(-12, -0.5, 270);
            }
        }
        else if (pos == RobotBasePolaris.BEACON_BLUE_RED) {
            robotBase.turn(288 + (float)correctionAngle);
            try {robotBase.pushButton(288 + (int)correctionAngle, 288, 2);}
            catch (TimeoutException e) {
                robotBase.driveStraight(-12, -0.5, 288);
            }
        }
        robotBase.deconstruct();
        robotBase = null;
    }

}