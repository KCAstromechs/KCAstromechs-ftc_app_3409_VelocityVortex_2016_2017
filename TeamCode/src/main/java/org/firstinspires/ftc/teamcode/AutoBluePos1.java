package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Blue Pos 1", group="Blue")
public class AutoBluePos1 extends LinearOpMode {

    RobotBaseMars robotBase;

    @Override
    public void runOpMode() throws InterruptedException {
        robotBase = new RobotBaseMars();
        robotBase.init(hardwareMap, this);
        int pos;
        double shiftedAvg;
        double deltaX;
        double correctionAngle;
        waitForStart();

        //initial drive
        robotBase.driveStraight(5, 0);

        robotBase.turn(10);

        sleep(2000);

        robotBase.turn(0);

        robotBase.driveStraight(15, 0);

        //turns parallel to ramp
        robotBase.turn(38);

        //second drive to align the robot to the first beacon one one axis
        robotBase.driveStraight(24, 38);

        //turns robot to face beacon
        robotBase.turn(90);

        //waits for robot to come to rest, then takes picture to determine beacon orientation
        sleep(2000);
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

        telemetry.addData("",pos);
        telemetry.update();

        System.out.println(pos);

        if (pos == RobotBasePolaris.BEACON_RED_BLUE){
            robotBase.turn(100 +(float)correctionAngle);
            try {
                robotBase.pushButton(100 + (int)correctionAngle, 100, 2);
            }
            catch (TimeoutException e) {
                robotBase.driveStraight(-30, -0.5, 100);
            }
        }
        else if (pos == RobotBasePolaris.BEACON_BLUE_RED) {
            try {robotBase.pushButton(90 + (int)correctionAngle, 90, 2);}
            catch (TimeoutException e) {
                robotBase.driveStraight(-30, -0.5, 90);
            }
        }
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
    }

}