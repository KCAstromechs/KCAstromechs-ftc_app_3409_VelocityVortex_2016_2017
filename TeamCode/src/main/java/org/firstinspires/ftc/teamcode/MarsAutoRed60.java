package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Mars Red 60", group="Red")
public class MarsAutoRed60 extends LinearOpMode {
    
    RobotBaseMarsRD robotBase;                                      //Pull in the robotBase for our use
    boolean debug = false;                                          //Turn debug mode on or off (won't sout if off)

    @Override
    public void runOpMode() throws InterruptedException {
        robotBase = new RobotBaseMarsRD();                          //Finish pulling in robotBase
        robotBase.init(hardwareMap, this);                          //Call init to est. hardware map and sensors
        robotBase.initVuforia();                                    //Open vuforia
        robotBase.setDebug(debug);
        
        int pos = -1;                                               //beacon position to 'null'
        
        //Variables for angle analysis before turns
        double shiftedAvg;
        double deltaX;
        double correctionAngle;

        //If the shooter isn't cocked, cock it
        while (!robotBase.isCocked()) {
            robotBase.shooterHandler(false, true);
        }

        //If we want debug mode, print info
        if (debug)
            System.out.println("SSS left isCocked");
        
        robotBase.shooterHandler(false, false);                     //Make sure the shooter isn't moving

        waitForStart();

        if (debug)
            System.out.println("SSS reloaderPos @Start: " + robotBase.reloaderServo.getPosition());

        if (opModeIsActive()) robotBase.driveStraight(25, 0);       //initial drive out towards center vortex

        if (opModeIsActive()) robotBase.turn(322);                  //turns to be parallel to corner vortex
        
        if (opModeIsActive()) robotBase.driveStraight(24, 322);     //second drive to set robot in front of beacon
        
        if (opModeIsActive()) robotBase.turn(275);                  //turns robot to face beacon

        //waits for robot to come to rest, then takes picture to determine beacon orientation
        if (opModeIsActive()) sleep(500);
        if (debug)
            System.out.println("SSS heading after turn to take picture: " + robotBase.zRotation);
        if (opModeIsActive()) pos = robotBase.takePicture();

        //do some math to determine how to modify the angle the robot should drive to the beacon with, based on where the robot has determined it is in relation to the beacon
        shiftedAvg = ((270 - robotBase.getZRotation()) * robotBase.PIXELS_PER_DEGREE) + robotBase.getLastPicBeaconAvg();
        deltaX = (340 - shiftedAvg) / robotBase.PIXELS_PER_INCH;
        correctionAngle = Math.toDegrees(Math.atan(deltaX / 30.));

        //if debug, output beacon info
        if (debug) {
            System.out.println("px per degree: " + robotBase.PIXELS_PER_DEGREE);
            System.out.println("lastPicBeaconAvg: " + robotBase.getLastPicBeaconAvg());
            System.out.println("zRotation: " + robotBase.getZRotation());
            System.out.println("shiftedAvg: " + shiftedAvg);
            System.out.println("deltaX: " + deltaX);
            System.out.println("correctionAngle: " + correctionAngle);
            System.out.println(pos);
        }
        
        //Tell user which position the robot believes the beacon to be in
        telemetry.addData("pos ", pos);
        telemetry.update();

        //For either position, we turn to face angle we'll drive at, & try to push the button. If you time out, turn back. 
        if (pos == RobotBasePolaris.BEACON_RED_BLUE) {
            if (opModeIsActive()) robotBase.turn(270 + (float) correctionAngle);
            try {
                if (opModeIsActive()) robotBase.pushButton(270 + (int) correctionAngle, 270, 2);
            } catch (TimeoutException e) {
                if (opModeIsActive()) robotBase.driveStraight(-20, -0.5, 270);
            }
        } else if (pos == RobotBasePolaris.BEACON_BLUE_RED) {
            try {
                if (opModeIsActive()) robotBase.pushButton(280 + (int) correctionAngle, 280, 2);
            } catch (TimeoutException e) {
                if (opModeIsActive()) robotBase.driveStraight(-20, -0.5, 280);
            }
        }

        if (opModeIsActive()) robotBase.turn(122);                  //Turn the robot to the angle to shoot into the center vortex

        //Shoot, then reload sequence, then shoot again
        while (opModeIsActive() && robotBase.shooterHandler(true, false)) ;
        while (opModeIsActive() && robotBase.reloadHandler(true)) ;
        while (opModeIsActive() && robotBase.shooterHandler(true, false)) ;

        //Wait a hot sec, then turn everything off.
        if(opModeIsActive()) sleep(1000);
        robotBase.deconstruct();
        robotBase = null;
    }
}
