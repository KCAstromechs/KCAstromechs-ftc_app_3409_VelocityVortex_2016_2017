package org.firstinspires.ftc.teamcode;

import android.view.View;
import android.view.ViewGroup;
import android.widget.RelativeLayout;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.internal.AppUtil;

@Autonomous(name="Blue 100", group="Blue")
public class NovaAutoBlue100 extends LinearOpMode {

    RobotBaseMarsRD robotBase;
    boolean debug = false;
    boolean shortTurn;

    protected RelativeLayout squaresOverlay = null;
    protected AppUtil appUtil = AppUtil.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {

        robotBase = new RobotBaseMarsRD();
        robotBase.init(hardwareMap, this);
        robotBase.initVuforia();
        robotBase.setDebug(debug);

        int pos = -1;
        int beacon2 = -1;

        final int adjustmentAngle = 19;           //Adjust the angle on everything by this

        double shiftedAvg;
        double deltaX;
        double correctionAngle;

        //Sets up our beacon lineup "layout"
        appUtil.synchronousRunOnUiThread(new Runnable() {
            @Override
            public void run() {
                squaresOverlay = (RelativeLayout) View.inflate(appUtil.getActivity(), R.layout.beacon_line_up_squares, null);
                squaresOverlay.findViewById(R.id.blueSideBeacon).setVisibility(View.VISIBLE);
                squaresOverlay.findViewById(R.id.Origin).setVisibility(View.VISIBLE);
                appUtil.getActivity().addContentView(squaresOverlay, new RelativeLayout.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.MATCH_PARENT));
            }
        });

        //While shooter touch sensor isn't activated, continue cocking shooter
        while(!robotBase.isCocked()){
            robotBase.shooterHandler(false, true);
        }
        //After shooter cocks, stop motor
        robotBase.shooterHandler(false, false);

        if(debug)
            System.out.println("SSS left isCocked");

        //All of that is set-up doing things like setting up the robotBase, cocking the shooter, setting up variables

        telemetry.addData(">", "Robot Ready.");
        telemetry.update();

        waitForStart();

        //sets flag that tells robotbase to re-zero gyro.
        robotBase.hasBeenZeroed = false;

        System.out.println("SSS zRotation after start " + robotBase.getZRotation());

        if (debug)
            System.out.println("SSS reloaderPos @Start: " + robotBase.reloaderServo.getPosition());

        //Terminates "layout" for beacon lineup
        appUtil.synchronousRunOnUiThread(new Runnable() {
            @Override
            public void run() {
                if (squaresOverlay != null){
                    ((ViewGroup)squaresOverlay.getParent()).removeView(squaresOverlay);
                }
                squaresOverlay = null;
            }
        });

        if(getCurrentAutoType() != AutoType.OpMode60) {
            if (opModeIsActive()) beacon2 = robotBase.takeLongDistancePicture(66, 650);
            if(debug)
                System.out.println("second beacon: " + beacon2);
        }

        //initial drive out to first beacon
        if(opModeIsActive()) robotBase.driveStraight(48, 19 - adjustmentAngle);

        //turns robot to face beacon
        if(opModeIsActive()) robotBase.turn(85 - adjustmentAngle);

        //waits for robot to come to rest, then takes picture to determine beacon orientation
        if(opModeIsActive()) sleep(500);

        if (debug)
            System.out.println("SSS heading after turn to take picture: " + robotBase.zRotation);

        if(opModeIsActive()) pos = robotBase.takePicture();

        //do some math to determine the angle the robot should drive to the beacon with
        shiftedAvg = ((90 - robotBase.getZRotation() - adjustmentAngle) * robotBase.PIXELS_PER_DEGREE) + robotBase.getLastPicBeaconAvg();
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
        if (pos == RobotBaseMarsRD.BEACON_BLUE_RED){
            try {
                if(opModeIsActive()) robotBase.pushButton(90 - adjustmentAngle + (int)correctionAngle, 90 - adjustmentAngle, 2);
            }
            catch (TimeoutException e) {
                if(opModeIsActive()) robotBase.driveStraight(-20, -0.5, 90 - adjustmentAngle);

            }
        }
        //If they match and say blue is on the right, go for the right
        else if (pos == RobotBaseMarsRD.BEACON_RED_BLUE){
            if(opModeIsActive()) robotBase.turn(100 - adjustmentAngle +(float)correctionAngle);
            try {
                if(opModeIsActive()) robotBase.pushButton(100 - adjustmentAngle+ (int)correctionAngle, 100 - adjustmentAngle, 2);
            }
            catch (TimeoutException e) {
                if(opModeIsActive()) robotBase.driveStraight(-20, -0.5, 100 - adjustmentAngle);
            }
        }

        //turn around to shoot into the center vortex
        if(opModeIsActive()) robotBase.turn(299 - adjustmentAngle);

        //shoot into center vortex
        while (opModeIsActive() && robotBase.shooterHandler(true, false));
        while (opModeIsActive() && robotBase.reloadHandler(true));
        while (opModeIsActive() && robotBase.shooterHandler(true, false));

        if (getCurrentAutoType() == AutoType.OpMode65Ramp) {

            if (opModeIsActive()) robotBase.turn(155);
            if (opModeIsActive()) robotBase.driveStraight(60, 155);

        }

        if(getCurrentAutoType() != AutoType.OpMode60 || getCurrentAutoType() != AutoType.OpMode65Ramp) {

            //Turn to drive to the second beacon
            if (opModeIsActive()) robotBase.turn(0 - adjustmentAngle);

            //Determine how far to drive depending on the orientations of both beacons, turn based on how far you drove
            if (beacon2 == RobotBaseMarsRD.BEACON_RED_BLUE && pos == RobotBaseMarsRD.BEACON_BLUE_RED) {
                if (opModeIsActive()) robotBase.driveStraight(32, 0 - adjustmentAngle);
                if (opModeIsActive()) robotBase.turn(75 - adjustmentAngle);
                shortTurn = false;
                System.out.println("SSS Short drive Red-Blue Blue-Red");
            } else if ((beacon2 == RobotBaseMarsRD.BEACON_BLUE_RED && pos == RobotBaseMarsRD.BEACON_BLUE_RED)) {
                if (opModeIsActive()) robotBase.driveStraight(43, 0 - adjustmentAngle);
                if (opModeIsActive()) robotBase.turn(85 - adjustmentAngle);
                shortTurn = true;
                System.out.println("SSS Medium drive Blue-Red Blue-Red");
            } else if (beacon2 == RobotBaseMarsRD.BEACON_RED_BLUE && pos == RobotBaseMarsRD.BEACON_RED_BLUE) {
                if (opModeIsActive()) robotBase.driveStraight(36, 0 - adjustmentAngle);
                if (opModeIsActive()) robotBase.turn(75 - adjustmentAngle);
                shortTurn = false;
                System.out.println("SSS Medium drive Red-Blue Red-Blue");
            } else if (beacon2 == RobotBaseMarsRD.BEACON_BLUE_RED && pos == RobotBaseMarsRD.BEACON_RED_BLUE) {
                if (opModeIsActive()) robotBase.driveStraight(46, 0 - adjustmentAngle);
                if (opModeIsActive()) robotBase.turn(85 - adjustmentAngle);
                shortTurn = true;
                System.out.println("SSS Long drive Blue-Red Red-Blue");
            } else {
                if (opModeIsActive()) robotBase.driveStraight(38, 0 - adjustmentAngle);
                if (opModeIsActive()) robotBase.turn(85 - adjustmentAngle);
                shortTurn = true;
                System.out.println("SSS Failsafe drive");
            }

            //distance from BLUE_RED endpoint to line = 40 inches

            //waits for robot to come to rest, then takes picture to determine beacon orientation
            if (opModeIsActive()) sleep(500);
            if (debug)
                System.out.println("SSS heading after turn to take picture: " + robotBase.zRotation);
            if (opModeIsActive()) pos = robotBase.takePicture();

            //if we can't see the beacon, let's reposition incase we overshot
            if (pos == 0) {
                if (shortTurn) {
                    if (opModeIsActive()) robotBase.turn(85 - adjustmentAngle);
                    if (debug)
                        System.out.println("Long turn re-turn");
                } else {
                    if (opModeIsActive()) robotBase.turn(75 - adjustmentAngle);
                    if (debug)
                        System.out.println("Short turn re-turn");
                }
            }

            if (opModeIsActive()) sleep(250);
            if (opModeIsActive()) pos = robotBase.takePicture();

            //do some math to determine the angle the robot should drive to the beacon with
            shiftedAvg = ((90 - robotBase.getZRotation() - adjustmentAngle) * robotBase.PIXELS_PER_DEGREE) + robotBase.getLastPicBeaconAvg();
            deltaX = (340 - shiftedAvg) / robotBase.PIXELS_PER_INCH;
            correctionAngle = Math.toDegrees(Math.atan(deltaX / 30.));

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
            if ((pos == RobotBaseMarsRD.BEACON_BLUE_RED) && (beacon2 == RobotBaseMarsRD.BEACON_BLUE_RED)) {
                try {
                    if (opModeIsActive())
                        robotBase.pushButton(90 - adjustmentAngle + (int) correctionAngle, 90 - adjustmentAngle, 2);
                }
                catch (TimeoutException e) {
                    if (opModeIsActive()) robotBase.driveStraight(-20, -0.5, 90 - adjustmentAngle);
                }
            }
            //If they match and say blue is on the right, go for the right
            else if ((pos == RobotBaseMarsRD.BEACON_RED_BLUE) && (beacon2 == RobotBaseMarsRD.BEACON_RED_BLUE)) {
                if (opModeIsActive())
                    robotBase.turn(105 - adjustmentAngle + (float) correctionAngle);
                try {
                    if (opModeIsActive())
                        robotBase.pushButton(105 - adjustmentAngle + (int) correctionAngle, 100 - adjustmentAngle, 2);
                }
                catch (TimeoutException e) {
                    if (opModeIsActive()) robotBase.driveStraight(-20, -0.5, 100 - adjustmentAngle);
                }
            }
            //Now if they don't match, we trust the picture we just took
            else {
                //If it says blue is on the right, we go right
                if (pos == RobotBasePolaris.BEACON_RED_BLUE) {
                    if (opModeIsActive())
                        robotBase.turn(105 - adjustmentAngle + (float) correctionAngle);
                    try {
                        if (opModeIsActive())
                            robotBase.pushButton(105 - adjustmentAngle + (int) correctionAngle, 100 - adjustmentAngle, 2);
                    }
                    catch (TimeoutException e) {
                        if (opModeIsActive())
                            robotBase.driveStraight(-20, -0.5, 100 - adjustmentAngle);
                    }
                }
                //If it says blue is on the left, we go left
                else if (pos == RobotBasePolaris.BEACON_BLUE_RED) {
                    try {
                        if (opModeIsActive())
                            robotBase.pushButton(90 - adjustmentAngle + (int) correctionAngle, 90 - adjustmentAngle, 2);
                    }
                    catch (TimeoutException e) {
                        if (opModeIsActive())
                            robotBase.driveStraight(-20, -0.5, 90 - adjustmentAngle);
                    }
                }
            }

            if (getCurrentAutoType() == AutoType.OpMode95Ramp) {
                if (opModeIsActive()) robotBase.turn(180);
                if (opModeIsActive()) robotBase.driveStraight(70, 180);
            }

            if (getCurrentAutoType() == AutoType.OpMode100) {
                //Turn to go hit cap ball
                if (opModeIsActive()) robotBase.turn(218 - adjustmentAngle);

                //go hit the ball and park on the center
                if (opModeIsActive()) robotBase.driveStraight(48, 218 - adjustmentAngle);
            }
        }
        //clean up the mess we made
        robotBase.deconstruct();
        robotBase = null;

    }

    //sets flag that indicates what version of autonomous we want to run, rest of code is from superclass
    protected AutoType getCurrentAutoType() {
        return AutoType.OpMode100;
    }

    //Declares options for OpModes for Blue side
    protected enum AutoType {
        OpMode65Ramp,
        OpMode95Ramp,
        OpMode100,
        OpMode90,
        OpMode60;
    }

}