package org.firstinspires.ftc.teamcode;

import android.view.View;
import android.view.ViewGroup;
import android.widget.RelativeLayout;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.internal.AppUtil;

@Autonomous(name="Blue Pos 1 Long Distance", group="Blue")
public class PictureFirst_AutoBlue100 extends LinearOpMode {

    RobotBaseMarsRD robotBase;
    boolean debug = true;

    protected RelativeLayout squaresOverlay = null;
    protected AppUtil appUtil = AppUtil.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {

        robotBase = new RobotBaseMarsRD();
        robotBase.init(hardwareMap, this);
        robotBase.initVuforia();
        robotBase.setDebug(debug);

        int pos = -1;
        int beacon2;

        double shiftedAvg;
        double deltaX;
        double correctionAngle;

        //Sets up our beacon lineup "layout"
        appUtil.synchronousRunOnUiThread(new Runnable() {
            @Override
            public void run() {
                squaresOverlay = (RelativeLayout) View.inflate(appUtil.getActivity(), R.layout.beacon_line_up_squares, null);
//              squaresOverlay.findViewById(R.id.firstBeacon).setVisibility(View.VISIBLE);
                squaresOverlay.findViewById(R.id.secondBeacon).setVisibility(View.VISIBLE);
                squaresOverlay.findViewById(R.id.Origin).setVisibility(View.VISIBLE);
                appUtil.getActivity().addContentView(squaresOverlay, new RelativeLayout.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.MATCH_PARENT));
            }
        });


        while(!robotBase.isCocked()){
            robotBase.shooterHandler(false, true);
        }

        robotBase.shooterHandler(false, false);

        if(debug)
            System.out.println("SSS left isCocked");

        //All of that is set-up doing things like setting up the robotBase, cocking the shooter, setting up variables

        telemetry.addData(">", "Robot Ready.");
        telemetry.update();

        waitForStart();

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

        beacon2 = robotBase.takeLongDistancePicture();

        if(debug)
            System.out.println("second beacon: " + beacon2);

        //BLUE_RED == 2
        //RED_BLUE == 1


        //initial drive out to first beacon
        if(opModeIsActive()) robotBase.driveStraight(48, 19 - robotBase.adjustmentAngle);

        //turns robot to face beacon
        if(opModeIsActive()) robotBase.turn(85 - robotBase.adjustmentAngle);

        //waits for robot to come to rest, then takes picture to determine beacon orientation
        if(opModeIsActive()) sleep(500);

        if (debug)
            System.out.println("SSS heading after turn to take picture: " + robotBase.zRotation);

        if(opModeIsActive()) pos = robotBase.takePicture();

        //do some math to determine the angle the robot should drive to the beacon with
        shiftedAvg = ((90 - robotBase.getZRotation() - robotBase.adjustmentAngle) * robotBase.PIXELS_PER_DEGREE) + robotBase.getLastPicBeaconAvg();
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
                if(opModeIsActive()) robotBase.pushButton(90 - robotBase.adjustmentAngle + (int)correctionAngle, 90 - robotBase.adjustmentAngle, 2);
            }
            catch (TimeoutException e) {
                if(opModeIsActive()) robotBase.driveStraight(-20, -0.5, 90 - RobotBaseMarsRD.adjustmentAngle);

            }
        }
        //If they match and say blue is on the right, go for the right
        else if (pos == RobotBaseMarsRD.BEACON_RED_BLUE){
            if(opModeIsActive()) robotBase.turn(105 - RobotBaseMarsRD.adjustmentAngle +(float)correctionAngle);
            try {
                if(opModeIsActive()) robotBase.pushButton(105 - RobotBaseMarsRD.adjustmentAngle+ (int)correctionAngle, 100 - RobotBaseMarsRD.adjustmentAngle, 2);
            }
            catch (TimeoutException e) {
                if(opModeIsActive()) robotBase.driveStraight(-20, -0.5, 100 - RobotBaseMarsRD.adjustmentAngle);
            }
        }
        //Now if they don't match, we trust the picture we just took
        else if (pos == RobotBasePolaris.BEACON_BLUE_RED) {
            try {
                if (opModeIsActive()) robotBase.pushButton(90 - RobotBaseMarsRD.adjustmentAngle + (int) correctionAngle, 90 - RobotBaseMarsRD.adjustmentAngle, 2);
            } catch (TimeoutException e) {
                if (opModeIsActive()) robotBase.driveStraight(-20, -0.5, 90 - RobotBaseMarsRD.adjustmentAngle);
            }
        }

        //turn around to shoot into the center vortex
        if(opModeIsActive()) robotBase.turn(299 - RobotBaseMarsRD.adjustmentAngle);

        //shoot into center vortex
        while (opModeIsActive() && robotBase.shooterHandler(true, false));
        while (opModeIsActive() && robotBase.reloadHandler(true));
        while (opModeIsActive() && robotBase.shooterHandler(true, false));

        //Turn to drive to the center beacon
        if (opModeIsActive()) robotBase.turn(0 - RobotBaseMarsRD.adjustmentAngle);

        //Determine how far to drive depending on the orientations of both beacons.
        if (beacon2 == RobotBaseMarsRD.BEACON_RED_BLUE && pos == RobotBaseMarsRD.BEACON_BLUE_RED){
            if (opModeIsActive()) robotBase.driveStraight(34, 0 - RobotBaseMarsRD.adjustmentAngle);
            System.out.println("SSS Short drive between beacons");
        } else if ((beacon2 == RobotBaseMarsRD.BEACON_BLUE_RED && pos == RobotBaseMarsRD.BEACON_BLUE_RED) || (beacon2 == RobotBaseMarsRD.BEACON_RED_BLUE && pos == RobotBaseMarsRD.BEACON_RED_BLUE)){
            if (opModeIsActive()) robotBase.driveStraight(40, 0 - RobotBaseMarsRD.adjustmentAngle);
            System.out.println("SSS Medium drive between beacons");
        } else if (beacon2 == RobotBaseMarsRD.BEACON_BLUE_RED && pos == RobotBaseMarsRD.BEACON_RED_BLUE){
            if (opModeIsActive()) robotBase.driveStraight(46, 0 - RobotBaseMarsRD.adjustmentAngle);
            System.out.println("SSS Long drive between beacons");
        }

        //turn to face second beacon
        if (opModeIsActive()) robotBase.turn(85 - RobotBaseMarsRD.adjustmentAngle);

        //waits for robot to come to rest, then takes picture to determine beacon orientation
        if(opModeIsActive()) sleep(500);
        if (debug)
            System.out.println("SSS heading after turn to take picture: " + robotBase.zRotation);
        if(opModeIsActive()) pos = robotBase.takePicture();

        //do some math to determine the angle the robot should drive to the beacon with
        shiftedAvg = ((90 - robotBase.getZRotation() - robotBase.adjustmentAngle) * robotBase.PIXELS_PER_DEGREE) + robotBase.getLastPicBeaconAvg();
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
        if ((pos == RobotBaseMarsRD.BEACON_BLUE_RED || pos == 0) && (beacon2 == RobotBaseMarsRD.BEACON_BLUE_RED)){
            try {
                if(opModeIsActive()) robotBase.pushButton(90 - RobotBaseMarsRD.adjustmentAngle + (int)correctionAngle, 90 - RobotBaseMarsRD.adjustmentAngle, 2);
            }
            catch (TimeoutException e) {
                if(opModeIsActive()) robotBase.driveStraight(-20, -0.5, 90 - RobotBaseMarsRD.adjustmentAngle);
            }
        }
        //If they match and say blue is on the right, go for the right
        else if ((pos == RobotBaseMarsRD.BEACON_RED_BLUE || pos == 0) && (beacon2 == RobotBaseMarsRD.BEACON_RED_BLUE)){
            if(opModeIsActive()) robotBase.turn(105 - RobotBaseMarsRD.adjustmentAngle +(float)correctionAngle);
            try {
                if(opModeIsActive()) robotBase.pushButton(105 - RobotBaseMarsRD.adjustmentAngle + (int)correctionAngle, 100 - RobotBaseMarsRD.adjustmentAngle, 2);
            }
            catch (TimeoutException e) {
                if(opModeIsActive()) robotBase.driveStraight(-20, -0.5, 100 - RobotBaseMarsRD.adjustmentAngle);
            }
        }
        //Now if they don't match, we trust the picture we just took
        else {
            //If it says blue is on the right, we go right
            if (pos == RobotBasePolaris.BEACON_RED_BLUE) {
                if (opModeIsActive()) robotBase.turn(105 - RobotBaseMarsRD.adjustmentAngle + (float) correctionAngle);
                try {
                    if (opModeIsActive()) robotBase.pushButton(105 - RobotBaseMarsRD.adjustmentAngle + (int) correctionAngle, 100 - RobotBaseMarsRD.adjustmentAngle, 2);
                } catch (TimeoutException e) {
                    if (opModeIsActive()) robotBase.driveStraight(-20, -0.5, 100 - RobotBaseMarsRD.adjustmentAngle);
                }
            }
            //If it says blue is on the left, we go left
            else if (pos == RobotBasePolaris.BEACON_BLUE_RED) {
                try {
                    if (opModeIsActive()) robotBase.pushButton(90 - RobotBaseMarsRD.adjustmentAngle + (int) correctionAngle, 90 - RobotBaseMarsRD.adjustmentAngle, 2);
                } catch (TimeoutException e) {
                    if (opModeIsActive()) robotBase.driveStraight(-20, -0.5, 90 - RobotBaseMarsRD.adjustmentAngle);
                }
            }
        }

        //Turn to go hit cap ball
        if (opModeIsActive()) robotBase.turn(218 - RobotBaseMarsRD.adjustmentAngle);

        //go hit the ball and park on the center
        if(opModeIsActive()) robotBase.driveStraight(48, 218 - RobotBaseMarsRD.adjustmentAngle);

        if(opModeIsActive()) sleep(1000);

        //clean up the mess we made
        robotBase.deconstruct();
        robotBase = null;

    }
}