package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.sql.Time;

@Autonomous(name="BeaconAutoRed", group="Red")
public class AutoRedPos1 extends LinearOpMode {

    AstroRobotBaseInterface robotBase;

    @Override
    public void runOpMode() throws InterruptedException {
        robotBase = new RobotBasePolaris();
        robotBase.initCallingOpMode(this);
        robotBase.init(hardwareMap);
        int pos;
        waitForStart();
        robotBase.driveStraight(31, 0);
        robotBase.turn(310);
        robotBase.driveStraight(20, 310);

        robotBase.turn(270);

        pos = robotBase.takePicture();

        System.out.println(pos);

        if (pos == 1){
            try {
                robotBase.pushButton(270, 2);
            }
            catch (TimeoutException e) {
                robotBase.driveStraight(-12, -0.5, 270);
            }
        }
        else if (pos == 2) {
            robotBase.turn(280);
            try {robotBase.pushButton(280, 2);}
            catch (TimeoutException e) {
                robotBase.driveStraight(-12, -0.5, 280);
            }
        }

        robotBase.turn(90);
        //TODO: test shooting
        robotBase.hanShotFirst();
        robotBase.hanShotFirst();
        robotBase.turn(0);
        robotBase.driveStraight(45, 0);

        robotBase.turn(270);
        pos = robotBase.takePicture();

        System.out.println(pos);

        //push chosen button
        if (pos == 1){
            try {robotBase.pushButton(270, 2);}
            catch (TimeoutException e) {
                robotBase.driveStraight(-12, -0.5, 270);
            }
        }
        else if (pos == 2) {
            robotBase.turn(288);
            try {robotBase.pushButton(288, 2);}
            catch (TimeoutException e) {
                robotBase.driveStraight(-12, -0.5, 288);
            }
        }

        robotBase.deconstruct();
        robotBase = null;
    }

}