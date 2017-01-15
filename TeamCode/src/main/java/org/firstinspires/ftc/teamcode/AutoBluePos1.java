package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Blue Pos 1", group="Blue")
public class AutoBluePos1 extends LinearOpMode {

    AstroRobotBaseInterface robotBase;

    @Override
    public void runOpMode() throws InterruptedException {
        robotBase = new RobotBasePolaris();
        robotBase.initCallingOpMode(this);
        robotBase.init(hardwareMap);
        int pos;
        waitForStart();
        robotBase.driveStraight(31, 0);
        robotBase.turn(50);
        robotBase.driveStraight(20, 50);

        robotBase.turn(90);

        pos = robotBase.takePicture();

        System.out.println(pos);

        if (pos == 1){
            robotBase.turn(100);
            try {
                robotBase.pushButton(100, 2);
            }
            catch (TimeoutException e) {
                robotBase.driveStraight(-12, -0.5, 100);
            }
        }
        else if (pos == 2) {
            try {robotBase.pushButton(90, 2);}
            catch (TimeoutException e) {
                robotBase.driveStraight(-12, -0.5, 90);
            }
        }

        robotBase.turn(145);
        robotBase.turn(270);
        if(opModeIsActive()){
            sleep(4000);
        }
        robotBase.turn(0);
        robotBase.driveStraight(44, 0);

        robotBase.turn(90);
        pos = robotBase.takePicture();

        System.out.println(pos);

        //push chosen button
        if (pos == 1){
            robotBase.turn(100);
            try {robotBase.pushButton(100, 2);}
            catch (TimeoutException e) {
                robotBase.driveStraight(-12, -0.5, 100);
            }
        }
        else if (pos == 2) {
            try {robotBase.pushButton(90, 2);}
            catch (TimeoutException e) {
                robotBase.driveStraight(-12, -0.5, 90);
            }
        }

        robotBase.deconstruct();
        robotBase = null;
    }

}