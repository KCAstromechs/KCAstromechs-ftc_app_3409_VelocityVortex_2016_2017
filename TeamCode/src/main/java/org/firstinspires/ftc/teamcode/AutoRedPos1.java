package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Red Pos 1", group="Red")
public class AutoRedPos1 extends LinearOpMode {

    AstroRobotBaseInterface robotBase;

    @Override
    public void runOpMode() throws InterruptedException {
        robotBase = new RobotBasePolaris(this);
        robotBase.init(hardwareMap);
        int pos;
        waitForStart();

        robotBase.driveStraight(31, 0);
        robotBase.turn(310);
        robotBase.driveStraight(20, 310);

        robotBase.turn(270);

        pos = robotBase.takePicture();

        System.out.println(pos);

        if (pos == 2){
            robotBase.turn(280);
            robotBase.pushButton(280);
        }
        else if (pos == 1) {
            robotBase.pushButton(270);
        }

        robotBase.turn(180);
        robotBase.turn(90);
        if(opModeIsActive()){
            sleep(4000);
        }
        robotBase.turn(0);
        robotBase.driveStraight(44, 0);

        robotBase.turn(270);
        pos = robotBase.takePicture();

        System.out.println(pos);

        //push chosen button
        if (pos == 2){
            robotBase.turn(280);
            robotBase.pushButton(280);
        }
        else if (pos == 1) {
            robotBase.pushButton(270);
        }

        robotBase.deconstruct();
        robotBase = null;
    }

}