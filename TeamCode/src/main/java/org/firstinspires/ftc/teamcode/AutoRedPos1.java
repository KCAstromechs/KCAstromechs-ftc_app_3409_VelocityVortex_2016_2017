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
        //initial drive out from wall
        robotBase.driveStraight(33, 0);
        robotBase.turn(310);
        robotBase.driveStraight(27.2, 310);

        //turning to beacon
        robotBase.turn(270);

        //Determines which button to push
        pos = robotBase.takePicture();

        System.out.println(pos);

        //pushes chosen button
        if (pos == 1) {
        }
        else if (pos == 2) {
        }

        //turn out of 1st beacon
        robotBase.turn(0);
        //drive to get in position 2nd beacon
        robotBase.driveStraight(48,0);
        //turn to 2nd beacon
        robotBase.turn(270);
        sleep(1000);
        //determine which button to push
        pos = robotBase.takePicture();

        System.out.println(pos);

        //push chosen button
        if (pos == 1) {
        }
        else if (pos == 2) {
        }

        //turn to middle, get in position to shoot
        robotBase.turn(140);

    }

}