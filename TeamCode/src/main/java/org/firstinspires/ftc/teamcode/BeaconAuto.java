package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

@Autonomous(name="BeaconAuto", group="Test")
public class BeaconAuto extends LinearOpMode {

    AstroRobotBaseInterface robotBase;
    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() throws InterruptedException {

        robotBase = new RobotBasePolaris();

        robotBase.initCallingOpMode(this);

        robotBase.init (hardwareMap);

        waitForStart();

        robotBase.driveStraight(-12, -0.5, 0);

        //TODO determine drive backwards

// robotBase.driveStraight(3, 90);

/*
        for (int x = 0; x <20; x++){
            robotBase.outputZAxis();
            sleep(200);
        }

        System.out.println("Resetting gyro!!");

        robotBase.resetGyro();

       while(opModeIsActive()){
            robotBase.outputZAxis();
            sleep(200);
        }
        /*
        robotBase.resetGyro();

        robotBase.driveStraight(8, 0);
        robotBase.outputZAxis();

        robotBase.turn(30, 0.5);
        robotBase.driveStraight(53, 45);

        robotBase.turn(80, 0.5);
        sleep(1000);

        robotBase.takePicture(); //analyzes the beacon
*/
    }


}
