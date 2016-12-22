package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.nio.ByteBuffer;

@Autonomous(name="BeaconAuto", group="Test")
public class BeaconAuto extends LinearOpMode {

    PrototypeRobotBaseInterface robotBase;
    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() throws InterruptedException {

        robotBase = new RobotBasePrototype (this);

        robotBase.init (hardwareMap);

        waitForStart();

        robotBase.turn(90);
        robotBase.turn(-90);
        robotBase.turn(0);
        robotBase.turn(180);




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
