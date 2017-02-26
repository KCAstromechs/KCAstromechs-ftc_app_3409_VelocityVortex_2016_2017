package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name="MarsTeleopRD", group="Teleop")
public class MarsTeleopRD extends OpMode{

    RobotBaseMarsRD robotBase;
    boolean released = true;
    boolean bReleased = false;
    boolean shotTaken = false;
    boolean reloading = false;
    boolean shotSequence = false;

    boolean manualReloadIsRunning = false;
    boolean manualReloadIsFinishing = false;

    @Override
    public void init(){
        robotBase = new RobotBaseMarsRD();
        robotBase.init(hardwareMap, this);
    }

    @Override
    public void loop(){
        //robotBase.lifterHandler(gamepad2.left_stick_y, gamepad2.right_stick_y);

        if (gamepad1.left_bumper || gamepad2.left_bumper) shotSequence = true;

        manualReloadIsRunning = shotSequence && !robotBase.shooterHandler(gamepad1.left_bumper || gamepad2.left_bumper, gamepad1.left_trigger > 0.2
                || gamepad2.left_trigger > 0.2);

        //manual reload
        if(gamepad2.right_bumper) {
            manualReloadIsRunning = true;
        }
        if(!robotBase.reloadHandler(manualReloadIsRunning) && robotBase.reloadJustFinished) {
            manualReloadIsRunning = false;
            manualReloadIsFinishing = true;
        }
        if(manualReloadIsFinishing) {
            robotBase.reloadJustFinished = false;
            shotSequence = false;
            manualReloadIsFinishing = false;
        }


        robotBase.reloadHandler(gamepad2.right_bumper);

        if(gamepad1.b || gamepad2.b) {
            robotBase.setMotorSpinner(-1.0);
            bReleased = true;
        } else if ((!gamepad1.b || gamepad2.b) && bReleased) {
            robotBase.setMotorSpinner(0);
            bReleased = false;
        } else if (!robotBase.spinnerIsRunning()) {
            if ((gamepad1.a || gamepad2.a) && released) {
                robotBase.setMotorSpinner(1.0);
                released = false;
            } else if (!gamepad1.a) {
                released = true;
            }
        } else {
            if ((gamepad1.a || gamepad2.a) && released) {
                robotBase.setMotorSpinner(0);
                released = false;
            } else if (!gamepad1.a) {
                released = true;
            }
        }


        robotBase.updateDriveMotors(gamepad1.left_stick_y, gamepad1.right_stick_y, gamepad1.right_bumper);
    }
}

