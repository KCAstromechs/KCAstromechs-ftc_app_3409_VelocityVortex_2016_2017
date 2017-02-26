package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp (name="MarsTeleopRD", group="Teleop")
public class MarsTeleopRD extends OpMode{

    RobotBaseMarsRD robotBase;
    boolean released = true;
    boolean shootIsResetting = false;
    boolean shootButtonIsReleased = true;

    @Override
    public void init(){
        robotBase = new RobotBaseMarsRD();
        robotBase.init(hardwareMap, this);
    }

    @Override
    public void loop(){
        //robotBase.lifterHandler(gamepad2.left_stick_y, gamepad2.right_stick_y);

        robotBase.shooterHandler(gamepad1.left_bumper, gamepad1.y || gamepad2.y);

        robotBase.reloadHandler(gamepad2.right_bumper);

        if(gamepad1.b) {
            robotBase.setMotorSpinner(0);
        }
        if(!robotBase.spinnerIsRunning()){
            if (gamepad1.a && released) {
                robotBase.setMotorSpinner(1.0);
                released = false;
            } else if (!gamepad1.a){
                released = true;
            }
        } else {
            if (gamepad1.a && released){
                robotBase.setMotorSpinner(0);
                released = false;
            } else if (!gamepad1.a){
                released = true;
            }
        }


        robotBase.updateDriveMotors(gamepad1.left_stick_y, gamepad1.right_stick_y, gamepad1.right_bumper);
    }
}

