package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name="TeleOp", group="TeleOp")
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
        robotBase.setReloadAfterShot(true);
    }

    @Override
    public void loop(){
        //Will uncomment this when lifter is mechanically implemented
        //robotBase.lifterHandler(gamepad2.left_stick_y, gamepad2.right_stick_y);

        //Starts automatic shot sequence on either gamepad's left bumper,
        // and manually adjusts position of shooter on either gamepad's left trigger
        robotBase.shooterHandler(gamepad1.left_bumper || gamepad2.left_bumper, gamepad1.left_trigger > 0.2 || gamepad2.left_trigger > 0.2);

        //manually reloads a ball when gamepad2's right bumper is activated
        robotBase.reloadHandler(gamepad2.right_bumper);




        //Case 3: Button "A" is being pressed but spinner is off

        //Case 4: Button "A" is n

        //Case 5:


        //Case 1: Button "B" is pressed, so run spinner in reverse
        if(gamepad1.b || gamepad2.b) {
            robotBase.setMotorSpinner(-1.0);
            bReleased = true;
        }

        //Case 2: Button "B" not pressed, but pressed last time through loop, so turn spinner off
        else if (!(gamepad1.b || gamepad2.b) && bReleased) {
            robotBase.setMotorSpinner(0);
            bReleased = false;
        }
        //As long as the spinner is off...
        else if (!robotBase.spinnerIsRunning()) {
            //Case 3: A is bring pressed, either has not been pressed or has been recently released, so run spinner forward
            if ((gamepad1.a || gamepad2.a) && released) {
                robotBase.setMotorSpinner(1.0);
                released = false;
            }
            //Case 4: A is not being pressed, so allow to enter Cases 3&5 again
            else if (!(gamepad1.a || gamepad2.a)) {
                released = true;
            }
        }
        //As long as the spinner is on...
        else {
            //Case 5: A is being pressed, either has not been pressed or has been recently released, so turn the spinner off
            if ((gamepad1.a || gamepad2.a) && released) {
                robotBase.setMotorSpinner(0);
                released = false;
            }
            //Case 6: A is not being pressed, so allow to enter Cases 3&5 again
            else if (!(gamepad1.a || gamepad2.a)) {
                released = true;
            }
        }

        //Set power based on the joysticks, set 'slow mode' based on right bumper
        robotBase.updateDriveMotors(-gamepad1.left_stick_y, -gamepad1.right_stick_y, gamepad1.right_bumper);

        if(gamepad2.dpad_up){
            robotBase.motorLifterLeft.setPower(-0.75);
            robotBase.motorLifterRight.setPower(0.75);
        } else if (gamepad2.dpad_down){
            robotBase.motorLifterLeft.setPower(0.75);
            robotBase.motorLifterRight.setPower(-0.75);
        } else {
            robotBase.motorLifterLeft.setPower(0);
            robotBase.motorLifterRight.setPower(0);
        }

        if(gamepad2.dpad_right){
            robotBase.lifterLeftServo.setPosition(1);
            robotBase.lifterRightServo.setPosition(0.4);

        } else if(gamepad2.dpad_left){
            robotBase.lifterLeftServo.setPosition(0.2);
            robotBase.lifterRightServo.setPosition(0);
        }

        if(gamepad2.x){
            robotBase.grabberServo.setPosition(0);
        } else if(gamepad2.y){
            robotBase.grabberServo.setPosition(1);
        }
    }
}

