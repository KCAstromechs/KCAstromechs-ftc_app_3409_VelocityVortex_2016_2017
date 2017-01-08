package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp (name="Polaris_TeleOp", group="Polaris")
public class GalactwoTeleop extends OpMode {

    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;
    DcMotor motorSpinner;
    DcMotor motorShooter;
    boolean spinToggle = false;

    float right;
    float left;
    boolean shooterIsBusy = false;
    long target;

    final float SHOOT_SPEED = 1.0f;
    int rotation;
    int targetPos = 5;
    int index;
    long nowTime = 0;
    boolean released = true;

    static final double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
            0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

    @Override
    public void init(){
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorSpinner = hardwareMap.dcMotor.get("motorSpinner");
        motorShooter = hardwareMap.dcMotor.get("motorShooter");

        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.FORWARD);
        motorShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        boolean spinToggle = false;

    }

    @Override
    public void loop(){

        // tank drive
        // note that if y equal -1 then joystick is pushed all of the way forward.
        left = gamepad1.left_stick_y;
        right = gamepad1.right_stick_y;

        // clip the right/left values so that the values never exceed +/- 1
        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);
        //spin = Range.clip(spin, -1, 1); *reinstate if turned back to float

        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        right = (float)scaleInput(right);
        left =  (float)scaleInput(left);
        //spin =  (float)scaleInput(spin);

        // write the values to the drive motors
        motorFrontRight.setPower(right);
        motorBackRight.setPower(right);
        motorFrontLeft.setPower(left);
        motorBackLeft.setPower(left);

        if(motorShooter.getCurrentPosition()>target || !shooterIsBusy ) {
            shooterIsBusy=false;
            if (gamepad1.y) {
                target=motorShooter.getCurrentPosition()+1600;
                motorShooter.setPower(0.5);
                shooterIsBusy=true;
            } else if (gamepad1.left_bumper) {
                motorShooter.setPower(0.5);
            } else {
                motorShooter.setPower(0);
            }
        }



        /*if(gamepad1.a) {

            if (Math.abs(nowTime) - Math.abs(System.currentTimeMillis()) > 200 || !hasBeenReleased) {
                if (!spinToggle && !recentlyReleased) {
                    hasBeenReleased = true;
                    spinToggle = true;
                    recentlyReleased = false;
                    nowTime = System.currentTimeMillis();
                    motorSpinner.setPower(-1.0);
                } else if (spinToggle && !recentlyReleased) {
                } else {
                    spinToggle = false;
                    recentlyReleased = true;
                    motorSpinner.setPower(0);
                }
            }
        }*/

        if(!spinToggle){
            if (gamepad1.a && released) {
                motorSpinner.setPower(-1.0);
                spinToggle = true;
                released = false;
            } else if (!gamepad1.a){
                released = true;
            }
        } else {
            if (gamepad1.a && released){
                motorSpinner.setPower(0);
                spinToggle = false;
                released = false;
            } else if (!gamepad1.a){
                released = true;
            }
        }
        if(gamepad1.x) {
            motorShooter.setPower(1.0);
        }
    }

    double scaleInput(double dVal)  {

        // get the corresponding index for the scaleInput array.
        index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }

}
