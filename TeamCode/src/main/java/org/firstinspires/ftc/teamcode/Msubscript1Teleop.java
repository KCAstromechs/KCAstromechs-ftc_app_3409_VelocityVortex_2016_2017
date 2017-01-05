package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp (name="TeleOp", group="competition")
public class Msubscript1Teleop extends OpMode {

    DcMotor motorRight;
    DcMotor motorLeft;
    DcMotor motorLifter;
    DcMotor motorShooter;
    Servo reloader;
    Servo release;

    final double RELOADER_DOWN;
    final double RELOADER_MID;
    final double RELOADER_UP;
    final double RELEASE_UP;
    final double RELEASE_DOWN;


    long target;
    boolean shooterIsBusy;

    public Msubscript1Teleop() {
        RELEASE_UP = 0.75;
        RELEASE_DOWN = 0;
        RELOADER_UP = 0.95;
        RELOADER_MID = 0.5;
        RELOADER_DOWN = 0.2;
    }

    @Override
    public void init(){
        motorRight=hardwareMap.dcMotor.get("right");
        motorLeft=hardwareMap.dcMotor.get("left");
        motorLifter=hardwareMap.dcMotor.get("lifter");
        motorShooter=hardwareMap.dcMotor.get("shooter");

        reloader=hardwareMap.servo.get("reloader");
        release=hardwareMap.servo.get("release");

        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        motorRight.setDirection(DcMotor.Direction.FORWARD);

        motorLeft.setPower(0);
        motorRight.setPower(0);

        motorShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        reloader.setPosition(RELOADER_UP);
        release.setPosition(RELEASE_UP);
    }

    @Override
    public void loop(){

        // tank drive
        // note that if y equal -1 then joystick is pushed all of the way forward.
        float left = -gamepad1.right_stick_y;
        float right = -gamepad1.left_stick_y;
        float lift = -gamepad2.left_stick_y;

        // clip the right/left values so that the values never exceed +/- 1
        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);
        lift = Range.clip(lift, -1, 1);

        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        right = (float)scaleInput(right);
        left =  (float)scaleInput(left);
        lift =  (float)scaleInput(lift);

        // write the values to the drive motors
        motorRight.setPower(right);
        motorLeft.setPower(left);
        motorLifter.setPower(lift);

        if(motorShooter.getCurrentPosition()>target || !shooterIsBusy ) {
            shooterIsBusy=false;
            if (gamepad1.y) {
                target=motorShooter.getCurrentPosition()+1600;
                motorShooter.setPower(0.5);
                shooterIsBusy=true;
            } else if (gamepad1.right_bumper) {
                motorShooter.setPower(0.5);
            } else {
                motorShooter.setPower(0);
            }
        }

        if(gamepad1.b){
            reloader.setPosition(RELOADER_MID);
        }

        if(gamepad1.a){
            reloader.setPosition(RELOADER_DOWN);
        }

        if(gamepad1.x){
            reloader.setPosition(RELOADER_UP);
        }

        if(gamepad2.left_bumper && gamepad2.right_bumper && gamepad2.a){
            release.setPosition(RELEASE_DOWN);
        }
    }

    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

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
