package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.teamcode.RobotBaseMars.reloadResetTime;

@Disabled
@TeleOp (name="MarsTeleop", group="Teleop")
public class MarsTeleop extends OpMode{

    private static final double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
            0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};

    private DcMotor motorFrontLeft = null;
    private DcMotor motorBackLeft = null;
    private DcMotor motorFrontRight = null;
    private DcMotor motorBackRight = null;
    private DcMotor motorShooter = null;
    private DcMotor motorSpinner = null;
    private DcMotor motorLifterLeft = null;
    private DcMotor motorLifterRight = null;
    private TouchSensor touchShooter = null;
    private Servo reloaderServo = null;

    private boolean shooterIsBusy;
    private boolean shooterIsResetting;
    private boolean released = true;
    private boolean spinToggle;
    private long target;

    private double RELOADER_CLOSED = 0.32; //22
    private double RELOADER_OPEN = 0.8; //0.6

    @Override
    public void init(){
        motorFrontRight = hardwareMap.dcMotor.get("frontRight");
        motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
        motorBackRight = hardwareMap.dcMotor.get("backRight");
        motorBackLeft = hardwareMap.dcMotor.get("backLeft");

        motorShooter = hardwareMap.dcMotor.get("shooter");
        motorSpinner = hardwareMap.dcMotor.get("spinner");
        motorLifterLeft = hardwareMap.dcMotor.get("lifterLeft");
        motorLifterRight = hardwareMap.dcMotor.get("lifterRight");

        reloaderServo = hardwareMap.servo.get("reloader");
        touchShooter = hardwareMap.touchSensor.get("touchShooter");

        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.FORWARD);
        motorLifterLeft.setDirection(DcMotor.Direction.REVERSE);

        motorShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }

    @Override
    public void loop(){
        // tank drive
        // note that if y equal -1 then joystick is pushed all of the way forward.

        // clip the right/left values so that the values never exceed +/- 1
        float right = Range.clip(gamepad1.right_stick_y, -1, 1);
        float left = Range.clip(gamepad1.left_stick_y, -1, 1);
        //spin = Range.clip(spin, -1, 1); *reinstate if turned back to float

        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        if(gamepad1.left_bumper){
            right /= 2.71828182845904523536028747135266249775724709369995957496696762772; // ~36.7%
            left /= 2.71828182845904523536028747135266249775724709369995957496696762772;  //this is a whole bunch of digits of e
        }



        right = (float)scaleInput(right);
        left =  (float)scaleInput(left);
        //spin =  (float)scaleInput(spin);

        // write the values to the drive motors
        motorFrontRight.setPower(right);
        motorBackRight.setPower(right);
        motorFrontLeft.setPower(left);
        motorBackLeft.setPower(left);


        if(gamepad2.left_bumper && reloadResetTime == -1) {
            reloaderServo.setPosition(RELOADER_OPEN);
            reloadResetTime = getRuntime() + 1;
        } else if(getRuntime() > reloadResetTime && reloadResetTime != -1) {
            reloaderServo.setPosition(RELOADER_CLOSED);
            reloadResetTime = -1;
        }

        if((motorShooter.getCurrentPosition() > target || !shooterIsBusy) && !shooterIsResetting) {
            shooterIsBusy=false;
            if (gamepad1.right_bumper) {
                target=motorShooter.getCurrentPosition() + 1560;
                motorShooter.setPower(0.7);
                shooterIsBusy=true;
            } else if (gamepad1.y||gamepad2.y) {
                motorShooter.setPower(0.7);
            } else if (gamepad2.right_bumper){
                motorShooter.setPower(0.7);
                shooterIsResetting = true;
            } else {
                motorShooter.setPower(0);
            }
        } else if (shooterIsResetting && touchShooter.isPressed()){
            motorShooter.setPower(0);
            shooterIsResetting = false;
        }

        if(gamepad1.b||gamepad2.b){
            motorShooter.setPower(0);
            shooterIsBusy = false;
            shooterIsResetting = false;
        }

        if(!spinToggle){
            if (gamepad2.a && released) {
                motorSpinner.setPower(1.0);
                spinToggle = true;
                released = false;
            } else if (!gamepad2.a){
                released = true;
            }
        } else {
            if (gamepad2.a && released){
                motorSpinner.setPower(0);
                spinToggle = false;
                released = false;
            } else if (!gamepad2.a){
                released = true;
            }
        }

        float right2 = Range.clip(gamepad2.right_stick_y, -1, 1);
        float left2 = Range.clip(gamepad2.left_stick_y, -1, 1);

        right2 = (float)scaleInput(right);
        left2 =  (float)scaleInput(left);

        motorLifterRight.setPower(right2);
        motorLifterLeft.setPower(left2);

        telemetry.addData("", motorShooter.getCurrentPosition());
        telemetry.update();
    }

    private double scaleInput(double dVal)  {

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

