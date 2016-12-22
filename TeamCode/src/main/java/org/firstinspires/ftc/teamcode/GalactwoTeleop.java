package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp (name="GalactwoTeleOp", group="test")
public class GalactwoTeleop extends OpMode {

    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;
    DcMotor motorSpinner;
    boolean spinToggle;
    @Override
    public void init(){
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorSpinner = hardwareMap.dcMotor.get("motorSpinner");

        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.FORWARD);

        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        boolean spinToggle = true;
    }

    @Override
    public void loop(){

        // tank drive
        // note that if y equal -1 then joystick is pushed all of the way forward.
        float left = gamepad1.left_stick_y;
        float right = gamepad1.right_stick_y;

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

        if(gamepad1.a) {
            spinToggle = true;
            if(gamepad1.a && spinToggle)
            {
                motorSpinner.setPower(-1.0);
            }
        }
        else if(gamepad1.a == false) {
            spinToggle = false;
            motorSpinner.setPower(0.0);
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