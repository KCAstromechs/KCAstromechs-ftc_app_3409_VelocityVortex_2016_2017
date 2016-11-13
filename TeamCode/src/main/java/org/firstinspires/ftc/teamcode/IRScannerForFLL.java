package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
@TeleOp(name="FLL IR Scanner")
public class IRScannerForFLL extends OpMode {

    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;
    IrSeekerSensor infraredSensor;
    private double thisTime, nextTime;
    private float leftPower, rightPower;


    @Override
    public void init() {

        motorFrontRight=hardwareMap.dcMotor.get("FrontRight");
        motorFrontLeft=hardwareMap.dcMotor.get("FrontLeft");
        motorBackRight=hardwareMap.dcMotor.get("BackRight");
        motorBackLeft= hardwareMap.dcMotor.get("BackLeft");
        infraredSensor = hardwareMap.irSeekerSensor.get("IRS");
    }


    @Override
    public void loop() {

        leftPower = gamepad1.left_stick_y;
        rightPower = gamepad1.right_stick_y;

        if (infraredSensor.signalDetected()){
            RightMotors(0);
            LeftMotors(0);
        }

        else{
            RightMotors(rightPower);
            LeftMotors(-leftPower);
        }
    }




















    // here be dragons (don't scroll further)























    public void RightMotors (float power) {
        motorFrontRight.setPower(power);
        motorBackRight.setPower(power);
    }
    public void LeftMotors (float power) {
        motorFrontLeft.setPower(power);
        motorBackLeft.setPower(power);
    }

    public void IR_DataDump () {
        thisTime = getRuntime();
        if (thisTime>nextTime) {
            nextTime = thisTime + 0.5;

            if (infraredSensor.signalDetected()) {
                // Display angle and strength
                System.out.println("Angle: " + infraredSensor.getAngle());
                System.out.println("Strength " + infraredSensor.getStrength());
                System.out.println("Signal found");
            } else {
                // Display loss of signal
                System.out.println("No IR Signal Found");
            }

        }

    }

    @Override
    public void stop() {


    }

}