package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
@Disabled
@TeleOp(name="Mecanum Teleop IR")
public class MecanumIRTest extends OpMode {

    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;
    IrSeekerSensor IRS;
    private double thisTime, nextTime;
    private float left, right;

    public MecanumIRTest() {

    }


    @Override
    public void init() {

        motorFrontRight=hardwareMap.dcMotor.get("FrontRight");
        motorFrontLeft=hardwareMap.dcMotor.get("FrontLeft");
        motorBackRight=hardwareMap.dcMotor.get("BackRight");
        motorBackLeft= hardwareMap.dcMotor.get("BackLeft");
        IRS = hardwareMap.irSeekerSensor.get("IRS");
    }


    @Override
    public void loop() {
        thisTime = getRuntime();
        if (thisTime>nextTime) {
            nextTime = thisTime + 0.5;

            if (IRS.signalDetected()) {
                // Display angle and strength
                System.out.println("Angle: " + IRS.getAngle());
                System.out.println("Strength " + IRS.getStrength());
                System.out.println("Signal found");
            } else {
                // Display loss of signal
                System.out.println("No IR Signal Found");
            }

            //telemetry.update();

        }
        if(!IRS.signalDetected()) {

            left = gamepad1.left_stick_y;
            right = -gamepad1.right_stick_y;
            if (gamepad1.right_trigger > 0.25 || gamepad1.left_trigger > 0.25) {
                if (gamepad1.right_trigger > 0.25) {
                    motorFrontRight.setPower(-1);
                    motorBackRight.setPower(1);
                    motorFrontLeft.setPower(-1);
                    motorBackLeft.setPower(1);
                }

                if (gamepad1.left_trigger > 0.25) {
                    motorFrontRight.setPower(1);
                    motorBackRight.setPower(-1);
                    motorFrontLeft.setPower(1);
                    motorBackLeft.setPower(-1);
                }
            } else {
                motorFrontRight.setPower(right);
                motorBackRight.setPower(right);
                motorFrontLeft.setPower(left);
                motorBackLeft.setPower(left);
            }
        } else {
            motorBackRight.setPower(0.0);
            motorFrontRight.setPower(0.0);
            motorFrontLeft.setPower(0.0);
            motorBackLeft.setPower(0.0);
        }
    }


    @Override
    public void stop() {


    }

}