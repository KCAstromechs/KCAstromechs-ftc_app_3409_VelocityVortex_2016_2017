/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name="Mecanum Auto Test Sprint 2", group ="Test")
public class MecanumAutoTest extends LinearOpMode {

    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;
    GyroSensor gyro;

    public void shift (double inches, boolean direction) throws InterruptedException{

        //1 inch = 74.67 ticks

        int ticks = (int)(inches * 71.5);

        System.out.println("inches: " + inches);

        int directionMultiplier = (direction ? 1 : -1);

        int targetPosRight = motorFrontRight.getCurrentPosition()+(ticks * -directionMultiplier); //targetPos is the amount of encoder ticks the motor already has + amnt

        System.out.println("targetPosRight " + targetPosRight);

        if (direction){
            motorBackLeft.setPower(-1);
            motorBackRight.setPower(1);
            motorFrontLeft.setPower(1);
            motorFrontRight.setPower(-1); // set power to the motors
        }
        else {
            motorBackLeft.setPower(1);
            motorBackRight.setPower(-1);
            motorFrontLeft.setPower(-1);
            motorFrontRight.setPower(1); // set power to the motors
        }

        if (motorFrontRight.getCurrentPosition() < targetPosRight) {
            while (motorFrontRight.getCurrentPosition() < targetPosRight){
                sleep(10);
            }
        }
        else {
            while (motorFrontRight.getCurrentPosition() > targetPosRight) {
                sleep(10);
            }
        }

        /*^^^ if the currentpos is less than the targetpos initially, then while that continues to be true, sleep.
        also if targetpos is less than currentpos while that is true*/

        motorBackLeft.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFrontRight.setPower(0); //once we are at the target pos we break out of the while loop and the motors are turned off

        sleep(1000); // wait for 1 sec so no continuous drive

    }



    public void Drive(double inchDist, double power) throws InterruptedException {

        System.out.println("Beginning Drive");

        double oneRotation = Math.PI * 4.0; //4 is wheel diameter, 2-3 ratio we get circumference, or how far we travel in one rotation

        double rotations = (inchDist / oneRotation); //with this we get the number of rotations we will travel

        int ticks = (int)(rotations * 1120.0 * (60.0/95.5)); // 1120 number of encoder ticks, this converts rotations number to tick number

        System.out.println("Beginning Drive");

        double direction = 1;

        if (inchDist < 0) {
            direction = -1;
        } //this makes it so negative inchDists make negative motor power
/*        else {
            ticks *= -1;
        }
*/
        System.out.println("End of Math");

        int targetPosRight = motorFrontRight.getCurrentPosition()+(int)(ticks*direction); //this sets targetPosRight which we use later
        System.out.println("target: " + targetPosRight);

        System.out.println("After first encoder talk");

        motorBackLeft.setPower(power * direction);
        motorBackRight.setPower(power*direction);
        motorFrontLeft.setPower(power* direction);
        motorFrontRight.setPower(power*direction); // set power to the motors

        System.out.println("After give motor power");

        if (motorFrontRight.getCurrentPosition() < targetPosRight) {
            while (motorFrontRight.getCurrentPosition() < targetPosRight){
                System.out.println("FR currentPos " + motorFrontRight.getCurrentPosition());
//                System.out.println("FL currentPos " + motorFrontLeft.getCurrentPosition());
//                System.out.println("BL currentPos " + motorBackLeft.getCurrentPosition());
//                System.out.println("BR currentPos " + motorBackRight.getCurrentPosition());
                sleep(10);
            }
        }
        else {
            while (motorFrontRight.getCurrentPosition() > targetPosRight) {
                System.out.println("FR currentPos " + motorFrontRight.getCurrentPosition());
//                System.out.println("FL currentPos " + motorFrontLeft.getCurrentPosition());
//                System.out.println("BL currentPos " + motorBackLeft.getCurrentPosition());
//                System.out.println("BR currentPos " + motorBackRight.getCurrentPosition());

                sleep(10);
            }
        }

        System.out.println("FR currentPos " + motorFrontRight.getCurrentPosition());
        System.out.println("After sleep");

        /*^^^ if the currentpos is less than the targetpos initially, then while that continues to be true, sleep.
        also if targetpos is less than currentpos while that is true*/

        motorBackLeft.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFrontRight.setPower(0); //once we are at the target pos we break out of the while loop and the motors are turned off

        System.out.println("after stop");

        sleep(1000); // wait for 1 sec so no continuous drive

    }

    public double NormalizeAngle(double ang)  {
        while (ang > 360)
            ang -= 360;
        while (ang < 0)
            ang += 360;
        return ang;
    }

    public void Turn(int targetDegrees, double turnSpeed) throws InterruptedException {
        double currentDegrees = NormalizeAngle(gyro.getHeading()); //gets current angle on 360 wheel from gyro
        System.out.println("Gyro Start Position: " + currentDegrees);

        targetDegrees = (int) NormalizeAngle(targetDegrees);
        telemetry.addData("Text", "Starting turn: " + targetDegrees + " from " + currentDegrees);

        //take current-target, target-current, normalize, find difference, travel shorter distance :)))))))

        double cclockwise = currentDegrees - targetDegrees; //calculates how many degrees it would take to travel to targetpos clockwise
        double clockwise = targetDegrees - currentDegrees;//same, but for counterclockwise

        telemetry.addData("Text1", "Clockwise: " + clockwise + " ccw: " + cclockwise);

        //^^^'normalizes' data by converting it into degrees like clock math
        clockwise = NormalizeAngle(clockwise); //calculates how many degrees it would take to travel to targetpos clockwise
        cclockwise = NormalizeAngle(cclockwise);//same, but for counterclockwise

        telemetry.addData("Text1a", "Clockwise: " + clockwise + " ccw: " + cclockwise);

        //double turnSpeed = 0.25; //speed

        System.out.println("Counterclockwise val: " + cclockwise);
        System.out.println("Clockwise val: " + clockwise);
        if (cclockwise > clockwise) {

            /*if it takes more degrees to turn counterclockwise than clockwise, the robot should turn clockwise.
            thus, inside of this if statement, the robot turns left or counterclockwise*/
            telemetry.addData("Text2", "Turning clockwise: ");

            // Turn clockwise
            motorFrontLeft.setPower(turnSpeed);
            motorBackLeft.setPower(turnSpeed);
            motorFrontRight.setPower(-turnSpeed);
            motorBackRight.setPower(-turnSpeed);

            while (Math.abs(currentDegrees - targetDegrees) > 2.0) {
                sleep(10);
                currentDegrees = gyro.getHeading();
                System.out.println("current Degrees: " + currentDegrees);
                //while the place where you are at is not where you want to be give or take 2 degrees, keep going

            }
        }
        else {

            /*if it takes more degrees to turn clockwise than counterclockwise, the robot should turn counterclockwise.
            thus, inside of this if statement, the robot turns right or clockwise*/

            telemetry.addData("Text2", "Turning counter clockwise: ");

            // Turn counter clockwise
            motorFrontRight.setPower(turnSpeed);
            motorBackRight.setPower(turnSpeed);
            motorFrontLeft.setPower(-turnSpeed);
            motorBackLeft.setPower(-turnSpeed);

            while (Math.abs(currentDegrees-targetDegrees) > 2.0) {
                sleep(10);
                currentDegrees = gyro.getHeading();
                System.out.println("current Degrees: " + currentDegrees);
                //while Andromeda's current position is not where it wants to be, keep going.
            }
        }


        motorBackLeft.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFrontRight.setPower(0);
        //turn off the motors

        //sleep(1000);
        sleep(250);

    }

    @Override
    public void runOpMode() throws InterruptedException {

        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        gyro = hardwareMap.gyroSensor.get("gyro");
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //calibrate the gyro
        System.out.println("calibrating gyro");
        gyro.calibrate();
        sleep(1000);
        while (gyro.isCalibrating())
        {
            //x waitForNextHardwareCycle();
            sleep(50);
        }
        sleep(50);
        telemetry.addData("left calib", "ready to roll");

        waitForStart();

        Drive(30, 0.5);

        Turn(315, 0.25);

        Drive(30, 0.5);

        Turn(270, 0.25);

        Turn(270, 0.25); //turn twice to correct for overshoot

        System.out.println("BEFORE: " + gyro.getHeading());

        shift(44, true);

        Turn(270, 0.25);

        Turn(270, 0.25); //turn twice to correct for overshoot

        System.out.println("AFTER: " + gyro.getHeading());

    }
}