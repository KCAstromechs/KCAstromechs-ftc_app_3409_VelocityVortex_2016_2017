/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
@Disabled
@TeleOp(name="Mecanum Teleop")
public class MecanumTeleOp extends OpMode {

    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;

    public MecanumTeleOp() {

    }


    @Override
    public void init() {

        motorFrontRight=hardwareMap.dcMotor.get("FrontRight");
        motorFrontLeft=hardwareMap.dcMotor.get("FrontLeft");
        motorBackRight=hardwareMap.dcMotor.get("BackRight");
        motorBackLeft=hardwareMap.dcMotor.get("BackLeft");

    }


    @Override
    public void loop() {

        float left = gamepad1.left_stick_y;
        float right = -gamepad1.right_stick_y;
        float leftT = gamepad1.left_trigger;
        float rightT = gamepad1.right_trigger;
        if (Math.abs(left+leftT+rightT)<0.25) {
            motorBackLeft.setPower(0);
            motorFrontLeft.setPower(0);
        } else {
            motorBackLeft.setPower(left-rightT+leftT);
            if(left-rightT+leftT>1.0) motorBackLeft.setPower(1);
            motorFrontLeft.setPower(left+rightT-leftT);
            if(left+rightT-leftT>1.0) motorFrontLeft.setPower(1);
        }
        if (Math.abs(right+leftT+rightT)<0.25) {
            motorBackRight.setPower(0);
            motorFrontRight.setPower(0);
        } else {
            motorBackRight.setPower(right+rightT-leftT);
            if(right+rightT-leftT>1.0) motorBackRight.setPower(1);
            motorFrontRight.setPower(right-rightT+leftT);
            if(right-rightT+leftT>1.0) motorFrontRight.setPower(1);
        }

    }


    @Override
    public void stop() {


    }

}