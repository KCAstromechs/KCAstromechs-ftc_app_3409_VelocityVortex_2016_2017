package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp (name="TeleOp", group="Polaris")
public class PolarisTeleop extends OpMode {

    AstroRobotBaseInterface robotBase;

    @Override
    public void init(){
        robotBase = new RobotBasePolaris();
        robotBase.initTeleop(hardwareMap);

    }

    @Override
    public void loop(){

        robotBase.teleopUpdateDrive(gamepad1.left_stick_y, gamepad1.right_stick_y);

        robotBase.shooterHandler(gamepad1.y, gamepad1.left_bumper);

        robotBase.spinnerToggle(gamepad1.a, gamepad1.x);

        //robotBase.teleopUpdateLifter(gamepad2.left_stick_y, gamepad2.right_stick_y);

        //robotBase.grabberInREPLACE(gamepad2.b);

        //robotBase.grabberOutREPLACE(gamepad2.a);

        //robotBase.grabberMidREPLACE(gamepad2.x);
    }

}
