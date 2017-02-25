package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp (name="MarsTeleopWithBase", group="Teleop")
public class MarsTeleopWithBase extends OpMode{

    RobotBaseMars robotBase;

    @Override
    public void init(){
        robotBase.init(hardwareMap, this);
    }

    boolean insertButton;
    float insertJoyStick;

    @Override
    public void loop(){
        robotBase.shooterHandler(insertButton, insertButton, insertButton);

        robotBase.reloadHandler(insertButton);

        robotBase.teleopUpdateDrive(insertJoyStick, insertJoyStick, insertButton);
    }
}

