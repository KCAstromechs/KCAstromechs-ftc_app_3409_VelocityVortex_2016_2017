package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by N2Class1 on 3/26/2017.
 */
@Autonomous (name = "Red 65 Ramp", group = "Red")
public class NovaAutoRed65Ramp extends NovaAutoRed100 {

    //sets flag that indicates what version of autonomous we want to run, however actual drivecode is from superclass
    @Override
    public AutoType getCurrentAutoType() {
        return AutoType.OpMode65Ramp;
    }
}
