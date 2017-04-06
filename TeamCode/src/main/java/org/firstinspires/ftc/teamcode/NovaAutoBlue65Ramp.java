package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by N2Class1 on 3/26/2017.
 */

@Autonomous(name="Blue 65 Ramp", group="Blue")
public class NovaAutoBlue65Ramp extends NovaAutoBlue100 {

    //sets flag that indicates what version of autonomous we want to run, rest of code is from superclass
    @Override
    protected AutoType getCurrentAutoType() {
        return AutoType.OpMode65Ramp;
    }
}
