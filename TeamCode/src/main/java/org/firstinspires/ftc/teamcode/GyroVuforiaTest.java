package org.firstinspires.ftc.teamcode;

import android.view.View;
import android.view.ViewGroup;
import android.widget.RelativeLayout;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.internal.AppUtil;


/**
 * Created by N2Class1 on 3/20/2017.
 */

@Autonomous(name="Angle tester", group="Test")
public class GyroVuforiaTest extends LinearOpMode{

    protected AppUtil appUtil = AppUtil.getInstance();
    protected RelativeLayout squaresOverlay = null;
    RobotBaseNova robotBase;


    @Override
    public void runOpMode() throws InterruptedException {
        robotBase = new RobotBaseNova();
        robotBase.initVuforia();
        robotBase.init(hardwareMap, this);

        appUtil.synchronousRunOnUiThread(new Runnable() {
            @Override
            public void run() {
                squaresOverlay = (RelativeLayout) View.inflate(appUtil.getActivity(), R.layout.beacon_line_up_squares, null);
//              squaresOverlay.findViewById(R.id.firstBeacon).setVisibility(View.VISIBLE);
                squaresOverlay.findViewById(R.id.redSideBeacon).setVisibility(View.VISIBLE);
                squaresOverlay.findViewById(R.id.Origin).setVisibility(View.VISIBLE);
                appUtil.getActivity().addContentView(squaresOverlay, new RelativeLayout.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.MATCH_PARENT));
            }
        });


        while(!Thread.interrupted()) {
            telemetry.addData("Gyro value:", robotBase.getZRotation());
            telemetry.update();
        }


        waitForStart();


        appUtil.synchronousRunOnUiThread(new Runnable() {
            @Override
            public void run() {
                if (squaresOverlay != null){
                    ((ViewGroup)squaresOverlay.getParent()).removeView(squaresOverlay);
                }
                squaresOverlay = null;
            }
        });

    }
}
