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
@Autonomous (name="Heading Test")
public class HeadingTest extends LinearOpMode {

    protected RelativeLayout squaresOverlay = null;
    protected AppUtil appUtil = AppUtil.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {
        RobotBaseMarsRD rb = new RobotBaseMarsRD();
        rb.initVuforia();

        appUtil.synchronousRunOnUiThread(new Runnable() {
            @Override
            public void run() {
                squaresOverlay = (RelativeLayout) View.inflate(appUtil.getActivity(), R.layout.beacon_line_up_squares, null);
                squaresOverlay.findViewById(R.id.firstBeacon).setVisibility(View.VISIBLE);
                squaresOverlay.findViewById(R.id.secondBeacon).setVisibility(View.VISIBLE);
                squaresOverlay.findViewById(R.id.Origin).setVisibility(View.VISIBLE);
                appUtil.getActivity().addContentView(squaresOverlay, new RelativeLayout.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.MATCH_PARENT));
            }
        });

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

        while(!Thread.interrupted()) {
            telemetry.addData("Orientation: ", rb.zRotation);
            telemetry.update();
        }
    }
}