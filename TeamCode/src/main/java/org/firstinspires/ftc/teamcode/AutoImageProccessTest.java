package org.firstinspires.ftc.teamcode;

import android.view.View;
import android.view.ViewGroup;
import android.widget.RelativeLayout;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.internal.AppUtil;

/**
 * Created by N2Class1 on 1/29/2017.
 */
@Autonomous(name = "AutoImageProccessTest", group = "State testing")
public class AutoImageProccessTest extends LinearOpMode {

    RobotBaseMarsRD robotBase;
    protected RelativeLayout squaresOverlay = null;
    protected AppUtil appUtil = AppUtil.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {

        robotBase = new RobotBaseMarsRD();
        robotBase.initVuforia();

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

        robotBase.setDebug(true);

        appUtil.synchronousRunOnUiThread(new Runnable() {
            @Override
            public void run() {
                if (squaresOverlay != null){
                    ((ViewGroup)squaresOverlay.getParent()).removeView(squaresOverlay);
                }
                squaresOverlay = null;
            }
        });

        int pos = robotBase.takeLongDistancePicture();

        if (pos == RobotBasePolaris.BEACON_RED_BLUE) {
            System.out.println("TTT BEACON_RED_BLUE");
        } else if (pos == RobotBasePolaris.BEACON_BLUE_RED) {
            System.out.println("TTT BEACON_BLUE_RED");
        } else {
            System.out.println("TTT SAMUEL");
        }
    }
}