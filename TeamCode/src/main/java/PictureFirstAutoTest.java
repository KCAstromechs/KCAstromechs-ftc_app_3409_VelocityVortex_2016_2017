import android.view.View;
import android.view.ViewGroup;
import android.widget.RelativeLayout;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.internal.AppUtil;
import org.firstinspires.ftc.teamcode.BeaconOrientation;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.RobotBaseMarsRD;
import org.firstinspires.ftc.teamcode.TimeoutException;

/**
 * Created by N2Class1 on 3/19/2017.
 */

@Autonomous(name = "PictureFirstTest")
public class PictureFirstAutoTest extends LinearOpMode {

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

        BeaconOrientation pos = robotBase.takeLongDistancePicture();

        System.out.println("SSS " + pos.toString());

        robotBase.driveStraight(35, 0);

        robotBase.turn(45);

        try {
            robotBase.pushButton(45, 45, 2);
        } catch (TimeoutException e) {
            robotBase.driveStraight(-20, -0.5, 45);
        }

        robotBase.turn(315);

        if (pos == BeaconOrientation.BLUE_RED_RED_BLUE) {
            robotBase.driveStraight(40, 315);
        }
        else if (pos == BeaconOrientation.BLUE_RED_BLUE_RED || pos == BeaconOrientation.RED_BLUE_RED_BLUE) {
            robotBase.driveStraight(35, 315);
        }
        else if (pos == BeaconOrientation.RED_BLUE_BLUE_RED) {
            robotBase.driveStraight(30, 315);
        }
        robotBase.turn(45);
        try {
            robotBase.pushButton(45, 45, 2);
        } catch (TimeoutException e){
            robotBase.driveStraight(-20, -0.5, 45);
        }

    }
}