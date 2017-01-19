package org.firstinspires.ftc.teamcode;

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


/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
@Disabled
@TeleOp(name="VuforiaMecanumNavigation")
public class VuforiaMecanumNavigation extends OpMode {

    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;

   // long getY = transformationMatrix.getTranslation().get(1);

    public static final String TAG = "Vuforia Sample";

    OpenGLMatrix lastLocation = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    public VuforiaMecanumNavigation() {

    }


    @Override
    public void init() {

        motorFrontRight=hardwareMap.dcMotor.get("FrontRight");
        motorFrontLeft=hardwareMap.dcMotor.get("FrontLeft");
        motorBackRight=hardwareMap.dcMotor.get("BackRight");
        motorBackLeft=hardwareMap.dcMotor.get("BackLeft");

        //Gets vuforia working and shooses a camera
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "Ac8xsqH/////AAAAGcG2OeE2NECwo7mM5f9KX1RKmDT79NqkIHc/ATgW2+loN9Fr8fkfb6jE42RZmiRYeei1FvM2M3kUPdl53j+oeuhahXi7ApkbRv9cef0kbffj+4EkWKWCgQM39sRegfX+os6PjJh1fwGdxxijW0CYXnp2Rd1vkTjIs/cW2/7TFTtuJTkc17l+FNJAeqLEfRnwrQ0FtxvBjO8yQGcLrpeKJKX/+sN+1kJ/cvO345RYfPSoG4Pi+wo/va1wmhuZ/WCLelUeww8w8u0douStuqcuzufrsWmQThsHqQDfDh0oGKZGIckh3jwCV2ABkP0lT6ICBDm4wOZ8REoyiY2kjsDnnFG6cT803cfzuVuPJl+uGTEf";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        //picks the file and sets up the image objects
        VuforiaTrackables visionTargets = this.vuforia.loadTrackablesFromAsset("ftc_velocity_vortex");
        VuforiaTrackable wheels = visionTargets.get(0);
        wheels.setName("Wheels");  // Wheels

        VuforiaTrackable tools  = visionTargets.get(1);
        tools.setName("Tools");  // Tools

        VuforiaTrackable gears = visionTargets.get(2);
        gears.setName("Gears"); //Gears

        VuforiaTrackable legos = visionTargets.get(3);
        legos.setName("Legos"); //Legos

        //puts all the images in one spot
        allTrackables.addAll(visionTargets);

        float mmPerInch        = 25.4f;
        float mmBotWidth       = 18 * mmPerInch;
        float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;

        //Places images on field
        OpenGLMatrix wheelsLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the RED WALL. Our translation here
                is a negative translation in X.*/
                .translation(-mmFTCFieldWidth / 12, mmFTCFieldWidth / 2, 0)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 270, 0)); //Z val possibly wrong. No good way to test. Fix later
        wheels.setLocation(wheelsLocationOnField);
        RobotLog.ii(TAG, "Wheels=%s", format(wheelsLocationOnField));

        OpenGLMatrix legosLocationOnField = OpenGLMatrix.identityMatrix()
                .translation(mmFTCFieldWidth / 4, mmFTCFieldWidth / 2, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90,270,0)); //See same piece of code on wheels
        legos.setLocation((legosLocationOnField));
        RobotLog.ii(TAG, "Legos=%s", format(legosLocationOnField));

        OpenGLMatrix toolsLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the Blue Audience wall.
                Our translation here is a positive translation in Y.*/
                .translation(mmFTCFieldWidth / 2, mmFTCFieldWidth / 4, 0)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 180, 0)); //See same piece of code on wheels
        tools.setLocation(toolsLocationOnField);
        RobotLog.ii(TAG, "Tools=%s", format(toolsLocationOnField));

        OpenGLMatrix gearsLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth/2, -mmFTCFieldWidth/12, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 180,0)); //See same piece of code on wheels
        gears.setLocation(gearsLocationOnField);
        RobotLog.ii(TAG, "Gears=%s", format(gearsLocationOnField));

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(mmBotWidth/2,0,0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZY,
                        AngleUnit.DEGREES, -90, 0, 0));
        RobotLog.ii(TAG, "phone=%s", format(phoneLocationOnRobot));

        ((VuforiaTrackableDefaultListener)wheels.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)tools.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);

        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();

        /** Start tracking the data sets we care about. */
        visionTargets.activate();

    }


    @Override
    public void loop() {



        boolean autoMode = false;
        if(gamepad1.x) {
            autoMode = true;
        }
        if(gamepad1.b)
            autoMode = false;


        if (autoMode = true){



        }

        float left = gamepad1.left_stick_y;
        float right = -gamepad1.right_stick_y;
        if(gamepad1.right_trigger>0.25 || gamepad1.left_trigger>0.25) {
            if (gamepad1.right_trigger > 0.25) {
                motorFrontRight.setPower(-1);
                motorBackRight.setPower(1);
                motorFrontLeft.setPower(-1);
                motorBackLeft.setPower(1);
            }

            if (gamepad1.left_trigger > 0.25) {
                motorFrontRight.setPower(1);
                motorBackRight.setPower(-1);
                motorFrontLeft.setPower(1);
                motorBackLeft.setPower(-1);
            }
        } else {
            motorFrontRight.setPower(right);
            motorBackRight.setPower(right);
            motorFrontLeft.setPower(left);
            motorBackLeft.setPower(left);
        }

        for (VuforiaTrackable trackable : allTrackables) {
            /**
             * getUpdatedRobotLocation() will return null if no new information is available since
             * the last time that call was made, or if the trackable is not currently visible.
             * getRobotLocation() will return null if the trackable is not currently visible.
             */
            telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible() ? "Visible" : "Not Visible");    //

            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
            if (robotLocationTransform != null) {
                lastLocation = robotLocationTransform;
            }
        }
        /**
         * Provide feedback as to where the robot was last located (if we know).
         */

        if (lastLocation != null) {
            //  RobotLog.vv(TAG, "robot=%s", format(lastLocation));
            telemetry.addData("Pos", format(lastLocation));
        } else {
            telemetry.addData("Pos", "Unknown");
        }
        telemetry.update();

    }


    @Override
    public void stop() {


    }

    String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();

    }


}