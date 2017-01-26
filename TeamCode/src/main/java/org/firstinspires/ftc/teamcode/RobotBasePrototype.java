package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.text.method.Touch;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.nio.ByteBuffer;
import java.sql.SQLOutput;

import static android.content.Context.SENSOR_SERVICE;


public class RobotBasePrototype implements AstroRobotBaseInterface, SensorEventListener {
    private static final double     COUNTS_PER_MOTOR_REV    = 1100 ;    // NeveRest Motor Encoder
    private static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    private static final double     driveSpeed             = 0.75;     // Default drive speed for better accuracy.
    private static final double     turnSpeed              = 0.5;      // Default turn speed for better accuracy.
    private static final double     P_DRIVE_COEFF           = 0.1;    // Larger is more responsive, but also less stable
    private static final int        BUTTON_PUSH_TURN = 10;

    private float zero;

    private DcMotor motorLeft    = null;
    private DcMotor motorRight   = null;
    private DcMotor encoderMotor = null;
    private DcMotor motorLifter  = null;
    private DcMotor motorShooter = null;
    private TouchSensor touch    = null;
    private boolean hasBeenZeroed= false;

    long target;

    public ModernRoboticsI2cGyro gyro = null;

    private HardwareMap hwMap    =  null;

    private LinearOpMode callingOpMode;
    private SensorManager mSensorManager;
    private Sensor mRotationVectorSensor;

    // This is relative to the initial position of the robot.
    // Possible values are:  0-360
    // 0 is set as straight ahead of the robot, 90 is the right, 270 is to the left
    private float zRotation;

    private VuforiaLocalizer vuforia;

    RobotBasePrototype(LinearOpMode _callingOpMode){callingOpMode=_callingOpMode;}

    @Override
    public float getZRotation() {
        return 0;
    }

    @Override
    public double getLastPicBeaconAvg() {
        return 0;
    }

    @Override
    public void initCallingOpMode(LinearOpMode _callingOpMode) {

    }

    @Override
    public void initTeleop(HardwareMap ahwMap) {

    }

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors

        motorLeft   = hwMap.dcMotor.get("left");
        motorRight  = hwMap.dcMotor.get("right");
        motorLifter = hwMap.dcMotor.get("lifter");
        motorShooter= hwMap.dcMotor.get("shooter");

        encoderMotor= hwMap.dcMotor.get("left");

        motorLeft.setDirection(DcMotor.Direction.FORWARD);
        motorRight.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        motorLeft.setPower(0);
        motorRight.setPower(0);

        // Set all motors to run without encoders.
        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.


        // Define and initialize sensors
        gyro = (ModernRoboticsI2cGyro)hwMap.gyroSensor.get("gyro");

        touch = hwMap.touchSensor.get("touch");
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to alert driver that we are calibrating;
        callingOpMode.telemetry.addData(">", "Calibrating Gyro");    //
        callingOpMode.telemetry.update();

        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mSensorManager = (SensorManager)hwMap.appContext.getSystemService(SENSOR_SERVICE);
        mRotationVectorSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR);
        mSensorManager.registerListener(this, mRotationVectorSensor, 10000);

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "Ac8xsqH/////AAAAGcG2OeE2NECwo7mM5f9KX1RKmDT79NqkIHc/ATgW2+loN9Fr8fkfb6jE42RZmiRYeei1FvM2M3kUPdl53j" +
                "+oeuhahXi7ApkbRv9cef0kbffj+4EkWKWCgQM39sRegfX+os6PjJh1fwGdxxijW0CYXnp2Rd1vkTjIs/cW2/7TFTtuJTkc17l" +
                "+FNJAeqLEfRnwrQ0FtxvBjO8yQGcLrpeKJKX/+sN+1kJ/cvO345RYfPSoG4Pi+wo/va1wmhuZ/WCLelUeww8w8u0douStuqcuz" +
                "ufrsWmQThsHqQDfDh0oGKZGIckh3jwCV2ABkP0lT6ICBDm4wOZ8REoyiY2kjsDnnFG6cT803cfzuVuPJl+uGTEf";
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB888, true);
        vuforia.setFrameQueueCapacity(1);

        callingOpMode.telemetry.addData(">", "Robot Ready.");    //
        callingOpMode.telemetry.update();
    }


    public double zeroOutGyro(double heading) {
        double x = heading - zero;

        return x;
    }

    public void resetGyro () {
        mSensorManager = (SensorManager)hwMap.appContext.getSystemService(SENSOR_SERVICE);
        mRotationVectorSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR);
        mSensorManager.registerListener(this, mRotationVectorSensor, 10000);

        callingOpMode.telemetry.addData("Reset gyro! zAxis", zRotation);
    }

    public void driveStraight (double inches, int heading) throws InterruptedException {driveStraight(inches, driveSpeed, heading);}

    public void driveStraight (double inches, double power, int heading) throws InterruptedException {
        int target;
        double max;
        double error;
        double correction;
        double leftPower;
        double rightPower;

        target = (int)(inches * COUNTS_PER_INCH);

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        power = Range.clip(power, -1.0, 1.0);
        motorLeft.setPower(power);
        motorRight.setPower(power);

        while (Math.abs(encoderMotor.getCurrentPosition()) < Math.abs(target)) {
            error = heading -zRotation;
            while (error >= 180)  error = (error-360);
            while (error <= -180) error = (error+360);

            correction = Range.clip(error * P_DRIVE_COEFF, -1, 1);

            /*if (inches < 0)
                correction *= -1.0;
*/
            leftPower = power + correction;
            rightPower = power - correction;

            max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (max > 1.0) {
                leftPower /= max;
                rightPower /= max;
            }

            motorLeft.setPower(leftPower);
            motorRight.setPower(rightPower);

            callingOpMode.idle();
        }

        motorLeft.setPower(0);
        motorRight.setPower(0);

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public int takePicture() throws InterruptedException {
        int thisR, thisB, thisG;
        int xRedAvg = 0;
        int xBlueAvg = 0;
        int totalBlue = 1;
        int totalRed = 1;
        int xRedSum = 0;
        int xBlueSum = 0;
        int idx = 0;
        float[] hsv = new float[3];
        float thisH;

        VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take();
        for (int i = 0; i < frame.getNumImages(); i++) {
            if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB888) {
                idx = i;
                break;
            }
        }

        Image image = frame.getImage(idx);
        ByteBuffer px = image.getPixels();

        for (int i = 0; i < image.getHeight(); i++) {
            for (int j = 0; j < image.getWidth(); j++) {

                thisR = px.get() & 0xFF;
                thisG = px.get() & 0xFF;
                thisB = px.get() & 0xFF;

                if (thisB > 230 || thisG > 230 || thisR > 230) {

                    Color.RGBToHSV(thisR, thisG, thisB, hsv);

                    thisH = hsv[0];

                    //We now have the colors (one byte each) for any pixel, (j, i)
                    if (thisH <= 220 && thisH >= 180) {
                        totalBlue++;
                        xBlueSum += i;
                    } else if (thisH <= 360 && thisH >= 330) {
                        totalRed++;
                        xRedSum += i;
                    }
                }
            }
        }

        xRedAvg = xRedSum / totalRed;
        xBlueAvg = xBlueSum / totalBlue;

        System.out.println("");
        System.out.println("width=" + image.getWidth());
        System.out.println("height=" + image.getHeight());
        System.out.println("totalRed=" + totalRed);
        System.out.println("totalBlue=" + totalBlue);
        System.out.println("xRedSum=" + xRedSum);
        System.out.println("xBlueSum=" + xBlueSum);
        System.out.println("xRedAvg=" + xRedAvg);
        System.out.println("xBlueAvg=" + xBlueAvg);

        if (totalBlue < 1000 || totalRed < 1000){
            return 0;
        } else if (xRedAvg > xBlueAvg) {
            return 1;
        } else if (xBlueAvg > xRedAvg) {
            return 2;
        } else {
            return 0;
        }
    }

    @Override
    public void outputZAxis() throws InterruptedException {
        System.out.println("zAxis: " + zRotation);
        callingOpMode.telemetry.addData("zAxis: " + zRotation, 0);
        callingOpMode.telemetry.update();
    }

    @Override
    public void pushButton(int heading, int outHeading, double timeOutSec) throws InterruptedException, TimeoutException {

    }

    public void pushButton(int heading, double timeOutSec) throws InterruptedException, TimeoutException {

    }

    @Override
    public void deconstruct() {

    }

    @Override
    public void teleopUpdateDrive(float left_stick, float right_stick) {

    }

    @Override
    public void shooterHandler(boolean y, boolean lb, boolean rb) {

    }

    public void shooterHandler(boolean y, boolean lb) {

    }

    @Override
    public void spinnerToggle(boolean a, boolean x) {

    }

    @Override
    public void teleopUpdateLifter(float left_stick, float right_stick) {

    }

    @Override
    public void grabberOutREPLACE(boolean a) {

    }

    @Override
    public void grabberMidREPLACE(boolean x) {

    }

    @Override
    public void grabberInREPLACE(boolean b) {

    }

    // Normalize the angle to be between 0 and 360
    public float normalize360(float val) {
        while (val > 360 || val < 0) {

            if (val > 360) {
                val -= 360;
            }

            if (val < 0) {
                val += 360;
            }
        }
        return val;
    }

    public void turn(float turnHeading)throws InterruptedException { turn(turnHeading, turnSpeed); }

    public void Dbg(String label, int value, boolean toDriverStation)
    {
        if (toDriverStation) {
            callingOpMode.telemetry.addData(label, value);
        }
        System.out.println(label + " = " + value);
    }
    public void Dbg(String label, double value, boolean toDriverStation)
    {
        if (toDriverStation) {
            callingOpMode.telemetry.addData(label, value);
        }
        System.out.println(label + " = " + value);
    }
    public void Dbg(String label, float value, boolean toDriverStation) {
        if (toDriverStation) {
            callingOpMode.telemetry.addData(label, value);
            callingOpMode.telemetry.update();
        }
        System.out.println(label + " = " + value);
    }

    public void turn(float turnHeading, double power)throws InterruptedException{
        for (int i = 0; i < 1; i++) {
            int wrapFix = 0;
            double rightPower;
            double leftPower;
            float shiftedTurnHeading = turnHeading;

            turnHeading = normalize360(turnHeading);

            float cclockwise = zRotation - turnHeading;
            float clockwise = turnHeading - zRotation;

            clockwise = normalize360(clockwise);
            cclockwise = normalize360(cclockwise);

            int error = 3 - i; //sets the distance to the target gyro value that we will accept
            if (turnHeading - error < 0 || turnHeading + error > 360) {
                wrapFix = 180; //if within the range where the clockmath breaks, shift to an easier position
                shiftedTurnHeading = normalize360(turnHeading + wrapFix);
            }

            if ((Math.abs(normalize360(zRotation + wrapFix) - shiftedTurnHeading) > error)) {
                if (Math.abs(cclockwise) >= Math.abs(clockwise)) {

                    System.out.println("TTT turning clockwise");
                    System.out.println("TTT Shifted Turn Heading: " + shiftedTurnHeading);
                    System.out.println("TTT ZRotation: " + zRotation);
                    System.out.println("TTT Fixed ZRotation: " + normalize360(zRotation + wrapFix));

                    leftPower = power;
                    rightPower = -power;
                    motorRight.setPower(rightPower);
                    motorLeft.setPower(leftPower);

                    while (shiftedTurnHeading - normalize360(zRotation + wrapFix) > error) {
                        //callingOpMode.idle();
                        callingOpMode.sleep(1);
                    }
                    motorRight.setPower(0);
                    motorLeft.setPower(0);
                } else if (Math.abs(clockwise) > Math.abs(cclockwise)) {

                    System.out.println("TTT turning cclockwise");
                    System.out.println("TTT Shifted Turn Heading: " + shiftedTurnHeading);
                    System.out.println("TTT ZRotation: " + zRotation);
                    System.out.println("TTT Fixed ZRotation: " + normalize360(zRotation + wrapFix));

                    leftPower = -power;
                    rightPower = power;
                    motorRight.setPower(rightPower);
                    motorLeft.setPower(leftPower);


                    while (Math.abs(normalize360(zRotation + wrapFix) - shiftedTurnHeading) > error) {
                        //callingOpMode.idle();
                        callingOpMode.sleep(1);
                    }
                    motorRight.setPower(0);
                    motorLeft.setPower(0);

                }
            }
            callingOpMode.sleep(1000);
            callingOpMode.telemetry.update();
        }
    }

    public void hanShotFirst() throws InterruptedException {

    }

    @Override
    public void onSensorChanged(SensorEvent sensorEvent) {
        float[] rotationMatrix = new float[9];
        SensorManager.getRotationMatrixFromVector(rotationMatrix, sensorEvent.values);
        float[] orientation = new float[3];
        SensorManager.getOrientation(rotationMatrix, orientation);
//        Dbg("orientation: " , orientation[0], false);

        float rawGyro = (float) Math.toDegrees(orientation[0]);
        if (!hasBeenZeroed) {
            hasBeenZeroed = true;
            zero = rawGyro;
            Dbg("zero: ", zero, false);
        }
        zRotation = normalize360(rawGyro - zero);
//        Dbg("zRotation in callback: " , zRotation, false);
    }


    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }


}
