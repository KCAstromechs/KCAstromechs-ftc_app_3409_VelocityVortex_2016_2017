package org.firstinspires.ftc.teamcode;

/**
 * Created by N2Class1 on 11/30/2016.
 */

import android.graphics.Color;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

import java.io.File;
import java.io.FileOutputStream;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;
import android.os.Environment;
import android.graphics.Bitmap;

import static android.content.Context.SENSOR_SERVICE;


public class RobotBasePolaris implements AstroRobotBaseInterface, SensorEventListener {
    static final double COUNTS_PER_MOTOR_REV = 1100;    // NeveRest Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double driveSpeed = 0.75;     // Default drive speed for better accuracy.
    static final double turnSpeed = 0.5;      // Default turn speed for better accuracy.
    static final double P_DRIVE_COEFF = 0.1;    // Larger is more responsive, but also less stable
    static final double RELEASE_UP = 0.75;
    static final double RELEASE_DOWN = 0;
    static final double RELOADER_UP = 0.95;
    static final double RELOADER_MID = 0.5;
    static final double RELOADER_DOWN = 0.2;


    float zero;

    public DcMotor motorFrontLeft = null;
    public DcMotor motorBackLeft = null;
    public DcMotor motorFrontRight = null;
    public DcMotor motorBackRight = null;
    public DcMotor encoderMotor     = null;
    public DcMotor motorLifterLeft  = null;
    public DcMotor motorLifterRight = null;
    public DcMotor motorShooter     = null;
    public DcMotor motorSpinner     = null;
    public Servo   leftGrabber      = null;
    public Servo   rightGrabber     = null;
    public TouchSensor touch        = null;
    public boolean hasBeenZeroed= false;


    public ModernRoboticsI2cGyro gyro = null;

    HardwareMap hwMap = null;

    LinearOpMode callingOpMode;
    private SensorManager mSensorManager;
    private Sensor mRotationVectorSensor;

    // This is relative to the initial position of the robot.
    // Possible values are:  0-360
    // 0 is set as straight ahead of the robot, 90 is the right, 270 is to the left
    public float zRotation;
    public double lastPicBeaconAvg;

    VuforiaLocalizer vuforia;

    boolean spinToggle = false;

    boolean shooterIsBusy = false;

    final float SHOOT_SPEED = 1.0f;
    int rotation;
    int targetPos = 5;
    int index;
    long nowTime = 0;
    boolean released = true;
    long target;

    static final double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
            0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};

    @Override
    public float getZRotation() {
        return zRotation;
    }

    @Override
    public double getLastPicBeaconAvg() {
        return lastPicBeaconAvg;
    }

    @Override
    public void initCallingOpMode(LinearOpMode _callingOpMode) {
        callingOpMode = _callingOpMode;
    }

    @Override
    public void initTeleop(HardwareMap ahwMap) {
        hwMap = ahwMap;
        motorFrontRight = hwMap.dcMotor.get("frontRight");
        motorFrontLeft = hwMap.dcMotor.get("frontLeft");
        motorBackRight = hwMap.dcMotor.get("backRight");
        motorBackLeft = hwMap.dcMotor.get("backLeft");
        motorSpinner = hwMap.dcMotor.get("spinner");
        motorShooter = hwMap.dcMotor.get("shooter");
        motorLifterLeft = hwMap.dcMotor.get("lifterLeft");
        motorLifterRight = hwMap.dcMotor.get("lifterRight");

        leftGrabber = hwMap.servo.get("leftGrabber");
        rightGrabber = hwMap.servo.get("rightGrabber");

        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.FORWARD);
        motorShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        boolean spinToggle = false;
    }

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors

        motorFrontLeft = hwMap.dcMotor.get("frontLeft");
        motorBackLeft = hwMap.dcMotor.get("backLeft");
        motorFrontRight = hwMap.dcMotor.get("frontRight");
        motorBackRight = hwMap.dcMotor.get("backRight");
        //motorLifter = hwMap.dcMotor.get("lifter");
        motorShooter = hwMap.dcMotor.get("shooter");
        motorSpinner = hwMap.dcMotor.get("spinner");

        encoderMotor = hwMap.dcMotor.get("frontLeft");

        motorFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotor.Direction.FORWARD);
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

        // Set all motors to run without encoders.
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        touch = hwMap.touchSensor.get("touch");

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to alert driver that we are calibrating;
        callingOpMode.telemetry.addData(">", "Calibrating Gyro");    //
        callingOpMode.telemetry.update();

        callingOpMode.telemetry.addData(">", "Robot Ready.");    //
        callingOpMode.telemetry.update();

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mSensorManager = (SensorManager) hwMap.appContext.getSystemService(SENSOR_SERVICE);
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
    }


    public double zeroOutGyro(double heading) {
        double x = heading - zero;

        return x;
    }

    public void resetGyro() {
        mSensorManager = (SensorManager) hwMap.appContext.getSystemService(SENSOR_SERVICE);
        mRotationVectorSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR);
        mSensorManager.registerListener(this, mRotationVectorSensor, 10000);

        callingOpMode.telemetry.addData("Reset gyro! zAxis", zRotation);
    }

    public void driveStraight(double inches, int heading) throws InterruptedException {
        driveStraight(inches, driveSpeed, heading);
    }

    public void driveStraight(double inches, double power, int heading) throws InterruptedException {
        int target;
        double max;
        double error;
        double correction;
        double leftPower;
        double rightPower;
        long counter = 0;

        target = (int) (inches * COUNTS_PER_INCH);

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        power = Range.clip(power, -1.0, 1.0);
        motorFrontLeft.setPower(power);
        motorFrontRight.setPower(power);
        motorBackLeft.setPower(power);
        motorBackRight.setPower(power);

        while (Math.abs(encoderMotor.getCurrentPosition()) < Math.abs(target) && callingOpMode.opModeIsActive()) {
            error = heading - zRotation;
            while (error > 180) error = (error - 360);
            while (error <= -180) error = (error + 360);

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

            motorFrontLeft.setPower(leftPower);
            motorFrontRight.setPower(rightPower);
            motorBackLeft.setPower(leftPower);
            motorBackRight.setPower(rightPower);

            counter++;

            // Display drive status for the driver.
            callingOpMode.telemetry.addData("Err/St", "%5.1f/%5.1f", error, correction);
            callingOpMode.telemetry.addData("Target", "%7d", target);
            callingOpMode.telemetry.addData("Actual", "%7d:%7d", motorFrontLeft.getCurrentPosition(),
                    motorFrontRight.getCurrentPosition());
            callingOpMode.telemetry.addData("Speed", "%5.2f:%5.2f", leftPower, rightPower);
            callingOpMode.telemetry.addData("Counter ", counter);
            callingOpMode.telemetry.update();

            callingOpMode.idle();
        }

        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

        VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take();
        for (int i = 0; i < frame.getNumImages(); i++) {
            if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB888) {
                idx = i;
                break;
            }
        }

        Image image = frame.getImage(idx);
        ByteBuffer px = image.getPixels();

        int j, i;
        int w = image.getWidth();
        int h = image.getHeight();
        for (j = 0; j < h; j++) {
            for (i = 0; i < w; i++) {
                thisR = px.get() & 0xFF;
                thisG = px.get() & 0xFF;
                thisB = px.get() & 0xFF;
                //We now have the colors (one byte each) for any pixel, (j, i)

                if (thisB < 100 || thisG > 100) { //filters through noise
                    continue;
                }
                if (thisR < thisB && thisR < 100) {
                    totalBlue++;
                    xBlueSum += j;
                } else if (thisR > thisB) {
                    totalRed++;
                    xRedSum += j;
                }
            }
        }

        boolean bSavePicture = true;
        if (bSavePicture) {
            // Reset the pixel pointer to the start of the image
            px = image.getPixels();

            // Create a buffer to hold 32-bit image dataa and fill it
            int bmpData[] = new int[w * h];
            int pixel;
            int index = 0;
            int x,y;
            for (y = 0; y < h; y++) {
                for (x = 0; x < w; x++) {
                    thisR = px.get() & 0xFF;
                    thisG = px.get() & 0xFF;
                    thisB = px.get() & 0xFF;
                    bmpData[index] = Color.rgb(thisR, thisG, thisB);
                    index++;
                }
            }

            // Now create a bitmap object from the buffer
            Bitmap bmp = Bitmap.createBitmap(bmpData, w, h, Bitmap.Config.ARGB_8888);

            // And save the bitmap to the file system
            // NOTE:  AndroidManifest.xml needs <uses-permission android:name="android.permission.WRITE_EXTERNAL_STORAGE"/>
            try {
                //to convert Date to String, use format method of SimpleDateFormat class.
                DateFormat dateFormat = new SimpleDateFormat("mm-dd__hh-mm-ss");
                String strDate = dateFormat.format(new Date());

                String path = Environment.getExternalStorageDirectory() + "/Snapshot__" + strDate + ".png";
                Dbg("Snapshot filename", path);

                File file = new File(path);
                file.createNewFile();

                FileOutputStream fos = new FileOutputStream(file);
                bmp.compress(Bitmap.CompressFormat.PNG, 100, fos);
                fos.flush();
                fos.close();
            } catch (Exception e) {
                Dbg("Snapshot exception", e.getStackTrace().toString());
            }
        }

        xRedAvg = xRedSum / totalRed;
        xBlueAvg = xBlueSum / totalBlue;
        lastPicBeaconAvg = (xBlueAvg + xRedAvg) / 2.0;

        System.out.println("xRedAvg: " + xRedAvg);
        System.out.println("xBlueAvg: " + xBlueAvg);
        System.out.println("xAvg: " + ((xBlueAvg + xRedAvg) / 2.0));
        System.out.println("xRedSum: " + xRedSum);
        System.out.println("xBlueSum: " + xBlueSum);
        System.out.println("totalRed: " + totalRed);
        System.out.println("totalBlue: " + totalBlue);
        System.out.println("width: " + image.getWidth());
        System.out.println("height: " + image.getHeight());
        System.out.println("heading: " + zRotation);

        if (!callingOpMode.opModeIsActive() || totalRed < 100 || totalBlue < 100) {
            //didn't see enough color or opmode is not active
            return 0;
        }
        if (xRedAvg > xBlueAvg) {
            //red on left; blue on right
            return 1;
        } else if (xBlueAvg > xRedAvg) {
            //blue on left; red on right
            return 2;
        } else {
            //don't understand
            return 0;
        }
    }

    @Override
    public void outputZAxis() throws InterruptedException {
        System.out.println("zAxis: " + zRotation);
        callingOpMode.telemetry.addData("zAxis: " + zRotation, 0);
        callingOpMode.telemetry.update();
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

    public void Dbg(String label, int value, boolean toDriverStation) {
        Dbg(label, Integer.toString(value), toDriverStation);
    }
    public void Dbg(String label, int value) {
        Dbg(label, value, false);
    }
    public void Dbg(String label, double value, boolean toDriverStation)
    {
        Dbg(label, Double.toString(value), toDriverStation);
    }
    public void Dbg(String label, double value) {
        Dbg(label, value, false);
    }
    public void Dbg(String label, float value, boolean toDriverStation) {
        Dbg(label, Float.toString(value), toDriverStation);
    }
    public void Dbg(String label, float value) {
        Dbg(label, value, false);
    }
    public void Dbg(String label) {
        System.out.println(label);
    }
    public void Dbg(String label, String value, boolean toDriverStation) {
        if (toDriverStation) {
            callingOpMode.telemetry.addData(label, value);
            callingOpMode.telemetry.update();
        }
        
        System.out.println(label + " = " + value);
        
        // TODO: Also output to a log file on the phone?
    }
    public void Dbg(String label, String value) {
        Dbg(label, value, false);
    }

    public void turn(float turnHeading)throws InterruptedException { turn(turnHeading, turnSpeed); }

    public void turn(float turnHeading, double power)throws InterruptedException{
        int wrapFix = 0;
        double rightPower;
        double leftPower;
        float shiftedTurnHeading = turnHeading;

        turnHeading = normalize360(turnHeading);

        float cclockwise = zRotation - turnHeading;
        float clockwise = turnHeading - zRotation;

        clockwise = normalize360(clockwise);
        cclockwise = normalize360(cclockwise);

        int error = 3; //sets the distance to the target gyro value that we will accept
        if (turnHeading - error < 0|| turnHeading + error > 360) {
            wrapFix = 180; //if within the range where the clockmath breaks, shift to an easier position
            shiftedTurnHeading = normalize360(turnHeading + wrapFix);
        }

        if(Math.abs(cclockwise) >= Math.abs(clockwise)){
            leftPower=power;
            rightPower=-power;
            motorFrontRight.setPower(rightPower);
            motorFrontLeft.setPower(leftPower);
            motorBackRight.setPower(rightPower);
            motorBackLeft.setPower(leftPower);

            while(Math.abs(normalize360(zRotation + wrapFix)- shiftedTurnHeading) > error &&
                    Math.abs(cclockwise) >= Math.abs(clockwise) && callingOpMode.opModeIsActive()) {
                callingOpMode.sleep(20);
                cclockwise = normalize360(zRotation - turnHeading);
                clockwise = normalize360(turnHeading - zRotation);
                callingOpMode.idle();
            }
            motorFrontRight.setPower(0);
            motorFrontLeft.setPower(0);
            motorBackRight.setPower(0);
            motorBackLeft.setPower(0);
        }
        else if(Math.abs(clockwise) > Math.abs(cclockwise)){
            leftPower=-power;
            rightPower=power;
            motorFrontRight.setPower(rightPower);
            motorFrontLeft.setPower(leftPower);
            motorBackRight.setPower(rightPower);
            motorBackLeft.setPower(leftPower);
            while(Math.abs(normalize360(zRotation + wrapFix)- shiftedTurnHeading) > error &&
                    Math.abs(clockwise) > Math.abs(cclockwise) && callingOpMode.opModeIsActive()) {
                callingOpMode.sleep(20);
                cclockwise = normalize360(zRotation - turnHeading);
                clockwise = normalize360(turnHeading - zRotation);
                callingOpMode.idle();
            }
            motorFrontRight.setPower(0);
            motorFrontLeft.setPower(0);
            motorBackRight.setPower(0);
            motorBackLeft.setPower(0);
        }
        callingOpMode.telemetry.update();

    }

    public void hanShotFirst() throws InterruptedException {
        target = motorShooter.getCurrentPosition() + 1600;
        motorShooter.setPower(0.5);
        while (motorShooter.getCurrentPosition() < target) callingOpMode.sleep(1);
        motorShooter.setPower(0);
        motorSpinner.setPower(-0.5);
        callingOpMode.sleep(1000);
        motorSpinner.setPower(0);
        callingOpMode.sleep(250);
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
        }
        zRotation = normalize360(rawGyro - zero);
//        Dbg("zRotation in callback: " , zRotation, false);
    }


    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }


    @Override
    public void pushButton(int heading, int outHeading, double timeOutSec) throws InterruptedException, TimeoutException {
        double max;
        double error;
        double correction;
        double leftPower;
        double rightPower;
        double power = 0.5;
        long initialTime;
        timeOutSec *= 1000; //converts to millis

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        power = Range.clip(Math.abs(power), 0.0, 1.0);
        motorFrontLeft.setPower(power);
        motorFrontRight.setPower(power);
        motorBackLeft.setPower(power);
        motorBackRight.setPower(power);
        initialTime = System.currentTimeMillis();
        while (!touch.isPressed() && callingOpMode.opModeIsActive() && (System.currentTimeMillis() - initialTime <= timeOutSec )) {
            error = heading - zRotation;
            while (error > 180) error = -(error - 360);
            while (error <= -180) error = -(error + 360);

            correction = Range.clip(error * P_DRIVE_COEFF, -1, 1);

            leftPower = power + correction;
            rightPower = power - correction;

            max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (max > 1.0) {
                leftPower /= max;
                rightPower /= max;
            }

            motorFrontLeft.setPower(leftPower);
            motorFrontRight.setPower(rightPower);
            motorBackLeft.setPower(leftPower);
            motorBackRight.setPower(rightPower);
        }

        if (System.currentTimeMillis() - initialTime >= timeOutSec) {
            motorFrontLeft.setPower(0);
            motorFrontRight.setPower(0);
            motorBackLeft.setPower(0);
            motorBackRight.setPower(0);
            throw new TimeoutException();
        }

        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        driveStraight(-12, -0.5, outHeading);
    }

    @Override
    public void deconstruct(){
        mSensorManager.unregisterListener(this);
    }


    double scaleInput(double dVal)  {

        // get the corresponding index for the scaleInput array.
        index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }

    @Override
    public void teleopUpdateDrive(float left, float right){
        // tank drive
        // note that if y equal -1 then joystick is pushed all of the way forward.

        // clip the right/left values so that the values never exceed +/- 1
        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);
        //spin = Range.clip(spin, -1, 1); *reinstate if turned back to float

        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        right = (float)scaleInput(right);
        left =  (float)scaleInput(left);
        //spin =  (float)scaleInput(spin);

        // write the values to the drive motors
        motorFrontRight.setPower(right);
        motorBackRight.setPower(right);
        motorFrontLeft.setPower(left);
        motorBackLeft.setPower(left);
    }

    @Override
    public void shooterHandler(boolean y, boolean lb){

        if(motorShooter.getCurrentPosition()>target || !shooterIsBusy ) {
            shooterIsBusy=false;
            if (y) {
                target=motorShooter.getCurrentPosition()+1600;
                motorShooter.setPower(0.5);
                shooterIsBusy=true;
            } else if (lb) {
                motorShooter.setPower(0.5);
            } else {
                motorShooter.setPower(0);
            }
        }
    }

    @Override
    public void spinnerToggle(boolean a, boolean x){
        if(!spinToggle){
            if (a && released) {
                motorSpinner.setPower(-1.0);
                spinToggle = true;
                released = false;
            } else if (!a){
                released = true;
            }
        } else {
            if (a && released){
                motorSpinner.setPower(0);
                spinToggle = false;
                released = false;
            } else if (!a){
                released = true;
            }
        }
        if(x) {
            motorShooter.setPower(1.0);
        }
    }

    @Override
    public void teleopUpdateLifter(float left, float right){
         // tank drive
        // note that if y equal -1 then joystick is pushed all of the way forward.
        // clip the right/left values so that the values never exceed +/- 1
        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);
        //spin = Range.clip(spin, -1, 1); *reinstate if turned back to float

        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        right = (float)scaleInput(right);
        left =  (float)scaleInput(left);
        //spin =  (float)scaleInput(spin);

        // write the values to the drive motors
        motorLifterRight.setPower(right);
        motorLifterLeft.setPower(left);
    }

    @Override
    public void grabberOutREPLACE(boolean a){
        if (a) {
            leftGrabber.setPosition(0.2);
            rightGrabber.setPosition(0.9);
        }
    }

    @Override
    public void grabberMidREPLACE(boolean x){
        if (x) {
            leftGrabber.setPosition(0.4);
            rightGrabber.setPosition(0.7);
        }
    }

    @Override
    public void grabberInREPLACE(boolean b){
        if (b) {
            leftGrabber.setPosition(1);
            rightGrabber.setPosition(0.1);
        }
    }
}
