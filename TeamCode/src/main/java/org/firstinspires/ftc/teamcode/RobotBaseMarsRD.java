package org.firstinspires.ftc.teamcode;

/**
 * Created by Kevin on 2/12/2017.
 */

import android.graphics.Color;
import android.graphics.Path;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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
import org.firstinspires.ftc.robotcore.external.Telemetry;
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

public class RobotBaseMarsRD implements SensorEventListener {
    static final double COUNTS_PER_MOTOR_REV = 1100;    // NeveRest Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double driveSpeed = 0.3;     // Default drive speed for better accuracy.
    static final double turnSpeed = 0.5;      // Default turn speed for better accuracy.
    static final double P_DRIVE_COEFF = 0.02;    // Larger is more responsive, but also less stable
    static final double P_TURN_COEFF = 0.01;    // Larger is more responsive, but also less stable
    static final double D_TURN_COEFF = -0.03;    // Larger is more responsive, but also less stable
    static final double k_MOTOR_STALL_SPEED = 0.25;
    static final double P_RAMP_COEFF = 0.00164;

    //defines orientation constants for beacons
    static final int BEACON_BLUE_RED = 2;
    static final int BEACON_RED_BLUE = 1;

    public static double reloadResetTime = -1;

    private boolean debug = false;

    final int PIXELS_PER_INCH = 35;

    final int PIXELS_PER_DEGREE = 22;

    float zero;
    float rawGyro;
    float leftLiftPower;
    float rightLiftPower;

    float[] rotationMatrix = new float[9];
    float[] orientation = new float[3];

    int index;
    double dScale;

    public DcMotor motorFrontLeft = null;
    public DcMotor motorBackLeft = null;
    public DcMotor motorFrontRight = null;
    public DcMotor motorBackRight = null;

    public DcMotor motorShooter = null;
    public DcMotor motorSpinner = null;

    public DcMotor motorLifterLeft = null;
    public DcMotor motorLifterRight = null;

    public DcMotor encoderMotor = null;
    public TouchSensor touchPow = null;
    public TouchSensor touchShooter = null;
    public Servo reloaderServo = null;
    public boolean hasBeenZeroed= false;

    double timeToFinishReload = -1;

    HardwareMap hwMap = null;

    static final double RELOADER_CLOSED = 0.32;
    static final double RELOADER_OPEN = 0.6;

    OpMode callingOpMode;
    private SensorManager mSensorManager;
    private Sensor mRotationVectorSensor;

    // This is relative to the initial position of the robot.
    // Possible values are:  0-360
    // 0 is set as straight ahead of the robot, 90 is the right, 270 is to the left
    public float zRotation;
    public double lastPicBeaconAvg;

    boolean isReloadResetting = false;

    public long targetShooterPos = 0;
    public boolean shooterIsBusy = false;
    public boolean shooterIsNudged = false;
    public boolean touchToggle = false;
    public boolean spinnerIsRunning = false;
    static boolean reloadJustFinished = false;


    private static final double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
            0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};

    VuforiaLocalizer vuforia;

    /**
     * TODO
     * @param ahwMap
     * @param _callingOpMode
     */
    public void init(HardwareMap ahwMap, OpMode _callingOpMode) {

        callingOpMode = _callingOpMode;
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors

        motorFrontLeft = hwMap.dcMotor.get("frontLeft");
        motorBackLeft = hwMap.dcMotor.get("backLeft");
        motorFrontRight = hwMap.dcMotor.get("frontRight");
        motorBackRight = hwMap.dcMotor.get("backRight");
        encoderMotor = hwMap.dcMotor.get("frontLeft");

        motorSpinner = hwMap.dcMotor.get("spinner");
        motorShooter = hwMap.dcMotor.get("shooter");

        motorLifterLeft = hwMap.dcMotor.get("lifterLeft");
        motorLifterRight = hwMap.dcMotor.get("lifterRight");

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

        reloaderServo = hwMap.servo.get("reloader");
        touchShooter = hwMap.touchSensor.get("touchShooter");
        touchPow = hwMap.touchSensor.get("touchPow");
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mSensorManager = (SensorManager) hwMap.appContext.getSystemService(SENSOR_SERVICE);
        mRotationVectorSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR);
        mSensorManager.registerListener(this, mRotationVectorSensor, 10000);

        reloaderServo.setPosition(RELOADER_CLOSED);

        callingOpMode.telemetry.addData(">", "Robot Ready.");    //
        callingOpMode.telemetry.update();
    }

    /**
     * TODO
     */
    public void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "Ac8xsqH/////AAAAGcG2OeE2NECwo7mM5f9KX1RKmDT79NqkIHc/ATgW2+loN9Fr8fkfb6jE42RZmiRYeei1FvM2M3kUPdl53j" +
                "+oeuhahXi7ApkbRv9cef0kbffj+4EkWKWCgQM39sRegfX+os6PjJh1fwGdxxijW0CYXnp2Rd1vkTjIs/cW2/7TFTtuJTkc17l" +
                "+FNJAeqLEfRnwrQ0FtxvBjO8yQGcLrpeKJKX/+sN+1kJ/cvO345RYfPSoG4Pi+wo/va1wmhuZ/WCLelUeww8w8u0douStuqcuz" +
                "ufrsWmQThsHqQDfDh0oGKZGIckh3jwCV2ABkP0lT6ICBDm4wOZ8REoyiY2kjsDnnFG6cT803cfzuVuPJl+uGTEf";
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB888, true);
        vuforia.setFrameQueueCapacity(1);
    }

    /**
     * Stops all motors on robot
     */
    public void stopOpmode() {

        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorLifterLeft.setPower(0);
        motorLifterRight.setPower(0);
        motorShooter.setPower(0);
        motorSpinner.setPower(0);
    }

    public void driveStraight(double inches, int heading) {
        driveStraight(inches, driveSpeed, heading);
    }

    public void driveStraight(double inches, double power, int heading) {
        int target;
        double max;
        double error;
        double correction;
        double leftPower;
        double rightPower;
        double distanceFromInitial, errorFromEndpoint;
        double encoderInitialPos = encoderMotor.getCurrentPosition();
        target = (int) (inches * COUNTS_PER_INCH);

        callingOpMode.telemetry.addData("Initial position", encoderInitialPos);

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        power = Range.clip(power, -1.0, 1.0);

        updateDriveMotors(power, power, false);

        while (Math.abs(encoderMotor.getCurrentPosition()) < Math.abs(target) && !Thread.interrupted()) {
            error = heading - zRotation;

            distanceFromInitial = Math.abs(encoderMotor.getCurrentPosition() - encoderInitialPos);
            errorFromEndpoint = Math.abs(encoderMotor.getCurrentPosition() - target);

            while (error > 180) error = (error - 360);
            while (error <= -180) error = (error + 360);

            correction = Range.clip(error * P_DRIVE_COEFF, -1, 1);

            if(target > 550) {
                if (distanceFromInitial < 550) {
                    power = distanceFromInitial * P_RAMP_COEFF;

                    callingOpMode.telemetry.addData("Initial position", encoderInitialPos);
                    callingOpMode.telemetry.addData("distanceFromInitial: ", distanceFromInitial);
                    callingOpMode.telemetry.addData("power: ", power);
                    callingOpMode.telemetry.update();
                }
                else if (errorFromEndpoint < 550) {
                    power = errorFromEndpoint * P_RAMP_COEFF;

                    callingOpMode.telemetry.addData("power: ", power);
                    callingOpMode.telemetry.update();
                }
            }

            if(Math.abs(power) < k_MOTOR_STALL_SPEED) {
                if(power >= 0) {
                    power = k_MOTOR_STALL_SPEED;
                }
                else {
                    power = -k_MOTOR_STALL_SPEED;
                }
            }

            leftPower = power + correction;
            rightPower = power - correction;

            max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (max > 1.0) {
                leftPower /= max;
                rightPower /= max;
            }

            updateDriveMotors(leftPower, rightPower, false);

            Thread.yield();
        }

        updateDriveMotors(0, 0, false);

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
        float[] hsv = new float[3];
        float thisH;

        if(vuforia == null) initVuforia();

        VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take();
        for (int i = 0; i < frame.getNumImages(); i++) {
            if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB888) {
                idx = i;
                break;
            }
        }

        Image image = frame.getImage(idx);
        ByteBuffer px = image.getPixels();

        int h = image.getHeight();
        int w = image.getWidth();
        for (int i = 0; i < h; i++) {

            if(Thread.interrupted()) break;

            for (int j = 0; j < w; j++) {//355 to 365
                //925, 935
                thisR = px.get(i * w * 3 + (j * 3)) & 0xFF;
                thisG = px.get(i * w * 3 + (j * 3) + 1) & 0xFF;
                thisB = px.get(i * w * 3 + (j * 3) + 2) & 0xFF;

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
        lastPicBeaconAvg = (xBlueAvg + xRedAvg) / 2.0;


        if(debug)
        {
            System.out.println("");
            System.out.println("width=" + image.getWidth());
            System.out.println("height=" + image.getHeight());
            System.out.println("totalRed=" + totalRed);
            System.out.println("totalBlue=" + totalBlue);
            System.out.println("xRedSum=" + xRedSum);
            System.out.println("xBlueSum=" + xBlueSum);
            System.out.println("xRedAvg=" + xRedAvg);
            System.out.println("xBlueAvg=" + xBlueAvg);
        }

        if (totalBlue < 50 || totalRed < 50) {
            return 0;
        } else if (xRedAvg > xBlueAvg) {
            callingOpMode.telemetry.addData("BEACON_RED_BLUE", "");
            return BEACON_RED_BLUE;
        } else if (xBlueAvg > xRedAvg) {
            callingOpMode.telemetry.addData("BEACON_BLUE_RED", "");
            return BEACON_BLUE_RED;
        } else {
            return 0;
        }
    }

    public void turn(float turnHeading) throws InterruptedException { turn(turnHeading, turnSpeed);  }

    public void turn(float turnHeading, double power) throws InterruptedException {
        int wrapFix = 0;
        double rightPower;
        double leftPower;
        double motorSpeed = power;
        float shiftedTurnHeading = turnHeading;

        turnHeading = normalize360(turnHeading);

        float cclockwise = zRotation - turnHeading;
        float clockwise = turnHeading - zRotation;

        clockwise = normalize360(clockwise);
        cclockwise = normalize360(cclockwise);

        float lastError;
        double lastTime;
        double d;

        int error = 1; //sets the distance to the target gyro value that we will accept
        if (turnHeading - error < 0|| turnHeading + error > 360) {
            wrapFix = 180; //if within the range where the clockmath breaks, shift to an easier position
            shiftedTurnHeading = normalize360(turnHeading + wrapFix);
        }

        if(Math.abs(cclockwise) >= Math.abs(clockwise)){
            lastError = clockwise;
            lastTime = callingOpMode.getRuntime();
            while(Math.abs(normalize360(zRotation + wrapFix)- shiftedTurnHeading) > error &&
                    Math.abs(cclockwise) >= Math.abs(clockwise) && !Thread.interrupted()) {

                Thread.sleep(10);

                d = (Math.abs(lastError-clockwise))/(callingOpMode.getRuntime()-lastTime);

                motorSpeed = clockwise*P_TURN_COEFF+d*D_TURN_COEFF;
                if(motorSpeed>power){
                    motorSpeed=power;
                } else if(motorSpeed<k_MOTOR_STALL_SPEED){
                    motorSpeed=k_MOTOR_STALL_SPEED;
                }

                leftPower=motorSpeed;
                rightPower=-motorSpeed;

                updateDriveMotors(leftPower, rightPower, false);

                cclockwise = normalize360(zRotation - turnHeading);
                clockwise = normalize360(turnHeading - zRotation);

                // reset the variables right now for the next time through the loop
                lastError = clockwise;
                lastTime = callingOpMode.getRuntime();
            }

        }
        else if(Math.abs(clockwise) > Math.abs(cclockwise)){
            lastError = cclockwise;
            lastTime = callingOpMode.getRuntime();
            while(Math.abs(normalize360(zRotation + wrapFix)- shiftedTurnHeading) > error &&
                    Math.abs(clockwise) > Math.abs(cclockwise) && !Thread.interrupted()) {

                Thread.sleep(10);

                d = (Math.abs(lastError-cclockwise))/(callingOpMode.getRuntime()-lastTime);
                motorSpeed = cclockwise*P_TURN_COEFF+d*D_TURN_COEFF;
                if(motorSpeed>power){
                    motorSpeed=power;
                }else if(motorSpeed<k_MOTOR_STALL_SPEED){
                    motorSpeed=k_MOTOR_STALL_SPEED;
                }
                leftPower=-motorSpeed;
                rightPower=motorSpeed;

                updateDriveMotors(leftPower, rightPower, false);

                cclockwise = normalize360(zRotation - turnHeading);
                clockwise = normalize360(turnHeading - zRotation);

                // reset the variables right now for the next time through the loop
                lastError = cclockwise;
                lastTime = callingOpMode.getRuntime();

            }
        }
        updateDriveMotors(0, 0, false);
    }
    public boolean reloadHandler(boolean reloadRequested) {
        if(callingOpMode.getRuntime() > timeToFinishReload) {
            reloadJustFinished = true;
        }
        if(reloadRequested && reloadResetTime == -1 && !isReloadResetting) {
            reloaderServo.setPosition(RELOADER_OPEN);
            reloadResetTime = callingOpMode.getRuntime() + 0.4;
            timeToFinishReload = callingOpMode.getRuntime() + 0.8;
            return true;
        }
        else if(callingOpMode.getRuntime() > reloadResetTime && reloadResetTime != -1) {
            reloaderServo.setPosition(RELOADER_CLOSED);
            reloadResetTime = -1;
            isReloadResetting = true;
            return true;
        }
        else if(callingOpMode.getRuntime() < timeToFinishReload) {
            return true;
        }
        isReloadResetting = false;
        return false;
    }

    public void lifterHandler(float leftPower, float rightPower){
        leftLiftPower = Range.clip(rightPower, -1, 1);
        rightLiftPower = Range.clip(leftPower, -1, 1);

        rightLiftPower = (float)scaleInput(rightPower);
        leftLiftPower =  (float)scaleInput(leftPower);

        motorLifterRight.setPower(rightLiftPower);
        motorLifterLeft.setPower(leftLiftPower);
    }

    /**
     * TODO
     * @param left
     * @param right
     * @param slowDrive
     */
    public void updateDriveMotors(double left, double right, boolean slowDrive){
        // tank drive
        // note that if y equal -1 then joystick is pushed all of the way forward.

        // clip the right/left values so that the values never exceed +/- 1
        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);
        //spin = Range.clip(spin, -1, 1); *reinstate if turned back to float

        if(slowDrive){
            right /= 2.71828182845904523536028747135266249775724709369995957496696762772;
            left /= 2.71828182845904523536028747135266249775724709369995957496696762772;
        }
        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        right = scaleInput(right);
        left =  scaleInput(left);
        //spin =  (float)scaleInput(spin);

        // write the values to the drive motors
        motorFrontRight.setPower(right);
        motorBackRight.setPower(right);
        motorFrontLeft.setPower(left);
        motorBackLeft.setPower(left);
    }

    public boolean shooterHandler(boolean shotRequested, boolean manualRequested){

        //case 0 - shoot isn't busy and nothing is requested
        if (!shooterIsBusy && !manualRequested && !shotRequested){
            motorShooter.setPower(0);
            return false;
        }

        // case 1 - shoot isn't busy and manual is requested
        else if (!shooterIsBusy && manualRequested){
            motorShooter.setPower(0.7);
            return true;
        }

        // case 2 - shooter isn't busy and shot was requested
        else if (!shooterIsBusy && shotRequested){
            motorShooter.setPower(0.7);
            shooterIsBusy = true;
            touchToggle = false;
            return true;
        }

        // case 3 - shoot is busy and touch sensor not yet released (active)
        else if (shooterIsBusy && touchShooter.isPressed() && !touchToggle){
            return true;
        }

        // case 4 - shoot is still busy and touch sensor is released (not active)
        else if (shooterIsBusy && !touchShooter.isPressed()){
            touchToggle = true;
            return true;
        }

        // case 5 - shoot is busy and touch sensor has been released but is now active
        else if (shooterIsBusy && touchToggle && touchShooter.isPressed()){
            shooterIsBusy = false;
            motorShooter.setPower(0);
            return false;
        }
        return false;
    }

    private double scaleInput(double dVal)  {

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
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }



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
        updateDriveMotors(power, power, false);
        initialTime = System.currentTimeMillis();
        while (!touchPow.isPressed() && !Thread.interrupted() && (System.currentTimeMillis() - initialTime <= timeOutSec )) {
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

            updateDriveMotors(leftPower, rightPower, false);
        }



        if (System.currentTimeMillis() - initialTime >= timeOutSec) {
            updateDriveMotors(0, 0, false);
            throw new TimeoutException();
        }

        updateDriveMotors(0, 0, false);

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        driveStraight(-20, -0.5, outHeading);
    }

    public void setMotorSpinner(double power) {
        motorSpinner.setPower(power);
    }

    public void deconstruct(){
        mSensorManager.unregisterListener(this);
    }


    @Override
    public void onSensorChanged(SensorEvent sensorEvent) {
        SensorManager.getRotationMatrixFromVector(rotationMatrix, sensorEvent.values);
        SensorManager.getOrientation(rotationMatrix, orientation);
//        Dbg("orientation: " , orientation[0], false);

        rawGyro = (float) Math.toDegrees(orientation[0]);
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

    public float getZRotation() {
        return zRotation;
    }

    public double getLastPicBeaconAvg() {
        return lastPicBeaconAvg;
    }

    public void setDebug(boolean _debug) {
        debug = _debug;
    }
    public boolean getDebug() {
        return debug;
    }

    public boolean isCocked(){
        return touchShooter.isPressed();
    }

    public boolean spinnerIsRunning() {
        if(motorSpinner.getPower() != 0) {
            return true;
        }
        else {
            return false;
        }
    }
}