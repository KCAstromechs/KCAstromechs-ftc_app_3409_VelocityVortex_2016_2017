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
    static final double WHEEL_DIAMETER_INCHES = 4.0;    // For figuring circumference
    static final double driveSpeed = 0.3;               // Default drive speed for better accuracy.
    static final double turnSpeed = 0.5;                // Default turn speed for better accuracy.
    static final double P_DRIVE_COEFF = 0.02;           // Larger is more responsive, but also less stable
    static final double P_TURN_COEFF = 0.01;          // Larger is more responsive, but also less stable
    static final double D_TURN_COEFF = -0.03;           // Larger is more responsive, but also less stable
    static final double k_MOTOR_STALL_SPEED = 0.25;     //Minimum speed at which robot can turn
    static final double P_RAMP_COEFF = 0.00164;         //Propotional constant for driveStraight

    //encoder ticks per one inch
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    
    //defines orientation constants for beacons
    static final int BEACON_BLUE_RED = 2; 
    static final int BEACON_RED_BLUE = 1; 
    
    public static double reloadResetTime = -1; 
    
    private boolean debug = false;                      //indicates whether we want to dump data
    
    //constants used in angle analysis (picture)
    final int PIXELS_PER_INCH = 35;                     
    final int PIXELS_PER_DEGREE = 22;                   
    
    //variables for gyro operation
    float zero;
    float rawGyro;
    
    float leftLiftPower;
    float rightLiftPower;
    
    //arrays for gyro operation
    float[] rotationMatrix = new float[9];
    float[] orientation = new float[3];
    
    int index;
    double dScale;

    //Hardware reference variables
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
    public Servo grabberServo = null;
    public Servo lifterLeftServo = null;
    public Servo lifterRightServo = null;

    //Allows the program to decide whether to set a default 'zero' orientation
    public boolean hasBeenZeroed= false;

    //Used to help time reloadHandler and shooterHandler
    double timeToFinishReload = -1;

    //hardware map, opmode, & vuforia
    HardwareMap hwMap = null;
    OpMode callingOpMode;
    VuforiaLocalizer vuforia;
    
    //sets positions for ball indexer (servo)
    static final double RELOADER_CLOSED = 0.32;
    static final double RELOADER_OPEN = 0.6;
    
    //more required vars for gyro operation
    private SensorManager mSensorManager;
    private Sensor mRotationVectorSensor;

    // This is relative to the initial position of the robot.
    // Possible values are:  0-360
    // 0 is set as straight ahead of the robot, 90 is the right, 270 is to the left
    public float zRotation;

    //helps with turn/push math to give an idea of where the beacon is/was
    public double lastPicBeaconAvg;

    //for reloadHandler and shooterHandler's usage
    boolean isReloadResetting = false;
    public boolean shooterIsBusy = false;
    public boolean touchToggle = false;
    static boolean reloadJustFinished = false;
    public boolean reloadAfterShot = false;

    //defines each possible amount of power we are able to give to the motor based on joystick
    private static final double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
            0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};

    /**
     * Initializes Hardware map, motors, servo & sensors,
     * @param ahwMap to save reference to the Hardware map
     * @param _callingOpMode to save reference to the OpMode
     */
    public void init(HardwareMap ahwMap, OpMode _callingOpMode) {
        // Save reference to OpMode
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

        motorLifterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLifterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Sets motors to drive in the correct directions
        motorFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotor.Direction.FORWARD);
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        updateDriveMotors(0,0,false);

        // Set all motors to run without encoders.
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize servos.
        reloaderServo = hwMap.servo.get("reloader");
        grabberServo = hwMap.servo.get("grabber");
        lifterLeftServo = hwMap.servo.get("left");
        lifterRightServo = hwMap.servo.get("right");
        grabberServo.setPosition(0);
        lifterLeftServo.setPosition(0);
        lifterRightServo.setPosition(0);
        
        // Define and initialize touch sensors
        touchShooter = hwMap.touchSensor.get("touchShooter");
        touchPow = hwMap.touchSensor.get("touchPow");
        
        //resets all encoders
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Accessing gyro and accelerometer from Android
        mSensorManager = (SensorManager) hwMap.appContext.getSystemService(SENSOR_SERVICE);
        mRotationVectorSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR);
        mSensorManager.registerListener(this, mRotationVectorSensor, 10000);
        
        //moves servo to preset position
        reloaderServo.setPosition(RELOADER_CLOSED);
        
        //tells user initialization sequence is completed
        callingOpMode.telemetry.addData(">", "Robot Ready.");    //
        callingOpMode.telemetry.update();
    }

    /**
     * Initialize Vuforia for our usage so we can access it
     */
    public void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "Ac8xsqH/////AAAAGcG2OeE2NECwo7mM5f9KX1RKmDT79NqkIHc/ATgW2+loN9Fr8fkfb6jE42RZmiRYeei1FvM2M3kUPdl53j" +
                "+oeuhahXi7ApkbRv9cef0kbffj+4EkWKWCgQM39sRegfX+os6PjJh1fwGdxxijW0CYXnp2Rd1vkTjIs/cW2/7TFTtuJTkc17l" +
                "+FNJAeqLEfRnwrQ0FtxvBjO8yQGcLrpeKJKX/+sN+1kJ/cvO345RYfPSoG4Pi+wo/va1wmhuZ/WCLelUeww8w8u0douStuqcuz" +
                "ufrsWmQThsHqQDfDh0oGKZGIckh3jwCV2ABkP0lT6ICBDm4wOZ8REoyiY2kjsDnnFG6cT803cfzuVuPJl+uGTEf";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB888, true);
        vuforia.setFrameQueueCapacity(1);
    }

    /**
     * Stops all motors on robot
     */
    public void stopMotors() {
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorLifterLeft.setPower(0);
        motorLifterRight.setPower(0);
        motorShooter.setPower(0);
        motorSpinner.setPower(0);
    }

    /**
     * Method overloading to use default power for driveStraight
     */
    public void driveStraight(double inches, int heading) {
        driveStraight(inches, driveSpeed, heading);
    }

    /**
     * Tells the robot to drive forward at a certain heading for a specified distance
     * @param inches number of inches we ask the robot to drive, negative numbers drive bot backwards
     * @param power amount of power given to motors, does not affect distance driven, -1-1 range, neg #s drive bot backwards
     * @param heading heading of bot as it drives, range 0-360, DO NOT use to turn as it drives but instead to keep it in a straight line
     */
    public void driveStraight(double inches, double power, int heading) {
        int target = (int) (inches * COUNTS_PER_INCH);          //translates the number of inches to be driven into encoder ticks
        double max;                                             //To be used to keep powers from exceeding 1
        double error;                                           //The number of degrees between the true heading and desired heading
        double correction;                                      //Modifies power to account for error
        double leftPower;                                       //Power being fed to left side of bot
        double rightPower;                                      //Power being fed to right side of bot
        double distanceFromInitial, errorFromEndpoint;          //Used to determine how much the bot should be de/accelerating

        //Ensure that motors are set up correctly to drive
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Find initial number of ticks on encoder when beginning drive
        double encoderInitialPos = encoderMotor.getCurrentPosition();
        callingOpMode.telemetry.addData("Initial position", encoderInitialPos);

        //Clip the input power to keep it from exceeding 1 & -1
        power = Range.clip(power, -1.0, 1.0);

        //Set drive motors to initial input power
        updateDriveMotors(power, power, false);

        //Begin to correct for heading error
        //While: we have not driven correct distance & bot is not stopped
        while (Math.abs(encoderMotor.getCurrentPosition()) < Math.abs(target) && !Thread.interrupted()) {
            error = heading - zRotation;

            //Determines current distance from initial and final positions of drive
            distanceFromInitial = Math.abs(encoderMotor.getCurrentPosition() - encoderInitialPos);
            errorFromEndpoint = Math.abs(encoderMotor.getCurrentPosition() - target);

            //Modify error onto the -180-180 range
            while (error > 180) error = (error - 360);
            while (error <= -180) error = (error + 360);

            //Determine how much correction to be placed on each side of robot based on error
            correction = Range.clip(error * P_DRIVE_COEFF, -1, 1);

            //Check to de/accelerate
            //If our target is large enough to merit de/acceleration
            if(target > 550) {
                // and we are close to beginning enough to accelerate
                if (distanceFromInitial < 550) {
                    //Change pwr based on how far we are and the acceleration coefficient
                    power = distanceFromInitial * P_RAMP_COEFF;

                    callingOpMode.telemetry.addData("Initial position", encoderInitialPos);
                    callingOpMode.telemetry.addData("distanceFromInitial: ", distanceFromInitial);
                    callingOpMode.telemetry.addData("power: ", power);
                    callingOpMode.telemetry.update();
                }
                // and we are close to end enough to decelerate
                else if (errorFromEndpoint < 550) {
                    //Change pwr based on how far we are and the acceleration coefficient
                    power = errorFromEndpoint * P_RAMP_COEFF;

                    callingOpMode.telemetry.addData("power: ", power);
                    callingOpMode.telemetry.update();
                }
            }

            // If we're below stall speed, go to stall speed
            if(Math.abs(power) < k_MOTOR_STALL_SPEED) {
                if(power >= 0) {
                    power = k_MOTOR_STALL_SPEED;
                }
                else {
                    power = -k_MOTOR_STALL_SPEED;
                }
            }

            //Incorporate our correction for our heading into the power
            leftPower = power + correction;
            rightPower = power - correction;

            //Take the larger of the two powers
            max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            //If the largest power is too big, divide them both by it.
            if (max > 1.0) {
                leftPower /= max;
                rightPower /= max;
            }

            //Put the power on and hit pause for a second
            updateDriveMotors(leftPower, rightPower, false);
            Thread.yield();
        }

        //When the drive is finished, it is time to turn off the drive motors
        updateDriveMotors(0, 0, false);

        //Reset the motors for future use, just in case
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public BeaconOrientation takeLongDistancePicture() throws InterruptedException {
        int thisR, thisB, thisG;                    //RGB values of current pixel to translate into HSV
        int xRedAvg = 0;                            //Average X position of red pixels to help find red side location
        int xBlueAvg = 0;                           //Average X position of blue pixels to help find blue side location
        int totalBlue = 1;                          //Total number of blue pixels to help find blue side location
        int totalRed = 1;                           //Total number of red pixels to help find red side location
        int xRedSum = 0;                            //Added-up X pos of red pixels to find red side location
        int xBlueSum = 0;                           //Added-up X pos of blue pix to find blue side location
        int idx = 0;                                //Ensures we get correct image type from Vuforia
        int beacon1 = 0;
        float[] hsv = new float[3];                 //Array to hold Hue, Saturation, Value values for each pixel
        float thisH;                                //Hue value of current pixel to find its color


        //If Vuforia has not yet started, we're screwed. Start it up now in the name of hope
        if(vuforia == null) initVuforia();

        //Take an image from Vuforia in the correct format
        VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take();
        for (int i = 0; i < frame.getNumImages(); i++) {
            if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB888) {
                idx = i;
                break;
            }
        }

        //Create an instance of the image and then of the pixels
        Image image = frame.getImage(idx);
        ByteBuffer px = image.getPixels();

        //Loop through every pixel column
        int h = image.getHeight();
        int w = image.getWidth();
        for (int i = 15; i < 150; i++) { //h

            //If the bot stops you should really stop.
            if(Thread.interrupted()) break;

            //Loop through a certain number of rows to cover a certain area of the image
            for (int j = 650; j < 750; j++) { //925, 935 // w

                //Take the RGB vals of current pix
                thisR = px.get(i * w * 3 + (j * 3)) & 0xFF;
                thisG = px.get(i * w * 3 + (j * 3) + 1) & 0xFF;
                thisB = px.get(i * w * 3 + (j * 3) + 2) & 0xFF;

                //If the pixel is bright enough that we know it's from the beacon
                if (thisB > 230 || thisG > 230 || thisR > 230) {

                    //Convert the RGB vals into the HSV array
                    Color.RGBToHSV(thisR, thisG, thisB, hsv);

                    //Get the hue
                    thisH = hsv[0];

                    //We now have the colors (one byte each) for any pixel, (j, i) so we can add to the totals
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

        //Find the averages
        xRedAvg = xRedSum / totalRed;
        xBlueAvg = xBlueSum / totalBlue;

        if (totalBlue < 50 || totalRed < 50) {
            beacon1 = 0;
        }
        else if (xRedAvg > xBlueAvg) {
            beacon1 = BEACON_RED_BLUE;
        }
        else if (xBlueAvg > xRedAvg) {
            beacon1 = BEACON_BLUE_RED;
        }



        if(debug)
        {
            System.out.println("SSS FIRST BEACON DATA:");
            System.out.println("SSS width=" + image.getWidth());
            System.out.println("SSS height=" + image.getHeight());
            System.out.println("SSS totalRed=" + totalRed);
            System.out.println("SSS totalBlue=" + totalBlue);
            System.out.println("SSS xRedSum=" + xRedSum);
            System.out.println("SSS xBlueSum=" + xBlueSum);
            System.out.println("SSS xRedAvg=" + xRedAvg);
            System.out.println("SSS xBlueAvg=" + xBlueAvg);
            System.out.println("SSS ");
            System.out.println("SSS ");
            System.out.println("SSS ");
        }

        totalRed  = 1;
        totalBlue = 1;
        xRedSum   = 0;
        xBlueSum  = 0;
        xRedAvg   = 0;
        xBlueAvg  = 0;

        //LOOP FOR SECOND BEACON
        for (int i = 470; i < 550; i++) { //h

            //If the bot stops you should really stop.
            if(Thread.interrupted()) break;

            //Loop through a certain number of rows to cover a certain area of the image
            for (int j = 690; j < 777; j++) { //925, 935 // w

                //Take the RGB vals of current pix
                thisR = px.get(i * w * 3 + (j * 3)) & 0xFF;
                thisG = px.get(i * w * 3 + (j * 3) + 1) & 0xFF;
                thisB = px.get(i * w * 3 + (j * 3) + 2) & 0xFF;

                //If the pixel is bright enough that we know it's from the beacon
                if (thisB > 230 || thisG > 230 || thisR > 230) {

                    //Convert the RGB vals into the HSV array
                    Color.RGBToHSV(thisR, thisG, thisB, hsv);

                    //Get the hue
                    thisH = hsv[0];

                    //We now have the colors (one byte each) for any pixel, (j, i) so we can add to the totals
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

        if(debug)
        {


            System.out.println("SSS Second BEACON DATA:");
            System.out.println("SSS width=" + image.getWidth());
            System.out.println("SSS height=" + image.getHeight());
            System.out.println("SSS totalRed=" + totalRed);
            System.out.println("SSS totalBlue=" + totalBlue);
            System.out.println("SSS xRedSum=" + xRedSum);
            System.out.println("SSS xBlueSum=" + xBlueSum);
            System.out.println("SSS xRedAvg=" + xRedAvg);
            System.out.println("SSS xBlueAvg=" + xBlueAvg);
            System.out.println("SSS ");
            System.out.println("SSS ");
            System.out.println("SSS ");
        }
        if (totalBlue < 50 || totalRed < 50 || beacon1 == 0 || xRedAvg == xBlueAvg) {
            return BeaconOrientation.ORIENTATION_UNKNOWN;
        }
        else if (beacon1 == BEACON_RED_BLUE){
            if (xRedAvg > xBlueAvg) {
                return BeaconOrientation.RED_BLUE_RED_BLUE;
            } else{
                return BeaconOrientation.BLUE_RED_RED_BLUE;
            }
        }
        else if (beacon1 == BEACON_BLUE_RED) {
            if(xRedAvg > xBlueAvg) {
                return BeaconOrientation.RED_BLUE_BLUE_RED;
            } else {
                return  BeaconOrientation.BLUE_RED_BLUE_RED;
            }
        }

        return BeaconOrientation.ORIENTATION_UNKNOWN;
    }


    /**
     * Stops to pull a picture from Vuforia and analyze it for beacon colors
     * @return which orientation the beacon is sitting in
     * @throws InterruptedException
     */
    public int takePicture() throws InterruptedException {
        int thisR, thisB, thisG;                    //RGB values of current pixel to translate into HSV
        int xRedAvg = 0;                            //Average X position of red pixels to help find red side location
        int xBlueAvg = 0;                           //Average X position of blue pixels to help find blue side location
        int totalBlue = 1;                          //Total number of blue pixels to help find blue side location
        int totalRed = 1;                           //Total number of red pixels to help find red side location
        int xRedSum = 0;                            //Added-up X pos of red pixels to find red side location
        int xBlueSum = 0;                           //Added-up X pos of blue pix to find blue side location
        int idx = 0;                                //Ensures we get correct image type from Vuforia
        float[] hsv = new float[3];                 //Array to hold Hue, Saturation, Value values for each pixel
        float thisH;                                //Hue value of current pixel to find its color

        //If Vuforia has not yet started, we're screwed. Start it up now in the name of hope
        if(vuforia == null) initVuforia();

        //Take an image from Vuforia in the correct format
        VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take();
        for (int i = 0; i < frame.getNumImages(); i++) {
            if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB888) {
                idx = i;
                break;
            }
        }

        //Create an instance of the image and then of the pixels
        Image image = frame.getImage(idx);
        ByteBuffer px = image.getPixels();

        //Loop through every pixel column
        int h = image.getHeight();
        int w = image.getWidth();
        for (int i = 0; i < h; i++) {

            //If the bot stops you should really stop.
            if(Thread.interrupted()) break;

            //Loop through a certain number of rows to cover a certain area of the image
            for (int j = 0; j < w; j++) { //925, 935

                //Take the RGB vals of current pix
                thisR = px.get(i * w * 3 + (j * 3)) & 0xFF;
                thisG = px.get(i * w * 3 + (j * 3) + 1) & 0xFF;
                thisB = px.get(i * w * 3 + (j * 3) + 2) & 0xFF;

                //If the pixel is bright enough that we know it's from the beacon
                if (thisB > 230 || thisG > 230 || thisR > 230) {

                    //Convert the RGB vals into the HSV array
                    Color.RGBToHSV(thisR, thisG, thisB, hsv);

                    //Get the hue
                    thisH = hsv[0];

                    //We now have the colors (one byte each) for any pixel, (j, i) so we can add to the totals
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

        //Find the averages
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

        //If the number of pixels for either color is too low, return a null value
        if (totalBlue < 50 || totalRed < 50) {
            return 0;
        }
        //If red is further left than blue, return the RED-BLUE value
        else if (xRedAvg > xBlueAvg) {
            callingOpMode.telemetry.addData("BEACON_RED_BLUE", "");
            return BEACON_RED_BLUE;
        }
        //If blue is further left than red, return the BLUE-RED value
        else if (xBlueAvg > xRedAvg) {
            callingOpMode.telemetry.addData("BEACON_BLUE_RED", "");
            return BEACON_BLUE_RED;
        }
        //If they're equal, return a null value
        else {
            return 0;
        }
    }

    /**
     * Overloading method turn to run with default power
     */
    public void turn(float turnHeading) throws InterruptedException { turn(turnHeading, turnSpeed);  }

    /**
     * Tells the robot to turn to a specified heading according to the GAME-ROTATION-VECTOR
     * @param turnHeading the desired heading for the bot. 0-360 range
     * @param power the desired power for the turn. -1-1 range
     * @throws InterruptedException
     */
    public void turn(float turnHeading, double power) throws InterruptedException {
        int wrapFix = 0;                                        //Can be used to modify values and make math around 0 easier
        double rightPower;                                      //Power to be given to right side of bot
        double leftPower;                                       //Power to be given to left side of bot
        double motorSpeed = power;                              //Used to safely modify power by the deceleration at end of turn
        float shiftedTurnHeading = turnHeading;                 //Can be used in conjunction with wrapFix to make math around 0 easier
        float lastError;                                        //The amount of error last time we looped through, for derivative control
        double lastTime;                                        //The runtime last time we looped through, for derivative control
        double d;                                               //Derivative variable to help adjust robot speed

        //If heading is not on correct scale, put it between 0-360
        turnHeading = normalize360(turnHeading);

        //Figure out how far the robot would have to turn in counterclockwise & clockwise directions
        float cclockwise = zRotation - turnHeading;
        float clockwise = turnHeading - zRotation;

        //Normalize cwise & ccwise values to between 0=360
        clockwise = normalize360(clockwise);
        cclockwise = normalize360(cclockwise);

        int error = 1;                                          //sets the distance to the target gyro value that we will accept
        if (turnHeading - error < 0|| turnHeading + error > 360) {
            wrapFix = 180;                                      //if within the range where the clockmath breaks, shift to an easier position
            shiftedTurnHeading = normalize360(turnHeading + wrapFix);
        }

        //If it would be longer to take the ccwise path, we go *** CLOCKWISE ***
        if(Math.abs(cclockwise) >= Math.abs(clockwise)){
            lastError = clockwise;
            lastTime = callingOpMode.getRuntime();
            //While we're not within our error, and we haven't overshot, and the bot is running
            while(Math.abs(normalize360(zRotation + wrapFix)- shiftedTurnHeading) > error &&
                    Math.abs(cclockwise) >= Math.abs(clockwise) && !Thread.interrupted()) {

                //Wait a hot decisecond
                Thread.sleep(10);

                //Calc derivative
                d = (Math.abs(lastError-clockwise))/(callingOpMode.getRuntime()-lastTime);

                //Take motor speed from clockwise, Proportional & Derivative coeffs, and derivative
                motorSpeed = clockwise*P_TURN_COEFF+d*D_TURN_COEFF;

                //If necessary, pare down motorspeed, or bring it up to stall speed
                if(motorSpeed>power){
                    motorSpeed=power;
                } else if(motorSpeed<k_MOTOR_STALL_SPEED){
                    motorSpeed=k_MOTOR_STALL_SPEED;
                }

                //Set motor powers equal to motorSpeed
                leftPower=motorSpeed;
                rightPower=-motorSpeed;

                //Start driving
                updateDriveMotors(leftPower, rightPower, false);

                //Update lastError, lastTime, cwise & ccwise
                cclockwise = normalize360(zRotation - turnHeading);
                clockwise = normalize360(turnHeading - zRotation);
                lastError = clockwise;
                lastTime = callingOpMode.getRuntime();
            }

        }
        //If it would take longer to take the cwise path, we go *** COUNTERCLOCKWISE ***
        else if(Math.abs(clockwise) > Math.abs(cclockwise)){
            lastError = cclockwise;
            lastTime = callingOpMode.getRuntime();
            //While we're not within our error, and we haven't overshot, and the bot is running
            while(Math.abs(normalize360(zRotation + wrapFix)- shiftedTurnHeading) > error &&
                    Math.abs(clockwise) > Math.abs(cclockwise) && !Thread.interrupted()) {

                //Stop a hot decisecond
                Thread.sleep(10);

                //Calc derivative
                d = (Math.abs(lastError-cclockwise))/(callingOpMode.getRuntime()-lastTime);

                //Take motorSpeed from ccwise, Proportional & Derivative coeffs, and derivative
                motorSpeed = cclockwise*P_TURN_COEFF+d*D_TURN_COEFF;

                //If necessary, pare down motorSpeed or bring it up to stall speed
                if(motorSpeed>power){
                    motorSpeed=power;
                }else if(motorSpeed<k_MOTOR_STALL_SPEED){
                    motorSpeed=k_MOTOR_STALL_SPEED;
                }

                //Set motor powers equal to motorSpeed
                leftPower=-motorSpeed;
                rightPower=motorSpeed;

                //Start driving
                updateDriveMotors(leftPower, rightPower, false);

                //Update lastError, lastTime, ccwise & cwise
                cclockwise = normalize360(zRotation - turnHeading);
                clockwise = normalize360(turnHeading - zRotation);
                lastError = cclockwise;
                lastTime = callingOpMode.getRuntime();

            }
        }

        //When all's said and done, stop driving
        updateDriveMotors(0, 0, false);
    }

    /**
     * Handles the servo that is connected to the indexer and allows balls to be reloaded into the beacon, TeleOp compatible
     * @param reloadRequested only uses handler if reload is requested to make TeleOp compatible. If Tele, set equal to a button. If Auto, just put it in a while loop
     * @return to make Auto usable, put it in a loop, return true only while reloading
     */
    public boolean reloadHandler(boolean reloadRequested) {
        //If the time is more than the time it takes to reloac
        if(callingOpMode.getRuntime() > timeToFinishReload) {
            reloadJustFinished = true;                                      //we know we just finished a reload
        }
        //If we want a reload, the reset time is 'null', and we're not resetting the reload
        if(reloadRequested && reloadResetTime == -1 && !isReloadResetting) {
            reloaderServo.setPosition(RELOADER_OPEN);                       //Open the reloader for a ball
            reloadResetTime = callingOpMode.getRuntime() + 0.4;             //Put reload reset time to current time + 0.4 for timing
            timeToFinishReload = callingOpMode.getRuntime() + 0.8;          //Change the time to finish reload for timing
            return true;                                                    //Tell loop we're doing stuff
        }
        //Elseif we've been running for more time than it takes to reload and the reset time isn't 'null'
        else if(callingOpMode.getRuntime() > reloadResetTime && reloadResetTime != -1) {
            reloaderServo.setPosition(RELOADER_CLOSED);                     //Close the reloader to stop balls
            reloadResetTime = -1;                                           //Put the reset time back to 'null'
            isReloadResetting = true;                                       //tell the handler we are resetting the reloader now
            return true;                                                    //Tell loop we're doing stuff
        }
        //Elseif we've not been running long enough to get stuff done
        else if(callingOpMode.getRuntime() < timeToFinishReload) {
            return true;                                                    //Tell loop we're still working on doing stuff
        }
        //If none of the above are true, we aren't doing anything. Tell handler and loop so.
        isReloadResetting = false;
        return false;
    }

    /**
     * !! CURRENTLY NO LIFTER Used to run cap ball lifter and is TeleOp compatible with joysticks in for arguments, Auto compatible in a loop
     * @param leftPower to input amount of power for left lifter motor
     * @param rightPower to input amount of power for right lifter motor
     */
    public void lifterHandler(float leftPower, float rightPower){
        //If power is too big or small, put it at legal power
        leftLiftPower = Range.clip(rightPower, -1, 1);
        rightLiftPower = Range.clip(leftPower, -1, 1);

        //Set to one of 16 ok powers to scale
        rightLiftPower = (float)scaleInput(rightPower);
        leftLiftPower =  (float)scaleInput(leftPower);

        //Give the motors power
        motorLifterRight.setPower(rightLiftPower);
        motorLifterLeft.setPower(leftLiftPower);
    }

    /**
     * Sets drive motors with 100% legal powers, Tele compatible, used inside methods, DO NOT CALL FROM AUTO
     * @param left the power being input for the left side
     * @param right the power being input for the right side
     * @param slowDrive can turn on a slow-driving mode for TeleOp drivers
     */
    public void updateDriveMotors(double left, double right, boolean slowDrive){
        // tank drive
        // note that if y equal -1 then joystick is pushed all of the way forward.

        // clip the right/left values so that the values never exceed +/- 1
        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);
        //spin = Range.clip(spin, -1, 1); *reinstate if turned back to float

        //For slow drive mode, divide pwr for each side by e
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

    /**
     * Used to run particle shooter, Tele & Auto compatible. For tele set booleans to buttons, for Auto, set up in loop
     * @param shotRequested Only takes shot if requested to make Tele compatible.
     * @param manualRequested Only manually runs shoot motor if requested to make Tele compatible
     * @return whether the shooter is doing stuff to make Auto compatible with loops
     */
    public boolean shooterHandler(boolean shotRequested, boolean manualRequested){

        //case 0 - shoot isn't busy and nothing is requested
        if ((!shooterIsBusy && !manualRequested && !shotRequested) || callingOpMode.getRuntime() < timeToFinishReload){
            motorShooter.setPower(0);                                       //Confirm stuff is off, we're doing nothing
            return false;
        }

        // case 1 - shoot isn't busy and manual is requested
        else if (manualRequested){
            motorShooter.setPower(0.7);                                     //Turn on power, return we're doing nothing (no subroutines)
            shooterIsBusy = false;
            return false;
        }

        // case 2 - shooter isn't busy and shot was requested
        else if (!shooterIsBusy && shotRequested){
            motorShooter.setPower(0.7);                                     //Turn on power
            shooterIsBusy = true;                                           //We're finally doing something
            touchToggle = false;                                            //Touch sensor hasn't been released
            return true;                                                    //we're in a subroutine
        }

        // case 3 - shoot is busy and touch sensor not yet released (active)
        else if (shooterIsBusy && touchShooter.isPressed() && !touchToggle){
            return true;                                                    //We're in a subroutine. Will keep motors on
        }

        // case 4 - shoot is still busy and touch sensor is released (not active)
        else if (shooterIsBusy && !touchShooter.isPressed()){
            touchToggle = true;                                             //Touch sensor has been released
            return true;                                                    //We're in a subroutine still
        }

        // case 5 - shoot is busy and touch sensor has been released but is now active
        else if (shooterIsBusy && touchToggle && touchShooter.isPressed()){
            shooterIsBusy = false;                                          //We're not doing anything anymore
            motorShooter.setPower(0);                                       //Stop trying to move
            if (reloadAfterShot) reloadHandler(true);                       //If we want to reload, run the reloader
            return false;                                                   //The subroutine is over
        }
        return false;                                                       //If none of above (never), we're doing nothing
    }

    /**
     * Scale an input to the scaleArray (for teleop driving)
     * @param dVal
     * @return
     */
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

    /**
     * Drives forward and stops when POW touch sensor is pushed, to be used to push the beacon button
     * @param heading the direction the robot ought to drive in. DON'T use to turn during drive, the robot ought to drive straight
     * @param outHeading the direction the robot ought to drive in as it backs off the beacon. DON'T use to turn during drive.
     * @param timeOutSec The amount of seconds we'll wait before throwing a TimeoutException
     * @throws InterruptedException
     * @throws TimeoutException a custom exception we throw whenever pushing the button has been taking too long & we need to stop
     */
    public void pushButton(int heading, int outHeading, double timeOutSec) throws InterruptedException, TimeoutException {
        double max;                                                         //
        double error;                                                       //
        double correction;                                                  //
        double leftPower;                                                   //
        double rightPower;                                                  //
        double power = 0.5;                                                 //
        long initialTime;                                                   //
        timeOutSec *= 1000;                                                 //converts to millis

        //Reset encoders in case that hadn't been done
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Clip the power to 0-1 (we only want to run fwd to hit beacon) then update motors
        power = Range.clip(Math.abs(power), 0.0, 1.0);
        updateDriveMotors(power, power, false);

        initialTime = System.currentTimeMillis();
        //While the POW sensor is free, we're not stopped, and we haven't timed out
        while (!touchPow.isPressed() && !Thread.interrupted() && (System.currentTimeMillis() - initialTime <= timeOutSec )) {
            //Calculate the distance between desired and actual heading, and normalize it to use as error
            error = heading - zRotation;
            while (error > 180) error = -(error - 360);
            while (error <= -180) error = -(error + 360);

            //Calculate the correction to be used to modify the power
            correction = Range.clip(error * P_DRIVE_COEFF, -1, 1);

            //Modify power by correction
            leftPower = power + correction;
            rightPower = power - correction;

            //take the larger of the powers and if its too big, divide both by that
            max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (max > 1.0) {
                leftPower /= max;
                rightPower /= max;
            }

            //Start 2 drive
            updateDriveMotors(leftPower, rightPower, false);
        }

        //If we timed out, turn off motors, and throw the TimeoutException
        if (System.currentTimeMillis() - initialTime >= timeOutSec) {
            updateDriveMotors(0, 0, false);
            throw new TimeoutException();
        }

        //If we've gotten this far, time to stop driving
        updateDriveMotors(0, 0, false);

        //Reset encoders again just in case
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Drive back out from the beacon
        driveStraight(-20, -0.5, outHeading);
    }

    /**
     * Set motor spinner to a particular power
     * @param power the power it's set to
     */
    public void setMotorSpinner(double power) {
        motorSpinner.setPower(power);
    }

    /**
     * Deconstruct gyro by telling opmode to stop listening to it
     */
    public void deconstruct(){
        mSensorManager.unregisterListener(this);
    }

    /**
     * When the gyro changes, jump down here to fix it
     * @param sensorEvent
     */
    @Override
    public void onSensorChanged(SensorEvent sensorEvent) {
        SensorManager.getRotationMatrixFromVector(rotationMatrix, sensorEvent.values);
        SensorManager.getOrientation(rotationMatrix, orientation);
//        Dbg("orientation: " , orientation[0], false);

        rawGyro = (float) Math.toDegrees(orientation[0]);
        //If the zero hasn't been zeroed do the zero
        if (!hasBeenZeroed) {
            hasBeenZeroed = true;
            zero = rawGyro;
        }
        //Normalize zRotation to be used
        zRotation = normalize360(rawGyro - zero);
//        Dbg("zRotation in callback: " , zRotation, false);
    }

    /**
     * NOTHING
     * @param sensor
     * @param accuracy
     */
    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }

    /**
     * If 0-360 variables are not on a 0-360 scale, move them up and down 360 at a time to put them on one
     * @param val the variable to be changed
     * @return
     */
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

    /**
     * get zRotation (gyro heading) safely
     * @return
     */
    public float getZRotation() {
        return zRotation;
    }

    /**
     * get the beacon avg of last picture taken safely
     * @return
     */
    public double getLastPicBeaconAvg() {
        return lastPicBeaconAvg;
    }

    /**
     * Tell if debug mode is on or off
     * @param _debug
     */
    public void setDebug(boolean _debug) {
        debug = _debug;
    }

    /**
     * Get the debug var safely
     * @return
     */
    public boolean getDebug() {
        return debug;
    }

    /**
     * Tell if the shooter is cocked
     * @return
     */
    public boolean isCocked(){
        return touchShooter.isPressed();
    }

    /**
     * Tell if the collector is running
     * @return
     */
    public boolean spinnerIsRunning() {
        if(motorSpinner.getPower() != 0) {
            return true;
        }
        else {
            return false;
        }
    }

    /**
     * safely tell whether we reload after our shots
     * @param _reload
     */
    public void setReloadAfterShot (boolean _reload) { reloadAfterShot = _reload; }
}
