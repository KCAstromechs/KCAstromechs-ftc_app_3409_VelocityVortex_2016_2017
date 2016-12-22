package org.firstinspires.ftc.teamcode;

/**
 * Created by N2Class1 on 11/30/2016.
 */

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.nio.ByteBuffer;

import static android.content.Context.SENSOR_SERVICE;


public class RobotBasePrototype implements PrototypeRobotBaseInterface, SensorEventListener {
    static final double     COUNTS_PER_MOTOR_REV    = 1100 ;    // NeveRest Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     driveSpeed             = 0.75;     // Default drive speed for better accuracy.
    static final double     turnSpeed              = 0.5;      // Default turn speed for better accuracy.
    static final double     P_DRIVE_COEFF           = 0.1;    // Larger is more responsive, but also less stable
    static final double RELEASE_UP = 0.75;
    static final double RELEASE_DOWN = 0;
    static final double RELOADER_UP = 0.95;
    static final double RELOADER_MID = 0.5;
    static final double RELOADER_DOWN = 0.2;


    float zero;

    public DcMotor motorLeft   = null;
    public DcMotor motorRight  = null;
    public DcMotor encoderMotor= null;
    public DcMotor motorLifter = null;
    public DcMotor motorShooter= null;
    public Servo reloader      = null;
    public Servo release       = null;
    public boolean hasBeenZeroed = false;

    long target;

    public ModernRoboticsI2cGyro gyro = null;

    HardwareMap hwMap           =  null;

    LinearOpMode callingOpMode;
    private SensorManager mSensorManager;
    private Sensor mRotationVectorSensor;

    // This is relative to the initial position of the robot.
    // Possible values are:  0-360
    // 0 is set as straight ahead of the robot, 90 is the right, 270 is to the left
    public float zRotation;

    VuforiaLocalizer vuforia;

    RobotBasePrototype(LinearOpMode _callingOpMode){callingOpMode=_callingOpMode;}

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors

        motorLeft   = hwMap.dcMotor.get("left");
        motorRight  = hwMap.dcMotor.get("right");
        motorLifter = hwMap.dcMotor.get("lifter");
        motorShooter= hwMap.dcMotor.get("shooter");

        reloader    =hwMap.servo.get("reloader");
        release     =hwMap.servo.get("release");

        reloader.setPosition(RELOADER_UP);
        release.setPosition(RELEASE_UP);

        encoderMotor= hwMap.dcMotor.get("left");

        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        motorRight.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        motorLeft.setPower(0);
        motorRight.setPower(0);

        // Set all motors to run without encoders.
        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.


        // Define and initialize sensors
        gyro = (ModernRoboticsI2cGyro)hwMap.gyroSensor.get("gyro");

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to alert driver that we are calibrating;
        callingOpMode.telemetry.addData(">", "Calibrating Gyro");    //
        callingOpMode.telemetry.update();

        gyro.calibrate();

        while (gyro.isCalibrating())  {
            callingOpMode.sleep(50);
            callingOpMode.idle();
        }

        callingOpMode.telemetry.addData(">", "Robot Ready.");    //
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
        long counter = 0;

        target = (int)(inches * COUNTS_PER_INCH);

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        power = Range.clip(Math.abs(power), 0.0, 1.0);
        motorLeft.setPower(power);
        motorRight.setPower(power);

        while (Math.abs(encoderMotor.getCurrentPosition()) < target) {
            error = heading -zRotation;
            while (error > 180)  error = -(error-360);
            while (error <= -180) error = -(error+360);

            correction = Range.clip(error * P_DRIVE_COEFF, -1, 1);

            if (inches < 0)
                correction *= -1.0;

            leftPower = power - correction;
            rightPower = power + correction;

            max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (max > 1.0) {
                leftPower /= max;
                rightPower /= max;
            }

            //motorLeft.setPower(leftPower);
            //motorRight.setPower(rightPower);

            counter ++;

            // Display drive status for the driver.
            callingOpMode.telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, correction);
            callingOpMode.telemetry.addData("Target",  "%7d",      target);
            callingOpMode.telemetry.addData("Actual",  "%7d:%7d",      motorLeft.getCurrentPosition(),
                    motorRight.getCurrentPosition());
            callingOpMode.telemetry.addData("Speed",   "%5.2f:%5.2f",  leftPower, rightPower);
            callingOpMode.telemetry.addData("Counter ", counter);
            callingOpMode.telemetry.update();

            callingOpMode.idle();
        }

        motorLeft.setPower(0);
        motorRight.setPower(0);

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public int takePicture() throws InterruptedException{
        int thisR, thisB, thisG;
        int xRedAvg   =0;
        int xBlueAvg  =0;
        int totalBlue =0;
        int totalRed  =0;
        int xRedSum   =0;
        int xBlueSum  =0;
        int idx = 0;

        VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take();
        for (int i = 0; i < frame.getNumImages(); i++){
            if(frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB888){
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

        xRedAvg = xRedSum / totalRed;
        xBlueAvg = xBlueSum / totalBlue;

        if (xRedAvg > xBlueAvg) {
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
        int wrapFix = 0;
        double rightPower;
        double leftPower;
        float shiftedTurnHeading = turnHeading;

        turnHeading = normalize360(turnHeading);

        float cclockwise = zRotation - turnHeading;
        float clockwise = turnHeading - zRotation;

        Dbg("TTT Starting turn at ", zRotation, false);
        callingOpMode.telemetry.update();

        clockwise = normalize360(clockwise);
        cclockwise = normalize360(cclockwise);
        Dbg("TTT zRotation:", zRotation, false);
        Dbg("TTT ccwise", cclockwise, false);
        Dbg("TTT cwise", clockwise, false);

        int error = 3; //sets the distance to the target gyro value that we will accept
        if (turnHeading - error < 0|| turnHeading + error > 360) {
            wrapFix = 180; //if within the range where the clockmath breaks, shift to an easier position
            shiftedTurnHeading = normalize360(turnHeading + wrapFix);
        }

        if(Math.abs(cclockwise) >= Math.abs(clockwise)){
            leftPower=-power;
            rightPower=power;
            motorRight.setPower(rightPower);
            motorLeft.setPower(leftPower);

            while(Math.abs(normalize360(zRotation + wrapFix)- shiftedTurnHeading) > error) {
                Dbg("TTT zRotationC:", zRotation, false);
                callingOpMode.sleep(20);
                callingOpMode.idle();
            }
            motorRight.setPower(0);
            motorLeft.setPower(0);
        }
        else if(Math.abs(clockwise) > Math.abs(cclockwise)){
            leftPower=power;
            rightPower=-power;
            motorRight.setPower(rightPower);
            motorLeft.setPower(leftPower);
            while(Math.abs(normalize360(zRotation + wrapFix)- shiftedTurnHeading) > error)
            {
                Dbg("TTT zRotationCC:", zRotation, false);
                callingOpMode.sleep(20);
                callingOpMode.idle();
            }
            motorRight.setPower(0);
            motorLeft.setPower(0);
        }
        Dbg("TTT Ending turn at: ", zRotation, true);
        callingOpMode.telemetry.update();

    }

    public void hanShotFirst() throws InterruptedException {
        reloader.setPosition(RELOADER_UP);
        callingOpMode.sleep(500);
        reloader.setPosition((((RELOADER_UP-RELOADER_MID)/4)*3)+RELOADER_MID);
        callingOpMode.sleep(500);
        reloader.setPosition(((RELOADER_UP-RELOADER_MID)/2)+RELOADER_MID);
        callingOpMode.sleep(500);
        reloader.setPosition(((RELOADER_UP-RELOADER_MID)/4)+RELOADER_MID);
        callingOpMode.sleep(500);

        reloader.setPosition(RELOADER_MID);
        callingOpMode.sleep(500);
        target = motorShooter.getCurrentPosition() + 1600;
        motorShooter.setPower(0.5);
        while (motorShooter.getCurrentPosition() < target) callingOpMode.sleep(1);
        motorShooter.setPower(0);
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
            Dbg("zero: ", zero, false);
        }
        zRotation = normalize360(rawGyro - zero);
//        Dbg("zRotation in callback: " , zRotation, false);
    }


    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }
}
