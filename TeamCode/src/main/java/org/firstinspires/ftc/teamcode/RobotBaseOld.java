package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Color;
import android.graphics.SurfaceTexture;
import android.hardware.Camera;
import android.hardware.Camera.CameraInfo;
import android.hardware.Camera.PictureCallback;
import android.os.Environment;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.io.File;
import java.io.FileOutputStream;
import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * Created by Kevin on 12/6/2015.
 */
public class RobotBaseOld implements AstroRobotBaseInterface {

    double inchesToEncoder = 86.3;

    //motors
    public DcMotor motorFrontRight;
    public DcMotor motorFrontLeft;
    public DcMotor motorBackRight;
    public DcMotor motorBackLeft;
    public DcMotor motorWinch;
    public DcMotor motorDrawerSlide;
    public DcMotor motorRight;
    public DcMotor motorLeft;
    public DcMotor encoderMotor;

    //LED control
    //public DcMotor redLED;
    //public DcMotor blueLED;

    //Servos
    Servo mjolnir;
    Servo grabber;
    Servo leftZipline;
    Servo rightZipline;
    Servo leftLock;
    Servo rightLock;
    Servo leftHook;
    Servo rightHook;
    Servo push;

    //sensors
    GyroSensor gyro;

    HardwareMap hardwareMap;

    LinearOpMode callingOpMode;

    //camera init
    Camera camera;
    PictureCallback picDone;
    int CameraID = -1;
    public static final int MEDIA_TYPE_IMAGE = 1;
    public static final int MEDIA_TYPE_VIDEO = 2;
    SurfaceTexture texture;

    //camera processing
    int yRedAvg = 0;
    int yBlueAvg = 0;
    int totalBlue = 0;
    int totalRed = 0;
    boolean cameraProcessDone = false;

    //time catch
    double driveTime;



    public RobotBaseOld(HardwareMap hardwareMap, LinearOpMode _callingOpMode) {
        this.initializeVariables(hardwareMap);
        callingOpMode=_callingOpMode;
    }

    public RobotBaseOld(HardwareMap hardwareMap) {
        this.initializeVariables(hardwareMap);
    }

    public void initializeVariables (HardwareMap _hardwareMap){

        boolean rightZiplineDown = false;
        boolean leftZiplineDown = false;

        //motor init
        hardwareMap = _hardwareMap;
        setDriveForward();
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //color controls
        //redLED = hardwareMap.dcMotor.get("red");
        //blueLED = hardwareMap.dcMotor.get("blue");

        //Servo init


        //sensor init
        gyro=hardwareMap.gyroSensor.get("gyro");
    }

    @Override
    public void setColorRed(){
        //redLED.setPower(1);
        //blueLED.setPower(0);
    }
    @Override
    public void setColorBlue(){
        //redLED.setPower(0);
        //blueLED.setPower(1);
    }

    @Override
    public void setDriveReverse(){
        motorRight = hardwareMap.dcMotor.get("left");
        motorLeft = hardwareMap.dcMotor.get("right");
        encoderMotor = motorLeft;
        motorRight.setDirection(DcMotor.Direction.REVERSE);
        motorLeft.setDirection(DcMotor.Direction.FORWARD);
        //callingOpMode.sleep(250);
    }

    @Override
    public void setDriveForward(){
        motorFrontRight = hardwareMap.dcMotor.get("frontRight");
        motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
        motorBackRight = hardwareMap.dcMotor.get("backRight");
        motorBackLeft = hardwareMap.dcMotor.get("backLeft");
        encoderMotor = motorFrontRight;
    }

    public void snapPic(){
        camera.startPreview();
        //callingOpMode.telemetry.addData("cameraStatus", "Preview Started");
        System.out.println("Preview Started");
        camera.takePicture(null, null, picDone);
        //callingOpMode.telemetry.addData("cameraStatus", "Picture requested");
        System.out.println("Picture requested");
    }

    public void cameraSetup() throws InterruptedException {
        //callingOpMode.telemetry.addData("cameraStatus", "cameraSetup");
        //finds back camera
        //mounted screen up, using back camera with mirror
        int numOfCameras = Camera.getNumberOfCameras();
        for (int i = 0; i < numOfCameras; i++){
            CameraInfo info = new CameraInfo();
            System.out.println("CameraInfo defined");
            Camera.getCameraInfo(i, info);
            System.out.println("CameraInfo initialized");
            if(info.facing == CameraInfo.CAMERA_FACING_BACK){
                CameraID = i;
                System.out.println("CameraID: " + CameraID);

                try {
                    texture = new SurfaceTexture(0);
                    System.out.println("Texture intitialized");
                    camera = Camera.open(CameraID);
                    System.out.println("Camera.open");
                    camera.setPreviewTexture(texture);
                    System.out.println("setPreviewTexture");
                    picDone = getPicCallback();
                    System.out.println("Found Camera");
                    //callingOpMode.telemetry.addData("cameraStatus", "Found Camera");
                    Thread.sleep(2000);
                }
                catch (Exception e){
                    System.out.println("cameraSetup Failed, exception " + e.getMessage());
                    System.out.println("CameraID: " + CameraID);
                    //callingOpMode.telemetry.addData("cameraStatus", "cameraSetup Failed");
                }
            }
        }
    }

    private static File getOutputMediaFile(int type, String folder_name) {
        // To be safe, you should check that the SDCard is mounted
        // using Environment.getExternalStorageState() before doing this.

        File mediaStorageDir = new File(Environment.getExternalStoragePublicDirectory(
                Environment.DIRECTORY_PICTURES), folder_name);
        // This location works best if you want the created images to be shared
        // between applications and persist after your app has been uninstalled.

        // Create the storage directory if it does not exist
        if (! mediaStorageDir.exists()){
            if (! mediaStorageDir.mkdirs()){
                Log.d("DEBUG", "Unable to create directory!");
                return null;
            }
        }

        // Create a media file name
        String timeStamp = new SimpleDateFormat("yyyyMMdd_HHmmss").format(new Date());
        File mediaFile;
        if (type == MEDIA_TYPE_IMAGE){
            mediaFile = new File(mediaStorageDir.getPath() + File.separator +
                    "IMG_"+ timeStamp + ".jpg");
        } else if(type == MEDIA_TYPE_VIDEO) {
            mediaFile = new File(mediaStorageDir.getPath() + File.separator +
                    "VID_"+ timeStamp + ".mp4");
        } else {
            return null;
        }
//        Log.d(TAG,mediaStorageDir.getPath() + File.separator +
        //              "IMG_"+ timeStamp + ".jpg");
        return mediaFile;
    }

    public PictureCallback getPicCallback(){
        //callingOpMode.telemetry.addData("cameraStatus", "Starting callback");
        PictureCallback picture = new PictureCallback() {
            @Override
            public void onPictureTaken(byte[] data, Camera camera) {
                System.out.println("got Data");
                //callingOpMode.telemetry.addData("cameraStatus", "got Data");
                Bitmap picture = BitmapFactory.decodeByteArray(data, 0, data.length);

                System.out.println("width: " + picture.getWidth());
                System.out.println("Hight: " + picture.getHeight());

                /*
                int clr = picture.getPixel(60,80);
                System.out.println("Red: " +Color.red(clr));
                System.out.println("Blue: " + Color.blue(clr));
                System.out.println("Green: " + Color.green(clr));
                */

                int yRedSum = 0;

                int yBlueSum = 0;

                int currentPixel = 0;


                File picturefile = getOutputMediaFile(MEDIA_TYPE_IMAGE, "capture");

                try {
                    //write the file
                    FileOutputStream fos = new FileOutputStream(picturefile);
                    fos.write(data);
                    fos.close();
                } catch (Exception e) {
                    System.out.println("Camera: " + "failed to save pic, exception" + e.getMessage());

                    //callingOpMode.telemetry.addData("Status", "Failed to save pic, Exception" + e.getMessage());
                }
                for (int x = 0; x < picture.getWidth(); x++) {
                    for (int y = 0; y < picture.getHeight(); y++) {
                        currentPixel = picture.getPixel(x, y); // view is rotated 90 deg. counterclockwise
                        // eliminate pixels of unwanted color
                        if (Color.blue(currentPixel) < 80 || Color.green(currentPixel) > 120) { //filters through noise
                            continue;
                        }
                        /*
                        System.out.println("Pixel (x,y): " + "(" + x + "," + y + ")");  //location of pixel
                        System.out.print(Color.red(currentPixel));
                        System.out.print(",");
                        System.out.print(Color.green(currentPixel));
                        System.out.print(",");
                        System.out.println(Color.blue(currentPixel));
                        */
                        if (Color.red(currentPixel) < Color.blue(currentPixel)) {
                            totalBlue++;
                            yBlueSum += y;
                            //System.out.println("Current pixel is Blue");
                        } else {
                            totalRed++;
                            yRedSum += y;
                            //System.out.println("Current pixel is Red");
                        }
                    }
                }
                if(totalBlue!=0){
                    yBlueAvg = yBlueSum / totalBlue;
                }
                if(totalRed!=0) {
                    yRedAvg = yRedSum / totalRed;
                }
                /*
                if (yBlueAvg > yRedAvg) {
                    System.out.println("Blue Side: " + "Left");
                    System.out.println("Red Side: " + "Right");
                    System.out.println("==>   Blue | Red      avgBlueY:" + yBlueAvg + " avgRedY:" + yRedAvg);

                } else {
                    System.out.println("Blue Side: " + "Right");
                    System.out.println("Red Side: " + "Left");
                    System.out.println("==>   Red | Blue      avgBlueY:" + yBlueAvg + " avgRedY:" + yRedAvg);
                }
                */
                cameraProcessDone = true;
            }
        };

        //callingOpMode.telemetry.addData("Status", "PictureCallback is done");
        return picture;

    }

    @Override
    public int get_yRedAvg(){
        return yRedAvg;
    }

    @Override
    public int get_yBlueAvg() {
        return yBlueAvg;
    }

    @Override
    public int get_BlueTotal(){
        return totalBlue;
    }

    @Override
    public int get_RedTotal(){
        return totalRed;
    }

    @Override
    public Boolean get_cameraProcessDone(){
        return cameraProcessDone;
    }

    public void calibrateGyro()throws InterruptedException{
        gyro.calibrate();

        Thread.sleep(100);
        while (gyro.isCalibrating()) {
            Thread.sleep(50);
        }
        // checks drift rate
        gyroDriftCatch();
        System.out.println("gyro (PreCalibration) = " + gyro.getHeading());
    }

    @Override
    public void gyroDriftCatch() throws InterruptedException{
        int initial = gyro.getHeading();
        Thread.sleep(15000);
        int current = gyro.getHeading();

        int driftChange = current - initial;
        float driftRate = driftChange/15f; //degrees per second
        callingOpMode.telemetry.addData("Drift Rate", driftRate);

        if (driftRate < 0.0333){
            gyro.resetZAxisIntegrator();
            callingOpMode.telemetry.addData("gyro", "Calibrated");
        }
        else{
            calibrateGyro();
        }
    }

    @Override
    public void gyroResetZaxisIntegrator() throws InterruptedException{
        gyro.resetZAxisIntegrator();
        Thread.sleep(100);
    }

    public void setGrabberUp() {

    }

    public void setGrabberMiddle() {

    }

    public void setGrabberDown() {

    }

    public void setLeftZiplineUp() {
        leftZipline.setPosition(0.45);
    }

    public void setLeftZiplineDown() {
        leftZipline.setPosition(1.0);
    }

    public void setRightZiplineUp() {
        rightZipline.setPosition(1.0);
    }

    public void setRightZiplineDown() {
        rightZipline.setPosition(0.4);
    }

    public void setMjolnirDown(){
        mjolnir.setPosition(1);
    }

    public void setMjolnirUp(){
        mjolnir.setPosition(0);
    }

    public void hammerTime() throws InterruptedException {
        setMjolnirUp();
        System.out.println("Mjolnir Down");
        Thread.sleep(5000);
        System.out.println("Mjolnir sleep done");
        setMjolnirDown();
        System.out.println("Mjolnir Up");
        Thread.sleep(2000);
        System.out.println("Mjolnir last sleep done");
    }

    public void setLeftLockOpen() {
        leftLock.setPosition(0.2);
    }

    public void setRightLockOpen(){
        rightLock.setPosition(0.98);
    }

    public void setLeftLockClosed(){
        leftLock.setPosition(1.0);
    }

    public void setRightLockClosed(){
        rightLock.setPosition(0.18);
    }

    @Override
    public void setRightHookUp() {
        setRightHookPosition(0.15);
    }

    @Override
    public void setLeftHookUp() {
        setLeftHookPosition(0.90);
    }

    @Override
    public void setRightHookDown() {
        setRightHookPosition(0.9);
    }

    @Override
    public void setLeftHookDown() {
        setLeftHookPosition(0.1);
    }

    @Override
    public void setPushDown(){
        push.setPosition(.7);
    }

    @Override
    public void setPushUp(){
        push.setPosition(0);
    }

    public void initializeServos() {
        setLeftZiplineUp();
        setRightZiplineUp();
        setMjolnirDown();
        setRightHookUp();
        setLeftHookUp();
        setPushUp();

    }

    public void setRightPower(double rightPower) {
        motorRight.setPower(rightPower);
    }

    public void setLeftPower(double leftPower){
        motorLeft.setPower(leftPower);
    }

    @Override
    public void turn(int turnHeading, double power)throws InterruptedException{
        double rightPower;
        double leftPower;
        double cclockwise = gyro.getHeading() - turnHeading;
        double clockwise = turnHeading - gyro.getHeading();

        if (clockwise >= 360){
            clockwise -= 360;
        }
        if (clockwise < 0){
            clockwise += 360;
        }
        if (cclockwise >= 360){
            cclockwise -= 360;
        }
        if (cclockwise < 0){
            cclockwise += 360;
        }

        System.out.println("turn... c="+clockwise+" cc="+cclockwise);



        if(cclockwise > clockwise){
            leftPower=power;
            rightPower=-power;
            motorFrontRight.setPower(rightPower);
            motorFrontLeft.setPower(leftPower);
            motorBackRight.setPower(rightPower);
            motorBackLeft.setPower(leftPower);
            while(Math.abs(gyro.getHeading() - turnHeading) > 2){
                callingOpMode.waitForNextHardwareCycle();
            }
            motorFrontRight.setPower(0);
            motorFrontLeft.setPower(0);
            motorBackRight.setPower(0);
            motorBackLeft.setPower(0);
        }
        else {
            leftPower=-power;
            rightPower=power;
            motorFrontRight.setPower(rightPower);
            motorFrontLeft.setPower(leftPower);
            motorBackRight.setPower(rightPower);
            motorBackLeft.setPower(leftPower);
            while(Math.abs(gyro.getHeading() - turnHeading) > 2){
                //java.lang.Thread.sleep(25);
                callingOpMode.waitForNextHardwareCycle();
            }
            motorFrontRight.setPower(0);
            motorFrontLeft.setPower(0);
            motorBackRight.setPower(0);
            motorBackLeft.setPower(0);
        }
        System.out.println("turn Complete");
        Thread.sleep(100);
    }

    public void driveStraight(double inches, double power, int heading, float direction) throws InterruptedException {
        this.driveStraightEncoder((int) ((inches * inchesToEncoder)/2), power, heading, direction);
    }

    @Override
    public void driveStraightEncoder(int dist, double power, int heading, float direction)throws InterruptedException {
        //gyro stabilization - PID
        float proportionalConst = 0.025f;
        int degErr;
        float correction;
        int lastDegErr;
        double localLeftPower = power;
        double localRightPower = power;

        //time limiting
        double startTime = System.currentTimeMillis();
        System.out.println("Drive start: " + startTime);



        //reset encoders
        encoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        callingOpMode.waitOneFullHardwareCycle();
        //callingOpMode.waitOneFullHardwareCycle();
        callingOpMode.sleep(250);

        encoderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while(encoderMotor.getCurrentPosition()>20) {
            callingOpMode.waitOneFullHardwareCycle();
            System.out.println("Resetting Encoder= " + encoderMotor.getCurrentPosition());
        }

        //set motor Power
        motorFrontRight.setPower(power);
        motorFrontLeft.setPower(power);
        motorBackRight.setPower(power);
        motorBackLeft.setPower(power);

        callingOpMode.waitOneFullHardwareCycle();

        degErr=0;


        System.out.println("Start Encoder= " + encoderMotor.getCurrentPosition());
        System.out.println("Target " + dist);
        while (Math.abs(encoderMotor.getCurrentPosition()) < dist) {
            System.out.println("Encoder= " +encoderMotor.getCurrentPosition());
            lastDegErr = degErr;
            degErr = gyro.getHeading() - heading;
            if (degErr > 180){
                degErr -= 360;
            }
            if (degErr != lastDegErr) {
                correction = degErr * proportionalConst;
                localLeftPower = Range.clip((power - correction) * direction, -1.0f, 1.0f);
                localRightPower = Range.clip((power + correction) * direction, -1.0f, 1.0f);

                motorFrontRight.setPower(localRightPower);
                motorFrontLeft.setPower(localLeftPower);
                motorBackRight.setPower(localRightPower);
                motorBackLeft.setPower(localLeftPower);

                callingOpMode.waitForNextHardwareCycle();

            }
            driveTime = System.currentTimeMillis()-startTime;
            if(driveTime>5000){ //
                break;
            }

        }
        System.out.println("End Encoder= " +encoderMotor.getCurrentPosition());
        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);
        callingOpMode.sleep(500);
        System.out.println("End Encoder= " +encoderMotor.getCurrentPosition());
    }

    @Override
    public void setLeftHookPosition(double position){
        leftHook.setPosition(position);
    }

    @Override
    public void setRightHookPosition(double position){
        rightHook.setPosition(position);
    }

    @Override
    public void updateWinch(float winch) {
        motorWinch.setPower(winch);
    }

    @Override
    public void updateDrawerSlide(float DrawerSlide){
        motorDrawerSlide.setPower(DrawerSlide);
    }
}
