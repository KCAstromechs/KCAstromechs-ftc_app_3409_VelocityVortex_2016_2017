package org.firstinspires.ftc.teamcode;

import android.hardware.Camera;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Kevin on 1/7/2016.
 */
public interface AstroRobotBaseInterface {
    float getZRotation();
    double getLastPicBeaconAvg();

    final int PIXELS_PER_INCH = 35;

    final int PIXELS_PER_DEGREE = 22;

    double lastPicBeaconAvg =0;

    void initCallingOpMode(LinearOpMode _callingOpMode);

    void initTeleop(HardwareMap ahwMap);

    void init(HardwareMap ahwMap);

    void resetGyro();

    void turn(float turnHeading)throws InterruptedException;

    void turn(float turnHeading, double power)throws InterruptedException;

    double zeroOutGyro(double heading);

    void driveStraight (double inches, int heading) throws InterruptedException;

    void driveStraight(double inches, double power, int heading)throws InterruptedException;

    void hanShotFirst() throws InterruptedException;

    int takePicture() throws InterruptedException;

    void outputZAxis() throws InterruptedException;

    void pushButton (int heading, int outHeading, double timeOutSec) throws InterruptedException, TimeoutException;

    void deconstruct();

    void teleopUpdateDrive(float left_stick, float right_stick);

    void shooterHandler(boolean y, boolean lb, boolean rb);

    void spinnerToggle(boolean a, boolean x);

    void teleopUpdateLifter(float left_stick, float right_stick);

    void grabberOutREPLACE(boolean a);

    void grabberMidREPLACE(boolean x);

    void grabberInREPLACE(boolean b);
}
