package org.firstinspires.ftc.teamcode;

import android.hardware.Camera;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Kevin on 1/7/2016.
 */
public interface AstroRobotBaseInterface {

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

    void pushButton (int heading) throws InterruptedException;
}
