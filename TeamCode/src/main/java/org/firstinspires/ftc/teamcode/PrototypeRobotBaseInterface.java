package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;


public interface PrototypeRobotBaseInterface {

    void init(HardwareMap ahwMap);

    void resetGyro();

    void turn(float turnHeading)throws InterruptedException;

    void turn(float turnHeading, double power)throws InterruptedException;

    public double zeroOutGyro(double heading);

    void driveStraight (double inches, int heading) throws InterruptedException;

    void driveStraight(double inches, double power, int heading)throws InterruptedException;

    void hanShotFirst() throws InterruptedException;

    int takePicture() throws InterruptedException;

    void outputZAxis() throws InterruptedException;

    void pushLeft(int heading) throws InterruptedException;

    void pushRight(int heading) throws InterruptedException;
}
