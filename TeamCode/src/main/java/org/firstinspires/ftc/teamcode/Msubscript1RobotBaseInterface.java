package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;


public interface Msubscript1RobotBaseInterface {

    void init(HardwareMap ahwMap);

    void resetGyro();

    void turn(int turnHeading)throws InterruptedException;

    void turn(int turnHeading, double power)throws InterruptedException;

    void driveStraight (double inches, int heading) throws InterruptedException;

    void driveStraight(double inches, double power, int heading)throws InterruptedException;

    void hanShotFirst() throws InterruptedException;

}
