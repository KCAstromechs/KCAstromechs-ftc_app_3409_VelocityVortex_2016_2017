package org.firstinspires.ftc.teamcode;

/**
 * Created by Kevin on 1/29/2017.
 */

public interface AstroDriveBaseInterface {

    void turn(float turnHeading)throws InterruptedException;

    void turn(float turnHeading, double power)throws InterruptedException;

    double zeroOutGyro(double heading);

    void driveStraight (double inches, int heading) throws InterruptedException;

    void driveStraight(double inches, double power, int heading)throws InterruptedException;

}
