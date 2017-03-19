package org.firstinspires.ftc.teamcode;

/**
 * Created by N2Class1 on 3/19/2017.
 */

public enum BeaconOrientation {
    RED_BLUE_RED_BLUE, //MEDIUM DRIVE
    RED_BLUE_BLUE_RED, //if blue, SHORT DRIVE, if red, LONG DRIVE
    BLUE_RED_RED_BLUE, //if blue LONG DRIVE, if red SHORT DRIVE
    BLUE_RED_BLUE_RED, //MEDIUM DRIVE
    ORIENTATION_UNKNOWN; //TAKES TWO SHOTS
}
