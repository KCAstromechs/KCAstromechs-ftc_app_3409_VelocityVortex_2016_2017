package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.nio.ByteBuffer;

@TeleOp(name="VuforiaTest", group="Vuforia")
public class EuphoriaTesting extends LinearOpMode {

    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() throws InterruptedException {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "Ac8xsqH/////AAAAGcG2OeE2NECwo7mM5f9KX1RKmDT79NqkIHc/ATgW2+loN9Fr8fkfb6jE42RZmiRYeei1FvM2M3kUPdl53j" +
                                       "+oeuhahXi7ApkbRv9cef0kbffj+4EkWKWCgQM39sRegfX+os6PjJh1fwGdxxijW0CYXnp2Rd1vkTjIs/cW2/7TFTtuJTkc17l" +
                                       "+FNJAeqLEfRnwrQ0FtxvBjO8yQGcLrpeKJKX/+sN+1kJ/cvO345RYfPSoG4Pi+wo/va1wmhuZ/WCLelUeww8w8u0douStuqcuz" +
                                       "ufrsWmQThsHqQDfDh0oGKZGIckh3jwCV2ABkP0lT6ICBDm4wOZ8REoyiY2kjsDnnFG6cT803cfzuVuPJl+uGTEf";
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB888, true);
        vuforia.setFrameQueueCapacity(1);

        waitForStart();

        int thisR, thisB, thisG;

        while (opModeIsActive()) {

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

            /*for(int y=0; y<image.getHeight(); y++) {

                System.out.println("");
                for(int x=0; x<image.getWidth(); x++){
                    int r = px.get() & 0xFF;
                    int g = px.get() & 0xFF;
                    int b = px.get() & 0xFF;

                    if (y == image.getHeight()/2) {
                        System.out.println(r + "," + g + "," + b + "  ;  " + x);
                    }
                }

            }

            return;*/


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
                        //System.out.print(thisR + " ");
                        //System.out.print(thisG + " ");
                        //System.out.print(thisB + " ");
                        //System.out.println("Current pixel is Blue");
                    } else if (thisR > thisB) {
                        totalRed++;
                        xRedSum += j;
                        //System.out.print(thisR + " ");
                        //System.out.print(thisG + " ");
                        //System.out.print(thisB + " ");
                        //System.out.println("Current pixel is Red");
                    }
                }
            }

            xRedAvg = xRedSum / totalRed;
            xBlueAvg = xBlueSum / totalBlue;

            //System.out.println("redAvg >" + xRedAvg);
            //System.out.println("blueAvg >" + xBlueAvg);
            //System.out.println("blueTotal >" + totalBlue);
            //System.out.println("redTotal >" + totalRed);
            //System.out.println("blueSum >" + xBlueSum);
            //System.out.println("redSum >" + xRedSum);

            if (xRedAvg > xBlueAvg) {
                System.out.println("Blue, Red");
            } else if (xBlueAvg > xRedAvg) {
                System.out.println("Red, Blue");
            } else {
                System.out.println("idk");
            }

        }
            /*StringBuffer line = new StringBuffer();
            for(int i = 0; i < myImage.getHeight(); i++){
                line.delete(0,line.length());
                for(int j = 0; j < myImage.getStride(); j++){
                    byte pixel = myImage.getPixels().get();
                    line.append(pixel);
                }
                System.out.println(line);
            }

            sleep(2000);


            System.out.println("");
            System.out.println("************************************************************************");
            System.out.println("");
            */


    }
}
