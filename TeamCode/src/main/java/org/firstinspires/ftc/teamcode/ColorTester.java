package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.nio.ByteBuffer;


@Autonomous(name="ColorTester", group="ColorTest")
public class ColorTester extends LinearOpMode {

    VuforiaLocalizer vuforia;
    final int boxSize =50;

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
        int pos = 0;
        System.out.println("waiting for start...");
        waitForStart();

        int thisR, thisB, thisG;
        int xRedAvg   =0;
        int xBlueAvg  =0;
        int totalBlue =0;
        int totalRed  =0;
        int xRedSum   =0;
        int xBlueSum  =0;
        int idx = 0;
        float[] hsv = new float[3];
        float thisH, thisS, thisV;

        System.out.println("STARTOFLOOP");

        VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take();
        for (int i = 0; i < frame.getNumImages(); i++){
            if(frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB888){
                idx = i;
                break;
            }
        }

        Image image = frame.getImage(idx);
        ByteBuffer px = image.getPixels();

        System.out.println("rahisodhg");

        for (int i = 0; i < image.getHeight(); i++) {
            for (int j = 0; j < image.getWidth(); j++) {

                thisR = px.get() & 0xFF;
                thisG = px.get() & 0xFF;
                thisB = px.get() & 0xFF;

                Color.RGBToHSV(thisR, thisG, thisB, hsv);

                thisH = hsv[0];
                thisS = hsv[1];
                thisV = hsv[2];


                //We now have the colors (one byte each) for any pixel, (j, i)
                if (thisV < 0.9) { //filters through noise
                    continue;
                }
                if (thisH <= 220 && thisH >= 180) {
                    totalBlue++;
                    xBlueSum += i;
                } else if (thisH <= 360 && thisH >= 330) {
                    totalRed++;
                    xRedSum += i;
                }
            }
        }


        totalRed +=1;
        totalBlue +=1;
        xRedAvg = xRedSum / totalRed;
        xBlueAvg = xBlueSum / totalBlue;

        System.out.println("");
        System.out.println("width="+image.getWidth());
        System.out.println("height="+image.getHeight());
        System.out.println("totalRed="+totalRed);
        System.out.println("totalBlue="+totalBlue);
        System.out.println("xRedSum="+xRedSum);
        System.out.println("xBlueSum="+xBlueSum);
        System.out.println("xRedAvg="+xRedAvg);
        System.out.println("xBlueAvg="+xBlueAvg);

        if (xRedAvg > xBlueAvg) {
            pos =  1;
        } else if (xBlueAvg > xRedAvg) {
            pos = 2;
        } else {
            pos = 0;
        }

        System.out.println(pos);
    }
}
