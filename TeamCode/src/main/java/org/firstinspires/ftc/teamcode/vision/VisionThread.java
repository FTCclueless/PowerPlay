package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.util.RobotLogger;
import org.firstinspires.ftc.teamcode.util.SafeSleep;

import java.util.List;

public class VisionThread extends Thread {
    private String TAG = "VisionThread";
    private TFODWrapper tfodWrapper;
    private LinearOpMode opMode;
    public boolean keepRunning;

    public VisionThread(String model_file, String[] labels, boolean webCamera, Telemetry tele, HardwareMap hwMap)
    {
        this.setName("VisionThread");
        RobotLogger.dd(TAG, "Vision thread constructor" + getName());
        tfodWrapper = new TFODWrapper(model_file, labels, webCamera, tele, hwMap);
        tfodWrapper.init();
        keepRunning = true;
    }
    public void setOpMode(LinearOpMode opmode){
        opMode = opmode;
    }

    public void setCameraFindControlFlag(boolean flag) {
        tfodWrapper.setCameraFindControlFlag(flag);
    }
    public void stopRunning() {
        keepRunning = false;
    }
    // called when tread.start is called. thread stays in loop to do what it does until exit is
    // signaled by main code calling thread.interrupt.

    @Override
    public void run()
    {
        RobotLogger.dd(TAG, "Starting thread %s", this.getName());
        try
        {
            while (keepRunning == true && !isInterrupted())
            {
                tfodWrapper.start();
                SafeSleep.sleep_milliseconds(opMode,50);             // slow down the main while() loop
            }
            throw new InterruptedException("interrupt!");
        }
        // interrupted means time to shutdown. note we can stop by detecting isInterrupted = true
        // or by the interrupted exception thrown from the sleep function.
        catch (InterruptedException e) {
            e.printStackTrace();
            RobotLogger.dd(TAG, "%s interrupted", this.getName());
        }
        // an error occurred in the run loop.
        catch (Exception e) {
            e.printStackTrace();
        }
        tfodWrapper.stop();
        RobotLogger.dd(TAG, "end of thread %s", this.getName());
    }
    public List<Recognition> getDetectionsUpdate() {
        return tfodWrapper.getDetectionsUpdate();
    }
    public TFObjectDetector getTFOD() {
        return tfodWrapper.getTFOD();
    }
}
