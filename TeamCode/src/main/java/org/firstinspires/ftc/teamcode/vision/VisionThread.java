package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.RobotLogger;

public class VisionThread extends Thread {
    private String TAG = "VisionThread";
    private TFODWrapper tfod;
    public VisionThread(String model_file, String[] labels, boolean webCamera, Telemetry tele, HardwareMap hwMap)
    {
        this.setName("VisionThread");
        RobotLogger.dd(TAG, "Vision thread constructor" + getName());
        tfod = new TFODWrapper(model_file, labels, webCamera, tele, hwMap);
        tfod.init();
    }

    // called when tread.start is called. thread stays in loop to do what it does until exit is
    // signaled by main code calling thread.interrupt.
    @Override
    public void run()
    {
        RobotLogger.dd(TAG, "Starting thread %s", this.getName());
        try
        {
            while (!isInterrupted())
            {
                tfod.start();
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
        tfod.stop();
        RobotLogger.dd(TAG, "end of thread %s", this.getName());
    }

    public TFODWrapper getTFODWrapper() {
        return tfod;
    }
}
