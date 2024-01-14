package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.*;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class VisionAuto {
    private TfodProcessor tfod;
    private VisionPortal visionPortal;

    public VisionAuto(HardwareMap hardwareMap) {
        tfod = TfodProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), tfod);
        visionPortal.setProcessorEnabled(tfod, false);
        visionPortal.stopLiveView(); // Save CPU resources
    }

    public String objectInFront(){
        //visionPortal.resumeStreaming();
        visionPortal.setProcessorEnabled(tfod, true);
        List<Recognition> currentRecognitions = tfod.getRecognitions();

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            if(recognition.getLabel().equals("Pixel")) {
                visionPortal.setProcessorEnabled(tfod, false);
                if (x >= 100 && x <= 200) { // Center
                    return "Center";
                } else if (x < 100) { // Left
                    return "Left";
                }
                // Right
                return "Right";
            }
        }

        visionPortal.setProcessorEnabled(tfod, false);
        return "None";
    }

    public void stop() {
        visionPortal.stopStreaming();
        visionPortal.close();
    }
}
