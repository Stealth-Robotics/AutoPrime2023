package org.firstinspires.ftc.teamcode.subsystems;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import android.util.Size;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.pipelines.PropProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.stealthrobotics.library.Alliance;

public class CameraSubsystem extends SubsystemBase {

    private final VisionPortal portal;
    private final Alliance alliance;
    PropProcessor processor;
    public CameraSubsystem(HardwareMap hardwareMap, Alliance alliance){

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        this.alliance = alliance;
        //inits processor based on specified alliance
        processor = new PropProcessor(alliance);
        WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        //sets camera info using processor
        portal = new VisionPortal.Builder()
                .setCamera(webcam)
                .setCameraResolution(new Size(640, 480))
                .setCamera(BuiltinCameraDirection.BACK)
                .addProcessor(processor)
                .build();


    }

    @Override
    public void periodic() {
        telemetry.addData("position: ", processor.getOutStr());
    }

    public String getConePos(){ return processor.getOutStr(); }

}