package org.firstinspires.ftc.teamcode.subsystems;


import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.pipelines.TeamPropDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class CameraSubsystem extends SubsystemBase {
    private final OpenCvCamera webcam;
    private final TeamPropDetection pipeline;

    private static final int CAMERA_WIDTH = 320;
    private static final int CAMERA_HEIGHT = 240;

    public CameraSubsystem(HardwareMap hardwareMap) {
        //initialize the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new TeamPropDetection();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                // This will be called if the camera could not be opened
            }
        });
    }

    public int GetConePosition() {
        return pipeline.GetConePosition();
    }


    @Override
    public void periodic() {
        telemetry.addData("Camera fps", webcam.getFps());
        telemetry.addData("Parking Position", GetConePosition());
    }
}
