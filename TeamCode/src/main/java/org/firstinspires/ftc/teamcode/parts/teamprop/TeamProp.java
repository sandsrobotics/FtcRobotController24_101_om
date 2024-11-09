package org.firstinspires.ftc.teamcode.parts.teamprop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.lang3.ObjectUtils;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraException;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import om.self.ezftc.core.Robot;
import om.self.ezftc.core.part.LoopedPartImpl;

public class TeamProp extends LoopedPartImpl<Robot, ObjectUtils.Null, ObjectUtils.Null> {
    OpenCvCamera camera;
    private VisionPortal visionPortal;
    public TeamPropDetectionPipeline pipeline;
    public TeamPropDetectionPipeline.PixelPosition pixPos;

    public TeamProp(Robot parent) {
        super(parent, "team prop");
    }

    @Override
    public void onBeanLoad() {}

    @Override
    public void onInit() {
//        visionPortal

        HardwareMap hardwareMap = parent.opMode.hardwareMap;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        FtcDashboard.getInstance().startCameraStream(camera, 10);
        pipeline = new TeamPropDetectionPipeline();
        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1280,720, OpenCvCameraRotation.SENSOR_NATIVE);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
    }

    @Override
    public void onStart() {
        pipeline.position = pipeline.getAnalysis();
    }

    @Override
    public void onRun() {
//        if(pixPos != pipeline.pixelPosition)
        pixPos = pipeline.getPixAnalysis();
        pipeline.pixelPosition = pipeline.getPixAnalysis();
    }

    @Override
    public void onStop() {
        if (camera.getClass() != null) {
        }
        try {
            if (camera != null) {
                camera.stopStreaming();
                camera.closeCameraDevice();
            }
        } catch (OpenCvCameraException E) {}
    }
}
