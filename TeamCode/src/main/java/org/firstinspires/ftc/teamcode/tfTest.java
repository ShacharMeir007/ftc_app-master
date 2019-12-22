package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@TeleOp(name = "tf test")

public class tfTest extends LinearOpMode
{
    private static final String VUFORIA_KEY = "AUdml4//////AAABmeZVu2rww0Djk9HZI25Yl2tK9IRIqatToj5noMuDOFM8oAOwJ9sHqM2u1mEMS/7A4erXo2FAtiOKfHEPmkjKjVY0HFwcSihMHyAlJLPY+3BLatpNd87YkM1ONfjF9RqYKh4eZ5tQQZcvIZWddUWvdmogf2DAIzW518d5oxRtf88tdoBGpRKsn4Zi8D3WBdBYK68vLmkh6jDqSvQbp9yjvnj8u0RWGy6ZU1F9aVqvpa7nXJDVLxYdyrxcvW/dei3VJ8uqhTAKpEv2FHD39kJNiG0ftpghdmd5JiHprU/sgbc18XUuQQ983aW9zYl10ZYM0xMkcyl0BLcxLWxc8vZC5tpSFl138+NGFP8TEYyTITt4";
    private static final String SKYSTONE = "skystone";
    private static final String STONE = "stone";
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;


    class objectDetection extends Thread
    {
        boolean flag;

        objectDetection(boolean Flag)
        {
            flag = Flag;
        }


        private void setFlag(boolean newFlag)
        {
            flag = newFlag;
        }

        private void initVuforia()
        {
            /*
             * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
             */
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

            parameters.vuforiaLicenseKey = VUFORIA_KEY;
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

            //  Instantiate the Vuforia engine
            vuforia = ClassFactory.getInstance().createVuforia(parameters);

            // Loading trackables is not necessary for the TensorFlow Object Detection engine.
        }

        /**
         * Initialize the TensorFlow Object Detection engine.
         */
        private void initTfod()
        {
            int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, STONE, SKYSTONE);
        }

        @Override
        public void run()
        {
            // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
            // first.
            initVuforia();

            if (ClassFactory.getInstance().canCreateTFObjectDetector())
            {
                initTfod();
            } else
            {
                telemetry.addData("Sorry!", "This device is not compatible with TFOD");
            }


            if (flag)
            {
                /** Activate TensorFlow Object Detection. */

                if (tfod != null)
                {
                    tfod.activate();
                }

                while (opModeIsActive())
                {
                    if (tfod != null)
                    {
                        // getUpdatedRecognitions() will return null if no new information is available since
                        // the last time that call was made.
                        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                        if (updatedRecognitions != null)
                        {
                            telemetry.addData("# Object Detected", updatedRecognitions.size());
                            for(Recognition recognition : updatedRecognitions)
                            {
                                if(recognition.getLabel().equals(SKYSTONE))
                                {
                                    telemetry.addData("skystone","found");
                                    telemetry.addData("left val",recognition.getLeft());
                                }
                            }
                            telemetry.update();
                        }
                    }
                }
            }
        }
    }
    @Override
    public void runOpMode () throws InterruptedException
    {
        objectDetection test = new objectDetection(true);
        test.start();
        waitForStart();
        //noinspection StatementWithEmptyBody
        while (opModeIsActive())
        {

        }
            test.setFlag(false);
    }
}

