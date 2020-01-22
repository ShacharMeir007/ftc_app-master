package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


@TeleOp(name = "movement")

public class movement extends LinearOpMode
{
    //values for object detection
    private static final String VUFORIA_KEY = "AUdml4//////AAABmeZVu2rww0Djk9HZI25Yl2tK9IRIqatToj5noMuDOFM8oAOwJ9sHqM2u1mEMS/7A4erXo2FAtiOKfHEPmkjKjVY0HFwcSihMHyAlJLPY+3BLatpNd87YkM1ONfjF9RqYKh4eZ5tQQZcvIZWddUWvdmogf2DAIzW518d5oxRtf88tdoBGpRKsn4Zi8D3WBdBYK68vLmkh6jDqSvQbp9yjvnj8u0RWGy6ZU1F9aVqvpa7nXJDVLxYdyrxcvW/dei3VJ8uqhTAKpEv2FHD39kJNiG0ftpghdmd5JiHprU/sgbc18XUuQQ983aW9zYl10ZYM0xMkcyl0BLcxLWxc8vZC5tpSFl138+NGFP8TEYyTITt4";
    private static final String SKYSTONE = "skystone";
    private static final String STONE = "stone";
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final int SIDES = 2;
    private static final double THRESHOLD_LEFT = 270;
    private static final double THRESHOLD_RIGHT = 400;
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
            double power = 0;
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

                while (flag)
                {
                    if (tfod != null)
                    {
                        // getUpdatedRecognitions() will return null if no new information is available since
                        // the last time that call was made.
                        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                        if (updatedRecognitions != null)
                        {
                            telemetry.addData("# Object Detected", updatedRecognitions.size());
                            telemetry.addData("power",power);
                            power += 1;
                            for(Recognition recognition : updatedRecognitions)
                            {
                                double recognitionSideAvg = (recognition.getRight()+recognition.getLeft())/SIDES;
                                if(recognition.getLabel().equals(SKYSTONE) && recognitionSideAvg >= THRESHOLD_LEFT && recognitionSideAvg >= THRESHOLD_RIGHT)
                                {
                                    power = 0;
                                    telemetry.addData("skystone","found");
                                    telemetry.addData("recognitionSideAvg=",recognitionSideAvg);
                                }
                            }
                            telemetry.update();
                        }
                    }
                }
            }
        }
    }


    class driveThread extends Thread
    {
        String motor1Name;
        String motor2Name;
        String motor3Name;
        String motor4Name;
        double motor1Val;
        double motor2Val;
        double motor3Val;
        double motor4Val;
        DcMotor motor1;
        DcMotor motor2;
        DcMotor motor3;
        DcMotor motor4;
        boolean flag = true;
        driveThread(String NameMotor1,String NameMotor2,String NameMotor3,String NameMotor4)
        {
            motor1Name = NameMotor1;
            motor2Name = NameMotor2;
            motor3Name = NameMotor3;
            motor4Name = NameMotor4;

            motor1 = hardwareMap.dcMotor.get(motor1Name);
            motor2 = hardwareMap.dcMotor.get(motor2Name);
            motor3 = hardwareMap.dcMotor.get(motor3Name);
            motor4 = hardwareMap.dcMotor.get(motor4Name);
        }

        void setFlag(boolean newState)
        {
            flag = newState;
        }

        Double normelize(Double value)
        {
            double maxSpeed = 1;
            if(value > maxSpeed) value = maxSpeed;
            else if (value < -maxSpeed) value = -maxSpeed;
            return value;
        }

        Double sumValues(double val1, double val2 , double val3)
        {
            return val1+val2+val3;
        }


        public void run()
        {
            double speed = 1;
            do
            {
                //fast mode
                if(gamepad1.left_bumper)
                {
                    speed = 0.8;
                }

                //slow mode
                if(gamepad1.right_bumper)
                {
                    speed = 0.3;
                }

                //break mode
                if(gamepad1.right_stick_button)
                {
                    motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    motor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
                //float mode
                else
                {
                    motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    motor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                }
                //movement function
                /*FL*/motor1Val = (normelize(sumValues(-gamepad1.left_stick_y,gamepad1.right_stick_x,gamepad1.left_stick_x)))*speed;
                /*FR*/motor2Val = (normelize(sumValues(gamepad1.left_stick_y,gamepad1.right_stick_x,gamepad1.left_stick_x)))*speed;
                /*BL*/motor3Val = (normelize(sumValues(-gamepad1.left_stick_y,gamepad1.right_stick_x,-gamepad1.left_stick_x)))*speed;
                /*BR*/motor4Val = (-normelize(sumValues(gamepad1.left_stick_y,gamepad1.right_stick_x,-gamepad1.left_stick_x)))*speed;

                if(motor1Val==-1 && motor3Val == 1)
                {
                    motor2Val = 1;
                    motor4Val = -1;
                }


                if(motor1Val==-1 && motor3Val == 1)
                {
                    motor2Val = -1;
                    motor4Val = -1;
                }

                else if(motor1Val == 1 && motor3Val == -1)
                {
                    motor2Val = 1;
                    motor4Val = 1;
                }
                motor1.setPower(motor1Val);
                motor2.setPower(motor2Val);
                motor3.setPower(motor3Val);
                motor4.setPower(motor4Val);
            }while (flag);

        }
    }



    @Override
    public void runOpMode () throws InterruptedException
    {
        driveThread drive = new driveThread("FL","FR","BL","BR");
        drive.start();
        waitForStart();
        //noinspection StatementWithEmptyBody
        while (opModeIsActive());
        drive.setFlag(false);
    }
}
