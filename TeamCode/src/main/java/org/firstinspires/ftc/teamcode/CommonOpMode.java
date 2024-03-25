package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public abstract class CommonOpMode extends LinearOpMode {

    // constants
    final public double restrictorCap = 1;
    final public double width = 16.063, length = 17.126, diag = Math.hypot(width, length);
    final public double hWidth = width / 2, hLength = length / 2, hDiag = diag / 2, fieldHalf = 36;

    // actuators
    public MecanumDrive drive;
    public PlacementAssembly arm;

    // webcam and rand-related information
    public OpenCvCamera webcam;
    public BingusPipeline pipeline;
    public int rand = 0;
    public DuplexMotor riserMotor;
    public DcMotorEx collectorMotor;
    public DcMotor ledStrip;
    public Servo riserServoA, riserServoB, pusherServo;
    // TODO: изменить максимальную высоту подъёма в соответствии
    long upperArmLimit = 7600;

    // sensors
    public Rev2mDistanceSensor distanceSensor;
    public RevColorSensorV3 lineSensor;
    public RevTouchSensor armLimiter;

    // i/o
    double forward_axis, strafe_axis, turn_axis, riser_axis, collector_axis; // analog inputs
    boolean riserArm, pusher; // digital inputs
    boolean pRiserArm, ppusher; // last-loop digital inputs
    int collector, pCollector;
    double riserPos;
    double restrictor = restrictorCap;

    public void Initialize(HardwareMap hardwareMap) {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        arm = new PlacementAssembly(hardwareMap);
        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "DistanceSensor");
        ledStrip = hardwareMap.get(DcMotor)
        //lineSensor = hardwareMap.get(RevColorSensorV3.class, "LineSensor");

        //lineSensor.initialize();

        //TODO: отрегулировать сервомоторы

    }

    public void safeSleep(int millis) {
        ElapsedTime localTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while (localTime.time() < millis && opModeIsActive() && !isStopRequested()){}
    }

    public int armExtensionToEncoderTicks(double h){
        return (int) -(h/975*2839);
    }

    public double encoderTicksToArmExtension(long t){
        return -t/2879.0*975.0;
    }

    public void camUp(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new BingusPipeline(false); //FIXME: isRed isn't always true
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("error at "+ errorCode);
            }
        });
    }

    public class PlacementAssembly{
        public PlacementAssembly(HardwareMap hardwareMap) {
            double upperBound = 1, lowerBound = 0.14;
            riserMotor = new DuplexMotor((DcMotorEx) hardwareMap.get(DcMotor.class, "riserMotorA"),
                    (DcMotorEx) hardwareMap.get(DcMotor.class, "riserMotorB"));
            collectorMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "collectorMotor");
            riserServoA = hardwareMap.get(Servo.class, "riserServoA");
            riserServoB = hardwareMap.get(Servo.class, "riserServoB");
            pusherServo = hardwareMap.get(Servo.class, "pusherServo");
            armLimiter = hardwareMap.get(RevTouchSensor.class, "armLimiter");

            riserMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            riserMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            collectorMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            collectorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            riserServoA.scaleRange(lowerBound, upperBound);
            riserServoB.scaleRange(lowerBound, upperBound);
            pusherServo.scaleRange(0.4, 0.45);
            riserServoA.setDirection(Servo.Direction.REVERSE); //BREAKING CHANGE, FIX IF NEEDED ================================================
        }

        public class ChangePos implements Action {
            int position;
            boolean init = false;
            public ChangePos(int pos){
                this.position = pos;
            }
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(!init){
                    init = true;
                    riserMotor.setPower(1);
                    riserMotor.setTargetPosition(position);
                    riserMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

                if(riserMotor.isBusy()){
                    return true;
                }
                else{
                    riserMotor.setPower(0);
                    return false;
                }
            }
        }

        public Action changePos(int pos){
            return new ChangePos(pos);
        }

        public class ChangeCassetteTilt implements Action {
            int pos;

            public ChangeCassetteTilt(int a){
                this.pos = a;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket){
                riserServoA.setPosition(pos);
                riserServoB.setPosition(pos);
                return false;
            }
        }

        public Action changeCassetteTilt(int pos){
            return new ChangeCassetteTilt(pos);
        }

        public class ChangePusherState implements Action {
            boolean state; // false = closed, true = open

            public ChangePusherState(boolean a){
                this.state = a;
            }
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                pusherServo.setPosition(state?1:0);
                return false;
            }
        }

        public Action changePusherState(boolean state){
            return new ChangePusherState(state);
        }

        public class DispenseViaCollector implements Action {
            ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            boolean started = false;
            public boolean run(@NonNull TelemetryPacket telemetryPacket){
                collectorMotor.setPower(0.1);
                if(!started){timer.reset(); started = true;}
                return timer.time() < 200;
            }
        }

        public Action dispenseViaCollector() { return new DispenseViaCollector();}
    }
}