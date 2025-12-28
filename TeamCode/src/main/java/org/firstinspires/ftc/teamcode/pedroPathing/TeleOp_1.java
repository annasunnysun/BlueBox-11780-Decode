package org.firstinspires.ftc.teamcode.pedroPathing;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.function.Supplier;

@Configurable
@TeleOp(name = "TeleOp 1", group = "Pedro Pathing")
public class TeleOp_1 extends OpMode {
    private Follower follower;
//odometry is i2c bus 0
    //motors
    private DcMotorEx transferMotor, spindexerMotor, intakeMotor, flywheelMotor;
    private CRServo turretServo;
    private Servo arcLeftServo, arcRightServo;
    private NormalizedColorSensor spindexerColorSensor, transferColorSensor;
    private Limelight3A limelight;
    public static Pose startingPose;
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;

    //variable values
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    private double spindexerMotorValue = 0.2;
    private double spindexerReleaseValue = 0.25;
    private double intakeMotorIntakeValue = 1;
    private double transferMotorRotatingValue = -0.4;
    private double transferMotorReleaseValue = 1;
    private double flywheelMotorReleaseFar = -1;
    private double flywheelMotorReleaseNear = -0.5;

    private double arcServoFarRight = 0.405;    //higher hood, higher arc, higher range
    private double arcServoFarLeft = 0.72;    //higher hood, higher arc, higher range
    private double arcServoCloseRight = 0.435;     //lower hood, shorter range, front triangle
    private double arcServoCloseLeft = 0.69;


    //button variables
    boolean topRightBumper1Previous = false;
    boolean topLeftBumper1Previous = false;
    boolean continueSpindexerLoop = false;
    boolean isLaunching = false;
    boolean isIntaking = false;
    boolean isOuttaking = false;


    @Override
    public void init() {

        //hardware map declarations
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");  //2, expansion
        transferMotor = hardwareMap.get(DcMotorEx.class, "transferMotor");  //3, expansion
        spindexerMotor = hardwareMap.get(DcMotorEx.class, "spindexerMotor");    //1, expansion
        turretServo = hardwareMap.get(CRServo.class, "turretServo");    //0, expansion
        arcLeftServo = hardwareMap.get(Servo.class, "arcLeftServo");    //1, expansion
        arcRightServo = hardwareMap.get(Servo.class, "arcRightServo");  //2, expansion
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheelMotor");  //0, expansion
        spindexerColorSensor = hardwareMap.get(NormalizedColorSensor.class, "spindexerColorSensor");    //i2c bus 2
        spindexerColorSensor = hardwareMap.get(NormalizedColorSensor.class, "transferColorSensor");     //i2c bus 1
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        spindexerMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        spindexerMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();

        arcRightServo.setPosition(arcServoCloseRight);
        arcLeftServo.setPosition(arcServoCloseLeft);
    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        limelight.start();
        limelight.pipelineSwitch(0);
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        //variables
        NormalizedRGBA colors = spindexerColorSensor.getNormalizedColors();

        boolean topRightBumper1 = gamepad1.right_bumper;
        boolean topLeftBumper1 = gamepad1.left_bumper;

        follower.update();
        telemetryM.update();

        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors

            //normal TeleOp
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.right_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true // Robot Centric
            );

            //slowMode on
            else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier,
                    true // Robot Centric
            );
        }

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {


        } else {

            telemetry.addLine("No April Tag Scanned");
        }

        //arc hood
        if (gamepad1.xWasPressed()) {
            if(arcRightServo.getPosition()==arcServoFarRight && arcLeftServo.getPosition() == arcServoFarLeft){    //if at servo position 0, set to low arc position
                arcLeftServo.setPosition(arcServoCloseLeft);
                arcRightServo.setPosition(arcServoCloseRight);
            }
            else{
                arcLeftServo.setPosition(arcServoFarLeft);
                arcRightServo.setPosition(arcServoFarRight);
            }
        }

        //intake
        if(gamepad1.aWasPressed()){
            if(intakeMotor.getPower()!=intakeMotorIntakeValue){
                intakeMotor.setPower(intakeMotorIntakeValue);
                isIntaking = true;
                isOuttaking=false;
            }
            else{
                intakeMotor.setPower(0);
                isIntaking = false;
            }
        }

        //outtake
        if(gamepad1.yWasPressed()){
            if(intakeMotor.getPower()!=-1){
                intakeMotor.setPower(-1);
                isOuttaking = true;
                isIntaking=false;
            }
            else{
                intakeMotor.setPower(0);
                isOuttaking = false;
            }
        }


        //collecting spindexer
        if(!isLaunching){
            if (isIntaking) {
                if(colors.red > 0.0006 || colors.green > 0.0009 || colors.blue > 0.0009) {
                    spindexerMotor.setPower(spindexerMotorValue);
                    transferMotor.setPower(transferMotorRotatingValue);
                }
                else{
                    spindexerMotor.setPower(0);
                    transferMotor.setPower(0);
                }
            }
            else if(isOuttaking){
                spindexerMotor.setPower(-spindexerMotorValue);
                transferMotor.setPower(-transferMotorRotatingValue);
            }
            else{
                spindexerMotor.setPower(0);
                transferMotor.setPower(0);
            }
        }


        /*
        if(gamepad1.bWasPressed()){
            if(spindexerMotor.getPower()==0){
                spindexerMotor.setPower(spindexerMotorValue);
                intakeMotor.setPower(intakeMotorRotatingValue);
            }
            else{
                spindexerMotor.setPower(0);
                intakeMotor.setPower(0);
            }
        }
        */

        //flywheel
        if(topRightBumper1){
            if(!topRightBumper1Previous){
                isLaunching = true;
                if(arcLeftServo.getPosition()==arcServoCloseLeft && arcRightServo.getPosition()==arcServoCloseRight){
                    flywheelMotor.setPower(flywheelMotorReleaseNear);
                }
                else{
                    flywheelMotor.setPower(flywheelMotorReleaseFar);
                }
                topRightBumper1Previous = true;
            }
            else{
                isLaunching = false;
                flywheelMotor.setPower(0);
                spindexerMotor.setPower(0);
                transferMotor.setPower(0);
                topRightBumper1Previous = false;
            }
        }

        //pushing ball into flywheel
        if(topLeftBumper1){
            if(!topLeftBumper1Previous){
                transferMotor.setPower(transferMotorReleaseValue);
                spindexerMotor.setPower(spindexerReleaseValue);
                topLeftBumper1Previous = true;
            }
            else{
                transferMotor.setPower(0);
                spindexerMotor.setPower(0);
                topLeftBumper1Previous = false;
            }
        }

        //530 encoder ticks for a full spindexer revolution
        //170 encoder ticks to reach next ball storing location, 336 for 2, 520 for 3
        //testing
        /*
        if(gamepad2.aWasPressed()) {
            isSpindexing = !isSpindexing;
        }

        if (isSpindexing) {
            if (spindexerMotor.getCurrentPosition() % 200 < 35) {
                spindexerMotor.setPower(0.0005);
            } else {
                spindexerMotor.setPower(0.2);
            }
        }
        */

        //Determining the amount of red, green, and blue
        telemetry.addData("Red",  colors.red);
        telemetry.addData("Green",  colors.green);
        telemetry.addData("Blue", colors.blue);
        telemetry.addData("LeftServo", arcLeftServo.getPosition());
        telemetry.addData("RightServo", arcRightServo.getPosition());

        //updating telemetry
        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetry.addData("spindexer encoder: ", spindexerMotor.getCurrentPosition());
        telemetryM.debug("automatedDrive", automatedDrive);

        /*
        //Automated PathFollowing
        if (gamepad1.aWasPressed()) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }

        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }
*/
    }
}