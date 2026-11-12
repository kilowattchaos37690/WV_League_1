package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;

/**
 * Drive, launcher, and intake on the REV hub's built-in motor PID.
 *
 * TeleOp uses {@link DcMotor.RunMode#RUN_USING_ENCODER} (velocity PID).
 * Autonomous distance uses {@link DcMotor.RunMode#RUN_TO_POSITION}, which uses
 * that same velocity PID plus the hub's position P. This class never calls
 * {@code setVelocityPIDFCoefficients} or {@code setPositionPIDFCoefficients}.
 *
 * Config names (see {@code test_config.xml}): {@code left_front}, {@code left_back},
 * {@code right_front}, {@code right_back}, {@code launcher}, {@code intake}.
 * The motor type in the config sets the hub's max ticks/sec for {@code setPower}.
 *
 * In Blocks: call {@link #initRobot()} once. In Java OpModes: call
 * {@link #bindFromLinearOpMode} first, then {@link #initRobot()}.
 */
public class RobotEncoderRevPID extends BlocksOpModeCompanion {

  /** NeveRest 40 counts per motor revolution (typical). */
  private static final double COUNTS_PER_MOTOR_REV = 1120.0;
  /** Drive gearing: 1.0 if the wheel is on the motor shaft. */
  private static final double DRIVE_GEAR = 1.0;
  /** Wheel diameter inches — measure and update. */
  private static final double WHEEL_DIAMETER_IN = 4.0;
  private static final double COUNTS_PER_INCH =
      (COUNTS_PER_MOTOR_REV * DRIVE_GEAR) / (Math.PI * WHEEL_DIAMETER_IN);
  /** Rough inches of wheel travel for a 90° spin in place — tune on the mat. */
  private static final double INCHES_PER_90_DEG_TURN = 12.0;

  /**
   * TeleOp stick feel inside {@link #driveArcade}.
   * 1.0 = full stick → full speed command; cubic softens small stick moves.
   */
  private static final double TELEOP_SCALE = 0.55;
  /** 1 = linear; 3 = cubic (gentle near center, full at the stick ends). */
  private static final int TELEOP_EXPO = 3;

  private static final int POS_TOLERANCE = 20;
  private static final double DEFAULT_TIMEOUT_SEC = 5.0;
  /**
   * Soft-start for encoder Auto: seconds to ramp 0 → commanded power (0 = off).
   */
  private static final double POWER_RAMP_SEC = 0.20;

  private static DcMotorEx left_front;
  private static DcMotorEx left_back;
  private static DcMotorEx right_front;
  private static DcMotorEx right_back;
  private static DcMotorEx launcher;
  private static DcMotorEx intake;
  private static boolean ready = false;

  /** Call from Java LinearOpModes so these methods see that OpMode's hardware. */
  public static void bindFromLinearOpMode(com.qualcomm.robotcore.eventloop.opmode.LinearOpMode op) {
    opMode = op;
    linearOpMode = op;
    hardwareMap = op.hardwareMap;
    telemetry = op.telemetry;
    gamepad1 = op.gamepad1;
    gamepad2 = op.gamepad2;
  }

  @ExportToBlocks(
      heading = "Robot",
      comment = "Init once — drive, launcher, intake on hub default PID",
      tooltip = "Call at the start of Auto or TeleOp")
  public static void initRobot() {
    left_front = hardwareMap.get(DcMotorEx.class, "left_front");
    left_back = hardwareMap.get(DcMotorEx.class, "left_back");
    right_front = hardwareMap.get(DcMotorEx.class, "right_front");
    right_back = hardwareMap.get(DcMotorEx.class, "right_back");
    launcher = hardwareMap.get(DcMotorEx.class, "launcher");
    intake = hardwareMap.get(DcMotorEx.class, "intake");

    prepareClosedLoop(left_front, DcMotor.Direction.REVERSE);
    prepareClosedLoop(left_back, DcMotor.Direction.REVERSE);
    prepareClosedLoop(right_front, DcMotor.Direction.FORWARD);
    prepareClosedLoop(right_back, DcMotor.Direction.FORWARD);
    prepareClosedLoop(launcher, DcMotor.Direction.FORWARD);
    prepareClosedLoop(intake, DcMotor.Direction.FORWARD);

    setDrivePower(0);
    launcher.setPower(0);
    intake.setPower(0);
    ready = true;
  }

  /**
   * Encoder zero, then velocity PID. Coefficients stay at the hub defaults.
   */
  private static void prepareClosedLoop(DcMotorEx m, DcMotor.Direction direction) {
    m.setDirection(direction);
    m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    m.setTargetPositionTolerance(POS_TOLERANCE);
    m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
  }

  private static void ensureReady() {
    if (!ready) {
      initRobot();
    }
  }

  // -------------------------------------------------------------------------
  // Drive
  // -------------------------------------------------------------------------

  @ExportToBlocks(
      heading = "Drive",
      comment = "Drive forward (+) or back (−) using encoders + hub PID",
      parameterLabels = {"Inches", "Max power 0..1"})
  public static void driveForward(double inches, double power) {
    ensureReady();
    encoderDrive(inches, inches, inches, inches, power, DEFAULT_TIMEOUT_SEC);
  }

  @ExportToBlocks(
      heading = "Drive",
      comment = "Strafe right (+) or left (−) — mecanum",
      parameterLabels = {"Inches", "Max power 0..1"})
  public static void strafeRight(double inches, double power) {
    ensureReady();
    encoderDrive(inches, -inches, -inches, inches, power, DEFAULT_TIMEOUT_SEC);
  }

  @ExportToBlocks(
      heading = "Drive",
      comment = "Spin right (+) or left (−) degrees (encoder estimate — tune)",
      parameterLabels = {"Degrees", "Max power 0..1"})
  public static void turnDegrees(double degrees, double power) {
    ensureReady();
    double inches = (degrees / 90.0) * INCHES_PER_90_DEG_TURN;
    encoderDrive(inches, inches, -inches, -inches, power, DEFAULT_TIMEOUT_SEC);
  }

  @ExportToBlocks(
      heading = "Drive",
      comment = "TeleOp mecanum on RUN_USING_ENCODER (hub velocity PID)",
      parameterLabels = {"Forward", "Strafe", "Turn"})
  public static void driveArcade(double forward, double strafe, double turn) {
    ensureReady();
    setDriveRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    forward = shapeStick(forward);
    strafe = shapeStick(strafe);
    turn = shapeStick(turn);
    double fl = forward + strafe + turn;
    double bl = forward - strafe + turn;
    double fr = forward - strafe - turn;
    double br = forward + strafe - turn;
    double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(bl), Math.max(Math.abs(fr), Math.abs(br)))));
    left_front.setPower(fl / max);
    left_back.setPower(bl / max);
    right_front.setPower(fr / max);
    right_back.setPower(br / max);
  }

  /** Expo plus overall scale so small stick moves are less twitchy. */
  private static double shapeStick(double v) {
    double s = Math.max(-1.0, Math.min(1.0, v));
    if (TELEOP_EXPO == 3) {
      s = s * s * s;
    } else if (TELEOP_EXPO > 1) {
      s = Math.copySign(Math.pow(Math.abs(s), TELEOP_EXPO), s);
    }
    return s * TELEOP_SCALE;
  }

  @ExportToBlocks(
      heading = "Drive",
      comment = "Stop drive. Leaves motors in RUN_USING_ENCODER (velocity hold at 0)")
  public static void stopDrive() {
    ensureReady();
    setDrivePower(0);
    setDriveRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
  }

  // -------------------------------------------------------------------------
  // Launcher + intake
  // -------------------------------------------------------------------------

  @ExportToBlocks(
      heading = "Launcher",
      comment = "Launcher speed −1..1 on the hub velocity PID",
      parameterLabels = {"Power"})
  public static void launcherPower(double power) {
    ensureReady();
    launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    launcher.setPower(clamp11(power));
  }

  @ExportToBlocks(heading = "Launcher", comment = "Stop launcher (velocity command 0)")
  public static void launcherStop() {
    ensureReady();
    launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    launcher.setPower(0);
  }

  @ExportToBlocks(
      heading = "Launcher",
      comment = "Run launcher at power for N seconds, then stop",
      parameterLabels = {"Power −1..1", "Seconds"})
  public static void launcherForSeconds(double power, double seconds) {
    runForSeconds(launcher, power, seconds);
  }

  @ExportToBlocks(
      heading = "Intake",
      comment = "Intake speed −1..1 on the hub velocity PID",
      parameterLabels = {"Power"})
  public static void intakePower(double power) {
    ensureReady();
    intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    intake.setPower(clamp11(power));
  }

  @ExportToBlocks(heading = "Intake", comment = "Stop intake (velocity command 0)")
  public static void intakeStop() {
    ensureReady();
    intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    intake.setPower(0);
  }

  @ExportToBlocks(
      heading = "Intake",
      comment = "Run intake at power for N seconds, then stop",
      parameterLabels = {"Power −1..1", "Seconds"})
  public static void intakeForSeconds(double power, double seconds) {
    runForSeconds(intake, power, seconds);
  }

  @ExportToBlocks(heading = "Robot", comment = "Stop drive, launcher, and intake")
  public static void stopAll() {
    stopDrive();
    launcherStop();
    intakeStop();
  }

  /**
   * Encoder ticks and ticks/sec for every motor. Call from a TeleOp loop after
   * {@link #bindFromLinearOpMode} so this writes to that OpMode's telemetry.
   */
  public static void reportEncoders() {
    ensureReady();
    if (telemetry == null) {
      return;
    }
    reportOne("LF", left_front);
    reportOne("LB", left_back);
    reportOne("RF", right_front);
    reportOne("RB", right_back);
    reportOne("launcher", launcher);
    reportOne("intake", intake);
  }

  private static void reportOne(String name, DcMotorEx m) {
    telemetry.addData(name, "%d ticks   %.0f t/s", m.getCurrentPosition(), m.getVelocity());
  }

  // -------------------------------------------------------------------------
  // Internals
  // -------------------------------------------------------------------------

  private static void encoderDrive(
      double lfIn, double lbIn, double rfIn, double rbIn, double power, double timeoutSec) {
    int newLf = left_front.getCurrentPosition() + (int) Math.round(lfIn * COUNTS_PER_INCH);
    int newLb = left_back.getCurrentPosition() + (int) Math.round(lbIn * COUNTS_PER_INCH);
    int newRf = right_front.getCurrentPosition() + (int) Math.round(rfIn * COUNTS_PER_INCH);
    int newRb = right_back.getCurrentPosition() + (int) Math.round(rbIn * COUNTS_PER_INCH);

    left_front.setTargetPosition(newLf);
    left_back.setTargetPosition(newLb);
    right_front.setTargetPosition(newRf);
    right_back.setTargetPosition(newRb);

    setDriveRunMode(DcMotor.RunMode.RUN_TO_POSITION);
    double p = Math.abs(power);

    ElapsedTime timer = new ElapsedTime();
    while (opModeIsActiveCompat()
        && timer.seconds() < timeoutSec
        && (left_front.isBusy() || left_back.isBusy() || right_front.isBusy() || right_back.isBusy())) {
      double pNow = p;
      if (POWER_RAMP_SEC > 0 && timer.seconds() < POWER_RAMP_SEC) {
        pNow = p * (timer.seconds() / POWER_RAMP_SEC);
      }
      setDrivePower(pNow);
      idleCompat();
    }
    // Stay in RUN_TO_POSITION so the next drive segment can retarget without a mode reset.
    // stopDrive() returns the wheels to RUN_USING_ENCODER.
    setDrivePower(0);
  }

  private static void runForSeconds(DcMotorEx motor, double power, double seconds) {
    ensureReady();
    motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    motor.setPower(clamp11(power));
    ElapsedTime timer = new ElapsedTime();
    while (opModeIsActiveCompat() && timer.seconds() < seconds) {
      idleCompat();
    }
    motor.setPower(0);
  }

  private static void setDrivePower(double p) {
    left_front.setPower(p);
    left_back.setPower(p);
    right_front.setPower(p);
    right_back.setPower(p);
  }

  private static void setDriveRunMode(DcMotor.RunMode mode) {
    left_front.setMode(mode);
    left_back.setMode(mode);
    right_front.setMode(mode);
    right_back.setMode(mode);
  }

  private static boolean opModeIsActiveCompat() {
    return linearOpMode == null || linearOpMode.opModeIsActive();
  }

  private static void idleCompat() {
    if (linearOpMode != null) {
      linearOpMode.idle();
    }
  }

  private static double clamp11(double v) {
    if (v < -1) return -1;
    if (v > 1) return 1;
    return v;
  }
}
