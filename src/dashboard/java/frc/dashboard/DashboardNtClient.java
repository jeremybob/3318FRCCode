package frc.dashboard;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;

public class DashboardNtClient implements AutoCloseable {

    public enum DashboardCommand {
        ZERO_HEADING,
        STOP_DRIVE,
        INTAKE_HOME,
        ALIGN_SHOOT,
        FALLBACK_SHOOT,
        LEVEL1_CLIMB
    }

    private final NetworkTableInstance nt = NetworkTableInstance.create();
    private final NetworkTable table = nt.getTable("Dashboard");

    private final StringSubscriber modeSub = table.getStringTopic("robot/mode").subscribe("UNKNOWN");
    private final BooleanSubscriber enabledSub = table.getBooleanTopic("robot/enabled").subscribe(false);
    private final StringSubscriber allianceSub = table.getStringTopic("robot/alliance").subscribe("UNKNOWN");
    private final DoubleSubscriber matchTimeSub = table.getDoubleTopic("robot/match_time_sec").subscribe(-1.0);
    private final DoubleSubscriber robotTimestampSub = table.getDoubleTopic("robot/timestamp_sec").subscribe(0.0);

    private final DoubleSubscriber poseXSub = table.getDoubleTopic("drive/pose_x_m").subscribe(0.0);
    private final DoubleSubscriber poseYSub = table.getDoubleTopic("drive/pose_y_m").subscribe(0.0);
    private final DoubleSubscriber headingSub = table.getDoubleTopic("drive/heading_deg").subscribe(0.0);

    private final DoubleSubscriber shooterLeftSub = table.getDoubleTopic("shooter/left_rps").subscribe(0.0);
    private final DoubleSubscriber shooterRightSub = table.getDoubleTopic("shooter/right_rps").subscribe(0.0);
    private final BooleanSubscriber shooterAtSpeedSub = table.getBooleanTopic("shooter/at_speed").subscribe(false);

    private final BooleanSubscriber intakeHomedSub = table.getBooleanTopic("intake/homed").subscribe(false);
    private final BooleanSubscriber intakeLimitSub = table.getBooleanTopic("intake/limit_switch_pressed").subscribe(false);
    private final DoubleSubscriber intakeTiltSub = table.getDoubleTopic("intake/tilt_deg").subscribe(0.0);
    private final DoubleSubscriber intakeRollerCurrentSub =
            table.getDoubleTopic("intake/roller_current_amps").subscribe(0.0);

    private final DoubleSubscriber feederCurrentSub = table.getDoubleTopic("feeder/current_amps").subscribe(0.0);
    private final DoubleSubscriber hopperCurrentSub = table.getDoubleTopic("hopper/current_amps").subscribe(0.0);

    private final BooleanSubscriber climberArmedSub = table.getBooleanTopic("climber/armed").subscribe(false);
    private final DoubleSubscriber climberPositionSub = table.getDoubleTopic("climber/position_rot").subscribe(0.0);
    private final DoubleSubscriber climberCurrentSub = table.getDoubleTopic("climber/current_amps").subscribe(0.0);

    private final StringSubscriber alignStateSub = table.getStringTopic("align/state").subscribe("IDLE");
    private final BooleanSubscriber alignCommandActiveSub =
            table.getBooleanTopic("align/command_active").subscribe(false);
    private final BooleanSubscriber alignHasTargetSub = table.getBooleanTopic("align/has_target").subscribe(false);
    private final BooleanSubscriber alignGeometrySub =
            table.getBooleanTopic("align/geometry_feasible").subscribe(false);
    private final BooleanSubscriber alignShootableSub =
            table.getBooleanTopic("align/has_shootable_target").subscribe(false);
    private final DoubleSubscriber alignYawSub = table.getDoubleTopic("align/yaw_deg").subscribe(Double.NaN);
    private final DoubleSubscriber alignPitchSub = table.getDoubleTopic("align/pitch_deg").subscribe(Double.NaN);
    private final StringSubscriber alignAbortSub = table.getStringTopic("align/last_abort_reason").subscribe("");

    private final BooleanSubscriber readySub = table.getBooleanTopic("shot/ready").subscribe(false);
    private final StringSubscriber readyReasonSub = table.getStringTopic("shot/ready_reason").subscribe("");

    private final StringSubscriber ackCommandSub = table.getStringTopic("ack/last_command").subscribe("");
    private final StringSubscriber ackStatusSub = table.getStringTopic("ack/last_status").subscribe("");
    private final IntegerSubscriber ackSeqSub = table.getIntegerTopic("ack/last_seq").subscribe(0);
    private final StringSubscriber ackMessageSub = table.getStringTopic("ack/message").subscribe("");
    private final DoubleSubscriber ackTimestampSub = table.getDoubleTopic("ack/timestamp_sec").subscribe(0.0);

    private final IntegerPublisher zeroHeadingPub = table.getIntegerTopic("cmd/zero_heading_seq").publish();
    private final IntegerPublisher stopDrivePub = table.getIntegerTopic("cmd/stop_drive_seq").publish();
    private final IntegerPublisher intakeHomePub = table.getIntegerTopic("cmd/intake_home_seq").publish();
    private final IntegerPublisher alignShootPub = table.getIntegerTopic("cmd/align_shoot_seq").publish();
    private final IntegerPublisher fallbackShootPub = table.getIntegerTopic("cmd/fallback_shoot_seq").publish();
    private final IntegerPublisher level1ClimbPub = table.getIntegerTopic("cmd/level1_climb_seq").publish();

    private long zeroHeadingSeq = 0;
    private long stopDriveSeq = 0;
    private long intakeHomeSeq = 0;
    private long alignShootSeq = 0;
    private long fallbackShootSeq = 0;
    private long level1ClimbSeq = 0;

    public DashboardNtClient(String clientName, int teamNumber, String hostOverride) {
        nt.startClient4(clientName);
        if (teamNumber > 0) {
            nt.setServerTeam(teamNumber);
        } else {
            nt.setServer(hostOverride);
        }
    }

    public DashboardData read() {
        return new DashboardData(
                nt.isConnected(),
                modeSub.get(),
                enabledSub.get(),
                allianceSub.get(),
                matchTimeSub.get(),
                robotTimestampSub.get(),
                poseXSub.get(),
                poseYSub.get(),
                headingSub.get(),
                shooterLeftSub.get(),
                shooterRightSub.get(),
                shooterAtSpeedSub.get(),
                intakeHomedSub.get(),
                intakeLimitSub.get(),
                intakeTiltSub.get(),
                intakeRollerCurrentSub.get(),
                feederCurrentSub.get(),
                hopperCurrentSub.get(),
                climberArmedSub.get(),
                climberPositionSub.get(),
                climberCurrentSub.get(),
                alignStateSub.get(),
                alignCommandActiveSub.get(),
                alignHasTargetSub.get(),
                alignGeometrySub.get(),
                alignShootableSub.get(),
                alignYawSub.get(),
                alignPitchSub.get(),
                alignAbortSub.get(),
                readySub.get(),
                readyReasonSub.get(),
                ackCommandSub.get(),
                ackStatusSub.get(),
                ackSeqSub.get(),
                ackMessageSub.get(),
                ackTimestampSub.get());
    }

    public synchronized void sendCommand(DashboardCommand command) {
        switch (command) {
            case ZERO_HEADING -> zeroHeadingPub.set(++zeroHeadingSeq);
            case STOP_DRIVE -> stopDrivePub.set(++stopDriveSeq);
            case INTAKE_HOME -> intakeHomePub.set(++intakeHomeSeq);
            case ALIGN_SHOOT -> alignShootPub.set(++alignShootSeq);
            case FALLBACK_SHOOT -> fallbackShootPub.set(++fallbackShootSeq);
            case LEVEL1_CLIMB -> level1ClimbPub.set(++level1ClimbSeq);
            default -> throw new IllegalStateException("Unhandled command " + command);
        }
    }

    @Override
    public void close() {
        nt.close();
    }
}
