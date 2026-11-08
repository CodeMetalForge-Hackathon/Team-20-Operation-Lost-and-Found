package c2.mission;

import c2.input.Command;
import c2.input.CommandType;

import java.time.Duration;
import java.time.Instant;

/** Sortie is out in the denied area. The drone is shown HOLO along its planned route until it reports. */
public class ExecutePhase implements MissionPhase {
    @Override public String name() { return "EXECUTE: sortie in denied area"; }

    @Override public void enter(MissionController controller) { }

    @Override
    public MissionPhase onTick(MissionController controller, Instant now, Duration elapsed) {
        Sortie sortie = controller.currentSortie();
        if (sortie == null) return new InitPhase();
        if (sortie.isReportReceived()) return new RegroupPhase();
        controller.predictDroneAlongRoute(now);
        if (!sortie.isLanded() && !now.isBefore(sortie.expectedReturn())) controller.sortieLanded(now);
        if (sortie.becameOverdue(now)) controller.raiseOverdue(sortie);
        return this;
    }

    @Override
    public MissionPhase onCommand(MissionController controller, Command command) {
        return command.type() == CommandType.HALT ? new HaltedPhase(this) : this;
    }
}
