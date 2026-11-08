package c2.mission;

import c2.input.Command;
import c2.input.CommandType;

import java.time.Duration;
import java.time.Instant;

/** Sorties are out in the denied area. Each drone is shown HOLO along its planned route until it reports. */
public class ExecutePhase implements MissionPhase {
    @Override public String name() { return "EXECUTE"; }

    @Override public void enter(MissionController controller) { }

    @Override
    public MissionPhase onTick(MissionController controller, Instant now, Duration elapsed) {
        if (controller.activeSorties().isEmpty()) return new InitPhase();
        boolean allFinished = true;
        for (Sortie sortie : controller.activeSorties()) {
            if (sortie.isFinished()) continue;
            allFinished = false;
            if (!sortie.isLanded()) controller.predictDroneAlongRoute(sortie, now);
            if (!sortie.isLanded() && !now.isBefore(sortie.expectedReturn())) controller.sortieLanded(sortie, now);
            if (sortie.becameOverdue(now)) controller.raiseOverdue(sortie);
            if (sortie.becameLost(now)) controller.writeOffLost(sortie);
        }
        controller.publishSorties();
        return allFinished ? new RegroupPhase() : this;
    }

    @Override
    public MissionPhase onCommand(MissionController controller, Command command) {
        return command.type() == CommandType.HALT ? new HaltedPhase(this) : this;
    }
}
