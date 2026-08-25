package tasks

import "navlab/orchestration-sim/internal/config"

const (
	explorationDDSDiscoveryBudgetSec    = 15.0
	// The external adapter keeps the 120 s active window spatially strict and
	// allows 3 s for the 4 Hz controller to observe a final in-radius sample.
	externalExplorationStageBudgetSec   = 123.0
	explorationRuntimePublisherGraceSec = 5.0
	explorationWatchdogMarginSec        = 10.0
	explorationProbeContainerMarginSec  = 30.0
)

// explorationStageBudgetSec reserves a bounded window for the selected
// planner. The in-process frontier_lite implementation follows the configured
// window; an external planner needs enough time to complete its own three-goal
// gate before return-home and landing consume their separate budgets.
func explorationStageBudgetSec(runtimeConfig config.TaskRuntimeConfig) float64 {
	budgetSec := runtimeConfig.ExplorationGate.ExplorationWindowSec
	if budgetSec <= 0 {
		budgetSec = 26.0
	}
	if runtimeConfig.ExplorationGate.Strategy == "external" && budgetSec < externalExplorationStageBudgetSec {
		return externalExplorationStageBudgetSec
	}
	return budgetSec
}

func explorationRuntimeServiceTimeoutSec(runtimeConfig config.TaskRuntimeConfig) float64 {
	return explorationSpec(runtimeConfig).ProbeTimeoutSec + explorationRuntimePublisherGraceSec
}

func explorationProbeContainerTimeoutSec(runtimeConfig config.TaskRuntimeConfig, durationSec float64) float64 {
	timeoutSec := probeTimeoutSec("exploration_probe", durationSec)
	candidate := explorationSpec(runtimeConfig).ProbeTimeoutSec + explorationProbeContainerMarginSec
	if candidate > timeoutSec {
		return candidate
	}
	return timeoutSec
}
