package tasks

import (
	"fmt"

	"navlab/orchestration-sim/internal/config"
)

func ApplyHoverSLOPolicy(runtimeConfig config.TaskRuntimeConfig, plan Plan) (config.TaskRuntimeConfig, error) {
	if !isHoverSLAMProfileTask(plan.TaskID) {
		return runtimeConfig, nil
	}
	if plan.HoverSpanTargetM > 0 {
		runtimeConfig.SlamHover.HoverSpanTargetM = plan.HoverSpanTargetM
	}
	if plan.HoverSpanHardCapM > 0 {
		runtimeConfig.SlamHover.HoverSpanHardCapM = plan.HoverSpanHardCapM
	}
	if err := config.NormalizeHoverSLOPolicy(&runtimeConfig.SlamHover); err != nil {
		return runtimeConfig, fmt.Errorf("hover SLO policy: %w", err)
	}
	return runtimeConfig, nil
}

// ApplyExplorationStrategyOverride applies the --exploration-strategy run
// override so harnesses select frontier_lite vs external at run time instead
// of editing the tracked task YAML in place (Review 001 P0-3).
func ApplyExplorationStrategyOverride(runtimeConfig config.TaskRuntimeConfig, plan Plan) (config.TaskRuntimeConfig, error) {
	if plan.ExplorationStrategy == "" {
		return runtimeConfig, nil
	}
	if plan.TaskID != "exploration" {
		return runtimeConfig, fmt.Errorf("--exploration-strategy is only valid for task \"exploration\", got %q", plan.TaskID)
	}
	switch plan.ExplorationStrategy {
	case "frontier_lite", "external":
	default:
		return runtimeConfig, fmt.Errorf("unknown exploration strategy %q (allowed: frontier_lite, external)", plan.ExplorationStrategy)
	}
	runtimeConfig.ExplorationGate.Strategy = plan.ExplorationStrategy
	return runtimeConfig, nil
}
