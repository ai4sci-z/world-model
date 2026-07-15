package tasks

import (
	"fmt"
	"sort"
	"strings"

	"navlab/orchestration-sim/internal/config"
	"navlab/orchestration-sim/internal/tasks/helpers"
)

const (
	ProfileIdeal                 = "ideal"
	ProfileRealistic             = "realistic"
	ProfileSlamDirect            = "slam-direct"
	ProfileSlamDirectNoOdomPrior = "slam-direct-no-odom-prior"
	// GATE-4b root-cause bisection arms (names owned by the helpers package
	// because the execution plan adds per-profile runtime services).
	ProfileGPSEKFServices          = helpers.HoverProfileGPSEKFServices
	ProfileTruthExternalNav        = helpers.HoverProfileTruthExternalNav
	ProfileIMUFLUCorrection        = helpers.HoverProfileIMUFLUCorrection
	ProfilePurposeLegacyDebug      = "legacy-debug"
	ProfilePurposeDiagnostic       = "diagnostic"
	ProfilePurposeMainline         = "mainline"
	ExternalNavInputDefault        = "default-selector-candidate"
	ExternalNavInputDirectSlamOdom = "direct-slam-odom"
	ExternalNavInputGazeboTruth    = "gazebo-truth-odom"
	FCUParamProfileGPSBaseline     = "gps-baseline"
	IMUSourceCorrectionRoll180FLU  = "roll180_flu"
)

type HoverSimulationProfile struct {
	Name                       string
	Purpose                    string
	AllowTasks                 []string
	ExternalNavInputOdomMode   string
	CartographerConfigBasename string
	FCUParamProfile            string
	IMUSourceCorrection        string
	Mainline                   bool
}

func HoverSimulationProfiles() []HoverSimulationProfile {
	profiles := []HoverSimulationProfile{
		{
			Name:                       ProfileIdeal,
			Purpose:                    ProfilePurposeLegacyDebug,
			AllowTasks:                 []string{"hover", "hover-slam-only"},
			ExternalNavInputOdomMode:   ExternalNavInputDefault,
			CartographerConfigBasename: helpers.HoverCartographerConfigBasename,
		},
		{
			Name:                       ProfileSlamDirect,
			Purpose:                    ProfilePurposeDiagnostic,
			AllowTasks:                 []string{"hover", "hover-slam-only"},
			ExternalNavInputOdomMode:   ExternalNavInputDirectSlamOdom,
			CartographerConfigBasename: helpers.HoverCartographerConfigBasename,
		},
		{
			Name:                       ProfileSlamDirectNoOdomPrior,
			Purpose:                    ProfilePurposeMainline,
			AllowTasks:                 []string{"hover", "hover-slam-only"},
			ExternalNavInputOdomMode:   ExternalNavInputDirectSlamOdom,
			CartographerConfigBasename: helpers.HoverNoOdomPriorConfigBasename,
			Mainline:                   true,
		},
		// GATE-4b arm L1: full service stack, FCU flies the official GPS EKF.
		{
			Name:                       ProfileGPSEKFServices,
			Purpose:                    ProfilePurposeDiagnostic,
			AllowTasks:                 []string{"hover"},
			ExternalNavInputOdomMode:   ExternalNavInputDirectSlamOdom,
			CartographerConfigBasename: helpers.HoverNoOdomPriorConfigBasename,
			FCUParamProfile:            FCUParamProfileGPSBaseline,
		},
		// GATE-4b arm L1.5: identical feed mechanism, truth content.
		{
			Name:                       ProfileTruthExternalNav,
			Purpose:                    ProfilePurposeDiagnostic,
			AllowTasks:                 []string{"hover"},
			ExternalNavInputOdomMode:   ExternalNavInputGazeboTruth,
			CartographerConfigBasename: helpers.HoverNoOdomPriorConfigBasename,
		},
		// GATE-4b arm L2-fix: mainline feed, IMU mount convention corrected
		// before Cartographer.
		{
			Name:                       ProfileIMUFLUCorrection,
			Purpose:                    ProfilePurposeDiagnostic,
			AllowTasks:                 []string{"hover", "hover-slam-only"},
			ExternalNavInputOdomMode:   ExternalNavInputDirectSlamOdom,
			CartographerConfigBasename: helpers.HoverNoOdomPriorConfigBasename,
			IMUSourceCorrection:        IMUSourceCorrectionRoll180FLU,
		},
	}
	sort.Slice(profiles, func(i, j int) bool { return profiles[i].Name < profiles[j].Name })
	return profiles
}

func hoverSimulationProfileForTask(taskID string, profileName string) (HoverSimulationProfile, bool) {
	for _, profile := range HoverSimulationProfiles() {
		if profile.Name == profileName && stringInSlice(taskID, profile.AllowTasks) {
			return profile, true
		}
	}
	return HoverSimulationProfile{}, false
}

func AllowedProfilesForTask(taskID string) []string {
	allowed := []string{}
	for _, profile := range HoverSimulationProfiles() {
		if stringInSlice(taskID, profile.AllowTasks) {
			allowed = append(allowed, profile.Name)
		}
	}
	sort.Strings(allowed)
	return allowed
}

func applyHoverSimulationProfile(runtimeConfig config.TaskRuntimeConfig, profile HoverSimulationProfile) config.TaskRuntimeConfig {
	switch profile.ExternalNavInputOdomMode {
	case ExternalNavInputDirectSlamOdom:
		runtimeConfig.SlamHover.ExternalNavInputOdomTopic = runtimeConfig.SlamHover.SlamOdomTopic
	case ExternalNavInputGazeboTruth:
		runtimeConfig.SlamHover.ExternalNavInputOdomTopic = helpers.GazeboTruthOdomTopic
		runtimeConfig.SlamHover.UsesGazeboTruthAsInput = true
	}
	if profile.CartographerConfigBasename != "" {
		runtimeConfig.SlamBackend.CartographerConfigurationBasename = profile.CartographerConfigBasename
	}
	if profile.FCUParamProfile != "" {
		runtimeConfig.FCUParamProfile = profile.FCUParamProfile
	}
	if profile.IMUSourceCorrection != "" {
		runtimeConfig.SlamHover.IMUSourceCorrection = profile.IMUSourceCorrection
	}
	return runtimeConfig
}

func ApplySimulationProfile(runtimeConfig config.TaskRuntimeConfig, plan Plan) (config.TaskRuntimeConfig, error) {
	if isHoverSLAMProfileTask(plan.TaskID) {
		if plan.SimulationProfile == "" {
			return runtimeConfig, nil
		}
		profile, ok := hoverSimulationProfileForTask(plan.TaskID, plan.SimulationProfile)
		if !ok {
			return runtimeConfig, invalidSimulationProfileError(plan.TaskID, plan.SimulationProfile, AllowedProfilesForTask(plan.TaskID))
		}
		return applyHoverSimulationProfile(runtimeConfig, profile), nil
	}
	if plan.TaskID != "scan-robustness" || plan.SimulationProfile == "" {
		return runtimeConfig, nil
	}
	if !stringInSlice(plan.SimulationProfile, runtimeConfig.AirframeDisturbanceGate.ProfileSet) {
		return runtimeConfig, invalidSimulationProfileError(plan.TaskID, plan.SimulationProfile, runtimeConfig.AirframeDisturbanceGate.ProfileSet)
	}
	runtimeConfig.AirframeDisturbance.Profile = plan.SimulationProfile
	return runtimeConfig, nil
}

func invalidSimulationProfileError(taskID string, profileName string, allowed []string) error {
	choices := append([]string(nil), allowed...)
	sort.Strings(choices)
	return fmt.Errorf("simulation profile %q is not valid for task %q; allowed profiles: %s", profileName, taskID, strings.Join(choices, ", "))
}

func isHoverSLAMProfileTask(taskID string) bool {
	switch taskID {
	case "hover", "hover-slam-only":
		return true
	default:
		return false
	}
}

func stringInSlice(value string, values []string) bool {
	for _, candidate := range values {
		if candidate == value {
			return true
		}
	}
	return false
}
