# mobile_manipulator_capabilities

Capability-layer package reserved for generic mobile-manipulator policy.

Planned contents:

- task-feasibility runners that call `mobile_manipulator/CheckTaskFeasibility`
- fallback policy runners that escalate from arm-only to base-plus-arm execution
- robot-agnostic orchestration that delegates platform details to robot-specific adapters
