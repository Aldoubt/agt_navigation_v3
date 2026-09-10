# Functional-domain migration plan

1. Commit this audit and baseline.
2. Create domain directories and move only launch/config wrappers, retaining
   package names and compatibility launch shims.
3. Relocate state-estimation packages, then localization, Nav2, map manager,
   mapping/cleaning/sensor; build affected packages after every commit.
4. Add `agt_bringup` only as a compatibility-preserving total-launch wrapper.
5. Remove nothing until `rg`, package discovery, launch parsing and regression
   prove it unreferenced.

No new universal manager/adapter/base class is planned. Future interface names
may use `/state_estimation/*`, `/localization/*`, `/navigation/*`, but existing
topic names remain the V1 contract.
