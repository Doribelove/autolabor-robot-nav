# N05 selection-only adapter

Source: `workspace/scripts/astra_gazebo/route_preview_selection_n05.py`.

- Method: `astra_analytic_preview_eligible_n05`.
- Observation/identity schema: `astra_route_preview_selection_observation_n05_v1`.
- Selection trace schema: `astra_analytic_preview_eligibility_n05_v1`.
- Pure function: `select_preview_eligible(observation, full_preview_clearances_m) -> Selection`. `full_preview_clearances_m` must be the eight original **unrounded** verifier outputs in action order. `Selection.legacy_result()` matches the inherited `(chosen, proposed, reason, actor_latency_ms)` tuple; `audit()` supplies the separate selection eligibility.
- Optional integration class: `PreviewEligibleRuntime(grid, route, urdf_xml, config=None)`. It inherits the frozen core, rejects any actor/checkpoint, captures the exact existing 16 verification returns, and adds two keys to `build`: `full_preview_clearances_m`, `preview_selection_audit`. All original build keys/array values remain unchanged. It overrides analytic `select`; `command` and its fresh backup checks remain inherited.

Use this class only for the treatment. Control remains the exact original `RouteAwareRuntime`. Importing this module does not modify/monkeypatch the original class. A caller must serialize `build` and `command` as the original node does. The capture validates exact `(action, preview/backup)` order and backup values; exceptions clear the capture and propagate. There are no new rollout/geometry queries in this wrapper.

Record both `observation.action_mask` (genuine backup eligibility) and `preview_selection_audit.selection_eligibility` (selection filter). Do not overwrite/serialize the latter as the former. Keep `original_analytic`, new chosen/proposed, and exact full values in the trace. Full-preview stop does not enter the extra filter: stop retains backup eligibility and can win the unchanged prior. If no backup passes, the old `no_prediction_model_backup_pass` reason survives; if some moving backups pass but all moving previews fail and stop fails, new reason `no_full_preview_eligible_action` returns zero proposal. The existing command path must still revalidate stop; returned zero is not a certificate.

The zero threshold is applied to original verifier float values, not the normalized float32 candidate feature. A small negative such as −1e−50 can round to signed zero in that feature; an explicit counterexample confirms the new selector still rejects it using the original value. Existing candidate features/prior are retained for exact factor isolation. The existing verifier includes both static geometry and the frozen dynamic bound when applicable; N05 does not silently change that prediction into static-only verification.

The node/collector/runner/release are root-owned and not changed here. Integration must publish the new method/identity and additional decision fields, preserve the common terminal/command authority, and undergo independent readiness review. No new simulator episodes, seed reservation, release, learned checkpoint or performance qualification were created. N04/M08 sources remain frozen.

Tests are 19 small offline cases (4 ms unittest body). The real inherited `build` is exercised with deterministic synthetic geometry returns to check byte-equal original arrays and 16 verifier calls per build. This is not a live geometry, full-planner latency or closed-loop qualification; no benchmark was run during N04.
