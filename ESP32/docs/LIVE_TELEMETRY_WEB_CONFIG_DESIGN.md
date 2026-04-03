# Live telemetry web configuration UI — detailed design

## Purpose

Add a configuration and diagnostics surface to the existing live telemetry web dashboard without turning it into a dangerous free-form command console.

The operator should be able to:

- tune the active controller mode and key balance parameters,
- configure the active IMU filter and its exposed parameters,
- perform basic motor diagnostics,
- see a terminal-style live log pane,
- inspect useful read-only status panels,
- reboot the ESP32 when needed.

The UI must **not** expose arbitrary command entry in the first iteration.

---

## Scope agreed for first implementation

### In scope

- Existing live telemetry charts remain the default landing view.
- Add a configuration view for:
	- controller mode (`PID` / `LQR`),
	- PID gains,
	- LQR gains,
	- adaptive trim enable/disable,
	- LQR command filters,
	- LQR yaw gains,
	- deadband (read + manual set),
	- min command,
	- motor gain compensation.
- Add a filter view for:
	- active filter,
	- available filters,
	- supported filter params (`ALPHA`, `KACC`, `KBIAS`, `BETA`) when available.
- Add a motor diagnostics view for:
	- enable motors,
	- disable motors,
	- set independent left/right normalized commands,
	- read-only motor direction inversion,
	- read-only enabled state / IDs.
- Add a terminal-style logs tab.
- Add simple read-only diagnostics panels for:
	- system snapshot,
	- motor status,
	- filter status,
	- controller status.
- Add an ESP reboot action.

### Explicitly out of scope for now

- Autotune UI.
- Tuning capture/stream UI.
- Free-form command textbox.
- IMU reinit button.
- Wi-Fi credential edition from the dashboard.
- Trim/deadband/start-threshold calibration assistants.
- Advanced motor low-level controls such as invert changes, raw driver commands, ACK toggles, scan/dump/register editing.

---

## Source of truth in current firmware

Current menu/command families still actively registered in `src/serial_commands.cpp`:

- `WIFI`
- `MOTOR`
- `BALANCE`
- `AUTOTUNE`
- `CALIB`
- `FILTER`
- `TUNING`
- `LOG`
- `FUSION`
- `SYS`

For the web UI v1, the useful active subsets are:

- `BALANCE`
- `FILTER`
- `MOTOR`
- `LOG`
- `SYS`
- read-only parts of `FUSION`

Relevant runtime APIs already exist in firmware:

- controller mode get/set in `include/balancer_controller.h`
- LQR config getters via `CascadedLqrStrategy::getConfig()`
- filter list/current selection via `FilterService`
- filter parameter `getParam()` / `setParam()` on active filter
- motor enabled state / inversion / IDs via `IMotorDriver`

This makes a structured web API feasible without invasive controller redesign.

---

## Design goals

1. **No free-form command UI** in normal operation.
2. **Strong separation** between charts, settings, logs, and diagnostics.
3. **Structured reads**, preferably machine-readable from firmware.
4. **Command profiles** for writes instead of raw string assembly scattered through the UI.
5. **Safe motor diagnostics** with explicit enable/disable and bounded normalized commands.
6. **Terminal-style logs** that preserve operator context without forcing them to open another client.

---

## UX information architecture

The current page is chart-first. The dashboard will evolve into a tabbed interface.

## Top-level tabs

### 1. Live

Keep the current telemetry chart dashboard as the default view.

Contains:

- existing metrics cards,
- existing chart visibility controls,
- existing live charts.

### 2. Settings

Primary runtime configuration surface.

Sections:

- **Controller**
	- strategy selector (`PID` / `LQR`)
	- PID gains form
	- LQR gains form
	- adaptive trim toggle
	- LQR filter form
	- LQR yaw form
	- deadband / min_cmd / motor_gains form
- **Filters**
	- current filter dropdown
	- supported parameter inputs for current filter
- **Motors**
	- enable / disable actions
	- left and right command inputs
	- quick stop / zero commands
- **System**
	- reboot button

### 3. Logs

Terminal-style live console panel.

Contains:

- scrolling log buffer,
- clear button,
- auto-scroll toggle,
- optional channel/prefix filters later,
- command-result lines annotated separately from passive logs.

### 4. Info

Read-only operational snapshot.

Contains:

- controller summary,
- filter summary,
- motor summary,
- system summary,
- log channel state summary.

This keeps maintenance/status information visible without crowding the Settings tab.

---

## Proposed layout changes

Current layout is two-column:

- left sidebar,
- right content.

Proposed evolution:

- keep the left sidebar for global status and quick metrics,
- add a top tab bar in the main content area,
- each tab renders its own content panel,
- the `Live` tab reuses the existing chart grid,
- `Settings`, `Logs`, and `Info` are separate panels hidden/shown by tab state.

### Sidebar content remains

- connection status,
- compact live metrics,
- chart visibility controls,
- maybe a small “active profile summary” block later.

This preserves the telemetry-first workflow while making configuration reachable.

---

## Backend architecture changes

The current backend can already send one-off console commands through `/api/command`, but it cannot yet support a polished UI because it lacks:

- a structured settings API,
- persistent live console log streaming,
- stable command profiles,
- a server-owned snapshot cache.

## New backend services

### 1. `ConsoleSession`

New long-lived service in the live telemetry backend.

Responsibilities:

- maintain a persistent TCP connection to the robot console,
- continuously read log lines,
- keep a ring buffer of recent log lines,
- send profiled commands over the same session,
- wait for structured machine-readable responses when needed,
- hide reconnect logic from the HTTP layer.

This replaces the dashboard’s dependency on only short-lived `RobotConsoleClient` usage for configuration flows.

### 2. `CommandProfileService`

Maps typed dashboard intents to safe console commands.

Examples:

- `set_balance_strategy("LQR")` → `BALANCE STRATEGY LQR`
- `set_lqr_gains({...})` → `BALANCE LQR GAINS ...`
- `set_filter_param("BETA", 0.02)` → `FILTER SET BETA 0.02`
- `set_motor_commands(left, right)` → `MOTOR SET LEFT ...`, `MOTOR SET RIGHT ...`
- `reboot_system()` → `SYS REBOOT`

This service owns:

- validation,
- formatting,
- post-write refresh of the structured snapshot.

### 3. `DashboardConfigSnapshot`

Server-side structured state cache built from firmware-readable responses.

Holds:

- controller settings,
- filter state,
- motor status,
- system info,
- log channel states,
- last successful refresh time.

### 4. `LogBuffer`

Simple bounded ring buffer for console lines.

Each item should contain at least:

- `seq`
- `timestamp`
- `text`
- `kind` (`log`, `command`, `response`, `error`, `api`)

---

## Why machine-readable firmware responses are worth adding

Current console commands mostly emit human-readable lines.

That is fine for an operator terminal, but fragile for a dashboard because parsing text like:

- `LQR: adaptive trim is ON`
- `MOTOR_GAINS: L=1.000 R=0.950`
- `FILTER: current=MADGWICK`

creates avoidable coupling to exact wording.

### Chosen approach

Add **structured read commands** in firmware, but keep existing text commands for human operators.

This gives the web dashboard:

- reliable parsing,
- flexibility for future UI additions,
- no need for free-text scraping logic all over the Python backend.

### Important design choice

For **writes**, the dashboard can continue to use existing human commands through command profiles.

For **reads**, firmware should expose a structured snapshot command.

This minimizes firmware churn while still solving the parsing problem.

---

## Proposed firmware command surface

Introduce a new command family dedicated to the web/dashboard API.

Suggested prefix:

- `WEBUI`

This avoids colliding with operator commands and makes log filtering easier.

## Required read command

### `WEBUI SNAPSHOT`

Returns one JSON line prefixed with a fixed marker.

Example output:

```text
WEBUI_JSON: {"controller":{...},"filter":{...},"motors":{...},"system":{...},"logs":{...}}
```

The backend only needs to parse lines starting with `WEBUI_JSON:`.

### Snapshot payload shape

```json
{
	"controller": {
		"mode": "LQR",
		"pid": {"kp": 0.0, "ki": 0.0, "kd": 0.0},
		"lqr": {
			"gains": {"k_pitch": 0.1, "k_gyro": 0.04, "k_dist": 0.0, "k_speed": 0.0},
			"adaptive_trim_enabled": true,
			"filters": {"pitch_rate_lpf_hz": 0.0, "cmd_lpf_hz": 0.0},
			"yaw": {"k_yaw": 0.0, "k_yaw_rate": 0.0}
		},
		"deadband": 0.0,
		"min_cmd": 0.0,
		"motor_gains": {"left": 1.0, "right": 1.0}
	},
	"filter": {
		"current": "MADGWICK",
		"available": ["MADGWICK", "COMPLEMENTARY1D", "KALMAN1D"],
		"params": {
			"ALPHA": null,
			"KACC": null,
			"KBIAS": null,
			"BETA": 0.0219
		},
		"supported": ["BETA"]
	},
	"motors": {
		"enabled": false,
		"left": {"id": 1, "inverted": false, "last_command": 0.0},
		"right": {"id": 2, "inverted": true, "last_command": 0.0}
	},
	"system": {
		"wifi_console_enabled": true,
		"telemetry_udp_active": true,
		"heap_bytes": 123456
	},
	"logs": {
		"enabled_channels": ["DEFAULT", "MOTOR", "BALANCER"]
	}
}
```

### Why one snapshot command first

- one round-trip,
- simpler backend,
- easier UI bootstrap,
- future fields can be added compatibly.

## Optional future structured reads

Only if the snapshot grows too large or expensive:

- `WEBUI CONTROLLER`
- `WEBUI FILTERS`
- `WEBUI MOTORS`
- `WEBUI SYSTEM`

But v1 should start with `WEBUI SNAPSHOT` only.

---

## Dashboard HTTP API design

## Read endpoints

### `GET /api/state`

Existing telemetry endpoint. Keep unchanged.

### `GET /api/config`

Returns the latest cached structured configuration snapshot.

### `GET /api/logs?since=<seq>`

Returns buffered console lines since a sequence number.

Response example:

```json
{
	"items": [
		{"seq": 101, "timestamp": 1710000000.1, "kind": "log", "text": "BALANCER: started"},
		{"seq": 102, "timestamp": 1710000000.3, "kind": "response", "text": "WEBUI_JSON: {...}"}
	],
	"next_seq": 103
}
```

### `GET /api/info`

Optional convenience alias for read-only snapshot display if we want to separate UI concerns. Can be deferred.

## Write endpoints

### `POST /api/settings/controller/mode`

Payload:

```json
{"mode":"PID"}
```

### `POST /api/settings/controller/pid`

```json
{"kp":0.1,"ki":0.0,"kd":0.01}
```

### `POST /api/settings/controller/lqr/gains`

```json
{"k_pitch":0.1,"k_gyro":0.04,"k_dist":0.0,"k_speed":0.0}
```

### `POST /api/settings/controller/lqr/trim`

```json
{"enabled":true}
```

### `POST /api/settings/controller/lqr/filters`

```json
{"pitch_rate_lpf_hz":0.0,"cmd_lpf_hz":0.0}
```

### `POST /api/settings/controller/lqr/yaw`

```json
{"k_yaw":0.0,"k_yaw_rate":0.0}
```

### `POST /api/settings/controller/output`

```json
{"deadband":0.0,"min_cmd":0.0,"motor_gains":{"left":1.0,"right":1.0}}
```

### `POST /api/settings/filter/select`

```json
{"name":"MADGWICK"}
```

### `POST /api/settings/filter/params`

```json
{"params":{"BETA":0.0219}}
```

### `POST /api/motors/enable`

Empty payload.

### `POST /api/motors/disable`

Empty payload.

### `POST /api/motors/command`

```json
{"left":0.1,"right":-0.1}
```

### `POST /api/system/reboot`

Empty payload, explicit confirmation in UI before send.

### `POST /api/logs/channels`

Deferred but planned.

```json
{"channel":"MOTOR","enabled":true}
```

---

## Command profile mapping

## Controller

- mode:
	- `BALANCE STRATEGY PID`
	- `BALANCE STRATEGY LQR`
- PID gains:
	- `BALANCE PID GAINS <kp> <ki> <kd>`
- LQR gains:
	- `BALANCE LQR GAINS <kp> <kg> <kd> <ks>`
- adaptive trim:
	- `BALANCE LQR TRIM ON`
	- `BALANCE LQR TRIM OFF`
- LQR filters:
	- `BALANCE LQR FILTER SET <pitch_rate_hz> <cmd_hz>`
- LQR yaw:
	- `BALANCE LQR YAW SET <ky> <kyr>`
- output shaping:
	- `BALANCE DEADBAND SET <v>`
	- `BALANCE MIN_CMD SET <v>`
	- `BALANCE MOTOR_GAINS SET <left> <right>`

## Filters

- select filter:
	- `FILTER SELECT <name>`
- set filter param:
	- `FILTER SET <PARAM> <value>`

## Motors

- enable:
	- `MOTOR ENABLE`
- disable:
	- `MOTOR DISABLE`
- diagnostic commands:
	- `MOTOR SET LEFT <v>`
	- `MOTOR SET RIGHT <v>`

## System

- reboot:
	- `SYS REBOOT`

---

## Validation rules

Validation should happen in the backend before any command is sent.

## Generic rules

- reject `NaN`, `inf`, empty strings,
- reject unsupported enum values,
- normalize uppercase enum tokens before mapping,
- require JSON body fields explicitly.

## Controller values

- `mode` must be `PID` or `LQR`.
- PID and LQR gains must be finite floats.
- LQR LPF frequencies must be `>= 0`.
- deadband and min_cmd must be finite floats and `>= 0`.

### Note on hard clamps

Do **not** artificially over-constrain LQR/PID gains in the UI with narrow fixed ranges.

Reason:

- tuning on this robot has already varied significantly,
- the operator may intentionally test unusual values,
- unsafe ranges are context-dependent.

Recommended approach:

- broad backend sanity bounds only, for example `abs(value) < 1e6`,
- numerical text boxes instead of sliders for gains,
- optional soft warning text instead of hard blocking for atypical values.

## Filter params

- only send parameters listed as supported in the structured snapshot.
- finite float values only.

## Motor diagnostics

- left/right commands must be in `[-1.0, 1.0]`.
- send both commands explicitly.
- include a fast “zero both” action.

## Reboot

- require an explicit confirmation modal in the UI.

---

## Logs tab design

## Goals

- give the operator live firmware feedback in the same browser session,
- avoid needing a separate TCP console window,
- preserve command/result context while tuning.

## Visual style

- dark terminal pane,
- monospace font,
- scrolling list,
- newest lines appended at bottom,
- optional subtle color coding by line kind.

## Log line kinds

- `log`: passive firmware line
- `command`: command sent from UI
- `response`: command-related output
- `api`: machine-readable internal line (`WEBUI_JSON:`), hidden by default from operator pane
- `error`: connection or backend error

## Buffering

- backend keeps last `N` lines, e.g. 2000
- frontend polls `GET /api/logs?since=<seq>` every 250–500 ms

## Filtering

Initial release:

- no heavy parsing,
- hide `WEBUI_JSON:` lines from the visible terminal by default,
- keep them in backend parsing path only.

Later:

- channel filters,
- severity highlighting,
- copy/export.

---

## Firmware work needed

## New command handler

Add `WebUiCommandHandler` to the serial command registry.

Responsibilities:

- implement `WEBUI SNAPSHOT`,
- gather runtime state from existing services,
- emit a single JSON line with `WEBUI_JSON:` prefix.

## Data sources for snapshot

### Controller

- `abbot::balancer::controller::getMode()`
- `abbot::balancer::controller::getGains(...)`
- `abbot::balancer::controller::getCascadedGains(...)`
- `abbot::balancer::controller::isAdaptiveTrimEnabled()`
- `CascadedLqrStrategy::getConfig()` for LQR filter and yaw values
- `abbot::balancer::controller::getDeadband()`
- `abbot::balancer::controller::getMinCmd()`
- `abbot::balancer::controller::getMotorGains(...)`

### Filter

- `FilterService::getCurrentFilterName()`
- `FilterService::getAvailableFilterCount()` / `getAvailableFilterName()`
- active filter `getParam()` for `ALPHA`, `KACC`, `KBIAS`, `BETA`

### Motors

- active driver `areMotorsEnabled()`
- `getMotorId(LEFT/RIGHT)`
- `isMotorInverted(LEFT/RIGHT)`
- `getLastMotorCommand(LEFT/RIGHT)`

### System

- `ESP.getFreeHeap()`
- maybe Wi-Fi console compile-time enable flag
- optionally telemetry state if accessible cheaply

### Logs

- existing enabled channel state via `abbot::log`

## JSON generation choice

Prefer a small hand-written JSON builder with `snprintf` into a sufficiently large buffer.

Reason:

- avoids new third-party dependency,
- keeps firmware deterministic,
- payload size is modest.

---

## Backend work needed

## Python packages to add

Suggested live telemetry backend structure additions:

- `tools/live_telemetry/console/`
	- `console_session.py`
	- `log_buffer.py`
	- `command_profiles.py`
	- `snapshot_parser.py`
- `tools/live_telemetry/web/api_handlers/`
	- or equivalent route split if desired later

## Responsibilities

### `ConsoleSession`

- connect once to robot TCP console,
- reconnect if lost,
- feed `LogBuffer`,
- support `send_profiled_command(...)`,
- support `request_snapshot()` by waiting for one `WEBUI_JSON:` line.

### `SnapshotParser`

- parse the JSON after `WEBUI_JSON:` prefix,
- reject malformed payloads,
- update backend cache atomically.

### `CommandProfileService`

- typed command builders,
- validation,
- refresh snapshot after successful writes.

### HTTP handler extensions

- new REST endpoints described above,
- replace generic public `/api/command` usage in the UI with typed routes.

The generic `/api/command` endpoint may remain available internally for troubleshooting, but must not be used by the normal web UI.

---

## Frontend work needed

## New JS modules

Suggested additions under `tools/live_telemetry/assets/js/`:

- `tab-controller.js`
- `settings-api.js`
- `settings-store.js`
- `settings-view.js`
- `logs-view.js`
- `info-view.js`
- `motor-diagnostics-view.js`

## New CSS groups

Suggested additions under `assets/styles/`:

- `tabs.css`
- `forms.css`
- `terminal.css`
- `panels.css`

## Frontend responsibilities

### `tab-controller.js`

- switch visible main panel
- preserve active tab state

### `settings-store.js`

- hold latest config snapshot
- notify views on refresh

### `settings-api.js`

- typed wrappers around `/api/config`, `/api/settings/...`, `/api/motors/...`, `/api/system/reboot`, `/api/logs`

### `settings-view.js`

- render controller/filter/system forms
- bind submit handlers
- display inline success/error feedback

### `motor-diagnostics-view.js`

- explicit enable/disable
- left/right normalized inputs
- zero/reset actions
- read-only inversion badges

### `logs-view.js`

- terminal rendering
- incremental polling
- auto-scroll behavior

### `info-view.js`

- render read-only snapshot cards

---

## Safety and operator experience

## Commands that should be safe in first UI release

- strategy switch
- gain changes
- filter selection / parameter change
- motor enable / disable
- motor low-speed normalized command
- reboot with confirmation

## Commands intentionally excluded from first UI release

- trim calibration
- deadband calibration
- start threshold calibration
- raw motor register-style commands
- IMU reinit
- autotune
- free-form console text entry

## Motor diagnostics UX guard rails

- motor commands disabled in UI until motors are enabled,
- visible warning banner when motors are enabled,
- quick “zero both” button,
- optional hold-to-send mode later if needed,
- start with numeric fields and an Apply button rather than continuously streaming slider commands.

This reduces accidental wheel motion.

---

## Incremental implementation plan

## Step 1 — firmware structured read support

- add `WebUiCommandHandler`
- add `WEBUI SNAPSHOT`
- validate JSON response on serial / TCP console

## Step 2 — backend console session and snapshot cache

- add persistent console session thread
- add live log buffer
- add snapshot refresh path
- add new HTTP endpoints

## Step 3 — frontend tab shell and logs view

- add tab system
- add Logs tab with terminal UI
- add Info tab with read-only snapshot rendering

## Step 4 — settings forms

- controller settings
- filter settings
- motor diagnostics
- reboot action

## Step 5 — polish and validation

- error toasts / inline messages
- disabled states while requests run
- refresh-after-write behavior
- connection loss handling

---

## Test plan

## Firmware

- build succeeds with new command handler
- `WEBUI SNAPSHOT` works on USB serial
- `WEBUI SNAPSHOT` works through Wi-Fi console
- payload remains valid when no active motor driver is present

## Backend

- reconnect handling after robot reboot
- log buffer continues after reconnect
- settings writes refresh snapshot correctly
- malformed JSON line does not crash dashboard

## Frontend

- tab switching works on desktop and narrow viewports
- logs panel auto-scrolls correctly
- settings forms reject invalid values
- reboot confirmation prevents accidental reset
- motor diagnostics actions visibly reflect enabled state

## Manual operator scenarios

1. Open dashboard and confirm telemetry still works.
2. Open Logs tab and confirm live firmware output appears.
3. Change strategy PID ↔ LQR.
4. Change LQR gains and see snapshot refresh.
5. Switch IMU filter and verify supported params update.
6. Enable motors, send low left/right commands, zero both, disable motors.
7. Reboot from dashboard and confirm reconnect flow.

---

## Recommended first implementation slice

If implementation is split into small PRs/patches, the best first slice is:

1. `WEBUI SNAPSHOT` in firmware
2. persistent backend console session + log buffer
3. `Logs` tab
4. read-only `Info` tab
5. controller/filter settings forms
6. motor diagnostics
7. reboot button

This gives value early while keeping risk manageable.

---

## Decision summary

- Keep charts as the default landing experience.
- Add **tabs**, not a giant all-in-one page.
- Add a **terminal-style Logs tab**.
- Add a **typed Settings tab** for controller, filters, motors, reboot.
- Add a **read-only Info tab**.
- Use **structured firmware reads** via `WEBUI SNAPSHOT`.
- Use **typed command profiles** for writes.
- Do **not** add free-form command text entry in v1.
- Do **not** add autotune/tuning UI in v1.

This design fits the current codebase, respects the operator workflow, and keeps the dashboard useful without turning it into an unsafe control free-for-all.
