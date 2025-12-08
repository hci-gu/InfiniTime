# Implementation Plan: Activity State Display

## Goals
- Classify each logged minute into activity states using counts: `<2700` = **still**, `2700–9515` = **moving**, `>9515` = **active**.
- Track how long the wearer has been in the current state and accumulate per-state minutes for the current day.
- Surface the current state and its ongoing duration on the watchface in the corner where “stored minutes” is currently shown (e.g., `Active - 5`).

## Work Plan
1) **State model & thresholds**
   - Add an `ActivityState { Still, Moving, Active }` enum and constants for the three count thresholds.
   - Keep a `currentActivityState`, `stateStartMinuteTick`, `currentStateStreakMinutes`, and an array of daily totals per state (minutes).
   - Default to `Still` until the first full-minute counts value is available.

2) **Hook into minute logging**
   - In `MotionController::MaybeStoreMinuteAverage` (after counts are computed), call a new `UpdateActivityState(counts, timestamp)` helper.
   - On each minute: classify the counts, detect transitions, increment the streak for the current state, and update per-state totals.
   - Handle the “no data” minute by keeping the previous state but not incrementing totals if no samples were recorded.

3) **Reset/rollover handling**
   - On `MotionController::AdvanceDay()` (already used for step rollover), reset state streak/totals and reinitialize `currentActivityState` to `Still`.
   - On controller init/reboot, clear streak/totals to avoid mixing old data with the new day; the first complete minute will repopulate.

4) **Expose data to the UI**
   - Add getters: `CurrentActivityState()`, `CurrentActivityStateMinutes()`, and `DailyActivityMinutes()` (array per state).
   - Keep existing “stored minutes” APIs unchanged so other screens continue working.
   - Map states to short labels (`Still`, `Move`, `Active`) for reuse by the watchface.

5) **Watchface update**
   - Replace the unsynced-minutes label on the digital watchface with `"<State> - <streakMinutes>"`, blank until the first minute arrives.
   - Use `DirtyValue` fields in `WatchFaceDigital` to detect state/duration updates and refresh the corner label; keep alignment/colour consistent with the previous corner text.