# UI Copy Reference

All copy is concise, action-led, and optimized for mobile readability. Strings are grouped by screen.

## Home / Dashboard
- Header: "AquaSweep"
- Mode tiles:
  - "Preset" — subtitle "Start saved plans fast"
  - "Gen Path" — subtitle "Create optimized sweep"
  - "Manual" — subtitle "Direct joystick control"
  - "End & Return" — subtitle "Safely dock the robot"
- Status cards:
  - Battery: "Approx. 34 min remaining"
  - Filter: "Rinse recommended soon"
  - Connectivity: "Poolside relay online"
  - Map status: "Last updated 5 min ago"
- Live tiles:
  - Active: "Cleaning now…"
  - Paused: "Paused"
  - Returning: "Returning"

## Preset Mode
- Title: "Presets"
- CTA: "New preset"
- Preset list items:
  - Quick Clean: subtitle "Shallow lap sweep • 12 min", action "Start"
  - Deep Floor: subtitle "Full floor spiral • 28 min", action "Start"
  - Walls & Waterline: subtitle "Vertical scrub • 18 min", action "Start"
- Build from manual description: "Save recorded manual paths as reusable presets."
- Filter chips: "All recordings", "This week", "High coverage"
- Empty state body: "No manual runs saved yet."
- Empty state CTA: "Record your first manual run"

## Generate Path
- Title: "Generate Path"
- Map header: "Coverage map"
- Pill: "Projected 94%"
- KPI labels: "Overlap", "Lane width", "Time est."
- Range sliders: "Overlap %", "Lane width (m)", "Sweep speed (m/s)"
- CTA: "Generate path"
- Toast (success): "Optimized path generated (94% coverage)"
- History list items include timestamp + run name, trailing coverage percentage (e.g., "92% coverage")

## Manual Control
- Title: "Manual Control"
- Safety list:
  - "No obstacles detected"
  - "Drop-off within 0.5 m"
  - "Auto-brake ready"
- Recording card text: "Record your manual session to reuse as a preset."
- Recording buttons:
  - Primary: Default "Start recording", toggled "Stop recording"
  - Secondary: "Save as preset" (enabled when recording stops)
- Toasts:
  - Start: "Manual run recording started"
  - Stop: "Manual run saved"
  - Save: "Preset draft ready to name"

## End & Return
- Title: "End & Return"
- Body text: "Stop cleaning and send AquaSweep back to dock."
- CTA: "End & Return"
- Dock selector label: "Choose dock"
- Confirmation dialog:
  - Title: "Confirm return?"
  - Body: "AquaSweep will stop cleaning and navigate to the selected dock."
  - Buttons: "Keep cleaning", "Confirm"
- Toast: "AquaSweep returning to dock"

## Map & History
- Title: "Map & History"
- Last run header: e.g., "Last run • 92% coverage"
- Legend labels: "High coverage", "Medium", "Missed"
- Run log entries: timestamp (e.g., "Nov 3, 10:20"), run name ("Deep floor"), KPI ("Coverage 92%")

## Settings
- Settings items:
  - "Pool profiles" — subtitle "Manage dimensions & depth"
  - "Sensor calibration" — subtitle "Schedule upcoming check"
  - "Firmware updates" — subtitle "Version 1.8.2 • Up to date"
  - "Wi-Fi & BLE pairing" — subtitle "Relay bridge connected"

## Global Strings
- Toast fallback: "Action completed"
- Mode label updates: derived from section titles ("Home", "Preset", "Gen Path", "Manual", "End & Return", "Map & History", "Settings")
- Light mode toggle toast: "Light mode preview" / "Dark mode preview"
