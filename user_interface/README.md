# AquaSweep Companion UI Deliverables

This folder captures the weekly UX/UI outputs for the AquaSweep pool-cleaning companion app. Everything is mobile-first and ready to showcase without additional hardware.

## Contents

- `prototype/` – clickable HTML prototype covering all modes and navigation.
- `copywriting.md` – production-ready UI strings for buttons, tooltips, and empty states.
- `ux_flow.md` – Mermaid state diagram linking the four primary modes and supporting screens.
- `demo_storyboard.md` – shot-by-shot outline for recording a short mode-switching demo GIF/video.

## Quick start

1. Open `prototype/index.html` in any modern browser (double-click in Finder or run `open prototype/index.html`).
2. Use the large mode tiles or bottom navigation to explore Home, Preset, Gen Path, Manual, End & Return, Map & History, and Settings.
3. The connectivity card shows rosbridge status; click it to change the WebSocket URL if you’re not on `ws://localhost:9090`.
4. Press `⌘ + L` to toggle light/dark previews while evaluating contrast.
5. Follow `demo_storyboard.md` when capturing the requested demo clip.

Tip: you can also append `?rosbridge=ws://<host>:<port>` to the prototype URL, and the UI will remember that endpoint for future sessions.

## Design highlights

- One-tap access to four main actions via the home dashboard.
- Large, high-contrast CTAs suitable for bright outdoor conditions.
- Safety affordances with confirm dialogs, color-coded states, and live alerts.
- Coverage feedback baked into Gen Path, Map & History, and status surfaces.

Please keep all new UI assets and documentation within this folder so the project structure remains clean.
