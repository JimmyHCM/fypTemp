# Mode & Screen Flow

```mermaid
stateDiagram-v2
    [*] --> Dashboard

    state Dashboard {
        Dashboard --> Preset : Tap "Preset"
        Dashboard --> Generate : Tap "Gen Path"
        Dashboard --> Manual : Tap "Manual"
        Dashboard --> EndReturn : Tap "End & Return"
    }

    Preset --> Manual : "Record your first manual run"
    Preset --> Generate : "Generate optimized path"

    Generate --> Dashboard : "Save plan" / auto-complete
    Generate --> Manual : "Adjust manually"
    Generate --> EndReturn : "Abort & return"

    Manual --> Preset : "Save as preset"
    Manual --> Dashboard : "Exit manual"
    Manual --> EndReturn : "End & Return"

    EndReturn --> Docking : Confirm return
    Docking --> Dashboard : Docked

    Dashboard --> MapHistory : Tap bottom nav "Map"
    Dashboard --> Settings : Tap bottom nav "Settings"

    MapHistory --> Dashboard : Back gesture
    Settings --> Dashboard : Back gesture
```

## Notes
- Primary entry point is the dashboard; bottom navigation keeps Map/History and Settings one tap away.
- Safety flow enforces confirmation before entering the Docking sub-state.
- Manual mode is the only path to saving new presets, reinforcing deliberate user control.
- Coverage data loops users back to Dashboard to encourage repeat engagement after each run.
