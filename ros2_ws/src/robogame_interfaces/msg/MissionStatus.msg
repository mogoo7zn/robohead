# Mission dashboard snapshot published by the bringup node (~1 Hz).
string hfsm_state           # e.g. MISSION/ACQUIRE/ACQUIRE_ALIGN
string current_task         # ACQUIRE_ORANGE | ACQUIRE_PURPLE | BUILD | ENDGAME
string current_skill
int32 retry_count
int32 orange_carried
int32 purple_carried
int32 placed_total
string[] tower_names
int32[] tower_heights
bool[] tower_purple_top
bool match_started
bool match_finished
float64 elapsed_time
float64 remaining_time
string phase                # NORMAL | SAFE | ENDGAME
float64 localization_confidence
bool watchdog_healthy
