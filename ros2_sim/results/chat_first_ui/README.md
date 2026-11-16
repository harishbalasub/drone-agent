# Chat-first mission controls

The left sidebar no longer presents a duplicate mission editor or Generate/Validate/Plan/Run workflow. It retains connection settings, provider/key settings, compact mission status and session logs. Optional PDDL export is available inside the PDDL tab.

The chat composer starts with the default mission. Sending an initial mission directly prepares its Mission Spec, using the selected LLM provider or the explicitly enabled local demo. Subsequent mission edits retain the reviewed-proposal flow. Default mission reloads the draft without silently changing the active mission.

Verification:

- Actual browser: default goal preloaded; Send prepared the mission; assumption confirmation reached an enabled Generate Behavior Tree card.
- Regression: 60 tests passed, including a complete browser mission driven only by chat actions and reset/plan reuse.
- Layout check: former duplicate controls are not user-visible; required settings and chat controls remain accessible; optional PDDL resides in its tab.
- Screenshot: `interface.png` shows the simplified sidebar and initial default mission draft.

Existing controller handlers are retained behind the chat interface so validation and execution behavior remain shared. The original v2 HTML is unchanged.
