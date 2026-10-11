ACCOUNTABLE DRONE AGENT PROTOTYPE — V2
======================================

Files
-----
1. accountable_drone_agent_prototype_v2.html
   Main browser UI.

2. prototype_server.py
   Recommended local static server + LLM proxy. This avoids direct OpenAI
   browser calls and keeps provider request handling on localhost.

3. drone_genie_worksheet.txt
   Reusable Genie-Worksheet-inspired agent specification.

4. drone_domain.pddl
   Fixed PDDL domain.

Recommended start
-----------------
From this directory:

    python prototype_server.py

Then open:

    http://127.0.0.1:8000/accountable_drone_agent_prototype_v2.html

You can still open the HTML directly. The local compiler will work and Gemini
may work directly, but OpenAI should be used through prototype_server.py.

What was fixed in V2
--------------------
- Added structured-output schema enforcement for Gemini.
- Added a canonical normalization layer for provider output. It repairs common
  variations such as predicate strings like found(red_package), lowercase
  provenance labels, missing top-level fields, capabilities vs
  requested_capabilities, and missing ordering fields.
- Updated default models to gpt-6-luna and gemini-3.8-flash.
- Added a local /api/llm proxy for OpenAI/Gemini calls.
- Fixed mission semantics: holding(object) is an intermediate milestone when a
  later PLACE goal exists, not an impossible terminal goal.
- Fixed PDDL goal compilation so PLACE and HOLDING are not simultaneously
  required at the final state.
- Added semantic ordering inference when a provider omits ordering clauses.
- Added clearer upstream API error messages in the accountability trace.

Validation sanity tests performed
---------------------------------
The core logic was run in a Node test harness against:
1. The built-in local compiler for the default mission.
2. A deliberately malformed/legacy Gemini-style result matching the failure
   shown in the screenshot: missing mission_id/user_goal/entities/ordering,
   predicates encoded as strings like found(red_package), lowercase source
   labels, and capabilities stored under capabilities.

Both normalize and validate as ACCEPTED after the V2 fixes.

Security note
-------------
The HTML still contains an API-key input because this is a local class
prototype. Do not deploy provider API keys in client-side browser code. The
recommended path is prototype_server.py, and a production system should store
keys server-side/environment-side rather than sending long-lived credentials
from the browser.

Viewpoint inspection
--------------------
Inspection goals retain their object and viewpoint IDs in PDDL, robot plans,
and runtime completion checks. The PDDL domain uses (inspected ?o ?v) with
a viewpoint type. The local compiler recognizes numeric counts and one through
twelve in words. The simulator supports up to 12 distinct viewpoints; a view
outside the canvas or without a safe route halts execution.
Run regression checks with: python tests/test_viewpoints.py (requires gjs).

Scenarios, contingencies, and session history
-------------------------------------------
- Scenario 1 preserves the original clear-gate world.
- Scenario 2 adds a separate gate-blocking obstacle. The original obstacle
  remains in place. Motion routes avoid both obstacles and their clearance.
- Select a scenario before generating a mission. Scenario changes invalidate
  the current spec/validation/plan; regenerate for the selected world.
- The initial plan is optimistic about gate clearance. The runtime verifier
  checks the gate before approaching it and pauses if it is blocked.
- The mission spec includes a gate_not_clear contingency. Examples:
    "If the gate is not clear, bypass or ask for human feedback."
      Authorizes bypass first; asks the human if a bypass plan is rejected.
    "If the gate is blocked, ask me before bypassing."
      Requires an explicit human decision in the simulation panel.
    No conditional instruction:
      Aborts the remaining mission and returns home by default.
- Human decisions time out after 30 seconds to abort-and-return. Low battery
  selects safe return without waiting. If return is unreachable or too costly,
  the controller attempts a safe local landing.
- Recovery uses a deterministic prototype agent/controller. It exports the
  current-state PDDL problem and a validated replacement plan; it does not
  make an extra LLM call. Bypassing explicitly waives the gate traversal goal.
- Completed milestones and held-package state are retained in replans; the
  trace distinguishes mission completion, authorized bypass, safe return
  after abort, safe landing after abort, and an unrecoverable halt.
- The history retains each spec, validation, PDDL problem and robot plan,
  including failed recovery validations, interruptions and human decisions.
  Clear only clears the visible trace; retained session history is preserved.
- Enter a relative project subdirectory such as session_logs or logs/drone,
  then click "Save session logs and all versions". The local server writes a
  unique timestamped folder with session.json, trace.txt, domain.pddl and
  snapshot folders containing mission_spec.json, validation.json,
  problem.pddl, robot_plan.json and complete snapshot.json. API key input is
  not part of the export. Restart the server after updating this prototype.
- Saving to project directories requires opening the page through the local
  server, rather than opening the HTML as a file.

Regression checks (no live provider calls):
    python tests/test_scenarios.py
    python tests/test_viewpoints.py
    python tests/test_server.py
JavaScript checks require gjs. They run the production browser script with an
in-memory DOM and accelerated motion, including collision checks, to exercise
scenarios, recovery decisions, viewpoint preservation and export.
