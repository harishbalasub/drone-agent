# Mission conversation verification

Pre-chat GitHub checkpoint: `0213b74` (2026-11-15).

The browser UI was exercised with the explicit local demo compiler (no live provider calls):

1. Generate the default two-viewpoint mission.
2. Review the proposed 30-degree separation in chat.
3. Change the separation to 45 degrees and revalidate.
4. Generate the Behavior Tree from the chat card and run it.
5. Verify search, two inspections, pickup, gate traversal, placement, home return and completion cards.
6. Verify labeled simulated schematic thumbnails.
7. Save session logs: 15 chat records, 8 milestone cards and 16 thumbnail records were included; outcome `MISSION_COMPLETED`.

`browser_delivery.png` shows the actual UI and placement evidence card. Browser renderings are schematic views, not physical camera frames.

Regression verification: 51 ROS2/app tests and 4 original server tests passed. Tests cover structured mocked LLM chat proposals, explicit application before mission mutation, assumption confirmation and edits, insufficient reserve, stale actions, runtime edit blocking, world reset reuse, external save merging, and fresh/stale/missing camera capture. Real LLM-provider responses and a new live Gazebo mission were not tested for this change. The gateway must be restarted for external event-time image capture and chat export.
