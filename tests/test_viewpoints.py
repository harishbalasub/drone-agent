"""Regression checks for distinct viewpoint compilation and execution."""
from test_scenarios import ROOT, run_js

run_js((ROOT / "tests/viewpoint_cases.js").read_text())
