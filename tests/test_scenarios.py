"""Run production UI/controller code with an in-memory DOM and accelerated motion."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]

def run_js(cases):
    html = (ROOT / "accountable_drone_agent_prototype_v2.html").read_text()
    source = html.split("<script>", 1)[1].split("</script>", 1)[0]
    stub = (ROOT / "tests/browser_stub.js").read_text()
    with tempfile.TemporaryDirectory() as folder:
        target = Path(folder) / "test.js"
        target.write_text(stub + source + cases)
        subprocess.run(["gjs", str(target)], check=True)

if __name__ == "__main__":
    run_js((ROOT / "tests/scenario_cases.js").read_text())
