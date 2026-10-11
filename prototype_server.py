#!/usr/bin/env python3
"""Local static server + LLM proxy for the Accountable Drone prototype.

Run from the prototype directory:
    python prototype_server.py
Then open:
    http://127.0.0.1:8000/accountable_drone_agent_prototype_v2.html

The API key is received from the local browser request, used only for the
upstream request, and is never written to disk or printed by this server.
"""
from __future__ import annotations

import json
import os
import re
from datetime import datetime, timezone
from uuid import uuid4
import urllib.error
import urllib.parse
import urllib.request
from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler
from pathlib import Path

ROOT = Path(__file__).resolve().parent
HOST = "127.0.0.1"
PORT = int(os.environ.get("DRONE_PROTO_PORT", "8000"))


def save_session(directory: str, session: dict) -> Path:
    """Save only session artifacts below a visible project subdirectory."""
    parts = directory.split("/")
    if not parts or any(not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]*", p) for p in parts):
        raise ValueError("Use a relative subdirectory such as session_logs or logs/drone")
    destination = (ROOT / directory).resolve()
    if ROOT not in destination.parents:
        raise ValueError("Save directory must be inside the project")
    allowed = {"session_id", "started_at", "saved_at", "events", "versions", "interrupts", "human_decisions", "outcome", "scenario", "world", "state"}
    if not isinstance(session, dict) or set(session) - allowed:
        raise ValueError("Invalid session artifact fields")
    # Never persist credentials, including credentials accidentally nested in input.
    def redact(value):
        if isinstance(value, dict):
            return {k: redact(v) for k, v in value.items() if not re.search(r"api.?key|authorization|secret|token", k, re.I)}
        if isinstance(value, list):
            return [redact(v) for v in value]
        return value
    session = redact(session)
    destination.mkdir(parents=True, exist_ok=True)
    folder = destination / (datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "_" + uuid4().hex[:8])
    folder.mkdir()
    (folder / "session.json").write_text(json.dumps(session, indent=2), encoding="utf-8")
    (folder / "trace.txt").write_text("\n".join(f"{e.get('time', '')} [{e.get('level', '')}] {e.get('message', '')}" for e in session.get("events", [])), encoding="utf-8")
    for index, version in enumerate(session.get("versions", []), 1):
        revision = folder / f"snapshot_{index:03d}"
        revision.mkdir()
        (revision / "snapshot.json").write_text(json.dumps(version, indent=2), encoding="utf-8")
        for field in ("mission_spec", "validation", "robot_plan"):
            (revision / f"{field}.json").write_text(json.dumps(version.get(field), indent=2), encoding="utf-8")
        (revision / "problem.pddl").write_text(version.get("pddl") or "; Not generated at this stage\n", encoding="utf-8")
    (folder / "domain.pddl").write_text((ROOT / "drone_domain.pddl").read_text(), encoding="utf-8")
    return folder


def post_json(url: str, payload: dict, headers: dict[str, str]) -> dict:
    data = json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(url, data=data, headers=headers, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=90) as resp:
            return json.loads(resp.read().decode("utf-8"))
    except urllib.error.HTTPError as e:
        body = e.read().decode("utf-8", errors="replace")
        raise RuntimeError(f"Upstream HTTP {e.code}: {body[:2000]}") from e


def gemini_response_schema(schema: dict) -> dict:
    """Translate the mission JSON Schema into Gemini's OpenAPI Schema dialect."""
    result = {k: v for k, v in schema.items() if k != "additionalProperties"}
    kind = result.get("type")
    if isinstance(kind, list):
        non_null = [t for t in kind if t != "null"]
        if len(non_null) != 1:
            raise ValueError("Gemini schema requires one non-null type")
        result["type"] = non_null[0]
        if "null" in kind:
            result["nullable"] = True
    if isinstance(result.get("type"), str):
        result["type"] = result["type"].upper()
    if "properties" in result:
        result["properties"] = {k: gemini_response_schema(v) for k, v in result["properties"].items()}
        result["propertyOrdering"] = list(result["properties"])
    if "items" in result:
        result["items"] = gemini_response_schema(result["items"])
    return result


def extract_openai_text(data: dict) -> str:
    if isinstance(data.get("output_text"), str) and data["output_text"]:
        return data["output_text"]
    parts: list[str] = []
    for item in data.get("output", []) or []:
        for c in item.get("content", []) or []:
            if c.get("type") == "output_text" and c.get("text"):
                parts.append(c["text"])
    return "".join(parts)


def extract_gemini_text(data: dict) -> str:
    out: list[str] = []
    for candidate in data.get("candidates", []) or []:
        for part in (candidate.get("content") or {}).get("parts", []) or []:
            if part.get("text"):
                out.append(part["text"])
        if out:
            break
    return "".join(out)


class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(ROOT), **kwargs)

    def log_message(self, fmt, *args):
        # Normal request logging only; request bodies/API keys are never logged.
        super().log_message(fmt, *args)

    def _json(self, status: int, obj: dict):
        body = json.dumps(obj).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        if self.path == "/api/save-session":
            try:
                n = int(self.headers.get("Content-Length", "0"))
                if n < 1 or n > 10_000_000:
                    raise ValueError("Session size must be between 1 byte and 10 MB")
                req = json.loads(self.rfile.read(n).decode("utf-8"))
                folder = save_session(str(req.get("directory", "session_logs")), req.get("session"))
                self._json(200, {"path": str(folder.relative_to(ROOT))})
            except (ValueError, TypeError, OSError) as e:
                self._json(400, {"error": str(e)})
            return
        if self.path != "/api/llm":
            self._json(404, {"error": "unknown endpoint"})
            return
        try:
            n = int(self.headers.get("Content-Length", "0"))
            req = json.loads(self.rfile.read(n).decode("utf-8"))
            provider = str(req.get("provider", "")).lower()
            api_key = str(req.get("api_key", ""))
            model = str(req.get("model", ""))
            prompt = str(req.get("prompt") or req.get("goal") or "")
            schema = req.get("schema") or {}
            if not api_key or not model or not prompt:
                self._json(400, {"error": "api_key, model, and prompt are required"})
                return

            if provider == "openai":
                payload = {
                    "model": model,
                    "instructions": "Compile the user drone mission into the supplied JSON schema. Do not invent unsupported capabilities or facts.",
                    "input": prompt,
                    "text": {
                        "format": {
                            "type": "json_schema",
                            "name": "drone_mission_spec",
                            "strict": True,
                            "schema": schema,
                        }
                    },
                }
                data = post_json(
                    "https://api.openai.com/v1/responses",
                    payload,
                    {"Content-Type": "application/json", "Authorization": f"Bearer {api_key}"},
                )
                text = extract_openai_text(data)
            elif provider == "gemini":
                qmodel = urllib.parse.quote(model, safe="")
                payload = {
                    "contents": [{"role": "user", "parts": [{"text": prompt}]}],
                    "generationConfig": {
                        "responseMimeType": "application/json",
                        "responseSchema": gemini_response_schema(schema),
                    },
                }
                data = post_json(
                    f"https://generativelanguage.googleapis.com/v1beta/models/{qmodel}:generateContent",
                    payload,
                    {"Content-Type": "application/json", "x-goog-api-key": api_key},
                )
                text = extract_gemini_text(data)
            else:
                self._json(400, {"error": f"unsupported provider: {provider}"})
                return

            if not text:
                self._json(502, {"error": "provider returned no text"})
                return
            self._json(200, {"text": text})
        except Exception as e:
            self._json(502, {"error": str(e)})


if __name__ == "__main__":
    print(f"Serving {ROOT} at http://{HOST}:{PORT}")
    print(f"Open http://{HOST}:{PORT}/accountable_drone_agent_prototype_v2.html")
    ThreadingHTTPServer((HOST, PORT), Handler).serve_forever()
