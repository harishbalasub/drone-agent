"""Session export and provider request regression checks; no network calls."""
import io
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import prototype_server as server


class SessionTests(unittest.TestCase):
    def test_export_artifacts_and_redaction(self):
        with tempfile.TemporaryDirectory() as temp, patch.object(server, 'ROOT', Path(temp)):
            (Path(temp) / 'drone_domain.pddl').write_text('(define (domain test))')
            session = {'events': [{'time': 'now', 'level': 'warn', 'message': 'blocked'}], 'versions': [{'version': 2, 'mission_spec': {'api_key': 'secret', 'goals': []}, 'validation': {'status': 'ACCEPTED'}, 'pddl': '(define (problem test))', 'robot_plan': [{'skill': 'RETURN_HOME'}]}]}
            folder = server.save_session('logs/drone', session)
            self.assertTrue((folder / 'trace.txt').exists())
            self.assertTrue((folder / 'domain.pddl').exists())
            self.assertTrue((folder / 'snapshot_001/problem.pddl').exists())
            self.assertNotIn('secret', (folder / 'session.json').read_text())
            self.assertEqual(json.loads((folder / 'snapshot_001/robot_plan.json').read_text())[0]['skill'], 'RETURN_HOME')
            self.assertNotEqual(folder, server.save_session('logs/drone', session))

    def test_path_restrictions(self):
        with tempfile.TemporaryDirectory() as temp, patch.object(server, 'ROOT', Path(temp)):
            for directory in ['../escape', '/tmp', '.codex', 'logs/../escape', '', 'logs//drone']:
                with self.assertRaises(ValueError):
                    server.save_session(directory, {})
            (Path(temp) / 'escape').symlink_to('/tmp', target_is_directory=True)
            with self.assertRaises(ValueError):
                server.save_session('escape', {})
            with self.assertRaises(ValueError):
                server.save_session('logs', {'api_key': 'secret'})

    def test_save_endpoint(self):
        h = object.__new__(server.Handler)
        h.path = '/api/save-session'
        data = json.dumps({'directory': 'logs', 'session': {'events': [], 'versions': []}}).encode()
        h.headers = {'Content-Length': str(len(data))}
        h.rfile = io.BytesIO(data)
        responses = []
        h._json = lambda code, body: responses.append((code, body))
        with patch.object(server, 'save_session', return_value=server.ROOT / 'logs/test') as save:
            h.do_POST()
        save.assert_called_once()
        self.assertEqual(responses, [(200, {'path': 'logs/test'})])

    def test_provider_schema_compatibility(self):
        schema = {'type': 'object', 'additionalProperties': False, 'required': ['degrees'], 'properties': {'degrees': {'type': ['number', 'null']}}}
        for provider in ['openai', 'gemini']:
            h = object.__new__(server.Handler)
            h.path = '/api/llm'
            data = json.dumps({'provider': provider, 'api_key': 'fixture', 'model': 'fixture', 'prompt': 'fixture', 'schema': schema}).encode()
            h.headers = {'Content-Length': str(len(data))}; h.rfile = io.BytesIO(data)
            responses = []; h._json = lambda code, body: responses.append(code)
            reply = {'output_text': '{}'} if provider == 'openai' else {'candidates': [{'content': {'parts': [{'text': '{}'}]}}]}
            with patch.object(server, 'post_json', return_value=reply) as upstream:
                h.do_POST()
            payload = upstream.call_args.args[1]
            self.assertEqual(responses, [200])
            if provider == 'openai':
                self.assertEqual(payload['text']['format']['schema'], schema)
            else:
                adapted = payload['generationConfig']['responseSchema']
                self.assertEqual(adapted['properties']['degrees'], {'type': 'NUMBER', 'nullable': True})
                self.assertNotIn('additionalProperties', adapted)


if __name__ == '__main__':
    unittest.main()
