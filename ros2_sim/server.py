#!/usr/bin/env python3
"""Serve v3, legacy LLM/save APIs, and the asynchronous ROS/Gazebo gateway."""
import argparse
import json
import os
import sys
from http.server import ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

ROOT=Path(__file__).resolve().parent.parent
sys.path.insert(0,str(ROOT))
from prototype_server import Handler, save_session
from drone_sim.mission import Executor


def make_handler(executor):
    class GatewayHandler(Handler):
        def do_GET(self):
            path=urlsplit(self.path).path
            if path=='/api/sim/state':
                self._json(200,executor.state());return
            if path.startswith('/api/sim/camera/'):
                camera=path.rsplit('/',1)[-1]
                with executor.transport.lock: data=executor.transport.images.get(camera)
                if not data:self._json(404,{'error':'No image received from this camera'});return
                self.send_response(200);self.send_header('Content-Type','image/png');self.send_header('Content-Length',str(len(data)));self.send_header('Cache-Control','no-store');self.end_headers();self.wfile.write(data);return
            if path=='/':
                self.send_response(302);self.send_header('Location','/accountable_drone_agent_ros2_v3.html');self.end_headers();return
            super().do_GET()

        def do_POST(self):
            path=urlsplit(self.path).path
            if not path.startswith('/api/sim/'):
                super().do_POST();return
            # No permissive CORS; commands must originate from the served app.
            origin=self.headers.get('Origin')
            if origin and origin!=f'http://{self.headers.get("Host")}':
                self._json(403,{'error':'Use the app served by this gateway'});return
            try:
                n=int(self.headers.get('Content-Length','0'))
                if not 0<n<=500000:raise ValueError('Request must contain 1–500000 bytes')
                req=json.loads(self.rfile.read(n))
                if path=='/api/sim/run': result=executor.start(req)
                elif path=='/api/sim/stop': result=executor.cancel()
                elif path=='/api/sim/decision': executor.human(req.get('action'));result={'status':'DECISION_RECEIVED'}
                elif path=='/api/sim/save':
                    state=executor.state();session=state['session']
                    if not session:raise ValueError('No simulation session to save')
                    # Existing safe save utility filters credentials and bounds output to project.
                    artifact={k:session[k] for k in ['session_id','started_at','events','versions','interrupts','human_decisions','outcome']}
                    artifact['state']={'facts':state['facts'],'sensors':state['sensors'],'metrics':session['metrics'],'backend':state['transport']}
                    folder=save_session(req.get('directory','session_logs/ros2'),artifact)
                    result={'path':str(folder.relative_to(ROOT))}
                else:self._json(404,{'error':'unknown simulation endpoint'});return
                self._json(200,result)
            except (ValueError,TypeError,KeyError) as e:self._json(400,{'error':str(e)})
            except Exception as e:self._json(503,{'error':str(e)})
    return GatewayHandler


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--transport',choices=['ros2','gazebo'],default='ros2');parser.add_argument('--port',type=int,default=8100);parser.add_argument('--config');parser.add_argument('--host',default=os.environ.get('DRONE_BIND_HOST','127.0.0.1'))
    args=parser.parse_args()
    try:
        if args.transport=='ros2':
            from drone_sim.transport import RosTransport
            transport=RosTransport()
        else:
            from drone_sim.transport import GazeboTransport
            transport=GazeboTransport()
    except ImportError as e:
        raise SystemExit(f'Missing simulator dependency: {e}. Source ROS 2 Humble and install the dependencies in ros2_sim/README.md. No simulated success or automatic transport fallback is used.')
    config=json.loads(Path(args.config).read_text()) if args.config else None
    executor=Executor(transport,config);server=ThreadingHTTPServer((args.host,args.port),make_handler(executor))
    print(f'App: http://{args.host}:{args.port}/accountable_drone_agent_ros2_v3.html (transport={transport.kind})',flush=True)
    try:server.serve_forever()
    except KeyboardInterrupt:pass
    finally:server.server_close();executor.close()

if __name__=='__main__':main()
