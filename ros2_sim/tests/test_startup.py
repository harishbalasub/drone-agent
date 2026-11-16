import os,signal,socket,subprocess,sys,time,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from drone_sim.startup import check_port
ROOT=Path(__file__).resolve().parents[2]
class StartupContracts(unittest.TestCase):
    def test_occupied_port_stops_launcher_before_simulator_start(self):
        with socket.socket() as occupied:
            occupied.bind(('127.0.0.1',0));occupied.listen();port=occupied.getsockname()[1]
            result=subprocess.run(['bash','ros2_sim/scripts/launch.sh','1','--render-headless','--rgbd'],cwd=ROOT,env={**os.environ,'DRONE_PORT':str(port),'DRONE_BIND_HOST':'127.0.0.1'},text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=5)
            self.assertNotEqual(result.returncode,0);self.assertIn('already in use',result.stdout);self.assertIn('DRONE_PORT=8101',result.stdout);self.assertNotIn('Traceback',result.stdout);self.assertNotIn('RGB-D runtime/checkpoint missing',result.stdout)
            with socket.create_connection(('127.0.0.1',port),timeout=1):pass
    def test_invalid_and_available_ports(self):
        for port in [0,65536]:
            with self.assertRaisesRegex(ValueError,'between'):check_port('127.0.0.1',port)
        with socket.socket() as s:s.bind(('127.0.0.1',0));port=s.getsockname()[1]
        check_port('127.0.0.1',port)
    def test_gateway_sigterm_closes_http_server(self):
        with socket.socket() as s:s.bind(('127.0.0.1',0));port=s.getsockname()[1]
        code="""
import sys
sys.path[:0]=['ros2_sim','ros2_sim/tests']
from test_gateway import IdleTransport
import drone_sim.transport as transport
transport.GazeboTransport=IdleTransport
import server
sys.argv=['server.py','--transport','gazebo','--port',sys.argv[1]]
server.main()
"""
        child=subprocess.Popen([sys.executable,'-c',code,str(port)],cwd=ROOT,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
        try:
            deadline=time.monotonic()+8
            while time.monotonic()<deadline:
                if child.poll() is not None:self.fail(child.stdout.read())
                try:
                    with socket.create_connection(('127.0.0.1',port),timeout=.1):break
                except OSError:time.sleep(.05)
            else:self.fail('Test gateway did not start')
            child.send_signal(signal.SIGTERM);output=child.communicate(timeout=5)[0]
            self.assertEqual(child.returncode,0,output);check_port('127.0.0.1',port)
        finally:
            if child.poll() is None:child.kill();child.communicate()
