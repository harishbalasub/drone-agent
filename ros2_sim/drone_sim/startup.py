"""Dependency-free gateway preflight, before launching simulator processes."""
import argparse
import errno
import socket


def check_port(host,port):
    if not 1<=port<=65535:raise ValueError('Gateway port must be between 1 and 65535')
    try:
        with socket.socket(socket.AF_INET,socket.SOCK_STREAM) as probe:
            probe.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
            probe.bind((host,port))
    except OSError as error:
        if error.errno==errno.EADDRINUSE:
            raise ValueError(f'Gateway address {host}:{port} is already in use. Stop the older launcher with Ctrl+C, or choose another port: DRONE_PORT=8101 bash ros2_sim/scripts/launch.sh 1 --render-headless --rgbd. No existing process was stopped.') from None
        raise ValueError(f'Cannot bind gateway at {host}:{port}: {error}') from None


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--host',default='127.0.0.1');parser.add_argument('--port',type=int,default=8100)
    args=parser.parse_args()
    try:check_port(args.host,args.port)
    except ValueError as error:raise SystemExit(str(error))
