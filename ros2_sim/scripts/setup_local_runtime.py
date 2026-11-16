#!/usr/bin/env python3
"""Optional Ubuntu 22.04 runtime overlay: signed official packages, no sudo.

Requires Gazebo Harmonic and the OSRF apt source to already be configured.
All downloaded indexes, keys, packages and extracted files stay in .runtime/.
"""
from pathlib import Path
import platform
import shlex
import subprocess
import urllib.request
import tempfile

SIM=Path(__file__).resolve().parents[1];RUNTIME=SIM/'.runtime'

def run(cmd):
    print('Running',cmd[0],cmd[-1],flush=True)
    with (RUNTIME/'setup.log').open('a') as log:subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,check=True)


def main():
    if platform.machine()!='x86_64':raise SystemExit('This overlay helper is tested on Ubuntu 22.04 x86_64 only. Use official ARM64 packages on Spark.')
    if not Path('/usr/bin/python3.10').exists():raise SystemExit('Requires Ubuntu Python 3.10')
    gazebo_source=Path('/etc/apt/sources.list.d/gazebo-stable.list')
    if not gazebo_source.exists():raise SystemExit('Configure the official Gazebo Harmonic apt repository first; see README.')
    for d in ['lists/partial','debs/partial','root','plugins']: (RUNTIME/d).mkdir(parents=True,exist_ok=True)
    key=urllib.request.urlopen('https://raw.githubusercontent.com/ros/rosdistro/master/ros.key',timeout=30).read()
    key_directory=tempfile.TemporaryDirectory(prefix='drone-ros-key-')
    keyfile=Path(key_directory.name)/('ros.asc' if key.startswith(b'-----BEGIN') else 'ros.gpg');keyfile.write_bytes(key)
    (RUNTIME/'sources.list').write_text(f'deb [signed-by="{keyfile}"] http://packages.ros.org/ros2/ubuntu jammy main\n'+gazebo_source.read_text())
    options=['-o','Dir::State::lists='+str(RUNTIME/'lists'),'-o','Dir::Etc::sourcelist='+str(RUNTIME/'sources.list'),'-o','Dir::Etc::sourceparts=-',
        '-o','Dir::Cache::archives='+str(RUNTIME/'debs'),'-o','Dir::Cache::pkgcache='+str(RUNTIME/'pkgcache.bin'),'-o','Dir::Cache::srcpkgcache='+str(RUNTIME/'srcpkgcache.bin'),'-o','Debug::NoLocking=1']
    run(['apt-get',*options,'update'])
    run(['apt-get',*options,'--download-only','--assume-yes','--no-install-recommends','install','ros-humble-rclpy','ros-humble-ros2cli','ros-humble-ros2run',
         'ros-humble-ros-gzharmonic-bridge','ros-humble-sensor-msgs','ros-humble-nav-msgs','ros-humble-geometry-msgs'])
    # Dev packages contain OGRE shader resources and discovery aliases missing
    # in some minimal runtime installations. Download only; don't install them.
    with (RUNTIME/'setup.log').open('a') as log:
        subprocess.run(['apt-get',*options,'download','libgz-rendering8-ogre2','libgz-rendering8-ogre2-dev','libgz-rendering8-core-dev'],cwd=RUNTIME/'debs',stdout=log,stderr=subprocess.STDOUT,check=True)
    for deb in (RUNTIME/'debs').glob('*.deb'):run(['dpkg-deb','-x',str(deb),str(RUNTIME/'root')])
    libs=list(Path('/usr/lib').glob('*/libgz-rendering8-ogre2.so.8'))
    if libs:
        alias=RUNTIME/'plugins/libgz-rendering-ogre2.so'
        if alias.is_symlink():alias.unlink()
        alias.symlink_to(libs[0])
    prefix=RUNTIME/'root/opt/ros/humble'
    lines=['# Generated local dependency environment; no system packages modified.']
    values={'AMENT_PREFIX_PATH':str(prefix)+':/opt/ros/humble','LD_LIBRARY_PATH':str(prefix/'lib')+':/opt/ros/humble/lib',
        'PYTHONPATH':str(prefix/'local/lib/python3.10/dist-packages')+':'+str(prefix/'lib/python3.10/site-packages')+':/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages',
        'PATH':str(prefix/'bin'),'GZ_RENDERING_PLUGIN_PATH':str(RUNTIME/'plugins'),
        'GZ_RENDERING_RESOURCE_PATH':str(RUNTIME/'root/usr/share/gz/gz-rendering8')}
    for name,value in values.items():lines.append('export '+name+'='+shlex.quote(value)+'${'+name+':+:$'+name+'}')
    lines.append('export ROS_LOG_DIR='+shlex.quote(str(RUNTIME/'ros_logs')))
    (RUNTIME/'env.sh').write_text('\n'.join(lines)+'\n')
    print('Ready. The launcher automatically loads '+str(RUNTIME/'env.sh'))

if __name__=='__main__':main()
