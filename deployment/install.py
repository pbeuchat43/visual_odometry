#!/usr/bin/env python3
"""Build one explicit estimator backend without replacing host CUDA/OpenCV."""
# Copyright (c) 2026 Paul N. Beuchat. MIT License.
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import shlex
import shutil
import subprocess
import urllib.request


ROOT = Path(__file__).resolve().parent.parent


def run(argv, **kwargs):
    return subprocess.run(list(map(str, argv)), check=True, **kwargs)


def output(argv, **kwargs):
    return subprocess.check_output(list(map(str, argv)), text=True, **kwargs).strip()


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def files(root):
    return {str(path.relative_to(root)): sha(path) for path in sorted(root.rglob('*'))
            if path.is_file() and not path.is_symlink()}


def host(backend):
    if platform.machine() != 'aarch64' or platform.freedesktop_os_release().get('VERSION_ID') != '24.04':
        raise ValueError('This dependency recipe requires Ubuntu 24.04 arm64')
    names = ['nvidia-l4t-core', 'cuda-cudart-13-2', 'ros-jazzy-rclcpp', 'ros-jazzy-nav-msgs']
    if backend == 'native':
        names += ['libnvvpi4', 'ros-jazzy-cv-bridge', 'ros-jazzy-visualization-msgs',
                  'ros-jazzy-diagnostic-msgs', 'ros-jazzy-tf2-geometry-msgs', 'ros-jazzy-message-filters']
    versions = {name: output(['dpkg-query', '-W', '-f=${Version}', name]) for name in names}
    if versions['nvidia-l4t-core'].split('-')[0] != '39.2.1' or not versions['cuda-cudart-13-2'].startswith('13.2.'):
        raise ValueError('Install the qualified L4T 39.2.1 / CUDA 13.2 host first')
    return versions


def download(entry, cache):
    path = cache / (entry['sha256'] + '.deb')
    if not path.exists():
        temporary = path.with_suffix('.partial')
        with urllib.request.urlopen(entry['url'], timeout=60) as response, temporary.open('wb') as stream:
            shutil.copyfileobj(response, stream)
        if sha(temporary) != entry['sha256']:
            raise ValueError('Archive hash mismatch: ' + entry['name'])
        temporary.rename(path)
    if sha(path) != entry['sha256']:
        raise ValueError('Corrupt archive cache: ' + entry['name'])
    return path


def sources(work, camera):
    source = work / 'src'
    source.mkdir()
    for name, origin in [('rgbd_camera_interfaces', camera / 'rgbd_camera_interfaces'),
                         ('visual_odometry_interfaces', ROOT / 'visual_odometry_interfaces'),
                         ('visual_odometry', ROOT / 'visual_odometry')]:
        shutil.copytree(origin, source / name, ignore=shutil.ignore_patterns('__pycache__', '.git'))
    return source


def native_environment(destination):
    dep = destination / 'dependencies'
    ros = dep / 'opt/ros/jazzy'
    return ('source /opt/ros/jazzy/setup.bash\n'
            f'export AMENT_PREFIX_PATH={shlex.quote(str(ros))}:"${{AMENT_PREFIX_PATH:-}}"\n'
            f'export CMAKE_PREFIX_PATH={shlex.quote(str(ros))}:"${{CMAKE_PREFIX_PATH:-}}"\n'
            f'export LD_LIBRARY_PATH={shlex.quote(str(ros / "lib"))}:{shlex.quote(str(dep / "opt/nvidia/cvcuda0/lib"))}:"${{LD_LIBRARY_PATH:-}}"\n'
            f'export PYTHONPATH={shlex.quote(str(ros / "local/lib/python3.12/dist-packages"))}:{shlex.quote(str(ros / "lib/python3.12/site-packages"))}:"${{PYTHONPATH:-}}"\n')


def install(args):
    destination, work, cache = args.destination.resolve(), args.work.resolve(), args.cache.resolve()
    if destination.exists() or work.exists():
        raise ValueError('Use new destination/work paths; preserve failed attempts for inspection')
    for revision in (args.source_id, args.camera_source_id):
        if not re.fullmatch('[0-9a-f]{40}', revision):
            raise ValueError('Exact source commit identities are required')
    lock_path = ROOT / 'deployment/dependencies.lock.json'
    lock = json.loads(lock_path.read_text())
    host_versions = host(args.backend)
    destination.mkdir(parents=True)
    work.mkdir(parents=True)
    cache.mkdir(parents=True, exist_ok=True)
    source = sources(work, args.camera_source.resolve())
    source_hashes = files(source)
    preset = ROOT / 'visual_odometry/config/estimator.yaml'
    shutil.copyfile(preset, destination / 'estimator.yaml')
    manifest = dict(schema_version=1, backend=args.backend, source_id=args.source_id,
                    camera_source_id=args.camera_source_id, source_files=source_hashes,
                    dependency_lock_sha256=sha(lock_path), host_packages=host_versions,
                    preset_sha256=sha(preset))
    if args.backend == 'docker':
        shutil.copyfile(ROOT / 'deployment/Dockerfile', work / 'Dockerfile')
        (work / 'run.sh').write_text('#!/bin/bash\nset -e\nsource /opt/ros/jazzy/setup.bash\nsource /opt/visual_odometry/install/local_setup.bash\nexec ros2 launch visual_odometry odometry.launch.py config_file:="${1:-/run/robot/prepared.json}"\n')
        run(['docker', 'build', '--build-arg', 'BASE=' + lock['base_image'], '--build-arg',
             'ISAAC_VERSION=' + lock['upstream_version'], '--iidfile', destination / 'image.id', work])
        image = (destination / 'image.id').read_text().strip()
        if not re.fullmatch('sha256:[0-9a-f]{64}', image):
            raise ValueError('Builder returned an invalid image ID')
        packages = output(['docker', 'run', '--rm', '--network=none', '--entrypoint=/bin/cat', image, '/opt/visual-odometry-packages.txt'])
        (destination / 'packages.txt').write_text(packages + '\n')
        manifest.update(image=image, base_image=lock['base_image'], argv=['/bin/bash', '/opt/visual_odometry/run.sh'])
    else:
        dep = destination / 'dependencies'
        dep.mkdir()
        for entry in lock['native_archives']:
            run(['dpkg-deb', '-x', download(entry, cache), dep])
        environment = native_environment(destination)
        (destination / 'environment.sh').write_text(environment)
        command = ['colcon', 'build', '--base-paths', source, '--build-base', work / 'build',
                   '--install-base', destination / 'install', '--merge-install', '--executor', 'sequential',
                   '--cmake-args', '-DCMAKE_BUILD_TYPE=Release', '-DBUILD_TESTING=OFF', '-DPython3_EXECUTABLE=/usr/bin/python3']
        run(['/bin/bash', '-c', 'set -e\n' + environment + 'export CMAKE_BUILD_PARALLEL_LEVEL=2\nexec "$@"', 'native-build', *map(str, command)], cwd=work)
        # Resolve ELF dependencies before exposing a usable prefix. Dynamic GPU
        # plugin loading and real sensor execution remain qualification checks.
        check = 'set -e\n' + environment + 'source ' + shlex.quote(str(destination / 'install/local_setup.bash')) + '\nexec "$@"'
        library = dep / 'opt/ros/jazzy/lib/libvisual_slam_node.so'
        linked = output(['/bin/bash', '-c', check, 'native-check', 'ldd', str(library)])
        if 'not found' in linked:
            raise ValueError('Missing native dependencies:\n' + linked)
        (destination / 'link-check.txt').write_text(linked + '\n')
        (destination / 'run.sh').write_text('#!/bin/bash\nset -e\n' + environment +
            'source ' + shlex.quote(str(destination / 'install/local_setup.bash')) +
            '\nexec ros2 launch visual_odometry odometry.launch.py config_file:="$1"\n')
        manifest.update(archives=lock['native_archives'], argv=['/bin/bash', str(destination / 'run.sh')])
    manifest['files'] = files(destination)
    (destination / 'manifest.json').write_text(json.dumps(manifest, indent=2, sort_keys=True) + '\n')
    print(json.dumps(dict(manifest=str(destination / 'manifest.json'), sha256=sha(destination / 'manifest.json'))))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend', required=True, choices=['docker', 'native'])
    for name in ('camera-source', 'destination', 'work', 'cache'):
        parser.add_argument('--' + name, required=True, type=Path)
    parser.add_argument('--source-id', required=True)
    parser.add_argument('--camera-source-id', required=True)
    install(parser.parse_args())
