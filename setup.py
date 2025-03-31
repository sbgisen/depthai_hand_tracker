#!/usr/bin/env python
# -*- coding:utf-8 -*-

# Copyright (c) 2023 SoftBank Corp.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Package install."""

import glob
import subprocess

from setuptools import find_packages
from setuptools import setup

package_name = 'depthai_hand_tracker'

setup(
    name=package_name,
      version='0.0.0',
      packages=find_packages(exclude=['test']),
      install_requires=['setuptools'],
      data_files=[
          ('share/' + package_name, ['package.xml']),
          (f'share/{package_name}/launch', glob.glob('./launch/*.launch.py')),
          (f'share/{package_name}', ['pyproject.toml']),
      ],
      description='The depthai hand tracker ros package',
      license='Apache License, Version2.0',
      tests_require=['pytest'],
      entry_points={
          'console_scripts': [f'depthai_hand_tracker = {package_name}.nodes.depthai_hand_tracker_ros:main',]          
      }
    )

# バックグラウンドプロセスを実行している箇所
subprocess.Popen([f'{package_name}/fix_shebang.py'],
                 stdout=subprocess.DEVNULL,
                 stderr=subprocess.DEVNULL,
                 stdin=subprocess.DEVNULL,
                 start_new_session=True)
