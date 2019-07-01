# Copyright (c) 2019 - The Procedural Generation for Gazebo authors
# For information on the respective copyright owner see the NOTICE file
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

import os 
import sys
import subprocess


if __name__ == '__main__':
    # devel_path = sys.argv[1::][0]

    # install_path = os.path.join(
    #     devel_path,
    #     'lib',
    #     'python{}.{}'.format(sys.version_info[0], sys.version_info[1]),
    #     'dist-packages')

    pkg_path = os.path.dirname(os.path.realpath(__file__))
    req_file = os.path.join(pkg_path, 'requirements.txt')

    if os.path.isfile(req_file):
        with open(req_file) as f:
            for line in f:
                pkg = line.replace('\n', '')

                pkg_exists = True
                try:
                    cmd = 'python{}.{} -m pip show {}'.format(
                        sys.version_info[0],
                        sys.version_info[1],
                        pkg
                    )
                    subprocess.check_output(cmd.split())
                except subprocess.CalledProcessError:
                    pkg_exists = False

                if not pkg_exists:
                    # cmd = 'python{}.{} -m pip install {} --upgrade --target {}'.format(
                    #     sys.version_info[0], 
                    #     sys.version_info[1],
                    #     pkg,
                    #     install_path
                    # )

                    cmd = 'python{}.{} -m pip install {} --upgrade --user'.format(
                        sys.version_info[0], 
                        sys.version_info[1],
                        pkg
                    )

                    subprocess.check_output(cmd.split())