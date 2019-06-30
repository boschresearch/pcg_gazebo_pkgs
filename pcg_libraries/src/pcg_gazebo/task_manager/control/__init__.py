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
import pkgutil
import inspect

__all__ = []
for loader, module_name, is_pkg in pkgutil.walk_packages(__path__):    
    __all__.append(module_name)
    _module = loader.find_module(module_name).load_module(module_name)    
    globals()[module_name] = _module
    for name, obj in inspect.getmembers(_module):
        if inspect.isclass(obj):            
            globals()[name] = obj
