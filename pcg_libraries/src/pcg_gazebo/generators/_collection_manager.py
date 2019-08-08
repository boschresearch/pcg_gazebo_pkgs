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

class _CollectionManager(object):
    _INSTANCE = None

    def __init__(self):
        self._collection = dict()

    @property
    def tags(self):
        return self._collection.keys()

    @property
    def size(self):
        return len(self.tags)

    def add(self, element):
        if not self.has_element(element.name):
            self._collection[element.name] = element
            return True
        else:
            return False

    def create_empty(self, *args, **kwargs):
        raise NotImplementedError()

    def remove(self, tag):
        if self.has_element(tag):
            del self._collection[tag]
            return True
        return False

    def get(self, tag):
        if self.has_element(tag):
            return self._collection[tag]
        else:
            return None

    def has_element(self, tag):
        return tag in self.tags