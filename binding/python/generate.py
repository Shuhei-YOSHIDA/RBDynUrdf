# This file is part of Tasks.
#
# Tasks is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# Tasks is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with Tasks.  If not, see <http://www.gnu.org/licenses/>.

from pybindgen import *
import sys


def import_rbd_types(mod):
  mod.add_class('MultiBodyGraph', foreign_cpp_namespace='rbd', import_from_module='rbdyn')
  mod.add_class('MultiBody', foreign_cpp_namespace='rbd', import_from_module='rbdyn')


if __name__ == '__main__':
  if len(sys.argv) < 2:
    sys.exit(1)

  rbdyn_urdf = Module('_rbdyn_urdf', cpp_namespace='::rbdyn_urdf')
  rbdyn_urdf.add_include('<Reader.h>')
  rbdyn_urdf.add_include('<Writer.h>')

  run_ex = rbdyn_urdf.add_exception('std::runtime_error', foreign_cpp_namespace=' ',
                                    message_rvalue='%(EXC)s.what()')

  # import rbd types
  import_rbd_types(rbdyn_urdf)

  # build list type
  rbdyn_urdf.add_container('std::map<std::string, double>', ('std::string', 'double'), 'map')

  # build struct
  limits = rbdyn_urdf.add_struct('Limits')
  limits.add_instance_attribute('ql', 'std::map<std::string, double>')
  limits.add_instance_attribute('qu', 'std::map<std::string, double>')
  limits.add_instance_attribute('vl', 'std::map<std::string, double>')
  limits.add_instance_attribute('vu', 'std::map<std::string, double>')
  limits.add_instance_attribute('tl', 'std::map<std::string, double>')
  limits.add_instance_attribute('tu', 'std::map<std::string, double>')

  urdf = rbdyn_urdf.add_struct('Urdf')
  urdf.add_instance_attribute('mbg', 'rbd::MultiBodyGraph')
  urdf.add_instance_attribute('limits', 'rbdyn_urdf::Limits')

  # build function
  rbdyn_urdf.add_function('readUrdf', retval('rbdyn_urdf::Urdf'),
                          [param('const std::string&', 'urdf')],
                          throw=[run_ex])
  rbdyn_urdf.add_function('readUrdfFile', retval('rbdyn_urdf::Urdf'),
                          [param('const std::string&', 'fileName')],
                          throw=[run_ex])

  rbdyn_urdf.add_function('writeUrdf', None,
                          [param('const std::string&', 'filename'),
                           param('const std::string&', 'robotName'),
                           param('const rbd::MultiBody&', 'mb'),
                           param('const rbdyn_urdf::Limits&', 'limits')],
                          throw=[run_ex])
  rbdyn_urdf.add_function('writeUrdf', None,
                          [param('const std::string&', 'filename'),
                           param('const std::string&', 'robotName'),
                           param('const rbd::MultiBody&', 'mb')],
                          throw=[run_ex])

  with open(sys.argv[1], 'w') as f:
    rbdyn_urdf.generate(f)

