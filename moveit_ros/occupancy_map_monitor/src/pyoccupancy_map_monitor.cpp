/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, Peter Mitrano
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * The name of Peter Mitrano may not be used to endorse or promote
 *     products derived from this software without specific prior
 *     written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Peter Mitrano */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <moveit/occupancy_map_monitor/occupancy_map_monitor.h>

namespace py = pybind11;

using namespace occupancy_map_monitor;

PYBIND11_MODULE(pymoveit_ros_occupancy_map_monitor, m)
{
  py::class_<OccMapTree, OccMapTreePtr>(m, "OccMapTree")
      //
      ;
  py::class_<OccupancyMapMonitor, std::shared_ptr<OccupancyMapMonitor>>(m, "OccpancyMapMonitor")
      .def(py::init<double>(), py::arg("map_resolution") = 0.0)
      .def("getMapFrame", &OccupancyMapMonitor::getMapFrame)
      .def("getMapResolution", &OccupancyMapMonitor::getMapResolution)
      .def("isActive", &OccupancyMapMonitor::isActive)
      .def("publishDebugInformation", &OccupancyMapMonitor::publishDebugInformation)
      .def("setMapFrame", &OccupancyMapMonitor::setMapFrame)
      .def("startMonitor", &OccupancyMapMonitor::startMonitor)
      .def("stopMonitor", &OccupancyMapMonitor::stopMonitor)
      .def("getOcTree", py::overload_cast<>(&OccupancyMapMonitor::getOcTreePtr, py::const_))
      //
      ;
}
