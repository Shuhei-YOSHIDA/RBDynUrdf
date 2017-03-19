// This file is part of RBDyn.
//
// RBDyn is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// RBDyn is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with RBDyn.  If not, see <http://www.gnu.org/licenses/>.

#pragma once

// includes
// std
#include <string>

// RBDyn
#include <RBDyn/Body.h>
#include <RBDyn/Joint.h>
#include <RBDyn/MultiBodyGraph.h>

// RBDynUrdf
#include "Reader.h"

// XYZSarm Robot

//                b4
//             j3 | RevX
//  Root     j0   |   j1     j2
//  ---- b0 ---- b1 ---- b2 ----b3
//  Fixed    RevX   RevY    RevZ

//  X
//  ^
//  |
//   -- > Y

std::string XYZSarmUrdf(
R"(
  <robot name="XYZSarm">
    <link name="b0">
      <inertial>
        <origin rpy="0 0 0" xyz="0 0 0" />
        <mass value="1" />
        <inertia ixx="0.1" ixy="0.0" ixz="0.0"
                iyy="0.05" iyz="0.0" izz="0.001" />
      </inertial>
    </link>
    <link name="b1">
      <inertial>
        <origin rpy="0 0 0" xyz="0 0.5 0" />
        <mass value="5." />
        <inertia ixx="0.1" ixy="0.0" ixz="0.0"
                iyy="0.05" iyz="0.0" izz="0.001" />
      </inertial>
    </link>
    <link name="b2">
      <inertial>
        <origin rpy="0 0 0" xyz="0 0.5 0" />
        <mass value="2." />
        <inertia ixx="0.1" ixy="0.0" ixz="0.0"
                iyy="0.05" iyz="0.0" izz="0.001" />
      </inertial>
    </link>
    <link name="b3">
      <inertial>
        <origin rpy="0 0 0" xyz="0 0.5 0" />
        <mass value="1.5" />
        <inertia ixx="0.1" ixy="0.0" ixz="0.0"
                iyy="0.05" iyz="0.0" izz="0.001" />
      </inertial>
    </link>
    <link name="b4">
      <inertial>
        <origin rpy="0 0 0" xyz="0.5 0 0" />
        <mass value="1" />
        <inertia ixx="0.1" ixy="0.0" ixz="0.0"
                iyy="0.05" iyz="0.0" izz="0.001" />
      </inertial>
    </link>


    <joint name="j0" type="revolute">
      <parent link="b0" />
      <child link="b1" />
      <origin rpy="0 0 0" xyz="0 1 0" />
      <axis xyz="1 0 0" />
      <limit lower="-1" upper="1" velocity="10" effort="50" />
    </joint>
    <joint name="j1" type="revolute">
      <parent link="b1" />
      <child link="b2" />
      <origin rpy="0 0 0" xyz="0 1 0" />
      <axis xyz="0 1 0" />
      <limit lower="-1" upper="1" velocity="10" effort="50" />
    </joint>
    <joint name="j2" type="revolute">
      <parent link="b2" />
      <child link="b3" />
      <origin rpy="0 0 0" xyz="0 1 0" />
      <axis xyz="0 0 1" />
      <limit lower="-1" upper="1" velocity="10" effort="50" />
    </joint>
    <joint name="j3" type="continuous">
      <parent link="b1" />
      <child link="b4" />
      <origin rpy="1. 0 0" xyz="1 0 0" />
      <axis xyz="1 0 0" />
    </joint>
  </robot>
)"
);



/// @return An simple XYZ spherical arm with Y as up axis.
rbdyn_urdf::Urdf makeXYZSarm()
{
  using namespace Eigen;
  using namespace sva;
  using namespace rbd;

  rbdyn_urdf::Urdf urdf;

  Matrix3d I0, I1, I2, I3, I4;

  // computed with rbdyn_urdf python
  I0 << 0.1 , 0.0 , 0.0,
        0.0 , 0.05 , 0.0,
        0.0 , 0.0 , 0.001;

  I1 << 1.35 , 0.0 , 0.0,
        0.0 , 0.05 , 0.0,
        0.0 , 0.0 , 1.251;

  I2 << 0.6 , 0.0 , 0.0,
        0.0 , 0.05 , 0.0,
        0.0 , 0.0 , 0.501;

  I3 << 0.475 , 0.0 , 0.0,
        0.0 , 0.05 , 0.0,
        0.0 , 0.0 , 0.376;

  I4 << 0.1 , 0.0 , 0.0,
        0.0 , 0.3 , 0.0,
        0.0 , 0.0 , 0.251;

  Body b0(1., Vector3d::Zero(), I0, "b0");
  Body b1(5., Vector3d(0., 0.5, 0.), I1, "b1");
  Body b2(2., Vector3d(0., 0.5, 0.), I2, "b2");
  Body b3(1.5, Vector3d(0., 0.5, 0.), I3, "b3");
  Body b4(1., Vector3d(0.5, 0., 0.), I4, "b4");

  urdf.mbg.addBody(b0);
  urdf.mbg.addBody(b1);
  urdf.mbg.addBody(b2);
  urdf.mbg.addBody(b3);
  urdf.mbg.addBody(b4);

  Joint j0(Joint::RevX, true, "j0");
  Joint j1(Joint::RevY, true, "j1");
  Joint j2(Joint::RevZ, true, "j2");
  Joint j3(Joint::RevX, true, "j3");

  urdf.mbg.addJoint(j0);
  urdf.mbg.addJoint(j1);
  urdf.mbg.addJoint(j2);
  urdf.mbg.addJoint(j3);


  PTransformd to(Vector3d(0., 1., 0.));
  PTransformd from(PTransformd::Identity());

  urdf.mbg.linkBodies("b0", to, "b1", from, "j0");
  urdf.mbg.linkBodies("b1", to, "b2", from, "j1");
  urdf.mbg.linkBodies("b2", to, "b3", from, "j2");
  urdf.mbg.linkBodies("b1", PTransformd(sva::RotX(1.), Vector3d(1., 0., 0.)),
                      "b4", from, "j3");

  // fill limits
  urdf.limits.ql["j0"] = -1.;
  urdf.limits.ql["j1"] = -1.;
  urdf.limits.ql["j2"] = -1.;

  urdf.limits.qu["j0"] = 1.;
  urdf.limits.qu["j1"] = 1.;
  urdf.limits.qu["j2"] = 1.;

  urdf.limits.vl["j0"] = -10.;
  urdf.limits.vl["j1"] = -10.;
  urdf.limits.vl["j2"] = -10.;

  urdf.limits.vu["j0"] = 10.;
  urdf.limits.vu["j1"] = 10.;
  urdf.limits.vu["j2"] = 10.;

  urdf.limits.tl["j0"] = -50.;
  urdf.limits.tl["j1"] = -50.;
  urdf.limits.tl["j2"] = -50.;

  urdf.limits.tu["j0"] = 50.;
  urdf.limits.tu["j1"] = 50.;
  urdf.limits.tu["j2"] = 50.;

  return std::move(urdf);
}
