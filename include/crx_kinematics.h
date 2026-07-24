/****************************************************************************
**
** Copyright (c) 2015-2026 RoboDK Global.
** Contact: https://robodk.com/
**
** This file is part of the RoboDK API.
**
** Licensed under the Apache License, Version 2.0 (the "License");
** you may not use this file except in compliance with the License.
** You may obtain a copy of the License at
**
**     http://www.apache.org/licenses/LICENSE-2.0
**
** Unless required by applicable law or agreed to in writing, software
** distributed under the License is distributed on an "AS IS" BASIS,
** WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
** See the License for the specific language governing permissions and
** limitations under the License.
**
** RoboDK is a registered trademark of RoboDK Global.
**
****************************************************************************/

#ifndef CRXKINEMATICS_H
#define CRXKINEMATICS_H

#define CRX_KINEMATICS_VERSION_MAJOR 0
#define CRX_KINEMATICS_VERSION_MINOR 3
#define CRX_KINEMATICS_VERSION_PATCH 1
#define CRX_KINEMATICS_VERSION_STRING "0.3.1"

#if defined(_WIN32)
#define MYLIB_EXPORT __declspec(dllexport)
#define MYLIB_IMPORT __declspec(dllimport)
#else
#define MYLIB_EXPORT
#define MYLIB_IMPORT
#endif

// Define real_T if not already defined by your environment
typedef double real_T;
typedef void robot_T;

extern "C" {

/*!
 * \brief SolveFK
 * Calculate the forward kinematics solution: provided a set of joints in mm/deg
 * calculates the pose of the end effector with repect to the robot base
 * (Matrix4x4/pose[16])
 * \param joints
 * robot joints in mm or deg
 * \param pose
 * forward kinematics solution. Values are a 16-double array [nx,ny,nz,0,
 * ox,oy,oz,0, ax,ay,az,0, x,y,z,1]
 * \param ptr_robot
 * pointer to the robot parameters, if any
 * \return returns 1 if the solution is valid, returns 0 if there is no solution
 * (such as joints out of limits), return -1 if we want to use the
 * default/generic forward kinematics of RoboDK
 */
MYLIB_EXPORT int SolveFK(const real_T *joints, real_T pose[16],
                         const robot_T *ptr_robot);

/*!
 * \brief SolveFK_CAD
 * Calculate the forward kinematics solution including the poses for all joints:
 * this function is similar to SolveFK but it is used to display the 3D model.
 * \param joints
 * robot joints (in mm or deg)
 * \param pose
 * forward kinematics solution. Values are a 16-double array [nx,ny,nz,0,
 * ox,oy,oz,0, ax,ay,az,0, x,y,z,1]
 * \param joint_poses
 * array of poses packed as a multiple of 16*nposes
 * \param max_poses
 * number of poses available or that must be set
 * \param ptr_robot
 * pointer to the robot parameters, if any.
 * \return Returns 1 if the solution is valid, returns 0 if there is no solution
 * (such as joints out of limits), return -1 if we want to use the
 * default/generic forward kinematics of RoboDK
 */
MYLIB_EXPORT int SolveFK_CAD(const real_T *joints, real_T pose[16],
                             real_T *joint_poses, int max_poses,
                             const robot_T *ptr_robot);

/*!
 * \brief Calculate the inverse kinematics solution: calculates the robot joints
 * given the pose of the robot flange with respect to the robot base
 * \param pose
 * pose of the robot. Values are a 16-double array [nx,ny,nz,0, ox,oy,oz,0,
 * ax,ay,az,0, x,y,z,1]
 * \param joints
 * robot joints solution in mm or deg
 * \param joints_all
 * list of optional solutions in mm or deg as a 12*n_solutions array (it should
 * include the joints solution)
 * \param max_solutions
 * maximum number of solution. This is the size of the joints_all solutions
 * buffer. For example, if max_solutions is 8, you can provide up to 8 valid
 * solutions (8x12 array)
 * \param joints_approx
 * These joints hold the current robot joints. This can be used as a hint to
 * obtain or choose a solution.
 * \param ptr_robot
 * pointer to the robot parameters, if any.
 * \return returns the number of valid solutions (equal or less than
 * max_solutions), if any. Returns 0 if there is no solution (for example:
 * target out of reach), return -1 if we want to use the default iterative
 * solution provided by RoboDK
 */
MYLIB_EXPORT int SolveIK(const real_T pose[16], real_T *joints,
                         real_T *joints_all, int max_solutions,
                         const real_T *joints_approx, const robot_T *ptr_robot);

/*!
 * \brief Return RoboDK's configuration flags for the provided joints.
 * \param joints robot joints in mm or deg
 * \param config configuration flags [REAR, LOWERARM, FLIP], each 0 or 1
 * \param ptr_robot pointer to the robot parameters
 * \return 1 when config was provided, or -1 to use RoboDK's default values
 */
MYLIB_EXPORT int Joints2Config(const real_T *joints, real_T config[3],
                               const robot_T *ptr_robot);
}

#endif // CRXKINEMATICS_H
