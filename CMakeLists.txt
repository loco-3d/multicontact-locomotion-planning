#
# Copyright (c) 2020 CNRS
# Authors: Pierre Fernbach
#
#
# This file is part of multicontact-locomotion-planning
# multicontact-locomotion-planning is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp-core is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-core  If not, see
# <http://www.gnu.org/licenses/>.

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

SET(PROJECT_ORG loco-3d)
SET(PROJECT_NAME multicontact-locomotion-planning)
SET(PROJECT_DESCRIPTION "Wholebody motion planner for multicontact locomotion")
SET(PROJECT_URL "https://github.com/${PROJECT_ORG}/${PROJECT_NAME}")

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/python.cmake)

COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX)
PROJECT(${PROJECT_NAME} ${PROJECT_ARGS})

FINDPYTHON()

ADD_PROJECT_DEPENDENCY(eigenpy 2.0 REQUIRED)
ADD_PROJECT_DEPENDENCY(ndcurves 1.0 REQUIRED)
ADD_PROJECT_DEPENDENCY(pinocchio 2.2 REQUIRED)
ADD_PROJECT_DEPENDENCY(multicontact-api 2.0 REQUIRED)
ADD_PROJECT_DEPENDENCY(hpp-rbprm-corba 4.8 REQUIRED)

ADD_PROJECT_DEPENDENCY(tsid 1.2)
ADD_PROJECT_DEPENDENCY(momentumopt 1.0)
#ADD_PROJECT_DEPENDENCY(sl1m 1.0) #TODO

INSTALL(DIRECTORY momentumopt_configs
  DESTINATION "share/mlp")

ADD_SUBDIRECTORY(python)
ADD_SUBDIRECTORY(tests)
