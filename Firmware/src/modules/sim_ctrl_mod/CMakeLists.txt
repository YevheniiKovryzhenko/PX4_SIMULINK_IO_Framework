############################################################################
#          Auburn University Aerospace Engineering Department
#             Aero-Astro Computational and Experimental lab
#
#     Copyright (C) 2024  Yevhenii Kovryzhenko
#
#     This program is free software: you can redistribute it and/or modify
#     it under the terms of the GNU General Public License as published by
#     the Free Software Foundation, either version 3 of the License, or
#     (at your option) any later version.
#
#     This program is distributed in the hope that it will be useful,
#     but WITHOUT ANY WARRANTY; without even the implied warranty of
#     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#     GNU General Public License for more details.
#
#     You should have received a copy of the
#     GNU AFFERO GENERAL PUBLIC LICENSE Version 3
#     along with this program.  If not, see <https://www.gnu.org/licenses/>
############################################################################

px4_add_module(
	MODULE custom__sim_ctrl_mod
	MAIN sim_ctrl_mod
	SRCS
		sim_ctrl_mod.cpp
		sticks.cpp
	MODULE_CONFIG
		module.yaml
		module_tilt_wing.yaml
		module_quadcopter.yaml
	)

