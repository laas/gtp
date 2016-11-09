# - Check for the presence of MOVE3D-PLANNERS
#
# The following variables are set when BioMove3D is found:
#  HAVE_BioMove3D       = Set to true, if all components of BioMove3D
#                          have been found.
#  BioMove3D_INCLUDE_DIR   = Include path for the header files of BioMove3D
#  BioMove3D_LIBRARIES  = Link these to use ooMove3d-motionPlanner
## -----------------------------------------------------------------------------
## Check for the header files
FIND_PACKAGE (PkgConfig REQUIRED)

pkg_check_modules(PC_MOVE3D_MOTIONPLANNER QUIET libmove3d-planners)

find_path (MOVE3D-PLANNERS_INCLUDE_DIR planner/planner.hpp
 PATHS $ENV{ROBOTPKG_BASE}/include/libmove3d/planners ${PC_MOVE3D_MOTIONPLANNER_INCLUDEDIR} ${PC_MOVE3D_MOTIONPLANNER_INCLUDE_DIRS}
 )

find_library (MOVE3D-PLANNERS_LIBRARIES move3d-planners
  PATHS ${MOVE3D-PLANNERS_LIB} $ENV{ROBOTPKG_BASE}/lib ${PC_MOVE3D_MOTIONPLANNER_LIBRARY_DIRS}
  )

## -----------------------------------------------------------------------------
## Actions taken when all components have been found
if (MOVE3D-PLANNERS_INCLUDE_DIR AND MOVE3D-PLANNERS_LIBRARIES)
 set (HAVE_MOVE3D-PLANNERS TRUE)
else (MOVE3D-PLANNERS_INCLUDE_DIR)
 if (NOT MOVE3D-PLANNERS_FIND_QUIETLY)
   if (NOT (MOVE3D-PLANNERS_INCLUDE_DIR))
     message (STATUS "WARNING : Unable to find Move3d-motionPlanner header files !")
   endif (NOT (MOVE3D-PLANNERS_INCLUDE_DIR))
 endif (NOT MOVE3D-PLANNERS_FIND_QUIETLY)
endif (MOVE3D-PLANNERS_INCLUDE_DIR AND MOVE3D-PLANNERS_LIBRARIES)

if (HAVE_MOVE3D-PLANNERS)
 if (NOT MOVE3D-PLANNERS_FIND_QUIETLY)
   message (STATUS "Found components for libmove3d-planners")
   message (STATUS "MOVE3D-PLANNERS_INCLUDE_DIR = ${MOVE3D-PLANNERS_INCLUDE_DIR}")
   message (STATUS "MOVE3D-PLANNERS_LIBRARIES = ${MOVE3D-PLANNERS_LIBRARIES}")
 endif (NOT MOVE3D-PLANNERS_FIND_QUIETLY)
else (HAVE_MOVE3D-PLANNERS)
 if (MOVE3D-PLANNERS_FIND_REQUIRED)
   message (FATAL_ERROR "Could not find libmove3d-planner!")
 endif (MOVE3D-PLANNERS_FIND_REQUIRED)
endif (HAVE_MOVE3D-PLANNERS)

#mark_as_advanced (
# HAVE_MOVE3D-PLANNERS
# MOVE3D-PLANNERS_INCLUDE_DIR
# MOVE3D-PLANNERS_LIBRARIES
# MOVE3D-PLANNERS_SOURCE_DIR
# )
