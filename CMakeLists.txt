cmake_minimum_required(VERSION 3.11)
if(POLICY CMP0169)
    cmake_policy(SET CMP0169 OLD)
endif()
project(kalibrate-hydrasdr C CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

option(HYDRASDR_FETCH_FROM_GIT "Fetch libhydrasdr from GitHub if not found" ON)
set(HYDRASDR_FETCH_GIT_REPO "https://github.com/hydrasdr/hydrasdr-host.git"
    CACHE STRING "Git repository URL for hydrasdr-host")
set(HYDRASDR_FETCH_GIT_TAG "main"
    CACHE STRING "Git tag/branch/commit for hydrasdr-host")

include_directories(${CMAKE_SOURCE_DIR}/src)

# ==============================================================================
# Helper: Print Library Status
# ==============================================================================
function(print_lib_status name source lib inc)
    message(STATUS "---------------------------------------------------")
    message(STATUS "[${name}] Status:")
    message(STATUS "   + Source:  ${source}")
    message(STATUS "   + Library: ${lib}")
    message(STATUS "   + Include: ${inc}")
    message(STATUS "---------------------------------------------------")
endfunction()

message(STATUS "=============================================================")
message(STATUS "Configuring kalibrate-hydrasdr")
message(STATUS "System : ${CMAKE_SYSTEM_NAME} (${CMAKE_SYSTEM_PROCESSOR})")
message(STATUS "Source : ${CMAKE_SOURCE_DIR}")
message(STATUS "Prefix : ${CMAKE_PREFIX_PATH}")
message(STATUS "=============================================================")

find_package(PkgConfig QUIET)

# ==============================================================================
# 1. LibUSB-1.0
# ==============================================================================
message(STATUS ">> Searching for LibUSB-1.0...")

if(PKG_CONFIG_FOUND)
    pkg_check_modules(PC_LIBUSB QUIET libusb-1.0)
endif()

find_path(LIBUSB_INCLUDE_DIR
    NAMES libusb.h
    HINTS ${PC_LIBUSB_INCLUDE_DIRS} ${CMAKE_PREFIX_PATH}
    PATH_SUFFIXES include/libusb-1.0 include
)

find_library(LIBUSB_LIBRARY
    NAMES usb-1.0 libusb-1.0
    HINTS ${PC_LIBUSB_LIBRARY_DIRS} ${CMAKE_PREFIX_PATH}
    PATH_SUFFIXES lib bin
)

if(NOT LIBUSB_INCLUDE_DIR OR NOT LIBUSB_LIBRARY)
    message(FATAL_ERROR "[FAIL] LibUSB-1.0 not found.")
endif()

set(LIBUSB_FOUND TRUE)
set(LIBUSB_INCLUDE_DIRS ${LIBUSB_INCLUDE_DIR})
set(LIBUSB_LIBRARIES ${LIBUSB_LIBRARY})

set(LIBUSB_SOURCE
    $<IF:$<BOOL:${PC_LIBUSB_FOUND}>,PkgConfig,Manual Search>
)

get_filename_component(LIBUSB_LIBRARY_DIR "${LIBUSB_LIBRARY}" DIRECTORY)

print_lib_status("LibUSB" "${LIBUSB_SOURCE}" "${LIBUSB_LIBRARIES}" "${LIBUSB_INCLUDE_DIRS}")

# Force cache for HydraSDRConfig / FindLIBUSB compatibility
set(LIBUSB_INCLUDE_DIR "${LIBUSB_INCLUDE_DIRS}" CACHE PATH "" FORCE)
set(LIBUSB_LIBRARY     "${LIBUSB_LIBRARIES}"    CACHE FILEPATH "" FORCE)
set(LIBUSB_LIBRARIES   "${LIBUSB_LIBRARIES}"    CACHE FILEPATH "" FORCE)


# ==============================================================================
# 2. FFTW3
# ==============================================================================
message(STATUS ">> Searching for FFTW3...")

if(PKG_CONFIG_FOUND)
    pkg_check_modules(PC_FFTW3 QUIET fftw3)
endif()

if(NOT PC_FFTW3_FOUND)
    find_path(FFTW3_INCLUDE_DIRS NAMES fftw3.h
        HINTS ${CMAKE_PREFIX_PATH} ${PC_FFTW3_INCLUDE_DIRS}
    )
    find_library(FFTW3_LIBRARIES NAMES fftw3 libfftw3-3
        HINTS ${CMAKE_PREFIX_PATH} ${PC_FFTW3_LIBRARY_DIRS}
    )
else()
    set(FFTW3_INCLUDE_DIRS ${PC_FFTW3_INCLUDE_DIRS})
    set(FFTW3_LIBRARIES    ${PC_FFTW3_LIBRARIES})
endif()

# Fix pkg-config returning "fftw3" (name only) instead of a real path
# This happens on macOS (Homebrew) and MSVC (vcpkg)
if(NOT EXISTS "${FFTW3_LIBRARIES}")
    message(STATUS "pkg-config returned '${FFTW3_LIBRARIES}', resolving actual file...")

    # Use a DISTINCT variable name to avoid shadowing the pkg-config local variable
    find_library(FFTW3_RESOLVED_PATH
        NAMES fftw3 libfftw3-3
        HINTS ${CMAKE_PREFIX_PATH} /opt/homebrew/lib /usr/local/lib
        PATH_SUFFIXES lib
    )

    if(FFTW3_RESOLVED_PATH)
        set(FFTW3_LIBRARIES "${FFTW3_RESOLVED_PATH}")
        message(STATUS "Resolved to '${FFTW3_LIBRARIES}'")
    else()
        message(FATAL_ERROR "[FAIL] Could not resolve absolute path for fftw3.")
    endif()
endif()

if(NOT FFTW3_LIBRARIES)
    message(FATAL_ERROR "[FAIL] FFTW3 not found.")
endif()

set(FFTW3_SOURCE
    $<IF:$<BOOL:${PC_FFTW3_FOUND}>,PkgConfig,Manual Search>
)

get_filename_component(FFTW3_LIBRARY_DIR "${FFTW3_LIBRARIES}" DIRECTORY)

print_lib_status("FFTW3" "${FFTW3_SOURCE}" "${FFTW3_LIBRARIES}" "${FFTW3_INCLUDE_DIRS}")

# ==============================================================================
# 3. HydraSDR
# ==============================================================================
message(STATUS ">> Searching for HydraSDR...")

set(HYDRASDR_FETCHED FALSE)
set(HYDRASDR_TARGET "")
set(HYDRASDR_DISPLAY_LIB "")
set(HYDRASDR_DISPLAY_INC "")

# A. Try installed SDK via find_package
find_package(HydraSDR QUIET NO_CMAKE_SYSTEM_PACKAGE_REGISTRY NO_CMAKE_PACKAGE_REGISTRY)

if(HydraSDR_FOUND)
    foreach(tgt HydraSDR::HydraSDR HydraSDR::hydrasdr hydrasdr)
        if(TARGET ${tgt})
            set(HYDRASDR_TARGET ${tgt})
            set(HYDRASDR_SOURCE "CMake Config (Target: ${tgt})")
            break()
        endif()
    endforeach()

    if(HYDRASDR_TARGET)
        get_target_property(HYDRASDR_DISPLAY_LIB ${HYDRASDR_TARGET} IMPORTED_LOCATION)
        if(NOT HYDRASDR_DISPLAY_LIB)
            get_target_property(HYDRASDR_DISPLAY_LIB ${HYDRASDR_TARGET} IMPORTED_LOCATION_RELEASE)
        endif()
        get_target_property(HYDRASDR_DISPLAY_INC ${HYDRASDR_TARGET} INTERFACE_INCLUDE_DIRECTORIES)
    else()
        message(WARNING "[!] HydraSDRConfig found but no usable target.")
        set(HydraSDR_FOUND FALSE)
    endif()
endif()

# B. FetchContent fallback -- clone hydrasdr-host and build only libhydrasdr/
if(NOT HydraSDR_FOUND)
    if(NOT HYDRASDR_FETCH_FROM_GIT)
        message(FATAL_ERROR
            "[FAIL] HydraSDR not found and HYDRASDR_FETCH_FROM_GIT is OFF.\n"
            "Install the SDK or set -DCMAKE_PREFIX_PATH=<sdk-root>."
        )
    endif()

    message(STATUS ">> HydraSDR not found -- fetching from ${HYDRASDR_FETCH_GIT_REPO} (${HYDRASDR_FETCH_GIT_TAG})")

    FetchContent_Declare(hydrasdr-host
        GIT_REPOSITORY ${HYDRASDR_FETCH_GIT_REPO}
        GIT_TAG        ${HYDRASDR_FETCH_GIT_TAG}
        GIT_SHALLOW    TRUE
    )

    FetchContent_GetProperties(hydrasdr-host)
    if(NOT hydrasdr-host_POPULATED)
        FetchContent_Populate(hydrasdr-host)
    endif()

    # MSVC needs pthreads4w (normally set up by hydrasdr-host top-level CMakeLists.txt)
    # With vcpkg: install pthreads via "vcpkg install pthreads:x64-windows"
    if(MSVC)
        add_compile_definitions(_TIMESPEC_DEFINED _CRT_SECURE_NO_WARNINGS)

        find_package(PThreads4W QUIET)
        if(TARGET PThreads4W::PThreads4W)
            # vcpkg-provided pthreads4w -- create alias expected by libhydrasdr
            add_library(pthreads4w_static ALIAS PThreads4W::PThreads4W)
            get_target_property(_ptw32_inc PThreads4W::PThreads4W
                INTERFACE_INCLUDE_DIRECTORIES)
            set(THREADS_PTHREADS_INCLUDE_DIR "${_ptw32_inc}"
                CACHE PATH "pthreads include directory" FORCE)
            message(STATUS ">> pthreads4w: using vcpkg PThreads4W::PThreads4W")
        else()
            message(FATAL_ERROR
                "[FAIL] PThreads4W not found. Install via: vcpkg install pthreads:x64-windows\n"
                "and pass -DCMAKE_TOOLCHAIN_FILE=<vcpkg>/scripts/buildsystems/vcpkg.cmake"
            )
        endif()
    endif()

    # For MSVC multi-config: set LIBUSB_DLL so libhydrasdr's post-build copy
    # doesn't fail with an empty argument, and redirect its output directory
    if(MSVC)
        # Find the libusb DLL for post-build copy (prefer release over debug)
        # vcpkg's find_file may resolve the debug variant; use EXISTS to bypass.
        get_filename_component(_libusb_inc_parent "${LIBUSB_INCLUDE_DIR}" DIRECTORY)
        get_filename_component(_libusb_inc_root "${_libusb_inc_parent}" DIRECTORY)
        set(_libusb_release_dll "${_libusb_inc_root}/bin/libusb-1.0.dll")
        if(EXISTS "${_libusb_release_dll}")
            set(LIBUSB_DLL "${_libusb_release_dll}" CACHE FILEPATH "libusb DLL" FORCE)
        else()
            get_filename_component(_libusb_lib_dir "${LIBUSB_LIBRARY}" DIRECTORY)
            get_filename_component(_libusb_prefix "${_libusb_lib_dir}" DIRECTORY)
            find_file(LIBUSB_DLL NAMES libusb-1.0.dll
                HINTS "${_libusb_prefix}/bin" "${_libusb_lib_dir}"
            )
            if(LIBUSB_DLL)
                set(LIBUSB_DLL "${LIBUSB_DLL}" CACHE FILEPATH "libusb DLL" FORCE)
            endif()
        endif()

        # Override CMAKE_RUNTIME_OUTPUT_DIRECTORY so libhydrasdr puts its
        # DLL next to kal.exe (also satisfies hydrasdr-tools copy targets)
        set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}" CACHE PATH "" FORCE)
        set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/Debug" CACHE PATH "" FORCE)
        set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/Release" CACHE PATH "" FORCE)
    endif()

    # Build only libhydrasdr/ (skip hydrasdr-tools which is always added by top-level)
    add_subdirectory(
        ${hydrasdr-host_SOURCE_DIR}/libhydrasdr
        ${hydrasdr-host_BINARY_DIR}/libhydrasdr
        EXCLUDE_FROM_ALL
    )

    set(HYDRASDR_TARGET hydrasdr)
    set(HYDRASDR_FETCHED TRUE)
    set(HYDRASDR_SOURCE "FetchContent (${HYDRASDR_FETCH_GIT_REPO} @ ${HYDRASDR_FETCH_GIT_TAG})")
    set(HYDRASDR_DISPLAY_LIB "<built from source>")
    set(HYDRASDR_DISPLAY_INC "${hydrasdr-host_SOURCE_DIR}/libhydrasdr")
endif()

print_lib_status("HydraSDR"
    "${HYDRASDR_SOURCE}"
    "${HYDRASDR_DISPLAY_LIB}"
    "${HYDRASDR_DISPLAY_INC}"
)


# ==============================================================================
# Build Target
# ==============================================================================
file(GLOB SOURCES "src/*.cc")
file(GLOB HEADERS "src/*.h")

add_executable(kal ${SOURCES} ${HEADERS})

target_include_directories(kal PRIVATE
    ${FFTW3_INCLUDE_DIRS}
)

set(KAL_LIBS
    ${HYDRASDR_TARGET}
    ${FFTW3_LIBRARIES}
)

# ==============================================================================
# Platform Specific Configuration
# ==============================================================================
if(WIN32 AND MINGW)
    target_compile_options(kal PRIVATE -O3)
    target_link_options(kal PRIVATE -static -static-libgcc -static-libstdc++)
elseif(WIN32 AND MSVC)
    target_compile_definitions(kal PRIVATE _WIN32 _CRT_SECURE_NO_WARNINGS _USE_MATH_DEFINES)
elseif(APPLE)
    target_compile_options(kal PRIVATE -O3)
elseif(UNIX)
    target_compile_options(kal PRIVATE -O3 -pthread)
    target_link_options(kal PRIVATE -pthread)
    list(APPEND KAL_LIBS m)
endif()

target_link_libraries(kal PRIVATE ${KAL_LIBS})


# ==============================================================================
# DLL Copy (Windows)
# ==============================================================================
if(WIN32)
    message(STATUS ">> Configuring DLL copy steps...")

    # Copy libhydrasdr DLL next to kal.exe
    if(HYDRASDR_FETCHED)
        add_custom_command(TARGET kal POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                $<TARGET_FILE:hydrasdr> $<TARGET_FILE_DIR:kal>
            COMMENT "Copying libhydrasdr.dll (FetchContent)"
        )
    elseif(TARGET ${HYDRASDR_TARGET})
        add_custom_command(TARGET kal POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                $<TARGET_FILE:${HYDRASDR_TARGET}> $<TARGET_FILE_DIR:kal>
            COMMENT "Copying libhydrasdr.dll (installed SDK)"
        )
    endif()

    # Copy libusb-1.0.dll — prefer release DLL
    # For MSVC, LIBUSB_DLL is already set by the FetchContent block above.
    # For MINGW, derive from include directory (always points to release prefix).
    if(NOT LIBUSB_DLL)
        get_filename_component(_lusb_inc_parent "${LIBUSB_INCLUDE_DIR}" DIRECTORY)
        get_filename_component(_lusb_inc_root "${_lusb_inc_parent}" DIRECTORY)
        set(_lusb_release_dll "${_lusb_inc_root}/bin/libusb-1.0.dll")
        if(EXISTS "${_lusb_release_dll}")
            set(LIBUSB_DLL "${_lusb_release_dll}")
        else()
            get_filename_component(_lusb_dir "${LIBUSB_LIBRARY}" DIRECTORY)
            get_filename_component(_lusb_prefix "${_lusb_dir}" DIRECTORY)
            set(_lusb_dll_hints "${_lusb_prefix}/bin" "${_lusb_dir}")
            if(DEFINED ENV{MSYSTEM_PREFIX})
                list(APPEND _lusb_dll_hints "$ENV{MSYSTEM_PREFIX}/bin")
            endif()
            unset(LIBUSB_DLL CACHE)
            find_file(LIBUSB_DLL NAMES libusb-1.0.dll
                HINTS ${_lusb_dll_hints}
            )
        endif()
    endif()
    if(LIBUSB_DLL)
        message(STATUS "   Will copy: ${LIBUSB_DLL}")
        add_custom_command(TARGET kal POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${LIBUSB_DLL}" $<TARGET_FILE_DIR:kal>
            COMMENT "Copying libusb-1.0.dll"
        )
    else()
        message(WARNING "   libusb-1.0.dll not found — kal.exe may fail at runtime")
    endif()

    # Copy pthreads DLL (MSVC with dynamic vcpkg triplet)
    # libhydrasdr already sets PTW32_STATIC_LIB and links pthreads4w_static.
    # If the vcpkg triplet is static, no DLL exists and this is a no-op.
    # For truly static pthreads: vcpkg install pthreads:x64-windows-static-md
    if(MSVC AND TARGET PThreads4W::PThreads4W)
        get_filename_component(_ptw32_prefix "${THREADS_PTHREADS_INCLUDE_DIR}" DIRECTORY)
        find_file(PTHREADS_DLL
            NAMES pthreadVC3.dll pthreadVC2.dll pthreadVCE3.dll
            HINTS "${_ptw32_prefix}/bin" "${_ptw32_prefix}/lib"
        )
        if(PTHREADS_DLL)
            message(STATUS "   Will copy: ${PTHREADS_DLL}")
            message(STATUS "   NOTE: To eliminate pthreads DLL dependency, use:")
            message(STATUS "         vcpkg install pthreads:x64-windows-static-md")
            add_custom_command(TARGET kal POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "${PTHREADS_DLL}" $<TARGET_FILE_DIR:kal>
                COMMENT "Copying pthreads DLL"
            )
        else()
            message(STATUS "   pthreads DLL not found (static linkage assumed)")
        endif()
    endif()

    # Copy FFTW3 DLL (MINGW only — vcpkg handles this for MSVC)
    if(NOT MSVC)
        get_filename_component(_fftw_dir "${FFTW3_LIBRARIES}" DIRECTORY)
        get_filename_component(_fftw_prefix "${_fftw_dir}" DIRECTORY)
        set(_fftw_dll_hints "${_fftw_prefix}/bin" "${_fftw_dir}")
        if(DEFINED ENV{MSYSTEM_PREFIX})
            list(APPEND _fftw_dll_hints "$ENV{MSYSTEM_PREFIX}/bin")
        endif()
        find_file(FFTW3_DLL NAMES libfftw3-3.dll fftw3.dll
            HINTS ${_fftw_dll_hints}
        )
        if(FFTW3_DLL)
            message(STATUS "   Will copy: ${FFTW3_DLL}")
            add_custom_command(TARGET kal POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "${FFTW3_DLL}" $<TARGET_FILE_DIR:kal>
                COMMENT "Copying FFTW3 DLL"
            )
        else()
            message(WARNING "   FFTW3 DLL not found — kal.exe may fail at runtime")
        endif()
    endif()
endif()


# ==============================================================================
# Installation
# ==============================================================================
include(GNUInstallDirs)
install(TARGETS kal RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

# EXCLUDE_FROM_ALL suppresses libhydrasdr's own install rules, so add ours
if(HYDRASDR_FETCHED)
    if(WIN32)
        install(FILES $<TARGET_FILE:hydrasdr> DESTINATION ${CMAKE_INSTALL_BINDIR})
    else()
        install(TARGETS hydrasdr LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
    endif()
endif()

# ==============================================================================
# Optional Test Suite
# ==============================================================================
if(IS_DIRECTORY "${CMAKE_SOURCE_DIR}/tests")
    enable_testing()
    add_subdirectory(tests)
endif()
