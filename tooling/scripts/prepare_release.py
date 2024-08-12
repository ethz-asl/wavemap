#!/usr/bin/env python3

from dataclasses import dataclass
from enum import Enum
import os
import re
import xml.etree.ElementTree as et
from datetime import datetime
import git


# Filter and convert tags to version numbers
def extract_version(tag_name):
    # Strip 'v' prefix if present
    version_str = tag_name.lstrip('v')
    # Match the major, minor and patch number
    match = re.match(r'^(\d+)\.(\d+)\.(\d+)$', version_str)
    if match is None:
        raise ValueError(f'Invalid version string, '
                         f'must be int.int.int: "{version_str}"')
    # Cast all numbers to integers
    version = [int(x) for x in match.groups()]
    return version


# Find the tag with the highest version number in a list of git tags
def extract_highest_version(tag_list):
    # Filter out tags that don't have valid versions
    versioned_tags = []
    for tag in tag_list:
        try:
            version = extract_version(tag.name)
            versioned_tags.append((tag, version))
        except ValueError:
            pass
    # Sort tags by their version numbers
    versioned_tags.sort(key=lambda x: x[1], reverse=True)
    # Return the highest tag or None
    return versioned_tags[0][0] if versioned_tags else None


# Increment a version number by a given bump level
def bump_version(version, level='patch'):
    # Extract version number
    bumped_version = extract_version(version)
    # Increment appropriate part
    bump_idx = {'major': 0, 'minor': 1, 'patch': 2}[level]
    bumped_version[bump_idx] += 1
    # Reset trailing parts
    bumped_version = [
        x if idx <= bump_idx else 0 for idx, x in enumerate(bumped_version)
    ]
    return bumped_version


# Types of packages we support
class PkgType(Enum):
    CMake = 1
    ROS1 = 2


# Class used to specify a package in our repository
@dataclass
class Pkg:
    name: str
    type: PkgType


# Determine the relative path to a package given its name and type
def determine_package_path(repository_path, package):
    if package.type is PkgType.CMake:
        sub_paths = ['library/cpp', 'library', 'libraries']
        return [
            os.path.join(repository_path, sub_path) for sub_path in sub_paths
        ]
    if package.type is PkgType.ROS1:
        sub_paths = ['interfaces/ros1', 'ros']
        return [
            os.path.join(repository_path, sub_path, pkg.name)
            for sub_path in sub_paths
        ]

    raise SystemExit


# Parameters
bump_level = 'major'
repo_path = '/home/victor/catkin_ws/src/wavemap'
pkgs = [
    Pkg('wavemap', PkgType.CMake),
    Pkg('wavemap', PkgType.ROS1),
    Pkg('wavemap_msgs', PkgType.ROS1),
    Pkg('wavemap_ros_conversions', PkgType.ROS1),
    Pkg('wavemap_ros', PkgType.ROS1),
    Pkg('wavemap_rviz_plugin', PkgType.ROS1),
    Pkg('wavemap_all', PkgType.ROS1)
]

# Find the most recent release
repo = git.Repo(repo_path)
tags = repo.tags
most_recent_release = extract_highest_version(tags)
if most_recent_release is None:
    raise SystemExit
most_recent_release = most_recent_release.name

# Determine the new release number
new_version = bump_version(most_recent_release, bump_level)
new_version_str = '.'.join([str(x) for x in new_version])

# Generate the changelogs for each package in our repo
for pkg in pkgs:
    # Package variables
    pkg_debug_name = f'{pkg.type.name} package {pkg.name}'
    pkg_all_paths = determine_package_path(repo_path, pkg)
    pkg_current_path = pkg_all_paths[0]
    print(f'Processing {pkg_debug_name}')

    pkg_changelog_path = os.path.join(pkg_current_path, "CHANGELOG.rst")
    if os.path.exists(pkg_changelog_path):
        # Read the package's changelog
        with open(pkg_changelog_path, "r") as f:
            changelog = f.readlines()

        # Check the title to ensure we're editing the right file
        changelog_title = changelog[1][:-1]
        changelog_title_expected = f'Changelog for package {pkg.name}'
        if changelog_title != changelog_title_expected:
            print(
                f'Changelog title for {pkg_debug_name} is "{changelog_title}"'
                f' does not match expected "{changelog_title_expected}"')
            raise SystemExit

        #  Generate the title for the new section
        date_str = datetime.now().strftime("%Y-%m-%d")
        section_title = f'{new_version_str} ({date_str})'
        section_title_underline = "-" * len(section_title)

        # Generate an overview of the current changes
        pkg_commits = repo.iter_commits(rev=f'{most_recent_release}..HEAD',
                                        paths=pkg_all_paths)
        commit_msgs = []
        for commit in pkg_commits:
            commit_msg = commit.message.partition(os.linesep)[0].strip()
            commit_msgs.append(f'* {commit_msg}')
        section_content = os.linesep.join(commit_msgs)

        # Append the new section, below the changelog title
        changelog.insert(4, section_title + os.linesep)
        changelog.insert(5, section_title_underline + os.linesep)
        changelog.insert(6, section_content + 2 * os.linesep)

        # Write the updated content back to the file
        with open(pkg_changelog_path, "w") as f:
            changelog = "".join(changelog)
            f.write(changelog)

    else:
        print(f'Could NOT find changelog for {pkg_debug_name}')
        raise SystemExit

    if pkg.type == PkgType.CMake:
        pkg_cmake_path = os.path.join(pkg_current_path, "CMakeLists.txt")
        if os.path.exists(pkg_cmake_path):
            # Read the existing content of the CMakeLists.txt file
            with open(pkg_cmake_path, 'r', encoding='utf-8') as file:
                cmake_content = file.read()

            # Replace the old version number with the new version number
            pattern = re.compile(r'(project\(' + pkg.name +
                                 r'\s+VERSION)(\s+\d+\.\d+\.\d+\s+)')
            substitution = r'\1 ' + new_version_str + r' '
            new_content, count = pattern.subn(substitution, cmake_content)

            # Make the replacement was successful and unique
            if count == 0:
                raise SystemExit
            if 1 < count:
                raise SystemExit

            # Write the updated content back to the file
            with open(pkg_cmake_path, 'w', encoding='utf-8') as file:
                file.write(new_content)

        else:
            print(f'Could NOT find CMakeLists.txt for {pkg_debug_name}')
            raise SystemExit

    if pkg.type == PkgType.ROS1:
        pkg_xml_path = os.path.join(pkg_current_path, "package.xml")
        if os.path.exists(pkg_xml_path):
            # Parse the XML file
            tree = et.parse(pkg_xml_path)
            root = tree.getroot()

            # Find all 'version' elements
            version_tags = root.findall('version')

            # Make sure the tag is unique
            if len(version_tags) == 0:
                raise SystemExit
            if 1 < len(version_tags):
                raise SystemExit
            version_tag = version_tags[0]

            # Update the version tag and save the changes
            version_tag.text = new_version_str
            tree.write(pkg_xml_path, encoding='utf-8', xml_declaration=True)
        else:
            print(f'Could NOT find package.xml for {pkg_debug_name}')
            raise SystemExit
