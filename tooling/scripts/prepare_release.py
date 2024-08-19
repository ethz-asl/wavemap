#!/usr/bin/env python3

from dataclasses import dataclass
from enum import Enum
from typing import List, AnyStr
import os
import re
import argparse
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

    def __str__(self):
        return {PkgType.CPP: "C++", PkgType.ROS1: "ROS1"}[self]

    CPP = 1
    ROS1 = 2


# Class used to specify a package in our repository
@dataclass
class Pkg:
    name: AnyStr
    type: PkgType
    current_path: AnyStr
    old_paths: List[AnyStr]


def draft_release_notes():
    out = "# Summary\n"
    out += "...\n"
    out += "\n"
    out += "### Detailed description\n"
    out += "...\n"
    out += "\n"
    out += "# Package changelogs\n"
    out += "\n"

    out += "### Libraries\n"
    pkg = pkgs["libraries"][0]
    out += f"* [{pkg.type}](https://github.com/ethz-asl/wavemap/blob/"
    out += "v{new_version_str}/{pkg.current_path}/CHANGELOG.rst)\n"
    out += "\n"

    out += "### Interfaces\n"
    out += "* ROS1\n"
    for pkg in pkgs["interfaces"]:
        out += f"  * [{pkg.name}](https://github.com/ethz-asl/wavemap/blob/"
        out += f"v{new_version_str}/{pkg.current_path}/CHANGELOG.rst)\n"
    out += "\n"

    out += "### Examples\n"
    for pkg in pkgs["examples"]:
        out += f"* [{pkg.type}](https://github.com/ethz-asl/wavemap/blob/"
        out += f"v{new_version_str}/{pkg.current_path}/CHANGELOG.rst)\n"
    out += "\n"

    out += "# Upgrade notes\n"
    out += "Upgrade instructions for\n"
    out += "* Catkin\n"
    out += "  * Go to your catkin workspace src directory: "
    out += "`cd ~/catkin_ws/src`\n"
    out += "  * Pull the newest wavemap code:"
    out += "`cd wavemap && git checkout main && git pull`\n"
    out += "  * Rebuild wavemap: `catkin build wavemap_all`\n"
    out += "* Docker\n"
    out += "  * `docker build --tag=wavemap_ros1 "
    out += f"--build-arg=\"VERSION=v{new_version_str}\" -"
    out += "<<< $(curl -s https://raw.githubusercontent.com/ethz-asl/wavemap/"
    out += "main/tooling/docker/ros1/incremental.Dockerfile)`\n"
    out += "For more info, see the [installation page](https://"
    out += "ethz-asl.github.io/wavemap/pages/installation) in the docs.)"
    out += "\n"

    print(out)


def prepare_release_files():
    # Generate the changelogs for each package in our repo
    packages = [pk for key, value in pkgs.items() for pk in value]
    for pkg in packages:
        # Package variables
        pkg_debug_name = f'{pkg.type} package {pkg.name}'
        pkg_all_paths = pkg.old_paths.append(pkg.current_path)
        print(f'Processing {pkg_debug_name}')

        pkg_changelog_path = os.path.join(pkg.current_path, "CHANGELOG.rst")
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

            # Generate an overview of the current changes and contributors
            pkg_commits = repo.iter_commits(rev=f'{most_recent_release}..HEAD',
                                            paths=pkg_all_paths)
            commit_msgs = []
            authors = set()
            for commit in pkg_commits:
                commit_msg = commit.message.partition(os.linesep)[0].strip()
                commit_msgs.append(f'* {commit_msg}')
                author = commit.author.name
                authors.add(author)
            section_changelog = os.linesep.join(commit_msgs)
            section_contributors = f'* Contributors: {", ".join(authors)}'

            # Append the new section, below the changelog title
            changelog.insert(4, section_title + os.linesep)
            changelog.insert(5, section_title_underline + os.linesep)
            changelog.insert(6, section_changelog + os.linesep)
            changelog.insert(7, section_contributors + 2 * os.linesep)

            # Write the updated content back to the file
            with open(pkg_changelog_path, "w") as f:
                changelog = "".join(changelog)
                f.write(changelog)

        else:
            print(f'Could NOT find changelog for {pkg_debug_name}')
            raise SystemExit

        if pkg.type == PkgType.CPP:
            pkg_cmake_path = os.path.join(pkg.current_path, "CMakeLists.txt")
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
            pkg_xml_path = os.path.join(pkg.current_path, "package.xml")
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
                tree.write(pkg_xml_path,
                           encoding='utf-8',
                           xml_declaration=True)
            else:
                print(f'Could NOT find package.xml for {pkg_debug_name}')
                raise SystemExit


# Parameters
pkgs = {
    "libraries": [Pkg('wavemap', PkgType.CPP, 'library/cpp', [])],
    "interfaces": [
        Pkg('wavemap', PkgType.ROS1, 'interfaces/ros1/wavemap', []),
        Pkg('wavemap_msgs', PkgType.ROS1, 'interfaces/ros1/wavemap_msgs', []),
        Pkg('wavemap_ros_conversions', PkgType.ROS1,
            'interfaces/ros1/wavemap_ros_conversions', []),
        Pkg('wavemap_ros', PkgType.ROS1, 'interfaces/ros1/wavemap_ros', []),
        Pkg('wavemap_rviz_plugin', PkgType.ROS1,
            'interfaces/ros1/wavemap_rviz_plugin', []),
        Pkg('wavemap_all', PkgType.ROS1, 'interfaces/ros1/wavemap_all', [])
    ],
    "helpers": [
        Pkg('catkin_setup', PkgType.ROS1, 'tooling/packages/catkin_setup', []),
    ],
    "examples": [
        Pkg('wavemap_examples_cpp', PkgType.CPP, 'examples/cpp', []),
        Pkg('wavemap_examples_ros1', PkgType.ROS1, 'examples/ros1', [])
    ]
}

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        prog='PrepareRelease',
        description='Prepares wavemap\'s release files.')
    parser.add_argument('--bump_level', required=False)
    parser.add_argument('--new_version', required=False)
    parser.add_argument('--draft_release_notes',
                        action='store_true',
                        default=False)
    args = parser.parse_args()

    if (args.bump_level == '') != (args.new_version == ''):
        print("Specify either the bump level or new version number, not both.")
        raise SystemExit

    # Load our git repo
    repo = git.Repo(os.path.abspath(__file__), search_parent_directories=True)

    # Run the rest of the script from the repo's root
    os.chdir(repo.git.rev_parse("--show-toplevel"))

    # Find the most recent release
    tags = repo.tags
    most_recent_release = extract_highest_version(tags)
    if most_recent_release is None:
        raise SystemExit
    most_recent_release = most_recent_release.name

    # Determine the new release number
    if args.new_version:
        new_version_str = args.new_version
    else:
        new_version = bump_version(most_recent_release, args.bump_level)
        new_version_str = '.'.join([str(x) for x in new_version])

    if args.draft_release_notes:
        draft_release_notes()
    else:
        prepare_release_files()
