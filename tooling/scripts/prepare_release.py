#!/usr/bin/env python3
from typing import Optional, List, AnyStr, Iterable
from dataclasses import dataclass
from enum import Enum
import os
import re
import argparse
import subprocess
from datetime import datetime
import xml.etree.ElementTree as et
import git


# Types of packages we support
class PkgType(Enum):

    def __str__(self):
        return {
            PkgType.CPP: 'C++',
            PkgType.PYTHON_BINDINGS: 'Python',
            PkgType.ROS1: 'ROS1',
            PkgType.PYTHON: 'Python'
        }[self]

    CPP = 1
    PYTHON_BINDINGS = 2
    ROS1 = 3
    PYTHON = 4


# Class used to specify a package in our repository
@dataclass
class Pkg:
    name: AnyStr
    type: PkgType
    current_path: AnyStr
    old_paths: List[AnyStr]


# Parameters
pkgs = {
    'libraries': [
        Pkg('wavemap', PkgType.CPP, 'library/cpp', []),
        Pkg('pywavemap', PkgType.PYTHON_BINDINGS, 'library/python', [])
    ],
    'interfaces': [
        Pkg('wavemap', PkgType.ROS1, 'interfaces/ros1/wavemap', []),
        Pkg('wavemap_msgs', PkgType.ROS1, 'interfaces/ros1/wavemap_msgs', []),
        Pkg('wavemap_ros_conversions', PkgType.ROS1,
            'interfaces/ros1/wavemap_ros_conversions', []),
        Pkg('wavemap_ros', PkgType.ROS1, 'interfaces/ros1/wavemap_ros', []),
        Pkg('wavemap_rviz_plugin', PkgType.ROS1,
            'interfaces/ros1/wavemap_rviz_plugin', []),
        Pkg('wavemap_all', PkgType.ROS1, 'interfaces/ros1/wavemap_all', [])
    ],
    'helpers': [
        Pkg('catkin_setup', PkgType.ROS1, 'tooling/packages/catkin_setup', []),
    ],
    'examples': [
        Pkg('wavemap_examples_cpp', PkgType.CPP, 'examples/cpp', []),
        Pkg('wavemap_examples_ros1', PkgType.ROS1, 'examples/ros1', []),
        Pkg('wavemap_examples_python', PkgType.PYTHON, 'examples/python', [])
    ]
}


# Filter and convert tags to version numbers
def extract_version_numbers(version: str) -> List[int]:
    # Strip 'v' prefix if present
    version = version.lstrip('v')
    # Match the major, minor and patch number
    match = re.match(r'^(\d+)\.(\d+)\.(\d+)$', version)
    if match is None:
        raise ValueError(f'Invalid version string, '
                         f'must be int.int.int: "{version}"')
    # Cast all numbers to integers
    version = [int(x) for x in match.groups()]
    return version


# Increment a version number by a given bump level
def bump_version(version: str, level='patch') -> str:
    # Extract version number
    bumped_version = extract_version_numbers(version)
    # Increment the appropriate number
    bump_idx = {'major': 0, 'minor': 1, 'patch': 2}[level]
    bumped_version[bump_idx] += 1
    # Reset the trailing numbers
    bumped_version = [
        x if idx <= bump_idx else 0 for idx, x in enumerate(bumped_version)
    ]
    return '.'.join([str(x) for x in bumped_version])


# Find the tag with the highest version number in a list of git tags
def find_highest_version(tag_list) -> Optional[List[int]]:
    # Filter out tags that don't have valid versions
    versioned_tags = []
    for tag in tag_list:
        try:
            version = extract_version_numbers(tag.name)
            versioned_tags.append(version)
        except ValueError:
            pass
    # Sort tags by their version numbers
    versioned_tags.sort(reverse=True)
    # Return the highest tag or None
    return versioned_tags[0] if versioned_tags else None


def version_from_cmake_project(project_root_path: str) -> Optional[str]:
    cmakelists_path = os.path.join(project_root_path, 'CMakeLists.txt')
    if os.path.exists(cmakelists_path):
        # Read the existing content of the CMakeLists.txt file
        with open(cmakelists_path, 'r', encoding='utf-8') as file:
            cmake_content = file.read()

        # Replace the old version number with the new version number
        pattern = re.compile(
            r'(project\(wavemap\s+VERSION)(\s+\d+\.\d+\.\d+\s+)')
        matches = pattern.findall(cmake_content)

        # Make the replacement was successful and unique
        if len(matches) == 1 and len(matches[0]) == 2:
            return matches[0][1].strip()

        print(f'Failed to find version number in {cmakelists_path}')
    else:
        print(f'Failed to find {cmakelists_path}')

    return None


def version_from_git_tags(
        tag_list: Iterable[git.TagReference]) -> Optional[str]:
    most_recent_version = find_highest_version(tag_list)
    if most_recent_version and isinstance(most_recent_version, list):
        return '.'.join(map(str, most_recent_version))
    return None


def update_release_files(previous_version: str, new_version: str) -> None:
    # Generate the changelogs for each package in our repo
    packages = [pk for key, value in pkgs.items() for pk in value]
    for pkg in packages:
        # Package variables
        pkg_debug_name = f'{pkg.type} package {pkg.name}'
        pkg_all_paths = pkg.old_paths + [pkg.current_path]
        print(f'Processing {pkg_debug_name}')

        pkg_changelog_path = os.path.join(pkg.current_path, 'CHANGELOG.rst')
        if os.path.exists(pkg_changelog_path):
            # Read the package's changelog
            with open(pkg_changelog_path, 'r') as f:
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
            date_str = datetime.now().strftime('%Y-%m-%d')
            section_title = f'{new_version} ({date_str})'
            section_title_underline = '-' * len(section_title)

            # Generate an overview of the current changes and contributors
            pkg_commits = repo.iter_commits(rev=f'v{previous_version}..HEAD',
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
            if len(commit_msgs) == 0:
                changelog.insert(6, os.linesep)
            else:
                changelog.insert(6, section_changelog + os.linesep)
                changelog.insert(7, section_contributors + 2 * os.linesep)

            # Write the updated content back to the file
            with open(pkg_changelog_path, 'w') as f:
                changelog = ''.join(changelog)
                f.write(changelog)

        else:
            print(f'Could NOT find changelog for {pkg_debug_name}')
            raise SystemExit

        if pkg.type in (PkgType.CPP, PkgType.PYTHON_BINDINGS):
            pkg_cmake_path = os.path.join(pkg.current_path, 'CMakeLists.txt')
            if os.path.exists(pkg_cmake_path):
                # Read the existing content of the CMakeLists.txt file
                with open(pkg_cmake_path, 'r', encoding='utf-8') as file:
                    cmake_content = file.read()

                # Replace the old version number with the new version number
                pattern = re.compile(r'(project\(' + pkg.name +
                                     r'\s+VERSION)(\s+\d+\.\d+\.\d+\s+)')
                substitution = r'\1 ' + new_version + r' '
                new_content, count = pattern.subn(substitution, cmake_content)

                # Make the replacement was successful and unique
                if count == 0 or 1 < count:
                    print('Failed to update version number in '
                          f'{pkg_cmake_path}')
                    raise SystemExit

                # Write the updated content back to the file
                with open(pkg_cmake_path, 'w', encoding='utf-8') as file:
                    file.write(new_content)

            else:
                print(f'Could NOT find CMakeLists.txt for {pkg_debug_name}')
                raise SystemExit

        if pkg.type == PkgType.PYTHON_BINDINGS:
            pyproject_toml_path = os.path.join(pkg.current_path,
                                               'pyproject.toml')
            if os.path.exists(pyproject_toml_path):
                # Read the existing content of the CMakeLists.txt file
                with open(pyproject_toml_path, 'r', encoding='utf-8') as file:
                    cmake_content = file.read()

                # Replace the old version number with the new version number
                pattern = re.compile(
                    r'(version\s+=\s+)(\"\d+\.\d+\.\d+\")(.*)')
                substitution = r'\1"' + new_version + r'"\3'
                new_content, count = pattern.subn(substitution, cmake_content)

                # Make the replacement was successful and unique
                if count == 0 or 1 < count:
                    print('Failed to update version number in '
                          f'{pyproject_toml_path}')
                    raise SystemExit

                # Write the updated content back to the file
                with open(pyproject_toml_path, 'w', encoding='utf-8') as file:
                    file.write(new_content)

            else:
                print(f'Could NOT find CMakeLists.txt for {pkg_debug_name}')
                raise SystemExit

        if pkg.type == PkgType.ROS1:
            pkg_xml_path = os.path.join(pkg.current_path, 'package.xml')
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
                version_tag.text = new_version
                tree.write(pkg_xml_path,
                           encoding='utf-8',
                           xml_declaration=True)
            else:
                print(f'Could NOT find package.xml for {pkg_debug_name}')
                raise SystemExit

    print("\nRunning pre-commit to ensure consistent formatting:")
    subprocess.run(['pre-commit', 'run', '--all-files'])


def create_release_tag(version: str) -> None:
    tag = f'v{version}'
    message = f'Release v{version}'
    repo.create_tag(tag, message=message)


def draft_release_notes(version: str) -> None:
    out = '# Summary\n'
    out += '...\n'
    out += '\n'
    out += '### Detailed description\n'
    out += '...\n'
    out += '\n'
    out += '# Package changelogs\n'
    out += '\n'

    out += '### Libraries\n'
    for pkg in pkgs['libraries']:
        out += f'* [{pkg.type}](https://github.com/ethz-asl/wavemap/blob/'
        out += f'v{version}/{pkg.current_path}/CHANGELOG.rst)\n'
    out += '\n'

    out += '### Interfaces\n'
    out += '* ROS1\n'
    for pkg in pkgs['interfaces']:
        out += f'  * [{pkg.name}](https://github.com/ethz-asl/wavemap/blob/'
        out += f'v{version}/{pkg.current_path}/CHANGELOG.rst)\n'
    out += '\n'

    out += '### Examples\n'
    for pkg in pkgs['examples']:
        out += f'* [{pkg.type}](https://github.com/ethz-asl/wavemap/blob/'
        out += f'v{version}/{pkg.current_path}/CHANGELOG.rst)\n'
    out += '\n'

    out += '# Upgrade notes\n'
    out += 'Upgrade instructions for\n'
    out += '* C++ Library\n'
    out += '  * To use wavemap as a standalone CMake project, please see '
    out += '[these instructions]'
    out += '(https://ethz-asl.github.io/wavemap/pages/installation/cpp)\n'
    out += '* Python Library\n'
    out += '  * To install wavemap\'s Python API, please see '
    out += '[these instructions]'
    out += '(https://ethz-asl.github.io/wavemap/pages/installation/python)\n'
    out += '* ROS1\n'
    out += '  * Catkin\n'
    out += '    * Go to your catkin workspace src directory: '
    out += '`cd ~/catkin_ws/src`\n'
    out += '    * Pull the newest wavemap code:'
    out += '`cd wavemap && git checkout main && git pull`\n'
    out += '    * Rebuild wavemap: `catkin build wavemap_all`\n'
    out += '  * Docker\n'
    out += '    * `docker build --tag=wavemap_ros1 '
    out += f'--build-arg=\'VERSION=v{version}\' -<<< $(curl -s https://'
    out += f'raw.githubusercontent.com/ethz-asl/wavemap/v{version}'
    out += '/tooling/docker/ros1/incremental.Dockerfile)`\n\n'
    out += 'For more info, see our guides on [installing wavemap](https://'
    out += 'ethz-asl.github.io/wavemap/pages/installation).'
    out += '\n'

    print(out)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        prog='PrepareRelease',
        description='Prepares wavemap\'s release files.')
    parser.add_argument('--update-release-files',
                        action='store_true',
                        default=False)
    parser.add_argument('--create-release-tag',
                        action='store_true',
                        default=False)
    parser.add_argument('--draft-release-notes',
                        action='store_true',
                        default=False)
    parser.add_argument('--level', required=False)
    parser.add_argument('--version', required=False)
    args = parser.parse_args()

    # Load our git repo
    repo = git.Repo(os.path.abspath(__file__), search_parent_directories=True)

    # Run the rest of the script from the repo's root
    os.chdir(repo.git.rev_parse('--show-toplevel'))

    # Get the current versions
    wavemap_cpp_root = os.path.join('library', 'cpp')
    last_released_version = version_from_git_tags(repo.tags)
    current_file_version = version_from_cmake_project(wavemap_cpp_root)

    # Determine the new release number
    next_version = None
    if args.version and args.level:
        print('Specify either the bump level or the new version, not both.')
        raise SystemExit
    if args.version:
        next_version = args.version
    elif args.level:
        next_version = bump_version(last_released_version, args.level)

    if args.update_release_files:
        if next_version is None:
            print('Specify either a --level=(major|minor|patch),'
                  ' or a specific --version=X.Y.Z.')
            raise SystemExit
        update_release_files(last_released_version, next_version)
    elif args.create_release_tag:
        # print(current_file_version)
        if current_file_version == last_released_version:
            print(f'Current project file version ({current_file_version}) '
                  'already has a corresponding git release tag '
                  f'(v{last_released_version}).')
            raise SystemExit
        create_release_tag(current_file_version)
    elif args.draft_release_notes:
        draft_release_notes(current_file_version)
    else:
        print('No release action specified. Please choose '
              '--update-release-files, --create-release-tag, '
              'or --draft-release-notes.')
