import argparse
import os
import re
import sys
import subprocess

from shutil import which
from catkin_pkg import metapackage
from catkin_pkg.changelog import CHANGELOG_FILENAME, get_changelog_from_path
from catkin_pkg.package import InvalidPackage, PACKAGE_MANIFEST_FILENAME
from catkin_pkg.package_version import bump_version
from catkin_pkg.package_version import (get_forthcoming_label,
                                        update_changelog_sections,
                                        update_versions)
from catkin_pkg.packages import find_packages, verify_equal_package_versions
from catkin_pkg.terminal_color import disable_ANSI_colors, fmt
from catkin_pkg.workspace_vcs import get_repository_type
from catkin_pkg.cli.prepare_release import (prompt_continue, has_changes,
                                            check_clean_working_copy)


# pylint: disable=consider-using-f-string
def _find_executable(vcs_type):
    file_path = which(vcs_type)
    if file_path is None:
        raise RuntimeError(fmt('@{rf}Could not find vcs binary: %s' %
                               vcs_type))
    return file_path


# pylint: disable=R0913
def commit_files(base_path,
                 vcs_type,
                 packages,
                 packages_with_changelogs,
                 message,
                 dry_run=False):
    cmd = [_find_executable(vcs_type), 'commit', '-m', message]
    cmd += [
        os.path.join(p, PACKAGE_MANIFEST_FILENAME) for p in packages.keys()
    ]
    cmd += [
        s for s in [os.path.join(p, 'setup.py') for p in packages.keys()]
        if os.path.exists(s)
    ]
    cmd += [path for path, _, _ in packages_with_changelogs.values()]
    if not dry_run:
        try:
            subprocess.check_call(cmd, cwd=base_path)
        except subprocess.CalledProcessError as e:
            raise RuntimeError(
                fmt('@{rf}Failed to commit package.xml files: %s' %
                    str(e))) from e
    return cmd


def main():
    print("Starting")
    try:
        _main()
    except RuntimeError as e:
        print(e, file=sys.stderr)
        sys.exit(1)


def _main():
    parser = argparse.ArgumentParser(
        description='Runs the commands to bump the version number '
        'and commit the modified %s files.' % PACKAGE_MANIFEST_FILENAME)
    parser.add_argument(
        '--bump',
        choices=('major', 'minor', 'patch'),
        default='patch',
        help='Which part of the version number to bump? (default: %(default)s)'
    )
    parser.add_argument('--version', help='Specify a specific version to use')
    parser.add_argument('--no-color',
                        action='store_true',
                        default=False,
                        help='Disables colored output')
    parser.add_argument(
        '-y',
        '--non-interactive',
        action='store_true',
        default=False,
        help="Run without user interaction, confirming all questions with 'yes'"
    )
    args = parser.parse_args()

    if not args.non_interactive:
        raise RuntimeError(
            fmt('@{rf}This script should only be used in non-interactive '
                'mode in CI. For all normal purposes, use catkin\'s original '
                '\"catkin_prepare_release\" script.'))

    if args.version and not re.match(
            r'^(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)$',
            args.version):
        parser.error('The passed version must follow the conventions '
                     '(positive integers x.y.z with no leading zeros)')

    # force --no-color if stdout is non-interactive
    if not sys.stdout.isatty():
        args.no_color = True
    # disable colors if asked
    if args.no_color:
        disable_ANSI_colors()

    base_path = '.'

    print(fmt('@{gf}Prepare the source repository for a release.'))

    # determine repository type
    vcs_type = get_repository_type(base_path)
    if vcs_type is None:
        raise RuntimeError(
            fmt("@{rf}Could not determine repository type "
                "of @{boldon}'%s'@{boldoff}" % base_path))
    print(fmt('Repository type: @{boldon}%s@{boldoff}' % vcs_type))

    # find packages
    try:
        packages = find_packages(base_path)
    except InvalidPackage as e:
        raise RuntimeError(
            fmt("@{rf}Invalid package at path @{boldon}'%s'@{boldoff}:\n  %s" %
                (os.path.abspath(base_path), str(e)))) from e
    if not packages:
        raise RuntimeError(fmt('@{rf}No packages found'))
    print('Found packages: %s' % ', '.join([
        fmt('@{bf}@{boldon}%s@{boldoff}@{reset}' % p.name)
        for p in packages.values()
    ]))

    # complain about packages with unsupported build_type as they might require
    # additional steps before being released
    # complain about packages with upper case character since they
    # won't be releasable with bloom
    unsupported_pkg_names = []
    invalid_pkg_names = []
    valid_build_types = ['catkin', 'ament_cmake', 'ament_python']
    for package in packages.values():
        build_types = package.get_unconditional_build_types()
        if any(build_type not in valid_build_types
               for build_type in build_types):
            unsupported_pkg_names.append(package.name)
        if package.name != package.name.lower():
            invalid_pkg_names.append(package.name)
    if unsupported_pkg_names:
        print(
            fmt("@{yf}Warning: the following package are not of build_type %s "
                "and may require manual steps to release': %s" %
                (str(valid_build_types), ', '.join(
                    [('@{boldon}%s@{boldoff}' % p)
                     for p in sorted(unsupported_pkg_names)]))),
            file=sys.stderr)
        if not args.non_interactive and not prompt_continue('Continue anyway',
                                                            default=False):
            raise RuntimeError(
                fmt('@{rf}Aborted release, verify that unsupported packages '
                    'are ready to be released or release manually.'))
    if invalid_pkg_names:
        print(
            fmt("@{yf}Warning: the following package names contain upper case "
                "characters which violate both ROS and Debian "
                "naming conventions': %s" %
                ', '.join([('@{boldon}%s@{boldoff}' % p)
                           for p in sorted(invalid_pkg_names)])),
            file=sys.stderr)
        if not args.non_interactive and not prompt_continue('Continue anyway',
                                                            default=False):
            raise RuntimeError(
                fmt('@{rf}Aborted release, fix the names of the packages.'))

    local_modifications = []
    for pkg_path, package in packages.items():
        # verify that the package.xml files don't have modifications pending
        package_xml_path = os.path.join(pkg_path, PACKAGE_MANIFEST_FILENAME)
        if has_changes(base_path, package_xml_path, vcs_type):
            local_modifications.append(package_xml_path)
        # verify that metapackages are valid
        if package.is_metapackage():
            try:
                metapackage.validate_metapackage(pkg_path, package)
            except metapackage.InvalidMetapackage as e:
                raise RuntimeError(
                    fmt("@{rf}Invalid metapackage at path "
                        "'@{boldon}%s@{boldoff}':\n  %s\n\n"
                        "See requirements for metapackages: %s" %
                        (os.path.abspath(pkg_path), str(e),
                         metapackage.DEFINITION_URL))) from e
        # verify that the setup.py files don't have modifications pending
        setup_py_path = os.path.join(pkg_path, 'setup.py')
        if os.path.exists(setup_py_path) and has_changes(
                base_path, setup_py_path, vcs_type):
            local_modifications.append(setup_py_path)

    # fetch current version and verify that all packages have same version num
    old_version = verify_equal_package_versions(packages.values())
    if args.version:
        new_version = args.version
    else:
        new_version = bump_version(old_version, args.bump)

    if (not args.non_interactive and not prompt_continue(fmt(
            "Prepare release of version '@{bf}@{boldon}%s@{boldoff}@{reset}'%s"
            % (new_version, '')),
                                                         default=True)):
        raise RuntimeError(
            fmt("@{rf}Aborted release, use option '--bump' to release "
                "a different version."))

    # check for changelog entries
    missing_changelogs = []
    missing_changelogs_but_forthcoming = {}
    for pkg_path, package in packages.items():
        changelog_path = os.path.join(pkg_path, CHANGELOG_FILENAME)
        if not os.path.exists(changelog_path):
            missing_changelogs.append(package.name)
            continue
        # verify that the changelog files don't have modifications pending
        if has_changes(base_path, changelog_path, vcs_type):
            local_modifications.append(changelog_path)
        changelog = get_changelog_from_path(changelog_path, package.name)
        try:
            changelog.get_content_of_version(new_version)
        except KeyError:
            # check that forthcoming section exists
            forthcoming_label = get_forthcoming_label(changelog.rst)
            if forthcoming_label:
                missing_changelogs_but_forthcoming[package.name] = (
                    changelog_path, changelog, forthcoming_label)
            else:
                missing_changelogs.append(package.name)

    if local_modifications:
        raise RuntimeError(
            fmt('@{rf}The following files have modifications, '
                'please commit/revert them before:' +
                ''.join([('\n- @{boldon}%s@{boldoff}' % path)
                         for path in local_modifications])))

    if missing_changelogs:
        print(fmt(
            "@{yf}Warning: the following packages do not have a changelog file "
            "or entry for version '@{boldon}%s@{boldoff}': %s" %
            (new_version, ', '.join([('@{boldon}%s@{boldoff}' % p)
                                     for p in sorted(missing_changelogs)]))),
              file=sys.stderr)
        raise RuntimeError(
            fmt("@{rf}Aborted release, populate the changelog "
                "with '@{boldon}catkin_generate_changelog@{boldoff}' "
                "and review / clean up the content."))

    # check for staged changes and modified and untracked files
    print(
        fmt('@{gf}Checking if working copy is clean '
            '(no staged changes, no modified files, no untracked files)...'))
    is_clean = check_clean_working_copy(base_path, vcs_type)
    if not is_clean:
        print(fmt(
            '@{yf}Warning: the working copy contains other changes. Consider '
            'reverting/committing/stashing them before preparing a release.'),
              file=sys.stderr)
        if not args.non_interactive and not prompt_continue('Continue anyway',
                                                            default=False):
            raise RuntimeError(
                fmt('@{rf}Aborted release, clean the working copy '
                    'before trying again.'))

    # tag forthcoming changelog sections
    update_changelog_sections(missing_changelogs_but_forthcoming, new_version)
    print(
        fmt("@{gf}Rename the forthcoming section@{reset} of the following "
            "packages to version '@{bf}@{boldon}%s@{boldoff}@{reset}': %s" %
            (new_version, ', '.join([
                ('@{boldon}%s@{boldoff}' % p)
                for p in sorted(missing_changelogs_but_forthcoming.keys())
            ]))))

    # bump version number
    update_versions(packages, new_version)
    print(
        fmt("@{gf}Bump version@{reset} of all packages from '@{bf}%s@{reset}' "
            "to '@{bf}@{boldon}%s@{boldoff}@{reset}'" %
            (old_version, new_version)))

    # for other vcs types the changes are first done locally
    print(fmt('@{gf}Committing the package.xml files...'))
    commit_files(base_path, vcs_type, packages,
                 missing_changelogs_but_forthcoming,
                 f"Prepare release {new_version}")


if __name__ == '__main__':
    main()
