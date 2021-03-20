# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from bloom.generators import resolve_dependencies
from bloom.generators.common import evaluate_package_conditions
from dataclasses import dataclass
from os import path
from os import devnull
import re
import subprocess

try:
    from catkin_pkg.packages import find_packages
except ImportError:
    print("catkin_pkg was not detected, please install it.")


@dataclass
class ProcessedPackage:
    name: str = ""
    version: str = ""
    licenses: list = ()
    depends: list = ()
    build_depends: list = ()
    replaces: list = ()
    conflicts: list = ()
    resolved_depends: list = ()


def default_fallback_resolver(key, peer_packages):
    if key not in peer_packages:
        print("Failed to resolve rosdep key '{0}'.".format(key))


def format_peer_depends(depends, ros_distro):
    temp = depends.name.replace("_", "-")
    return "ros-{0}-{1}".format(ros_distro, temp)


def format_depends(depends, resolved_depends, peer_packages, ros_distro):
    versions = {
        'version_lt': '<<',
        'version_lte': '<=',
        'version_eq': '=',
        'version_gte': '>=',
        'version_gt': '>>'
    }
    formatted = []
    for d in depends:
        if resolved_depends[d.name] is not None:
            for resolved_dep in resolved_depends[d.name]:
                version_depends = [k
                                   for k in versions.keys()
                                   if getattr(d, k, None) is not None]
                if not version_depends:
                    formatted.append(resolved_dep)
                else:
                    for v in version_depends:
                        formatted.append("{0} ({1} {2})".format(
                            resolved_dep, versions[v], getattr(d, v)))
        else:
            if d.name in peer_packages:
                formatted.append(format_peer_depends(d, ros_distro))
            else:
                formatted.append(d.name + "(Not Found)")

    return formatted


def extract_ubuntu_package_license(depends_name):
    pattern = re.compile("^License:(.*)$")
    file_path = "/usr/share/doc/{0}/copyright".format(depends_name)
    licenses = []
    if path.exists(file_path):
        for i, line in enumerate(open(file_path)):
            for match in re.finditer(pattern, line):
                licenses.append(match.group(1).strip())
    else:
        licenses.append("Unknown")

    return licenses


def extract_ubuntu_package_version(depends_name):
    pattern = re.compile("^Version:(.*)$")
    output = subprocess.run(['dpkg', '-s', depends_name], stdout=subprocess.PIPE, stderr=open(devnull, 'wb')).stdout.decode('utf-8')
    for i, line in enumerate(output.splitlines()):
        for match in re.finditer(pattern, line):
            return match.group(1).strip()

    return "Unknown"


def mine_packages():
    os_name = "ubuntu"
    os_version = "focal"
    ros_distro = "noetic"
    system_package = find_packages("/opt/ros/noetic/share")
    workspace_packages = find_packages("tesseract-1")

    if type(workspace_packages) == dict and workspace_packages != {}:
        peer_packages = [p.name for p in workspace_packages.values()]

        processed_pkgs = {}
        for k, v in dict(workspace_packages).items():
            evaluate_package_conditions(v, ros_distro)

            processed_pkg = ProcessedPackage()
            processed_pkg.name = v.name
            processed_pkg.version = v.version
            processed_pkg.licenses = v.licenses
            processed_pkg.depends = [
                dep for dep in (v.run_depends + v.buildtool_export_depends)
                if dep.evaluated_condition is not False]
            processed_pkg.build_depends = [
                dep for dep in (v.build_depends + v.buildtool_depends + v.test_depends)
                if dep.evaluated_condition is not False]
            processed_pkg.replaces = [
                dep for dep in v.replaces
                if dep.evaluated_condition is not False]
            processed_pkg.conflicts = [
                dep for dep in v.conflicts
                if dep.evaluated_condition is not False]
            unresolved_keys = processed_pkg.depends + processed_pkg.build_depends + processed_pkg.replaces + processed_pkg.conflicts
            processed_pkg.resolved_depends = resolve_dependencies(unresolved_keys, os_name,
                                                                  os_version, ros_distro,
                                                                  peer_packages + [d.name for d in (processed_pkg.replaces + processed_pkg.conflicts)],
                                                                  default_fallback_resolver)

            processed_pkgs[processed_pkg.name] = processed_pkg

        print("")
        # Print Workspace packages
        print("{:<40} {:<15} {:<30}".format('Workspace Package', 'Version', 'Licenses'))
        for k, v in dict(processed_pkgs).items():
            print("{:<40} {:<15} {:<30}".format(v.name, v.version, ",".join(v.licenses)))

        for k, v in dict(processed_pkgs).items():
            formatted_depends = format_depends(v.depends, v.resolved_depends, peer_packages, ros_distro)
            formatted_build_depends = format_depends(v.build_depends, v.resolved_depends, peer_packages, ros_distro)
            print("")
            print("{0}".format(v.name))
            print("    Depends      : {0}".format(";".join(formatted_depends)))
            print("    Build Depends: {0}".format(";".join(formatted_build_depends)))

        # Print each packages dependencies
        for k, v in dict(processed_pkgs).items():
            print("")
            print("{:<40} {:<40} {:<15} {:<30}".format(v.name, 'Depends', 'Version', 'Licenses'))
            for d in v.depends:
                if d.name in system_package:
                    dpkg = system_package[d.name]
                    print("{:<40} {:<40} {:<15} {:<30}".format("", d.name, dpkg.version, ",".join(dpkg.licenses)))
                elif d.name in processed_pkgs:
                    dpkg = processed_pkgs[d.name]
                    print("{:<40} {:<40} {:<15} {:<30}".format("", d.name, dpkg.version, ",".join(dpkg.licenses)))
                else:
                    for rd in v.resolved_depends[d.name]:
                        rd_pkg_version = extract_ubuntu_package_version(rd)
                        rd_pkg_licenses = extract_ubuntu_package_license(rd)
                        filtered_licenses = list(set(rd_pkg_licenses))
                        print("{:<40} {:<40} {:<15} {:<30}".format("", rd, rd_pkg_version, ",".join(filtered_licenses)))

        # Print each packages build dependencies
        for k, v in dict(processed_pkgs).items():
            print("")
            print("{:<40} {:<40} {:<15} {:<30}".format(v.name, 'Build Depends', 'Version', 'Licenses'))
            for d in v.build_depends:
                if d.name in system_package:
                    dpkg = system_package[d.name]
                    print("{:<40} {:<40} {:<15} {:<30}".format("", d.name, dpkg.version, ",".join(dpkg.licenses)))
                elif d.name in processed_pkgs:
                    dpkg = processed_pkgs[d.name]
                    print("{:<40} {:<40} {:<15} {:<30}".format("", d.name, dpkg.version, ",".join(dpkg.licenses)))
                else:
                    for rd in v.resolved_depends[d.name]:
                        rd_pkg_version = extract_ubuntu_package_version(rd)
                        rd_pkg_licenses = extract_ubuntu_package_license(rd)
                        filtered_licenses = list(set(rd_pkg_licenses))
                        print("{:<40} {:<40} {:<15} {:<30}".format("", rd, rd_pkg_version, ",".join(filtered_licenses)))


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    mine_packages()
