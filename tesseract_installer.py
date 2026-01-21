#!/usr/bin/env python3

# Testing locally
# docker run -it -v ~/catkin_ws:/home/catkin_ws --env DEBIAN_FRONTEND=noninteractive ubuntu:jammy bash
# apt update && apt install -y python3 python3-pip curl unzip sudo
# pip3 install requests
# apt install -y --no-install-recommends software-properties-common
# add-apt-repository -y ppa:levi-armstrong/tesseract-robotics
# apt update && apt install -y libompl-dev taskflow
# python3 tesseract_installer.py 0.28 jammy
# export TESSERACT_RESOURCE_PATH=/usr/share/tesseract_support:/usr/share/tesseract_task_composer

import requests
import os
import subprocess
import argparse

# List of GitHub API URLs for the upstream repositories
UPSTREAM_DEPENDS_URLS = [
  "https://api.github.com/repos/ros-industrial/ros_industrial_cmake_boilerplate/releases/tags/0.7.2",
  "https://api.github.com/repos/Jmeyer1292/opw_kinematics/releases/tags/0.5.2",
  "https://api.github.com/repos/swri-robotics/descartes_light/releases/tags/0.4.6",
]

# List of GitHub API URLs for the repositories
REPO_API_URLS = [
    "https://api.github.com/repos/tesseract-robotics/tesseract/releases",
    "https://api.github.com/repos/tesseract-robotics/trajopt/releases",
    "https://api.github.com/repos/tesseract-robotics/tesseract_planning/releases",
    "https://api.github.com/repos/tesseract-robotics/tesseract_qt/releases",
]

TOKEN = os.getenv("GITHUB_TOKEN")  # Optional: GitHub personal access token

def fetch_latest_patch(tag_version_prefix, api_url, token=None):
    headers = {"Authorization": f"token {token}"} if token else {}
    response = requests.get(api_url, headers=headers)
    response.raise_for_status()
    releases = response.json()
    # Find the latest release matching the tag prefix
    for release in releases:
        if release["tag_name"].startswith(tag_version_prefix):
            return release
    raise ValueError(f"No release found for prefix {tag_version_prefix} in {api_url}")


def fetch_assets(release):
    return release.get("assets", [])


def download_file(url, output_path):
    response = requests.get(url, stream=True)
    response.raise_for_status()
    with open(output_path, "wb") as f:
        for chunk in response.iter_content(chunk_size=8192):
            f.write(chunk)


def install_debs_from_zip(zip_file):
    print(f"Extracting {zip_file}...")
    subprocess.run(["unzip", "-o", zip_file], check=True)

    print(f"Installing all .deb files from {zip_file}...")
    deb_files = [file for file in os.listdir() if file.endswith(".deb")]
    if deb_files:
      try:
          # Step 1: Attempt to install all .deb files
          print(f"Running dpkg to install the following .deb files: {deb_files}")
          subprocess.run(["sudo", "dpkg", "-i"] + deb_files, check=True)
      except subprocess.CalledProcessError:
          print("dpkg encountered dependency issues. Attempting to resolve...")

      # Step 2: Resolve dependencies
      print("Resolving missing dependencies with apt-get -f install...")
      subprocess.run(["sudo", "env", "DEBIAN_FRONTEND=noninteractive", "apt", "-f", "install", "-y"], check=True)

      # Step 3: Reconfigure packages to ensure all are configured
      print("Reconfiguring packages to complete the installation...")
      subprocess.run(["sudo", "dpkg", "--configure", "-a"], check=True)

      # Clean up .deb files
      for file in deb_files:
          os.remove(file)
    else:
        print(f"No .deb files found in {zip_file}.")

    os.remove(zip_file)


def main(tag_version_prefix, distro_name):

    for api_url in UPSTREAM_DEPENDS_URLS:
        print(f"Processing repository: {api_url}")
        try:
            # Get the release
            headers = {"Authorization": f"token {TOKEN}"} if TOKEN else {}
            response = requests.get(api_url, headers=headers)
            response.raise_for_status()
            release_data = response.json()

            # Fetch assest
            assets = fetch_assets(release_data)

            # Filter assets based on distribution name
            filtered_assets = [
                asset for asset in assets if f"debian_package_{distro_name}.zip" in asset["name"]
            ]

            if not filtered_assets:
                print(f"No matching assets found for {distro_name} in {api_url}.")
                continue

            for asset in filtered_assets:
                name = asset["name"]
                download_url = asset["browser_download_url"]
                print(f"Downloading {name}...")
                download_file(download_url, name)

                install_debs_from_zip(name)

            print(f"Finished processing repository: {api_url}")
        except Exception as e:
            print(f"Error processing repository {api_url}: {e}")

    for api_url in REPO_API_URLS:
        print(f"Processing repository: {api_url}")
        try:
            latest_release = fetch_latest_patch(tag_version_prefix, api_url, TOKEN)
            print(f"Latest release found: {latest_release['tag_name']}")

            assets = fetch_assets(latest_release)
            # Filter assets based on distribution name
            filtered_assets = [
                asset for asset in assets if f"debian_package_{distro_name}.zip" in asset["name"]
            ]

            if not filtered_assets:
                print(f"No matching assets found for {distro_name} in {api_url}.")
                continue

            for asset in filtered_assets:
                name = asset["name"]
                download_url = asset["browser_download_url"]
                print(f"Downloading {name}...")
                download_file(download_url, name)

                install_debs_from_zip(name)

            print(f"Finished processing repository: {api_url}")
        except Exception as e:
            print(f"Error processing repository {api_url}: {e}")

    print("All repositories processed!")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Download and install Debian artifacts from multiple GitHub releases for a version prefix."
    )
    parser.add_argument(
        "tag_version_prefix",
        help="The version prefix (e.g., 0.28) to fetch the latest patch for all repositories.",
    )
    parser.add_argument(
        "distro_name",
        choices=["focal", "jammy", "noble"],
        help="The distribution name to filter assets (e.g., focal, jammy, noble).",
    )
    args = parser.parse_args()

    main(args.tag_version_prefix, args.distro_name)
