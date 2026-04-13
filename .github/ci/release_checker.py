# SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: Apache-2.0

# flake8: noqa

import os
import yaml
import subprocess
from tabulate import tabulate
import re
from datetime import datetime
import wcwidth
from pathlib import Path
import pytz

repo_path = Path(".")

target_dirs = [
    os.path.join(repo_path, "bsp"),
    os.path.join(repo_path, "components"),
]

deprecated = [
    "esp-box",
    "esp-box-lite",
    "esp32_azure_iot_kit",
    "esp32_s2_kaluga_kit",
    "hts221",

]

priority_order = {
    "⛔ Yes": 0,
    "⚠️ MD ": 1,
    "✔️ No ": 2
}

results = []

release_commits = {}
component_paths = {}


def run_git_command(args, cwd):
    result = subprocess.run(["git"] + args, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    return result.stdout.strip()


for base_dir in target_dirs:
    if os.path.exists(base_dir):
        for root, dirs, files in os.walk(base_dir):
            if "idf_component.yml" in files:
                yml_path = os.path.join(root, "idf_component.yml")
                component_name = os.path.basename(root)
                version = "N/A"
                release_date = "?"
                changes_since_version = "N/A"
                commit_count = "0"

                if component_name in deprecated:
                    continue

                try:
                    with open(yml_path, "r") as f:
                        yml_data = yaml.safe_load(f)
                        version = yml_data.get("version", "N/A")
                except Exception as e:
                    print(f"Chyba: {e}")

                if version != "N/A":
                    try:
                        rel_yml_path = os.path.relpath(yml_path, repo_path).replace("\\", "/")
                        log_output = run_git_command(["log", "-p", "-m", "--", rel_yml_path], cwd=repo_path)

                        current_commit = None
                        current_date = None
                        old_version = None
                        new_version = None
                        commit_hash = None

                        lines = log_output.splitlines()
                        for i, line in enumerate(lines):
                            if line.startswith("commit "):
                                current_commit = line.split()[1]
                                old_version = None
                                new_version = None
                            elif line.startswith("Date:"):
                                raw_date = line.replace("Date:", "").strip()
                                try:
                                    dt = datetime.strptime(raw_date, "%a %b %d %H:%M:%S %Y %z")
                                    current_date = dt.strftime("%d.%m.%Y")
                                except Exception as e:
                                    print(f"Chyba: {e}")
                                    current_date = raw_date
                            elif line.startswith("-version:") and not line.startswith("  "):
                                match = re.match(r"-version:\s*['\"]?([\w\.\-~]+)['\"]?", line)
                                if match:
                                    old_version = match.group(1)
                            elif line.startswith("+version:") and not line.startswith("  "):
                                match = re.match(r"\+version:\s*['\"]?([\w\.\-~]+)['\"]?", line)
                                if match:
                                    new_version = match.group(1)

                            if old_version and new_version and old_version != new_version:
                                commit_hash = current_commit
                                release_date = current_date
                                break

                        if not commit_hash:
                            first_commit = run_git_command(["log", "--diff-filter=A", "--format=%H %aD", "--", rel_yml_path], cwd=repo_path)
                            if first_commit:
                                parts = first_commit.split()
                                commit_hash = parts[0]
                                try:
                                    dt = datetime.strptime(" ".join(parts[1:]), "%a, %d %b %Y %H:%M:%S %z")
                                    release_date = dt.strftime("%d.%m.%Y")
                                except Exception as e:
                                    print(f"Chyba: {e}")
                                    release_date = "?"

                        if commit_hash:
                            rel_component_path = os.path.relpath(root, repo_path).replace("\\", "/")

                            # Save
                            release_commits[component_name] = commit_hash
                            component_paths[component_name] = rel_component_path

                            diff_output = run_git_command(["diff", "--name-only", f"{commit_hash}..HEAD", "--", rel_component_path], cwd=repo_path)

                            extensions = {}
                            changed_paths = diff_output.splitlines()
                            if diff_output:
                                extensions = {os.path.splitext(path)[1] for path in changed_paths}

                            if extensions == {'.md'}:
                                changes_since_version = "⚠️ MarkDown "
                            elif changed_paths and all("test_apps/" in path for path in changed_paths):
                                changes_since_version = "🧪 Test only"
                            elif extensions:
                                changes_since_version = "⛔ Yes"
                            else:
                                changes_since_version = "✔️ No "

                            # count_output = run_git_command(["rev-list", f"{commit_hash}..HEAD", "--count", rel_component_path], cwd=repo_path)
                            commit_count = len(changed_paths) if changed_paths else "0"

                    except Exception as e:
                        print(f"Chyba: {e}")

                    if release_date != "?":
                        extension_str = ", ".join(sorted(extensions)) if extensions else " "
                        results.append([component_name, version, release_date, changes_since_version + f" ({commit_count})", extension_str])


def show_diff_for_component(component_name):
    commit_hash = release_commits.get(component_name)
    rel_path = component_paths.get(component_name)

    if not commit_hash or not rel_path:
        print("Commit path not found.")
        return

    # List of changed files
    changed_files = run_git_command(["diff", "--name-only", f"{commit_hash}..HEAD", "--", rel_path], cwd=repo_path)
    changed_files = [f for f in changed_files.splitlines() if not f.endswith(".md")]

    if not changed_files:
        print("No changes except *.md files.")
        return

    print(f"Changes for component '{component_name}' from last release (except *.md files):\n")
    subprocess.run(["git", "diff", "--color=always", f"{commit_hash}..HEAD", "--"] + changed_files, cwd=repo_path)


# Calculate width
def visual_width(text):
    return sum(wcwidth.wcwidth(c) for c in text)


# Text align
def pad_visual(text, target_width):
    current_width = visual_width(text)
    padding = max(0, target_width - current_width)
    return text + " " * padding


def get_change_key(row):
    change = row[3].strip()
    extensions = row[4].split(", ")
    has_code_change = any(ext in ['.c', '.h'] for ext in extensions)

    if change.startswith("⛔") and has_code_change:
        return 0
    elif change.startswith("⛔"):
        return 1
    elif change.startswith("🧪"):
        return 2
    elif change.startswith("⚠️"):
        return 3
    elif change.startswith("✔️"):
        return 4
    return 99


# Sort by priority
results.sort(key=get_change_key)

# Column align
for row in results:
    row[3] = pad_visual(row[3], 8)

# Table header
headers = ["Component", "Version", "Released", "Changed", "File Types"]

tz = pytz.timezone("Europe/Prague")
last_updated = datetime.now(tz).strftime("%d.%m.%Y %H:%M:%S %Z")
if os.getenv("CI") != "true":
    markdown_table = tabulate(results, headers=headers, tablefmt="github")
    print("# Component/BSP release version checker")
    print("This page shows all components in the BSP repository with their latest versions and indicates whether any changes have not yet been released.")
else:
    markdown_table = tabulate(results, headers=headers, tablefmt="html")
    deprecated_str = ", ".join(deprecated)
    print("<html><head>")
    print("<title>Component/BSP release version checker</title>")
    print(f"""<style>
        body {{ font-family: sans-serif; padding: 2em; }}
        table {{ border-collapse: collapse; width: 100%; }}
        th, td {{ border: 1px solid #ccc; padding: 0.5em; text-align: left; }}
        th {{ background-color: #f0f0f0; }}
        td:nth-child(4) {{ text-align: center; }}
        td:nth-child(5) {{ font-style: italic; color: #888; }}
    </style>""")
    print("</head><body>")
    print("<h1>Component/BSP release version checker</h1>")
    print(f"<p>Last updated: {last_updated}</p>")
    print("<p>This page shows all components in the BSP repository with their latest versions and indicates whether any changes have not yet been released.</p>")
    print(f"<p>Deprecated components: {deprecated_str}</p>")
    print("</body></html>")

print(markdown_table)

if os.getenv("CI") != "true":
    while True:
        component_name = input("Input the component name for diff (or type 'exit' to quit): ")
        if component_name.lower() == 'exit':
            break
        show_diff_for_component(component_name)
