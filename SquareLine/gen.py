import sys
import os
import json
import shutil
import argparse
from zipfile import ZipFile

# Directory of all boards
BOARDS_DIR = "boards/"
# Other board files, which are same for all boards (there can be used placeholders)
COMMON_DIR = "common/"
# JSON file which describing the board (must be in each board directory)
BOARD_GENFILE = "manifest.json"
# Components specific folder in specific board folder. Will be copied from boards/BOARD/ directory
COMPONENTS_SPECIFIC_FILES = "components"
# Files, which are specific for each board. Will be copied from boards/BOARD/ directory
BOARD_SPECIFIC_FILES = {"sdkconfig.defaults", "main/idf_component.yml"}
# Generated output directory
OUT_DIR = "espressif"
# SquareLine output directory
SQUARELINE_OUTDIR = "__ui_project_name__"
# Template of SLB file
SLB_FILE = {
    "version":"",
    "group":"Espressif",
    "title": "",
    "keywords": "Espressif",
    "width": 0,
    "height": 0,
    "width_min": 0,
    "height_min": 0,
    "width_max": 0,
    "height_max": 0,
    "offset_x": 0,
    "offset_y": 0,
    "rotation": 0,
    "color_depth": "",
    "lvgl_export_path": "",
    "lvgl_include_path": "lvgl.h",
    "supported_lvgl_version": "8.2.0, 8.3.3",
    "pattern_match_files": "./CMakeLists.txt",
    "language":"C",
    "ui_export_path":"./main/ui",
    "url":"https://github.com/espressif/esp-bsp",
    "short_description": "",
    "long_description": ""
}

# ANSI terminal codes
ANSI_RED = '\033[1;31m'
ANSI_YELLOW = '\033[0;33m'
ANSI_GREEN = '\033[1;32m'
ANSI_NORMAL = '\033[0m'

# Print colored message
def color_print(message, color, newline='\n'):  # type: (str, str, Optional[str]) -> None
    """ Print a message to stderr with colored highlighting """
    sys.stderr.write('%s%s%s%s' % (color, message, ANSI_NORMAL, newline))
    sys.stderr.flush()

# Print green text
def print_ok(message):
    color_print(message, ANSI_GREEN)

# Print red error
def print_error(message):
    color_print(message, ANSI_RED)

# Remove folder with all inside on specific path
def remove_folder(path):
    for filename in os.listdir(path):
        new_path = os.path.join(path, filename)
        if os.path.isdir(new_path):
            remove_folder(new_path)
        else:
            os.remove(new_path)
    os.rmdir(path)

# When placeholder is not set in JSON, but find in code, it will be ignored (no error and no substituted)
class SafeDict(dict):
    def __missing__(self, key):
        return '{' + key + '}'

# Replace placeholder, which are set in JSON
def replace_placeholders(file, placeholders):
    if os.path.exists(file):
        with open(file, "r") as f:
            file_str = f.read()
        file_str = file_str.format_map(SafeDict(placeholders))
        with open(file, "w") as f:
            f.write(file_str)

# Copy file and replace placeholders in copied file
def copy_file(src_path, dst_path, placeholders, translate=1):
    print(f"  Copying {src_path} to {dst_path}")
    shutil.copyfile(src_path, dst_path)
    if translate:
        replace_placeholders(dst_path, placeholders)

# Copy all files in directory
def copy_directory(from_dir, to_dir, placeholders, translate=1):
    if not os.path.exists(to_dir):
        os.makedirs(to_dir)
    for filename in os.listdir(from_dir):
        f = os.path.join(from_dir, filename)
        out_f = os.path.join(to_dir, filename)
        if os.path.isdir(f):
            copy_directory(f, out_f, placeholders, translate)
        else:
            copy_file(f, out_f, placeholders, translate)

# Add folder into ZIP archive recursively
def zip_add_folder(zip_obj, path, outdir):
    for filename in os.listdir(path):
        file = os.path.join(path, filename)
        out_file = os.path.join(outdir, filename)
        out_file_rel = os.path.join(outdir, os.path.basename(file))
        zip_obj.write(file, out_file_rel)
        if os.path.isdir(file):
            zip_add_folder(zip_obj, file, out_file)

# Creates ZIP archive
def make_zip(board_name, path):
    zip_obj = ZipFile(os.path.join(path,  board_name + ".zip"), 'w')
    zip_dir = os.path.join(path, SQUARELINE_OUTDIR)
    zip_add_folder(zip_obj, zip_dir, SQUARELINE_OUTDIR)
    zip_obj.close()

# Check if JSON key exists, if not error and exit the script
def check_json_key(manifest, key):
    if key not in manifest:
        print_error(f"Missing {key} in manifest JSON file")
        raise SystemExit(1)

# Get version in filename format
def get_board_version(manifest):
    check_json_key(manifest, "version")
    v = manifest["version"].split('.')
    return f"_v{v[0]}_{v[1]}_{v[2]}"

# Create SLB file by JSON manifest data
def create_slb_file(output, output_filename, manifest):
    check_json_key(manifest, "name")
    check_json_key(manifest, "version")
    check_json_key(manifest, "mcu")
    check_json_key(manifest, "screen_width")
    check_json_key(manifest, "screen_height")
    check_json_key(manifest, "screen_color_swap")
    check_json_key(manifest, "short_description")
    check_json_key(manifest, "long_description")

    SLB_FILE["version"] = manifest["version"]
    SLB_FILE["title"] = manifest["name"]
    SLB_FILE["keywords"] += ", " + manifest["name"] + ", " + manifest["mcu"]
    SLB_FILE["width"] = manifest["screen_width"]
    SLB_FILE["width_min"] = manifest["screen_width"]
    SLB_FILE["width_max"] = manifest["screen_width"]
    SLB_FILE["height"] = manifest["screen_height"]
    SLB_FILE["height_min"] = manifest["screen_height"]
    SLB_FILE["height_max"] = manifest["screen_height"]
    if manifest["screen_color_swap"]:
        SLB_FILE["color_depth"] = "16 sw"
    else:
        SLB_FILE["color_depth"] = "16"
    SLB_FILE["short_description"] = manifest["short_description"]
    SLB_FILE["long_description"] = manifest["long_description"]

    out_slb_json = json.dumps(SLB_FILE, indent=4)
    slb_file = os.path.join(output, output_filename + ".slb")
    with open(slb_file, "w") as f:
        f.write(out_slb_json)

    print(f"  File {output_filename}.slb created.")


# Process of the board generating
def process_board(board_name, output, dir):
    print(f"Processing board: {dir}")

    path = os.path.join(dir, BOARD_GENFILE)
    if os.path.exists(path):
        # Parse manifest file
        with open(path, "r") as f:
            manifest = json.loads(f.read())

        # Get string version for board folder names
        board_ver = get_board_version(manifest)
        output = output + board_ver
        output_filename = board_name

        check_json_key(manifest, "placeholders")
        placeholders = manifest["placeholders"]

        # Copy common files
        squareline_dir_path = os.path.join(output, SQUARELINE_OUTDIR)
        copy_directory(COMMON_DIR, squareline_dir_path, placeholders)
        # Copy specific board files
        for file in BOARD_SPECIFIC_FILES:
            copy_file(os.path.join(dir, file), os.path.join(squareline_dir_path, file), placeholders)
        # Copy components, if exists
        components_dir_path = os.path.join(dir, COMPONENTS_SPECIFIC_FILES)
        if os.path.exists(components_dir_path):
            copy_directory(components_dir_path, os.path.join(squareline_dir_path, "components"), placeholders, 0)
        # Copy image
        copy_file(os.path.join(dir, "image.png"), os.path.join(output, output_filename + ".png"), placeholders, 0)

        create_slb_file(output, output_filename, manifest)
        make_zip(output_filename, output)
        remove_folder(squareline_dir_path)
    else:
        print_error(f"ERROR: File '{path}' is not exists!")
        raise SystemExit(1)

# Print usage
def print_help():
    print("Not enough arguments! Usage:")
    print(" gen.py out board")
    print("\tout: path to the generated output")
    print("\tboard (optional): generate only specific board")

#Create gitignore file into output folder
def create_gitignore(path):
    with open(os.path.join(path, ".gitignore"), "w") as f:
        f.write("*")


def main(outdir, sel_board):
    output_folder = os.path.join(outdir, OUT_DIR)
    if not os.path.exists(outdir):
        os.mkdir(outdir)

    create_gitignore(outdir)

    if sel_board:
        # Process only one specific board
        board_name = sel_board
        board_path = BOARDS_DIR + board_name
        output_folder = os.path.join(output_folder, board_name)
        # Remove all in output directory
        if os.path.exists(output_folder):
            shutil.rmtree(output_folder)
        # Process
        process_board(board_name, output_folder, board_path)
    else:
        # Remove all in output directory
        if os.path.exists(output_folder):
            shutil.rmtree(output_folder)
        #loop all boards
        for dirname in os.listdir(BOARDS_DIR):
            d = os.path.join(BOARDS_DIR, dirname)
            # checking if it is a file
            if os.path.isdir(d) and "custom" not in dirname:
                process_board(dirname, os.path.join(output_folder, dirname), d)

    print_ok(f"Generating done! Output folder: {outdir}")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Generates SquareLine board packages.')
    parser.add_argument("-o", "--outdir", action="store", dest='outdir', default='out',
                    help='Output directory for generated files (default: out)')
    parser.add_argument("-b", "--board", action="store", dest='sel_board', default='',
                    help='Generate only selected board (default: generates all)')

    args = parser.parse_args()
    main(args.outdir, args.sel_board)

