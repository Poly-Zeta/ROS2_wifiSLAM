import sys
import re
from pathlib import Path


# -------------------------
# Utility: detect ROS deps
# -------------------------
def detect_ros_dep_from_include(include: str):
    msg_patterns = {
        "std_msgs": r"std_msgs/msg",
        "sensor_msgs": r"sensor_msgs/msg",
        "geometry_msgs": r"geometry_msgs/msg",
        "nav_msgs": r"nav_msgs/msg",
        "tf2": r"tf2/",
        "tf2_ros": r"tf2_ros/",
        "rclcpp": r"rclcpp/",
    }
    for pkg, pattern in msg_patterns.items():
        if pattern in include:
            return pkg
    return None


def is_external_library(include: str):
    external_patterns = [
        r"Eigen/", r"opencv2/", r"boost/", r"PCL/", r"yaml-cpp",
        r"GL/", r"cuda", r"fftw", r"serial/", r"asio/"
    ]
    return any(p.lower() in include.lower() for p in external_patterns)


# -------------------------
# Main generator
# -------------------------
def generate_files(project_name: str, base_dir: Path, dry_run: bool):
    deps = set(["rclcpp", "std_msgs"])

    # -------------------------
    # Detect msg files
    # -------------------------
    msg_dir = base_dir / "msg"
    msg_files = [f.name for f in msg_dir.glob("*.msg")] if msg_dir.exists() else []

    for msg in msg_files:
        text = (msg_dir / msg).read_text()
        for line in text.splitlines():
            if "/" in line:
                pkg = line.split("/")[0]
                if pkg.endswith("_msgs"):
                    deps.add(pkg)

    # -------------------------
    # Detect config files
    # -------------------------
    config_dir = base_dir / "config"
    config_files = [f.name for f in config_dir.glob("*.yaml")] if config_dir.exists() else []

    # -------------------------
    # Detect launch dependencies
    # -------------------------
    launch_dir = base_dir / "launch"
    if launch_dir.exists():
        for lf in launch_dir.glob("*.py"):
            text = lf.read_text()
            matches = re.findall(r'package\s*=\s*[\'"]([^\'"]+)[\'"]', text)
            for pkg in matches:
                deps.add(pkg)

            if "package=" in text and "package=\"" not in text:
                print(f"[WARN] launch 内で package 名が静的に判定できません: {lf.name}")

    # -------------------------
    # Detect C++ includes
    # -------------------------
    src_dir = base_dir / "src"
    include_dir = base_dir / "include" / project_name

    cpp_files = list(src_dir.glob("*.cpp"))
    hpp_files = list(include_dir.glob("*.hpp"))

    for f in cpp_files + hpp_files:
        text = f.read_text()
        includes = re.findall(r'#include\s*[<"]([^">]+)[">]', text)

        for inc in includes:
            inc = inc.strip()

            # 自パッケージ include の完全除外
            if inc.startswith(f"{project_name}/") or f"/{project_name}/" in inc:
                continue

            ros_dep = detect_ros_dep_from_include(inc)
            if ros_dep:
                deps.add(ros_dep)
                continue

            if is_external_library(inc):
                print(f"[WARN] 外部ライブラリと思われる include を検出: {inc}")
                continue

            if "/" in inc:
                print(f"[WARN] 依存パッケージを推論できません: {inc}")

    if project_name in deps:
        deps.remove(project_name)

    # -------------------------
    # Generate CMakeLists.txt
    # -------------------------
    cmake_lines = [
        "cmake_minimum_required(VERSION 3.8)",
        f"project({project_name})",
        "",
        "if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES \"Clang\")",
        "  add_compile_options(-Wall -Wextra -Wpedantic)",
        "endif()",
        "",
        "find_package(ament_cmake_auto REQUIRED)",
    ]

    if msg_files:
        cmake_lines.append("find_package(rosidl_default_generators REQUIRED)")

    cmake_lines += [
        "",
        "ament_auto_find_build_dependencies()",
        "",
        "include_directories(include)",
        "",
        "# Node library",
        "ament_auto_add_library(${PROJECT_NAME}_node SHARED",
        f"  src/{project_name}_node.cpp",
        f"  include/{project_name}/{project_name}.hpp",
        ")",
        "ament_target_dependencies(${PROJECT_NAME}_node " + " ".join(sorted(deps)) + ")",
        "",
        "# Main executable",
        "ament_auto_add_executable(${PROJECT_NAME}_main",
        f"  src/{project_name}_main.cpp",
        ")",
        "target_link_libraries(${PROJECT_NAME}_main ${PROJECT_NAME}_node)",
        "ament_target_dependencies(${PROJECT_NAME}_main " + " ".join(sorted(deps)) + ")",
        "",
        "# Install launch",
        "install(DIRECTORY launch",
        "  DESTINATION share/${PROJECT_NAME}",
        ")",
    ]

    if config_files:
        cmake_lines += [
            "",
            "# Install config",
            "install(DIRECTORY config",
            "  DESTINATION share/${PROJECT_NAME}/config",
            ")",
        ]

    if msg_files:
        cmake_lines += [
            "",
            "# Message generation",
            f"rosidl_generate_interfaces({project_name}",
            *[f"  msg/{m}" for m in msg_files],
            "  DEPENDENCIES std_msgs",
            ")",
        ]

    cmake_lines.append("")
    cmake_lines.append("ament_auto_package()")

    cmake_content = "\n".join(cmake_lines)

    # -------------------------
    # Generate package.xml
    # -------------------------
    xml_lines = [
        '<?xml version="1.0"?>',
        '<package format="3">',
        f"  <name>{project_name}</name>",
        "  <version>0.0.1</version>",
        f"  <description>Auto-generated package.xml for {project_name}</description>",
        '  <maintainer email="user@example.com">Your Name</maintainer>',
        "  <license>Apache-2.0</license>",
        "",
        "  <buildtool_depend>ament_cmake</buildtool_depend>",
    ]

    for d in sorted(deps):
        xml_lines.append(f"  <depend>{d}</depend>")

    if msg_files:
        xml_lines.append("  <depend>rosidl_default_generators</depend>")
        xml_lines.append("  <depend>rosidl_default_runtime</depend>")

    xml_lines += [
        "",
        "  <export>",
        "    <build_type>ament_cmake</build_type>",
        "  </export>",
        "</package>",
    ]

    xml_content = "\n".join(xml_lines)

    # -------------------------
    # Write output
    # -------------------------
    if dry_run:
        out_dir = Path(__file__).parent
        (out_dir / "CMakeLists.txt.test").write_text(cmake_content)
        (out_dir / "package.xml.test").write_text(xml_content)
        print("[DRY-RUN] テスト出力を生成しました")
    else:
        (base_dir / "CMakeLists.txt").write_text(cmake_content)
        (base_dir / "package.xml").write_text(xml_content)
        print("[OK] CMakeLists.txt と package.xml を上書き生成しました")


# -------------------------
# Entry point
# -------------------------
if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python gen_build_files.py <projectName> <projectPath> [--dry-run]")
        sys.exit(1)

    project_name = sys.argv[1]
    base_dir = Path(sys.argv[2])
    dry_run = ("--dry-run" in sys.argv)

    generate_files(project_name, base_dir, dry_run)