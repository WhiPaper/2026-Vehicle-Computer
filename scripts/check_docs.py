#!/usr/bin/env python3
"""Validate project documentation against the ROS launch and config contract."""

from __future__ import annotations

import ast
from pathlib import Path
import re
import sys
from typing import Iterable

import yaml


ROOT = Path(__file__).resolve().parents[1]
DOCS = ROOT / "docs"
PROJECT_MARKDOWN = [
    path
    for path in ROOT.rglob("*.md")
    if not any(
        part in {".git", "build", "install", "log", ".pytest_cache", "vendor"}
        for part in path.relative_to(ROOT).parts
    )
]
LAUNCH_ROOTS = (
    ROOT / "src" / "vc_bringup" / "launch",
    ROOT / "src" / "vc_safety" / "launch",
    ROOT / "src" / "vc_visualization" / "launch",
)
REQUIRED_DOCS = (
    ROOT / "README.md",
    ROOT / "CONTRIBUTING.md",
    DOCS / "interfaces.md",
    DOCS / "operations.md",
    DOCS / "development.md",
    DOCS / "architecture.md",
    DOCS / "ros2-native-design.md",
    DOCS / "visualization.md",
    DOCS / "acceptance-testing.md",
)
REQUIRED_HEADINGS = {
    ROOT / "README.md": ("문서 길찾기", "빠른 시작", "운영 배포", "검증"),
    DOCS / "interfaces.md": ("주요 토픽", "서비스", "Safety gate 파라미터", "Launch profiles"),
    DOCS / "operations.md": ("시작 전 preflight", "장애 대응", "업데이트·롤백"),
    DOCS / "development.md": ("빌드와 테스트", "실행 profile", "launch argument 확인"),
    DOCS / "architecture.md": ("시스템 경계", "Motion gate", "Runtime profiles"),
    DOCS / "ros2-native-design.md": ("구현 상태", "Launch 구조", "검증과 변경 순서"),
    DOCS / "visualization.md": ("Live hardware", "MCAP replay", "Gazebo simulation"),
    DOCS / "acceptance-testing.md": ("실행 기록", "판정 기준"),
}


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def check(condition: bool, message: str, errors: list[str]) -> None:
    if not condition:
        errors.append(message)


def local_links(path: Path) -> Iterable[tuple[str, Path]]:
    markdown = read(path)
    pattern = re.compile(r"\]\(([^)#\s]+)(?:#[^)]+)?\)")
    for target in pattern.findall(markdown):
        if target.startswith(("http://", "https://", "mailto:")):
            continue
        yield target, (path.parent / target).resolve()


def launch_arguments(path: Path) -> dict[str, ast.Call]:
    tree = ast.parse(read(path), filename=str(path))
    arguments: dict[str, ast.Call] = {}
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if not isinstance(node.func, ast.Name) or node.func.id != "DeclareLaunchArgument":
            continue
        if not node.args or not isinstance(node.args[0], ast.Constant):
            continue
        name = node.args[0].value
        if isinstance(name, str):
            arguments[name] = node
    return arguments


def check_links(errors: list[str]) -> None:
    for path in PROJECT_MARKDOWN:
        for target, resolved in local_links(path):
            check(
                resolved.is_file(),
                f"{path.relative_to(ROOT)} links to missing {target}",
                errors,
            )


def check_required_docs(errors: list[str]) -> None:
    for path in REQUIRED_DOCS:
        check(path.is_file(), f"missing required document: {path.relative_to(ROOT)}", errors)

    readme = read(ROOT / "README.md")
    for path in REQUIRED_DOCS[1:]:
        check(
            str(path.relative_to(ROOT)) in readme,
            f"README.md does not link {path.relative_to(ROOT)}",
            errors,
        )


def check_headings(errors: list[str]) -> None:
    for path, headings in REQUIRED_HEADINGS.items():
        content = read(path)
        for heading in headings:
            check(
                any(
                    line.startswith("#") and heading in line
                    for line in content.splitlines()
                ),
                f"{path.relative_to(ROOT)} is missing heading: {heading}",
                errors,
            )


def check_launch_contract(errors: list[str]) -> None:
    interface_doc = read(DOCS / "interfaces.md")
    for launch_root in LAUNCH_ROOTS:
        for path in sorted(launch_root.glob("*.launch.py")):
            arguments = launch_arguments(path)
            for name, node in arguments.items():
                check(
                    any(keyword.arg == "description" for keyword in node.keywords),
                    f"{path.relative_to(ROOT)} argument {name} has no description",
                    errors,
                )

    vehicle = launch_arguments(ROOT / "src/vc_bringup/launch/vehicle.launch.py")
    for name in vehicle:
        check(
            f"| {name} |" in interface_doc,
            f"interfaces.md is missing vehicle.launch.py argument: {name}",
            errors,
        )
    check(
        "log_level" not in interface_doc
        and "log_level" not in read(DOCS / "ros2-native-design.md"),
        "unsupported log_level appears in a design/interface document",
        errors,
    )


def check_config_contract(errors: list[str]) -> None:
    interface_doc = read(DOCS / "interfaces.md")
    vehicle_path = ROOT / "src" / "vc_bringup" / "config" / "vehicle.yaml"
    example_path = ROOT / "src" / "vc_bringup" / "config" / "vehicle.example.yaml"
    safety_path = ROOT / "src" / "vc_safety" / "config" / "safety.yaml"

    vehicle = yaml.safe_load(read(vehicle_path))
    example = yaml.safe_load(read(example_path))
    safety = yaml.safe_load(read(safety_path))["/**"]["ros__parameters"]

    check(
        vehicle["vehicle"]["calibrated"] is True,
        "vehicle.yaml must be the calibrated operating profile",
        errors,
    )
    check(
        example["vehicle"]["calibrated"] is False,
        "vehicle.example.yaml must remain an uncalibrated fail-closed example",
        errors,
    )

    vehicle_keys = {"schema_version", *vehicle["vehicle"], *vehicle["imu"]}
    for key in vehicle_keys:
        check(
            key in interface_doc,
            f"interfaces.md is missing vehicle config key: {key}",
            errors,
        )
    for key in safety:
        check(
            key in interface_doc,
            f"interfaces.md is missing safety parameter: {key}",
            errors,
        )


def check_tests_and_commands(errors: list[str]) -> None:
    test_path = ROOT / "src" / "vc_bringup" / "test" / "test_launch_profiles.py"
    test_content = read(test_path)
    check(
        "vehicle.example.yaml" in test_content,
        "launch profile test does not exercise vehicle.example.yaml",
        errors,
    )
    check(
        "python3-python3-yaml" not in read(ROOT / "README.md"),
        "README contains an invalid python3-yaml package name",
        errors,
    )


def main() -> int:
    errors: list[str] = []
    check_links(errors)
    check_required_docs(errors)
    check_headings(errors)
    check_launch_contract(errors)
    check_config_contract(errors)
    check_tests_and_commands(errors)

    if errors:
        for error in errors:
            print(f"ERROR: {error}", file=sys.stderr)
        return 1

    print("documentation contract checks passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
