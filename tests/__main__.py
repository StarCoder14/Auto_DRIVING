"""전체 시험 실행.

    python -m tests
"""
from __future__ import annotations

import sys

from tests import (test_circle_model, test_geometry, test_hybrid, test_planner, test_robot,
                   test_sensors)

SUITES = [
    ("처음 기하 모델 (계획서의 알고리즘 A)", test_circle_model),
    ("기하 계산", test_geometry),
    ("하이브리드 A*", test_hybrid),
    ("싼 센서 흉내", test_sensors),
    ("실물 준비 (통신, 가짜 아두이노)", test_robot),
    ("접선 그래프 계획기 (느림, 수십 분)", test_planner),
]


def main() -> int:
    failed = 0
    for name, mod in SUITES:
        print(f"\n[{name}]")
        failed += mod.main()
    print()
    if failed:
        print(f"{failed}개 묶음 실패")
        return 1
    print("전부 통과")
    return 0


if __name__ == "__main__":
    sys.exit(main())
