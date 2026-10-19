#!/usr/bin/env python3
"""
deploy.py -- 오렌지파이로 코드를 보내고, 보드에서 확인하고, 주행 기록을 가져온다

예전에는 scp 명령을 파일마다 손으로 쳤다 (실물_올리기.md). 이것 하나로 묶는다.

  python deploy.py 192.168.1.217            # 보드로 코드 보내고 시험까지 돌림
  python deploy.py 192.168.1.217 --check    # 접속만 확인 (아무것도 안 보냄)
  python deploy.py 192.168.1.217 --no-test  # 보내기만 하고 시험은 건너뜀
  python deploy.py 192.168.1.217 --pull-logs # 보드의 logs/ 를 노트북으로 가져옴
  python deploy.py 192.168.1.217 --time      # 가짜 아두이노로 계획 주기 시간만 잼

핀 번호는 아두이노 펌웨어에만 필요하다. 여기서 보내는 오렌지파이 파이썬 코드는 핀과 상관없다.
그래서 차체가 확정되기 전에도 올려서 보드 속도를 미리 잴 수 있다.

보내는 것   rccar/, tests/, run*.py, layouts/, requirements.txt, *.md
빼는 것     recordings/, logs/, 버릴것/, __pycache__  (보드에 필요 없거나 보드에서 생긴다)
"""

import argparse
import io
import subprocess
import sys
import tarfile
import tempfile
import time

USER = "orangepi"
REMOTE_DIR = "rccar"          # 보드의 ~/rccar
SSH_OPTS = ["-o", "ConnectTimeout=6", "-o", "BatchMode=yes",
            "-o", "StrictHostKeyChecking=accept-new"]

# 보낼 것: (로컬 경로, 파일이면 True)
INCLUDE_DIRS = ["rccar", "tests", "layouts"]
INCLUDE_GLOBS = ["run.py", "run_robot.py", "run_circle_model.py", "run_error_split.py",
                 "run_forever.py", "loop.py", "requirements.txt"]
INCLUDE_DOCS = ["README.md", "HANDOFF.md", "ALGORITHM.md", "실물_올리기.md",
                "하드웨어_준비.md", "보고서_구상.md", "알고리즘_정리.md"]
EXCLUDE_PARTS = ("__pycache__", ".pyc", "/recordings/", "/logs/", "/버릴것/")


def host(address):
    return f"{USER}@{address}"


def _skip(name):
    probe = "/" + name.replace("\\", "/") + "/"
    return any(part in probe for part in EXCLUDE_PARTS)


def check(address):
    """접속되는지 본다. 되면 보드의 파이썬 버전을 돌려준다."""
    result = subprocess.run(
        ["ssh", *SSH_OPTS, host(address), "python3 --version"],
        capture_output=True, text=True)
    if result.returncode != 0:
        return None, (result.stderr or result.stdout).strip()
    return (result.stdout + result.stderr).strip(), None


def explain_no_connection(address, detail):
    """못 붙었을 때, 9월에 겪은 일을 바탕으로 무엇을 볼지 알려준다."""
    print(f"보드({address})에 접속하지 못했습니다.\n  ({detail})\n")
    print("확인할 것 (실물_올리기.md 기록):")
    print("  - 노트북과 보드가 같은 대역이어야 합니다. 학교망은 날마다 다른 대역을 줍니다.")
    print("    노트북이 Wi-Fi 로 11.x 인데 보드가 192.168.x 이면 서로 다른 망이라 안 됩니다.")
    print("    이때는 노트북 랜선도 보드와 같은 스위치(공유기)에 꽂아 같은 대역을 받아야 합니다.")
    print("  - 보드 화면의 네트워크 아이콘에서 지금 주소를 다시 확인하세요.")
    print("  - 그래도 안 되면 보드에 키보드와 모니터를 꽂아 직접 작업합니다 (무선/원격 없이도 주행은 됩니다).")


def build_archive():
    """보낼 파일을 tar.gz 하나로 묶는다. 한글 경로와 빼기 규칙을 파이썬이 처리해 shell 따옴표 문제를 피한다."""
    import glob
    import os

    buffer = io.BytesIO()
    count = 0
    with tarfile.open(fileobj=buffer, mode="w:gz") as tar:
        def add(path):
            nonlocal count
            if os.path.isdir(path):
                for root, dirs, names in os.walk(path):
                    dirs[:] = [d for d in dirs if not _skip(os.path.join(root, d))]
                    for name in names:
                        full = os.path.join(root, name)
                        if not _skip(full):
                            tar.add(full, arcname=full.replace("\\", "/"))
                            count += 1
            elif os.path.exists(path):
                tar.add(path, arcname=path.replace("\\", "/"))
                count += 1

        for directory in INCLUDE_DIRS:
            add(directory)
        for pattern in INCLUDE_GLOBS + INCLUDE_DOCS:
            for match in glob.glob(pattern):
                add(match)
    buffer.seek(0)
    return buffer.read(), count


def push(address):
    """묶은 파일을 보드로 보내고 ~/rccar 에 푼다."""
    blob, count = build_archive()
    print(f"보낼 파일 {count}개 ({len(blob) // 1024} KB)")
    with tempfile.NamedTemporaryFile(suffix=".tar.gz", delete=False) as tmp:
        tmp.write(blob)
        local_tar = tmp.name
    remote_tar = "/tmp/rccar_deploy.tar.gz"
    if subprocess.run(["scp", *SSH_OPTS, local_tar, f"{host(address)}:{remote_tar}"]).returncode != 0:
        return False
    # 보드에서 풀기. 기존 파일 위에 덮어씀 (logs, recordings 는 건드리지 않음)
    unpack = (f"mkdir -p ~/{REMOTE_DIR} && tar xzf {remote_tar} -C ~/{REMOTE_DIR} && "
              f"rm -f {remote_tar} && echo 풀기완료")
    done = subprocess.run(["ssh", *SSH_OPTS, host(address), unpack]).returncode == 0
    import os
    os.unlink(local_tar)
    return done


def run_remote(address, command, label):
    print(f"\n== 보드에서: {label} ==")
    full = f"cd ~/{REMOTE_DIR} && PYTHONIOENCODING=utf-8 {command}"
    return subprocess.run(["ssh", *SSH_OPTS, host(address), full]).returncode == 0


def run_tests(address):
    quick = ("python3 -m tests.test_car_specs && python3 -m tests.test_hybrid && "
             "python3 -m tests.test_robot && python3 -m tests.test_circle_model")
    return run_remote(address, quick, "빠른 시험 (차 사양, 하이브리드, 실물, 기하 모델)")


def time_board(address):
    """보드에서 가짜 아두이노로 한 판 달리며 계획 주기 시간을 잰다.

    주행 기록을 1초에 50번 쓰는 것이 보드에서 주행 루프를 느리게 하지 않는지 본다.
    9월 16일 측정: 계획 평균 16.0 ms, 최악 68.1 ms (예산 100 ms).
    """
    return run_remote(address,
                      "python3 run_robot.py --fake --seed 2005 --quiet --log /tmp/board_timing.csv",
                      "계획 주기 시간 (가짜 아두이노)")


def pull_logs(address):
    import os
    os.makedirs("logs", exist_ok=True)
    print("보드의 logs/ 를 가져옵니다 (실차 주행 기록). 가짜 기록은 보드에 두고 실차 것만 받으려면 직접 고르세요.")
    return subprocess.run(
        ["scp", *SSH_OPTS, "-r", f"{host(address)}:~/{REMOTE_DIR}/logs/.", "logs/"]).returncode == 0


def main():
    parser = argparse.ArgumentParser(description="오렌지파이로 보내고 확인하기")
    parser.add_argument("address", help="보드 주소. 예 192.168.1.217")
    parser.add_argument("--check", action="store_true", help="접속만 확인")
    parser.add_argument("--no-test", action="store_true", help="보내되 시험은 건너뜀")
    parser.add_argument("--time", action="store_true", help="보드에서 계획 주기 시간을 잼")
    parser.add_argument("--pull-logs", action="store_true", help="보드의 주행 기록을 가져옴")
    args = parser.parse_args()

    version, error = check(args.address)
    if error:
        explain_no_connection(args.address, error)
        return 1
    print(f"접속됨: {host(args.address)}  (보드 {version})")
    if args.check:
        return 0

    if args.pull_logs:
        return 0 if pull_logs(args.address) else 1

    if not push(args.address):
        print("보내기 실패")
        return 1
    print("보내기 완료")

    ok = True
    if not args.no_test:
        ok = run_tests(args.address) and ok
    if args.time:
        ok = time_board(args.address) and ok
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
