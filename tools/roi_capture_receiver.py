#!/usr/bin/env python3
import argparse
import socket
from pathlib import Path


DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 5001
DEFAULT_OUTPUT_DIR = r"D:\aaa走马观碑代码\yolo\灵眼pro320\板端实拍传输ROI_PNG"
DEFAULT_FRAME_OUTPUT_DIR = r"D:\aaa走马观碑代码\yolo\灵眼pro320\板端实拍全帧图"
PROTOCOL_MAGIC = "BWROI1"


def recv_line(conn):
    data = bytearray()
    while True:
        chunk = conn.recv(1)
        if not chunk:
            raise ConnectionError("connection closed while reading line")
        if chunk == b"\n":
            return data.decode("utf-8", errors="replace")
        if chunk != b"\r":
            data.extend(chunk)


def recv_exact(conn, size):
    data = bytearray()
    while len(data) < size:
        chunk = conn.recv(size - len(data))
        if not chunk:
            raise ConnectionError("connection closed while reading payload")
        data.extend(chunk)
    return bytes(data)


def sanitize_filename(name):
    base = Path(name).name.strip()
    if not base:
        base = "roi.png"
    allowed = set("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_.")
    safe = "".join(ch if ch in allowed else "_" for ch in base)
    if not safe:
        safe = "roi.png"
    if "." not in safe:
        safe += ".png"
    return safe


def unique_output_path(output_dir, name):
    target = output_dir / sanitize_filename(name)
    if not target.exists():
        return target
    stem = target.stem
    suffix = target.suffix or ".png"
    index = 1
    while True:
        candidate = output_dir / f"{stem}_{index}{suffix}"
        if not candidate.exists():
            return candidate
        index += 1


def choose_output_dir(filename, roi_output_dir, frame_output_dir):
    safe_name = sanitize_filename(filename)
    if safe_name.startswith("frame_"):
        return frame_output_dir
    return roi_output_dir


def handle_connection(conn, roi_output_dir, frame_output_dir):
    magic = recv_line(conn)
    if magic != PROTOCOL_MAGIC:
        raise ValueError(f"unexpected magic: {magic!r}")

    filename = "roi.png"
    payload_size = None
    while True:
        line = recv_line(conn)
        if line == "":
            break
        if line.startswith("name "):
            filename = line[5:].strip() or filename
        elif line.startswith("size "):
            payload_size = int(line[5:].strip())

    if payload_size is None or payload_size <= 0:
        raise ValueError("invalid payload size")

    payload = recv_exact(conn, payload_size)
    output_dir = choose_output_dir(filename, roi_output_dir, frame_output_dir)
    output_path = unique_output_path(output_dir, filename)
    output_path.write_bytes(payload)
    ack = f"OK saved {output_path.name}\n".encode("utf-8")
    conn.sendall(ack)
    return output_path


def parse_args():
    parser = argparse.ArgumentParser(
        description="Receive lossless ROI/full-frame PNGs pushed from recognition board."
    )
    parser.add_argument("--host", default=DEFAULT_HOST, help="listen host, default: %(default)s")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="listen port, default: %(default)s")
    parser.add_argument(
        "--output",
        default=DEFAULT_OUTPUT_DIR,
        help="ROI save directory, default: %(default)s",
    )
    parser.add_argument(
        "--frame-output",
        default=DEFAULT_FRAME_OUTPUT_DIR,
        help="full-frame save directory, default: %(default)s",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    output_dir = Path(args.output)
    frame_output_dir = Path(args.frame_output)
    output_dir.mkdir(parents=True, exist_ok=True)
    frame_output_dir.mkdir(parents=True, exist_ok=True)

    print(f"[ROI RX] listen={args.host}:{args.port}")
    print(f"[ROI RX] roi_output={output_dir}")
    print(f"[ROI RX] frame_output={frame_output_dir}")

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((args.host, args.port))
        server.listen(4)

        while True:
            conn, addr = server.accept()
            with conn:
                peer = f"{addr[0]}:{addr[1]}"
                try:
                    saved_path = handle_connection(conn, output_dir, frame_output_dir)
                    print(f"[ROI RX] {peer} -> {saved_path}")
                except Exception as exc:
                    message = f"ERR {exc}\n"
                    try:
                        conn.sendall(message.encode("utf-8", errors="replace"))
                    except Exception:
                        pass
                    print(f"[ROI RX] {peer} -> {exc}")


if __name__ == "__main__":
    main()
