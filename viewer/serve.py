#!/usr/bin/env python3
"""割草机 3D 查看器 — 本地启动器.

在本机 127.0.0.1 上启动一个只读的静态网页服务器，并自动用默认浏览器打开查看器。
关闭这个窗口（或按 Ctrl+C）即可停止。
"""
import functools, http.server, os, socket, sys, threading, webbrowser

ROOT = os.path.dirname(os.path.abspath(__file__))


def free_port(start=8765):
    for p in range(start, start + 100):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            try:
                s.bind(("127.0.0.1", p))
                return p
            except OSError:
                continue
    raise SystemExit("找不到可用端口")


class Handler(http.server.SimpleHTTPRequestHandler):
    extensions_map = {**http.server.SimpleHTTPRequestHandler.extensions_map,
                      ".html": "text/html; charset=utf-8", ".css": "text/css; charset=utf-8",
                      ".js": "text/javascript; charset=utf-8", ".mjs": "text/javascript; charset=utf-8",
                      ".glb": "model/gltf-binary", ".json": "application/json; charset=utf-8",
                      ".png": "image/png", ".md": "text/markdown; charset=utf-8"}

    def end_headers(self):
        self.send_header("Cache-Control", "no-cache")   # always pick up replaced models / code
        super().end_headers()

    def log_message(self, *args):
        pass


def main():
    port = None
    if "--port" in sys.argv:
        try:
            port = int(sys.argv[sys.argv.index("--port") + 1])
            if not 1 <= port <= 65535:
                raise ValueError
        except (IndexError, ValueError):
            raise SystemExit("用法: python3 serve.py [--port 端口号(1-65535)] [--no-browser]")
    port = port or free_port()
    handler = functools.partial(Handler, directory=ROOT)
    httpd = http.server.ThreadingHTTPServer(("127.0.0.1", port), handler)
    url = f"http://127.0.0.1:{port}/index.html"
    print(f"割草机 3D 查看器已启动： {url}")
    print("关闭此窗口或按 Ctrl+C 停止。")
    if "--no-browser" not in sys.argv:
        threading.Timer(0.6, lambda: webbrowser.open(url)).start()
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        httpd.server_close()


if __name__ == "__main__":
    main()
