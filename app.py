import argparse
import socket
import threading
import time

import config
import dashboard_base


def _lan_ip() -> str:
    """Best-effort guess at this machine's LAN IP, so we can print a URL
    the watch simulator (running on a second laptop on the same WiFi) can
    actually reach. Doesn't send any traffic — just asks the OS which
    local interface it would use to reach the internet."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        s.close()


def main():
    parser = argparse.ArgumentParser(description="Mine Safety Control Room")
    parser.add_argument(
        "--no-simulate",
        action="store_true",
        help="Skip the built-in worker simulator (use when real telemetry hardware is POSTing to /api/telemetry)",
    )
    parser.add_argument(
        "--no-browser",
        action="store_true",
        help="Don't open the pywebview desktop window — just run the Flask server",
    )
    args = parser.parse_args()

    if config.AUTO_SIMULATE and not args.no_simulate:
        sim_thread = threading.Thread(target=dashboard_base.simulation_loop, daemon=True)
        sim_thread.start()

    flask_thread = threading.Thread(
        target=lambda: dashboard_base.app.run(
            host="0.0.0.0", port=config.PORT, threaded=True, use_reloader=False
        ),
        daemon=True,
    )
    flask_thread.start()

    time.sleep(1.0)

    lan_url = f"http://{_lan_ip()}:{config.PORT}"
    print(f"[Network] Dashboard is on the WiFi at {lan_url}")
    print(f"[Network] On the watch laptop, point the watch simulator's server field at: {lan_url}")

    url = f"http://127.0.0.1:{config.PORT}"

    if args.no_browser:
        print(f"Server running at {url} (no browser window — pass no args to open one)")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass
        return

    import webview

    webview.create_window(
        config.WINDOW_TITLE,
        url,
        width=config.WINDOW_WIDTH,
        height=config.WINDOW_HEIGHT,
        min_size=(1024, 700),
    )
    webview.start()


if __name__ == "__main__":
    main()
