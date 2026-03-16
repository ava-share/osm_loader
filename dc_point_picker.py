"""
DC Start/End point picker (local web UI).

What this script does:
- Starts a small local HTTP server and opens a browser with an interactive map.
- Lets you select exactly two points (START and END) using keyboard shortcuts.
- Saves the selection to a JSON file (default: output/selected_points.json).

How to use:
- Run: `python dc_point_picker.py`
- In the opened map:
  - Press S to set START at the mouse cursor
  - Press E to set END at the mouse cursor
  - Press Enter (or click ENTER) to save and exit

Outputs:
- selected_points.json containing START/END and a saved timestamp.

Author: Ahmad Mansour GWU
"""

from __future__ import annotations

import argparse
import hashlib
import json
import mimetypes
import re
import threading
import urllib.request
import webbrowser
from dataclasses import dataclass
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple


LatLon = Tuple[float, float]

DC_CENTER: LatLon = (38.9072, -77.0369)
# Default view: "central DC" (more zoomed-in than the metro-wide bounds).
# If you want a wider view, pass `--zoom 11` (or edit these bounds).
DC_BOUNDS: Tuple[LatLon, LatLon] = ((38.885, -77.085), (38.925, -76.985))

_CDN_ASSET_RE = re.compile(r'(?P<prefix>\b(?:src|href)\s*=\s*")(?P<url>https?://[^"]+)(?P<suffix>")')


def _asset_filename(url: str) -> str:
    name = url.split("?")[0].rstrip("/").split("/")[-1] or "asset"
    h = hashlib.sha1(url.encode("utf-8")).hexdigest()[:10]
    safe = re.sub(r"[^a-zA-Z0-9._-]+", "_", name)
    return f"{h}_{safe}"


def _download_asset(url: str, dest: Path) -> None:
    dest.parent.mkdir(parents=True, exist_ok=True)
    if dest.exists() and dest.stat().st_size > 0:
        return
    req = urllib.request.Request(url, headers={"User-Agent": "dc_point_picker/1.0"})
    with urllib.request.urlopen(req, timeout=30) as resp:  # noqa: S310
        data = resp.read()
    dest.write_bytes(data)


def _rewrite_html_assets_to_local(html: str, static_dir: Path) -> str:
    urls: List[str] = []
    for m in _CDN_ASSET_RE.finditer(html):
        urls.append(m.group("url"))

    replacements: Dict[str, str] = {}
    for url in sorted(set(urls)):
        fname = _asset_filename(url)
        dest = static_dir / fname
        try:
            _download_asset(url, dest)
        except Exception:
            continue
        replacements[url] = f"/static/{fname}"

    if not replacements:
        return html

    def _sub(m: re.Match) -> str:
        url = m.group("url")
        return f'{m.group("prefix")}{replacements.get(url, url)}{m.group("suffix")}'

    return _CDN_ASSET_RE.sub(_sub, html)


@dataclass
class SelectionState:
    points: List[LatLon]

    def __init__(self) -> None:
        self.points = []

    @property
    def start(self) -> Optional[LatLon]:
        return self.points[0] if len(self.points) >= 1 else None

    @property
    def end(self) -> Optional[LatLon]:
        return self.points[1] if len(self.points) >= 2 else None

    def add_point(self, lat: float, lon: float) -> None:
        if len(self.points) >= 2:
            return
        self.points.append((float(lat), float(lon)))

    def reset(self) -> None:
        self.points = []


def _build_picker_html(center: LatLon, zoom: int) -> str:
    import folium

    fmap = folium.Map(
        location=[center[0], center[1]],
        zoom_start=zoom,
        tiles="OpenStreetMap",
        max_bounds=True,
    )
    folium.TileLayer("cartodbpositron", name="CartoDB Positron").add_to(fmap)
    folium.LayerControl(collapsed=True).add_to(fmap)
    folium.FitBounds([[DC_BOUNDS[0][0], DC_BOUNDS[0][1]], [DC_BOUNDS[1][0], DC_BOUNDS[1][1]]]).add_to(fmap)
    map_var = fmap.get_name()

    panel_html = """
    <div id="dcpp_panel" style="
        position: fixed;
        top: 12px;
        right: 12px;
        z-index: 9999;
        background: rgba(255,255,255,0.95);
        border: 1px solid #bbb;
        border-radius: 8px;
        padding: 10px 12px;
        box-shadow: 0 2px 10px rgba(0,0,0,0.15);
        font-family: Arial, sans-serif;
        font-size: 12px;
        min-width: 300px;
    ">
      <div style="font-size: 14px; font-weight: 700; margin-bottom: 6px;">DC Start/End Picker</div>
      <div>Move your mouse on the map, then press:</div>
      <div style="margin-top:6px;"><b>S</b> = set <b>START</b> at cursor</div>
      <div><b>E</b> = set <b>END</b> at cursor</div>
      <div style="margin-top:6px;"><b>Selected</b>: <span id="dcpp_count">0</span>/2</div>
      <div style="margin-top:8px; display:flex; gap:8px;">
        <button id="dcpp_enter" style="padding:6px 10px; font-weight:700;" disabled>ENTER</button>
        <button id="dcpp_reset" style="padding:6px 10px;">Reset</button>
      </div>
      <div id="dcpp_msg" style="margin-top:8px; color:#1b5e20;"></div>
      <div style="margin-top:6px; color:#555;">Then press the on-screen <b>ENTER</b> button (or keyboard Enter) to save.</div>
    </div>
    """
    fmap.get_root().html.add_child(folium.Element(panel_html))

    js = f"""
    (function() {{
      function setMsg(text, ok) {{
        var el = document.getElementById("dcpp_msg");
        if (!el) return;
        el.style.color = ok ? "#1b5e20" : "#b71c1c";
        el.textContent = text || "";
      }}

      if (typeof L === "undefined") {{
        setMsg("Map libraries failed to load (Leaflet missing). This usually means /static assets were blocked. Try refreshing.", false);
        return;
      }}

      window.addEventListener("error", function(ev) {{
        try {{
          var msg = (ev && ev.message) ? ev.message : "Unknown JS error";
          setMsg("Map error: " + msg, false);
        }} catch (e) {{}}
      }});

      function whenMapReady(cb) {{
        var tries = 0;
        function tick() {{
          tries++;
          try {{
            if (typeof {map_var} !== "undefined" && {map_var} && typeof {map_var}.on === "function") {{
              cb({map_var});
              return;
            }}
          }} catch (e) {{}}
          if (tries > 200) {{
            setMsg("Map did not initialize. Try refreshing the page.", false);
            return;
          }}
          setTimeout(tick, 50);
        }}
        tick();
      }}

      var dcpp = {{
        points: [],
        markers: [],
        cursorLatLng: null,
        map: null
      }};

      function updateUI() {{
        var countEl = document.getElementById("dcpp_count");
        if (countEl) countEl.textContent = String(dcpp.points.length);
        var enterBtn = document.getElementById("dcpp_enter");
        if (enterBtn) enterBtn.disabled = !(dcpp.points.length === 2);
      }}

      function postJson(path, payload) {{
        return fetch(path, {{
          method: "POST",
          headers: {{"Content-Type": "application/json"}},
          body: JSON.stringify(payload || {{}})
        }}).then(function(r){{ return r.json(); }});
      }}

      function addMarker(lat, lon, label) {{
        if (!dcpp.map) return;
        // Use CircleMarker so we don't depend on Leaflet's marker PNG assets.
        var style = (label === "START")
          ? {{radius: 8, color: "#1b5e20", fillColor: "#66bb6a", fillOpacity: 0.95, weight: 2}}
          : {{radius: 8, color: "#b71c1c", fillColor: "#ef5350", fillOpacity: 0.95, weight: 2}};
        var m = L.circleMarker([lat, lon], style).addTo(dcpp.map).bindPopup(label);
        dcpp.markers.push(m);
        m.openPopup();
      }}

      function setStartAtCursor() {{
        if (!dcpp.cursorLatLng) {{
          setMsg("Move the mouse over the map first.", false);
          return;
        }}
        if (dcpp.points.length >= 1) {{
          setMsg("START already set. Use Reset to change.", false);
          return;
        }}
        var lat = dcpp.cursorLatLng.lat;
        var lon = dcpp.cursorLatLng.lng;
        dcpp.points.push([lat, lon]);
        addMarker(lat, lon, "START");
        setMsg("", true);
        updateUI();
        postJson("/click", {{lat: lat, lon: lon}}).catch(function(_){{}});
      }}

      function setEndAtCursor() {{
        if (!dcpp.cursorLatLng) {{
          setMsg("Move the mouse over the map first.", false);
          return;
        }}
        if (dcpp.points.length === 0) {{
          setMsg("Set START first (press S).", false);
          return;
        }}
        if (dcpp.points.length >= 2) {{
          setMsg("END already set. Use Reset to change.", false);
          return;
        }}
        var lat = dcpp.cursorLatLng.lat;
        var lon = dcpp.cursorLatLng.lng;
        dcpp.points.push([lat, lon]);
        addMarker(lat, lon, "END");
        setMsg("", true);
        updateUI();
        postJson("/click", {{lat: lat, lon: lon}}).catch(function(_){{}});
      }}

      function resetAll() {{
        dcpp.points = [];
        if (dcpp.map) {{
          dcpp.markers.forEach(function(m){{ try {{ dcpp.map.removeLayer(m); }} catch(e) {{}} }});
        }}
        dcpp.markers = [];
        setMsg("", true);
        updateUI();
        postJson("/reset", {{}}).catch(function(_){{}});
      }}

      function saveIfReady() {{
        if (dcpp.points.length !== 2) {{
          setMsg("Select 2 points first.", false);
          return;
        }}
        postJson("/save", {{start: dcpp.points[0], end: dcpp.points[1]}})
          .then(function(resp) {{
            if (resp && resp.ok) {{
              setMsg("Saved to: " + resp.path, true);
            }} else {{
              setMsg("Save failed.", false);
            }}
          }})
          .catch(function(err) {{
            setMsg("Save failed: " + String(err), false);
          }});
      }}

      whenMapReady(function(map) {{
        dcpp.map = map;
        map.on("mousemove", function(e) {{
          dcpp.cursorLatLng = e.latlng;
        }});
      }});

      var enterBtn = document.getElementById("dcpp_enter");
      if (enterBtn) enterBtn.addEventListener("click", saveIfReady);
      var resetBtn = document.getElementById("dcpp_reset");
      if (resetBtn) resetBtn.addEventListener("click", resetAll);

      document.addEventListener("keydown", function(ev) {{
        if (ev.key === "Enter") {{
          saveIfReady();
          return;
        }}
        if (ev.key === "s" || ev.key === "S") {{
          setStartAtCursor();
          return;
        }}
        if (ev.key === "e" || ev.key === "E") {{
          setEndAtCursor();
          return;
        }}
      }});

      updateUI();
    }})();
    """
    fmap.get_root().script.add_child(folium.Element(js))
    return fmap.get_root().render()


def run_server(
    *,
    output_json: Path,
    host: str = "127.0.0.1",
    port: int = 8765,
    center: LatLon = DC_CENTER,
    zoom: int = 12,
) -> None:
    state = SelectionState()
    lock = threading.Lock()
    done_event = threading.Event()

    output_json.parent.mkdir(parents=True, exist_ok=True)
    static_dir = Path(__file__).with_name("static_dc_picker")

    html = _build_picker_html(center=center, zoom=zoom)
    html = _rewrite_html_assets_to_local(html, static_dir=static_dir)

    class Handler(BaseHTTPRequestHandler):
        def _send(self, status: int, content_type: str, body: bytes) -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _json(self, status: int, payload: Dict[str, Any]) -> None:
            body = json.dumps(payload).encode("utf-8")
            self._send(status, "application/json; charset=utf-8", body)

        def do_GET(self) -> None:  # noqa: N802
            if self.path.startswith("/static/"):
                name = self.path.split("/static/", 1)[1]
                name = name.split("?", 1)[0].split("#", 1)[0]
                if not name or "/" in name or "\\" in name:
                    self._send(400, "text/plain; charset=utf-8", b"Bad Request")
                    return
                fpath = static_dir / name
                if not fpath.exists():
                    self._send(404, "text/plain; charset=utf-8", b"Not Found")
                    return
                ctype, _ = mimetypes.guess_type(str(fpath))
                self._send(200, (ctype or "application/octet-stream"), fpath.read_bytes())
                return

            if self.path not in ("/", "/index.html"):
                self._send(404, "text/plain; charset=utf-8", b"Not Found")
                return
            self._send(200, "text/html; charset=utf-8", html.encode("utf-8"))

        def do_POST(self) -> None:  # noqa: N802
            length = int(self.headers.get("Content-Length", "0") or "0")
            raw = self.rfile.read(length) if length > 0 else b"{}"
            try:
                payload = json.loads(raw.decode("utf-8"))
                if not isinstance(payload, dict):
                    payload = {}
            except Exception:
                payload = {}

            if self.path == "/click":
                try:
                    lat = float(payload["lat"])
                    lon = float(payload["lon"])
                except Exception:
                    self._json(400, {"ok": False})
                    return
                with lock:
                    state.add_point(lat, lon)
                self._json(200, {"ok": True, "count": len(state.points)})
                return

            if self.path == "/reset":
                with lock:
                    state.reset()
                self._json(200, {"ok": True})
                return

            if self.path == "/save":
                try:
                    start = payload["start"]
                    end = payload["end"]
                    lat1, lon1 = float(start[0]), float(start[1])
                    lat2, lon2 = float(end[0]), float(end[1])
                except Exception:
                    self._json(400, {"ok": False, "error": "Invalid start/end"})
                    return

                doc = {
                    "START": [lat1, lon1],
                    "END": [lat2, lon2],
                    "saved_at_utc": datetime.now(timezone.utc).isoformat(),
                }
                output_json.write_text(json.dumps(doc, indent=2), encoding="utf-8")
                done_event.set()
                self._json(200, {"ok": True, "path": str(output_json)})
                return

                # fallthrough

            self._send(404, "text/plain; charset=utf-8", b"Not Found")

        def log_message(self, format: str, *args: Any) -> None:  # noqa: A002
            return

    httpd = ThreadingHTTPServer((host, port), Handler)
    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()

    url = f"http://{host}:{port}/"
    webbrowser.open(url)

    done_event.wait()
    httpd.shutdown()
    httpd.server_close()


def main() -> int:
    parser = argparse.ArgumentParser(description="Pick START/END points on a DC map and save to JSON.")
    parser.add_argument("--host", default="127.0.0.1", help="Local server host.")
    parser.add_argument("--port", type=int, default=8765, help="Local server port.")
    parser.add_argument("--out", default="output/selected_points.json", help="Output JSON path.")
    parser.add_argument("--zoom", type=int, default=12, help="Initial zoom.")
    args = parser.parse_args()

    run_server(
        output_json=Path(args.out),
        host=args.host,
        port=args.port,
        center=DC_CENTER,
        zoom=args.zoom,
    )
    print(f"Wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())