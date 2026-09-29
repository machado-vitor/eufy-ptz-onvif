#!/usr/bin/env python3
"""Minimal ONVIF PTZ shim for eufy pan/tilt cameras.

Presents the Device, Media and PTZ services that Frigate's ONVIF client needs
in order to light up its PTZ controls, and translates ContinuousMove into
Home Assistant `button.press` calls on the eufy_security PTZ buttons.

Eufy pan/tilt cameras (e.g. the T8410 family) have no ONVIF support at all,
and their P2P PTZ command is a fixed step per call (roughly a 90 degree pan),
so: one ContinuousMove = one button press, Stop is a no-op, presets are empty
and no relative/absolute move is advertised (no native autotracking).

Config via environment:
  HASS_URL, HASS_TOKEN   Home Assistant REST API
  CAMERAS                comma separated list of <name>:<port>:<button_prefix>
                         e.g. front:8999:button.front_ptz
  RTSP_URL_<NAME>        optional RTSP URL advertised in the media profile
  TRACKING_SWITCH_<NAME> HA switch of the camera's on-board motion tracking;
                         turned off before a move, restored after TRACKING_RESUME_S
  DEBOUNCE_S             collapse repeated moves inside this window (default 3)
  AUTOTRACK_PAN_THRESHOLD / AUTOTRACK_TILT_THRESHOLD
                         RelativeMove magnitude (FOV units) that triggers one step
  STEP_TIME_S            how long GetStatus reports MOVING after a step (default 5)
  AUTOTRACK_MIN_GAP_S    minimum seconds between autotrack steps (default 8)
"""

import json
import logging
import os

import sys
import threading
import time
import urllib.request
import xml.etree.ElementTree as ET
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

log = logging.getLogger("onvif-eufy")
logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")

NS = {
    "s": "http://www.w3.org/2003/05/soap-envelope",
    "tds": "http://www.onvif.org/ver10/device/wsdl",
    "trt": "http://www.onvif.org/ver10/media/wsdl",
    "tptz": "http://www.onvif.org/ver20/ptz/wsdl",
    "tt": "http://www.onvif.org/ver10/schema",
}
NS_DECL = " ".join(f'xmlns:{k}="{v}"' for k, v in NS.items())


def get_hass_config():
    """Read HASS_URL/HASS_TOKEN lazily so --help works without them set."""
    try:
        return os.environ["HASS_URL"].rstrip("/"), os.environ["HASS_TOKEN"]
    except KeyError as e:
        sys.exit(f"missing required environment variable: {e}")


def envelope(body: str) -> bytes:
    return (
        f'<?xml version="1.0" encoding="UTF-8"?><s:Envelope {NS_DECL}>'
        f"<s:Body>{body}</s:Body></s:Envelope>"
    ).encode()


def fault(msg: str) -> bytes:
    return envelope(
        "<s:Fault><s:Code><s:Value>s:Receiver</s:Value></s:Code>"
        f"<s:Reason><s:Text xml:lang=\"en\">{msg}</s:Text></s:Reason></s:Fault>"
    )


def ha_press(entity_id: str, hass_url: str, hass_token: str) -> None:
    ha_call("button", "press", entity_id, hass_url, hass_token)


def ha_call(domain: str, service: str, entity_id: str, hass_url: str, hass_token: str) -> None:
    req = urllib.request.Request(
        f"{hass_url}/api/services/{domain}/{service}",
        data=json.dumps({"entity_id": entity_id}).encode(),
        headers={"Authorization": f"Bearer {hass_token}", "Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=10) as r:
        log.info("HA %s.%s %s -> %s", domain, service, entity_id, r.status)


def ha_state(entity_id: str, hass_url: str, hass_token: str) -> str:
    req = urllib.request.Request(
        f"{hass_url}/api/states/{entity_id}",
        headers={"Authorization": f"Bearer {hass_token}"},
    )
    with urllib.request.urlopen(req, timeout=10) as r:
        return json.load(r).get("state", "unknown")


class Camera:
    def __init__(self, name: str, port: int, button_prefix: str, hass_url: str, hass_token: str):
        self.name = name
        self.port = port
        self.button_prefix = button_prefix
        self.hass_url = hass_url
        self.hass_token = hass_token
        self.rtsp = os.environ.get(f"RTSP_URL_{name.upper()}", f"rtsp://{name}/")
        self.profile = f"{name}_profile"
        self.lock = threading.Lock()
        # eufy's on-board motion tracking swallows manual PTZ commands. When a
        # move arrives we switch it off, and switch it back on after the camera
        # has been idle for TRACKING_RESUME_S so the user's normal mode returns.
        self.tracking_entity = os.environ.get(f"TRACKING_SWITCH_{name.upper()}")
        self.resume_after = float(os.environ.get("TRACKING_RESUME_S", "60"))
        self.tracking_was_on = False
        self.resume_timer: threading.Timer | None = None
        self.debounce = float(os.environ.get("DEBOUNCE_S", "3"))
        self.settle = float(os.environ.get("TRACKING_SETTLE_S", "2"))
        self.last_move = 0.0
        # --- autotracking emulation ---------------------------------------
        # Frigate's autotracker sends RelativeMove in FOV units (-1..1 = one
        # frame width/height) and polls GetStatus until MoveStatus is IDLE.
        # The eufy has a single fixed step (~90 deg pan) and no position
        # feedback, so: a relative move fires ONE step only when the request
        # exceeds STEP_THRESHOLD (object far off-centre), we report MOVING for
        # STEP_TIME_S afterwards so Frigate waits out the motion, and a
        # software "home" preset undoes the net steps taken.
        self.pan_threshold = float(os.environ.get("AUTOTRACK_PAN_THRESHOLD", "0.4"))
        self.tilt_threshold = float(os.environ.get("AUTOTRACK_TILT_THRESHOLD", "0.6"))
        self.step_time = float(os.environ.get("STEP_TIME_S", "5"))
        self.min_step_gap = float(os.environ.get("AUTOTRACK_MIN_GAP_S", "8"))
        self.moving_until = 0.0
        self.net_pan = 0  # +right / -left steps away from home
        self.net_tilt = 0  # +up / -down

    def suspend_tracking(self) -> None:
        if not self.tracking_entity:
            return
        if self.resume_timer:
            self.resume_timer.cancel()
        if not self.tracking_was_on:
            try:
                if ha_state(self.tracking_entity, self.hass_url, self.hass_token) == "on":
                    ha_call("switch", "turn_off", self.tracking_entity, self.hass_url, self.hass_token)
                    self.tracking_was_on = True
                    # The camera drops a PTZ command that lands while it is
                    # still leaving tracking mode; give it a moment.
                    time.sleep(self.settle)
            except Exception as e:  # noqa: BLE001
                log.error("%s: could not read/disable tracking: %s", self.name, e)
        self.resume_timer = threading.Timer(self.resume_after, self.resume_tracking)
        self.resume_timer.daemon = True
        self.resume_timer.start()

    def resume_tracking(self) -> None:
        if not self.tracking_was_on:
            return
        try:
            ha_call("switch", "turn_on", self.tracking_entity, self.hass_url, self.hass_token)
            self.tracking_was_on = False
            log.info("%s: motion tracking restored after %.0fs idle", self.name, self.resume_after)
        except Exception as e:  # noqa: BLE001
            log.error("%s: could not restore tracking, retrying: %s", self.name, e)
            self.resume_timer = threading.Timer(15, self.resume_tracking)
            self.resume_timer.daemon = True
            self.resume_timer.start()

    # --- responses -------------------------------------------------------

    def capabilities(self, base: str) -> str:
        return (
            "<tds:GetCapabilitiesResponse><tds:Capabilities>"
            f"<tt:Device><tt:XAddr>{base}/onvif/device_service</tt:XAddr></tt:Device>"
            f"<tt:Media><tt:XAddr>{base}/onvif/media_service</tt:XAddr>"
            "<tt:StreamingCapabilities><tt:RTPMulticast>false</tt:RTPMulticast>"
            "<tt:RTP_TCP>true</tt:RTP_TCP><tt:RTP_RTSP_TCP>true</tt:RTP_RTSP_TCP>"
            "</tt:StreamingCapabilities></tt:Media>"
            f"<tt:PTZ><tt:XAddr>{base}/onvif/ptz_service</tt:XAddr></tt:PTZ>"
            "</tds:Capabilities></tds:GetCapabilitiesResponse>"
        )

    def device_info(self) -> str:
        return (
            "<tds:GetDeviceInformationResponse>"
            "<tds:Manufacturer>eufy (onvif-eufy shim)</tds:Manufacturer>"
            f"<tds:Model>T8410</tds:Model><tds:FirmwareVersion>shim</tds:FirmwareVersion>"
            f"<tds:SerialNumber>{self.name}</tds:SerialNumber>"
            f"<tds:HardwareId>{self.name}</tds:HardwareId>"
            "</tds:GetDeviceInformationResponse>"
        )

    def ptz_configuration(self) -> str:
        return (
            f'<tt:PTZConfiguration token="{self.name}_ptzcfg">'
            f"<tt:Name>{self.name}</tt:Name><tt:UseCount>1</tt:UseCount>"
            f"<tt:NodeToken>{self.name}_node</tt:NodeToken>"
            "<tt:DefaultRelativePanTiltTranslationSpace>"
            "http://www.onvif.org/ver10/tptz/PanTiltSpaces/TranslationSpaceFov"
            "</tt:DefaultRelativePanTiltTranslationSpace>"
            "<tt:DefaultContinuousPanTiltVelocitySpace>"
            "http://www.onvif.org/ver10/tptz/PanTiltSpaces/VelocityGenericSpace"
            "</tt:DefaultContinuousPanTiltVelocitySpace>"
            "<tt:DefaultPTZSpeed><tt:PanTilt x=\"0.5\" y=\"0.5\" "
            "space=\"http://www.onvif.org/ver10/tptz/PanTiltSpaces/GenericSpeedSpace\"/>"
            "</tt:DefaultPTZSpeed><tt:DefaultPTZTimeout>PT1S</tt:DefaultPTZTimeout>"
            "</tt:PTZConfiguration>"
        )

    def profiles(self) -> str:
        return (
            "<trt:GetProfilesResponse>"
            f'<trt:Profiles token="{self.profile}" fixed="true">'
            f"<tt:Name>{self.name}</tt:Name>"
            f'<tt:VideoSourceConfiguration token="{self.name}_vsc">'
            f"<tt:Name>{self.name}</tt:Name><tt:UseCount>1</tt:UseCount>"
            f"<tt:SourceToken>{self.name}_vs</tt:SourceToken>"
            '<tt:Bounds x="0" y="0" width="1920" height="1080"/>'
            "</tt:VideoSourceConfiguration>"
            f'<tt:VideoEncoderConfiguration token="{self.name}_vec">'
            f"<tt:Name>{self.name}</tt:Name><tt:UseCount>1</tt:UseCount>"
            "<tt:Encoding>H264</tt:Encoding>"
            "<tt:Resolution><tt:Width>1920</tt:Width><tt:Height>1080</tt:Height></tt:Resolution>"
            "<tt:Quality>5</tt:Quality>"
            "<tt:Multicast><tt:Address><tt:Type>IPv4</tt:Type>"
            "<tt:IPv4Address>0.0.0.0</tt:IPv4Address></tt:Address>"
            "<tt:Port>0</tt:Port><tt:TTL>0</tt:TTL><tt:AutoStart>false</tt:AutoStart>"
            "</tt:Multicast><tt:SessionTimeout>PT60S</tt:SessionTimeout>"
            "</tt:VideoEncoderConfiguration>"
            + self.ptz_configuration()
            + "</trt:Profiles></trt:GetProfilesResponse>"
        )

    def video_sources(self) -> str:
        return (
            "<trt:GetVideoSourcesResponse>"
            f'<trt:VideoSources token="{self.name}_vs"><tt:Framerate>15</tt:Framerate>'
            "<tt:Resolution><tt:Width>1920</tt:Width><tt:Height>1080</tt:Height></tt:Resolution>"
            "</trt:VideoSources></trt:GetVideoSourcesResponse>"
        )

    def stream_uri(self) -> str:
        return (
            "<trt:GetStreamUriResponse><trt:MediaUri>"
            f"<tt:Uri>{self.rtsp}</tt:Uri><tt:InvalidAfterConnect>false</tt:InvalidAfterConnect>"
            "<tt:InvalidAfterReboot>false</tt:InvalidAfterReboot><tt:Timeout>PT60S</tt:Timeout>"
            "</trt:MediaUri></trt:GetStreamUriResponse>"
        )

    def ptz_status(self) -> str:
        # No position feedback on the eufy P2P API. Position is a fake fixed
        # point; MoveStatus is MOVING for STEP_TIME_S after a step so the
        # autotracker waits for the motion to finish before re-evaluating.
        state = "MOVING" if time.monotonic() < self.moving_until else "IDLE"
        return (
            "<tptz:GetStatusResponse><tptz:PTZStatus>"
            "<tt:Position><tt:PanTilt x=\"0\" y=\"0\" "
            "space=\"http://www.onvif.org/ver10/tptz/PanTiltSpaces/PositionGenericSpace\"/>"
            "</tt:Position>"
            f"<tt:MoveStatus><tt:PanTilt>{state}</tt:PanTilt></tt:MoveStatus>"
            "<tt:UtcTime>1970-01-01T00:00:00Z</tt:UtcTime>"
            "</tptz:PTZStatus></tptz:GetStatusResponse>"
        )

    def configuration_options(self) -> str:
        return (
            "<tptz:GetConfigurationOptionsResponse><tptz:PTZConfigurationOptions>"
            "<tt:Spaces>"
            "<tt:RelativePanTiltTranslationSpace>"
            "<tt:URI>http://www.onvif.org/ver10/tptz/PanTiltSpaces/TranslationSpaceFov</tt:URI>"
            "<tt:XRange><tt:Min>-1</tt:Min><tt:Max>1</tt:Max></tt:XRange>"
            "<tt:YRange><tt:Min>-1</tt:Min><tt:Max>1</tt:Max></tt:YRange>"
            "</tt:RelativePanTiltTranslationSpace>"
            "<tt:ContinuousPanTiltVelocitySpace>"
            "<tt:URI>http://www.onvif.org/ver10/tptz/PanTiltSpaces/VelocityGenericSpace</tt:URI>"
            "<tt:XRange><tt:Min>-1</tt:Min><tt:Max>1</tt:Max></tt:XRange>"
            "<tt:YRange><tt:Min>-1</tt:Min><tt:Max>1</tt:Max></tt:YRange>"
            "</tt:ContinuousPanTiltVelocitySpace>"
            "<tt:PanTiltSpeedSpace>"
            "<tt:URI>http://www.onvif.org/ver10/tptz/PanTiltSpaces/GenericSpeedSpace</tt:URI>"
            "<tt:XRange><tt:Min>0</tt:Min><tt:Max>1</tt:Max></tt:XRange>"
            "</tt:PanTiltSpeedSpace>"
            "</tt:Spaces>"
            "<tt:PTZTimeout><tt:Min>PT1S</tt:Min><tt:Max>PT10S</tt:Max></tt:PTZTimeout>"
            "</tptz:PTZConfigurationOptions></tptz:GetConfigurationOptionsResponse>"
        )

    def presets(self) -> str:
        return (
            "<tptz:GetPresetsResponse>"
            '<tptz:Preset token="home"><tt:Name>home</tt:Name></tptz:Preset>'
            "</tptz:GetPresetsResponse>"
        )

    def _step(self, direction: str, origin: str = "auto") -> bool:
        """One physical step. Caller holds self.lock.

        origin: "auto" counts toward the net offset the autotracker undoes;
        "manual" (UI click) redefines where "home" is instead;
        "home" is the undo itself and must not be counted.
        """
        self.suspend_tracking()
        try:
            ha_press(f"{self.button_prefix}_{direction}", self.hass_url, self.hass_token)
        except Exception as e:  # noqa: BLE001
            log.error("%s: HA call failed: %s", self.name, e)
            return False
        self.last_move = time.monotonic()
        self.moving_until = self.last_move + self.step_time
        if origin == "home":
            return True
        if origin == "manual":
            if self.net_pan or self.net_tilt:
                log.info("%s: manual move, new home here (dropping net pan %+d tilt %+d)",
                         self.name, self.net_pan, self.net_tilt)
            self.net_pan = self.net_tilt = 0
            return True
        if direction == "right":
            self.net_pan += 1
        elif direction == "left":
            self.net_pan -= 1
        elif direction == "up":
            self.net_tilt += 1
        elif direction == "down":
            self.net_tilt -= 1
        return True

    def relative_move(self, body: str) -> str:
        x = y = 0.0
        try:
            root = ET.fromstring(body)
            # zeep emits the request in the tptz namespace and the PanTilt child
            # in tt; match by local name so either layout works.
            pt = None
            for el in root.iter():
                if el.tag.endswith("}Translation") or el.tag == "Translation":
                    for ch in el:
                        if ch.tag.endswith("}PanTilt") or ch.tag == "PanTilt":
                            pt = ch
                    break
            if pt is not None:
                x, y = float(pt.get("x", 0)), float(pt.get("y", 0))
        except ET.ParseError:
            pass
        direction = None
        if abs(x) >= self.pan_threshold and abs(x) >= abs(y):
            direction = "right" if x > 0 else "left"
        elif abs(y) >= self.tilt_threshold:
            direction = "up" if y > 0 else "down"
        with self.lock:
            since = time.monotonic() - self.last_move
            if direction is None:
                log.info("%s RelativeMove x=%.2f y=%.2f -> below threshold, hold", self.name, x, y)
            elif since < self.min_step_gap:
                log.info("%s RelativeMove x=%.2f y=%.2f -> %s suppressed (%.1fs since last step)",
                         self.name, x, y, direction, since)
            else:
                if not self._step(direction):
                    return None
                log.info("%s RelativeMove x=%.2f y=%.2f -> step %s (net pan %+d tilt %+d)",
                         self.name, x, y, direction, self.net_pan, self.net_tilt)
        return "<tptz:RelativeMoveResponse/>"

    def goto_home(self) -> str:
        # Undo the net steps. Each press is a fixed step, so the inverse
        # sequence lands back where autotracking started (within eufy slop).
        with self.lock:
            steps = []
            steps += ["left"] * self.net_pan if self.net_pan > 0 else ["right"] * -self.net_pan
            steps += ["down"] * self.net_tilt if self.net_tilt > 0 else ["up"] * -self.net_tilt
            if not steps:
                log.info("%s GotoPreset home: already home", self.name)
                return "<tptz:GotoPresetResponse/>"
            log.info("%s GotoPreset home: %s", self.name, steps)
            for d in steps:
                if not self._step(d, origin="home"):
                    return None
                time.sleep(self.step_time)
            self.net_pan = self.net_tilt = 0
        return "<tptz:GotoPresetResponse/>"

    # --- actions ---------------------------------------------------------

    def continuous_move(self, body: str) -> str:
        x = y = 0.0
        try:
            root = ET.fromstring(body)
            pt = root.find(".//{http://www.onvif.org/ver10/schema}PanTilt")
            if pt is not None:
                x, y = float(pt.get("x", 0)), float(pt.get("y", 0))
        except ET.ParseError:
            pass
        direction = None
        if abs(x) >= abs(y) and x != 0:
            direction = "right" if x > 0 else "left"
        elif y != 0:
            direction = "up" if y > 0 else "down"
        if direction:
            # Serialize: the eufy executes one step per command; overlapping
            # presses from a held button would queue several 90-degree turns.
            # The Frigate UI also re-sends move/stop on every pointer event, so
            # collapse anything inside DEBOUNCE_S into a single step.
            with self.lock:
                now = time.monotonic()
                if now - self.last_move < self.debounce:
                    log.info("%s: debounced %s", self.name, direction)
                    return "<tptz:ContinuousMoveResponse/>"
                if not self._step(direction, origin="manual"):
                    return None
        log.info("%s ContinuousMove x=%s y=%s -> %s", self.name, x, y, direction)
        return "<tptz:ContinuousMoveResponse/>"


class Handler(BaseHTTPRequestHandler):
    camera: Camera = None  # set per server
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt, *args):  # quieter than the default
        log.debug(fmt, *args)

    def do_GET(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/plain")
        self.end_headers()
        self.wfile.write(f"onvif-eufy shim: {self.camera.name}\n".encode())

    def do_POST(self):
        length = int(self.headers.get("Content-Length", 0))
        body = self.rfile.read(length).decode(errors="replace")
        cam = self.camera
        host = self.headers.get("Host", f"127.0.0.1:{cam.port}")
        base = f"http://{host}"
        op = "?"
        try:
            root = ET.fromstring(body)
            b = root.find("{http://www.w3.org/2003/05/soap-envelope}Body")
            if b is None:
                b = root.find("{http://schemas.xmlsoap.org/soap/envelope/}Body")
            if b is not None and len(b):
                op = b[0].tag.split("}")[-1]
        except ET.ParseError:
            pass
        log.debug("%s %s op=%s", cam.name, self.path, op)

        handlers = {
            "GetCapabilities": lambda: cam.capabilities(base),
            "GetServices": lambda: cam.capabilities(base).replace("GetCapabilities", "GetServices"),
            "GetDeviceInformation": cam.device_info,
            "GetSystemDateAndTime": lambda: (
                "<tds:GetSystemDateAndTimeResponse><tds:SystemDateAndTime>"
                "<tt:DateTimeType>NTP</tt:DateTimeType><tt:DaylightSavings>false</tt:DaylightSavings>"
                "</tds:SystemDateAndTime></tds:GetSystemDateAndTimeResponse>"
            ),
            "GetProfiles": cam.profiles,
            "GetProfile": lambda: cam.profiles().replace("GetProfiles", "GetProfile").replace("trt:Profiles", "trt:Profile"),
            "GetVideoSources": cam.video_sources,
            "GetStreamUri": cam.stream_uri,
            "GetConfigurations": lambda: (
                "<tptz:GetConfigurationsResponse>"
                + cam.ptz_configuration().replace("tt:PTZConfiguration", "tptz:PTZConfiguration")
                + "</tptz:GetConfigurationsResponse>"
            ),
            "GetConfiguration": lambda: (
                "<tptz:GetConfigurationResponse>"
                + cam.ptz_configuration().replace("tt:PTZConfiguration", "tptz:PTZConfiguration")
                + "</tptz:GetConfigurationResponse>"
            ),
            "GetNodes": lambda: (
                f'<tptz:GetNodesResponse><tptz:PTZNode token="{cam.name}_node">'
                f"<tt:Name>{cam.name}</tt:Name><tt:SupportedPTZSpaces>"
                "<tt:ContinuousPanTiltVelocitySpace><tt:URI>"
                "http://www.onvif.org/ver10/tptz/PanTiltSpaces/VelocityGenericSpace</tt:URI>"
                "<tt:XRange><tt:Min>-1</tt:Min><tt:Max>1</tt:Max></tt:XRange>"
                "<tt:YRange><tt:Min>-1</tt:Min><tt:Max>1</tt:Max></tt:YRange>"
                "</tt:ContinuousPanTiltVelocitySpace>"
                "<tt:RelativePanTiltTranslationSpace><tt:URI>"
                "http://www.onvif.org/ver10/tptz/PanTiltSpaces/TranslationSpaceFov</tt:URI>"
                "<tt:XRange><tt:Min>-1</tt:Min><tt:Max>1</tt:Max></tt:XRange>"
                "<tt:YRange><tt:Min>-1</tt:Min><tt:Max>1</tt:Max></tt:YRange>"
                "</tt:RelativePanTiltTranslationSpace></tt:SupportedPTZSpaces>"
                "<tt:MaximumNumberOfPresets>1</tt:MaximumNumberOfPresets>"
                "<tt:HomeSupported>false</tt:HomeSupported>"
                "</tptz:PTZNode></tptz:GetNodesResponse>"
            ),
            "GetConfigurationOptions": cam.configuration_options,
            "GetServiceCapabilities": lambda: (
                "<tptz:GetServiceCapabilitiesResponse>"
                '<tptz:Capabilities EFlip="false" Reverse="false" GetCompatibleConfigurations="false" '
                'MoveStatus="true" StatusPosition="false"/>'
                "</tptz:GetServiceCapabilitiesResponse>"
            ),
            "GetPresets": cam.presets,
            "GotoPreset": cam.goto_home,
            "GetStatus": cam.ptz_status,
            "ContinuousMove": lambda: cam.continuous_move(body),
            "RelativeMove": lambda: cam.relative_move(body),
            "Stop": lambda: "<tptz:StopResponse/>",
        }
        fn = handlers.get(op)
        if fn is None:
            log.warning("%s: unsupported op %s", cam.name, op)
            payload, status = fault(f"ActionNotSupported: {op}"), 400
        else:
            out = fn()
            payload, status = (envelope(out), 200) if out else (fault("backend error"), 500)
        self.send_response(status)
        self.send_header("Content-Type", "application/soap+xml; charset=utf-8")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)


def serve(cam: Camera):
    handler = type(f"Handler_{cam.name}", (Handler,), {"camera": cam})
    srv = ThreadingHTTPServer(("0.0.0.0", cam.port), handler)
    log.info("%s: ONVIF shim on :%d -> %s_*", cam.name, cam.port, cam.button_prefix)
    srv.serve_forever()


def main():
    hass_url, hass_token = get_hass_config()
    cams = []
    for spec in os.environ.get("CAMERAS", "").split(","):
        spec = spec.strip()
        if not spec:
            continue
        name, port, prefix = spec.split(":")
        cams.append(Camera(name, int(port), prefix, hass_url, hass_token))
    if not cams:
        sys.exit("CAMERAS is empty")
    threads = [threading.Thread(target=serve, args=(c,), daemon=True) for c in cams]
    for t in threads:
        t.start()
    for t in threads:
        t.join()


if __name__ == "__main__":
    main()
