#!/usr/bin/env python3
"""Configuration for the ADS-B map.

Settings that are not secret live in a private JSON file (adsb_data/config.json, readable only by you). Secrets live in the
macOS Keychain, under the service name "adsb-map". They are handed to the `security` tool on its standard input, so they
never appear in a process list, and they are never written to the config file or returned by the web interface.
"""
import copy
import json
import os
import re
import subprocess
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PATH = os.path.join(HERE, "adsb_data", "config.json")
SERVICE = "adsb-map"
SECRETS = {                       # name -> what it is, shown in the settings dialog
    "swim_user": "FAA SWIM connection user name",
    "swim_password": "FAA SWIM connection password",
    "swim_queue": "FAA SWIM queue name",
    "adsbhub_ckey": "ADSBHub station key (dynamic IP update key)",
}
DEFAULTS = {
    "receiver": {"lat": None, "lon": None},
    "radio": {"lna": 32, "vga": 36, "amp": 1},
    "history": {"keep_days": 30},
    "routes": {"enabled": True},
    "swim": {"url": "", "vpn": ""},
    "sbs": {"port": 30003},
    "share": {"enabled": False, "host": "data.adsbhub.org", "port": 5001},
    "hub": {"enabled": False, "host": "data.adsbhub.org", "port": 5002, "radius_mi": 100},     # receive the shared feed; off until you switch it on
}
LEGACY_FILES = {"swim": os.path.expanduser("~/.swim_credentials"), "adsbhub": os.path.expanduser("~/.adsbhub_credentials")}
RADII = (10, 20, 50, 100, 200, 500, 1000, 2000, 0)      # miles around the receiver for tracking shared aircraft, 0 is the whole world
HOST_RE = re.compile(r"^[A-Za-z0-9]([A-Za-z0-9.-]{0,251}[A-Za-z0-9])?$")


class ConfigError(ValueError):
    pass


def _num(v, lo, hi, kind=float, what="value"):
    try:
        x = kind(v)
    except (TypeError, ValueError):
        raise ConfigError("%s must be a number" % what)
    if not lo <= x <= hi:
        raise ConfigError("%s must be between %s and %s" % (what, lo, hi))
    return x


def _validate(section, key, v):
    """Returns the cleaned value or raises ConfigError. Only known settings are accepted."""
    if section == "receiver" and key in ("lat", "lon"):
        if v in (None, ""):
            return None
        return _num(v, -90 if key == "lat" else -180, 90 if key == "lat" else 180, float, "latitude" if key == "lat" else "longitude")
    if section == "radio":
        if key == "lna":
            x = _num(v, 0, 40, int, "LNA gain")
            if x % 8:
                raise ConfigError("LNA gain goes in steps of 8 dB")
            return x
        if key == "vga":
            x = _num(v, 0, 62, int, "VGA gain")
            if x % 2:
                raise ConfigError("VGA gain goes in steps of 2 dB")
            return x
        if key == "amp":
            return int(_num(v, 0, 1, int, "amplifier"))
    if section == "history" and key == "keep_days":
        return _num(v, 1, 3650, int, "days to keep")
    if section in ("routes", "share", "hub") and key == "enabled":
        if isinstance(v, bool):
            return v
        raise ConfigError("enabled must be true or false")
    if section == "swim" and key in ("url", "vpn"):
        v = str(v).strip()
        if len(v) > 200 or re.search(r"[\s\"'<>]", v):
            raise ConfigError("%s looks invalid" % key)
        if key == "url" and v and not re.match(r"^(tcps?|ssl)://[A-Za-z0-9.-]+(:\d+)?$", v):
            raise ConfigError("the SWIM URL should look like tcps://host:port")
        return v
    if section == "sbs" and key == "port":
        return _num(v, 0, 65535, int, "port")
    if section == "hub" and key == "radius_mi":
        x = _num(v, 0, 100000, int, "radius in miles")
        if x not in RADII:
            raise ConfigError("the tracking radius must be one of: " + ", ".join(str(r) for r in RADII if r) + " miles, or 0 for the whole world")
        return x
    if section in ("share", "hub"):
        if key == "host":
            v = str(v).strip()
            if not HOST_RE.match(v):
                raise ConfigError("host name looks invalid")
            return v
        if key == "port":
            return _num(v, 1, 65535, int, "port")
    raise ConfigError("unknown setting %s.%s" % (section, key))


class Config:
    def __init__(self, path=PATH):
        self.path = path
        self.lock = threading.RLock()
        self.data = copy.deepcopy(DEFAULTS)
        self.existed = os.path.exists(path)
        self._cache = {}                                   # secret name -> (time, value), a short cache to avoid spawning `security`
        if self.existed:
            try:
                with open(path) as f:
                    saved = json.load(f)
                for sec, vals in saved.items():
                    if sec in self.data and isinstance(vals, dict):
                        for k, v in vals.items():
                            if k in self.data[sec]:
                                try:
                                    self.data[sec][k] = _validate(sec, k, v)
                                except ConfigError:
                                    pass
            except (OSError, ValueError):
                pass

    # ---- plain settings
    def get(self, section, key):
        with self.lock:
            return self.data[section][key]

    def update(self, changes, persist=True):
        """changes: {"section": {"key": value}}. All or nothing: nothing is changed if any value is invalid.
        With persist=False the change applies in memory only (used for command line overrides)."""
        clean = {}
        for sec, vals in (changes or {}).items():
            if sec not in DEFAULTS or not isinstance(vals, dict):
                raise ConfigError("unknown section %s" % sec)
            for k, v in vals.items():
                clean.setdefault(sec, {})[k] = _validate(sec, k, v)
        with self.lock:
            for sec, vals in clean.items():
                self.data[sec].update(vals)
            if persist:
                self.save()
        return clean

    def save(self):
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        tmp = self.path + ".tmp"
        fd = os.open(tmp, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)          # private from the first byte
        with os.fdopen(fd, "w") as f:
            json.dump(self.data, f, indent=1)
        os.replace(tmp, self.path)
        os.chmod(self.path, 0o600)
        self.existed = True

    # ---- secrets, in the Keychain
    @staticmethod
    def _security(args, stdin=None):
        try:
            return subprocess.run(["security"] + args, input=stdin, capture_output=True, text=True, timeout=20)
        except (OSError, subprocess.SubprocessError) as e:
            raise ConfigError("the macOS Keychain tool is not available: %s" % e)

    def secret(self, name):
        if name not in SECRETS:
            raise ConfigError("unknown secret")
        hit = self._cache.get(name)
        if hit and time.time() - hit[0] < 60:
            return hit[1]
        r = self._security(["find-generic-password", "-a", name, "-s", SERVICE, "-w"])
        val = r.stdout.rstrip("\n") if r.returncode == 0 else None
        self._cache[name] = (time.time(), val)
        return val

    def has_secret(self, name):
        return bool(self.secret(name))

    def set_secret(self, name, value):
        if name not in SECRETS:
            raise ConfigError("unknown secret")
        value = str(value)
        if not value or "\n" in value or "\r" in value or len(value) > 2000:
            raise ConfigError("that value cannot be stored")
        esc = value.replace("\\", "\\\\").replace('"', '\\"')
        # `security -i` reads its commands from standard input, so the secret is not visible in any process's arguments
        r = self._security(["-i"], stdin='add-generic-password -U -a %s -s %s -w "%s"\n' % (name, SERVICE, esc))
        if r.returncode != 0 or "error" in (r.stderr or "").lower():
            raise ConfigError("the Keychain refused to store the secret")
        self._cache.pop(name, None)

    def clear_secret(self, name):
        if name not in SECRETS:
            raise ConfigError("unknown secret")
        self._security(["delete-generic-password", "-a", name, "-s", SERVICE])
        self._cache.pop(name, None)

    # ---- what the web interface may see: never a secret value
    def public(self):
        with self.lock:
            out = copy.deepcopy(self.data)
        out["secrets"] = {n: {"label": label, "stored": self.has_secret(n)} for n, label in SECRETS.items()}
        out["legacy_files"] = [k for k, p in LEGACY_FILES.items() if os.path.exists(p)]
        return out

    # ---- moving the old plain text credential files into the Keychain
    @staticmethod
    def _read_legacy(path):
        vals = {}
        with open(path, encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith("#") and "=" in line:
                    k, v = line.split("=", 1)
                    vals[k.strip()] = v.strip().strip('"').strip("'")
        return vals

    def import_legacy(self):
        """Copies the values from ~/.swim_credentials and ~/.adsbhub_credentials into the Keychain and the config file.
        The plain text files are left alone. Returns only the names that were imported."""
        done = []
        if os.path.exists(LEGACY_FILES["swim"]):
            v = self._read_legacy(LEGACY_FILES["swim"])
            plain = {}
            if v.get("SWIM_URL"):
                plain["url"] = v["SWIM_URL"]
            if v.get("SWIM_VPN"):
                plain["vpn"] = v["SWIM_VPN"]
            if plain:
                self.update({"swim": plain})
                done += ["swim." + k for k in plain]
            for env, name in (("SWIM_USER", "swim_user"), ("SWIM_PASSWORD", "swim_password"), ("SWIM_QUEUE", "swim_queue")):
                if v.get(env):
                    self.set_secret(name, v[env])
                    done.append(name)
        if os.path.exists(LEGACY_FILES["adsbhub"]):
            v = self._read_legacy(LEGACY_FILES["adsbhub"])
            if v.get("ADSBHUB_CKEY"):
                self.set_secret("adsbhub_ckey", v["ADSBHUB_CKEY"])
                done.append("adsbhub_ckey")
        return done

    def delete_legacy(self):
        """Deletes the plain text credential files, after overwriting them."""
        gone = []
        for k, p in LEGACY_FILES.items():
            if os.path.exists(p):
                size = os.path.getsize(p)
                with open(p, "r+b") as f:
                    f.write(b"\0" * size)
                    f.flush()
                    os.fsync(f.fileno())
                os.remove(p)
                gone.append(k)
        return gone
