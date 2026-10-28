"""Merge DecisionAP's CM service manifests into the bridge PARA runtime."""

import json
import shutil
from pathlib import Path


ROOT = Path("/opt/ap")
COM = ROOT / "etc/com/someip"
PLANNING_COM = Path("/tmp/planning-com")
PLANNING_LIB = Path("/tmp/planning-lib")
BRIDGE_ADDRESS = "192.168.100.92"
PLANNING_ADDRESS = "192.168.100.93"


def read_json(path: Path) -> dict:
    with path.open(encoding="utf-8") as stream:
        return json.load(stream)


def main() -> None:
    cm = read_json(ROOT / "etc/com/CM_CmInstantiation.json")
    if [entry["ip-address"] for entry in cm["endpoints"]] != [BRIDGE_ADDRESS]:
        raise SystemExit("Bridge CM must have exactly one .92 endpoint")

    bridge_manifests = sorted(COM.glob("*.json"))
    planning_manifests = sorted(PLANNING_COM.glob("*.json"))
    if len(bridge_manifests) != 9 or len(planning_manifests) != 9:
        raise SystemExit("Expected 9 bridge and 9 DecisionAP SOME/IP manifests")
    if {path.name for path in bridge_manifests} & {path.name for path in planning_manifests}:
        raise SystemExit("Bridge and DecisionAP CM manifest names overlap")

    for source in planning_manifests:
        manifest = read_json(source)
        if manifest.get("unicast-address") != PLANNING_ADDRESS:
            raise SystemExit(f"Unexpected DecisionAP address in {source.name}")
        manifest["unicast-address"] = BRIDGE_ADDRESS
        destination = COM / source.name
        destination.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")

    ports = {}
    for path in COM.glob("*.json"):
        manifest = read_json(path)
        if manifest.get("unicast-address") != BRIDGE_ADDRESS:
            raise SystemExit(f"Wrong unified CM address in {path.name}")
        for protocol in ("tcp-port", "udp-port"):
            key = (protocol, manifest.get(protocol))
            if key in ports:
                raise SystemExit(f"Port conflict: {path.name} and {ports[key]}")
            ports[key] = path.name

    libraries = sorted(PLANNING_LIB.glob("liblanelet2*"))
    if not libraries:
        raise SystemExit("DecisionAP Lanelet runtime libraries are missing")
    for source in libraries:
        destination = ROOT / "lib" / source.name
        if destination.exists() or destination.is_symlink():
            raise SystemExit(f"Library already exists in bridge runtime: {source.name}")
        if source.is_symlink():
            destination.symlink_to(source.readlink())
        else:
            shutil.copy2(source, destination)

    print(f"Unified CM: {len(bridge_manifests) + len(planning_manifests)} service manifests")


if __name__ == "__main__":
    main()
