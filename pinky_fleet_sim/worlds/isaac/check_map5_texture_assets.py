#!/usr/bin/env python3
import argparse, json, os, sys
import cv2
import numpy as np

IMAGES = ("map5_arena_texture.png", "map5_ground_truth_ortho.png")
CORNER_PX = 120

def detect(gray):
    aruco = cv2.aruco
    detector = aruco.ArucoDetector(
        aruco.getPredefinedDictionary(aruco.DICT_4X4_50), aruco.DetectorParameters()
    )
    corners, ids, _rejected = detector.detectMarkers(gray)
    found = []
    if ids is not None:
        for quad, mid in zip(corners, ids.flatten()):
            pts = quad.reshape(4, 2)
            cx, cy = pts.mean(axis=0)
            side = float(np.mean([np.linalg.norm(pts[k] - pts[(k + 1) % 4]) for k in range(4)]))
            found.append({"id": int(mid), "cx": round(float(cx), 1), "cy": round(float(cy), 1), "side_px": round(side, 1)})
    return sorted(found, key=lambda d: (d["id"], d["cx"], d["cy"]))

def corner_black(gray):
    h, w = gray.shape
    boxes = {
        "TL": gray[0:CORNER_PX, 0:CORNER_PX],
        "TR": gray[0:CORNER_PX, w - CORNER_PX:w],
        "BL": gray[h - CORNER_PX:h, 0:CORNER_PX],
        "BR": gray[h - CORNER_PX:h, w - CORNER_PX:w],
    }
    return {k: round(float((v < 8).mean()), 3) for k, v in boxes.items()}

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("folder")
    ap.add_argument("--tol-px", type=float, default=3.0)
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args()
    manifest_path = os.path.join(args.folder, "map5_texture_manifest.json")
    if not os.path.isfile(manifest_path):
        print("MISSING map5_texture_manifest.json", file=sys.stderr); return 2
    with open(manifest_path, encoding="utf-8") as fh:
        manifest = json.load(fh)
    want_size = tuple(manifest["texture"]["resolution_px"])
    refs = {int(k): v["pixel"] for k, v in manifest["reference_markers_m"].items()}
    report = {"opencv": cv2.__version__, "dictionary": "DICT_4X4_50", "tol_px": args.tol_px, "images": {}, "assertions": []}
    failures = 0
    def assert_(name, ok, detail):
        nonlocal failures
        report["assertions"].append({"name": name, "ok": bool(ok), "detail": detail})
        if not ok: failures += 1
    for name in IMAGES:
        path = os.path.join(args.folder, name)
        if not os.path.isfile(path):
            print("MISSING " + name, file=sys.stderr); return 2
        bgr = cv2.imread(path)
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape
        found = detect(gray)
        report["images"][name] = {
            "size_wh": [w, h],
            "pure_black_fraction": round(float((gray == 0).mean()), 4),
            "corner_black_fraction": corner_black(gray),
            "detections": found,
        }
        assert_(name + ":size", (w, h) == want_size, "got %dx%d want %dx%d" % (w, h, want_size[0], want_size[1]))
        for mid, (px, py) in sorted(refs.items()):
            hits = [d for d in found if d["id"] == mid]
            near = [d for d in hits if abs(d["cx"] - px) <= args.tol_px and abs(d["cy"] - py) <= args.tol_px]
            assert_("%s:id%d detected once at manifest pixel" % (name, mid), len(near) == 1, "near=%d total_with_id=%d" % (len(near), len(hits)))
            extra = [d for d in hits if d not in near]
            assert_("%s:id%d no duplicate elsewhere" % (name, mid), len(extra) == 0, "extra=%s" % json.dumps(extra))
        stray = [d for d in found if d["id"] not in refs]
        report["images"][name]["stray_ids"] = stray
    report["failures"] = failures
    report["assertion_count"] = len(report["assertions"])
    if args.json:
        print(json.dumps(report, ensure_ascii=False, indent=1))
    else:
        print("opencv %s  dict DICT_4X4_50  tol %.1f px" % (report["opencv"], args.tol_px))
        for name, info in report["images"].items():
            print("%s  size=%s  black=%.4f  corners=%s" % (name, info["size_wh"], info["pure_black_fraction"], info["corner_black_fraction"]))
            for d in info["detections"]:
                print("   id=%-3d center=(%.1f,%.1f) side=%.1f px" % (d["id"], d["cx"], d["cy"], d["side_px"]))
        for a in report["assertions"]:
            print(("PASS  " if a["ok"] else "FAIL  ") + a["name"] + "  (" + a["detail"] + ")")
        print("%d/%d assertions failed" % (failures, report["assertion_count"]))
    return 0 if failures == 0 else 1

if __name__ == "__main__":
    sys.exit(main())
