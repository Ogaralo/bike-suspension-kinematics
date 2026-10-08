"""
Run the kinematic analysis of a rear suspension defined in a JSON file.

Usage:
    python run_analysis.py                                   # default example
    python run_analysis.py geometries/my_bike.json --out results --no-gif
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

from suspension import BikeGeometry, analyse
from suspension.plotting import animate, plot_linkage, plot_summary


def write_csv(res: dict, path: Path) -> None:
    cols = ["wheel_travel", "shock_travel", "leverage_ratio", "anti_squat",
            "axle_x", "axle_y", "chain_growth"]
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["wheel_travel [mm]", "shock_travel [mm]", "leverage_ratio [-]",
                    "anti_squat [%]", "axle_x [mm]", "axle_y [mm]",
                    "chain_growth [mm]", "ic_x [mm]", "ic_y [mm]"])
        for i in range(len(res["wheel_travel"])):
            w.writerow([f"{res[c][i]:.3f}" for c in cols]
                       + [f"{res['instant_center'][i][0]:.1f}",
                          f"{res['instant_center'][i][1]:.1f}"])


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("geometry", nargs="?", default="geometries/trail_29_horst_link.json")
    p.add_argument("--out", default="results")
    p.add_argument("--no-gif", action="store_true", help="skip the animation")
    args = p.parse_args()

    geom = BikeGeometry.from_json(args.geometry)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    res = analyse(geom)
    print(f"\n{geom.name}\n" + "-" * len(geom.name))
    labels = {
        "rear_travel_mm": "Rear wheel travel [mm]",
        "shock_stroke_mm": "Shock stroke [mm]",
        "mean_leverage_ratio": "Mean leverage ratio [-]",
        "leverage_ratio_start": "Leverage ratio at top-out [-]",
        "leverage_ratio_end": "Leverage ratio at bottom-out [-]",
        "progression_pct": "Progression [%]",
        "anti_squat_at_sag_pct": "Anti-squat at 30 % sag [%]",
        "axle_rearward_shift_mm": "Max. rearward axle shift [mm]",
    }
    for k, label in labels.items():
        print(f"{label:<34}{res['summary'][k]:8.2f}")

    write_csv(res, out / "results.csv")
    plot_summary(res, out)
    plot_linkage(res, geom, out)
    if not args.no_gif:
        animate(res, geom, out)
    print(f"\nFigures and data saved to '{out}/'")


if __name__ == "__main__":
    main()
