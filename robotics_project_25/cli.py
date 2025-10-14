"""Console script for robotics_project_25."""
from __future__ import annotations

import sys

import memray
from codecarbon import track_emissions
import hydra

from robotics_project_25 import cool_things

@hydra.main(version_base=None, config_path="configs", config_name="base")
@track_emissions(offline=True, country_iso_code="DEU")
def main(cfg):
    """Console script for robotics_project_25."""
    with memray.Tracker("memray.bin"):
        print(f"Hello, I am a test! My ID is {cfg.id}")
        print("\n")
        print("Add arguments to this script like this:")
        print("     'python robotics_project_25/cli.py +hello=world'")
        print("Or use a yaml config file to store your arguments.")
        print("See click documentation at https://hydra.cc/docs/intro/")
        print("\n")
        cool_things(cfg)
        with open("./performance.csv", "w+") as f:
            f.write("epoch,train_loss,train_acc,val_loss,val_acc\n")
            f.write("1,0.1,0.2,0.3,0.4\n")
        with open("./done.txt", "w+") as f:
            f.write("yes")
        return 0


if __name__ == "__main__":
    sys.exit(main())  # pragma: no cover